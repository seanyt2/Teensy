/*
 * ============================================================
 *  Teensy 4.1 CAN Bus MITM Gateway  —  Firmware v10
 * ============================================================
 *
 *  v10 = v9 with COMMUNICATION-LAYER fixes only.
 *
 *  THE PROBLEM v10 SOLVES:
 *    With v9, COM1 streaming worked reliably, but COM2 would
 *    stop responding once COM1 was active for a few seconds.
 *    Diagnosis: every CAN frame triggered a separate
 *    Serial.write() of 12-20 bytes, producing ~5000 USB IN
 *    transactions per second on COM1.  The USB CDC scheduler
 *    on Teensy services BOTH endpoints from one shared
 *    transaction queue — so a saturated COM1 endpoint
 *    starves COM2's endpoint scheduling, and host-bound bytes
 *    on SerialUSB1 stop being acknowledged.
 *
 *  THE FIX (THREE LAYERS — A + B + C):
 *
 *    A.  BATCHED GVRET WRITES
 *        gvretSendFrame() now writes into a 256-byte RAM
 *        staging buffer instead of calling Serial.write()
 *        directly.  The buffer is flushed by gvretFlush()
 *        from the bottom of loop().
 *        Result: ~50-100 USB IN transactions/sec instead of
 *        ~5000.  USB scheduler has time for COM2 again.
 *
 *    B.  COM2-PRIORITY GATE
 *        We track lastCom2RxMs.  When COM2 has received bytes
 *        recently (within COM2_PRIORITY_WINDOW_MS), gvretFlush
 *        skips the flush so the USB scheduler can clear the
 *        COM2 endpoint without competition.  The staging
 *        buffer just accumulates a few extra ms of data.
 *        Safety net: if the staging buffer goes >85% full we
 *        flush anyway — never drop COM1 frames just because
 *        COM2 is busy.
 *
 *    C.  BACKPRESSURE CHECK
 *        Before each flush, check Serial.availableForWrite()
 *        against the staging buffer's used bytes.  If the USB
 *        TX buffer is still draining from the previous flush,
 *        skip this flush and try again next iteration.  Never
 *        block waiting for USB buffer room — that's exactly
 *        what was starving COM2 in v9.
 *
 *  PACKET INTERVAL TIMING IS PRESERVED:
 *    Each frame's GVRET timestamp (`micros() - gvretTimeBase`)
 *    is captured INSIDE gvretSendFrame() at the moment the
 *    frame is received from FlexCAN, BEFORE buffering.  The
 *    host's interval calculations are based on these
 *    timestamps, not on USB packet arrival times — so 10 ms
 *    vs 20 ms vs 100 ms intervals remain accurately
 *    measurable in the GUI.  Batching only adds a small
 *    end-to-end latency (a few ms), it doesn't blur intervals.
 *
 *  CROSS-BUS FORWARDING & RULES ARE UNAFFECTED:
 *    canECU.write() / canVehicle.write() and applyRules() run
 *    SYNCHRONOUSLY for every CAN frame, before the staging
 *    buffer is touched.  Only the COM1 USB output is batched.
 *
 *  LED HEARTBEAT NOW UNAMBIGUOUS:
 *    v9 toggled the LED every ~128 loop iterations, which at
 *    full speed looks like a steady glow.  v10 uses a
 *    millis()-based 2 Hz toggle (250 ms half-period).
 *    A working firmware shows a clean, slow blink even when
 *    both COM ports look dead from the host side.
 *
 *  EVERYTHING ELSE FROM v9 IS RETAINED:
 *    - V4 GVRET wire layout
 *    - CAN3 / CAN2 mapping
 *    - RX_SIZE_64 FIFOs
 *    - Non-blocking com2Println / com2Printf with send_now()
 *    - send_now() for GVRET command replies (latency)
 *    - Bounded gvretHandleCommands (max 32 cmds per call)
 *    - ADD/DEL/CLEAR commands set dirty flags only — host
 *      must call SAVE:NOW for persistence
 *    - 32-byte CANRule, 12-byte CRCTableEntry, 20-byte
 *      SendTableEntry on-wire formats unchanged
 *
 *  Boot banner reports "FW_VERSION=v10".
 *
 *  Hardware (Teensy 4.1):
 *    CAN3  Vehicle  TX = Pin 31  RX = Pin 30
 *    CAN2  ECU      TX = Pin  0  RX = Pin  1
 *
 *  Arduino IDE:
 *    Board    : Teensy 4.1
 *    USB Type : Dual Serial      <-- REQUIRED
 *    Speed    : 600 MHz (or higher)
 *
 *  COM Ports:
 *    COM1  Serial      GVRET binary stream  (SavvyCAN compatible)
 *    COM2  SerialUSB1  Rules + CRC + Send config channel
 *
 *  Libraries:
 *    FlexCAN_T4  https://github.com/tonton81/FlexCAN_T4
 *    LittleFS    (bundled with Teensyduino >= 1.56)
 *
 *  COM2 commands: see CRCTBL_SENDTBL_PROTOCOL.md (no protocol
 *  changes from v9 — the batching is invisible to the host).
 * ============================================================
 */

#include <FlexCAN_T4.h>
#include <LittleFS.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

// ============================================================
//  CONFIGURATION
// ============================================================

#define CAN_VEHICLE_BAUD       500000UL
#define CAN_ECU_BAUD           500000UL
#define MAX_RULES              64
#define MAX_CRC_TABLE          32
#define MAX_SEND_ENTRIES       32
#define RULES_FILENAME         "/rules.bin"
#define CRCTABLE_FILENAME      "/crctable.bin"
#define SENDTABLE_FILENAME     "/sendtable.bin"
#define BOOT_TIMEOUT_MS        1000
#define CRC_LEARN_SAMPLES      5
#define LFS_SIZE               (512*1024)
#define CMD_BUFFER_SIZE        320

#define RULES_FILE_VERSION     2
#define CRCTABLE_FILE_VERSION  1
#define SENDTABLE_FILE_VERSION 1

// ── v10 batching parameters ─────────────────────────────────
// GVRET_BUF_SIZE: total staging buffer, big enough to hold
//   ~12 max-size frames (12 + 8 = 20 bytes per frame).
// GVRET_BUF_FORCE_FLUSH: if used bytes climb above this, flush
//   even when COM2 is busy — COM1 must not lose frames.
// GVRET_FLUSH_INTERVAL_MS: max time bytes can sit in the buffer
//   before being flushed (caps end-to-end latency).
// COM2_PRIORITY_WINDOW_MS: how long after the last COM2 byte we
//   politely defer COM1 flushes to give COM2's endpoint room.
#define GVRET_BUF_SIZE             256
#define GVRET_BUF_FORCE_FLUSH      218     // ~85% of 256
#define GVRET_FLUSH_INTERVAL_MS    5
#define COM2_PRIORITY_WINDOW_MS    20

// ============================================================
//  ENUMS
// ============================================================

typedef enum : uint8_t {
    RULE_EXCLUDE_PID   = 0,
    RULE_ZERO_BYTE     = 1,
    RULE_SET_BYTE      = 2,
    RULE_BIT_MULTIPLY  = 4
} RuleType;

typedef enum : uint8_t {
    DIR_VEH_TO_ECU     = 0,
    DIR_ECU_TO_VEH     = 1,
    DIR_BOTH           = 2
} Direction;

typedef enum : uint8_t {
    DUR_PERMANENT      = 0,
    DUR_CYCLES         = 1,
    DUR_SECONDS        = 2
} DurationMode;

typedef enum : uint8_t {
    CRC_XOR_ALL        = 0,
    CRC_SUM_MOD256     = 1,
    CRC_CRC8_0x07      = 2,
    CRC_J1850          = 3
} CRCType;

// ============================================================
//  STRUCTS
// ============================================================

typedef struct __attribute__((packed)) {
    uint8_t  enabled;           // 0
    uint8_t  type;              // 1
    uint8_t  direction;         // 2
    uint8_t  extended_id;       // 3
    uint32_t can_id;            // 4-7
    uint8_t  byte_mask;         // 8
    uint8_t  auto_recalc_crc;   // 9
    uint8_t  byte_values[8];    // 10-17
    uint8_t  start_bit;         // 18
    uint8_t  bit_count;         // 19
    uint16_t multiplier_x100;   // 20-21
    uint8_t  result_byte;       // 22
    uint8_t  result_width;      // 23
    uint8_t  duration_mode;     // 24
    uint32_t duration_value;    // 25-28
    uint8_t  reserved[3];       // 29-31
} CANRule;
static_assert(sizeof(CANRule) == 32, "CANRule must be exactly 32 bytes");

struct __attribute__((packed)) CRCTableEntry {
    uint32_t can_id;
    uint8_t  crc_byte_index;
    uint8_t  algorithm;
    uint8_t  crc_start_byte;
    uint8_t  crc_end_byte;
    uint8_t  init;
    uint8_t  xor_out;
    uint8_t  dlc;
    uint8_t  reserved;
};
static_assert(sizeof(CRCTableEntry) == 12, "CRCTableEntry must be exactly 12 bytes");

struct __attribute__((packed)) SendTableEntry {
    uint32_t can_id;
    uint8_t  flags;
    uint8_t  crc_byte_index;
    uint8_t  counter_nibble_mask;
    uint8_t  bus;
    uint8_t  data[8];
    uint16_t interval_ms;
    uint8_t  counter_start;
    uint8_t  counter_end;
};
static_assert(sizeof(SendTableEntry) == 20, "SendTableEntry must be exactly 20 bytes");

#define SEND_FLAG_CRC     0x01
#define SEND_FLAG_COUNTER 0x02

typedef struct {
    bool     active;
    uint32_t cycles_remaining;
    uint32_t expire_ms;
} RuleRuntime;

typedef struct {
    uint32_t next_fire_ms;
    uint8_t  counter;
} SendRuntime;

typedef struct {
    bool     active;
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  count;
    uint8_t  samples[CRC_LEARN_SAMPLES][8];
} CRCLearner;

// ============================================================
//  GLOBALS
// ============================================================

FlexCAN_T4<CAN3, RX_SIZE_64, TX_SIZE_16> canVehicle;
FlexCAN_T4<CAN2, RX_SIZE_64, TX_SIZE_16> canECU;

LittleFS_Program lfs;
bool             lfsReady = false;

CANRule          rules[MAX_RULES];
RuleRuntime      ruleRT[MAX_RULES];
uint8_t          ruleCount = 0;

CRCTableEntry    crcTable[MAX_CRC_TABLE];
uint8_t          crcTableCount = 0;

SendTableEntry   sendTable[MAX_SEND_ENTRIES];
SendRuntime      sendRT[MAX_SEND_ENTRIES];
uint8_t          sendTableCount = 0;
bool             sendTableRunning = false;

CRCLearner       crcLearner = {false};
uint32_t         gvretTimeBase = 0;

char             cmdBuf[CMD_BUFFER_SIZE];
uint16_t         cmdIdx = 0;

// ── v10: GVRET batching state ──────────────────────────────
// gvretBuf accumulates per-frame GVRET packets; gvretFlush()
// pushes the whole thing to Serial in one write.
static uint8_t   gvretBuf[GVRET_BUF_SIZE];
static uint16_t  gvretBufUsed   = 0;
static uint32_t  gvretLastFlushMs = 0;
static uint32_t  gvretFramesDropped = 0;   // diagnostic counter

// ── v10: COM2 activity timestamp for priority gate ─────────
// Set by handleCOM2() whenever a byte is read from SerialUSB1.
volatile uint32_t lastCom2RxMs = 0;

// ── Deferred-save state (carried over from v9) ─────────────
volatile bool    rulesDirty = false;
volatile bool    crcDirty   = false;
volatile bool    sendDirty  = false;

// Activity timestamps (kept for SAVE:NOW gating / STATUS)
volatile uint32_t lastCanFrameMs = 0;
volatile uint32_t lastCom2CmdMs  = 0;

// Forward declarations
void analyzeCRC();
void initRuleRuntime(uint8_t idx);
void initSendRuntime(uint8_t idx);
bool saveRules();
bool loadRules();
bool saveCrcTable();
bool loadCrcTable();
bool saveSendTable();
bool loadSendTable();
void gvretFlush();

// ============================================================
//  NON-BLOCKING SERIAL HELPERS  +  send_now() FOR LIVENESS
// ============================================================
//
// On Teensy 4.x, USB CDC writes are double-buffered with an
// auto-flush timer of ~5 ms.  Without an explicit send_now()
// the host can wait up to 5 ms after the firmware emits a
// reply before actually receiving it.  Calling send_now()
// after each host-bound write tells the USB stack to schedule
// the IN transaction immediately, so the GUI sees ACKs at
// real-time speed.

// Best-effort short println on COM2.  Drops on overflow.
static bool com2Println(const char *s) {
    int n = (int)strlen(s);
    if (SerialUSB1.availableForWrite() < n + 1) return false;
    SerialUSB1.write((const uint8_t*)s, n);
    SerialUSB1.write('\n');
    SerialUSB1.send_now();
    return true;
}

static bool com2Printf(const char *fmt, ...) {
    char tmp[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n <= 0 || n >= (int)sizeof(tmp)) return false;
    if (SerialUSB1.availableForWrite() < n) return false;
    SerialUSB1.write((const uint8_t*)tmp, n);
    SerialUSB1.send_now();
    return true;
}

// ============================================================
//  CRC COMPUTATION
// ============================================================

static inline uint8_t crc_step_smbus(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? ((uint8_t)(crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

static inline uint8_t crc_step_j1850(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? ((uint8_t)(crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
    return crc;
}

uint8_t computeCRC(const uint8_t *data, uint8_t start, uint8_t end,
                   uint8_t type, uint8_t init, uint8_t xor_out)
{
    if (start > end) return init ^ xor_out;
    uint8_t r = init;
    for (uint8_t i = start; i <= end; i++) {
        switch (type) {
            case CRC_XOR_ALL:    r ^= data[i];                break;
            case CRC_SUM_MOD256: r  = (uint8_t)(r + data[i]); break;
            case CRC_CRC8_0x07:  r  = crc_step_smbus(r, data[i]); break;
            case CRC_J1850:      r  = crc_step_j1850(r, data[i]); break;
            default: break;
        }
    }
    return (uint8_t)(r ^ xor_out);
}

CRCTableEntry* crcTableFind(uint32_t id) {
    for (uint8_t i = 0; i < crcTableCount; i++)
        if (crcTable[i].can_id == id) return &crcTable[i];
    return NULL;
}

int crcTableFindIndex(uint32_t id) {
    for (uint8_t i = 0; i < crcTableCount; i++)
        if (crcTable[i].can_id == id) return (int)i;
    return -1;
}

void crcTableApply(const CRCTableEntry *ct, uint8_t *data, uint8_t len) {
    if (ct->crc_byte_index >= len) return;
    if (ct->crc_start_byte >= len || ct->crc_end_byte >= len) return;
    uint8_t crc = computeCRC(data, ct->crc_start_byte, ct->crc_end_byte,
                             ct->algorithm, ct->init, ct->xor_out);
    data[ct->crc_byte_index] = crc;
}

// ============================================================
//  BIT EXTRACT + HEX PARSE
// ============================================================

uint32_t extractBits(const uint8_t *data, uint8_t startBit, uint8_t bitCount)
{
    uint32_t result = 0;
    for (uint8_t i = 0; i < bitCount; i++) {
        uint8_t pos     = startBit + i;
        uint8_t byteIdx = pos >> 3;
        uint8_t bitIdx  = pos & 0x07;
        if ((data[byteIdx] >> bitIdx) & 0x01)
            result |= (1UL << i);
    }
    return result;
}

static inline int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool parseHexBytes(const char *hex, uint8_t *out, size_t byteCount) {
    for (size_t i = 0; i < byteCount; i++) {
        int hi = hexNibble(hex[i*2]);
        int lo = hexNibble(hex[i*2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

// ============================================================
//  RULE RUNTIME
// ============================================================

void initRuleRuntime(uint8_t idx)
{
    const CANRule &r  = rules[idx];
    RuleRuntime   &rt = ruleRT[idx];
    rt.active           = (r.enabled != 0);
    rt.cycles_remaining = r.duration_value;
    rt.expire_ms        = millis() + r.duration_value;
}

bool isRuleActive(uint8_t idx)
{
    const CANRule     &r  = rules[idx];
    const RuleRuntime &rt = ruleRT[idx];
    if (!r.enabled || !rt.active) return false;
    if (r.type == RULE_EXCLUDE_PID) return true;
    switch (r.duration_mode) {
        case DUR_PERMANENT: return true;
        case DUR_CYCLES:    return rt.cycles_remaining > 0;
        case DUR_SECONDS:   return millis() < rt.expire_ms;
    }
    return false;
}

void tickCycle(uint8_t idx)
{
    if (rules[idx].duration_mode == DUR_CYCLES) {
        if (ruleRT[idx].cycles_remaining > 0) {
            ruleRT[idx].cycles_remaining--;
            if (ruleRT[idx].cycles_remaining == 0)
                ruleRT[idx].active = false;
        }
    }
}

bool applyRules(CAN_message_t &msg, uint8_t dir)
{
    bool needsCrcRecalc = false;
    for (uint8_t i = 0; i < ruleCount; i++) {
        const CANRule &r = rules[i];
        if (!isRuleActive(i)) continue;
        if (r.can_id != msg.id) continue;
        if (r.direction != DIR_BOTH && r.direction != dir) continue;

        switch (r.type) {
            case RULE_EXCLUDE_PID:
                return false;

            case RULE_ZERO_BYTE: {
                bool hit = false;
                for (uint8_t b = 0; b < 8; b++) {
                    if ((r.byte_mask & (1 << b)) && b < msg.len) {
                        msg.buf[b] = 0x00; hit = true;
                    }
                }
                if (hit) {
                    if (r.auto_recalc_crc) needsCrcRecalc = true;
                    tickCycle(i);
                }
                break;
            }

            case RULE_SET_BYTE: {
                bool hit = false;
                for (uint8_t b = 0; b < 8; b++) {
                    if ((r.byte_mask & (1 << b)) && b < msg.len) {
                        msg.buf[b] = r.byte_values[b]; hit = true;
                    }
                }
                if (hit) {
                    if (r.auto_recalc_crc) needsCrcRecalc = true;
                    tickCycle(i);
                }
                break;
            }

            case RULE_BIT_MULTIPLY: {
                uint8_t endBit = (uint8_t)(r.start_bit + r.bit_count - 1);
                if (r.bit_count >= 4 && r.bit_count <= 16 &&
                    endBit < (uint8_t)(msg.len * 8))
                {
                    uint32_t extracted = extractBits(msg.buf, r.start_bit, r.bit_count);
                    uint32_t result    = (extracted * (uint32_t)r.multiplier_x100) / 100UL;
                    bool hit = false;
                    if (r.result_width == 2 && (r.result_byte + 1) < msg.len) {
                        if (result > 65535UL) result = 65535UL;
                        msg.buf[r.result_byte]     = (uint8_t)(result & 0xFF);
                        msg.buf[r.result_byte + 1] = (uint8_t)(result >> 8);
                        hit = true;
                    } else if (r.result_byte < msg.len) {
                        if (result > 255UL) result = 255UL;
                        msg.buf[r.result_byte] = (uint8_t)result;
                        hit = true;
                    }
                    if (hit) {
                        if (r.auto_recalc_crc) needsCrcRecalc = true;
                        tickCycle(i);
                    }
                }
                break;
            }
            default: break;
        }
    }

    if (needsCrcRecalc) {
        CRCTableEntry *ct = crcTableFind(msg.id);
        if (ct != NULL) crcTableApply(ct, msg.buf, msg.len);
    }
    return true;
}

// ============================================================
//  GVRET PROTOCOL  (V4 wire format)  —  v10 BATCHED
// ============================================================
//
// Wire format unchanged from v9.  What changed is HOW the
// bytes get to Serial: gvretSendFrame() stages into gvretBuf,
// gvretFlush() pushes the staged bytes in one Serial.write().

#define GVRET_MAGIC                  0xF1
#define GVRET_CMD_BUILD_CAN_FRAME    0x00
#define GVRET_CMD_TIME_SYNC          0x01
#define GVRET_CMD_GET_CANBUS_PARAMS  0x06
#define GVRET_CMD_GET_DEV_INFO       0x07
#define GVRET_CMD_SET_SW_MODE        0x08
#define GVRET_CMD_KEEPALIVE          0x09
#define GVRET_CMD_SET_SYSTYPE        0x0A
#define GVRET_CMD_GET_NUMBUSES       0x0C

// Stage one GVRET LOG frame into the batch buffer.
// Returns true if the frame was staged, false if the staging
// buffer didn't have room (frame is dropped — host side just
// sees a missed sample, but interval timestamps for following
// frames remain accurate).
//
// CRITICAL: ts is sampled HERE — the moment the frame leaves
// the FlexCAN FIFO — so batching does not blur intervals.
bool gvretSendFrame(const CAN_message_t &msg, uint8_t busNum)
{
    uint8_t dlc    = (msg.len > 8) ? 8 : msg.len;
    uint16_t pktLen = 12 + dlc;

    if (gvretBufUsed + pktLen > GVRET_BUF_SIZE) {
        // Buffer full — try a flush right now to make room.
        // gvretFlush() respects backpressure (won't block).
        gvretFlush();
        if (gvretBufUsed + pktLen > GVRET_BUF_SIZE) {
            gvretFramesDropped++;
            return false;
        }
    }

    uint32_t ts = micros() - gvretTimeBase;
    uint32_t id = msg.id;
    if (msg.flags.extended) id |= 0x80000000UL;

    uint8_t *p = &gvretBuf[gvretBufUsed];
    p[0] = GVRET_MAGIC;
    p[1] = GVRET_CMD_BUILD_CAN_FRAME;
    memcpy(&p[2], &ts, 4);
    memcpy(&p[6], &id, 4);
    p[10] = (uint8_t)(((busNum & 0x0F) << 4) | (dlc & 0x0F));
    memcpy(&p[11], msg.buf, dlc);
    p[11 + dlc] = 0;   // checksum byte (unused, host ignores)

    gvretBufUsed += pktLen;
    return true;
}

// Push staged GVRET bytes to Serial.
// Conditions / guards:
//   * Empty buffer            → nothing to do
//   * COM2 recently active    → defer UNLESS buffer is past
//                               GVRET_BUF_FORCE_FLUSH
//   * USB TX still draining   → defer (availableForWrite check)
//   * Buffer below threshold
//     AND interval not elapsed → defer
//
// The combination of these guards is what gives COM2 room to
// breathe on the shared USB scheduler without ever blocking
// the main loop.
void gvretFlush()
{
    if (gvretBufUsed == 0) {
        gvretLastFlushMs = millis();
        return;
    }

    uint32_t now = millis();
    bool bufNearFull = (gvretBufUsed >= GVRET_BUF_FORCE_FLUSH);
    bool intervalElapsed = ((uint32_t)(now - gvretLastFlushMs) >= GVRET_FLUSH_INTERVAL_MS);

    // Nothing forcing us to flush yet — let it accumulate.
    if (!bufNearFull && !intervalElapsed) return;

    // COM2-priority gate: if the host just sent a byte to
    // COM2, give that endpoint scheduling priority by
    // skipping our flush — UNLESS the buffer is near full,
    // in which case we'd start dropping CAN frames.
    if (!bufNearFull) {
        if ((uint32_t)(now - lastCom2RxMs) < COM2_PRIORITY_WINDOW_MS) return;
    }

    // Backpressure: if Serial's USB TX buffer is still
    // draining from the previous flush, don't try to push
    // more — that's exactly the call that blocks and starves
    // COM2 in v9.  Wait one loop iteration and try again.
    if ((int)Serial.availableForWrite() < (int)gvretBufUsed) return;

    Serial.write(gvretBuf, gvretBufUsed);
    // Note: NO send_now() here.  The whole point of batching
    // is to let the USB auto-flush coalesce transactions.
    // send_now() would defeat batching by forcing one IN per
    // flush regardless of size.  COM2 replies still call
    // send_now() — they need low latency.
    gvretBufUsed = 0;
    gvretLastFlushMs = now;
}

void gvretHandleCommands()
{
    for (uint8_t guard = 0; guard < 32 && Serial.available() >= 2; guard++) {
        if (Serial.peek() != GVRET_MAGIC) { Serial.read(); continue; }
        Serial.read();
        uint8_t cmd = Serial.read();
        if (Serial.availableForWrite() < 32) return;

        switch (cmd) {
            case GVRET_CMD_TIME_SYNC: {
                gvretTimeBase = micros();
                uint8_t r[6] = { GVRET_MAGIC, GVRET_CMD_TIME_SYNC, 0, 0, 0, 0 };
                Serial.write(r, 6); Serial.send_now();
                break;
            }
            case GVRET_CMD_GET_NUMBUSES: {
                uint8_t r[3] = { GVRET_MAGIC, GVRET_CMD_GET_NUMBUSES, 2 };
                Serial.write(r, 3); Serial.send_now();
                break;
            }
            case GVRET_CMD_GET_CANBUS_PARAMS: {
                uint32_t b0 = CAN_VEHICLE_BAUD, b1 = CAN_ECU_BAUD;
                uint32_t f0 = 0x00000001UL,     f1 = 0x00000001UL;
                uint8_t r[18];
                r[0] = GVRET_MAGIC; r[1] = GVRET_CMD_GET_CANBUS_PARAMS;
                memcpy(&r[2],  &b0, 4); memcpy(&r[6],  &f0, 4);
                memcpy(&r[10], &b1, 4); memcpy(&r[14], &f1, 4);
                Serial.write(r, 18); Serial.send_now();
                break;
            }
            case GVRET_CMD_GET_DEV_INFO: {
                uint8_t r[8] = { GVRET_MAGIC, GVRET_CMD_GET_DEV_INFO,
                                 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
                Serial.write(r, 8); Serial.send_now();
                break;
            }
            case GVRET_CMD_KEEPALIVE: {
                uint8_t r[4] = { GVRET_MAGIC, GVRET_CMD_KEEPALIVE, 0xDE, 0xAD };
                Serial.write(r, 4); Serial.send_now();
                break;
            }
            case GVRET_CMD_SET_SYSTYPE:
            case GVRET_CMD_SET_SW_MODE: {
                if (Serial.available()) Serial.read();
                uint8_t r[3] = { GVRET_MAGIC, cmd, 0 };
                Serial.write(r, 3); Serial.send_now();
                break;
            }
            default: break;
        }
    }
}

// ============================================================
//  CRC LEARNING
// ============================================================

void crcLearnProcess(const CAN_message_t &msg)
{
    if (!crcLearner.active) return;
    if (msg.id != crcLearner.can_id) return;
    if (crcLearner.count >= CRC_LEARN_SAMPLES) return;

    memcpy(crcLearner.samples[crcLearner.count], msg.buf, msg.len);
    crcLearner.dlc = msg.len;
    crcLearner.count++;

    if (crcLearner.count >= CRC_LEARN_SAMPLES) {
        crcLearner.active = false;
        analyzeCRC();
    }
}

void analyzeCRC()
{
    uint8_t dlc = crcLearner.dlc;
    bool changes[8] = {false};
    for (uint8_t b = 0; b < dlc; b++)
        for (uint8_t s = 1; s < CRC_LEARN_SAMPLES; s++)
            if (crcLearner.samples[s][b] != crcLearner.samples[0][b]) {
                changes[b] = true; break;
            }

    for (uint8_t crcByte = 0; crcByte < dlc; crcByte++) {
        if (!changes[crcByte]) continue;
        for (uint8_t algo = 0; algo < 3; algo++) {
            const uint8_t *s0 = crcLearner.samples[0];
            uint8_t derivedSeed = 0;

            if (algo == CRC_XOR_ALL) {
                derivedSeed = s0[crcByte];
                for (uint8_t b = 0; b < dlc; b++)
                    if (b != crcByte) derivedSeed ^= s0[b];
            } else if (algo == CRC_SUM_MOD256) {
                uint8_t sum = 0;
                for (uint8_t b = 0; b < dlc; b++)
                    if (b != crcByte) sum = (uint8_t)(sum + s0[b]);
                derivedSeed = (uint8_t)(s0[crcByte] - sum);
            } else {
                derivedSeed = 0x00;
                uint8_t testCRC = derivedSeed;
                for (uint8_t b = 0; b < dlc; b++) {
                    if (b == crcByte) continue;
                    testCRC = crc_step_smbus(testCRC, s0[b]);
                }
                if (testCRC != s0[crcByte]) continue;
            }

            bool match = true;
            for (uint8_t s = 0; s < CRC_LEARN_SAMPLES && match; s++) {
                uint8_t calc = derivedSeed;
                for (uint8_t b = 0; b < dlc; b++) {
                    if (b == crcByte) continue;
                    switch (algo) {
                        case CRC_XOR_ALL:    calc ^= crcLearner.samples[s][b]; break;
                        case CRC_SUM_MOD256: calc  = (uint8_t)(calc + crcLearner.samples[s][b]); break;
                        case CRC_CRC8_0x07:  calc  = crc_step_smbus(calc, crcLearner.samples[s][b]); break;
                    }
                }
                if (calc != crcLearner.samples[s][crcByte]) match = false;
            }

            if (match) {
                com2Printf("CRC:RESULT:0x%08lX:BYTE=%d:ALGO=%d:SEED=0x%02X:CONFIDENCE=HIGH\n",
                           (unsigned long)crcLearner.can_id, crcByte, algo, derivedSeed);
                return;
            }
        }
    }
    com2Printf("CRC:RESULT:0x%08lX:UNKNOWN\n", (unsigned long)crcLearner.can_id);
}

// ============================================================
//  SEND TABLE RUNTIME
// ============================================================

void initSendRuntime(uint8_t idx)
{
    const SendTableEntry &e = sendTable[idx];
    sendRT[idx].next_fire_ms = millis() + e.interval_ms;
    sendRT[idx].counter      = e.counter_start;
}

void sendTableTick()
{
    if (!sendTableRunning || sendTableCount == 0) return;
    uint32_t now = millis();
    for (uint8_t i = 0; i < sendTableCount; i++) {
        SendTableEntry &e  = sendTable[i];
        SendRuntime    &rt = sendRT[i];
        if ((int32_t)(now - rt.next_fire_ms) < 0) continue;

        uint8_t data[8];
        memcpy(data, e.data, 8);
        if (e.flags & SEND_FLAG_COUNTER) {
            data[1] = (uint8_t)((e.counter_nibble_mask & 0xF0) | (rt.counter & 0x0F));
            if (rt.counter >= e.counter_end) rt.counter = e.counter_start;
            else                             rt.counter++;
        }
        if (e.flags & SEND_FLAG_CRC) {
            CRCTableEntry *ct = crcTableFind(e.can_id);
            if (ct != NULL) crcTableApply(ct, data, 8);
        }

        CAN_message_t msg = {};
        msg.id             = e.can_id;
        msg.flags.extended = (e.can_id > 0x7FF) ? 1 : 0;
        msg.len            = 8;
        memcpy(msg.buf, data, 8);

        if (e.bus == 0) canVehicle.write(msg);
        else            canECU.write(msg);
        gvretSendFrame(msg, e.bus);

        rt.next_fire_ms += e.interval_ms;
        if ((int32_t)(now - rt.next_fire_ms) > (int32_t)e.interval_ms)
            rt.next_fire_ms = now + e.interval_ms;
    }
}

// ============================================================
//  LITTLEFS STORAGE  (called only from SAVE:NOW handler)
// ============================================================

bool saveRules() {
    if (!lfsReady) return false;
    File f = lfs.open(RULES_FILENAME, FILE_WRITE);
    if (!f) return false;
    uint8_t ver = RULES_FILE_VERSION;
    f.write(&ver, 1);
    f.write((uint8_t*)&ruleCount, 1);
    if (ruleCount > 0)
        f.write((uint8_t*)rules, sizeof(CANRule) * ruleCount);
    f.close();
    return true;
}

bool loadRules() {
    ruleCount = 0;
    memset(rules, 0, sizeof(rules));
    memset(ruleRT, 0, sizeof(ruleRT));
    if (!lfsReady) return false;
    File f = lfs.open(RULES_FILENAME, FILE_READ);
    if (!f) return false;
    uint8_t ver = 0;
    if (f.read(&ver, 1) != 1 || ver != RULES_FILE_VERSION) {
        f.close(); lfs.remove(RULES_FILENAME);
        com2Println("WARN:RULES_VERSION_MISMATCH:ERASED");
        return false;
    }
    uint8_t count = 0;
    if (f.read(&count, 1) != 1) { f.close(); return false; }
    if (count > MAX_RULES) count = MAX_RULES;
    size_t expected = sizeof(CANRule) * count;
    size_t got = expected ? f.read((uint8_t*)rules, expected) : 0;
    f.close();
    if (got != expected) { ruleCount = 0; memset(rules, 0, sizeof(rules)); return false; }
    ruleCount = count;
    for (uint8_t i = 0; i < ruleCount; i++) initRuleRuntime(i);
    return true;
}

void clearRules() {
    ruleCount = 0;
    memset(rules, 0, sizeof(rules));
    memset(ruleRT, 0, sizeof(ruleRT));
    if (lfsReady) lfs.remove(RULES_FILENAME);
}

bool saveCrcTable() {
    if (!lfsReady) return false;
    File f = lfs.open(CRCTABLE_FILENAME, FILE_WRITE);
    if (!f) return false;
    uint8_t ver = CRCTABLE_FILE_VERSION;
    f.write(&ver, 1);
    f.write(&crcTableCount, 1);
    if (crcTableCount > 0)
        f.write((uint8_t*)crcTable, sizeof(CRCTableEntry) * crcTableCount);
    f.close();
    return true;
}

bool loadCrcTable() {
    crcTableCount = 0;
    memset(crcTable, 0, sizeof(crcTable));
    if (!lfsReady) return false;
    File f = lfs.open(CRCTABLE_FILENAME, FILE_READ);
    if (!f) return false;
    uint8_t ver = 0;
    if (f.read(&ver, 1) != 1 || ver != CRCTABLE_FILE_VERSION) {
        f.close(); lfs.remove(CRCTABLE_FILENAME);
        com2Println("WARN:CRCTABLE_VERSION_MISMATCH:ERASED");
        return false;
    }
    uint8_t count = 0;
    if (f.read(&count, 1) != 1) { f.close(); return false; }
    if (count > MAX_CRC_TABLE) count = MAX_CRC_TABLE;
    size_t expected = sizeof(CRCTableEntry) * count;
    size_t got = expected ? f.read((uint8_t*)crcTable, expected) : 0;
    f.close();
    if (got != expected) { crcTableCount = 0; memset(crcTable, 0, sizeof(crcTable)); return false; }
    crcTableCount = count;
    return true;
}

void clearCrcTable() {
    crcTableCount = 0;
    memset(crcTable, 0, sizeof(crcTable));
    if (lfsReady) lfs.remove(CRCTABLE_FILENAME);
}

bool saveSendTable() {
    if (!lfsReady) return false;
    File f = lfs.open(SENDTABLE_FILENAME, FILE_WRITE);
    if (!f) return false;
    uint8_t ver = SENDTABLE_FILE_VERSION;
    f.write(&ver, 1);
    f.write(&sendTableCount, 1);
    if (sendTableCount > 0)
        f.write((uint8_t*)sendTable, sizeof(SendTableEntry) * sendTableCount);
    f.close();
    return true;
}

bool loadSendTable() {
    sendTableCount = 0;
    memset(sendTable, 0, sizeof(sendTable));
    memset(sendRT, 0, sizeof(sendRT));
    if (!lfsReady) return false;
    File f = lfs.open(SENDTABLE_FILENAME, FILE_READ);
    if (!f) return false;
    uint8_t ver = 0;
    if (f.read(&ver, 1) != 1 || ver != SENDTABLE_FILE_VERSION) {
        f.close(); lfs.remove(SENDTABLE_FILENAME);
        com2Println("WARN:SENDTABLE_VERSION_MISMATCH:ERASED");
        return false;
    }
    uint8_t count = 0;
    if (f.read(&count, 1) != 1) { f.close(); return false; }
    if (count > MAX_SEND_ENTRIES) count = MAX_SEND_ENTRIES;
    size_t expected = sizeof(SendTableEntry) * count;
    size_t got = expected ? f.read((uint8_t*)sendTable, expected) : 0;
    f.close();
    if (got != expected) { sendTableCount = 0; memset(sendTable, 0, sizeof(sendTable)); return false; }
    sendTableCount = count;
    for (uint8_t i = 0; i < sendTableCount; i++) initSendRuntime(i);
    return true;
}

void clearSendTable() {
    sendTableCount   = 0;
    sendTableRunning = false;
    memset(sendTable, 0, sizeof(sendTable));
    memset(sendRT, 0, sizeof(sendRT));
    if (lfsReady) lfs.remove(SENDTABLE_FILENAME);
}

// ============================================================
//  COM2 COMMAND PROCESSOR
// ============================================================

static void sendHexLine(const uint8_t *buf, size_t n, const char *suffix)
{
    char tmp[160];
    int p = 0;
    for (size_t j = 0; j < n; j++) {
        if (p + 2 >= (int)sizeof(tmp)) break;
        static const char hex[] = "0123456789ABCDEF";
        tmp[p++] = hex[(buf[j] >> 4) & 0xF];
        tmp[p++] = hex[buf[j] & 0xF];
    }
    if (suffix) {
        int sl = (int)strlen(suffix);
        if (p + sl < (int)sizeof(tmp)) { memcpy(&tmp[p], suffix, sl); p += sl; }
    }
    if (p < (int)sizeof(tmp)) tmp[p++] = '\n';
    if (SerialUSB1.availableForWrite() >= p) {
        SerialUSB1.write((const uint8_t*)tmp, p);
        SerialUSB1.send_now();
    }
}

void processCommand(char *cmd)
{
    // Boot ─────────────────────────────────────────────────
    if (strncmp(cmd, "BOOT:CLEAR", 10) == 0) {
        clearRules(); clearCrcTable(); clearSendTable();
        com2Println("OK:RULES_CLEARED");

    } else if (strncmp(cmd, "BOOT:LOAD", 9) == 0) {
        loadRules(); loadCrcTable(); loadSendTable();
        com2Printf("OK:RULES_LOADED:%d\n", ruleCount);

    // Rules ────────────────────────────────────────────────
    } else if (strncmp(cmd, "RULE:ADD:", 9) == 0) {
        if (ruleCount >= MAX_RULES) { com2Println("ERR:MAX_RULES_REACHED"); return; }
        const char *hex = cmd + 9;
        size_t hexLen = strlen(hex);
        size_t needed = sizeof(CANRule) * 2;
        if (hexLen < needed) {
            com2Printf("ERR:RULE_DATA_SHORT:GOT=%d:NEED=%d\n", (int)hexLen, (int)needed);
            return;
        }
        uint8_t tmp[sizeof(CANRule)];
        if (!parseHexBytes(hex, tmp, sizeof(CANRule))) {
            com2Println("ERR:RULE_DATA_INVALID_HEX"); return;
        }
        memcpy(&rules[ruleCount], tmp, sizeof(CANRule));
        initRuleRuntime(ruleCount);
        ruleCount++;
        rulesDirty = true;
        com2Printf("OK:RULE_ADDED:IDX=%d\n", ruleCount - 1);

    } else if (strncmp(cmd, "RULE:DEL:", 9) == 0) {
        uint8_t idx = (uint8_t)atoi(cmd + 9);
        if (idx < ruleCount) {
            rules[idx].enabled = 0;
            ruleRT[idx].active = false;
            rulesDirty = true;
            com2Printf("OK:RULE_DISABLED:IDX=%d\n", idx);
        } else com2Println("ERR:INVALID_INDEX");

    } else if (strncmp(cmd, "RULE:CLEAR", 10) == 0) {
        clearRules();
        rulesDirty = false;
        com2Println("OK:RULES_CLEARED");

    } else if (strncmp(cmd, "RULE:LIST", 9) == 0) {
        com2Printf("RULES:COUNT=%d\n", ruleCount);
        for (uint8_t i = 0; i < ruleCount; i++) {
            char suf[64];
            snprintf(suf, sizeof(suf), ":ACT=%d:CYC=%lu:EXP_MS=%lu",
                     ruleRT[i].active ? 1 : 0,
                     (unsigned long)ruleRT[i].cycles_remaining,
                     (unsigned long)ruleRT[i].expire_ms);
            sendHexLine((const uint8_t*)&rules[i], sizeof(CANRule), suf);
            yield();
        }

    // CRC Table ────────────────────────────────────────────
    } else if (strncmp(cmd, "CRCTBL:ADD:", 11) == 0) {
        const char *hex = cmd + 11;
        size_t hexLen = strlen(hex);
        size_t needed = sizeof(CRCTableEntry) * 2;
        if (hexLen < needed) {
            com2Printf("ERR:CRC_DATA_SHORT:GOT=%d:NEED=%d\n", (int)hexLen, (int)needed);
            return;
        }
        CRCTableEntry e;
        if (!parseHexBytes(hex, (uint8_t*)&e, sizeof(CRCTableEntry))) {
            com2Println("ERR:CRC_DATA_INVALID_HEX"); return;
        }
        int existing = crcTableFindIndex(e.can_id);
        int idx;
        if (existing >= 0) { crcTable[existing] = e; idx = existing; }
        else {
            if (crcTableCount >= MAX_CRC_TABLE) { com2Println("ERR:MAX_CRCS_REACHED"); return; }
            idx = crcTableCount;
            crcTable[crcTableCount++] = e;
        }
        crcDirty = true;
        com2Printf("OK:CRC_ADDED:IDX=%d\n", idx);

    } else if (strncmp(cmd, "CRCTBL:DEL:", 11) == 0) {
        int idx = atoi(cmd + 11);
        if (idx < 0 || idx >= (int)crcTableCount) { com2Println("ERR:INVALID_INDEX"); return; }
        for (int i = idx; i < (int)crcTableCount - 1; i++) crcTable[i] = crcTable[i + 1];
        crcTableCount--;
        memset(&crcTable[crcTableCount], 0, sizeof(CRCTableEntry));
        crcDirty = true;
        com2Printf("OK:CRC_DELETED:IDX=%d\n", idx);

    } else if (strncmp(cmd, "CRCTBL:CLEAR", 12) == 0) {
        clearCrcTable();
        crcDirty = false;
        com2Println("OK:CRC_CLEARED");

    } else if (strncmp(cmd, "CRCTBL:LIST", 11) == 0) {
        com2Printf("CRCS:COUNT=%d\n", crcTableCount);
        for (uint8_t i = 0; i < crcTableCount; i++) {
            sendHexLine((const uint8_t*)&crcTable[i], sizeof(CRCTableEntry), NULL);
            yield();
        }

    // Send Table ───────────────────────────────────────────
    } else if (strncmp(cmd, "SENDTBL:ADD:", 12) == 0) {
        const char *hex = cmd + 12;
        size_t hexLen = strlen(hex);
        size_t needed = sizeof(SendTableEntry) * 2;
        if (hexLen < needed) {
            com2Printf("ERR:SEND_DATA_SHORT:GOT=%d:NEED=%d\n", (int)hexLen, (int)needed);
            return;
        }
        SendTableEntry e;
        if (!parseHexBytes(hex, (uint8_t*)&e, sizeof(SendTableEntry))) {
            com2Println("ERR:SEND_DATA_INVALID_HEX"); return;
        }
        if (e.interval_ms == 0)              { com2Println("ERR:SEND_INTERVAL_ZERO"); return; }
        if (e.counter_end < e.counter_start) { com2Println("ERR:SEND_COUNTER_RANGE"); return; }
        if (e.bus > 1)                       { com2Println("ERR:SEND_BUS_INVALID");   return; }

        int existing = -1;
        for (uint8_t i = 0; i < sendTableCount; i++)
            if (sendTable[i].can_id == e.can_id) { existing = (int)i; break; }
        int idx;
        if (existing >= 0) {
            sendTable[existing] = e;
            initSendRuntime((uint8_t)existing);
            idx = existing;
        } else {
            if (sendTableCount >= MAX_SEND_ENTRIES) { com2Println("ERR:MAX_SENDS_REACHED"); return; }
            idx = sendTableCount;
            sendTable[sendTableCount] = e;
            initSendRuntime(sendTableCount);
            sendTableCount++;
        }
        sendDirty = true;
        com2Printf("OK:SEND_ADDED:IDX=%d\n", idx);

    } else if (strncmp(cmd, "SENDTBL:DEL:", 12) == 0) {
        int idx = atoi(cmd + 12);
        if (idx < 0 || idx >= (int)sendTableCount) { com2Println("ERR:INVALID_INDEX"); return; }
        for (int i = idx; i < (int)sendTableCount - 1; i++) {
            sendTable[i] = sendTable[i + 1];
            sendRT[i]    = sendRT[i + 1];
        }
        sendTableCount--;
        memset(&sendTable[sendTableCount], 0, sizeof(SendTableEntry));
        memset(&sendRT[sendTableCount],    0, sizeof(SendRuntime));
        sendDirty = true;
        com2Printf("OK:SEND_DELETED:IDX=%d\n", idx);

    } else if (strncmp(cmd, "SENDTBL:CLEAR", 13) == 0) {
        clearSendTable();
        sendDirty = false;
        com2Println("OK:SEND_CLEARED");

    } else if (strncmp(cmd, "SENDTBL:LIST", 12) == 0) {
        com2Printf("SENDS:COUNT=%d\n", sendTableCount);
        for (uint8_t i = 0; i < sendTableCount; i++) {
            sendHexLine((const uint8_t*)&sendTable[i], sizeof(SendTableEntry), NULL);
            yield();
        }

    } else if (strncmp(cmd, "SENDTBL:START", 13) == 0) {
        uint32_t now = millis();
        for (uint8_t i = 0; i < sendTableCount; i++) {
            sendRT[i].next_fire_ms = now + sendTable[i].interval_ms;
            sendRT[i].counter      = sendTable[i].counter_start;
        }
        sendTableRunning = true;
        com2Println("OK:SEND_STARTED");

    } else if (strncmp(cmd, "SENDTBL:STOP", 12) == 0) {
        sendTableRunning = false;
        com2Println("OK:SEND_STOPPED");

    // CRC Learn ────────────────────────────────────────────
    } else if (strncmp(cmd, "CRC:LEARN:", 10) == 0) {
        uint32_t id = (uint32_t)strtoul(cmd + 10, nullptr, 16);
        crcLearner.can_id = id;
        crcLearner.count  = 0;
        crcLearner.dlc    = 0;
        crcLearner.active = true;
        com2Printf("OK:CRC_LEARNING:0x%08lX\n", (unsigned long)id);

    // STATUS / SAVE ────────────────────────────────────────
    } else if (strncmp(cmd, "STATUS", 6) == 0) {
        com2Printf("STATUS:RULES=%d:CRCS=%d:SENDS=%d:DIRTY=%c%c%c:UPTIME_MS=%lu:LFS=%s:GVDROP=%lu:GVBUF=%u\n",
                   ruleCount, crcTableCount, sendTableCount,
                   rulesDirty ? 'R' : '-',
                   crcDirty   ? 'C' : '-',
                   sendDirty  ? 'S' : '-',
                   (unsigned long)millis(),
                   lfsReady ? "OK" : "FAIL",
                   (unsigned long)gvretFramesDropped,
                   (unsigned)gvretBufUsed);

    } else if (strncmp(cmd, "SAVE:NOW", 8) == 0) {
        // Optional explicit-flush command — bypasses the
        // quiet window.  Only safe when GUI knows traffic is
        // paused.  GUI should call this after a bulk upload.
        if (rulesDirty) { saveRules();     rulesDirty = false; }
        if (crcDirty)   { saveCrcTable();  crcDirty   = false; }
        if (sendDirty)  { saveSendTable(); sendDirty  = false; }
        com2Println("OK:SAVED");

    } else {
        com2Println("ERR:UNKNOWN_CMD");
    }
}

void handleCOM2()
{
    while (SerialUSB1.available()) {
        // v10: stamp the COM2-priority window every byte so
        // the GVRET flusher knows to back off.
        lastCom2RxMs = millis();

        char c = (char)SerialUSB1.read();
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                lastCom2CmdMs = millis();
                processCommand(cmdBuf);
                cmdIdx = 0;
            }
        } else if (cmdIdx < CMD_BUFFER_SIZE - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }
}

// ============================================================
//  BOOT SEQUENCE
// ============================================================

void bootSequence()
{
    com2Println("READY:AWAITING_BOOT_CMD");
    uint32_t t0 = millis();

    while (millis() - t0 < BOOT_TIMEOUT_MS) {
        while (SerialUSB1.available()) {
            char c = (char)SerialUSB1.read();
            if (c == '\n' || c == '\r') {
                if (cmdIdx > 0) {
                    cmdBuf[cmdIdx] = '\0';
                    cmdIdx = 0;
                    if (strncmp(cmdBuf, "BOOT:CLEAR", 10) == 0) {
                        clearRules(); clearCrcTable(); clearSendTable();
                        com2Println("OK:RULES_CLEARED");
                        return;
                    } else if (strncmp(cmdBuf, "BOOT:LOAD", 9) == 0) {
                        loadRules(); loadCrcTable(); loadSendTable();
                        com2Printf("OK:RULES_LOADED:%d\n", ruleCount);
                        return;
                    } else {
                        com2Println("ERR:INVALID_BOOT_CMD");
                    }
                }
            } else if (cmdIdx < CMD_BUFFER_SIZE - 1) {
                cmdBuf[cmdIdx++] = c;
            }
        }
        yield();
    }

    loadRules(); loadCrcTable(); loadSendTable();
    com2Printf("OK:BOOT_TIMEOUT_LOADED:%d\n", ruleCount);
    cmdIdx = 0;
}

// ============================================================
//  SETUP
// ============================================================

static void waitWithYield(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) { yield(); }
}

void setup()
{
    Serial.begin(2000000);
    SerialUSB1.begin(9600);

    // LED heartbeat (millis-based 2 Hz toggle in loop()).
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWriteFast(LED_BUILTIN, HIGH);

    waitWithYield(300);

    {
        uint32_t rsr = SRC_SRSR;
        SRC_SRSR = rsr;
        com2Println("BOOT:FW_VERSION=v10");
        com2Printf("BOOT:RESET_REASON=0x%08lX\n", (unsigned long)rsr);
        if (rsr & (1u << 3))  com2Println("BOOT:RESET_CAUSE=LOCKUP_HARDFAULT");
        if (rsr & (1u << 5))  com2Println("BOOT:RESET_CAUSE=WDOG");
        if (rsr & (1u << 16)) com2Println("BOOT:RESET_CAUSE=WDOG3");
        if (rsr & (1u << 0))  com2Println("BOOT:RESET_CAUSE=POR");
        if (rsr & (1u << 4))  com2Println("BOOT:RESET_CAUSE=BUTTON");
    }

    canVehicle.begin();
    canVehicle.setBaudRate(CAN_VEHICLE_BAUD);
    canVehicle.setMaxMB(16);
    canVehicle.enableFIFO();

    canECU.begin();
    canECU.setBaudRate(CAN_ECU_BAUD);
    canECU.setMaxMB(16);
    canECU.enableFIFO();

    lfsReady = lfs.begin(LFS_SIZE);
    if (!lfsReady) com2Println("WARN:LFS_INIT_FAILED:RULES_VOLATILE");

    gvretTimeBase    = micros();
    lastCanFrameMs   = millis();
    lastCom2CmdMs    = millis();
    lastCom2RxMs     = 0;
    gvretBufUsed     = 0;
    gvretLastFlushMs = millis();
    memset(&crcLearner, 0, sizeof(crcLearner));

    bootSequence();
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop()
{
    CAN_message_t msg;

    // ── CAN Vehicle → ECU ──────────────────────────────────
    // Forwarding is SYNCHRONOUS — happens on the same iteration
    // as receive.  Only the GVRET output (host-bound) is buffered.
    if (canVehicle.read(msg)) {
        lastCanFrameMs = millis();
        crcLearnProcess(msg);
        (void)gvretSendFrame(msg, 0);          // ← stages into batch
        if (applyRules(msg, DIR_VEH_TO_ECU))
            canECU.write(msg);                  // ← immediate forward
    }

    // ── CAN ECU → Vehicle ──────────────────────────────────
    if (canECU.read(msg)) {
        lastCanFrameMs = millis();
        crcLearnProcess(msg);
        (void)gvretSendFrame(msg, 1);
        if (applyRules(msg, DIR_ECU_TO_VEH))
            canVehicle.write(msg);
    }

    sendTableTick();
    gvretHandleCommands();
    handleCOM2();

    // ── v10: GVRET batched flush ───────────────────────────
    // Pushes the staging buffer to USB once per loop iteration
    // when conditions allow (size threshold OR interval, and
    // not blocked by COM2 priority gate or USB backpressure).
    gvretFlush();

    // ── v10: 2 Hz millis-based LED heartbeat ──────────────
    // Clearly visible "loop is alive" signal.  Independent of
    // CAN traffic, USB activity, or anything else.
    static uint32_t lastLedToggleMs = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - lastLedToggleMs) >= 250) {
        digitalToggleFast(LED_BUILTIN);
        lastLedToggleMs = now;
    }

    yield();
}
