Vibe coded Canbus MITM Application and Teensy Code with CRC Calculating. 
Still a WIP but uploading what is working so far.

Teensy 4.1 to be used with 2x Canbus Transcievers connected to CAN2/3 
This can be altered easily in the code as it uses the Flexcan Library. 

Currently it creates two com ports via the USB connection one streams Canbus data for both networks and meets GVRET standard so also works in Savvycan. 
The second comport is used to makes and recieve commands to alter rules via the GUI. This is set to Baud 9600 to aid with connecting. 

When Uploading via IDE choose Teensy 4.1 and ensure to select USB Type: Dual Serial. 


The GUI is used to show CAN ID's that are found on each network 
The Teensy will forward traffic bidirectionaly already. Rules are then added like blocking a CAN ID, altering Byte values. 

CRC calculation and Counters options for sending are also available. 
When altering Bytes values first the CanBus CRC needs to be known to be recalculated, This is done under the CRC tab. 
Where it collects 15x packets and automatically calculates the XorOut values so it can update the CRC when a rules is generated. 

Ensure to upload the new calculated CRC to the Teensy. 
Sending Can is the final option this also includes a Counter on Byte 1 option and CRC option. Especially usefull for Modern car networks. 


This was made to help with reverseengineering BMW F/G series canbus networks. 

