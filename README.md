# ArduinoSensor

*   This library contains classes to perform encoding of radio signals in a radio 
*   band like 433 Mhz, which are typical for home automation, while the intention was, to use
*   them for various self made Arduino sensors to transmit temperatures, humidity,
*   light, doorswitch info, ...
*   As this is an early state of the developpment, two encoding approaches
*   are realized. One uses manchester encoding and the other uses common pulse pause modulation.
* 
*   The decoding can be done with the RFD-FHEM/SIGNALduino. Modules for useage in FHEM exists also RFD-FHEM/RFFHEM.
*   Please note, that manchester decoding is not fully implemented in fhem
