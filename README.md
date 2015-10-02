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
*   

This lib is based and ispired on the sensor transmitter lib from Randy Simons:
https://bitbucket.org/fuzzillogic/433mhzforarduino/wiki/Home


Usage hints for your own sensor:

Define some basic things:
```c++
  #define TRANSMITTER             3   // 433Mhz Transmitter DATA PIN
  #include <SensorTransmitter.h>

  asTransmitter voltage(8,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)
  asTransmitter temp(6,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)
  asTransmitter hum(9,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)
```


Measure your temp and humidity value from your sensors, then convert data for temp and humidity:

```c++

  tempVal=(uint16_t)(tempVal+0x8000);
  humVal=(uint16_t)(humVal+0x8000);
```

  
You can also measure vcc, and send it as a sensor value...
Now we are ready to send data:
```c++
temp.send(tempVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
hum.send(humVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
voltage.send(vcc, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
```
