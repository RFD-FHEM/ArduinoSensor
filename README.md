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

You need to calibrate your ADC with a voltmeter:
```c++
void calibrate(){
  ///////////////////////// calibration  ///////////////////////////////////////////
  static long mean = 0;
  static int cnt = 0; // this will overflow after 4.5 hours, make shure you take your measurement before ;-)
  mean += measureVcc();
  ++cnt;
  Serial.print("Vcc: "); Serial.print((float)mean/cnt); Serial.println("mV");
  delay(500);
}
```
Then you can modify your sketch with the measured data:

```c++
/************************************
 * Calibration of Vcc
 * Calibration of the 1.1V reference requires an external measurement of Vcc with a voltmeter, for example.
 * Follow these steps:
 *  1. Measure Vcc with the voltmeter => Vcc1
 *  2. Define CALVCC to let Arduino measure Vcc internal and print the result to Serial
 *  3. Enter the above measured values in the formula, overwriting the default values
 *     Make shure to use the same units.
 ************************************/
//internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
#ifndef CALVCC
  //const int internalRef = 1.1 * 3160 / 3120; //with compensation
  const float internalRef=0.96173822714;
  const long scale_constant = internalRef * 1023 * 1000;
#else
  const float internalRef = 1.1; // for calibration no compensation is required
#endif

uint16_t measureVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(MUX3) | _BV(MUX2);

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion

  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t oldSREG = SREG;
  uint16_t adc_result = ADC; //In one read?
  SREG = oldSREG;
  long result=adc_result;
  result = scale_constant  / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return (uint16_t)result; // Vcc in millivolts

  //return result; // Vcc in millivolts
}
```

Now we are ready to send data:
```c++
temp.send(tempVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
hum.send(humVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
voltage.send(vcc, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
```
