/*  Demo Sketch for remote sensor lib for an ATTiny85 with 1Mhz
 *  RemoteSensor library v2.5 (201412) for Arduino
*   Library to encode, encrypt and transmits data
*   2014  N.Butzek, S.Butzek
*   This library contains classes to perform encoding of radio signals in the 433MHz
*   band, which are typical for home automation, while the intention was, to use
*   them for various self made Arduino sensors to transmit temperatures, humidity,
*   light, doorswitch info, ...
*   As this is an early state of the developpment, two encoding approaches
*   are realized. One uses manchester encoding and the other uses common pulse pause modulation.
*   Additional information on manchester encoding can be found in
*   https://github.com/mchr3k/arduino-libs-manchester
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


//#define TinyDHTLib // Use Adafruit Tiny DHT Lib
#define PlayDHT11Lib 1// Use Playground DHT11Lib

#define PROGNAME               "Reedsensor"
#define PROGVERS               "0.3"
#define PIN_REED                2   // INT0 / Reed Switch
#define PWRDEV                  0   // Power Supply for DHT11
#define TRANSMITTER             3   // 433Mhz Transmitter DATA PIN
#define VCC 4500.0 //nominal voltage of the batteries, serves for battery check
#define POWERDOWN
//#define TEST
//#define CALVCC // see below

//#define USE_MANCHESTER_CODING 1// Define to use Mancheser encoding, if not defined RTZ Coding will be used
#include <SensorTransmitter.h>
#include <avr/interrupt.h>
#include <EEPROM.h>


#ifdef POWERDOWN
#include <avr/sleep.h>
#include <avr/wdt.h>
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#endif // POWERDOWN
#define DHT11_PIN 4

#ifdef TinyDHTLib
#include "TinyDHT.h"
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHT11_PIN, DHTTYPE);
#elif PlayDHT11Lib
#include <dht11.h>
dht11 dht11;

#else
#define TINUDHT_OK				0
#define TINUDHT_ERROR_CHECKSUM	-1
#define TINUDHT_ERROR_TIMEOUT	-2

typedef struct  {
	uint16_t humidity;
	uint16_t temperature;
} TinuDHT;

uint8_t tinudht_read(TinuDHT *ptinudht, uint8_t dht_pin);
TinuDHT dht11;
#endif // TinyDHTLib

void setup_watchdog(int ii);
void system_sleep();


int epromAddr = 0;
uint8_t deviceID  = 0;


//deviceID=EEPROM.read(eeprom_addr);

 // Initialization of all Transmitters on pin 9, with defines ID
/************************************
 * Device ID
 * 0
 * 1
 * 2
 * 3    light hirange (outdoor)
 * 4	light hires (indoor)
 * 5	water
 * 6	temp
 * 7	reed gas
 * 8    voltage
 * 9    humidity
 * ..16
 ************************************/
//asTransmitter light(4,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)

asTransmitter temp(6,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)
asTransmitter voltage(8,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)
asTransmitter gas(7,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)
asTransmitter hum(9,0,TRANSMITTER);// (DeviceType, DeviceID, OutputPin)


const uint32_t reed_debounceDelay = 150; //minimu duration bewteen two low pulses


 //Thermo

uint16_t vcc = 0; //supply voltage
byte battery = 0;
uint16_t tempVal = 0;
uint16_t humVal = 0;
volatile uint16_t gasVal=0;
uint8_t transmitCnt=0;
//uint8_t battCnt = 0; //counter for measurement of the battery voltage
bool trigger = 1; // manual send (via reset button)
//AS_BH1750 sensor; //Lightsensor
//uint16_t lightVal = 0;
//int PinSensor2 = A2; //Light

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
  //uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  //uint8_t high = ADCH; // unlocks both
  uint8_t oldSREG = SREG;
  uint16_t adc_result = ADC; //In one read?
  SREG = oldSREG;
  //long result = (high<<8) | low;
  long result=adc_result;
  result = scale_constant  / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return (uint16_t)result; // Vcc in millivolts

  //return result; // Vcc in millivolts
}

byte checkBattery(){
  // use global vcc, battery
  vcc = measureVcc();
  if (vcc > 0.95*VCC) battery = 2;
  else if (vcc < 0.85*VCC) battery = 0;
  else battery = 1;
}

// Interruptserviceroutine
ISR (INT0_vect) {
  static uint32_t lastISR=0;
  if((millis() - lastISR) > reed_debounceDelay){
    gasVal++;
  } // else => trash
  lastISR = millis();
}


void aquire(){
  ///////////////////// aquire data  ///////////////////////////////////////////
  // sensors
//  digitalWrite(PWRSENSOR1,1); //power supply for sensor 1 an digital pin
//  digitalWrite(PWRSENSOR2,1); //power supply for sensor 2 an digital pin

  digitalWrite(PWRDEV,HIGH); //power supply for Devices ON


  #ifdef POWERDOWN
  //setup_watchdog(WDTO_4S);               // 4s for Wakeup of DHT11
  system_sleep();
  //delay(2000);
  #else
  delay(4000);
  #endif
  #ifdef TinyDHTLib
    tempVal = dht.readTemperature(0);
    humVal = dht.readHumidity();
  #elif PlayDHT11Lib
    dht11.read(DHT11_PIN);
  #else
    tinudht_read(&dht11,DHT11_PIN);
  #endif // TinyDHTLib

  digitalWrite(DHT11_PIN,LOW); //DHT11 Datapin low
  digitalWrite(PWRDEV,LOW); //power supply for Devices OFF

  if (!(transmitCnt % 5)){ // every 5 passes check the battery
    checkBattery(); // this also updates vcc
  }

  #ifndef TinyDHTLib
  tempVal=dht11.temperature*10;
  humVal=dht11.humidity*10;
  #endif // TinyDHTLib
  tempVal=(uint16_t)(tempVal+0x8000);
  humVal=(uint16_t)(humVal+0x8000);

  // battery

/*
  int aktVal;
  while (FiFo.getNewValue(&aktVal))
  {
      //gasVal+=1;
  }
*/

}
void transmit(){

// Reset Timer and Prescaler to default
/*
TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
TCCR0B = 0<<WGM02 | 1<<CS00;
TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
GTCCR = 1<<PWM1B | 2<<COM1B0;
*/
    static uint16_t old_tempVal;
    static uint16_t old_gasVal;
    static uint16_t old_humVal;
  ////////////////// transmitt data  ///////////////////////////////////////////
//  digitalWrite(PWRTRANSMITTER,1); //power supply for transmitter on digital pin
  //light.send(lightVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
  //LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);

  // Send only if a sensor has updated Information, or every 5. transmission
  if ((old_tempVal != tempVal) || (transmitCnt % 5==0))
    temp.send(tempVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
  old_tempVal = tempVal;

  if ((old_gasVal != gasVal) || (transmitCnt % 5==0))
    gas.send(gasVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
  old_gasVal = gasVal;

  if ((old_humVal != humVal) || (transmitCnt % 5 == 0))
    hum.send(humVal, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])
  old_humVal = humVal;

  // Update Battery every 5. Trnasmission because this will change always we do not check for changed value
  if (transmitCnt % 5==0)
    voltage.send(vcc, battery, trigger);//(value[16bit], battery[2bit], trigger[1bit])

  ++transmitCnt;


//  digitalWrite(PWRTRANSMITTER,0); //power supply for transmitter on digital pin
}

void sleep(){
  ///////////////////////// go sleep  ///////////////////////////////////////////
  #ifdef POWERDOWN
  setup_watchdog(WDTO_8S); // Take longest sleep mode (8 seconds sleep) to reduce wakeups
  #endif
  for (uint8_t i=0; i<1; i++) //=> 40S
  {
    #ifdef POWERDOWN
    system_sleep();
    #else
    delay(random(2000));
    #endif
  }

  // generate some random time to avoid sending at fix intervals
  #ifdef POWERDOWN
  setup_watchdog(random(1,7)); // 30ms to 2 Seconds extra Delay to get some variation between transmissions
  system_sleep();
  setup_watchdog(WDTO_2S); // Set Sleep mode to 2 Seconds
  #endif

}




void setup() {
//  bool lightSensOk = sensor.begin(RESOLUTION_HIGH);
#if defined(TEST) || defined(CALVCC)
#endif
  srand (analogRead(PWRDEV));
  pinMode(0,INPUT); // set all used port to intput to save power
  pinMode(1,INPUT); // set all used port to intput to save power
  pinMode(PWRDEV,OUTPUT);
  digitalWrite(PWRDEV,LOW); //power supply for Devices
  pinMode(TRANSMITTER,OUTPUT);
  digitalWrite(DHT11_PIN,LOW);
  pinMode(DHT11_PIN,INPUT);
  pinMode(PIN_REED,INPUT_PULLUP);
  //digitalWrite(PIN_REED, HIGH);   // Enable pull-up resistor

if(MCUSR & _BV(WDRF)){            // If a reset was caused by the Watchdog Timer...
    MCUSR &= ~_BV(WDRF);                 // Clear the WDT reset flag
    WDTCR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
    WDTCR = 0x00;                      // Disable the WDT
}
//  digitalWrite(PWRTRANSMITTER,1); //power supply for transmitter on digital pin
  checkBattery();// this will also measure and set vcc
  //attachInterrupt(0, ISR_reedswitch, FALLING);
  cli();
  MCUCR |=(0<<ISC01) | (0<<ISC00);             // Interrupt auslösen wenn INT0 = GND
  //MCUCR |=(1<<ISC01) | (0<<ISC00);             // Interrupt auslösen bei fallender Flanke
  GIMSK  |= (1<<INT0);                          //External Interrupt Request 0 Enable
  sei();                                       // Interrupts erlauben

#ifdef POWERDOWN
  setup_watchdog(WDTO_2S);                     // approximately 4 seconds sleep
#endif
#ifdef TinyDHTLib
  dht.begin();
#endif // TinyDHTLib
}

// possible sleep time: 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
void loop() {
  aquire();
  transmit();
  sleep();
  trigger = 0; //after first run, this will remain 0 (automatic) => on reset button, trigger will show manual
  //wdt_reset();
}

/*
  SLEEP FUNCTIONS
*/

#define BODS 7                   //BOD Sleep bit in MCUCR
#define BODSE 2                  //BOD Sleep enable bit in MCUCR

void system_sleep() {
#ifdef POWERDOWN
  pinMode(TRANSMITTER, INPUT);
  pinMode(DHT11_PIN, INPUT);


  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  cbi(ACSR,ACD);                       //disable the analog comparator

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  cli();
  uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;
  sei();                         // ensure interrupts enabled so we can wake up again


  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  sbi(ACSR,ACD);                       // enable the analog comparator
  pinMode(TRANSMITTER, OUTPUT);
#endif
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  cli();
//  wdt_reset();
  byte bb;
 // int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
//  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
  sei();
}

ISR (WDT_vect)
{
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	//wdt_disable();

}


/*
 * Tinu DHT - Tinusaur DHT11 Library
 *
 * @file: tinudht.c
 * @created: 2014-07-08
 * @author: neven
 *
 * Source code available at: https://bitbucket.org/tinusaur/tinudht
 *
 */
#ifdef TinyDHTLib
#elif PlayDHT11Lib
#else

#include <util/delay.h>

#define TINUDHT_RCV_TIMEOUT 255
#define TINUDHT_RCV_DELAY 10
#define TINUDHT_RCV_LENGTH 2

uint8_t tinudht_read(TinuDHT *ptinudht, uint8_t dht_pin) {

	// Buffer to received data
	uint8_t data[5];
	// Empty the buffer
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
//	for (uint8_t i=0; i< 5; i++) data[i] = 0;	// Another way to empty the data buffer.

	// Send request
	DDRB |= (1 << dht_pin);	// Set port as output
	PORTB &= ~(1 << dht_pin);	// Set to 0
	_delay_ms(18);	// Wait 18 ms
	PORTB |= (1 << dht_pin);	// Set to 1
	_delay_us(40);	// Wait 40 us

	// Receive response
	DDRB &= ~(1 << dht_pin);	// Set port as input
//		PORTB |= (1 << dht_pin);	// Set to 1, optional pull-up.

	uint8_t timeout;

	// Acknowledge
	timeout = TINUDHT_RCV_TIMEOUT;
	while(bit_is_clear(PINB, dht_pin))	// Wait for 1
		if (timeout-- == 0)
			return TINUDHT_ERROR_TIMEOUT;

	timeout = TINUDHT_RCV_TIMEOUT;
	while(bit_is_set(PINB, dht_pin))	// Wait for 0
		if (timeout-- == 0)
			return TINUDHT_ERROR_TIMEOUT;

	uint8_t bit_index = 7;
	uint8_t byte_index = 0;
	uint8_t i;
	// READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
	for (i = 0; i < 40; i++)
	{
		// Wait for start
		timeout = TINUDHT_RCV_TIMEOUT;
		while(bit_is_clear(PINB, dht_pin))	// Wait for 1
			if (timeout-- == 0)
				return TINUDHT_ERROR_TIMEOUT;

		// Determine the bit value
		uint8_t len = 0;
		while(bit_is_set(PINB, dht_pin)) {	// Wait for 0
			_delay_us(TINUDHT_RCV_DELAY);
			if (len++ == TINUDHT_RCV_TIMEOUT)
				return TINUDHT_ERROR_TIMEOUT;
			}

		if (len >= TINUDHT_RCV_LENGTH) data[byte_index] |= (1 << bit_index);

		// Experiments:
		// delay =    0,     len = 3..6 - 7:unstable
		// delay =  1us,     len = 3..5
		// delay =  2us,     len = 3..5
		// delay =  5us,     len = 3..5
		// delay = 10us,     len = 2..4
		// delay = 15us,     len = 2..3
		// delay = 20us,     len = 2    - 3:unstable
		// delay =  9..29us, len = 2

		// NOTE: The values of TINUDHT_RCV_DELAY and TINUDHT_RCV_LENGTH
		//       may need to be adjusted if the F_CPU changes.

		if (bit_index == 0)	// next byte?
		{
			bit_index = 7;	// restart at MSB
			byte_index++;	// next byte!
		}
		else bit_index--;
	}

	// DEBUG
	// lcdddd_gotoxy(0, 1);
	// lcdddd_print_string("DH:");
	// lcdddd_print_dec_padded(data[0]);
	// lcdddd_print_dec_padded(data[1]);
	// lcdddd_gotoxy(0, 2);
	// lcdddd_print_string("DT:");
	// lcdddd_print_dec_padded(data[2]);
	// lcdddd_print_dec_padded(data[3]);
	// lcdddd_gotoxy(0, 3);

	uint8_t checksum = data[0] + data[2];	// TODO: Fix this, does not work in border cases.
	if (data[4] != checksum) return TINUDHT_ERROR_CHECKSUM;

	// DEBUG
	// lcdddd_print_string("CS:");
	// lcdddd_print_dec_padded(data[4]);
	// lcdddd_print_dec_padded(checksum);

	// On DHT11 data[1],data[3] are always zero so not used.
	ptinudht->humidity = data[0];
	ptinudht->temperature = data[2];

	return TINUDHT_OK;
}
#endif
