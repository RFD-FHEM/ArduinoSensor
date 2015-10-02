/*
*   RemoteSensor library v2.5 (201412) for Arduino
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

#include <SensorTransmitter.h>

#include <util/delay.h>





#ifdef USE_MANCHESTER_CODING
ManchesterTransmit::ManchesterTransmit(uint8_t numMsgBytesIN, uint8_t transmitterPinIN, uint8_t clockIN=MAN_2400, uint8_t preambleIN=0xC)
{
    transmitterPin = transmitterPinIN;
	numMsgBytes = numMsgBytesIN;
	data = (byte*) calloc(numMsgBytes, sizeof(byte));
	repeat = 3; //default, set seperately
	pauseFact = 4; //default, set seperately
	preamble = preambleIN; // default
	pinMode(transmitterPin, OUTPUT);
	digitalWrite(transmitterPin, LOW);

  #if F_CPU == 1000000UL
    const uint8_t compensationFactor = 88; //must be divisible by 8 for workaround
  #elif F_CPU == 8000000UL
    const uint8_t compensationFactor = 12;
  #else //16000000Mhz
    const uint8_t compensationFactor = 4;
  #endif
    clock = (3072 >> clockIN) - compensationFactor;
  #ifndef SPARK_CORE
  #if F_CPU == 1000000UL
    //definition of micro delay is broken for 1MHz speed in tiny cores as of now (May 2013)
    //this is a workaround that will allow us to transmit on 1Mhz
    //divide the wait time by 8
    clock >>= 3;
  #endif
  #endif //Spark
  while (clock % 8 > 0)
  {
    clock++;
  }
}


void ManchesterTransmit::sendMessage()
{
	for (uint8_t cnt=0; cnt<repeat; ++cnt)
    {
		// send sync ( b10101010 1010) -- 12 Bit
		uint8_t i;
        for (i=0; i<6;++i)
        {
            sendOne();
            sendZero();
        }

        // Send Preamble ( b1100 ) -- 4 Bit
        uint8_t bit;
        for (bit=0x8; bit>0; bit>>=1)
        {
           if ((preamble & bit) == bit) sendOne();
           else sendZero();
        }
        //sendByte(0xAA); // Sends 8 Bit Sync Signal = 10101010
        //sendByte(0xAC); // Sends 4 Bit Sync + 4 bit Preamble 10101100

		// send data
		uint8_t b;
		for (i=0; i<numMsgBytes; ++i)
        {
            b=*(data+i);
            for (bit=0x80; bit>0; bit>>=1)
            {
                if ((b & bit) == bit) sendOne();
                else sendZero();
            }
		}
        // Send 1 terminatings 0's to correctly terminate the previous bit and to turn the transmitter off
        sendZero();
        //sendZero();

		delayMicroseconds(clock*pauseFact);
 	    // pause at the end
	}
}

void ManchesterTransmit::setRepeat(uint8_t repeatIN) {
	repeat = repeatIN;
}

void ManchesterTransmit::setPauseFact(uint8_t pauseFactIN) {
	pauseFact = pauseFactIN;
}

void ManchesterTransmit::sendZero(void)
{
  delayMicroseconds(clock);
  digitalWrite(transmitterPin, HIGH);
  delayMicroseconds(clock);
  digitalWrite(transmitterPin, LOW);
}//end of send a zero

void ManchesterTransmit::sendOne(void)
{
  delayMicroseconds(clock);
  digitalWrite(transmitterPin, LOW);
  delayMicroseconds(clock);
  digitalWrite(transmitterPin, HIGH);
}//end of send one
#else // USE_MANCHESTER_CODING
//Sensor base class
SensorTransmitter::SensorTransmitter(byte numMsgBytesIN, byte transmitterPinIN, int clockIN, byte syncFactIN, byte lowFactIN, byte highFactIN) {
	transmitterPin = transmitterPinIN;
	numMsgBytes = numMsgBytesIN;
	data = (byte*) calloc(numMsgBytes, sizeof(byte));
	clock = clockIN;
	syncFact = syncFactIN;
	lowFact = lowFactIN;
	highFact = highFactIN;
	repeat = 3; //default, set seperately
	pauseTime = 20; //default, set seperately
	pinMode(transmitterPin, OUTPUT);
	digitalWrite(transmitterPin, LOW);

/*
  #if F_CPU == 1000000UL
    const uint8_t compensationFactor = 88; //must be divisible by 8 for workaround
  #elif F_CPU == 8000000UL
    const uint8_t compensationFactor = 12;
  #else //16000000Mhz
    const uint8_t compensationFactor = 4;
  #endif
    clock = (3072 >> clockIN) - compensationFactor;
  #ifndef SPARK_CORE
  #if F_CPU == 1000000UL
    //clock -= 44; //22+2 = 24 is divisible by 8  while (clock % 8 > 0)
  {
    clock++;
  }
    //definition of micro delay is broken for 1MHz speed in tiny cores as of now (May 2013)
    //this is a workaround that will allow us to transmit on 1Mhz
    //divide the wait time by 8
    //clock >>= 3;
  #endif
  #endif //Spark
*/



}

void SensorTransmitter::sendBit(byte fact){
	digitalWrite(transmitterPin, HIGH);
	delayMicroseconds(clock-100);
	digitalWrite(transmitterPin, LOW);
    delayMicroseconds(fact*clock);
}

void SensorTransmitter::sendByte(byte b) {
	for (byte bit=0x80; bit>0; bit>>=1){
		sendBit(((b & bit)==bit) ? highFact : lowFact);
	}

}

void SensorTransmitter::sendMessage() {
	for (byte cnt=0; cnt<repeat; ++cnt){
		// send sync
		sendBit(syncFact);

		// send data
		for (int i=0; i<numMsgBytes; ++i){
			sendByte(*(data+i));
		}
		sendBit(pauseTime);
		// pause at the end

	}
}

void SensorTransmitter::setRepeat(byte repeatIN){
	repeat = repeatIN;
}

void SensorTransmitter::setPauseTime(byte pauseTimeIN){
	pauseTime = pauseTimeIN;
}

void SensorTransmitter::printMessageHexStr(){
	char hexStr[] ="00";
	for (byte cnt=0; cnt<numMsgBytes; ++cnt){
		sprintf(hexStr, "%02x",*(data+cnt));
		Serial.print(hexStr);
	}
	Serial.println();
}

#endif // USE_MANCHESTER_CODING

/************************************
 * ArduinoSensor transmitter
 *
 * Device ID (Byte 3) bit 5...0
 * 0
 * 1
 * 2
 * 3    light hirange (outdoor)
 * 4	light hires (indoor)
 * 5	water
 * 6	temp
 * 7	reed gas
 * 8    voltage
 * ..31
 ************************************/
/*
identifikatorAS [4Bit-8bit] 	[4bit]	[0bit]	[0bit]
trigger 			[1bit]	[1bit]	[1bit]
type [32 - 5Bit-8bit] 		[8bit]	[7bit]	[7bit]
device ID [5bit - 8bit] 	[8bit]	[6bit]	[6bit]
battery 			[3bit]	[2bit]	[2bit]
data low [8bit]			[8bit]	[8bit]	[8bit]
data high [8bit]		[8bit]	[8bit]	[8bit]
checksum [8bit]			[8bit]	[8bit]	[0bit]
---------------------------------------------------------
				48bit	40bit	32bit

Protocoll AS
-------------
byte 0 		- Bit 0..6	sensor type
			- Bit 7		trigger (0 auto, 1 manual)
byte 1		- Bit 0..5	device ID
			- Bit 6,7 battery status (00 - bad, 01 - change, 10 - ok, 11 - optimal)
byte 2		- data lowbyte
byte 3		- data high byte
byte 4		- crc8 Bit 0..7
*/


#ifdef USE_MANCHESTER_CODING
asTransmitter::asTransmitter(byte typeIN, byte idIN, byte transmitterPinIN):ManchesterTransmit(5,transmitterPinIN,MAN_2400,0xC) {
	type = typeIN &0x7f; //only bit 0..6
	id = idIN &0x3f; //only bit 0..5
}
#else // USE_MANCHESTER_CODING
asTransmitter::asTransmitter(byte typeIN, byte idIN, byte transmitterPinIN):SensorTransmitter(4,transmitterPinIN,500, 18, 1, 2) {
	type = typeIN &0x7f; //only bit 0..6
	id = idIN &0x3f; //only bit 0..5
	setPauseTime(40);
}
#endif //USE_MANCHESTER_CODING

void asTransmitter::send(uint16_t value, byte battery, bool trigger) {
	*(data+0) = type | ((trigger ? 1:0) <<7);
	*(data+1) = id | ((battery & 0x3) <<6);
	*(data+2) = lowByte(value);
	*(data+3) = highByte(value);
	//*(data+4) = crc(data, 4);//add crc

/*	
	// CRC Calculation
	*(data+4) = 0x00;
	for (uint8_t i=0; i<4;i++)
    {
        *(data+4) = _crc_ibutton_update(*(data+4),*(data+i));
    }
*/

	//printMessageHexStr();
	sendMessage();
}


