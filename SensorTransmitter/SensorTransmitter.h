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


#ifndef SensorTransmitter_h
#define SensorTransmitter_h

#include <Arduino.h>

//#define USE_MANCHESTER_CODING // Define to use Mancheser encoding, if not defined RZT Coding will be used
//#define TRANS_DEBUG // Prints out some Debug Message on serial Port


//#define USE_MANCHESTER_CODING // Define to use Mancheser encoding, if not defined RZT Coding will be used

    #define BAUD_300 0
    #define BAUD_600 1
    #define BAUD_1200 2
    #define BAUD_2400 3
    #define BAUD_4800 4
    #define BAUD_9600 5
    #define BAUD_19200 6
    #define BAUD_38400 7

#ifdef USE_MANCHESTER_CODING
    //timer scaling factors for different transmission speeds
    #define MAN_300 0
    #define MAN_600 1
    #define MAN_1200 2
    #define MAN_2400 3
    #define MAN_4800 4
    #define MAN_9600 5
    #define MAN_19200 6
    #define MAN_38400 7

class ManchesterTransmit {
	public:
		// Constructor
		ManchesterTransmit(uint8_t numMsgBytesIN, uint8_t transmitterPinIN, uint8_t clockIN,uint8_t preambleIN);
		void sendMessage();

	protected:
		byte* data;

	private:
		uint8_t transmitterPin;
		uint8_t repeat;
        uint16_t clock;
        uint8_t preamble;
        uint8_t numMsgBytes;
		uint8_t pauseFact;
		void sendZero(void);
		void sendOne(void);
		void setRepeat(uint8_t repeatIN);
        void setPauseFact(uint8_t pauseFactIN);

};
#else

class SensorTransmitter {
	public:
		// Constructor
		SensorTransmitter(byte numMsgBytesIN, byte transmitterPinIN, int clockIN, byte syncFactIN, byte lowFactIN, byte highFactIN);

		void sendMessage();
		void setRepeat(byte repeatIN);
		void setPauseTime(byte pauseTimeIN);

	protected:
		byte* data;
		uint16_t clock;
		byte numMsgBytes;
		byte pauseTime;
		void printMessageHexStr();
		void sendZero(void);
		void sendOne(void);


	private:
		void sendBit(byte fact);
		void sendByte(byte b);

		byte transmitterPin;
		byte repeat;
		byte syncFact;
		byte lowFact;
		byte highFact;


};
#endif

#ifdef USE_MANCHESTER_CODING
class asTransmitter : public ManchesterTransmit {
#else
class asTransmitter : public SensorTransmitter {
#endif
	public:
		/**
		 * ArduinoSensor transmitter
		 * transmitterPin 		- Arduino-pin connected to the 433MHz transmitter
		 * id 				- a unique ID for the sensor [0..64]
		 * type 			- device type [0..128]
		 * SensorTransmitter::SensorTransmitter (constructor)
		 */
		asTransmitter(byte type, byte IdIN, byte transmitterPin);
		/**
		 * value 			- sensor value
		 * battery			- battery status [0..3](0 - bad, 1 - change, 2 - ok, 3 - optimal)
		 * trigger			- auto (0) / manual (1)
		 */
		void send(uint16_t value, byte battery, bool trigger);

	private:
		byte type;
		byte id;
};

#endif
