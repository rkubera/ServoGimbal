/*
 * AutoSerial.h
 *
 *  Created on: 29 mar 2017
 *      Author: Radek Kubera (rkubera)
 */

#include "Arduino.h"
#include <Stream.h>
#if defined(ARDUINO_ARCH_AVR)
#include "SoftwareSerial.h"
#endif

#ifndef AUTOSERIAL_H_
#define AUTOSERIAL_H_

#define SERIAL_PORT_HARDWARE_COUNT 0

#ifdef SERIAL_PORT_HARDWARE
#undef SERIAL_PORT_SOFTWARE_COUNT
#define SERIAL_PORT_SOFTWARE_COUNT 1
#endif

#ifdef SERIAL_PORT_HARDWARE1
#undef SERIAL_PORT_SOFTWARE_COUNT
#define SERIAL_PORT_SOFTWARE_COUNT 2
#endif

#ifdef SERIAL_PORT_HARDWARE2
#undef SERIAL_PORT_SOFTWARE_COUNT
#define SERIAL_PORT_SOFTWARE_COUNT 3
#endif

#ifdef SERIAL_PORT_HARDWARE3
#undef SERIAL_PORT_SOFTWARE_COUNT
#define SERIAL_PORT_SOFTWARE_COUNT 4
#endif

class AutoSerial : public Stream {
private:
	uint8_t myType;
	uint8_t myNumber;
#if defined(ARDUINO_ARCH_AVR)
	SoftwareSerial *mySerial = NULL;
#endif
	bool RxTxDefined = false;
public:
	AutoSerial();
	virtual ~AutoSerial();
	bool isHardwareSerial();
	bool setRxTxPins (uint8_t Rx,uint8_t Tx);
	uint8_t getSerialNumber();
	void begin (long speed);
	int peek();
	bool listen();
	void end();
	bool isListening();
	bool stopListening();
	bool overflow();
	virtual size_t write(uint8_t byte);
	virtual int read();
	virtual int available();
	virtual void flush();
	using Print::write;
	operator bool() {
		switch (this->myType) {
		case 0:
			switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
			case 0:
				if (Serial) return true;
				else return false;
				break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
			case 1:
				if (Serial1) return true;
				else return false;
				break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
			case 2:
				if (Serial2) return true;
				else return false;
				break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
			case 3:
				if (Serial3) return true;
				else return false;
				break;
#endif
			default:
				break;
			}
			break;
#if defined(ARDUINO_ARCH_AVR)
			case 1:
				if (this->RxTxDefined) {
					if (this->mySerial) return true;
				}
				break;
#endif
		}
		return false;
	}
};

#endif /* AUTOSERIAL_H_ */
