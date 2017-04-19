/*
 * AutoSerial.cpp
 *
 *  Created on: 29 mar 2017
 *      Author: Radek Kubera (rkubera)
 */

#include "AutoSerial.h"

static uint8_t autoSerialPortsCount = 0;

AutoSerial::AutoSerial() {
	this->RxTxDefined = false;
	if (autoSerialPortsCount>SERIAL_PORT_HARDWARE_COUNT) {
		this->myType = 1;
	}
	else {
		this->myType = 0;
	}
	this->myNumber = autoSerialPortsCount;
	autoSerialPortsCount ++;
}

void AutoSerial::begin (long speed) {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			Serial.begin (speed);
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			Serial1.begin (speed);
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			Serial2.begin (speed);
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			Serial3.begin (speed);
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				this->mySerial->begin(speed);
			}
			break;
#endif
	}
}

size_t AutoSerial::write(uint8_t byte) {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			return Serial.write(byte);
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			return Serial1.write(byte);
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			return Serial2.write(byte);
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			return Serial3.write(byte);
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				return this->mySerial->write(byte);
			}
			break;
#endif
	}
	return 0;
}

int AutoSerial::read() {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			return Serial.read();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			return Serial1.read();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			return Serial2.read();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			return Serial3.read();
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				return this->mySerial->read();
			}
			break;
#endif
	}
	return 0;
}

int AutoSerial::available() {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			return Serial.available();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			return Serial1.available();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			return Serial2.available();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			return Serial3.available();
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				return this->mySerial->available();
			}
			break;
#endif
	}
	return 0;
}

void AutoSerial::flush() {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			Serial.flush();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			Serial1.flush();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			Serial2.flush();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			Serial3.flush();
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				this->mySerial->flush();
			}
			break;
#endif
	}
}

int AutoSerial::peek() {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			return Serial.peek();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			return Serial1.peek();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			return Serial2.peek();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			return Serial3.peek();
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				return this->mySerial->peek();
			}
			break;
#endif
	}
	return 0;
}

bool AutoSerial::listen() {
#if defined(ARDUINO_ARCH_AVR)
	if (this->myType==1) {
		if (this->RxTxDefined) {
			return this->mySerial->listen();
		}
		return false;
	}
#else
	if (this->myType==1) {
		return false;
	}
#endif
	return true;
}

void AutoSerial::end() {
	switch (this->myType) {
	case 0:
		switch (this->myNumber) {
#ifdef SERIAL_PORT_HARDWARE
		case 0:
			Serial.end();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE1
		case 1:
			Serial1.end();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE2
		case 2:
			Serial2.end();
			break;
#endif
#ifdef SERIAL_PORT_HARDWARE3
		case 3:
			return Serial3.end();
			break;
#endif
		default:
			break;
		}
		break;
#if defined(ARDUINO_ARCH_AVR)
		case 1:
			if (this->RxTxDefined) {
				this->mySerial->end();
			}
			break;
#endif
	}
}
bool AutoSerial::isListening() {
#if defined(ARDUINO_ARCH_AVR)
	if (this->myType==1) {
		if (this->RxTxDefined) {
			return this->mySerial->isListening();
		}
		return false;
	}
#else
	if (this->myType==1) {
		return false;
	}
#endif
	return true;
}
bool AutoSerial::stopListening() {
#if defined(ARDUINO_ARCH_AVR)
	if (this->myType==1) {
		if (this->RxTxDefined) {
			return this->mySerial->stopListening();
		}
		return true;
	}
#else
	if (this->myType==1) {
		return true;
	}
#endif
	return false;
}
bool AutoSerial::overflow() {
#if defined(ARDUINO_ARCH_AVR)
	if (this->myType==1) {
		if (this->RxTxDefined) {
			return this->mySerial->overflow();
		}
	}
#endif
	return false;
}

AutoSerial::~AutoSerial()
{
#if defined(ARDUINO_ARCH_AVR)
	if (this->RxTxDefined) {
		this->mySerial->~SoftwareSerial();
	}
#endif
	autoSerialPortsCount--;
}

bool AutoSerial::setRxTxPins (uint8_t Rx,uint8_t Tx) {
#if defined(ARDUINO_ARCH_AVR)
	if (this->myType==1) {
		if (this->RxTxDefined) {
			this->mySerial->~SoftwareSerial();
			this->RxTxDefined = false;
		}
		this->mySerial = new SoftwareSerial(Rx,Tx);
		this->RxTxDefined = true;
		return true;
	}
#endif
	return false;
}

bool AutoSerial::isHardwareSerial() {
	if (this->myType==0) return true;
	return false;
}

uint8_t AutoSerial::getSerialNumber() {
	return this->myNumber;
}
