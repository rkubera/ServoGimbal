/*
 * SerialDebug.h
 *
 *  Created on: 31 mar 2017
 *      Author: radoslawkubera
 */
#include "libraries/AutoSerial/AutoSerial.h"
#ifndef SERIALDEBUG_H_
#define SERIALDEBUG_H_

void SerialDebugReceive(AutoSerial &mySerial);
void SerialDebugSend(uint8_t type, AutoSerial &mySerial);
void printSpace (AutoSerial &mySerial);

#endif /* SERIALDEBUG_H_ */
