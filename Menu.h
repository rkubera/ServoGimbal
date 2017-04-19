/*
 * Menu.h
 *
 *  Created on: 29 mar 2017
 *      Author: radoslawkubera
 */
#include "libraries/AutoSerial/AutoSerial.h"

#ifndef MENU_H_
#define MENU_H_

void serviceMenu(AutoSerial &mySerial);
void showActualConfig(AutoSerial &mySerial);
void changePwmFrequency(AutoSerial &mySerial);
void showGyroValues(AutoSerial &mySerial);
uint8_t detectGyroOrientation(AutoSerial &mySerial);
void setPIDValues (AutoSerial &mySerial);
void changePIDValue (uint8_t type,AutoSerial &mySerial);
void changeDIRECTIONValue (uint8_t type,AutoSerial &mySerial);
void showPIDValues(AutoSerial &mySerial);
#if defined USE_MAVLINK
void mavlinkSettings (AutoSerial &mySerial);
void showMavlinkValues(AutoSerial &mySerial);
void setMavlinkRCChannel(uint8_t type,AutoSerial &mySerial);
#endif
void drawMenuSeparator(AutoSerial &mySerial);
void drawMainMenu(AutoSerial &mySerial);
void drawSetPIDValues(AutoSerial &mySerial);
void drawMavlinkProtocolSettings(AutoSerial &mySerial);
#endif /* MENU_H_ */
