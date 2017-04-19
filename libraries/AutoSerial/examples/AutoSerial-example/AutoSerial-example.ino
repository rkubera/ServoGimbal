/*
 * AutoSerial-example.ino
 *
 *  Created on: 29 mar 2017
 *      Author: Radek Kubera (rkubera)
 *
 * Example creates 3 serial ports: usbSerial, debugSerial and inoSerial
 * and check if serial it is hardware os sofrware serial port
 */

#include "AutoSerial.h"

AutoSerial usbSerial;
AutoSerial debugSerial;
AutoSerial infoSerial;


void setup() {
  // put your setup code here, to run once:

  //Declared as first serial.
  if (!usbSerial.isHardwareSerial()) {
    usbSerial.setRxTxPins(4,5);
  }
  usbSerial.begin(57600);
  usbSerial.print("This is usbSerial. Serial number is:");
  usbSerial.println(usbSerial.getSerialNumber(), DEC);
  if (usbSerial.isHardwareSerial()) {
    usbSerial.println("This is hardware serial port"); 
  }
  else {
    usbSerial.println("This is software serial port");
  }
  
  //Declared as second serial
  if (!debugSerial.isHardwareSerial()) {
    debugSerial.setRxTxPins(6,7);
  }
  debugSerial.begin(57600);
  debugSerial.print("This is debugSerial. Serial number is:");
  debugSerial.println(debugSerial.getSerialNumber(), DEC);
  if (debugSerial.isHardwareSerial()) {
    debugSerial.println("This is hardware serial port"); 
  }
  else {
    debugSerial.println("This is software serial port");
  }

  //Declared as third serial
  if (!infoSerial.isHardwareSerial()) {
    infoSerial.setRxTxPins(8,9);
  }
  infoSerial.begin(57600);
  infoSerial.print("This is infoSerial. Serial number is:");
  infoSerial.println(infoSerial.getSerialNumber(), DEC);
  if (infoSerial.isHardwareSerial()) {
    infoSerial.println("This is hardware serial port"); 
  }
  else {
    infoSerial.println("This is software serial port");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
