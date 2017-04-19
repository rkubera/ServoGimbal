/******************************************************************
 * PID Simple Example (Augmented with Processing.org Communication)
 * Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 ******************************************************************/
/********************************************
 * Serial Communication functions / helpers
 ********************************************/

#include "libraries/AutoSerial/AutoSerial.h"
#include "Arduino.h"
#include "EEPROM.h"

#include "GlobalDefines.h"
#include "libraries/PID/PID_v1.h"
#include "Menu.h"

extern float posRoll, posPitch;
extern float setRollPoint, setPitchPoint;

extern PID pitchPID;
extern PID rollPID;

extern int8_t rollPIDDirection;
extern int8_t pitchPIDDirection;

extern uint16_t pwmFrequency;

#if defined USE_MAVLINK
extern uint8_t mavlinkRollRCChannel;
extern uint8_t mavlinkPitchRCChannel;
#endif

extern float pitch, roll;

union {                // This Data structure lets
	byte asBytes[24];    // us take the byte array
	float asFloat[5];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2: 0=Roll, 1=Pitch, else = ? error ?
//  3-6: float setpoint
//  7-10: float input
//  11-14: float output
//  15-18: float P_Param
//  19-22: float I_Param
//  23-26: float D_Param
void SerialDebugReceive(AutoSerial &mySerial) {
	// read the bytes sent from Processing
	uint8_t index=0;
	byte Auto_Man = -1;
	char PWMFreq;
	char command;
	byte Direct_Reverse = -1;
	byte ROLL_PITCH = -1;
	while(mySerial.available() && index<27) {
		if(index==0) {
			command = mySerial.read();
			if (command=='b') {
				detectGyroOrientation(mySerial);
			}
			else if (command=='m') {
				PWMFreq = mySerial.read();
				if (PWMFreq==2) {
					pwmFrequency = 333;
				}
				else if (PWMFreq==1) {
					pwmFrequency = 50;
				}
				EEPROM.put(pwmFrequencyAddr, pwmFrequency);
#if defined USE_MAVLINK
				mavlinkRollRCChannel = mySerial.read();
				mavlinkPitchRCChannel = mySerial.read();
				EEPROM.put(mavlinkRollRCChannelAddr,mavlinkRollRCChannel);
				EEPROM.put(mavlinkPitchRCChannelAddr,mavlinkPitchRCChannel);
#else
				mySerial.read();
				mySerial.read();
#endif
			}
			else {
				Auto_Man = command;
				index++;
			}
		}
		else if(index==1) {
			Direct_Reverse = mySerial.read();
			index++;
		}
		else if(index==2) {
			ROLL_PITCH = mySerial.read();
			index++;
		}
		else {
			foo.asBytes[index-3] = mySerial.read();
			index++;
		}
	}

	// if the information we got was in the correct format,
	// read it into the system
	if(index==27  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1) && (ROLL_PITCH==0 || ROLL_PITCH==1)) {
		if (ROLL_PITCH==0) {
			//setRollPoint=float(foo.asFloat[0]);
			//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
			//   value of "Input"  in most cases (as
			//   in this one) this is not needed.
			if(Auto_Man==0) {                     // * only change the output if we are in manual mode.  otherwise we'll get an
				posRoll=float(foo.asFloat[2]);      //   output blip, then the controller will
			}                                     //   overwrite.

			float p, i, d;                       // * read in and set the controller tunings
			p = float(foo.asFloat[3]);           //
			i = float(foo.asFloat[4]);           //
			d = float(foo.asFloat[5]);           //
			rollPID.SetTunings(p, i, d);            //

			EEPROM.put(RollPIDKpAddr,p);
			EEPROM.put(RollPIDKiAddr,i);
			EEPROM.put(RollPIDKdAddr,d);

			if(Auto_Man==0) rollPID.SetMode(MANUAL);// * set the controller mode
			else rollPID.SetMode(AUTOMATIC);             //

			if(Direct_Reverse==0) {
				rollPID.SetControllerDirection(DIRECT);// * set the controller Direction
				EEPROM.put(rollPIDDirectionAddr, 1);
				rollPIDDirection = 1;
			}
			else {
				rollPID.SetControllerDirection(REVERSE);          //
				EEPROM.put(rollPIDDirectionAddr, -1);
				rollPIDDirection = -1;
			}
		}
		else {
			//setPitchPoint=float(foo.asFloat[0]);
			//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
			//   value of "Input"  in most cases (as
			//   in this one) this is not needed.
			if(Auto_Man==0) {                      // * only change the output if we are in manual mode.  otherwise we'll get an
				posPitch=float(foo.asFloat[2]);      //   output blip, then the controller will
			}                                     //   overwrite.

			float p, i, d;                       // * read in and set the controller tunings
			p = float(foo.asFloat[3]);           //
			i = float(foo.asFloat[4]);           //
			d = float(foo.asFloat[5]);           //
			pitchPID.SetTunings(p, i, d);            //

			EEPROM.put(PitchPIDKpAddr,p);
			EEPROM.put(PitchPIDKiAddr,i);
			EEPROM.put(PitchPIDKdAddr,d);

			if(Auto_Man==0) pitchPID.SetMode(MANUAL);// * set the controller mode
			else pitchPID.SetMode(AUTOMATIC);             //

			if(Direct_Reverse==0) {
				pitchPID.SetControllerDirection(DIRECT);// * set the controller Direction
				EEPROM.put(pitchPIDDirectionAddr, 1);
				pitchPIDDirection = 1;
			}
			else {
				pitchPID.SetControllerDirection(REVERSE);          //
				EEPROM.put(pitchPIDDirectionAddr, -1);
				pitchPIDDirection = -1;
			}
		}
	}
	mySerial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting

void printSpace (AutoSerial &mySerial) {
	mySerial.print(F(" "));
}
void SerialDebugSend(uint8_t type,AutoSerial &mySerial) {
	//type = 1 - roll
	//type = 2 - pitch
	//type = 3 - config
	if (type==3) {
		mySerial.print(F("CONFIG "));
		mySerial.print(pwmFrequency);
		printSpace(mySerial);
#if defined USE_MAVLINK
		mySerial.print(mavlinkRollRCChannel,DEC);
		printSpace(mySerial);
		mySerial.print(mavlinkPitchRCChannel,DEC);
#else
		mySerial.print(0,DEC);
		printSpace(mySerial);
		mySerial.print(0,DEC);
#endif
		mySerial.println();
	}
	else if (type==1) {
		mySerial.print(F("PID "));
		mySerial.print(setRollPoint,FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(roll,FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(posRoll,FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(rollPID.GetKp(),FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(rollPID.GetKi(),FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(rollPID.GetKd(),FLOAT_PRECISION);
		printSpace(mySerial);
		if(rollPID.GetMode()==AUTOMATIC) mySerial.print(F("Automatic"));
		else mySerial.print(F("Manual"));
		printSpace(mySerial);
		if(rollPID.GetDirection()==DIRECT) mySerial.print(F("Direct"));
		else mySerial.print(F("Reverse"));
		printSpace(mySerial);
		mySerial.println(F("ROLL"));
	}
	else if (type==2){
		mySerial.print(F("PID "));
		mySerial.print(setPitchPoint,FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(pitch,FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(posPitch,FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(pitchPID.GetKp(),FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(pitchPID.GetKi(),FLOAT_PRECISION);
		printSpace(mySerial);
		mySerial.print(pitchPID.GetKd(),FLOAT_PRECISION);
		printSpace(mySerial);
		if(pitchPID.GetMode()==AUTOMATIC) mySerial.print(F("Automatic"));
		else mySerial.print(F("Manual"));
		printSpace(mySerial);
		if(pitchPID.GetDirection()==DIRECT) mySerial.print(F("Direct"));
		else mySerial.print(F("Reverse"));
		printSpace(mySerial);
		mySerial.println(F("PITCH"));
	}
}
