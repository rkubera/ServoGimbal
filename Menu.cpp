/*
 * Menu.cpp
 *
 *  Created on: 29 mar 2017
 *      Author: radoslawkubera
 */
#include "Arduino.h"
#include "Menu.h"
#include "EEPROM.h"

#include "GlobalDefines.h"
#include "libraries/FrServo/FrServo.h"
#include "libraries/MPU6050/MPU6050.h"
#include "Gyro.h"
#include "libraries/AutoSerial/AutoSerial.h"
#include "libraries/PID/PID_v1.h"

extern MPU6050 mpu;

extern uint8_t debug_mode;
extern char rollOrientation;
extern char pitchOrientation;
extern char yawOrientation;

extern uint16_t pwmFrequency;

extern float pitchKp, pitchKd, pitchKi;
extern float rollKp, rollKd, rollKi;

extern int8_t rollPIDDirection;
extern int8_t pitchPIDDirection;

#if defined USE_MAVLINK
extern uint8_t mavlinkRollRCChannel;
extern uint8_t mavlinkPitchRCChannel;
#endif

extern FrServo rollServo;
extern FrServo pitchServo;

extern float pitch, roll;

extern PID pitchPID, rollPID;

void drawMenuSeparator(AutoSerial &mySerial) {
	for (uint8_t i=0; i<20; i++) {
		mySerial.print(F("-"));
	}
	mySerial.println();
}

void drawMainMenu(AutoSerial &mySerial) {
	mySerial.println (F("MainMenu->"));
}

void drawSetPIDValues(AutoSerial &mySerial) {
	mySerial.print (F("Set PID Values->"));
}

#if defined USE_MAVLINK
void drawMavlinkProtocolSettings(AutoSerial &mySerial) {
	mySerial.print("Mavlink protocol settings->");
}
#endif

void changeDIRECTIONValue (uint8_t type,AutoSerial &mySerial) {
	uint8_t ok = 1;
	while (true) {
		if (ok==1) {
			drawSetPIDValues(mySerial);
			if (type==_ROLL_DIRECTION) {
				drawMainMenu(mySerial);
				mySerial.println (F("PID Roll Direction"));
			}
			else {
				mySerial.println (F("POD Pitch Direction"));
			}

			drawMenuSeparator(mySerial);
			mySerial.println     (F("a) Direct"));
			mySerial.println     (F("b) Reverse"));
			mySerial.println     (F("x) Return to Set PID Values"));
			drawMenuSeparator(mySerial);
			ok = 0;
		}

		char incomingByte;
		while (mySerial.available()==0) {
			//wait for serial
		}
		incomingByte = mySerial.read();
		if (char(incomingByte)=='a') {
			ok = 1;
			mySerial.println (F("New direction: Direct"));
			if (type==_ROLL_DIRECTION) {
				rollPID.SetControllerDirection(DIRECT);
				EEPROM.put(rollPIDDirectionAddr, 1);
				rollPIDDirection = 1;
			}
			else {
				pitchPID.SetControllerDirection(DIRECT);
				EEPROM.put(pitchPIDDirectionAddr, 1);
				pitchPIDDirection = 1;
			}
			delay(500);
			drawMenuSeparator(mySerial);
		}
		if (char(incomingByte)=='b') {
			ok = 1;
			mySerial.println (F("New direction: Reverse"));
			if (type==_ROLL_DIRECTION) {
				rollPID.SetControllerDirection(REVERSE);
				EEPROM.put(rollPIDDirectionAddr, -1);
				rollPIDDirection = -1;
			}
			else {
				pitchPID.SetControllerDirection(REVERSE);
				EEPROM.put(pitchPIDDirectionAddr, -1);
				pitchPIDDirection = -1;
			}
			delay(500);
			drawMenuSeparator(mySerial);
		}
		if (incomingByte=='x') {
			return;
		}
	}
}

void showPIDValues(AutoSerial &mySerial) {
	drawMainMenu(mySerial);
	drawSetPIDValues(mySerial);
	mySerial.println (F("Show PID Values"));
	drawMenuSeparator(mySerial);
	mySerial.print   (F("Roll Kp :"));
	mySerial.println(rollKp,FLOAT_PRECISION);
	mySerial.print   (F("Roll Ki :"));
	mySerial.println(rollKi,FLOAT_PRECISION);
	mySerial.print   (F("Roll Kd :"));
	mySerial.println(rollKd,FLOAT_PRECISION);
	mySerial.print   (F("Roll Direction :"));
	mySerial.println(rollPIDDirection,DEC);
	mySerial.print   (F("Pitch Kp :"));
	mySerial.println(pitchKp,FLOAT_PRECISION);
	mySerial.print   (F("Pitch Ki :"));
	mySerial.println(pitchKi,FLOAT_PRECISION);
	mySerial.print   (F("Pitch Kd :"));
	mySerial.println(pitchKd,FLOAT_PRECISION);
	mySerial.print   (F("Pitch Direction :"));
	mySerial.println(pitchPIDDirection,DEC);
	delay(500);
	drawMenuSeparator(mySerial);
}

void changePIDValue (uint8_t type,AutoSerial &mySerial) {
	uint8_t ok = 1;
	while (true) {
		if (ok==1) {
			drawMainMenu(mySerial);
			drawSetPIDValues(mySerial);
			mySerial.print("Change ");

			if (type==_ROLL_KP || type==_ROLL_KI || type==_ROLL_KD) {
				mySerial.print (F("ROLL "));
			}
			else {
				mySerial.print (F("PITCH "));
			}
			if (type==_ROLL_KP) {
				mySerial.println (F("Kp"));
			}
			if (type==_ROLL_KI) {
				mySerial.println (F("Ki"));
			}
			if (type==_ROLL_KD) {
				mySerial.println (F("Kd"));
			}
			if (type==_PITCH_KP) {
				mySerial.println (F("Kp"));
			}
			if (type==_PITCH_KI) {
				mySerial.println (F("Ki"));
			}
			if (type==_PITCH_KD) {
				mySerial.println (F("Kd"));
			}
			drawMenuSeparator(mySerial);
			mySerial.print       (F("Enter new value and press Enter (set dot as delimiter):"));
			ok = 0;
		}
		String inString = "";
		int inChar = 0;
		while (true) {
			if (mySerial.available()>0) {
				inChar = Serial.read();
				if (inChar=='\n') {
					if (inString.length()>0) {
						break;
					}
				}
				else {
					inString += (char)inChar;
					mySerial.print((char)inChar);
				}
			}
		}
		mySerial.println();
		float val = inString.toFloat();
		mySerial.print (F("New value: "));
		mySerial.println (val,FLOAT_PRECISION);

		if (type==_ROLL_KP) {
			rollKp = val;
		}
		if (type==_ROLL_KI) {
			rollKi = val;
		}
		if (type==_ROLL_KD) {
			rollKd = val;
		}
		if (type==_PITCH_KP) {
			pitchKp = val;
		}
		if (type==_PITCH_KI) {
			pitchKi = val;
		}
		if (type==_PITCH_KD) {
			pitchKd = val;
		}

		pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
		rollPID.SetTunings(rollKp, rollKi, rollKd);

		EEPROM.put(RollPIDKpAddr,rollKp);
		EEPROM.put(RollPIDKiAddr,rollKi);
		EEPROM.put(RollPIDKdAddr,rollKd);

		EEPROM.put(PitchPIDKpAddr,pitchKp);
		EEPROM.put(PitchPIDKiAddr,pitchKi);
		EEPROM.put(PitchPIDKdAddr,pitchKd);
		delay(500);
		drawMenuSeparator(mySerial);
		return;
	}
}

void setPIDValues (AutoSerial &mySerial) {
	uint8_t ok = 1;
	while (true) {
		if (ok==1) {
			drawMainMenu(mySerial);
			drawSetPIDValues(mySerial);
			mySerial.println ();
			drawMenuSeparator(mySerial);
			mySerial.println (F("a) Show PID values"));
			mySerial.println (F("b) Change ROLL Kp"));
			mySerial.println (F("c) Change ROLL Ki"));
			mySerial.println (F("d) Change ROLL Kd"));
			mySerial.println (F("e) Change ROLL Direction"));
			mySerial.println (F("f) Change PITCH Kp"));
			mySerial.println (F("g) Change PITCH Ki"));
			mySerial.println (F("h) Change PITCH Kd"));
			mySerial.println (F("i) Change PITCH Direction"));
			mySerial.println (F("x) Return to Main Menu"));
			drawMenuSeparator(mySerial);
			ok = 0;
		}
		char incomingByte;
		while (mySerial.available()==0) {
			//wait for serial
		}
		incomingByte = mySerial.read();
		if (char(incomingByte)=='a') {
			ok = 1;
			showPIDValues(mySerial);
		}
		if (char(incomingByte)=='b') {
			ok = 1;
			changePIDValue (_ROLL_KP,mySerial);
		}
		if (char(incomingByte)=='c') {
			ok = 1;
			changePIDValue (_ROLL_KI,mySerial);
		}
		if (char(incomingByte)=='d') {
			ok = 1;
			changePIDValue (_ROLL_KD,mySerial);
		}
		if (char(incomingByte)=='e') {
			ok = 1;
			changeDIRECTIONValue (_ROLL_DIRECTION,mySerial);
		}
		if (char(incomingByte)=='f') {
			ok = 1;
			changePIDValue (_PITCH_KP,mySerial);
		}
		if (char(incomingByte)=='g') {
			ok = 1;
			changePIDValue (_PITCH_KI,mySerial);
		}
		if (char(incomingByte)=='h') {
			ok = 1;
			changePIDValue (_PITCH_KD,mySerial);
		}
		if (char(incomingByte)=='i') {
			ok = 1;
			changeDIRECTIONValue (_PITCH_DIRECTION,mySerial);
		}
		if (char(incomingByte)=='x') {
			return;
		}
	}
}

#if defined USE_MAVLINK
void showMavlinkValues(AutoSerial &mySerial) {
	drawMainMenu(mySerial);
	drawMavlinkProtocolSettings(mySerial);
	mySerial.println (F("Show Mavlink values"));
	drawMenuSeparator(mySerial);
	mySerial.print   (F("Roll RC Channel :"));
	mySerial.println (mavlinkRollRCChannel,DEC);
	mySerial.print   (F("Pitch RC Channel :"));
	mySerial.println (mavlinkPitchRCChannel,DEC);
	delay(500);
	drawMenuSeparator(mySerial);
}

void setMavlinkRCChannel(uint8_t type,AutoSerial &mySerial) {
	uint8_t ok = 1;
	while (true) {
		if (ok==1) {
			if (type == _ROLL) {
				drawMainMenu(mySerial);
				drawMavlinkProtocolSettings(mySerial);
				mySerial.println (F("Set Roll RC channel"));
			}
			else {
				mySerial.println (F( "Set Pitch RC channel"));
			}
			drawMenuSeparator(mySerial);
			for (int i = 0; i<16; i++) {
				mySerial.print (char('a'+i));
				mySerial.print(F(") "));
				mySerial.print(i+1,DEC);
				mySerial.println (F(" RC channel"));
			}
			mySerial.println     (F("x) Return to Mavlink protocol settings"));
			drawMenuSeparator(mySerial);
			ok=0;
		}
		char incomingByte;
		uint8_t rc_channel;
		while (mySerial.available()==0) {
			//wait for serial
		}
		incomingByte = mySerial.read();
		if (char(incomingByte)>='x') {
			return;
		}
		if (char(incomingByte)>='a' && char(incomingByte)<='p') {
			rc_channel = uint8_t(incomingByte)-uint8_t('a')+1;
			mySerial.print(F("New RC channel: "));
			mySerial.println (rc_channel,DEC);
			ok = 1;
			if (type == _ROLL) {
				mavlinkRollRCChannel = rc_channel;
				EEPROM.put(mavlinkRollRCChannelAddr,mavlinkRollRCChannel);

			}
			else {
				mavlinkPitchRCChannel = rc_channel;
				EEPROM.put(mavlinkPitchRCChannelAddr,mavlinkPitchRCChannel);
			}
			delay(500);
			drawMenuSeparator(mySerial);
		}
	}
}

void mavlinkSettings (AutoSerial &mySerial) {
	uint8_t ok = 1;
	while (true) {
		if (ok==1) {
			drawMainMenu(mySerial);
			mySerial.println (F("Mavlink protocol settings"));
			drawMenuSeparator(mySerial);
			mySerial.println (F("a) Show mavlink values"));
			mySerial.println (F("b) Set Roll RC channel"));
			mySerial.println (F("c) Set Pitch RC channel"));
			mySerial.println (F("x) Return to Main Menu"));
			drawMenuSeparator(mySerial);
			ok = 0;
		}
		char incomingByte;
		while (mySerial.available()==0) {
			//wait for serial
		}
		incomingByte = mySerial.read();
		if (char(incomingByte)=='a') {
			ok = 1;
			showMavlinkValues (mySerial);
		}
		if (char(incomingByte)=='b') {
			ok = 1;
			setMavlinkRCChannel (_ROLL,mySerial);
		}
		if (char(incomingByte)=='c') {
			ok = 1;
			setMavlinkRCChannel (_PITCH,mySerial);
		}
		if (char(incomingByte)=='x') {
			return;
		}
	}
}
#endif

void serviceMenu(AutoSerial &mySerial) {
	mySerial.flush();
	unsigned long start = millis();
	uint8_t pressed = 0;
	char incomingByte;
	while (pressed == 1 || (millis()-start)<10000) {
		drawMenuSeparator(mySerial);
		mySerial.print   (F("ServoGimbal v."));
		mySerial.print   (SG_VERSION);
		mySerial.println (F(" by Radek Kubera"));
		mySerial.println (F("Main Menu"));
		drawMenuSeparator(mySerial);
		mySerial.println (F("a) Select gimbal type"));
		mySerial.println (F("b) Detect gyro orientation"));
		mySerial.println (F("c) Show actual configuration"));
		mySerial.println (F("d) Show roll/pitch values"));
#if defined USE_MAVLINK
		mySerial.println (F("g) Mavlink protocol settings"));
#endif
		mySerial.println (F("h) Change servo PWM frequency"));
		mySerial.println (F("s) Change PID values"));
		mySerial.println (F("p) Switch to debug mode and exit"));
		mySerial.println (F("x) Exit"));
		drawMenuSeparator(mySerial);
		while (pressed == 1 || (millis()-start)<10000) {
			if (mySerial.available() > 0) {
				pressed = 1;
				incomingByte = mySerial.read();
				if (char(incomingByte)=='b') {
					drawMainMenu(mySerial);
					mySerial.println (F("Detect gyro orientation"));
					drawMenuSeparator(mySerial);
					if (detectGyroOrientation(mySerial)==1) {
						mySerial.println (F("ERROR. Try again."));
					}
					break;
				}

				if (char(incomingByte)=='c') {
					drawMainMenu(mySerial);
					mySerial.println (F("Actual configuration"));
					drawMenuSeparator(mySerial);
					showActualConfig(mySerial);
					break;
				}

				if (char(incomingByte)=='d') {
					drawMainMenu(mySerial);
					mySerial.println (F("Show roll/pitch values"));
					//drawMenuSeparator(mySerial);
					showGyroValues(mySerial);
					break;
				}
#if defined USE_MAVLINK
				if (char(incomingByte)=='g') {
					mavlinkSettings(mySerial);
					break;
				}
#endif
				if (char(incomingByte)=='h') {
					drawMainMenu(mySerial);
					mySerial.println (F("Change servo PWM frequency"));
					drawMenuSeparator(mySerial);
					changePwmFrequency(mySerial);
					break;
				}

				if (char(incomingByte)=='s') {
					setPIDValues(mySerial);
					break;
				}

				if (char(incomingByte)=='p') {
					debug_mode=1;
					return;
				}

				if (char(incomingByte)=='x') {
					return;
				}
			}
		}
	}
}

void showActualConfig(AutoSerial &mySerial) {
	mySerial.print (F("rollOrientation :"));
	mySerial.println(rollOrientation,DEC);
	mySerial.print (F("pitchOrientation :"));
	mySerial.println(pitchOrientation,DEC);
	mySerial.print (F("yawOrientation :"));
	mySerial.println(yawOrientation,DEC);
	mySerial.print (F("pwmFrequency :"));
	mySerial.println(pwmFrequency,DEC);
	mySerial.print (F("PID Roll Kp :"));
	mySerial.println(rollKp,FLOAT_PRECISION);
	mySerial.print (F("PID Roll Ki :"));
	mySerial.println(rollKi,FLOAT_PRECISION);
	mySerial.print (F("PID Roll Kd :"));
	mySerial.println(rollKd,FLOAT_PRECISION);
	mySerial.print (F("PID Roll Direction :"));
	mySerial.println(rollPIDDirection,DEC);
	mySerial.print (F("PID Pitch Kp :"));
	mySerial.println(pitchKp,FLOAT_PRECISION);
	mySerial.print (F("PID Pitch Ki :"));
	mySerial.println(pitchKi,FLOAT_PRECISION);
	mySerial.print (F("PID Pitch Kd :"));
	mySerial.println(pitchKd,FLOAT_PRECISION);
	mySerial.print (F("PID Pitch Direction :"));
	mySerial.println(pitchPIDDirection,DEC);
#if defined USE_MAVLINK
	mySerial.print (F("Mavlink Roll RC Channel :"));
	mySerial.println(mavlinkRollRCChannel,DEC);
	mySerial.print (F("Mavlink Pitch RC Channel :"));
	mySerial.println(mavlinkPitchRCChannel,DEC);
#endif
	delay(500);
}
void changePwmFrequency(AutoSerial &mySerial) {
	mySerial.print(F("PWM Frequency changed to "));
	if (pwmFrequency==50) {
		pwmFrequency = 333;
	}
	else {
		pwmFrequency = 50;
		FrServo::setFrequency(pwmFrequency);
	}
	mySerial.print(pwmFrequency, DEC);
	mySerial.println(F("Hz"));
	EEPROM.put(pwmFrequencyAddr, pwmFrequency);
	delay(500);
}

void showGyroValues(AutoSerial &mySerial) {
	mySerial.println("x) Return to Main Menu (at any moment)");
	delay(500);
	char incomingByte;
	mySerial.println(F("Roll\tPitch"));
	mySerial.println(F("----\t-----"));
	mySerial.flush();
	while (true) {
		if (mySerial.available() > 0) {
			incomingByte = mySerial.read();
			if (char(incomingByte)=='x') {
				return;
			}
		}
		getRollPitch (roll,pitch);
		mySerial.print(roll);
		mySerial.print(F("\t"));
		mySerial.println(pitch);
		delay(100);
	}
}

uint8_t detectGyroOrientation(AutoSerial &mySerial) {

	float min_gyroX, min_gyroY, min_gyroZ;
	float max_gyroX, max_gyroY, max_gyroZ;
	float maxX, maxY, maxZ;
	Vector rawGyro;
	uint16_t i;

	mySerial.println(F("Detecting PITCH axis orientation"));
	for (i=1500; i>=1000; i--) {
		pitchServo.writeMicroseconds(i);
		delay(1);
	}

	rawGyro = mpu.readRawGyro();
	min_gyroX = rawGyro.XAxis;
	min_gyroY = rawGyro.YAxis;
	min_gyroZ = rawGyro.ZAxis;

	max_gyroX = 0;
	max_gyroY = 0;
	max_gyroZ = 0;

	for (i = 0; i<1000; i++) {
		rawGyro = mpu.readRawGyro();

		if (min_gyroX>rawGyro.XAxis) {
			min_gyroX = rawGyro.XAxis;
		}
		if (min_gyroY>rawGyro.YAxis) {
			min_gyroY = rawGyro.YAxis;
		}
		if (min_gyroZ>rawGyro.ZAxis) {
			min_gyroZ = rawGyro.ZAxis;
		}
		if (max_gyroX<rawGyro.XAxis) {
			max_gyroX = rawGyro.XAxis;
		}
		if (max_gyroY<rawGyro.YAxis) {
			max_gyroY = rawGyro.YAxis;
		}
		if (max_gyroZ<rawGyro.ZAxis) {
			max_gyroZ = rawGyro.ZAxis;
		}


		pitchServo.writeMicroseconds(1000+i);
		delay(1);

	}

	for (i=2000; i>=1500; i--) {
		pitchServo.writeMicroseconds(i);
		delay(1);
	}

	maxX = max_gyroX - min_gyroX;
	maxY = max_gyroY - min_gyroY;
	maxZ = max_gyroZ - min_gyroZ;

	if (maxX>maxY && maxX>maxZ) {
		mySerial.println (F("Pitch direction: X axis"));
		pitchOrientation = 1;
	}
	else if (maxY>maxX && maxY>maxZ) {
		mySerial.println (F("Pitch direction: Y axis"));
		pitchOrientation = 2;
	}
	else if (maxZ>maxX && maxZ>maxY) {
		mySerial.println (F("Pitch direction: Z axis"));
		pitchOrientation = 3;
	}
	else {
		//mySerial.println (F("ERROR. Ty again."));
		return 0;
	}


	pitchServo.writeMicroseconds(1500);
	delay (1000);
	mySerial.println(F("Detecting ROLL axis orientation"));
	for (i=1500; i>=1000; i--) {
		rollServo.writeMicroseconds(i);
		delay(1);
	}

	rawGyro = mpu.readRawGyro();
	min_gyroX = rawGyro.XAxis;
	min_gyroY = rawGyro.YAxis;
	min_gyroZ = rawGyro.ZAxis;

	max_gyroX = 0;
	max_gyroY = 0;
	max_gyroZ = 0;

	for (i = 0; i<1000; i++) {
		rawGyro = mpu.readRawGyro();
		if (min_gyroX>rawGyro.XAxis) {
			min_gyroX = rawGyro.XAxis;
		}
		if (min_gyroY>rawGyro.YAxis) {
			min_gyroY = rawGyro.YAxis;
		}
		if (min_gyroZ>rawGyro.ZAxis) {
			min_gyroZ = rawGyro.ZAxis;
		}
		if (max_gyroX<rawGyro.XAxis) {
			max_gyroX = rawGyro.XAxis;
		}
		if (max_gyroY<rawGyro.YAxis) {
			max_gyroY = rawGyro.YAxis;
		}
		if (max_gyroZ<rawGyro.ZAxis) {
			max_gyroZ = rawGyro.ZAxis;
		}

		rollServo.writeMicroseconds(1000+i);
		delay(1);
	}

	maxX = max_gyroX - min_gyroX;
	maxY = max_gyroY - min_gyroY;
	maxZ = max_gyroZ - min_gyroZ;

	if (maxX>maxY && maxX>maxZ) {
		if (pitchOrientation==1) {
			return 0;
		}
		else {
			mySerial.println (F("Roll direction: X axis"));
			EEPROM.put(rollOrientationAddr, 1);
			rollOrientation = 1;
			if (pitchOrientation==2) {
				EEPROM.put(pitchOrientationAddr, 2);
				EEPROM.put(yawOrientationAddr, 3);
				yawOrientation = 3;
			}
			else {
				EEPROM.put(pitchOrientationAddr, 3);
				EEPROM.put(yawOrientationAddr, 2);
				yawOrientation = 2;
			}
		}
	}
	else if (maxY>maxX && maxY>maxZ) {
		if (pitchOrientation==2) {
			return 0;
		}
		else {
			mySerial.println (F("Roll direction: Y axis"));
			EEPROM.put(rollOrientationAddr, 2);
			rollOrientation = 2;
			if (pitchOrientation==1) {
				EEPROM.put(pitchOrientationAddr, 1);
				EEPROM.put(yawOrientationAddr, 3);
				yawOrientation = 3;
			}
			else {
				EEPROM.put(pitchOrientationAddr, 3);
				EEPROM.put(yawOrientationAddr, 1);
				yawOrientation = 1;
			}
		}

	}
	else if (maxZ>maxX && maxZ>maxY) {
		if (pitchOrientation==3) {
			return 0;
		}
		else {
			mySerial.println (F("OK. Roll direction: Z axis"));
			EEPROM.put(rollOrientationAddr, 3);
			rollOrientation = 3;
			if (pitchOrientation==1) {
				EEPROM.put(pitchOrientationAddr, 1);
				EEPROM.put(yawOrientationAddr, 2);
				yawOrientation = 2;
			}
			else {
				EEPROM.put(pitchOrientationAddr, 2);
				EEPROM.put(yawOrientationAddr, 1);
				yawOrientation = 1;
			}
		}
	}
	else {
		return 0;
	}

	for (i=2000; i>=1500; i--) {
		rollServo.writeMicroseconds(i);
		delay(1);
	}

	return 1;
}
