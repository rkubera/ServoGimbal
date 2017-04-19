/*
 * PIDServoGimbal.cpp
 *
 *  Created on: 4 kwi 2017
 *      Author: radoslawkubera
 */
#include "GlobalDefines.h"
#include "libraries/AutoSerial/AutoSerial.h"
#include "SerialDebug.h"
#include "Menu.h"
#include "GlobalVariables.h"
#include "Gyro.h"
#include "PWMIn.h"

void setup() {
	//Read config data from EEProm
	EEPROM.get(rollOrientationAddr, rollOrientation);
	EEPROM.get(pitchOrientationAddr, pitchOrientation);
	EEPROM.get(yawOrientationAddr, yawOrientation);
	EEPROM.get(pwmFrequencyAddr, pwmFrequency);

	EEPROM.get(RollPIDKpAddr,rollKp);
	EEPROM.get(RollPIDKiAddr,rollKi);
	EEPROM.get(RollPIDKdAddr,rollKd);
	EEPROM.get(rollPIDDirectionAddr, rollPIDDirection);

	EEPROM.get(PitchPIDKpAddr,pitchKp);
	EEPROM.get(PitchPIDKiAddr,pitchKi);
	EEPROM.get(PitchPIDKdAddr,pitchKd);
	EEPROM.get(pitchPIDDirectionAddr, pitchPIDDirection);
#if defined USE_MAVLINK
	EEPROM.get(mavlinkRollRCChannelAddr,mavlinkRollRCChannel);
	EEPROM.get(mavlinkPitchRCChannelAddr,mavlinkPitchRCChannel);
	if (mavlinkRollRCChannel<1 || mavlinkRollRCChannel>16) {
		mavlinkRollRCChannel = 16;
	}

	if (mavlinkPitchRCChannel<1 || mavlinkPitchRCChannel>16) {
		mavlinkPitchRCChannel = 16;
	}
#endif
	if (pwmFrequency!=50 && pwmFrequency!=333) {
		pwmFrequency = 50;
	}

	//Init Serials
	if (!debugSerial.isHardwareSerial()) {
		debugSerial.setRxTxPins(SOFTWARE_SERIAL_RX_PIN,SOFTWARE_SERIAL_TX_PIN);
	}

	//debugSerial.begin(57600);
	debugSerial.begin(115200);

#if defined USE_MAVLINK
	if (!mavlinkSerial.isHardwareSerial()) {
		mavlinkSerial.setRxTxPins(SOFTWARE_SERIAL_RX_PIN,SOFTWARE_SERIAL_TX_PIN);
	}
	mavlinkSerial.begin(57600);
#endif
	//Init Servos
	FrServo::setFrequency(pwmFrequency);

	rollServo.attach(ROLL_SERVO_PIN);
	rollServo.writeMicroseconds(1500);

	pitchServo.attach(PITCH_SERVO_PIN);
	pitchServo.writeMicroseconds(1500);

	//Init Gyro
	initGyro();

	//Init PID
	setPitchPoint = 0;
	setRollPoint = 0;

	posRoll = posRollPID = 0;
	posPitch = posPitchPID = 0;

	pitchPID.SetOutputLimits(-MAX_PID_LIMIT,MAX_PID_LIMIT);
	pitchPID.SetMode(AUTOMATIC);
	pitchPID.SetSampleTime(PID_SAMPLE_TIME);
	pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
	if (pitchPIDDirection==1) {
		pitchPID.SetControllerDirection(DIRECT);
	}
	else {
		pitchPID.SetControllerDirection(REVERSE);
	}

	rollPID.SetOutputLimits(-MAX_PID_LIMIT,MAX_PID_LIMIT);
	rollPID.SetMode(AUTOMATIC);
	rollPID.SetSampleTime(PID_SAMPLE_TIME);
	rollPID.SetTunings(rollKp, rollKi, rollKd);
	if (rollPIDDirection==1) {
		rollPID.SetControllerDirection(DIRECT);
	}
	else {
		rollPID.SetControllerDirection(REVERSE);
	}

	//Init PWMin
	InitPWNIn();

	//Menu
	serviceMenu(debugSerial);

#if defined USE_MAVLINK
	//Init mavlink
	for(int n = 0; n < 3; n++){
		request_mavlink_rates();//Three times to certify it will be readed
		delay(50);
	}
#endif
	//Init timers
	pid_timer = config_timer = serial_timer = millis();
}

void loop() {
	//Ifdebug mode is on, send and receive data from debug comm port
	if(debug_mode==1) {
		if (millis()>serial_timer) {
			SerialDebugReceive(debugSerial);
			SerialDebugSend(1,debugSerial);		//Send Roll PID
			SerialDebugSend(2,debugSerial);		//Send Pitch PID
			serial_timer+=100;
		}
		if (millis()>config_timer) {
			SerialDebugSend(3,debugSerial);		//Send configuration
			config_timer+=1000;
		}
	}
#if defined USE_MAVLINK
	//Read data from mavlink comm port
	float mavlink_dt = (float)(micros() - mavlink_timer) / 1000; // Calculate delta time in ms
	if (mavlink_dt>20) {
		mavlink_timer = micros();
		read_mavlink();
	}
#endif
	//Get RC channels values from PWM In (Pin 2 and 3)
	getPWMInValues(rollPWMIn, pitchPWMIn);

#if defined USE_MAVLINK
	//Get RC channels values from mavlink
	getMavlinkRCRollPitch (rollMavlinkIn, pitchMavlinkIn, mavlinkRollRCChannel, mavlinkPitchRCChannel);
#endif
	float pid_dt = (float)(micros() - pid_timer) / 1000;

	if (pid_dt>=PID_SAMPLE_TIME) {
		//Calculate setpoints - angles offsets for roll and pitch using RC channels)

		//Check if PWM RC channel is connected
		if (rollPWMIn>0) {
			rc_roll = ((((float)rollPWMIn-1000)*180)/1000)-90;
		}
		else {
#if defined USE_MAVLINK
			//PWM not connected, check mavlink RC channel value
			if (rollMavlinkIn>0) {
				rc_roll = map (rollMavlinkIn,1000,2000,-90,90);
			}
			//No PWM and Mavlink data
			else {
				rc_roll = 0;
			}
#else
			rc_roll = 0;
#endif
		}

		//Check if PWM RC channel is connected
		if (pitchPWMIn>0) {
			rc_pitch = ((((float)pitchPWMIn-1000)*180)/1000)-90;
		}
		else {
#if defined USE_MAVLINK
			//PWM not connected, check mavlink RC channel value
			if (pitchMavlinkIn>0) {
				rc_pitch = map (pitchMavlinkIn,1000,2000,-90,90);
			}
			//No PWM and Mavlink data
			else {
				rc_pitch = 0;
			}
#else
			rc_pitch = 0;
#endif
		}

		//Update PID setpoints
		setRollPoint = rc_roll;
		setPitchPoint = rc_pitch;

		//Calculate PID
		pid_timer = micros();
		pitchPID.Compute();
		rollPID.Compute();

		//Calclulate new angle positions using PID calculations
		posPitch = posPitch+(posPitchPID);
		if (posPitch>=90) {
			posPitch = 90;
		}
		else if (posPitch<=-90) {
			posPitch = -90;
		}

		posRoll = posRoll+(posRollPID);
		if (posRoll>=90) {
			posRoll = 90;
		}
		else if (posRoll<=-90) {
			posRoll = -90;
		}

		//Map angles to servo PWM values
		uint16_t rollPWM, pitchPWM;
		rollPWM = map (posRoll,-90,90,1000,2000);
		pitchPWM = map (posPitch,-90,90,1000,2000);

		//Set new servos positions
		rollServo.writeMicroseconds(rollPWM);
		pitchServo.writeMicroseconds(pitchPWM);
	}

	//Get data from Gyro
	getRollPitch (roll,pitch);
}
