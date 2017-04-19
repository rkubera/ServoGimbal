/*
 * GlobalVariables.h
 *
 *  Created on: 4 kwi 2017
 *      Author: radoslawkubera
 */

#ifndef _PIDServoGimbal_H_
#define _PIDServoGimbal_H_
//add your includes for the project PIDServoGimbal here
#include "Arduino.h"
#include "libraries/PID/PID_v1.h"
#if defined USE_MAVLINK
#include "mavlink.h"
#endif
#include "libraries/FrServo/FrServo.h"
#include "libraries/MPU6050/MPU6050.h"
#include "EEPROM.h"
#include "GlobalDefines.h"
#include "libraries/AutoSerial/AutoSerial.h"
//end of add your includes here

//Serials
AutoSerial debugSerial;
#if defined USE_MAVLINK
AutoSerial mavlinkSerial;
#endif

//Servos
FrServo rollServo;
FrServo pitchServo;

//RC
float rc_pitch;
float rc_roll;
uint16_t rollPWMIn, pitchPWMIn;
uint16_t rollMavlinkIn, pitchMavlinkIn;
//Roll, Pitch
float pitch, roll;

//MPU6050
MPU6050 mpu;

//Timers
#if defined USE_MAVLINK
uint32_t mavlink_timer;
#endif
uint32_t pid_timer, config_timer, serial_timer;

//Config
int8_t rollOrientation;
int8_t pitchOrientation;
int8_t yawOrientation;
int8_t rollPIDDirection;
int8_t pitchPIDDirection;
uint16_t pwmFrequency;
#if defined USE_MAVLINK
uint8_t mavlinkRollRCChannel;
uint8_t mavlinkPitchRCChannel;
#endif

//Debug
uint8_t debug_mode = 0;

//PID
float pitchKp=0.10, pitchKd=0.0001, pitchKi=0.001;
float rollKp=0.10, rollKd=0.0001, rollKi=0.001;

float posRoll, posPitch;
float posRollPID, posPitchPID;
float setRollPoint, setPitchPoint;

PID pitchPID(&pitch, &posPitchPID, &setPitchPoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&roll, &posRollPID, &setRollPoint, rollKp, rollKi, rollKd, DIRECT);

//Do not add code below this line
#endif /* _PIDServoGimbal_H_ */

