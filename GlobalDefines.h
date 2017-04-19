/*
 * GlobalDefines.h
 *
 *  Created on: 29 mar 2017
 *      Author: radoslawkubera
 */

#ifndef _Define_H_
#define _Define_H_

//Version
#define SG_VERSION 0.1

//QuickStats
#define QSTATS_BUFFER 40

//Mavlink
//#define USE_MAVLINK 1

//Pins
#define ROLL_SERVO_PIN 7
#define PITCH_SERVO_PIN 6

#define SOFTWARE_SERIAL_RX_PIN 5
#define SOFTWARE_SERIAL_TX_PIN 4

//PID
#define PID_SAMPLE_TIME 1 //ms
#define MAX_PID_LIMIT 90 //+-90 degs

//EEPROM
#define serviceModeAddr 0
#define rollOrientationAddr 2
#define pitchOrientationAddr 4
#define yawOrientationAddr 6
#define pwmFrequencyAddr 16
#define rollPIDDirectionAddr 8
#define pitchPIDDirectionAddr 10
#define mavlinkRollRCChannelAddr 12
#define mavlinkPitchRCChannelAddr 14
#define RollPIDKpAddr 20
#define RollPIDKiAddr 24
#define RollPIDKdAddr 28
#define PitchPIDKpAddr 32
#define PitchPIDKiAddr 36
#define PitchPIDKdAddr 40

#define eepromOffset 44

//Other delclarations
#define FLOAT_PRECISION 6

#define _ROLL 1
#define _PITCH 2

#define _ROLL_KP 1
#define _ROLL_KI 2
#define _ROLL_KD 3
#define _ROLL_DIRECTION 4
#define _PITCH_KP 5
#define _PITCH_KI 6
#define _PITCH_KD 7
#define _PITCH_DIRECTION 4

#endif /* _Define_H_ */



