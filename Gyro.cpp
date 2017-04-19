/*
 * Gyro.cpp
 *
 *  Created on: 29 mar 2017
 *      Author: radoslawkubera
 */
#include "Arduino.h"
#include "Gyro.h"

#include "GlobalDefines.h"
#include "libraries/MPU6050/MPU6050.h"
#include "libraries/AutoSerial/AutoSerial.h"


extern MPU6050 mpu;

extern char rollOrientation;
extern char pitchOrientation;
extern char yawOrientation;

extern AutoSerial debugSerial;

//Gyro values


/**************************************************
 * Gyro functions
 * ************************************************/

void initGyro() {
	//MPU6050
	while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		delay(500);
	}

	//disable MPU Low Pass Filter
	mpu.setDLPFMode(MPU6050_DLPF_6);
	debugSerial.println (F("MPU6050 ready"));
	mpu.calibrateGyro();
}

void getRollPitch (float &my_roll, float &my_pitch) {
	//static float lastRoll, lastPitch;
	static uint32_t timer, tmp_timer = 0;

	float accX, accY, accZ;
	float gyroX, gyroY /*, gyroZ*/;

	static float compAngleX, compAngleY; // Calculated angle using a Complimentary filter
	static float gyroXangle, gyroYangle; // Angle calculate using the gyro only

	Vector rawAccel = mpu.readRawAccel();
	Vector rawGyro = mpu.readRawGyro();

	if (rollOrientation==2) {
		accX = rawAccel.YAxis;
		gyroX = rawGyro.YAxis;
	}
	else if (rollOrientation==3) {
		accX = rawAccel.ZAxis;
		gyroX = rawGyro.ZAxis;
	}
	else {
		accX = rawAccel.XAxis;
		gyroX = rawGyro.XAxis;
	}

	if (pitchOrientation==2) {
		accY = rawAccel.YAxis;
		gyroY = rawGyro.YAxis;
	}
	else if (pitchOrientation==3) {
		accY = rawAccel.ZAxis;
		gyroY = rawGyro.ZAxis;
	}
	else {
		accY = rawAccel.XAxis;
		gyroY = rawGyro.XAxis;
	}

	if (yawOrientation==2) {
		accZ = rawAccel.YAxis;
		//gyroZ = rawGyro.YAxis;
	}
	else if (yawOrientation==3) {
		accZ = rawAccel.ZAxis;
		//gyroZ = rawGyro.ZAxis;
	}
	else {
		accZ = rawAccel.XAxis;
		//gyroZ = rawGyro.XAxis;
	}

	float calc_pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	float calc_roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

	float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
	tmp_timer = micros();

	if (dt>10000 || dt<=0) {

		gyroXangle = calc_roll;
		gyroYangle = calc_pitch;

		compAngleX = calc_roll;
		compAngleY = calc_pitch;

		//lastRoll = calc_roll;
		//lastPitch = calc_pitch;

		my_roll = calc_roll;;
		my_pitch = calc_pitch;

		timer = tmp_timer;
	}
	else {
		float gyroXrate = gyroX / 131.0; // Convert to deg/s
		float gyroYrate = gyroY / 131.0; // Convert to deg/s

		gyroXangle += gyroXrate * dt;
		gyroYangle += gyroYrate * dt;

		float tau=0.02;
		float a=0.0;
		a=tau/(tau+dt);

		compAngleX = a * (compAngleX + gyroXrate * dt) + (1-a) * calc_roll; // Calculate the angle using a Complimentary filter
		compAngleY = a * (compAngleY + gyroYrate * dt) + (1-a) * calc_pitch;

		/*
		if (isnan(compAngleX)) {
			compAngleX = lastRoll;
		}

		if (isnan(compAngleY)) {
			compAngleY = lastPitch;
		}

		if (isnan(gyroXangle)) {
			gyroXangle = lastRoll;
		}

		if (isnan(gyroYangle)) {
			gyroYangle = lastPitch;
		}

		my_roll = lastRoll = compAngleX;
		my_pitch = lastPitch = compAngleY;
		*/

		my_roll = compAngleX;
		my_pitch = compAngleY;

		timer = tmp_timer;
	}
}


