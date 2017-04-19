/*
 * Gyro.h
 *
 *  Created on: 29 mar 2017
 *      Author: radoslawkubera
 */

#ifndef GYRO_H_
#define GYRO_H_

void getRollPitch1 (float &my_roll, float &my_pitch, uint8_t get_median);
void getRollPitch (float &my_roll, float &my_pitch);
void initGyro();

#endif /* GYRO_H_ */
