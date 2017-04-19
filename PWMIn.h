/*
 * PWMIn.h
 *
 *  Created on: 4 kwi 2017
 *      Author: radoslawkubera
 */

#ifndef PWMIN_H_
#define PWMIN_H_

void getPWMInValues (uint16_t &pin2, uint16_t &pin3);
void InitPWNIn();

void pwmin_rising_2();
void pwmin_falling_2();
void pwmin_rising_3();
void pwmin_falling_3();

#endif /* PWMIN_H_ */
