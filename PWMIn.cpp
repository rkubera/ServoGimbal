/*
 * PWMIn.cpp
 *
 *  Created on: 4 kwi 2017
 *      Author: radoslawkubera
 */
#include "GlobalDefines.h"
#include "Arduino.h"
#include "PWMIn.h"
#include "libraries/QuickStats/QuickStats.h"

static QuickStats quickStats;

static volatile uint16_t pin_2_pwm_value = 0;
static volatile uint32_t pin_2_prev_time = 0;

static volatile uint16_t pin_3_pwm_value = 0;
static volatile uint32_t pin_3_prev_time = 0;

void pwmin_rising_2() {
	attachInterrupt(0, pwmin_falling_2, FALLING);
	pin_2_prev_time = micros();
}

void pwmin_falling_2() {
	attachInterrupt(0, pwmin_rising_2, RISING);
	pin_2_pwm_value = micros()-pin_2_prev_time;
}

void pwmin_rising_3() {
	attachInterrupt(1, pwmin_falling_3, FALLING);
	pin_3_prev_time = micros();
}

void pwmin_falling_3() {
	attachInterrupt(1, pwmin_rising_3, RISING);
	pin_3_pwm_value = micros()-pin_3_prev_time;
}

void InitPWNIn() {
	attachInterrupt(0, pwmin_rising_2, FALLING);
	attachInterrupt(1, pwmin_rising_3, FALLING);
}

void getPWMInValues (uint16_t &pin2, uint16_t &pin3) {

	static uint32_t last_timer;
	static float readings_pin2[QSTATS_BUFFER];
	static float readings_pin3[QSTATS_BUFFER];

	if (micros()-pin_2_prev_time>30000) {
		pin_2_pwm_value = 0;
	}
	else {
		if (pin_2_pwm_value<1000) pin_2_pwm_value = 1000;
		if (pin_2_pwm_value>2000) pin_2_pwm_value = 2000;
	}

	if (micros()-pin_3_prev_time>30000) {
		pin_3_pwm_value = 0;
	}
	else {
		if (pin_3_pwm_value<1000) pin_3_pwm_value = 1000;
		if (pin_3_pwm_value>2000) pin_3_pwm_value = 2000;
	}

	//Qstats: rotate buffers
	if (micros()-last_timer>=1000) {
		for (uint8_t i = 0; i<QSTATS_BUFFER-1; i++) {
			readings_pin2[i] = readings_pin2[i+1];
			readings_pin3[i] = readings_pin3[i+1];
		}
		readings_pin2[QSTATS_BUFFER-1] = pin_2_pwm_value;
		readings_pin3[QSTATS_BUFFER-1] = pin_3_pwm_value;
		last_timer = micros();
	}

	//Qstats: get median to remove noises on pin2 and pin3
	pin2 = (uint16_t) quickStats.median(readings_pin2, QSTATS_BUFFER);
	pin3 = (uint16_t) quickStats.median(readings_pin3, QSTATS_BUFFER);
}
