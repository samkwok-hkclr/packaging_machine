/*
 * motor_type.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef MOTOR_H_
#define MOTOR_H_

typedef enum {
	PILL_GATE = 0, PKG_DIS, PKG_LEN, ROLLER,
} motor_t;

typedef enum {
	PILL_GATE_STEPPER = 0, PKG_DIS_STEPPER,
} stepper_motor_t;

typedef enum {
	PKG_LEN_DC = 0, ROLLER_DC,
} dc_motor_t;

typedef enum {
	SQUEEZER = 0,
} zd_motor_t;

#endif /* MOTOR_H_ */
