/*
 * motor_state.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef MOTOR_STATE_H_
#define MOTOR_STATE_H_

typedef enum
{
	M_IDLE = 0,
	M_RUNNING,
	M_RESET,
	M_BRAKE,
	M_STOP,
	M_ERROR
} motor_state_t;

#endif /* MOTOR_STATE_H_ */
