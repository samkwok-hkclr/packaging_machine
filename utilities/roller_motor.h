/*
 * roller_motor.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef ROLLER_MOTOR_H_
#define ROLLER_MOTOR_H_

#include "od_utilities.h"

typedef enum {
	TRAY = 0, ROLLER_HOMING,
} roller_mode_t;

uint8_t get_od_roller_rotate_steps(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_rotate_dir(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_curr_step(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_target_braker(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_curr_braker(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_mode(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_status(OD_t *od, uint8_t sub_index);
uint8_t get_od_roller_ctrl(OD_t *od, uint8_t sub_index);

void set_od_roller_rotate_steps(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_curr_step(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_target_braker(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_curr_braker(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_mode(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_status(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_roller_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* ROLLER_MOTOR_H_ */
