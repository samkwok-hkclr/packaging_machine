/*
 * squeezer_motor.h
 *
 *  Created on: Oct 15, 2024
 *      Author: mskwok
 */

#ifndef SQUEEZER_MOTOR_H_
#define SQUEEZER_MOTOR_H_

#include "od_utilities.h"

typedef enum {
	HOME = 0, SQUEEZE,
} squeezer_mode_t;

typedef enum {
	SQUEEZE_HOME_PT = 0, SQUEEZE_PT,
} squeezer_loc_t;

uint16_t get_od_sq_speed(OD_t *od, uint8_t sub_index);
uint8_t get_od_sq_wait_time(OD_t *od, uint8_t sub_index);
uint8_t get_od_sq_dir(OD_t *od, uint8_t sub_index);
uint8_t get_od_sq_mode(OD_t *od, uint8_t sub_index);
uint8_t get_od_sq_loc(OD_t *od, uint8_t sub_index);
uint8_t get_od_sq_state(OD_t *od, uint8_t sub_index);
uint8_t get_od_sq_ctrl(OD_t *od, uint8_t sub_index);

void set_od_sq_speed(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_sq_wait_time(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_sq_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_sq_mode(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_sq_loc(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_sq_state(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_sq_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* SQUEEZER_MOTOR_H_ */
