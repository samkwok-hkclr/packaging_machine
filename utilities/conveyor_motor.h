/*
 * conveyor_motor.h
 *
 *  Created on: Oct 31, 2024
 *      Author: mskwok
 */

#ifndef CONVEYOR_MOTOR_H_
#define CONVEYOR_MOTOR_H_

#include "od_utilities.h"

typedef enum {
	NONE = 0,
} conveyor_mode_t;

uint16_t get_od_con_speed(OD_t *od, uint8_t sub_index);
uint8_t get_od_con_stop_by_ph(OD_t *od, uint8_t sub_index);
uint8_t get_od_con_dir(OD_t *od, uint8_t sub_index);
uint8_t get_od_con_mode(OD_t *od, uint8_t sub_index);
uint8_t get_od_con_state(OD_t *od, uint8_t sub_index);
uint8_t get_od_con_ctrl(OD_t *od, uint8_t sub_index);

void set_od_con_speed(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_con_stop_by_ph(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_con_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_con_mode(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_con_state(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_con_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);


#endif /* CONVEYOR_MOTOR_H_ */
