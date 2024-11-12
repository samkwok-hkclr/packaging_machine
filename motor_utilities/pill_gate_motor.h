/*
 * pill_gate_motor.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef PILL_GATE_MOTOR_H_
#define PILL_GATE_MOTOR_H_

#include "od_utilities.h"

typedef enum {
	NORMAL = 0, PILL_GATE_HOMING,
} pill_gate_mode_t;

typedef enum {
	PILL_GATE_PT_0 = 0, PILL_GATE_PT_1, PILL_GATE_PT_2, PILL_GATE_PT_3, PILL_GATE_PT_4, PILL_GATE_UNKOWN
} pill_gate_loc_t;

GPIO_PinState get_open_dir(bool_t open_door);
uint16_t cvt_dis_to_pulses(float distance);

uint16_t get_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index);
uint16_t get_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_target_loc(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_loc(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_mode(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_state(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index);

void set_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_pill_gate_target_loc(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_loc(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_mode(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_state(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* PILL_GATE_MOTOR_H_ */
