/*
 * pill_gate_motor.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef PILL_GATE_MOTOR_H_
#define PILL_GATE_MOTOR_H_

#include "od_utilities.h"

uint16_t get_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index);
uint16_t get_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_status(OD_t *od, uint8_t sub_index);
uint8_t get_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index);

void set_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_pill_gate_status(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* PILL_GATE_MOTOR_H_ */
