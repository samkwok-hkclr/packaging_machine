/*
 * pill_gate_func.h
 *
 *  Created on: Sep 30, 2024
 *      Author: mskwok
 */

#ifndef INC_PILL_GATE_FUNC_H_
#define INC_PILL_GATE_FUNC_H_

#include "od_utilities.h"

uint16_t get_od_pill_gate_rotate_pulses();
uint8_t get_od_pill_gate_rotate_dir();
uint16_t get_od_pill_gate_curr_pulses();
uint8_t get_od_pill_gate_status();
uint8_t get_od_pill_gate_ctrl();

void set_od_pill_gate_rotate_dir(uint8_t val);
void set_od_pill_gate_curr_pulses(uint16_t val);
void set_od_pill_gate_status(uint8_t val);
void set_od_pill_gate_ctrl(uint8_t val);

#endif /* INC_PILL_GATE_FUNC_H_ */
