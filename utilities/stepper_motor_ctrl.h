/*
 * motor_ctrl.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef STEPPER_MOTOR_CTRL_H_
#define STEPPER_MOTOR_CTRL_H_

#include "motor.h"
#include "motor_state.h"
#include "pill_gate_motor.h"
#include "pkg_dis_motor.h"

void stepper_motor_controller(const stepper_motor_t type, OD_t *od, motor_state_t *state, uint8_t *dir_ctrl, bool_t *start_flag);

uint16_t get_od_stepper_rotate_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_stepper_rotate_dir(const stepper_motor_t type, OD_t *od, uint8_t sub_index);
uint16_t get_od_stepper_curr_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_stepper_status(const stepper_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_stepper_ctrl(const stepper_motor_t type, OD_t *od, uint8_t sub_index);

void set_od_stepper_rotate_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_stepper_rotate_dir(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_stepper_curr_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_stepper_status(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_stepper_ctrl(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* STEPPER_MOTOR_CTRL_H_ */
