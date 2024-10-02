/*
 * dc_motor_ctrl.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef DC_MOTOR_CTRL_H_
#define DC_MOTOR_CTRL_H_

#include "motor.h"
#include "motor_state.h"

#include "pkg_len_motor.h"
#include "roller_motor.h"

void dc_motor_controller(const dc_motor_t type, OD_t *od, motor_state_t *state, uint8_t *dir_ctrl, bool_t *start_flag);

uint8_t get_od_dc_rotate_steps(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_rotate_dir(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_curr_step(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_target_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_curr_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_mode(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_status(const dc_motor_t type, OD_t *od, uint8_t sub_index);
uint8_t get_od_dc_ctrl(const dc_motor_t type, OD_t *od, uint8_t sub_index);

void set_od_dc_rotate_steps(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_rotate_dir(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_curr_step(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_target_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_curr_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_mode(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_status(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_dc_ctrl(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* DC_MOTOR_CTRL_H_ */
