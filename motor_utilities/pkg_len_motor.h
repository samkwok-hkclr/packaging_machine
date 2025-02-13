/*
 * pkg_len_motor.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef PKG_LEN_MOTOR_H_
#define PKG_LEN_MOTOR_H_

#include "od_utilities.h"

typedef enum {
	PKG_LEN_PT_0 = 0, PKG_LEN_PT_1, PKG_LEN_PT_UNKNOWN
} pkg_len_mode_t;

uint8_t get_od_pkg_len_rotate_steps(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_rotate_dir(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_curr_step(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_target_braker(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_curr_braker(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_loc(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_mode(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_state(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_len_ctrl(OD_t *od, uint8_t sub_index);

void set_od_pkg_len_rotate_steps(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_curr_step(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_target_braker(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_curr_braker(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_loc(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_mode(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_state(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_len_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* PKG_LEN_MOTOR_H_ */
