/*
 * pkg_len_motor.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef PKG_DIS_MOTOR_H_
#define PKG_DIS_MOTOR_H_

#include "od_utilities.h"

uint16_t get_od_pkg_dis_rotate_pulses(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_dis_rotate_dir(OD_t *od, uint8_t sub_index);
uint16_t get_od_pkg_dis_curr_pulses(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_dis_status(OD_t *od, uint8_t sub_index);
uint8_t get_od_pkg_dis_ctrl(OD_t *od, uint8_t sub_index);

void set_od_pkg_dis_rotate_pulses(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_pkg_dis_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_dis_curr_pulses(OD_t *od, uint8_t sub_index, uint16_t val);
void set_od_pkg_dis_status(OD_t *od, uint8_t sub_index, uint8_t val);
void set_od_pkg_dis_ctrl(OD_t *od, uint8_t sub_index, uint8_t val);

#endif /* PKG_DIS_MOTOR_H_ */
