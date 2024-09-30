/*
 * pkg_len_func.h
 *
 *  Created on: Sep 30, 2024
 *      Author: mskwok
 */

#ifndef INC_PKG_LEN_FUNC_H_
#define INC_PKG_LEN_FUNC_H_

#include "od_utilities.h"

uint16_t get_od_pkg_dis_rotate_pulses();
uint8_t get_od_pkg_dis_rotate_dir();
uint16_t get_od_pkg_dis_curr_pulses();
uint8_t get_od_pkg_dis_status();
uint8_t get_od_pkg_dis_ctrl();

void set_od_pkg_dis_rotate_dir(uint8_t val);
void set_od_pkg_dis_curr_pulses(uint16_t val);
void set_od_pkg_dis_status(uint8_t val);
void set_od_pkg_dis_ctrl(uint8_t val);

#endif /* INC_PKG_LEN_FUNC_H_ */
