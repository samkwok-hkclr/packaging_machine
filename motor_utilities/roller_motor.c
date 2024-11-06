/*
 * roller_motor.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include "roller_motor.h"

inline uint8_t get_od_roller_rotate_steps(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6030);
}

inline uint8_t get_od_roller_rotate_dir(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6032);
}

inline uint8_t get_od_roller_curr_step(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6033);
}

inline uint8_t get_od_roller_target_braker(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6034);
}

inline uint8_t get_od_roller_curr_braker(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6035);
}

inline uint8_t get_od_roller_mode(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6037);
}

inline uint8_t get_od_roller_state(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6038);
}

inline uint8_t get_od_roller_ctrl(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6039);
}

// ==============================================================================

inline void set_od_roller_rotate_steps(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6030, val);
}

inline void set_od_roller_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6032, val);
}

inline void set_od_roller_curr_step(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6033, val);
}

inline void set_od_roller_target_braker(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6034, val);
}

inline void set_od_roller_curr_braker(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6035, val);
}

inline void set_od_roller_mode(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6037, val);
}

inline void set_od_roller_state(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6038, val);
}

inline void set_od_roller_ctrl(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6039, val);
}
