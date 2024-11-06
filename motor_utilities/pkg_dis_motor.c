/*
 * pkg_len_motor.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include <pkg_dis_motor.h>

inline uint16_t get_od_pkg_dis_rotate_pulses(OD_t *od, uint8_t sub_index) {
	return _get_od_u16(od, sub_index, 0x6011);
}

inline uint8_t get_od_pkg_dis_rotate_dir(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6012);
}

inline uint16_t get_od_pkg_dis_curr_pulses(OD_t *od, uint8_t sub_index) {
	return _get_od_u16(od, sub_index, 0x6014);
}

inline uint8_t get_od_pkg_dis_state(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6018);
}

inline uint8_t get_od_pkg_dis_ctrl(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6019);
}

// ==============================================================================

inline void set_od_pkg_dis_rotate_pulses(OD_t *od, uint8_t sub_index, uint16_t val) {
	_set_od_u16(od, sub_index, 0x6011, val);
}

inline void set_od_pkg_dis_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6012, val);
}

inline void set_od_pkg_dis_curr_pulses(OD_t *od, uint8_t sub_index, uint16_t val) {
	_set_od_u16(od, sub_index, 0x6014, val);
}

inline void set_od_pkg_dis_state(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6018, val);
}

inline void set_od_pkg_dis_ctrl(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6019, val);
}
