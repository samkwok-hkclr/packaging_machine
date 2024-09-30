/*
 * pkg_len_func.c
 *
 *  Created on: Sep 30, 2024
 *      Author: mskwok
 */

#include "pkg_len_func.h"

inline uint16_t get_od_pkg_dis_rotate_pulses()
{
	return _get_od_u16(0x6011);
}

inline uint8_t get_od_pkg_dis_rotate_dir()
{
	return _get_od_u8(0x6012);
}

inline uint16_t get_od_pkg_dis_curr_pulses()
{
	return _get_od_u16(0x6014);
}

inline uint8_t get_od_pkg_dis_status()
{
	return _get_od_u8(0x6018);
}

inline uint8_t get_od_pkg_dis_ctrl()
{
	return _get_od_u8(0x6019);
}

inline void set_od_pkg_dis_rotate_dir(uint8_t val)
{
	_set_od_u8(0x6012, val);
}

inline void set_od_pkg_dis_curr_pulses(uint16_t val)
{
	_set_od_u16(0x6014, val);
}

inline void set_od_pkg_dis_status(uint8_t val)
{
	_set_od_u8(0x6018, val);
}

inline void set_od_pkg_dis_ctrl(uint8_t val)
{
	_set_od_u8(0x6019, val);
}
