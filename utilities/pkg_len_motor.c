/*
 * pkg_len_motor.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include "pkg_len_motor.h"

inline uint8_t get_od_pkg_len_rotate_steps(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6040);
}

inline uint8_t get_od_pkg_len_rotate_dir(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6042);
}

inline uint8_t get_od_pkg_len_curr_step(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6043);
}

inline uint8_t get_od_pkg_len_target_braker(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6044);
}

inline uint8_t get_od_pkg_len_curr_braker(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6045);
}

inline uint8_t get_od_pkg_len_mode(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6047);
}

inline uint8_t get_od_pkg_len_status(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6048);
}

inline uint8_t get_od_pkg_len_ctrl(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6049);
}

// ==============================================================================

inline void set_od_pkg_len_rotate_steps(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6040, val);
}

inline void set_od_pkg_len_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6042, val);
}

inline void set_od_pkg_len_curr_step(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6043, val);
}

inline void set_od_pkg_len_target_braker(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6044, val);
}

inline void set_od_pkg_len_curr_braker(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6045, val);
}

inline void set_od_pkg_len_mode(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6047, val);
}

inline void set_od_pkg_len_status(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6048, val);
}

inline void set_od_pkg_len_ctrl(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6049, val);
}
