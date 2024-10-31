/*
 * squeezer_motor.c
 *
 *  Created on: Oct 15, 2024
 *      Author: mskwok
 */

#include "squeezer_motor.h"

inline uint16_t get_od_sq_speed(OD_t *od, uint8_t sub_index)
{
	return _get_od_u16(od, sub_index, 0x6070);
}

inline uint8_t get_od_sq_wait_time(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6071);
}

inline uint8_t get_od_sq_dir(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6072);
}

inline uint8_t get_od_sq_mode(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6073);
}

inline uint8_t get_od_sq_status(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6078);
}

inline uint8_t get_od_sq_ctrl(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6079);
}

// ==============================================================================

inline void set_od_sq_speed(OD_t *od, uint8_t sub_index, uint16_t val)
{
	_set_od_u16(od, sub_index, 0x6070, val);
}

inline void set_od_sq_wait_time(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6071, val);
}

inline void set_od_sq_dir(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6072, val);
}

inline void set_od_sq_mode(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6073, val);
}

inline void set_od_sq_status(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6078, val);
}

inline void set_od_sq_ctrl(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6079, val);
}

