/*
 * conveyor_motor.c
 *
 *  Created on: Oct 31, 2024
 *      Author: mskwok
 */

#include "conveyor_motor.h"

inline uint16_t get_od_con_speed(OD_t *od, uint8_t sub_index)
{
	return _get_od_u16(od, sub_index, 0x6080);
}

inline uint8_t get_od_con_stop_by_ph(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6081);
}

inline uint8_t get_od_con_dir(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6082);
}

inline uint8_t get_od_con_mode(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6083);
}

inline uint8_t get_od_con_state(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6088);
}

inline uint8_t get_od_con_ctrl(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6089);
}

// ==============================================================================

inline void set_od_con_speed(OD_t *od, uint8_t sub_index, uint16_t val)
{
	_set_od_u16(od, sub_index, 0x6080, val);
}

inline void set_od_con_stop_by_ph(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6081, val);
}

inline void set_od_con_dir(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6082, val);
}

inline void set_od_con_mode(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6083, val);
}

inline void set_od_con_state(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6088, val);
}

inline void set_od_con_ctrl(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6089, val);
}

