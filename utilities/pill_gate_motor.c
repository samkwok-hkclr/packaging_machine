/*
 * pill_gate_motor.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include "pill_gate_motor.h"

inline uint16_t get_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index)
{
	return _get_od_u16(od, sub_index, 0x6021);
}

inline uint8_t get_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6022);
}

inline uint16_t get_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index)
{
	return _get_od_u16(od, sub_index, 0x6024);
}

inline uint8_t get_od_pill_gate_mode(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6027);
}

inline uint8_t get_od_pill_gate_status(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6028);
}

inline uint8_t get_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index)
{
	return _get_od_u8(od, sub_index, 0x6029);
}

// ==============================================================================

inline void set_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index, uint16_t val)
{
	_set_od_u16(od, sub_index, 0x6021, val);
}

inline void set_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6022, val);
}

inline void set_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index, uint16_t val)
{
	_set_od_u16(od, sub_index, 0x6024, val);
}

inline void set_od_pill_gate_mode(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6027, val);
}

inline void set_od_pill_gate_status(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6028, val);
}

inline void set_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index, uint8_t val)
{
	_set_od_u8(od, sub_index, 0x6029, val);
}
