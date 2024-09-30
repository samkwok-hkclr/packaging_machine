/*
 * pill_gate_func.c
 *
 *  Created on: Sep 30, 2024
 *      Author: mskwok
 */

#include "pill_gate_func.h"

inline uint16_t get_od_pill_gate_rotate_pulses()
{
	return _get_od_u16(0x6021);
}

inline uint8_t get_od_pill_gate_rotate_dir()
{
	return _get_od_u8(0x6022);
}

inline uint16_t get_od_pill_gate_curr_pulses()
{
	return _get_od_u16(0x6024);
}

inline uint8_t get_od_pill_gate_status()
{
	return _get_od_u8(0x6028);
}

inline uint8_t get_od_pill_gate_ctrl()
{
	return _get_od_u8(0x6029);
}

inline void set_od_pill_gate_rotate_dir(uint8_t val)
{
	_set_od_u8(0x6022, val);
}

inline void set_od_pill_gate_curr_pulses(uint16_t val)
{
	_set_od_u16(0x6024, val);
}

inline void set_od_pill_gate_status(uint8_t val)
{
	_set_od_u8(0x6028, val);
}

inline void set_od_pill_gate_ctrl(uint8_t val)
{
	_set_od_u8(0x6029, val);
}
