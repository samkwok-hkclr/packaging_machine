/*
 * pill_gate_motor.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include <math.h>

#include "pill_gate_motor.h"

#define PILL_GATE_RADIUS 10.0f
#define PILL_GATE_PULSES_PER_REV 3200.0f
#define GATE_DOOR_WIDTH 44.0f

// open_door == 1, open direction
// open_door == 0, close direction
inline GPIO_PinState get_open_dir(bool_t open_door) {
	return open_door == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

inline uint16_t cvt_dis_to_pulses(float distance) {
	float f_pulses = distance / (2.0f * M_PI * PILL_GATE_RADIUS) * PILL_GATE_PULSES_PER_REV;

	return f_pulses > 0.0f ? floor(f_pulses) : 0;
}

// ==============================================================================

inline uint16_t get_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index) {
	return _get_od_u16(od, sub_index, 0x6021);
}

inline uint8_t get_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6022);
}

inline uint16_t get_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index) {
	return _get_od_u16(od, sub_index, 0x6024);
}

inline uint8_t get_od_pill_gate_target_loc(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6025);
}

inline uint8_t get_od_pill_gate_loc(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6026);
}

inline uint8_t get_od_pill_gate_mode(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6027);
}

inline uint8_t get_od_pill_gate_state(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6028);
}

inline uint8_t get_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index) {
	return _get_od_u8(od, sub_index, 0x6029);
}

// ==============================================================================

inline void set_od_pill_gate_rotate_pulses(OD_t *od, uint8_t sub_index, uint16_t val) {
	_set_od_u16(od, sub_index, 0x6021, val);
}

inline void set_od_pill_gate_rotate_dir(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6022, val);
}

inline void set_od_pill_gate_curr_pulses(OD_t *od, uint8_t sub_index, uint16_t val) {
	_set_od_u16(od, sub_index, 0x6024, val);
}

inline void set_od_pill_gate_target_loc(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6025, val);
}

inline void set_od_pill_gate_loc(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6026, val);
}

inline void set_od_pill_gate_mode(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6027, val);
}

inline void set_od_pill_gate_state(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6028, val);
}

inline void set_od_pill_gate_ctrl(OD_t *od, uint8_t sub_index, uint8_t val) {
	_set_od_u8(od, sub_index, 0x6029, val);
}
