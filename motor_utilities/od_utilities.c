/*
 * od_utilities.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include "od_utilities.h"

inline uint8_t _get_od_u8(OD_t *od, uint8_t sub_index, uint16_t index) {
	uint8_t val;

	if (OD_get_u8(OD_find(od, index), 0x00, &val, false) != ODR_OK)
		show_err_LED();

	return val;
}

inline uint16_t _get_od_u16(OD_t *od, uint8_t sub_index, uint16_t index) {
	uint16_t val;

	if (OD_get_u16(OD_find(od, index), 0x00, &val, false) != ODR_OK)
		show_err_LED();

	return val;
}

inline void _set_od_u8(OD_t *od, uint8_t sub_index, uint16_t index, uint8_t val) {
	if (OD_set_u8(OD_find(od, index), 0x00, val, false) != ODR_OK)
		show_err_LED();
}

inline void _set_od_u16(OD_t *od, uint8_t sub_index, uint16_t index, uint16_t val) {
	if (OD_set_u16(OD_find(od, index), 0x00, val, false) != ODR_OK)
		show_err_LED();
}
