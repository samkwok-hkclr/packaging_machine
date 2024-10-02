/*
 * od_utilities.h
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#ifndef OD_UTILITIES_H_
#define OD_UTILITIES_H_

#include <stdint.h>

#include "CO_app_STM32.h"

uint8_t _get_od_u8(OD_t *od, uint8_t sub_index, uint16_t index);
uint16_t _get_od_u16(OD_t *od, uint8_t sub_index, uint16_t index);

void _set_od_u8(OD_t *od, uint8_t sub_index, uint16_t index, uint8_t val);
void _set_od_u16(OD_t *od, uint8_t sub_index, uint16_t index, uint16_t val);


#endif /* OD_UTILITIES_H_ */
