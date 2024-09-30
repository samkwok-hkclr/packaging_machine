/*
 * od_utilities.h
 *
 *  Created on: Sep 30, 2024
 *      Author: mskwok
 */

#ifndef INC_OD_UTILITIES_H_
#define INC_OD_UTILITIES_H_

#include <stdint.h>

#include "CO_app_STM32.h"
#include "OD.h"

uint8_t _get_od_u8(uint16_t index);
uint16_t _get_od_u16(uint16_t index);

void _set_od_u8(uint16_t index, uint8_t val);
void _set_od_u16(uint16_t index, uint16_t val);

#endif /* INC_OD_UTILITIES_H_ */
