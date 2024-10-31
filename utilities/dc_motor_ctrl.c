/*
 * dc_motor_ctrl.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include "dc_motor_ctrl.h"

uint8_t brake_t = 0;

void dc_motor_controller(const dc_motor_t type, OD_t *od, motor_state_t *state, uint8_t *dir_ctrl, bool_t *start_flag) {
	*state = get_od_dc_state(type, od, 0);

	switch (*state) {
	case M_IDLE: {
		uint8_t ctrl, steps;

		ctrl = get_od_dc_ctrl(type, od, 0);
		steps = get_od_dc_rotate_steps(type, od, 0);

		*start_flag = ctrl > 0 && steps > 0;
		if (*start_flag) {
			set_od_dc_state(type, od, 0, M_RUNNING);
			*dir_ctrl = get_od_dc_rotate_dir(type, od, 0);
		}
		break;
	}
	case M_RUNNING: {
		uint8_t ctrl, curr_step, rotate_steps;

		ctrl = get_od_dc_ctrl(type, od, 0);
		curr_step = get_od_dc_curr_step(type, od, 0);
		rotate_steps = get_od_dc_rotate_steps(type, od, 0);

		if (curr_step >= rotate_steps)
			set_od_dc_state(type, od, 0, M_BRAKE);
		if (ctrl == 0)
			set_od_dc_state(type, od, 0, M_BRAKE);
		break;
	}
	case M_BRAKE: {
		uint8_t target_braker, curr_braker;

		target_braker = get_od_dc_target_braker(type, od, 0);
		curr_braker = get_od_dc_curr_braker(type, od, 0);

		if (curr_braker > target_braker) {
			set_od_dc_state(type, od, 0, M_STOP);
			set_od_dc_curr_braker(type, od, 0, 0);
		} else
			set_od_dc_curr_braker(type, od, 0, ++curr_braker);
		break;
	}
	case M_STOP: {
		set_od_dc_state(type, od, 0, M_RESET);
		break;
	}
	case M_RESET: {
		set_od_dc_rotate_steps(type, od, 0, 0);
		set_od_dc_rotate_dir(type, od, 0, 0);
		set_od_dc_curr_step(type, od, 0, 0);
		set_od_dc_ctrl(type, od, 0, 0);
		set_od_dc_state(type, od, 0, M_IDLE);
		break;
	}
	case M_ERROR:
		set_od_dc_state(type, od, 0, M_BRAKE);
		show_err_LED();
		break;
	}
}

// ==============================================================================

uint8_t get_od_dc_rotate_steps(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_rotate_steps(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_rotate_steps(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_rotate_dir(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_rotate_dir(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_rotate_dir(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_curr_step(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_curr_step(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_curr_step(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_target_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_target_braker(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_target_braker(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_curr_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_curr_braker(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_curr_braker(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_mode(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_mode(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_mode(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_state(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_state(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_state(od, sub_index);
		break;
	}

	return 0;
}

uint8_t get_od_dc_ctrl(const dc_motor_t type, OD_t *od, uint8_t sub_index) {
	switch (type) {
	case PKG_LEN_DC:
		return get_od_pkg_len_ctrl(od, sub_index);
		break;
	case ROLLER_DC:
		return get_od_roller_ctrl(od, sub_index);
		break;
	}

	return 0;
}

// ==============================================================================

void set_od_dc_rotate_steps(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_rotate_steps(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_rotate_steps(od, sub_index, val);
		break;
	}
}

void set_od_dc_rotate_dir(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_rotate_dir(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_rotate_dir(od, sub_index, val);
		break;
	}
}

void set_od_dc_curr_step(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_curr_step(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_curr_step(od, sub_index, val);
		break;
	}
}

void set_od_dc_target_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_target_braker(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_target_braker(od, sub_index, val);
		break;
	}
}

void set_od_dc_curr_braker(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_curr_braker(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_curr_braker(od, sub_index, val);
		break;
	}
}

void set_od_dc_mode(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_mode(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_mode(od, sub_index, val);
		break;
	}
}

void set_od_dc_state(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_state(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_state(od, sub_index, val);
		break;
	}
}

void set_od_dc_ctrl(const dc_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val) {
	switch (type) {
	case PKG_LEN_DC:
		set_od_pkg_len_ctrl(od, sub_index, val);
		break;
	case ROLLER_DC:
		set_od_roller_ctrl(od, sub_index, val);
		break;
	}
}
