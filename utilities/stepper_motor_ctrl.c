/*
 * stepper_motor_ctrl.c
 *
 *  Created on: Oct 2, 2024
 *      Author: mskwok
 */

#include <stepper_motor_ctrl.h>

void stepper_motor_controller(const stepper_motor_t type, OD_t *od, motor_state_t *state, uint8_t *dir_ctrl, bool_t *start_flag)
{
	*state = get_od_stepper_status(type, od, 0);
	switch (*state)
	{
	case M_IDLE:
	{
		uint8_t ctrl = get_od_stepper_ctrl(type, od, 0);
		uint16_t rotate_pulses = get_od_stepper_rotate_pulses(type, od, 0);

		*start_flag = ctrl > 0 && rotate_pulses > 0;
		if (*start_flag)
		{
			set_od_stepper_status(type, od, 0, M_RUNNING);
			*dir_ctrl = get_od_stepper_rotate_dir(type, od, 0);
		}
		break;
	}
	case M_RUNNING:
	{
		uint16_t curr_pulses = get_od_stepper_curr_pulses(type, od, 0);
		set_od_stepper_curr_pulses(type, od, 0, ++curr_pulses);

		uint16_t rotate_pulses = get_od_stepper_rotate_pulses(type, od, 0);

		if (curr_pulses >= rotate_pulses)
			set_od_stepper_status(type, od, 0, M_STOP);

		break;
	}
	case M_BRAKE:
	{
		// stepper motor do not have brake function
		break;
	}
	case M_STOP:
	{
		set_od_stepper_status(type, od, 0, M_RESET);
		break;
	}
	case M_RESET:
	{
		set_od_stepper_rotate_pulses(type, od, 0, 0);
		set_od_stepper_curr_pulses(type, od, 0, 0);
		set_od_stepper_rotate_dir(type, od, 0, 0);
		set_od_stepper_ctrl(type, od, 0, 0);
		set_od_stepper_status(type, od, 0, M_IDLE);
		break;
	}
	case M_ERROR:
		show_err_LED();
		break;
	}
}

// ==============================================================================

inline uint16_t get_od_stepper_rotate_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		return get_od_pill_gate_rotate_pulses(od, sub_index);
		break;
	case PKG_DIS_STEPPER:
		return get_od_pkg_dis_rotate_pulses(od, sub_index);
		break;
	}

	return 0;
}

inline uint8_t get_od_stepper_rotate_dir(const stepper_motor_t type, OD_t *od, uint8_t sub_index)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		return get_od_pill_gate_rotate_dir(od, sub_index);
		break;
	case PKG_DIS_STEPPER:
		return get_od_pkg_dis_rotate_dir(od, sub_index);
		break;
	}

	return 0;
}

inline uint16_t get_od_stepper_curr_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		return get_od_pill_gate_curr_pulses(od, sub_index);
		break;
	case PKG_DIS_STEPPER:
		return get_od_pkg_dis_curr_pulses(od, sub_index);
		break;
	}

	return 0;
}

inline uint8_t get_od_stepper_status(const stepper_motor_t type, OD_t *od, uint8_t sub_index)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		return get_od_pill_gate_status(od, sub_index);
		break;
	case PKG_DIS_STEPPER:
		return get_od_pkg_dis_status(od, sub_index);
		break;
	}

	return 0;
}

inline uint8_t get_od_stepper_ctrl(const stepper_motor_t type, OD_t *od, uint8_t sub_index)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		return get_od_pill_gate_ctrl(od, sub_index);
		break;
	case PKG_DIS_STEPPER:
		return get_od_pkg_dis_ctrl(od, sub_index);
		break;
	}

	return 0;
}

// ==============================================================================

inline void set_od_stepper_rotate_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint16_t val)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		set_od_pill_gate_rotate_pulses(od, sub_index, val);
		break;
	case PKG_DIS_STEPPER:
		set_od_pkg_dis_rotate_pulses(od, sub_index, val);
		break;
	}
}

inline void set_od_stepper_rotate_dir(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		set_od_pill_gate_rotate_dir(od, sub_index, val);
		break;
	case PKG_DIS_STEPPER:
		set_od_pkg_dis_rotate_dir(od, sub_index, val);
		break;
	}
}

inline void set_od_stepper_curr_pulses(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint16_t val)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		set_od_pill_gate_curr_pulses(od, sub_index, val);
		break;
	case PKG_DIS_STEPPER:
		set_od_pkg_dis_curr_pulses(od, sub_index, val);
		break;
	}
}

inline void set_od_stepper_status(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		set_od_pill_gate_status(od, sub_index, val);
		break;
	case PKG_DIS_STEPPER:
		set_od_pkg_dis_status(od, sub_index, val);
		break;
	}
}

inline void set_od_stepper_ctrl(const stepper_motor_t type, OD_t *od, uint8_t sub_index, uint8_t val)
{
	switch (type)
	{
	case PILL_GATE_STEPPER:
		set_od_pill_gate_ctrl(od, sub_index, val);
		break;
	case PKG_DIS_STEPPER:
		set_od_pkg_dis_ctrl(od, sub_index, val);
		break;
	}
}


