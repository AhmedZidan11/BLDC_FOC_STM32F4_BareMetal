/**
 * @file motor_driver.c
 * @brief Bridge layer between motor commutation logic and hardware driver.
 *
 */

#include "motor/motor_driver.h"
#include <stddef.h>

static motor_driver_cmd_map_t motor_driver_cmd_map_default(void)
{
	motor_driver_cmd_map_t cmd_map = {
		.phase_a = MOTOR_DRIVER_CMD_FLOAT,
		.phase_b = MOTOR_DRIVER_CMD_FLOAT,
		.phase_c = MOTOR_DRIVER_CMD_FLOAT
	};

	return cmd_map;
}

/**
 * @brief Convert one logical phase state to one driver command.
 *
 * @param phase_state Logical commutation state of one motor phase.
 * @return Abstract drive command for the phase.
 */
static motor_driver_cmd_t motor_driver_convert_phase_state(motor_6step_phase_state_t phase_state)
{
	switch (phase_state)
	{
	case MOTOR_6STEP_PHASE_HIGH:
		return MOTOR_DRIVER_CMD_HIGH;

	case MOTOR_6STEP_PHASE_LOW:
		return MOTOR_DRIVER_CMD_LOW;

	case MOTOR_6STEP_PHASE_FLOAT:
	default:
		return MOTOR_DRIVER_CMD_FLOAT;
	}
}

bool motor_driver_init(motor_driver_handle_t *motor_driver_h,
					   const motor_driver_cfg_t *motor_driver_cfg)
{
	/* Validate input pointers. */
	if ((motor_driver_h == NULL) || (motor_driver_cfg == NULL)) return false;

	/* Initialize runtime state. */
	motor_driver_h->cfg = motor_driver_cfg;
	motor_driver_h->last_cmd_map = motor_driver_cmd_map_default();
	motor_driver_h->is_initialized = true;

	return true;
}

bool motor_driver_apply_phase_map(motor_driver_handle_t *motor_driver_h,
								  const motor_6step_phase_map_t *phase_map)
{
	/* Validate input pointers and driver state. */
	if ((motor_driver_h == NULL) || (phase_map == NULL)) return false;
	if ((motor_driver_h->cfg == NULL) || (motor_driver_h->is_initialized == false)) return false;

	/* Convert logical phase map to abstract driver command map. */
	motor_driver_h->last_cmd_map.phase_a = motor_driver_convert_phase_state(phase_map->phase_a);
	motor_driver_h->last_cmd_map.phase_b = motor_driver_convert_phase_state(phase_map->phase_b);
	motor_driver_h->last_cmd_map.phase_c = motor_driver_convert_phase_state(phase_map->phase_c);

	/* Hardware PWM phase driving will be added in the next stage. */
	return true;
}

motor_driver_cmd_map_t motor_driver_get_cmd_map(const motor_driver_handle_t *motor_driver_h)
{
	if (motor_driver_h == NULL)
	{
		return motor_driver_cmd_map_default();
	}

	return motor_driver_h->last_cmd_map;
}
