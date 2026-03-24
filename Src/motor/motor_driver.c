/**
 * @file motor_driver.c
 * @brief Bridge layer between motor commutation logic and PWM hardware driver.
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

/**
 * @brief Apply one duty value in permyriad units to one PWM channel.
 *
 * @param pwm_h Pointer to PWM handle.
 * @param ch PWM channel number (1..3).
 * @param duty_permyriad Duty in permyriad units (0..10000).
 * @return true if duty update succeeded, false otherwise.
 */
static bool motor_driver_set_pwm_permyriad(pwm_tim1_handle_t *pwm_h,
										   uint8_t ch,
										   uint16_t duty_permyriad)
{
	if (pwm_h == NULL) return false;
	if ((ch < 1u) || (ch > 3u)) return false;

	if (duty_permyriad > 10000u)
	{
		duty_permyriad = 10000u;
	}

	uint16_t duty_ticks = (uint16_t)(((uint32_t)pwm_h->arr * (uint32_t)duty_permyriad) / 10000u);
	return pwm_tim1_set_duty(pwm_h, ch, duty_ticks);
}

/**
 * @brief Apply the stored driver command map to TIM1 PWM channels.
 *
 * Current hardware mapping:
 * - HIGH  -> configured active PWM duty
 * - LOW   -> 0% duty
 * - FLOAT -> 0% duty
 *
 * @note The current 3-PWM bench setup cannot generate true per-phase Hi-Z.
 *
 * @param motor_driver_h Pointer to motor driver handle.
 * @return true if all PWM updates succeeded, false otherwise.
 */
static bool motor_driver_apply_pwm_map(motor_driver_handle_t *motor_driver_h)
{
	if ((motor_driver_h == NULL) || (motor_driver_h->cfg == NULL)) return false;
	if (motor_driver_h->cfg->pwm_h == NULL) return false;

	uint16_t duty_a = 0u;
	uint16_t duty_b = 0u;
	uint16_t duty_c = 0u;

	/* Convert command map to 3-PWM duty values. */
	if (motor_driver_h->last_cmd_map.phase_a == MOTOR_DRIVER_CMD_HIGH)
	{
		duty_a = motor_driver_h->cfg->pwm_duty_permyriad;
	}
	if (motor_driver_h->last_cmd_map.phase_b == MOTOR_DRIVER_CMD_HIGH)
	{
		duty_b = motor_driver_h->cfg->pwm_duty_permyriad;
	}
	if (motor_driver_h->last_cmd_map.phase_c == MOTOR_DRIVER_CMD_HIGH)
	{
		duty_c = motor_driver_h->cfg->pwm_duty_permyriad;
	}

	/* Update TIM1 compare registers for all three phases. */
	if (!motor_driver_set_pwm_permyriad(motor_driver_h->cfg->pwm_h, 1u, duty_a)) return false;
	if (!motor_driver_set_pwm_permyriad(motor_driver_h->cfg->pwm_h, 2u, duty_b)) return false;
	if (!motor_driver_set_pwm_permyriad(motor_driver_h->cfg->pwm_h, 3u, duty_c)) return false;

	return true;
}

bool motor_driver_init(motor_driver_handle_t *motor_driver_h,
					   const motor_driver_cfg_t *motor_driver_cfg)
{
	/* Validate input pointers and configuration. */
	if ((motor_driver_h == NULL) || (motor_driver_cfg == NULL)) return false;
	if (motor_driver_cfg->pwm_h == NULL) return false;
	if (motor_driver_cfg->pwm_duty_permyriad > 10000u) return false;

	/* Initialize runtime state. */
	motor_driver_h->cfg = motor_driver_cfg;
	motor_driver_h->last_cmd_map = motor_driver_cmd_map_default();
	motor_driver_h->is_initialized = true;
	motor_driver_h->is_started = false;

	/* Force safe initial output state. */
	return motor_driver_apply_pwm_map(motor_driver_h);
}

bool motor_driver_start(motor_driver_handle_t *motor_driver_h)
{
	if (motor_driver_h == NULL) return false;
	if ((motor_driver_h->cfg == NULL) || (motor_driver_h->cfg->pwm_h == NULL)) return false;
	if (motor_driver_h->is_initialized == false) return false;

	if (!pwm_tim1_start(motor_driver_h->cfg->pwm_h)) return false;

	motor_driver_h->is_started = true;
	return true;
}

bool motor_driver_stop(motor_driver_handle_t *motor_driver_h)
{
	if (motor_driver_h == NULL) return false;
	if ((motor_driver_h->cfg == NULL) || (motor_driver_h->cfg->pwm_h == NULL)) return false;
	if (motor_driver_h->is_initialized == false) return false;

	/* Clear PWM compare values before stopping the timer. */
	motor_driver_h->last_cmd_map = motor_driver_cmd_map_default();
	if (!motor_driver_apply_pwm_map(motor_driver_h)) return false;
	if (!pwm_tim1_stop(motor_driver_h->cfg->pwm_h)) return false;

	motor_driver_h->is_started = false;
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

	/* Apply command map to TIM1 compare registers. */
	return motor_driver_apply_pwm_map(motor_driver_h);
}

motor_driver_cmd_map_t motor_driver_get_cmd_map(const motor_driver_handle_t *motor_driver_h)
{
	if (motor_driver_h == NULL)
	{
		return motor_driver_cmd_map_default();
	}

	return motor_driver_h->last_cmd_map;
}
