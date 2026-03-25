/**
 * @file motor_3pwm.c
 * @brief Narrow 3-PWM power-stage abstraction for TIM1.
 *
 */

#include "motor/motor_3pwm.h"
#include <stddef.h>

/**
 * @brief Apply one duty value in permyriad units to one PWM channel.
 *
 * @param pwm_h Pointer to PWM handle.
 * @param ch PWM channel number (1..3).
 * @param duty_permyriad Duty in permyriad units (0..10000).
 * @return true if duty update succeeded, false otherwise.
 */
static bool motor_3pwm_set_pwm_permyriad(pwm_tim1_handle_t *pwm_h,
									 uint8_t ch,
									 uint16_t duty_permyriad)
{
	if (pwm_h == NULL) return false;
	if ((ch < 1u) || (ch > 3u)) return false;
	if (duty_permyriad > 10000u) return false;

	uint16_t duty_ticks = (uint16_t)(((uint32_t)pwm_h->arr * (uint32_t)duty_permyriad) / 10000u);
	return pwm_tim1_set_duty(pwm_h, ch, duty_ticks);
}

bool motor_3pwm_init(motor_3pwm_handle_t *motor_3pwm_h,
					 const motor_3pwm_cfg_t *motor_3pwm_cfg)
{
	if ((motor_3pwm_h == NULL) || (motor_3pwm_cfg == NULL)) return false;
	if (motor_3pwm_cfg->pwm_h == NULL) return false;

	motor_3pwm_h->cfg = motor_3pwm_cfg;
	motor_3pwm_h->phase_a_duty_permyriad = 0u;
	motor_3pwm_h->phase_b_duty_permyriad = 0u;
	motor_3pwm_h->phase_c_duty_permyriad = 0u;
	motor_3pwm_h->is_initialized = true;
	motor_3pwm_h->is_started = false;

	return true;
}

bool motor_3pwm_start(motor_3pwm_handle_t *motor_3pwm_h)
{
	if (motor_3pwm_h == NULL) return false;
	if ((motor_3pwm_h->cfg == NULL) || (motor_3pwm_h->cfg->pwm_h == NULL)) return false;
	if (motor_3pwm_h->is_initialized == false) return false;
	if (motor_3pwm_h->is_started == true) return true;

	if (!pwm_tim1_start(motor_3pwm_h->cfg->pwm_h)) return false;

	motor_3pwm_h->is_started = true;
	return true;
}

bool motor_3pwm_stop(motor_3pwm_handle_t *motor_3pwm_h)
{
	if (motor_3pwm_h == NULL) return false;
	if ((motor_3pwm_h->cfg == NULL) || (motor_3pwm_h->cfg->pwm_h == NULL)) return false;
	if (motor_3pwm_h->is_initialized == false) return false;
	if (motor_3pwm_h->is_started == false) return true;

	if (!pwm_tim1_stop(motor_3pwm_h->cfg->pwm_h)) return false;

	motor_3pwm_h->is_started = false;
	return true;
}

bool motor_3pwm_set_duty_abc(motor_3pwm_handle_t *motor_3pwm_h,
							 uint16_t phase_a_duty_permyriad,
							 uint16_t phase_b_duty_permyriad,
							 uint16_t phase_c_duty_permyriad)
{
	if (motor_3pwm_h == NULL) return false;
	if ((motor_3pwm_h->cfg == NULL) || (motor_3pwm_h->cfg->pwm_h == NULL)) return false;
	if (motor_3pwm_h->is_initialized == false) return false;
	if (phase_a_duty_permyriad > 10000u) return false;
	if (phase_b_duty_permyriad > 10000u) return false;
	if (phase_c_duty_permyriad > 10000u) return false;

	if (!motor_3pwm_set_pwm_permyriad(motor_3pwm_h->cfg->pwm_h, 1u, phase_a_duty_permyriad)) return false;
	if (!motor_3pwm_set_pwm_permyriad(motor_3pwm_h->cfg->pwm_h, 2u, phase_b_duty_permyriad)) return false;
	if (!motor_3pwm_set_pwm_permyriad(motor_3pwm_h->cfg->pwm_h, 3u, phase_c_duty_permyriad)) return false;

	motor_3pwm_h->phase_a_duty_permyriad = phase_a_duty_permyriad;
	motor_3pwm_h->phase_b_duty_permyriad = phase_b_duty_permyriad;
	motor_3pwm_h->phase_c_duty_permyriad = phase_c_duty_permyriad;
	return true;
}

bool motor_3pwm_set_neutral(motor_3pwm_handle_t *motor_3pwm_h)
{
	return motor_3pwm_set_duty_abc(motor_3pwm_h, 0u, 0u, 0u);
}
