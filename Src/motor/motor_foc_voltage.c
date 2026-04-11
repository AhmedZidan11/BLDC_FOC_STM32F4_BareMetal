/**
 * @file motor_foc_voltage.c
 * @brief Q-axis-only voltage-mode FOC actuation kernel.
 *
 */

#include "motor/motor_foc_voltage.h"
#include "motor/motor_sine_3pwm.h"
#include <stddef.h>

#define MOTOR_FOC_VOLTAGE_Q_AXIS_SHIFT_90         16384u

bool motor_foc_voltage_init(motor_foc_voltage_handle_t *motor_foc_voltage_h,
							const motor_foc_voltage_cfg_t *motor_foc_voltage_cfg)
{
	if ((motor_foc_voltage_h == NULL) || (motor_foc_voltage_cfg == NULL)) return false;
	if ((motor_foc_voltage_cfg->motor_h == NULL) || (motor_foc_voltage_cfg->motor_3pwm_h == NULL)) return false;

	motor_foc_voltage_h->cfg = motor_foc_voltage_cfg;
	motor_foc_voltage_h->motor_h = motor_foc_voltage_cfg->motor_h;
	motor_foc_voltage_h->motor_3pwm_h = motor_foc_voltage_cfg->motor_3pwm_h;
	motor_foc_voltage_h->is_initialized = true;

	return true;
}

bool motor_foc_voltage_apply_q_only(motor_foc_voltage_handle_t *motor_foc_voltage_h,
									uint16_t uq_permyriad)
{
	if (motor_foc_voltage_h == NULL) return false;
	if ((motor_foc_voltage_h->cfg == NULL) ||
		(motor_foc_voltage_h->motor_h == NULL) ||
		(motor_foc_voltage_h->motor_3pwm_h == NULL))
	{
		return false;
	}
	if (motor_foc_voltage_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_foc_voltage_h->motor_h;
	if (motor_h->status.has_valid_electrical_angle == false) return false;
	if (motor_h->limits.max_amplitude_permyriad == 0u) return false;
	if (motor_h->limits.max_amplitude_permyriad > MOTOR_SINE_3PWM_MAX_AMPLITUDE_PERMYRIAD) return false;
	if (uq_permyriad > motor_h->limits.max_amplitude_permyriad) return false;

	/* For Ud = 0 and Uq > 0, voltage vector angle = measured electrical angle - 90 degrees. */
	uint16_t uq_vector_angle_u16 = (uint16_t)(motor_h->measurements.electrical_angle_u16 -
											   MOTOR_FOC_VOLTAGE_Q_AXIS_SHIFT_90);

	/* Apply shared sine 3-phase modulation for the q-only vector angle. */
	if (!motor_sine_3pwm_apply(motor_foc_voltage_h->motor_3pwm_h,
							   uq_vector_angle_u16,
							   uq_permyriad))
	{
		return false;
	}

	return true;
}
