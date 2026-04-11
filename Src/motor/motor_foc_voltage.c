/**
 * @file motor_foc_voltage.c
 * @brief Voltage-mode FOC actuation helper.
 *
 */

#include "motor/motor_foc_voltage.h"
#include "motor/motor_sine_3pwm.h"
#include <stddef.h>

#define MOTOR_FOC_VOLTAGE_Q_AXIS_SHIFT_90         16384u
#define MOTOR_FOC_VOLTAGE_Q15_MAX                 32767u
#define MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX           10000
#define MOTOR_FOC_VOLTAGE_SQRT3_X10000            17321

/**
 * @brief Divide one signed value with nearest-integer rounding.
 *
 * @param num Signed numerator.
 * @param den Positive denominator.
 * @return Rounded signed quotient.
 */
static int32_t motor_foc_voltage_div_round_signed(int64_t num, int32_t den)
{
	if (num >= 0)
	{
		return (int32_t)((num + (den / 2)) / den);
	}

	return (int32_t)((num - (den / 2)) / den);
}

/**
 * @brief Clamp one signed value to one symmetric limit.
 *
 * @param value Signed value.
 * @param limit Positive magnitude limit.
 * @return Clamped signed value.
 */
static int32_t motor_foc_voltage_clamp_signed(int32_t value, int32_t limit)
{
	if (value > limit) return limit;
	if (value < -limit) return -limit;
	return value;
}

bool motor_foc_voltage_init(motor_foc_voltage_handle_t *motor_foc_voltage_h,
							const motor_foc_voltage_cfg_t *motor_foc_voltage_cfg)
{
	if ((motor_foc_voltage_h == NULL) || (motor_foc_voltage_cfg == NULL)) return false;
	if ((motor_foc_voltage_cfg->motor_h == NULL) || (motor_foc_voltage_cfg->motor_3pwm_h == NULL)) return false;
	if ((motor_foc_voltage_cfg->phase_sequence_sign != 1) &&
		(motor_foc_voltage_cfg->phase_sequence_sign != -1))
	{
		return false;
	}

	motor_foc_voltage_h->cfg = motor_foc_voltage_cfg;
	motor_foc_voltage_h->motor_h = motor_foc_voltage_cfg->motor_h;
	motor_foc_voltage_h->motor_3pwm_h = motor_foc_voltage_cfg->motor_3pwm_h;
	motor_foc_voltage_h->is_initialized = true;

	return true;
}

bool motor_foc_voltage_apply_dq(motor_foc_voltage_handle_t *motor_foc_voltage_h,
								int16_t ud_permyriad,
								int16_t uq_permyriad)
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
	if ((ud_permyriad > (int16_t)motor_h->limits.max_amplitude_permyriad) ||
		(ud_permyriad < -(int16_t)motor_h->limits.max_amplitude_permyriad))
	{
		return false;
	}
	if ((uq_permyriad > (int16_t)motor_h->limits.max_amplitude_permyriad) ||
		(uq_permyriad < -(int16_t)motor_h->limits.max_amplitude_permyriad))
	{
		return false;
	}

	/* Read electrical angle sine/cosine basis for direct inverse Park conversion. */
	uint16_t electrical_angle_u16 = motor_h->measurements.electrical_angle_u16;
	int32_t sin_theta_q15 =
			(int32_t)motor_sine_3pwm_get_phase_q15(electrical_angle_u16);
	int32_t cos_theta_q15 =
			(int32_t)motor_sine_3pwm_get_phase_q15((uint16_t)(electrical_angle_u16 + MOTOR_FOC_VOLTAGE_Q_AXIS_SHIFT_90));

	/* Keep q-axis handedness configurable for phase sequence/wiring differences. */
	int32_t uq_signed_permyriad = (int32_t)uq_permyriad *
								  (int32_t)motor_foc_voltage_h->cfg->phase_sequence_sign;
	int32_t ud_signed_permyriad = (int32_t)ud_permyriad;

	/* Inverse Park in project basis (alpha=sin-axis, beta=cos-axis) to match existing sine modulation convention. */
	/* This keeps Ud=0 q-only runtime behavior consistent with the previously verified rotation direction. */
	int32_t alpha_permyriad = motor_foc_voltage_div_round_signed(
			((int64_t)ud_signed_permyriad * (int64_t)sin_theta_q15) +
			((int64_t)uq_signed_permyriad * (int64_t)cos_theta_q15),
			MOTOR_FOC_VOLTAGE_Q15_MAX);
	int32_t beta_permyriad = motor_foc_voltage_div_round_signed(
			((int64_t)ud_signed_permyriad * (int64_t)cos_theta_q15) -
			((int64_t)uq_signed_permyriad * (int64_t)sin_theta_q15),
			MOTOR_FOC_VOLTAGE_Q15_MAX);

	/* Inverse Clarke: convert (alpha,beta) into three phase commands in permyriad units. */
	int32_t phase_a_permyriad = alpha_permyriad;
	int32_t phase_b_permyriad = motor_foc_voltage_div_round_signed(
			(-(int64_t)alpha_permyriad * MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX) +
			((int64_t)beta_permyriad * MOTOR_FOC_VOLTAGE_SQRT3_X10000),
			2 * MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX);
	int32_t phase_c_permyriad = motor_foc_voltage_div_round_signed(
			(-(int64_t)alpha_permyriad * MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX) -
			((int64_t)beta_permyriad * MOTOR_FOC_VOLTAGE_SQRT3_X10000),
			2 * MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX);

	/* Clamp phase commands to motor amplitude limit before duty synthesis. */
	int32_t phase_limit = (int32_t)motor_h->limits.max_amplitude_permyriad;
	phase_a_permyriad = motor_foc_voltage_clamp_signed(phase_a_permyriad, phase_limit);
	phase_b_permyriad = motor_foc_voltage_clamp_signed(phase_b_permyriad, phase_limit);
	phase_c_permyriad = motor_foc_voltage_clamp_signed(phase_c_permyriad, phase_limit);

	/* Convert permyriad phase commands into Q15-like normalized phase commands. */
	int16_t phase_a_q15 = (int16_t)motor_foc_voltage_div_round_signed(
			(int64_t)phase_a_permyriad * MOTOR_FOC_VOLTAGE_Q15_MAX,
			MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX);
	int16_t phase_b_q15 = (int16_t)motor_foc_voltage_div_round_signed(
			(int64_t)phase_b_permyriad * MOTOR_FOC_VOLTAGE_Q15_MAX,
			MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX);
	int16_t phase_c_q15 = (int16_t)motor_foc_voltage_div_round_signed(
			(int64_t)phase_c_permyriad * MOTOR_FOC_VOLTAGE_Q15_MAX,
			MOTOR_FOC_VOLTAGE_PERMYRIAD_MAX);

	/* Apply direct phase commands without the previous sqrt/angle-search path. */
	if (!motor_sine_3pwm_apply_phase_q15(motor_foc_voltage_h->motor_3pwm_h,
										 phase_a_q15,
										 phase_b_q15,
										 phase_c_q15))
	{
		return false;
	}

	return true;
}

bool motor_foc_voltage_apply_q_only(motor_foc_voltage_handle_t *motor_foc_voltage_h,
									uint16_t uq_permyriad)
{
	return motor_foc_voltage_apply_dq(motor_foc_voltage_h, 0, (int16_t)uq_permyriad);
}
