/**
 * @file motor_speed_feedback.c
 * @brief Low-latency mechanical speed feedback helper.
 *
 */

#include "motor/motor_speed_feedback.h"
#include <stddef.h>

#define MOTOR_SPEED_FEEDBACK_MECHANICAL_TURN_COUNTS_U16  65536LL
#define MOTOR_SPEED_FEEDBACK_MRPM_PER_RPM                1000LL
#define MOTOR_SPEED_FEEDBACK_US_PER_MINUTE               60000000LL
#define MOTOR_SPEED_FEEDBACK_Q15_SCALE                   32768LL

/**
 * @brief Divide one signed integer and round to nearest integer.
 *
 * @param numerator Signed numerator.
 * @param denominator Positive denominator.
 * @return Rounded signed integer quotient.
 */
static int64_t motor_speed_feedback_divide_round_nearest(int64_t numerator,
														 int64_t denominator)
{
	if (numerator >= 0)
	{
		return (numerator + (denominator / 2LL)) / denominator;
	}

	return (numerator - (denominator / 2LL)) / denominator;
}

bool motor_speed_feedback_init(motor_speed_feedback_handle_t *motor_speed_feedback_h,
							   const motor_speed_feedback_cfg_t *motor_speed_feedback_cfg)
{
	if ((motor_speed_feedback_h == NULL) || (motor_speed_feedback_cfg == NULL)) return false;
	if (motor_speed_feedback_cfg->motor_h == NULL) return false;
	if (motor_speed_feedback_cfg->sample_period_us == 0u) return false;
	if ((motor_speed_feedback_cfg->control_direction_sign != 1) &&
		(motor_speed_feedback_cfg->control_direction_sign != -1)) return false;

	motor_handle_t *motor_h = motor_speed_feedback_cfg->motor_h;
	uint64_t sample_period_us = motor_speed_feedback_cfg->sample_period_us;
	uint64_t filter_time_constant_us =
			(uint64_t)motor_speed_feedback_cfg->filter_time_constant_ms * 1000u;
	uint64_t filter_coeff_den = filter_time_constant_us + sample_period_us;
	uint64_t filter_coeff_q15_u64 = 0u;

	/* Derive k = dt / (tau + dt) in Q15 for the first-order low-pass filter. */
	if (filter_time_constant_us == 0u)
	{
		filter_coeff_q15_u64 = (uint64_t)MOTOR_SPEED_FEEDBACK_Q15_SCALE;
	}
	else
	{
		filter_coeff_q15_u64 =
				((sample_period_us * (uint64_t)MOTOR_SPEED_FEEDBACK_Q15_SCALE) +
				 (filter_coeff_den / 2u)) / filter_coeff_den;
		if (filter_coeff_q15_u64 == 0u)
		{
			filter_coeff_q15_u64 = 1u;
		}
	}

	/* Reset feedback runtime state and shared outputs. */
	motor_speed_feedback_h->cfg = motor_speed_feedback_cfg;
	motor_speed_feedback_h->motor_h = motor_h;
	motor_speed_feedback_h->filter_coeff_q15 = (uint16_t)filter_coeff_q15_u64;
	motor_speed_feedback_h->previous_mechanical_angle_u16 = 0u;
	motor_speed_feedback_h->has_previous_mechanical_angle = false;
	motor_speed_feedback_h->is_initialized = true;

	motor_h->speed_feedback.raw_mechanical_speed_mrpm = 0;
	motor_h->speed_feedback.filtered_mechanical_speed_mrpm = 0;
	motor_h->measurements.measured_mechanical_speed_mrpm = 0;
	motor_h->status.has_valid_mechanical_speed = false;

	return true;
}

bool motor_speed_feedback_update(motor_speed_feedback_handle_t *motor_speed_feedback_h,
								 uint16_t mechanical_angle_u16)
{
	if (motor_speed_feedback_h == NULL) return false;
	if ((motor_speed_feedback_h->cfg == NULL) || (motor_speed_feedback_h->motor_h == NULL)) return false;
	if (motor_speed_feedback_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_speed_feedback_h->motor_h;
	int32_t mechanical_angle_delta_counts = 0;
	int64_t raw_mechanical_speed_num = 0;
	int64_t raw_mechanical_speed_mrpm = 0;
	int64_t filter_error_mrpm = 0;
	int64_t filter_step_mrpm = 0;
	int64_t filtered_mechanical_speed_mrpm = 0;

	/* Wait for one previous angle sample before speed conversion starts. */
	if (motor_speed_feedback_h->has_previous_mechanical_angle == false)
	{
		motor_speed_feedback_h->previous_mechanical_angle_u16 = mechanical_angle_u16;
		motor_speed_feedback_h->has_previous_mechanical_angle = true;
		motor_h->speed_feedback.raw_mechanical_speed_mrpm = 0;
		motor_h->speed_feedback.filtered_mechanical_speed_mrpm = 0;
		motor_h->measurements.measured_mechanical_speed_mrpm = 0;
		motor_h->status.has_valid_mechanical_speed = false;
		return true;
	}

	/* Compute one shortest signed wrapped delta in full-turn counts. */
	mechanical_angle_delta_counts =
			(int32_t)((int16_t)((uint16_t)(mechanical_angle_u16 -
									    motor_speed_feedback_h->previous_mechanical_angle_u16)));
	motor_speed_feedback_h->previous_mechanical_angle_u16 = mechanical_angle_u16;

	/* raw_mrpm = delta_counts * 60e6 * 1000 / (65536 * sample_period_us). */
	raw_mechanical_speed_num =
			(int64_t)mechanical_angle_delta_counts *
			(int64_t)MOTOR_SPEED_FEEDBACK_US_PER_MINUTE *
			(int64_t)MOTOR_SPEED_FEEDBACK_MRPM_PER_RPM;
	raw_mechanical_speed_mrpm =
			motor_speed_feedback_divide_round_nearest(
					raw_mechanical_speed_num,
					(int64_t)MOTOR_SPEED_FEEDBACK_MECHANICAL_TURN_COUNTS_U16 *
					(int64_t)motor_speed_feedback_h->cfg->sample_period_us);

	/* Apply the project control-direction sign before publishing and filtering speed. */
	raw_mechanical_speed_mrpm *= (int64_t)motor_speed_feedback_h->cfg->control_direction_sign;

	/* Apply y += k * (x - y) with fixed-point Q15 coefficient k. */
	filter_error_mrpm =
			raw_mechanical_speed_mrpm -
			(int64_t)motor_h->speed_feedback.filtered_mechanical_speed_mrpm;
	filter_step_mrpm =
			motor_speed_feedback_divide_round_nearest(
					filter_error_mrpm * (int64_t)motor_speed_feedback_h->filter_coeff_q15,
					(int64_t)MOTOR_SPEED_FEEDBACK_Q15_SCALE);
	filtered_mechanical_speed_mrpm =
			(int64_t)motor_h->speed_feedback.filtered_mechanical_speed_mrpm + filter_step_mrpm;

	/* Publish raw and filtered speed in shared motor state. */
	motor_h->speed_feedback.raw_mechanical_speed_mrpm = (int32_t)raw_mechanical_speed_mrpm;
	motor_h->speed_feedback.filtered_mechanical_speed_mrpm = (int32_t)filtered_mechanical_speed_mrpm;
	motor_h->measurements.measured_mechanical_speed_mrpm = (int32_t)filtered_mechanical_speed_mrpm;
	motor_h->status.has_valid_mechanical_speed = true;

	return true;
}
