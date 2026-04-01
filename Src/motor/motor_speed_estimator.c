/**
 * @file motor_speed_estimator.c
 * @brief Narrow sensor-agnostic mechanical speed estimator.
 *
 */

#include "motor/motor_speed_estimator.h"
#include <stddef.h>

#define MOTOR_SPEED_ESTIMATOR_MECHANICAL_TURN_COUNTS_U16  65536LL
#define MOTOR_SPEED_ESTIMATOR_MRPM_PER_RPM                1000LL
#define MOTOR_SPEED_ESTIMATOR_MS_PER_MINUTE               60000LL

/**
 * @brief Compute signed shortest-path delta between two uint16 full-turn angles.
 *
 * @param current_mechanical_angle_u16 Current mechanical angle sample.
 * @param previous_mechanical_angle_u16 Previous mechanical angle sample.
 * @return Signed wrapped delta in angle counts.
 */
static int32_t motor_speed_estimator_angle_delta_u16(uint16_t current_mechanical_angle_u16,
													  uint16_t previous_mechanical_angle_u16)
{
	return (int32_t)((int16_t)((uint16_t)(current_mechanical_angle_u16 - previous_mechanical_angle_u16)));
}

bool motor_speed_estimator_init(motor_speed_estimator_handle_t *motor_speed_estimator_h,
							    const motor_speed_estimator_cfg_t *motor_speed_estimator_cfg)
{
	if ((motor_speed_estimator_h == NULL) || (motor_speed_estimator_cfg == NULL)) return false;
	if (motor_speed_estimator_cfg->motor_h == NULL) return false;
	if (motor_speed_estimator_cfg->sample_period_ms == 0u) return false;
	if (motor_speed_estimator_cfg->window_sample_count == 0u) return false;

	motor_handle_t *motor_h = motor_speed_estimator_cfg->motor_h;

	/* Reset estimator history and the active speed window. */
	motor_speed_estimator_h->cfg = motor_speed_estimator_cfg;
	motor_speed_estimator_h->motor_h = motor_h;
	motor_h->speed_estimator.previous_mechanical_angle_u16 = 0u;
	motor_h->speed_estimator.accumulated_mechanical_angle_delta = 0;
	motor_h->speed_estimator.accumulated_sample_count = 0u;
	motor_h->speed_estimator.has_previous_mechanical_angle_sample = false;
	motor_h->measurements.measured_mechanical_speed_mrpm = 0;
	motor_h->status.has_valid_mechanical_speed = false;
	motor_speed_estimator_h->is_initialized = true;

	return true;
}

bool motor_speed_estimator_update(motor_speed_estimator_handle_t *motor_speed_estimator_h,
							      uint16_t mechanical_angle_u16)
{
	if (motor_speed_estimator_h == NULL) return false;
	if ((motor_speed_estimator_h->cfg == NULL) || (motor_speed_estimator_h->motor_h == NULL)) return false;
	if (motor_speed_estimator_h->is_initialized == false) return false;
	if (motor_speed_estimator_h->cfg->sample_period_ms == 0u) return false;
	if (motor_speed_estimator_h->cfg->window_sample_count == 0u) return false;

	motor_handle_t *motor_h = motor_speed_estimator_h->motor_h;
	motor_h->measurements.mechanical_angle_u16 = mechanical_angle_u16;
	motor_h->status.has_valid_mechanical_angle = true;

	/* Seed the estimator with the first mechanical angle sample. */
	if (motor_h->speed_estimator.has_previous_mechanical_angle_sample == false)
	{
		motor_h->speed_estimator.previous_mechanical_angle_u16 = mechanical_angle_u16;
		motor_h->speed_estimator.accumulated_mechanical_angle_delta = 0;
		motor_h->speed_estimator.accumulated_sample_count = 0u;
		motor_h->measurements.measured_mechanical_speed_mrpm = 0;
		motor_h->speed_estimator.has_previous_mechanical_angle_sample = true;
		motor_h->status.has_valid_mechanical_speed = false;
		return true;
	}

	/* Convert the new sample into a signed wrapped angle delta. */
	int32_t mechanical_angle_delta = motor_speed_estimator_angle_delta_u16(
			mechanical_angle_u16,
			motor_h->speed_estimator.previous_mechanical_angle_u16);

	/* Accumulate angle delta and sample count for the active speed window. */
	motor_h->speed_estimator.previous_mechanical_angle_u16 = mechanical_angle_u16;
	motor_h->speed_estimator.accumulated_mechanical_angle_delta += mechanical_angle_delta;
	motor_h->speed_estimator.accumulated_sample_count++;

	/* Publish one new speed estimate when the configured window is complete. */
	if (motor_h->speed_estimator.accumulated_sample_count >= motor_speed_estimator_h->cfg->window_sample_count)
	{
		/* total_angle_delta / total_window_time -> signed mechanical speed in mrpm. */
		int64_t mechanical_speed_mrpm_num =
				(int64_t)motor_h->speed_estimator.accumulated_mechanical_angle_delta *
				MOTOR_SPEED_ESTIMATOR_MS_PER_MINUTE *
				MOTOR_SPEED_ESTIMATOR_MRPM_PER_RPM;
		int64_t mechanical_speed_mrpm_den =
				MOTOR_SPEED_ESTIMATOR_MECHANICAL_TURN_COUNTS_U16 *
				(int64_t)motor_speed_estimator_h->cfg->sample_period_ms *
				(int64_t)motor_h->speed_estimator.accumulated_sample_count;
		int32_t estimated_mechanical_speed_mrpm =
				(int32_t)(mechanical_speed_mrpm_num / mechanical_speed_mrpm_den);

		motor_h->measurements.measured_mechanical_speed_mrpm = estimated_mechanical_speed_mrpm;
		motor_h->speed_estimator.accumulated_mechanical_angle_delta = 0;
		motor_h->speed_estimator.accumulated_sample_count = 0u;
		motor_h->status.has_valid_mechanical_speed = true;
	}

	return true;
}
