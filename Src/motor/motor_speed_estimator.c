/**
 * @file motor_speed_estimator.c
 * @brief Narrow sensor-agnostic mechanical speed estimator.
 *
 */

#include "motor/motor_speed_estimator.h"
#include <stddef.h>

#define MOTOR_SPEED_ESTIMATOR_MECHANICAL_TURN_COUNTS_U16  65536LL
#define MOTOR_SPEED_ESTIMATOR_MRPM_PER_RPM                1000LL
#define MOTOR_SPEED_ESTIMATOR_US_PER_MINUTE               60000000LL

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
	if (motor_speed_estimator_cfg->sample_period_us == 0u) return false;
	if (motor_speed_estimator_cfg->history_sample_count == 0u) return false;
	if (motor_speed_estimator_cfg->history_sample_count > MOTOR_SPEED_ESTIMATOR_MAX_HISTORY_SAMPLES) return false;

	motor_handle_t *motor_h = motor_speed_estimator_cfg->motor_h;

	/* Reset sliding-history state and the published speed output. */
	motor_speed_estimator_h->cfg = motor_speed_estimator_cfg;
	motor_speed_estimator_h->motor_h = motor_h;
	for (uint16_t i = 0u; i < MOTOR_SPEED_ESTIMATOR_HISTORY_BUFFER_SIZE; i++)
	{
		motor_h->speed_estimator.mechanical_angle_history_u16[i] = 0u;
		motor_h->speed_estimator.last_window_angle_u16[i] = 0u;
	}

	motor_h->speed_estimator.history_write_index = 0u;
	motor_h->speed_estimator.history_valid_count = 0u;
	motor_h->speed_estimator.last_mechanical_angle_delta_counts = 0;
	motor_h->speed_estimator.last_elapsed_time_us = 0u;
	motor_h->speed_estimator.last_window_point_count = 0u;
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
	if (motor_speed_estimator_h->cfg->sample_period_us == 0u) return false;
	if (motor_speed_estimator_h->cfg->history_sample_count == 0u) return false;
	if (motor_speed_estimator_h->cfg->history_sample_count > MOTOR_SPEED_ESTIMATOR_MAX_HISTORY_SAMPLES) return false;

	motor_handle_t *motor_h = motor_speed_estimator_h->motor_h;
	motor_h->measurements.mechanical_angle_u16 = mechanical_angle_u16;
	motor_h->status.has_valid_mechanical_angle = true;

	uint16_t history_buffer_size = (uint16_t)(motor_speed_estimator_h->cfg->history_sample_count + 1u);
	uint16_t write_index = motor_h->speed_estimator.history_write_index;

	/* Store the new sample into the sliding angle-history buffer. */
	motor_h->speed_estimator.mechanical_angle_history_u16[write_index] = mechanical_angle_u16;
	write_index = (uint16_t)((write_index + 1u) % history_buffer_size);
	motor_h->speed_estimator.history_write_index = write_index;
	if (motor_h->speed_estimator.history_valid_count < history_buffer_size)
	{
		motor_h->speed_estimator.history_valid_count++;
	}

	/* Fill the sliding window before publishing the first speed estimate. */
	if (motor_h->speed_estimator.history_valid_count < history_buffer_size)
	{
		motor_h->measurements.measured_mechanical_speed_mrpm = 0;
		motor_h->status.has_valid_mechanical_speed = false;
		return true;
	}

	/* Copy the active sliding window in oldest-to-newest order for diagnostics. */
	for (uint16_t i = 0u; i < history_buffer_size; i++)
	{
		/* The oldest retained sample is at history_write_index after the new sample write. */
		uint16_t history_index =
				(uint16_t)((motor_h->speed_estimator.history_write_index + i) % history_buffer_size);
		motor_h->speed_estimator.last_window_angle_u16[i] =
				motor_h->speed_estimator.mechanical_angle_history_u16[history_index];
	}
	motor_h->speed_estimator.last_window_point_count = history_buffer_size;

	/* Select the retained oldest sample so delta = current_angle - angle_from_N_samples_ago. */
	uint16_t oldest_mechanical_angle_u16 = motor_h->speed_estimator.last_window_angle_u16[0];

	/* Convert the sliding history endpoints into a signed wrapped angle delta. */
	int32_t mechanical_angle_delta = motor_speed_estimator_angle_delta_u16(
			mechanical_angle_u16,
			oldest_mechanical_angle_u16);
	motor_h->speed_estimator.last_mechanical_angle_delta_counts = mechanical_angle_delta;

	/* elapsed_time_us = history_sample_count * sample_period_us for the fixed estimator cadence. */
	uint32_t elapsed_time_us =
			(uint32_t)((uint32_t)motor_speed_estimator_h->cfg->history_sample_count *
					  motor_speed_estimator_h->cfg->sample_period_us);
	motor_h->speed_estimator.last_elapsed_time_us = elapsed_time_us;
	if (elapsed_time_us == 0u)
	{
		motor_h->measurements.measured_mechanical_speed_mrpm = 0;
		motor_h->status.has_valid_mechanical_speed = false;
		return true;
	}

	/* sliding angle delta / real elapsed time -> signed mechanical speed in mrpm. */
	int64_t mechanical_speed_mrpm_num =
			(int64_t)mechanical_angle_delta *
			MOTOR_SPEED_ESTIMATOR_US_PER_MINUTE *
			MOTOR_SPEED_ESTIMATOR_MRPM_PER_RPM;
	int64_t mechanical_speed_mrpm_den =
			MOTOR_SPEED_ESTIMATOR_MECHANICAL_TURN_COUNTS_U16 *
			(int64_t)((uint64_t)elapsed_time_us);
	int32_t estimated_mechanical_speed_mrpm =
			(int32_t)(mechanical_speed_mrpm_num / mechanical_speed_mrpm_den);

	motor_h->measurements.measured_mechanical_speed_mrpm = estimated_mechanical_speed_mrpm;
	motor_h->status.has_valid_mechanical_speed = true;

	return true;
}
