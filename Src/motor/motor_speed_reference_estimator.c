/**
 * @file motor_speed_reference_estimator.c
 * @brief Window-based reference mechanical speed estimator.
 *
 * This module computes a reference mechanical speed from measured angle
 * samples. It is meant for comparison and diagnostic baseline work during
 * open-loop and future closed-loop evaluation, not as the intended fast
 * primary control-loop speed feedback path.
 */

#include "motor/motor_speed_reference_estimator.h"
#include <stddef.h>

#define MOTOR_SPEED_ESTIMATOR_MECHANICAL_TURN_COUNTS_U16  65536LL
#define MOTOR_SPEED_ESTIMATOR_MRPM_PER_RPM                1000LL
#define MOTOR_SPEED_ESTIMATOR_US_PER_MINUTE               60000000LL
#define MOTOR_SPEED_ESTIMATOR_REGRESSION_DENOM_EPSILON    1e-12

/**
 * @brief Compute signed shortest-path delta between two uint16 full-turn angles.
 *
 * @param current_mechanical_angle_u16 Current mechanical angle sample.
 * @param previous_mechanical_angle_u16 Previous mechanical angle sample.
 * @return Signed wrapped delta in angle counts.
 */
static int32_t motor_speed_reference_estimator_angle_delta_u16(uint16_t current_mechanical_angle_u16,
															   uint16_t previous_mechanical_angle_u16)
{
	return (int32_t)((int16_t)((uint16_t)(current_mechanical_angle_u16 - previous_mechanical_angle_u16)));
}

bool motor_speed_reference_estimator_init(
		motor_speed_reference_estimator_handle_t *motor_speed_reference_estimator_h,
		const motor_speed_reference_estimator_cfg_t *motor_speed_reference_estimator_cfg)
{
	if ((motor_speed_reference_estimator_h == NULL) || (motor_speed_reference_estimator_cfg == NULL)) return false;
	if (motor_speed_reference_estimator_cfg->motor_h == NULL) return false;
	if (motor_speed_reference_estimator_cfg->history_sample_count == 0u) return false;
	if (motor_speed_reference_estimator_cfg->history_sample_count > MOTOR_SPEED_ESTIMATOR_MAX_HISTORY_SAMPLES) return false;

	motor_handle_t *motor_h = motor_speed_reference_estimator_cfg->motor_h;

	/* Reset the sliding reference window state and the published speed output. */
	motor_speed_reference_estimator_h->cfg = motor_speed_reference_estimator_cfg;
	motor_speed_reference_estimator_h->motor_h = motor_h;
	for (uint16_t i = 0u; i < MOTOR_SPEED_ESTIMATOR_HISTORY_BUFFER_SIZE; i++)
	{
		motor_h->speed_reference_estimator.sample_timestamp_history_us[i] = 0u;
		motor_h->speed_reference_estimator.cumulative_unwrapped_angle_history_counts[i] = 0;
	}

	motor_h->speed_reference_estimator.history_write_index = 0u;
	motor_h->speed_reference_estimator.history_valid_count = 0u;
	motor_h->speed_reference_estimator.last_window_point_count = 0u;
	motor_h->speed_reference_estimator.previous_raw_mechanical_angle_u16 = 0u;
	motor_h->speed_reference_estimator.has_previous_raw_mechanical_angle = false;
	motor_h->speed_reference_estimator.last_mechanical_angle_delta_counts = 0;
	motor_h->speed_reference_estimator.last_elapsed_time_us = 0u;
	motor_h->speed_reference_estimator.cumulative_unwrapped_mechanical_angle_counts = 0;
	motor_h->measurements.measured_mechanical_speed_mrpm = 0;
	motor_h->status.has_valid_mechanical_speed = false;
	motor_speed_reference_estimator_h->is_initialized = true;

	return true;
}

bool motor_speed_reference_estimator_update(
		motor_speed_reference_estimator_handle_t *motor_speed_reference_estimator_h,
		uint16_t mechanical_angle_u16,
		uint64_t sample_timestamp_us)
{
	if (motor_speed_reference_estimator_h == NULL) return false;
	if ((motor_speed_reference_estimator_h->cfg == NULL) || (motor_speed_reference_estimator_h->motor_h == NULL)) return false;
	if (motor_speed_reference_estimator_h->is_initialized == false) return false;
	if (motor_speed_reference_estimator_h->cfg->history_sample_count == 0u) return false;
	if (motor_speed_reference_estimator_h->cfg->history_sample_count > MOTOR_SPEED_ESTIMATOR_MAX_HISTORY_SAMPLES) return false;

	motor_handle_t *motor_h = motor_speed_reference_estimator_h->motor_h;
	motor_speed_reference_estimator_state_t *motor_speed_reference_estimator_state =
			&motor_h->speed_reference_estimator;
	uint16_t history_buffer_size =
			(uint16_t)(motor_speed_reference_estimator_h->cfg->history_sample_count + 1u);
	uint16_t write_index = motor_speed_reference_estimator_state->history_write_index;
	int32_t mechanical_angle_delta_counts = 0;

	/* Store the newest mechanical angle sample in shared motor state. */
	motor_h->measurements.mechanical_angle_u16 = mechanical_angle_u16;
	motor_h->status.has_valid_mechanical_angle = true;

	/* Build one cumulative unwrapped angle count from adjacent wrapped angle steps. */
	if (motor_speed_reference_estimator_state->has_previous_raw_mechanical_angle == false)
	{
		motor_speed_reference_estimator_state->previous_raw_mechanical_angle_u16 = mechanical_angle_u16;
		motor_speed_reference_estimator_state->has_previous_raw_mechanical_angle = true;
		motor_speed_reference_estimator_state->cumulative_unwrapped_mechanical_angle_counts = 0;
	}
	else
	{
		/* Use the shortest wrapped raw-angle step between the previous and current sample. */
		mechanical_angle_delta_counts = motor_speed_reference_estimator_angle_delta_u16(
				mechanical_angle_u16,
				motor_speed_reference_estimator_state->previous_raw_mechanical_angle_u16);
		motor_speed_reference_estimator_state->previous_raw_mechanical_angle_u16 = mechanical_angle_u16;
		motor_speed_reference_estimator_state->cumulative_unwrapped_mechanical_angle_counts +=
				(int64_t)mechanical_angle_delta_counts;
	}
	motor_speed_reference_estimator_state->last_mechanical_angle_delta_counts = mechanical_angle_delta_counts;

	/* Store the newest sample in the sliding history buffer. */
	motor_speed_reference_estimator_state->sample_timestamp_history_us[write_index] = sample_timestamp_us;
	motor_speed_reference_estimator_state->cumulative_unwrapped_angle_history_counts[write_index] =
			motor_speed_reference_estimator_state->cumulative_unwrapped_mechanical_angle_counts;
	if (motor_speed_reference_estimator_state->history_valid_count < history_buffer_size)
	{
		motor_speed_reference_estimator_state->history_valid_count++;
	}
	motor_speed_reference_estimator_state->history_write_index =
			(uint16_t)((write_index + 1u) % history_buffer_size);
	motor_speed_reference_estimator_state->last_window_point_count =
			motor_speed_reference_estimator_state->history_valid_count;

	/* Wait until the full reference window is available before publishing speed. */
	if (motor_speed_reference_estimator_state->history_valid_count < history_buffer_size)
	{
		motor_speed_reference_estimator_state->last_elapsed_time_us = 0u;
		motor_h->measurements.measured_mechanical_speed_mrpm = 0;
		motor_h->status.has_valid_mechanical_speed = false;
		return true;
	}

	uint16_t oldest_index = motor_speed_reference_estimator_state->history_write_index;
	uint16_t newest_index = (uint16_t)((motor_speed_reference_estimator_state->history_write_index +
										history_buffer_size - 1u) % history_buffer_size);
	uint64_t oldest_timestamp_us =
			motor_speed_reference_estimator_state->sample_timestamp_history_us[oldest_index];
	uint64_t newest_timestamp_us =
			motor_speed_reference_estimator_state->sample_timestamp_history_us[newest_index];
	int64_t oldest_cumulative_unwrapped_counts =
			motor_speed_reference_estimator_state->cumulative_unwrapped_angle_history_counts[oldest_index];
	double point_count_f64 = (double)history_buffer_size;
	double regression_sum_x = 0.0;
	double regression_sum_y = 0.0;
	double regression_sum_xx = 0.0;
	double regression_sum_xy = 0.0;

	/* Rebuild one local regression window with the oldest sample as the origin. */
	for (uint16_t point_offset = 0u; point_offset < history_buffer_size; point_offset++)
	{
		uint16_t sample_index = (uint16_t)((oldest_index + point_offset) % history_buffer_size);
		uint64_t local_x_us =
				motor_speed_reference_estimator_state->sample_timestamp_history_us[sample_index] -
				oldest_timestamp_us;
		int64_t local_y_counts =
				motor_speed_reference_estimator_state->cumulative_unwrapped_angle_history_counts[sample_index] -
				oldest_cumulative_unwrapped_counts;
		double local_x_f64 = (double)local_x_us;
		double local_y_f64 = (double)local_y_counts;

		regression_sum_x += local_x_f64;
		regression_sum_y += local_y_f64;
		regression_sum_xx += local_x_f64 * local_x_f64;
		regression_sum_xy += local_x_f64 * local_y_f64;
	}

	/* Track the real active window span from the current oldest and newest timestamps. */
	motor_speed_reference_estimator_state->last_elapsed_time_us = newest_timestamp_us - oldest_timestamp_us;
	double regression_denom_f64 =
			(point_count_f64 * regression_sum_xx) - (regression_sum_x * regression_sum_x);
	if ((motor_speed_reference_estimator_state->last_elapsed_time_us == 0u) ||
		(regression_denom_f64 <= MOTOR_SPEED_ESTIMATOR_REGRESSION_DENOM_EPSILON))
	{
		motor_h->measurements.measured_mechanical_speed_mrpm = 0;
		motor_h->status.has_valid_mechanical_speed = false;
		return true;
	}

	/* slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x^2). */
	double slope_counts_per_us =
			((point_count_f64 * regression_sum_xy) - (regression_sum_x * regression_sum_y)) /
			regression_denom_f64;

	/* Convert counts/us slope into signed milli-rpm. */
	double estimated_mechanical_speed_mrpm_f64 =
			(slope_counts_per_us * (double)MOTOR_SPEED_ESTIMATOR_US_PER_MINUTE *
			 (double)MOTOR_SPEED_ESTIMATOR_MRPM_PER_RPM) /
			(double)MOTOR_SPEED_ESTIMATOR_MECHANICAL_TURN_COUNTS_U16;
	int32_t estimated_mechanical_speed_mrpm =
			(int32_t)((estimated_mechanical_speed_mrpm_f64 >= 0.0) ?
					  (estimated_mechanical_speed_mrpm_f64 + 0.5) :
					  (estimated_mechanical_speed_mrpm_f64 - 0.5));

	/* Store the newest signed mechanical speed estimate in milli-rpm. */
	motor_h->measurements.measured_mechanical_speed_mrpm = estimated_mechanical_speed_mrpm;
	motor_h->status.has_valid_mechanical_speed = true;

	return true;
}
