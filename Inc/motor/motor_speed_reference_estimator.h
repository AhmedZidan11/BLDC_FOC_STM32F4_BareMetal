#ifndef MOTOR_MOTOR_SPEED_REFERENCE_ESTIMATOR_H
#define MOTOR_MOTOR_SPEED_REFERENCE_ESTIMATOR_H

/**
 * @file motor_speed_reference_estimator.h
 * @brief Window-based reference mechanical speed estimator.
 *
 * This module converts sampled mechanical angle values into an estimated
 * mechanical speed by using the shared motor data model.
 *
 * Responsibilities:
 * - store reference-estimator configuration
 * - connect to shared motor-domain state
 * - accept sampled mechanical angle values
 * - keep the angle history used for the estimate
 * - update one reference mechanical speed estimate
 *
 * @note Mechanical angle uses full-turn uint16 units
 *       (0..65535 <=> 0..1 mechanical revolution).
 * @note Estimated speed uses signed milli-rpm units.
 * @note This module is a comparison and diagnostic baseline for open-loop and
 *       future closed-loop evaluation. It is not the intended fast primary
 *       control-loop speed feedback path.
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"

/**
 * @brief Reference speed-estimator configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	uint16_t history_sample_count;      /* Number of kept sample intervals in the sliding window. */
} motor_speed_reference_estimator_cfg_t;

/**
 * @brief Reference speed-estimator runtime handle.
 *
 */
typedef struct {
	const motor_speed_reference_estimator_cfg_t *cfg;
	motor_handle_t *motor_h;
	bool is_initialized;
} motor_speed_reference_estimator_handle_t;

/**
 * @brief Initialize reference speed-estimator handle.
 *
 * @param motor_speed_reference_estimator_h Pointer to reference-estimator handle.
 * @param motor_speed_reference_estimator_cfg Pointer to reference-estimator configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_speed_reference_estimator_init(
		motor_speed_reference_estimator_handle_t *motor_speed_reference_estimator_h,
		const motor_speed_reference_estimator_cfg_t *motor_speed_reference_estimator_cfg);

/**
 * @brief Update reference mechanical speed from one angle sample.
 *
 * The reference estimator stores the sample timestamp for each angle sample
 * and recomputes one local window estimate from the retained history.
 *
 * @param motor_speed_reference_estimator_h Pointer to reference-estimator handle.
 * @param mechanical_angle_u16 Mechanical angle in full-turn uint16 units
 *        (0..65535 <=> 0..1 mechanical revolution).
 * @param sample_timestamp_us Timestamp in microseconds for the angle sample.
 * @return true if update succeeded, false otherwise.
 */
bool motor_speed_reference_estimator_update(
		motor_speed_reference_estimator_handle_t *motor_speed_reference_estimator_h,
		uint16_t mechanical_angle_u16,
		uint64_t sample_timestamp_us);

#endif /* MOTOR_MOTOR_SPEED_REFERENCE_ESTIMATOR_H */
