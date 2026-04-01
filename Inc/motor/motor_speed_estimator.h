#ifndef MOTOR_MOTOR_SPEED_ESTIMATOR_H
#define MOTOR_MOTOR_SPEED_ESTIMATOR_H

/**
 * @file motor_speed_estimator.h
 * @brief Narrow sensor-agnostic mechanical speed estimator.
 *
 * This module converts sampled mechanical angle values into an estimated
 * mechanical speed using the shared motor data model.
 *
 * Responsibilities:
 * - store speed-estimator configuration
 * - bind shared motor-domain state
 * - accept sampled mechanical angle values
 * - update previous angle sample state
 * - update the most recent estimated mechanical speed
 *
 * @note Mechanical angle uses full-turn uint16 units
 *       (0..65535 <=> 0..1 mechanical revolution).
 * @note Estimated speed uses signed milli-rpm units.
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"

/**
 * @brief Speed-estimator configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	uint16_t sample_period_ms;
	uint16_t window_sample_count; /* Number of angle-delta samples per speed window. */
} motor_speed_estimator_cfg_t;

/**
 * @brief Speed-estimator runtime handle.
 *
 */
typedef struct {
	const motor_speed_estimator_cfg_t *cfg;
	motor_handle_t *motor_h;
	bool is_initialized;
} motor_speed_estimator_handle_t;

/**
 * @brief Initialize speed-estimator handle.
 *
 * @param motor_speed_estimator_h Pointer to speed-estimator handle.
 * @param motor_speed_estimator_cfg Pointer to speed-estimator configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_speed_estimator_init(motor_speed_estimator_handle_t *motor_speed_estimator_h,
							    const motor_speed_estimator_cfg_t *motor_speed_estimator_cfg);

/**
 * @brief Update estimated mechanical speed from one angle sample.
 *
 * @param motor_speed_estimator_h Pointer to speed-estimator handle.
 * @param mechanical_angle_u16 Mechanical angle in full-turn uint16 units
 *        (0..65535 <=> 0..1 mechanical revolution).
 * @return true if update succeeded, false otherwise.
 */
bool motor_speed_estimator_update(motor_speed_estimator_handle_t *motor_speed_estimator_h,
							      uint16_t mechanical_angle_u16);

#endif /* MOTOR_MOTOR_SPEED_ESTIMATOR_H */
