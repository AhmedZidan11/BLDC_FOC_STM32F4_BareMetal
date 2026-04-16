#ifndef MOTOR_MOTOR_SPEED_FEEDBACK_H
#define MOTOR_MOTOR_SPEED_FEEDBACK_H

/**
 * @file motor_speed_feedback.h
 * @brief Low-latency mechanical speed feedback helper.
 *
 * This module converts fixed-period mechanical angle samples into signed
 * mechanical speed and applies a first-order low-pass filter.
 *
 * Responsibilities:
 * - store fixed-period speed-feedback configuration
 * - connect to shared motor-domain state
 * - compute wrap-safe angle delta between adjacent samples
 * - convert delta and fixed dt into raw signed mechanical speed
 * - apply a first-order low-pass filter with time-constant tuning
 * - publish filtered speed for future control use
 *
 * @note Mechanical angle uses full-turn uint16 units.
 * @note Mechanical speed uses signed milli-rpm units.
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"

/**
 * @brief Speed-feedback configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	uint32_t sample_period_us;
	uint16_t filter_time_constant_ms; /* First-order LPF time constant tau. */
	int8_t control_direction_sign;    /* Control-positive mechanical speed sign (+1 or -1). */
} motor_speed_feedback_cfg_t;

/**
 * @brief Speed-feedback runtime handle.
 *
 */
typedef struct {
	const motor_speed_feedback_cfg_t *cfg;
	motor_handle_t *motor_h;
	uint16_t filter_coeff_q15;
	uint16_t previous_mechanical_angle_u16;
	bool has_previous_mechanical_angle;
	bool is_initialized;
} motor_speed_feedback_handle_t;

/**
 * @brief Initialize speed-feedback handle.
 *
 * @param motor_speed_feedback_h Pointer to speed-feedback handle.
 * @param motor_speed_feedback_cfg Pointer to speed-feedback configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_speed_feedback_init(motor_speed_feedback_handle_t *motor_speed_feedback_h,
							   const motor_speed_feedback_cfg_t *motor_speed_feedback_cfg);

/**
 * @brief Update speed feedback from one measured mechanical angle sample.
 *
 * @param motor_speed_feedback_h Pointer to speed-feedback handle.
 * @param mechanical_angle_u16 Measured mechanical angle in full-turn uint16 units.
 * @return true if update succeeded, false otherwise.
 */
bool motor_speed_feedback_update(motor_speed_feedback_handle_t *motor_speed_feedback_h,
								 uint16_t mechanical_angle_u16);

#endif /* MOTOR_MOTOR_SPEED_FEEDBACK_H */
