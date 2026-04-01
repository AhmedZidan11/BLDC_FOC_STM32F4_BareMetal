#ifndef MOTOR_MOTOR_H
#define MOTOR_MOTOR_H

/**
 * @file motor.h
 * @brief Shared motor-domain data model.
 *
 * This header collects shared motor-domain types that can be used by motor
 * observation and open-loop helper modules.
 *
 * Responsibilities:
 * - define shared motor measurements
 * - define shared motor targets and limits
 * - define shared motor status flags
 * - define shared runtime state for open-loop and speed-estimation helpers
 *
 * @note Mechanical and electrical angles use full-turn uint16 units.
 * @note Mechanical speed uses signed milli-rpm units for measurements,
 *       targets and limits.
 * @note Amplitude uses permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Motor operating mode within the current project scope.
 *
 */
typedef enum {
	MOTOR_MODE_INACTIVE = 0,
	MOTOR_MODE_OBSERVE,
	MOTOR_MODE_OPENLOOP_STATIC,
	MOTOR_MODE_OPENLOOP_DRIVE
} motor_mode_t;

/**
 * @brief Motor measurement values.
 *
 */
typedef struct {
	uint16_t mechanical_angle_u16;
	uint16_t electrical_angle_u16;
	int32_t measured_mechanical_speed_mrpm;
} motor_measurements_t;

/**
 * @brief Motor target values.
 *
 */
typedef struct {
	int32_t target_mechanical_speed_mrpm;
	uint16_t target_amplitude_permyriad;
} motor_targets_t;

/**
 * @brief Motor limit values.
 *
 */
typedef struct {
	uint16_t pole_pairs;
	int32_t max_target_mechanical_speed_mrpm;
	uint16_t max_amplitude_permyriad;
} motor_limits_t;

/**
 * @brief Motor status flags.
 *
 */
typedef struct {
	motor_mode_t mode;
	bool is_initialized;
	bool is_enabled;
	bool has_valid_mechanical_angle;
	bool has_valid_electrical_angle;
	bool has_valid_mechanical_speed;
} motor_status_t;

/**
 * @brief Open-loop runtime state.
 *
 */
typedef struct {
	uint32_t phase_accumulator_u32;
	uint32_t current_phase_increment_u32;
	uint32_t target_phase_increment_u32;
	uint16_t last_electrical_angle_u16;
} motor_openloop_state_t;

/**
 * @brief Speed-estimator runtime state.
 *
 */
typedef struct {
	uint16_t previous_mechanical_angle_u16;
	int32_t accumulated_mechanical_angle_delta;
	uint16_t accumulated_sample_count;
	bool has_previous_mechanical_angle_sample;
} motor_speed_estimator_state_t;

/**
 * @brief Shared motor runtime container.
 *
 */
typedef struct {
	motor_measurements_t measurements;
	motor_targets_t targets;
	motor_limits_t limits;
	motor_status_t status;
	motor_openloop_state_t openloop;
	motor_speed_estimator_state_t speed_estimator;
} motor_handle_t;

#endif /* MOTOR_MOTOR_H */
