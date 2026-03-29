#ifndef MOTOR_MOTOR_OPENLOOP_DRIVE_H
#define MOTOR_MOTOR_OPENLOOP_DRIVE_H

/**
 * @file motor_openloop_drive.h
 * @brief Small stateful open-loop drive layer above motor_openloop_sine.
 *
 * This module tracks open-loop electrical phase progression and applies the
 * updated sine vector through the open-loop sine layer.
 *
 * Responsibilities:
 * - store open-loop drive configuration
 * - store phase accumulator state
 * - store phase increment state
 * - apply the updated electrical angle through motor_openloop_sine
 *
 * @note Electrical angle uses full-turn uint16 units.
 * @note Amplitude inputs use permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_openloop_sine.h"

#define MOTOR_OPENLOOP_DRIVE_PHASE_ACCUM_ANGLE_SHIFT  16u
#define MOTOR_OPENLOOP_DRIVE_DEFAULT_TEST_POLE_PAIRS  1u

/**
 * @brief Open-loop drive configuration.
 *
 */
typedef struct {
	motor_openloop_sine_handle_t *motor_openloop_sine_h;
	uint16_t amplitude_permyriad;
	uint16_t target_mechanical_speed_rpm;
	uint16_t update_period_ms;
} motor_openloop_drive_cfg_t;

/**
 * @brief Open-loop drive runtime handle.
 *
 */
typedef struct {
	const motor_openloop_drive_cfg_t *cfg;
	uint32_t phase_accumulator_u32;
	uint32_t phase_increment_u32;
	uint16_t last_electrical_angle_u16;
	bool is_initialized;
} motor_openloop_drive_handle_t;

/**
 * @brief Initialize open-loop drive handle.
 *
 * @param motor_openloop_drive_h Pointer to open-loop drive handle.
 * @param motor_openloop_drive_cfg Pointer to open-loop drive configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_openloop_drive_init(motor_openloop_drive_handle_t *motor_openloop_drive_h,
							   const motor_openloop_drive_cfg_t *motor_openloop_drive_cfg);

/**
 * @brief Advance electrical angle and apply the updated sine vector.
 *
 * @param motor_openloop_drive_h Pointer to open-loop drive handle.
 * @return true if update succeeded, false otherwise.
 */
bool motor_openloop_drive_update(motor_openloop_drive_handle_t *motor_openloop_drive_h);

#endif /* MOTOR_MOTOR_OPENLOOP_DRIVE_H */
