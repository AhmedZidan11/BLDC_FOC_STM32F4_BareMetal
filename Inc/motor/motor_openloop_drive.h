#ifndef MOTOR_MOTOR_OPENLOOP_DRIVE_H
#define MOTOR_MOTOR_OPENLOOP_DRIVE_H

/**
 * @file motor_openloop_drive.h
 * @brief Small stateful open-loop drive layer above motor_openloop_sine.
 *
 * This module advances a stored electrical angle state and applies the updated
 * static sine vector through the open-loop sine layer.
 *
 * Responsibilities:
 * - store open-loop drive configuration
 * - track current electrical angle state
 * - advance electrical angle by a fixed step per update
 * - apply the updated vector through motor_openloop_sine
 *
 * @note Electrical angle uses full-turn uint16 units
 *       (0..65535 <=> 0..1 electrical revolution).
 * @note Amplitude inputs use permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_openloop_sine.h"

/**
 * @brief Open-loop drive configuration.
 *
 */
typedef struct {
	motor_openloop_sine_handle_t *motor_openloop_sine_h;
	uint16_t amplitude_permyriad;
	uint16_t electrical_angle_step_u16;
} motor_openloop_drive_cfg_t;

/**
 * @brief Open-loop drive runtime handle.
 *
 */
typedef struct {
	const motor_openloop_drive_cfg_t *cfg;
	uint16_t current_electrical_angle_u16;
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
