#ifndef MOTOR_MOTOR_OPENLOOP_SINE_H
#define MOTOR_MOTOR_OPENLOOP_SINE_H

/**
 * @file motor_openloop_sine.h
 * @brief Open-loop sine control layer above motor_3pwm.
 *
 * This module applies a static electrical vector command through the 3-PWM
 * motor stage.
 *
 * Responsibilities:
 * - store open-loop sine configuration
 * - accept electrical angle and amplitude commands
 * - store the last applied static vector command
 *
 * @note motor_openloop.h is the preferred merged public open-loop interface.
 *       This split helper remains temporarily for migration clarity.
 * @note Electrical angle uses full-turn uint16 units
 *       (0..65535 <=> 0..1 electrical revolution).
 * @note Amplitude inputs use permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_3pwm.h"

/**
 * @brief Open-loop sine configuration.
 *
 */
typedef struct {
	motor_3pwm_handle_t *motor_3pwm_h;
} motor_openloop_sine_cfg_t;

/**
 * @brief Open-loop sine runtime handle.
 *
 */
typedef struct {
	const motor_openloop_sine_cfg_t *cfg;
	uint16_t last_electrical_angle_u16;
	uint16_t last_amplitude_permyriad;
	bool is_initialized;
} motor_openloop_sine_handle_t;

/**
 * @brief Initialize open-loop sine handle.
 *
 * @param motor_openloop_sine_h Pointer to open-loop sine handle.
 * @param motor_openloop_sine_cfg Pointer to open-loop sine configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_openloop_sine_init(motor_openloop_sine_handle_t *motor_openloop_sine_h,
							  const motor_openloop_sine_cfg_t *motor_openloop_sine_cfg);

/**
 * @brief Apply one static electrical vector command.
 *
 * @param motor_openloop_sine_h Pointer to open-loop sine handle.
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units
 *        (0..65535 <=> 0..1 electrical revolution).
 * @param amplitude_permyriad Vector amplitude in permyriad units (0..10000).
 * @return true if apply succeeded, false otherwise.
 */
bool motor_openloop_sine_apply(motor_openloop_sine_handle_t *motor_openloop_sine_h,
						   uint16_t electrical_angle_u16,
						   uint16_t amplitude_permyriad);

#endif /* MOTOR_MOTOR_OPENLOOP_SINE_H */
