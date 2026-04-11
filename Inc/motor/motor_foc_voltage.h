#ifndef MOTOR_MOTOR_FOC_VOLTAGE_H
#define MOTOR_MOTOR_FOC_VOLTAGE_H

/**
 * @file motor_foc_voltage.h
 * @brief Q-axis-only voltage-mode FOC actuation kernel.
 *
 * This module applies one q-axis-only voltage vector through the 3-PWM stage.
 * This first step is limited to the special case Ud = 0.
 *
 * Responsibilities:
 * - bind shared motor-domain and 3-PWM handles
 * - read measured electrical angle from shared motor measurements
 * - apply q-axis-only voltage request in permyriad units
 *
 * @note Electrical angle uses full-turn uint16 units.
 * @note Amplitude uses permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"
#include "motor/motor_3pwm.h"

/**
 * @brief FOC voltage helper configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	motor_3pwm_handle_t *motor_3pwm_h;
} motor_foc_voltage_cfg_t;

/**
 * @brief FOC voltage helper runtime handle.
 *
 */
typedef struct {
	const motor_foc_voltage_cfg_t *cfg;
	motor_handle_t *motor_h;
	motor_3pwm_handle_t *motor_3pwm_h;
	bool is_initialized;
} motor_foc_voltage_handle_t;

/**
 * @brief Initialize FOC voltage helper.
 *
 * @param motor_foc_voltage_h Pointer to FOC voltage handle.
 * @param motor_foc_voltage_cfg Pointer to FOC voltage configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_foc_voltage_init(motor_foc_voltage_handle_t *motor_foc_voltage_h,
							const motor_foc_voltage_cfg_t *motor_foc_voltage_cfg);

/**
 * @brief Apply q-axis-only voltage request with Ud fixed to zero.
 *
 * This function reads the measured electrical angle from shared motor state,
 * shifts it by +90 electrical degrees for q-axis-only actuation, and applies
 * the resulting three phase duties through motor_3pwm.
 *
 * @param motor_foc_voltage_h Pointer to FOC voltage handle.
 * @param uq_permyriad Q-axis voltage request in permyriad units.
 * @return true if actuation succeeded, false otherwise.
 */
bool motor_foc_voltage_apply_q_only(motor_foc_voltage_handle_t *motor_foc_voltage_h,
									uint16_t uq_permyriad);

#endif /* MOTOR_MOTOR_FOC_VOLTAGE_H */
