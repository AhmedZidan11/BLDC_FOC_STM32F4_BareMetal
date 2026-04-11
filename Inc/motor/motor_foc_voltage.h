#ifndef MOTOR_MOTOR_FOC_VOLTAGE_H
#define MOTOR_MOTOR_FOC_VOLTAGE_H

/**
 * @file motor_foc_voltage.h
 * @brief Voltage-mode FOC actuation helper.
 *
 * This module applies rotor-frame d/q voltage requests through the 3-PWM stage.
 *
 * Responsibilities:
 * - bind shared motor-domain and 3-PWM handles
 * - read measured electrical angle from shared motor measurements
 * - apply d/q voltage requests in permyriad units
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
	int8_t phase_sequence_sign; /* +1 => q-axis +90 deg, -1 => q-axis -90 deg. */
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
 * @brief Apply d/q voltage request in rotor frame.
 *
 * This function reads measured electrical angle and converts the rotor-frame
 * voltage request (Ud, Uq) into the equivalent stationary-frame actuation.
 * The configured phase-sequence sign still controls q-axis handedness.
 *
 * @param motor_foc_voltage_h Pointer to FOC voltage handle.
 * @param ud_permyriad D-axis voltage request in permyriad units.
 * @param uq_permyriad Q-axis voltage request in permyriad units.
 * @return true if actuation succeeded, false otherwise.
 */
bool motor_foc_voltage_apply_dq(motor_foc_voltage_handle_t *motor_foc_voltage_h,
								int16_t ud_permyriad,
								int16_t uq_permyriad);

/**
 * @brief Apply q-axis-only voltage request with Ud fixed to zero.
 *
 * This helper wraps motor_foc_voltage_apply_dq(..., 0, uq_permyriad).
 *
 * @param motor_foc_voltage_h Pointer to FOC voltage handle.
 * @param uq_permyriad Q-axis voltage request in permyriad units.
 * @return true if actuation succeeded, false otherwise.
 */
bool motor_foc_voltage_apply_q_only(motor_foc_voltage_handle_t *motor_foc_voltage_h,
									uint16_t uq_permyriad);

#endif /* MOTOR_MOTOR_FOC_VOLTAGE_H */
