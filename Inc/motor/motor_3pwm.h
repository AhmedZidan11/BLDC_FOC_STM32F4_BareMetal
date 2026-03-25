#ifndef MOTOR_MOTOR_3PWM_H
#define MOTOR_MOTOR_3PWM_H

/**
 * @file motor_3pwm.h
 * @brief Narrow 3-PWM power-stage abstraction for TIM1.
 *
 * This module applies explicit per-phase duty values to the existing TIM1 PWM
 * driver.
 *
 * Responsibilities:
 * - store 3-PWM stage configuration
 * - start and stop the PWM output stage
 * - apply per-phase PWM duty values
 * - store the last applied duty values
 *
 * @note Duty inputs use permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/pwm_tim1.h"

/**
 * @brief 3-PWM motor stage configuration.
 *
 */
typedef struct {
	pwm_tim1_handle_t *pwm_h;
} motor_3pwm_cfg_t;

/**
 * @brief 3-PWM motor stage runtime handle.
 *
 */
typedef struct {
	const motor_3pwm_cfg_t *cfg;
	uint16_t phase_a_duty_permyriad;
	uint16_t phase_b_duty_permyriad;
	uint16_t phase_c_duty_permyriad;
	bool is_initialized;
	bool is_started;
} motor_3pwm_handle_t;

/**
 * @brief Initialize 3-PWM motor stage handle.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @param motor_3pwm_cfg Pointer to 3-PWM motor stage configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_3pwm_init(motor_3pwm_handle_t *motor_3pwm_h,
					 const motor_3pwm_cfg_t *motor_3pwm_cfg);

/**
 * @brief Start PWM output stage.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @return true if start succeeded, false otherwise.
 */
bool motor_3pwm_start(motor_3pwm_handle_t *motor_3pwm_h);

/**
 * @brief Stop PWM output stage and clear phase duties.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @return true if stop succeeded, false otherwise.
 */
bool motor_3pwm_stop(motor_3pwm_handle_t *motor_3pwm_h);

/**
 * @brief Apply per-phase duty values to the 3-PWM output stage.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @param phase_a_duty_permyriad Phase A duty in permyriad units (0..10000).
 * @param phase_b_duty_permyriad Phase B duty in permyriad units (0..10000).
 * @param phase_c_duty_permyriad Phase C duty in permyriad units (0..10000).
 * @return true if duty update succeeded, false otherwise.
 */
bool motor_3pwm_set_duty_abc(motor_3pwm_handle_t *motor_3pwm_h,
							 uint16_t phase_a_duty_permyriad,
							 uint16_t phase_b_duty_permyriad,
							 uint16_t phase_c_duty_permyriad);

/**
 * @brief Apply neutral output state to all three phases.
 *
 * Neutral is represented as 0% duty on phases A, B and C.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @return true if neutral state update succeeded, false otherwise.
 */
bool motor_3pwm_set_neutral(motor_3pwm_handle_t *motor_3pwm_h);

#endif /* MOTOR_MOTOR_3PWM_H */
