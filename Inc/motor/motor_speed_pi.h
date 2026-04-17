#ifndef MOTOR_MOTOR_SPEED_PI_H
#define MOTOR_MOTOR_SPEED_PI_H

/**
 * @file motor_speed_pi.h
 * @brief Digital PI controller for mechanical speed.
 *
 * This module computes the active q-axis command from target and measured
 * mechanical speed in the closed-loop speed-control path.
 *
 * Responsibilities:
 * - store PI configuration and runtime state
 * - compute one discrete PI update at fixed cadence
 * - apply output saturation and simple anti-windup
 * - publish one speed-control q-axis command
 *
 * @note Speed uses signed milli-rpm units.
 * @note q-axis command uses signed permyriad units.
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"

/**
 * @brief Speed PI configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	int32_t kp_q15;                    /* Proportional gain in Q15 (permyriad per mrpm). */
	int32_t ki_per_s_q15;              /* Integral gain in Q15 per second (permyriad per mrpm per second). */
	uint16_t update_period_ms;
	uint16_t output_limit_permyriad;
} motor_speed_pi_cfg_t;

/**
 * @brief Speed PI runtime handle.
 *
 */
typedef struct {
	const motor_speed_pi_cfg_t *cfg;
	motor_handle_t *motor_h;
	int32_t ki_dt_q15;
	int32_t integrator_term_permyriad;
	bool is_initialized;
} motor_speed_pi_handle_t;

/**
 * @brief Initialize speed PI handle.
 *
 * @param motor_speed_pi_h Pointer to speed PI handle.
 * @param motor_speed_pi_cfg Pointer to speed PI configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_speed_pi_init(motor_speed_pi_handle_t *motor_speed_pi_h,
						 const motor_speed_pi_cfg_t *motor_speed_pi_cfg);

/**
 * @brief Reset speed PI integrator and output state.
 *
 * @param motor_speed_pi_h Pointer to speed PI handle.
 * @return true if reset succeeded, false otherwise.
 */
bool motor_speed_pi_reset(motor_speed_pi_handle_t *motor_speed_pi_h);

/**
 * @brief Run one discrete speed PI update.
 *
 * @param motor_speed_pi_h Pointer to speed PI handle.
 * @param target_mechanical_speed_mrpm Target mechanical speed in mrpm.
 * @param measured_mechanical_speed_mrpm Measured mechanical speed in mrpm.
 * @return true if update succeeded, false otherwise.
 */
bool motor_speed_pi_update(motor_speed_pi_handle_t *motor_speed_pi_h,
						   int32_t target_mechanical_speed_mrpm,
						   int32_t measured_mechanical_speed_mrpm);

#endif /* MOTOR_MOTOR_SPEED_PI_H */
