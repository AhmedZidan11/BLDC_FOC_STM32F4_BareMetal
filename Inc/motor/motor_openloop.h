#ifndef MOTOR_MOTOR_OPENLOOP_H
#define MOTOR_MOTOR_OPENLOOP_H

/**
 * @file motor_openloop.h
 * @brief Combined open-loop actuation helper above motor_3pwm.
 *
 * This module applies the current open-loop actuation path directly through
 * the 3-PWM motor stage using the shared motor data model.
 *
 * Responsibilities:
 * - bind shared motor-domain state
 * - apply one static electrical vector
 * - advance ramped electrical phase progression
 * - update the 3-PWM output stage directly
 *
 * @note Electrical angle uses full-turn uint16 units.
 * @note Mechanical speed uses signed milli-rpm units.
 * @note Amplitude uses permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"
#include "motor/motor_3pwm.h"

#define MOTOR_OPENLOOP_PHASE_ACCUM_ANGLE_SHIFT             16u
#define MOTOR_OPENLOOP_PHASE_ACCUM_FULL_TURN_U32           (1ULL << 32)
#define MOTOR_OPENLOOP_MS_PER_MINUTE                       60000ULL
#define MOTOR_OPENLOOP_MIN_UPDATES_PER_ELECTRICAL_TURN     16u
#define MOTOR_OPENLOOP_DEFAULT_TEST_POLE_PAIRS             1u

/**
 * @brief Open-loop configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	motor_3pwm_handle_t *motor_3pwm_h;
	uint16_t update_period_ms;            /* Fixed control update period for phase update. */
	uint32_t phase_increment_ramp_step_u32;/* Largest phase-increment change in one update step. */
} motor_openloop_cfg_t;

/**
 * @brief Open-loop runtime handle.
 *
 */
typedef struct {
	const motor_openloop_cfg_t *cfg;
	motor_handle_t *motor_h;
	motor_3pwm_handle_t *motor_3pwm_h;
	bool is_initialized;
} motor_openloop_handle_t;

/**
 * @brief Initialize open-loop handle.
 *
 * This function checks the motor limits, calculates the target phase
 * increment from the mechanical speed, and resets the phase accumulator
 * state.
 *
 * @param motor_openloop_h Pointer to open-loop handle.
 * @param motor_openloop_cfg Pointer to open-loop configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_openloop_init(motor_openloop_handle_t *motor_openloop_h,
						 const motor_openloop_cfg_t *motor_openloop_cfg);

/**
 * @brief Apply one static electrical vector using the current target amplitude.
 *
 * @param motor_openloop_h Pointer to open-loop handle.
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units
 *        (0..65535 <=> 0..1 electrical revolution).
 * @return true if apply succeeded, false otherwise.
 */
bool motor_openloop_apply(motor_openloop_handle_t *motor_openloop_h,
						  uint16_t electrical_angle_u16);

/**
 * @brief Advance electrical angle and apply the updated open-loop output.
 *
 * This function moves the current phase increment toward the target phase
 * increment, advances the shared phase accumulator, and applies the next
 * electrical angle through the 3-PWM stage.
 *
 * @param motor_openloop_h Pointer to open-loop handle.
 * @return true if update succeeded, false otherwise.
 */
bool motor_openloop_update(motor_openloop_handle_t *motor_openloop_h);

#endif /* MOTOR_MOTOR_OPENLOOP_H */
