#ifndef MOTOR_MOTOR_SINE_3PWM_H
#define MOTOR_MOTOR_SINE_3PWM_H

/**
 * @file motor_sine_3pwm.h
 * @brief Shared sine modulation helper for motor-domain 3-PWM actuation.
 *
 * This helper is shared by open-loop and voltage-FOC paths. It converts one
 * electrical angle and one amplitude request into three sine phase duties and
 * applies them through the 3-PWM stage.
 *
 * @note Electrical angle uses full-turn uint16 units.
 * @note Amplitude uses permyriad units (0..10000).
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_3pwm.h"

#define MOTOR_SINE_3PWM_MAX_AMPLITUDE_PERMYRIAD 10000u

/**
 * @brief Apply sine-modulated 3-phase duties for one electrical angle.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @param amplitude_permyriad Requested amplitude in permyriad units.
 * @return true if duty synthesis and apply succeeded, false otherwise.
 */
bool motor_sine_3pwm_apply(motor_3pwm_handle_t *motor_3pwm_h,
						   uint16_t electrical_angle_u16,
						   uint16_t amplitude_permyriad);

/**
 * @brief Get one sine sample in Q15-like units from one electrical angle.
 *
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @return Signed Q15-like sine sample in the range [-32767, 32767].
 */
int16_t motor_sine_3pwm_get_phase_q15(uint16_t electrical_angle_u16);

/**
 * @brief Apply three signed normalized phase commands directly.
 *
 * Each phase command uses Q15-like scaling in the range [-32767, 32767].
 * The helper converts the three commands into centered PWM duties and applies
 * them to the 3-PWM stage.
 *
 * @param motor_3pwm_h Pointer to 3-PWM motor stage handle.
 * @param phase_a_q15 Phase A command in Q15-like units.
 * @param phase_b_q15 Phase B command in Q15-like units.
 * @param phase_c_q15 Phase C command in Q15-like units.
 * @return true if duty synthesis and apply succeeded, false otherwise.
 */
bool motor_sine_3pwm_apply_phase_q15(motor_3pwm_handle_t *motor_3pwm_h,
									 int16_t phase_a_q15,
									 int16_t phase_b_q15,
									 int16_t phase_c_q15);

#endif /* MOTOR_MOTOR_SINE_3PWM_H */
