#ifndef MOTOR_MOTOR_DRIVER_H
#define MOTOR_MOTOR_DRIVER_H

/**
 * @file motor_driver.h
 * @brief Bridge layer between motor commutation logic and PWM hardware driver.
 *
 * This module converts a logical 3-phase commutation map into per-phase
 * commands and applies them to the TIM1 PWM driver.
 *
 * Responsibilities:
 * - accept phase commutation state from motor_6step
 * - convert phase states to driver commands
 * - apply per-phase PWM duty values
 * - store the last applied command set
 *
 * @note In the current 3-PWM wiring, phase FLOAT is approximated with 0% duty.
 *       True per-phase Hi-Z is not available through this interface.
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_6step.h"
#include "drivers/pwm_tim1.h"

typedef enum {
	MOTOR_DRIVER_CMD_FLOAT = 0,
	MOTOR_DRIVER_CMD_HIGH  = 1,
	MOTOR_DRIVER_CMD_LOW   = 2
} motor_driver_cmd_t;

/**
 * @brief Per-phase drive command set.
 *
 */
typedef struct {
	motor_driver_cmd_t phase_a;
	motor_driver_cmd_t phase_b;
	motor_driver_cmd_t phase_c;
} motor_driver_cmd_map_t;

/**
 * @brief Motor driver configuration.
 *
 */
typedef struct {
	pwm_tim1_handle_t *pwm_h;
	uint16_t pwm_duty_permyriad;
} motor_driver_cfg_t;

/**
 * @brief Motor driver runtime handle.
 *
 */
typedef struct {
	const motor_driver_cfg_t *cfg;
	motor_driver_cmd_map_t last_cmd_map;
	bool is_initialized;
	bool is_started;
} motor_driver_handle_t;

/**
 * @brief Initialize motor driver handle.
 *
 * @param motor_driver_h Pointer to motor driver handle.
 * @param motor_driver_cfg Pointer to motor driver configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_driver_init(motor_driver_handle_t *motor_driver_h,
					   const motor_driver_cfg_t *motor_driver_cfg);

/**
 * @brief Start PWM output stage.
 *
 * @param motor_driver_h Pointer to motor driver handle.
 * @return true if start succeeded, false otherwise.
 */
bool motor_driver_start(motor_driver_handle_t *motor_driver_h);

/**
 * @brief Stop PWM output stage and clear phase duties.
 *
 * @param motor_driver_h Pointer to motor driver handle.
 * @return true if stop succeeded, false otherwise.
 */
bool motor_driver_stop(motor_driver_handle_t *motor_driver_h);

/**
 * @brief Apply a logical phase map to the motor driver layer.
 *
 * This function converts the given 6-step phase map into abstract drive
 * commands, updates PWM compare values and stores the result in the runtime
 * handle.
 *
 * @param motor_driver_h Pointer to motor driver handle.
 * @param phase_map Pointer to logical phase map.
 * @return true if apply succeeded, false otherwise.
 */
bool motor_driver_apply_phase_map(motor_driver_handle_t *motor_driver_h,
								  const motor_6step_phase_map_t *phase_map);

/**
 * @brief Get last applied drive command map.
 *
 * @param motor_driver_h Pointer to motor driver handle.
 * @return Last stored command map. Invalid handle returns all FLOAT.
 */
motor_driver_cmd_map_t motor_driver_get_cmd_map(const motor_driver_handle_t *motor_driver_h);

#endif /* MOTOR_MOTOR_DRIVER_H */
