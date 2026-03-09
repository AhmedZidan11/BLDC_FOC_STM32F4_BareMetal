#ifndef MOTOR_MOTOR_6STEP_H
#define MOTOR_MOTOR_6STEP_H

/**
 * @file motor_6step.h
 * @brief Basic 6-step commutation state machine.
 *
 * This module provides a software-only 6-step sequence.
 * It does not drive PWM outputs yet.
 *
 * Responsibilities:
 * - store 6-step state machine configuration
 * - track current commutation step
 * - advance step based on elapsed time
 * - provide current phase commutation map
 */

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	MOTOR_6STEP_DIR_CW  = 0,
	MOTOR_6STEP_DIR_CCW = 1
} motor_6step_dir_t;

typedef enum {
	MOTOR_6STEP_STATE_STOPPED = 0,
	MOTOR_6STEP_STATE_RUNNING = 1
} motor_6step_state_t;

typedef enum {
	MOTOR_6STEP_STEP_OFF = 0,
	MOTOR_6STEP_STEP_1   = 1,
	MOTOR_6STEP_STEP_2   = 2,
	MOTOR_6STEP_STEP_3   = 3,
	MOTOR_6STEP_STEP_4   = 4,
	MOTOR_6STEP_STEP_5   = 5,
	MOTOR_6STEP_STEP_6   = 6
} motor_6step_step_t;

typedef enum {
	MOTOR_6STEP_PHASE_FLOAT = 0,
	MOTOR_6STEP_PHASE_HIGH  = 1,
	MOTOR_6STEP_PHASE_LOW   = 2
} motor_6step_phase_state_t;

/**
 * @brief Commutation state of motor phases.
 *
 */
typedef struct {
	motor_6step_phase_state_t phase_a;
	motor_6step_phase_state_t phase_b;
	motor_6step_phase_state_t phase_c;
} motor_6step_phase_map_t;

/**
 * @brief 6-step motor configuration.
 *
 */
typedef struct {
	uint32_t step_period_ms;
	motor_6step_dir_t dir;
} motor_6step_cfg_t;

/**
 * @brief 6-step motor runtime handle.
 *
 */
typedef struct {
	const motor_6step_cfg_t *cfg;
	motor_6step_state_t state;
	motor_6step_step_t current_step;
	uint32_t last_step_time_ms;
} motor_6step_handle_t;

/**
 * @brief Initialize 6-step handle.
 *
 * @param motor_h Pointer to motor handle.
 * @param motor_cfg Pointer to motor configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_6step_init(motor_6step_handle_t *motor_h, const motor_6step_cfg_t *motor_cfg);

/**
 * @brief Start 6-step sequence from step 1.
 *
 * @param motor_h Pointer to motor handle.
 * @param now_ms Current system time in milliseconds.
 */
void motor_6step_start(motor_6step_handle_t *motor_h, uint32_t now_ms);

/**
 * @brief Stop 6-step sequence.
 *
 * @param motor_h Pointer to motor handle.
 */
void motor_6step_stop(motor_6step_handle_t *motor_h);

/**
 * @brief Update 6-step state machine.
 *
 * Advances one commutation step if the configured period elapsed.
 *
 * @param motor_h Pointer to motor handle.
 * @param now_ms Current system time in milliseconds.
 * @return true if step changed, false otherwise.
 */
bool motor_6step_update(motor_6step_handle_t *motor_h, uint32_t now_ms);

/**
 * @brief Get current commutation step.
 *
 * @param motor_h Pointer to motor handle.
 * @return Current step, or MOTOR_6STEP_STEP_OFF if handle is invalid.
 */
motor_6step_step_t motor_6step_get_step(const motor_6step_handle_t *motor_h);

/**
 * @brief Get current direction.
 *
 * @param motor_h Pointer to motor handle.
 * @return Configured direction.
 */
motor_6step_dir_t motor_6step_get_dir(const motor_6step_handle_t *motor_h);

/**
 * @brief Get current state.
 *
 * @param motor_h Pointer to motor handle.
 * @return Current run state.
 */
motor_6step_state_t motor_6step_get_state(const motor_6step_handle_t *motor_h);

/**
 * @brief Get current phase commutation state.
 *
 * @param motor_h Pointer to motor handle.
 * @return Current phase map. Invalid/stopped state returns all phases FLOAT.
 */
motor_6step_phase_map_t motor_6step_get_phase_map(const motor_6step_handle_t *motor_h);

#endif /* MOTOR_MOTOR_6STEP_H */
