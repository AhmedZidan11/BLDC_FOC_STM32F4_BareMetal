/**
 * @file motor_6step.c
 * @brief Basic 6-step commutation state machine implementation.
 *
 */

#include "motor/motor_6step.h"
#include <stddef.h>

/**
 * @brief Compute next commutation step based on current step and direction.
 *
 * If current step is outside the valid 6-step range, the sequence restarts
 * from step 1.
 *
 * @param current_step Current commutation step.
 * @param dir Requested sequence direction.
 * @return Next commutation step.
 */
static motor_6step_step_t motor_6step_next_step(motor_6step_step_t current_step,
												motor_6step_dir_t dir)
{
	/* Restart sequence if current step is invalid. */
	if ((current_step < MOTOR_6STEP_STEP_1) || (current_step > MOTOR_6STEP_STEP_6))
	{
		return MOTOR_6STEP_STEP_1;
	}

	if (dir == MOTOR_6STEP_DIR_CW)
	{
		if (current_step == MOTOR_6STEP_STEP_6) return MOTOR_6STEP_STEP_1;
		return (motor_6step_step_t)(current_step + 1);
	}

	if (current_step == MOTOR_6STEP_STEP_1) return MOTOR_6STEP_STEP_6;
	return (motor_6step_step_t)(current_step - 1);
}

bool motor_6step_init(motor_6step_handle_t *motor_h, const motor_6step_cfg_t *cfg)
{
	if ((motor_h == NULL) || (cfg == NULL)) return false;
	if (cfg->step_period_ms == 0u) return false;

	motor_h->cfg = cfg;
	motor_h->state = MOTOR_6STEP_STATE_STOPPED;
	motor_h->current_step = MOTOR_6STEP_STEP_OFF;
	motor_h->last_step_time_ms = 0u;

	return true;
}

void motor_6step_start(motor_6step_handle_t *motor_h, uint32_t now_ms)
{
	if ((motor_h == NULL) || (motor_h->cfg == NULL)) return;

	motor_h->state = MOTOR_6STEP_STATE_RUNNING;
	motor_h->current_step = MOTOR_6STEP_STEP_1;
	motor_h->last_step_time_ms = now_ms;
}

void motor_6step_stop(motor_6step_handle_t *motor_h)
{
	if (motor_h == NULL) return;

	motor_h->state = MOTOR_6STEP_STATE_STOPPED;
	motor_h->current_step = MOTOR_6STEP_STEP_OFF;
}

bool motor_6step_update(motor_6step_handle_t *motor_h, uint32_t now_ms)
{
	if ((motor_h == NULL) || (motor_h->cfg == NULL)) return false;
	if (motor_h->state != MOTOR_6STEP_STATE_RUNNING) return false;

	if ((now_ms - motor_h->last_step_time_ms) < motor_h->cfg->step_period_ms)
	{
		return false;
	}

	motor_h->last_step_time_ms = now_ms;
	motor_h->current_step = motor_6step_next_step(motor_h->current_step, motor_h->cfg->dir);

	return true;
}

motor_6step_step_t motor_6step_get_step(const motor_6step_handle_t *motor_h)
{
	if (motor_h == NULL) return MOTOR_6STEP_STEP_OFF;
	return motor_h->current_step;
}

motor_6step_dir_t motor_6step_get_dir(const motor_6step_handle_t *motor_h)
{
	if ((motor_h == NULL) || (motor_h->cfg == NULL))
	{
		return MOTOR_6STEP_DIR_CW;
	}

	return motor_h->cfg->dir;
}

motor_6step_state_t motor_6step_get_state(const motor_6step_handle_t *motor_h)
{
	if (motor_h == NULL)
	{
		return MOTOR_6STEP_STATE_STOPPED;
	}

	return motor_h->state;
}
