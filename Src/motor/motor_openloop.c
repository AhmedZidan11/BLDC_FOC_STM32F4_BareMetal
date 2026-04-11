/**
 * @file motor_openloop.c
 * @brief Combined open-loop actuation helper above motor_3pwm.
 *
 */

#include "motor/motor_openloop.h"
#include "motor/motor_sine_3pwm.h"
#include <stddef.h>

#define MOTOR_OPENLOOP_MRPM_PER_RPM             1000LL

/**
 * @brief Apply one electrical angle through the 3-PWM stage.
 *
 * @param motor_openloop_h Pointer to open-loop handle.
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @return true if the three duty commands were applied successfully.
 */
static bool motor_openloop_apply_electrical_angle(motor_openloop_handle_t *motor_openloop_h,
												  uint16_t electrical_angle_u16)
{
	if ((motor_openloop_h == NULL) || (motor_openloop_h->motor_h == NULL)) return false;
	if (motor_openloop_h->motor_3pwm_h == NULL) return false;
	if (motor_openloop_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_openloop_h->motor_h;
	uint16_t amplitude_permyriad = motor_h->targets.target_amplitude_permyriad;
	if (amplitude_permyriad > motor_h->limits.max_amplitude_permyriad) return false;

	/* Apply shared sine 3-phase modulation for the requested angle and amplitude. */
	if (!motor_sine_3pwm_apply(motor_openloop_h->motor_3pwm_h,
							   electrical_angle_u16,
							   amplitude_permyriad))
	{
		return false;
	}

	/* Store only the last commanded electrical angle for the open-loop path. */
	motor_h->openloop.last_electrical_angle_u16 = electrical_angle_u16;
	return true;
}

bool motor_openloop_init(motor_openloop_handle_t *motor_openloop_h,
						 const motor_openloop_cfg_t *motor_openloop_cfg)
{
	if ((motor_openloop_h == NULL) || (motor_openloop_cfg == NULL)) return false;
	if (motor_openloop_cfg->motor_h == NULL) return false;
	if (motor_openloop_cfg->motor_3pwm_h == NULL) return false;
	if (motor_openloop_cfg->update_period_ms == 0u) return false;
	if (motor_openloop_cfg->phase_increment_ramp_step_u32 == 0u) return false;

	motor_handle_t *motor_h = motor_openloop_cfg->motor_h;
	if (motor_h->limits.pole_pairs == 0u)
	{
		motor_h->limits.pole_pairs = MOTOR_OPENLOOP_DEFAULT_TEST_POLE_PAIRS;
	}
	if (motor_h->limits.max_amplitude_permyriad == 0u)
	{
		motor_h->limits.max_amplitude_permyriad = MOTOR_SINE_3PWM_MAX_AMPLITUDE_PERMYRIAD;
	}
	if (motor_h->limits.max_amplitude_permyriad > MOTOR_SINE_3PWM_MAX_AMPLITUDE_PERMYRIAD) return false;
	if (motor_h->targets.target_amplitude_permyriad > motor_h->limits.max_amplitude_permyriad) return false;
	if (motor_h->targets.target_mechanical_speed_mrpm < 0) return false;

	/* max_speed = 60000 * 1000 / (pole_pairs * update_period_ms * min_updates_per_turn). */
	uint64_t max_speed_denom = (uint64_t)motor_h->limits.pole_pairs *
							   (uint64_t)motor_openloop_cfg->update_period_ms *
							   (uint64_t)MOTOR_OPENLOOP_MIN_UPDATES_PER_ELECTRICAL_TURN;
	motor_h->limits.max_target_mechanical_speed_mrpm =
			(int32_t)((MOTOR_OPENLOOP_MS_PER_MINUTE * MOTOR_OPENLOOP_MRPM_PER_RPM) / max_speed_denom);
	if (motor_h->targets.target_mechanical_speed_mrpm > motor_h->limits.max_target_mechanical_speed_mrpm) return false;

	/* phase_increment = speed_mrpm * pole_pairs * dt_ms * 2^32 / (60000 * 1000). */
	uint64_t phase_increment_num = (uint64_t)motor_h->targets.target_mechanical_speed_mrpm *
								   (uint64_t)motor_h->limits.pole_pairs *
								   (uint64_t)motor_openloop_cfg->update_period_ms *
								   MOTOR_OPENLOOP_PHASE_ACCUM_FULL_TURN_U32;
	uint32_t target_phase_increment_u32 = (uint32_t)((phase_increment_num +
													 ((MOTOR_OPENLOOP_MS_PER_MINUTE * MOTOR_OPENLOOP_MRPM_PER_RPM) / 2u)) /
													 (MOTOR_OPENLOOP_MS_PER_MINUTE * MOTOR_OPENLOOP_MRPM_PER_RPM));

	motor_openloop_h->cfg = motor_openloop_cfg;
	motor_openloop_h->motor_h = motor_h;
	motor_openloop_h->motor_3pwm_h = motor_openloop_cfg->motor_3pwm_h;
	motor_h->openloop.phase_accumulator_u32 = 0u;
	motor_h->openloop.current_phase_increment_u32 = 0u;
	motor_h->openloop.target_phase_increment_u32 = target_phase_increment_u32;
	motor_h->openloop.last_electrical_angle_u16 = 0u;
	motor_h->measurements.electrical_angle_u16 = 0u;
	motor_h->status.mode = MOTOR_MODE_INACTIVE;
	motor_h->status.is_initialized = true;
	motor_h->status.has_valid_electrical_angle = false;
	motor_openloop_h->is_initialized = true;

	return true;
}

bool motor_openloop_apply(motor_openloop_handle_t *motor_openloop_h,
						  uint16_t electrical_angle_u16)
{
	if (!motor_openloop_apply_electrical_angle(motor_openloop_h, electrical_angle_u16)) return false;

	motor_handle_t *motor_h = motor_openloop_h->motor_h;
	/* Put the applied electrical angle into the shared 32-bit phase accumulator. */
	motor_h->openloop.phase_accumulator_u32 =
			((uint32_t)electrical_angle_u16 << MOTOR_OPENLOOP_PHASE_ACCUM_ANGLE_SHIFT);
	motor_h->status.mode = MOTOR_MODE_OPENLOOP_STATIC;
	return true;
}

bool motor_openloop_update(motor_openloop_handle_t *motor_openloop_h)
{
	if (motor_openloop_h == NULL) return false;
	if ((motor_openloop_h->cfg == NULL) || (motor_openloop_h->motor_h == NULL)) return false;
	if (motor_openloop_h->motor_3pwm_h == NULL) return false;
	if (motor_openloop_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_openloop_h->motor_h;

	/* Move the active phase increment toward the target increment. */
	if (motor_h->openloop.current_phase_increment_u32 < motor_h->openloop.target_phase_increment_u32)
	{
		uint32_t increment_delta = motor_h->openloop.target_phase_increment_u32 -
								   motor_h->openloop.current_phase_increment_u32;
		if (increment_delta > motor_openloop_h->cfg->phase_increment_ramp_step_u32)
		{
			increment_delta = motor_openloop_h->cfg->phase_increment_ramp_step_u32;
		}

		motor_h->openloop.current_phase_increment_u32 += increment_delta;
	}
	else if (motor_h->openloop.current_phase_increment_u32 > motor_h->openloop.target_phase_increment_u32)
	{
		uint32_t increment_delta = motor_h->openloop.current_phase_increment_u32 -
								   motor_h->openloop.target_phase_increment_u32;
		if (increment_delta > motor_openloop_h->cfg->phase_increment_ramp_step_u32)
		{
			increment_delta = motor_openloop_h->cfg->phase_increment_ramp_step_u32;
		}

		motor_h->openloop.current_phase_increment_u32 -= increment_delta;
	}

	/* Advance the phase accumulator and use the upper 16 bits as electrical angle. */
	uint32_t next_phase_accumulator_u32 = motor_h->openloop.phase_accumulator_u32 +
										  motor_h->openloop.current_phase_increment_u32;
	uint16_t next_electrical_angle_u16 = (uint16_t)(next_phase_accumulator_u32 >> MOTOR_OPENLOOP_PHASE_ACCUM_ANGLE_SHIFT);

	if (!motor_openloop_apply_electrical_angle(motor_openloop_h, next_electrical_angle_u16)) return false;

	motor_h->openloop.phase_accumulator_u32 = next_phase_accumulator_u32;
	motor_h->status.mode = MOTOR_MODE_OPENLOOP_DRIVE;
	return true;
}
