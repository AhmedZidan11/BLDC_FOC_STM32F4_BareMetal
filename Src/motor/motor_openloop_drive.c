/**
 * @file motor_openloop_drive.c
 * @brief Small stateful open-loop drive layer above motor_openloop_sine.
 *
 */

#include "motor/motor_openloop_drive.h"
#include <stddef.h>

#define MOTOR_OPENLOOP_DRIVE_MRPM_PER_RPM                  1000LL
#define MOTOR_OPENLOOP_DRIVE_MAX_AMPLITUDE_PERMYRIAD       10000u

/*
 * Open-loop speed model:
 *
 * mechanical_speed_mrpm [mrev_m/min]
 * electrical_speed_mrpm = mechanical_speed_mrpm * pole_pairs [mrev_e/min]
 * electrical_turns_per_update =
 *     electrical_speed_mrpm * update_period_ms / (60000 * 1000) [turn_e/update]
 *
 * The phase accumulator spans one full electrical turn over 2^32 counts, so:
 *
 * phase_increment_u32 =
 *     electrical_turns_per_update * 2^32 [counts/update]
 */
bool motor_openloop_drive_init(motor_openloop_drive_handle_t *motor_openloop_drive_h,
							   const motor_openloop_drive_cfg_t *motor_openloop_drive_cfg)
{
	if ((motor_openloop_drive_h == NULL) || (motor_openloop_drive_cfg == NULL)) return false;
	if (motor_openloop_drive_cfg->motor_h == NULL) return false;
	if (motor_openloop_drive_cfg->motor_openloop_sine_h == NULL) return false;
	if (motor_openloop_drive_cfg->update_period_ms == 0u) return false;
	if (motor_openloop_drive_cfg->phase_increment_ramp_step_u32 == 0u) return false;

	motor_handle_t *motor_h = motor_openloop_drive_cfg->motor_h;
	if (motor_h->limits.pole_pairs == 0u)
	{
		motor_h->limits.pole_pairs = MOTOR_OPENLOOP_DRIVE_DEFAULT_TEST_POLE_PAIRS;
	}
	if (motor_h->limits.max_amplitude_permyriad == 0u)
	{
		motor_h->limits.max_amplitude_permyriad = MOTOR_OPENLOOP_DRIVE_MAX_AMPLITUDE_PERMYRIAD;
	}
	if (motor_h->targets.target_amplitude_permyriad > motor_h->limits.max_amplitude_permyriad) return false;
	if (motor_h->targets.target_mechanical_speed_mrpm < 0) return false;

	uint64_t max_speed_denom = (uint64_t)motor_h->limits.pole_pairs *
							   (uint64_t)motor_openloop_drive_cfg->update_period_ms *
							   (uint64_t)MOTOR_OPENLOOP_DRIVE_MIN_UPDATES_PER_ELECTRICAL_TURN;
	motor_h->limits.max_target_mechanical_speed_mrpm =
			(int32_t)((MOTOR_OPENLOOP_DRIVE_MS_PER_MINUTE * MOTOR_OPENLOOP_DRIVE_MRPM_PER_RPM) / max_speed_denom);
	if (motor_h->targets.target_mechanical_speed_mrpm > motor_h->limits.max_target_mechanical_speed_mrpm) return false;

	/* Convert mechanical speed and update period into electrical turns per update. */
	uint64_t phase_increment_num = (uint64_t)motor_h->targets.target_mechanical_speed_mrpm *
								   (uint64_t)motor_h->limits.pole_pairs *
								   (uint64_t)motor_openloop_drive_cfg->update_period_ms *
								   MOTOR_OPENLOOP_DRIVE_PHASE_ACCUM_FULL_TURN_U32;
	uint32_t target_phase_increment_u32 = (uint32_t)((phase_increment_num +
													 ((MOTOR_OPENLOOP_DRIVE_MS_PER_MINUTE * MOTOR_OPENLOOP_DRIVE_MRPM_PER_RPM) / 2u)) /
													 (MOTOR_OPENLOOP_DRIVE_MS_PER_MINUTE * MOTOR_OPENLOOP_DRIVE_MRPM_PER_RPM));

	motor_openloop_drive_h->cfg = motor_openloop_drive_cfg;
	motor_openloop_drive_h->motor_h = motor_h;
	motor_h->openloop.phase_accumulator_u32 = 0u;
	motor_h->openloop.current_phase_increment_u32 = 0u;
	motor_h->openloop.target_phase_increment_u32 = target_phase_increment_u32;
	motor_h->openloop.last_electrical_angle_u16 = 0u;
	motor_h->measurements.electrical_angle_u16 = 0u;
	motor_h->status.mode = MOTOR_MODE_OPENLOOP_DRIVE;
	motor_h->status.has_valid_electrical_angle = false;
	motor_openloop_drive_h->is_initialized = true;

	return true;
}

/*
 * Phase progression model:
 *
 * phase_accumulator_u32 stores one full electrical turn over uint32_t wrap.
 * Each update adds phase_increment_u32, then the upper 16 bits are exported as
 * the uint16 electrical angle used by motor_openloop_sine.
 *
 * electrical_angle_u16 = phase_accumulator_u32 >> 16
 */
bool motor_openloop_drive_update(motor_openloop_drive_handle_t *motor_openloop_drive_h)
{
	if (motor_openloop_drive_h == NULL) return false;
	if ((motor_openloop_drive_h->cfg == NULL) || (motor_openloop_drive_h->motor_h == NULL)) return false;
	if (motor_openloop_drive_h->cfg->motor_openloop_sine_h == NULL) return false;
	if (motor_openloop_drive_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_openloop_drive_h->motor_h;

	/* Simple per-update speed-transition limiter, not a full motion-profile generator. */
	if (motor_h->openloop.current_phase_increment_u32 < motor_h->openloop.target_phase_increment_u32)
	{
		uint32_t increment_delta = motor_h->openloop.target_phase_increment_u32 -
								   motor_h->openloop.current_phase_increment_u32;
		if (increment_delta > motor_openloop_drive_h->cfg->phase_increment_ramp_step_u32)
		{
			increment_delta = motor_openloop_drive_h->cfg->phase_increment_ramp_step_u32;
		}

		motor_h->openloop.current_phase_increment_u32 += increment_delta;
	}
	else if (motor_h->openloop.current_phase_increment_u32 > motor_h->openloop.target_phase_increment_u32)
	{
		uint32_t increment_delta = motor_h->openloop.current_phase_increment_u32 -
								   motor_h->openloop.target_phase_increment_u32;
		if (increment_delta > motor_openloop_drive_h->cfg->phase_increment_ramp_step_u32)
		{
			increment_delta = motor_openloop_drive_h->cfg->phase_increment_ramp_step_u32;
		}

		motor_h->openloop.current_phase_increment_u32 -= increment_delta;
	}

	/* The accumulator spans one full electrical turn over uint32_t. */
	uint32_t next_phase_accumulator_u32 = motor_h->openloop.phase_accumulator_u32 +
										  motor_h->openloop.current_phase_increment_u32;
	/* The upper 16 bits map directly to the uint16 electrical-angle interface. */
	uint16_t next_electrical_angle_u16 = (uint16_t)(next_phase_accumulator_u32 >> MOTOR_OPENLOOP_DRIVE_PHASE_ACCUM_ANGLE_SHIFT);

	if (!motor_openloop_sine_apply(motor_openloop_drive_h->cfg->motor_openloop_sine_h,
						       next_electrical_angle_u16,
						       motor_h->targets.target_amplitude_permyriad)) return false;

	motor_h->openloop.phase_accumulator_u32 = next_phase_accumulator_u32;
	motor_h->openloop.last_electrical_angle_u16 = next_electrical_angle_u16;
	motor_h->measurements.electrical_angle_u16 = next_electrical_angle_u16;
	motor_h->status.has_valid_electrical_angle = true;
	return true;
}
