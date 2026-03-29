/**
 * @file motor_openloop_drive.c
 * @brief Small stateful open-loop drive layer above motor_openloop_sine.
 *
 */

#include "motor/motor_openloop_drive.h"
#include <stddef.h>

/*
 * Open-loop speed model:
 *
 * mechanical_speed_rpm [rev_m/min]
 * electrical_speed_rpm = mechanical_speed_rpm * pole_pairs [rev_e/min]
 * electrical_turns_per_update =
 *     electrical_speed_rpm * update_period_ms / 60000 [turn_e/update]
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
	if (motor_openloop_drive_cfg->motor_openloop_sine_h == NULL) return false;
	if (motor_openloop_drive_cfg->amplitude_permyriad > 10000u) return false;
	if (motor_openloop_drive_cfg->update_period_ms == 0u) return false;

	uint64_t max_speed_denom = (uint64_t)MOTOR_OPENLOOP_DRIVE_DEFAULT_TEST_POLE_PAIRS *
							   (uint64_t)motor_openloop_drive_cfg->update_period_ms *
							   (uint64_t)MOTOR_OPENLOOP_DRIVE_MIN_UPDATES_PER_ELECTRICAL_TURN;
	uint16_t max_target_mechanical_speed_rpm = (uint16_t)(MOTOR_OPENLOOP_DRIVE_MS_PER_MINUTE / max_speed_denom);
	if (motor_openloop_drive_cfg->target_mechanical_speed_rpm > max_target_mechanical_speed_rpm) return false;

	/* Convert mechanical RPM and update period into electrical turns per update. */
	uint64_t phase_increment_num = (uint64_t)motor_openloop_drive_cfg->target_mechanical_speed_rpm *
								   (uint64_t)MOTOR_OPENLOOP_DRIVE_DEFAULT_TEST_POLE_PAIRS *
								   (uint64_t)motor_openloop_drive_cfg->update_period_ms *
								   MOTOR_OPENLOOP_DRIVE_PHASE_ACCUM_FULL_TURN_U32;
	uint32_t phase_increment_u32 = (uint32_t)((phase_increment_num + (MOTOR_OPENLOOP_DRIVE_MS_PER_MINUTE / 2u)) /
											  MOTOR_OPENLOOP_DRIVE_MS_PER_MINUTE);

	motor_openloop_drive_h->cfg = motor_openloop_drive_cfg;
	motor_openloop_drive_h->phase_accumulator_u32 = 0u;
	motor_openloop_drive_h->phase_increment_u32 = phase_increment_u32;
	motor_openloop_drive_h->last_electrical_angle_u16 = 0u;
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
	if ((motor_openloop_drive_h->cfg == NULL) || (motor_openloop_drive_h->cfg->motor_openloop_sine_h == NULL)) return false;
	if (motor_openloop_drive_h->is_initialized == false) return false;

	/* The accumulator spans one full electrical turn over uint32_t. */
	uint32_t next_phase_accumulator_u32 = motor_openloop_drive_h->phase_accumulator_u32 +
										  motor_openloop_drive_h->phase_increment_u32;
	/* The upper 16 bits map directly to the uint16 electrical-angle interface. */
	uint16_t next_electrical_angle_u16 = (uint16_t)(next_phase_accumulator_u32 >> MOTOR_OPENLOOP_DRIVE_PHASE_ACCUM_ANGLE_SHIFT);

	if (!motor_openloop_sine_apply(motor_openloop_drive_h->cfg->motor_openloop_sine_h,
						       next_electrical_angle_u16,
						       motor_openloop_drive_h->cfg->amplitude_permyriad)) return false;

	motor_openloop_drive_h->phase_accumulator_u32 = next_phase_accumulator_u32;
	motor_openloop_drive_h->last_electrical_angle_u16 = next_electrical_angle_u16;
	return true;
}
