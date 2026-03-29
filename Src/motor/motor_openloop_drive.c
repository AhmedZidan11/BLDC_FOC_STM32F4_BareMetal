/**
 * @file motor_openloop_drive.c
 * @brief Small stateful open-loop drive layer above motor_openloop_sine.
 *
 */

#include "motor/motor_openloop_drive.h"
#include <stddef.h>

bool motor_openloop_drive_init(motor_openloop_drive_handle_t *motor_openloop_drive_h,
							   const motor_openloop_drive_cfg_t *motor_openloop_drive_cfg)
{
	if ((motor_openloop_drive_h == NULL) || (motor_openloop_drive_cfg == NULL)) return false;
	if (motor_openloop_drive_cfg->motor_openloop_sine_h == NULL) return false;
	if (motor_openloop_drive_cfg->amplitude_permyriad > 10000u) return false;

	motor_openloop_drive_h->cfg = motor_openloop_drive_cfg;
	motor_openloop_drive_h->phase_accumulator_u32 = 0u;
	motor_openloop_drive_h->phase_increment_u32 = 0u;
	motor_openloop_drive_h->last_electrical_angle_u16 = 0u;
	motor_openloop_drive_h->is_initialized = true;

	return true;
}

bool motor_openloop_drive_update(motor_openloop_drive_handle_t *motor_openloop_drive_h)
{
	if (motor_openloop_drive_h == NULL) return false;
	if ((motor_openloop_drive_h->cfg == NULL) || (motor_openloop_drive_h->cfg->motor_openloop_sine_h == NULL)) return false;
	if (motor_openloop_drive_h->is_initialized == false) return false;

	uint32_t next_phase_accumulator_u32 = motor_openloop_drive_h->phase_accumulator_u32 +
									  motor_openloop_drive_h->phase_increment_u32;
	uint16_t next_electrical_angle_u16 = (uint16_t)(next_phase_accumulator_u32 >> MOTOR_OPENLOOP_DRIVE_PHASE_ACCUM_ANGLE_SHIFT);

	if (!motor_openloop_sine_apply(motor_openloop_drive_h->cfg->motor_openloop_sine_h,
						       next_electrical_angle_u16,
						       motor_openloop_drive_h->cfg->amplitude_permyriad)) return false;

	motor_openloop_drive_h->phase_accumulator_u32 = next_phase_accumulator_u32;
	motor_openloop_drive_h->last_electrical_angle_u16 = next_electrical_angle_u16;
	return true;
}
