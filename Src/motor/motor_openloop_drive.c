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
	motor_openloop_drive_h->current_electrical_angle_u16 = 0u;
	motor_openloop_drive_h->is_initialized = true;

	return true;
}

bool motor_openloop_drive_update(motor_openloop_drive_handle_t *motor_openloop_drive_h)
{
	if (motor_openloop_drive_h == NULL) return false;
	if ((motor_openloop_drive_h->cfg == NULL) || (motor_openloop_drive_h->cfg->motor_openloop_sine_h == NULL)) return false;
	if (motor_openloop_drive_h->is_initialized == false) return false;

	uint16_t next_electrical_angle_u16 = (uint16_t)(motor_openloop_drive_h->current_electrical_angle_u16 +
									 motor_openloop_drive_h->cfg->electrical_angle_step_u16);

	if (!motor_openloop_sine_apply(motor_openloop_drive_h->cfg->motor_openloop_sine_h,
						       next_electrical_angle_u16,
						       motor_openloop_drive_h->cfg->amplitude_permyriad)) return false;

	motor_openloop_drive_h->current_electrical_angle_u16 = next_electrical_angle_u16;
	return true;
}
