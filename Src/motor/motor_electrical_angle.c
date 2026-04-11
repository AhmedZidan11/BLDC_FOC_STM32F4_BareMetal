/**
 * @file motor_electrical_angle.c
 * @brief simple helper that converts measured mechanical angle into measured electrical angle.
 *
 */

#include "motor/motor_electrical_angle.h"
#include <stddef.h>

bool motor_electrical_angle_init(motor_electrical_angle_handle_t *motor_electrical_angle_h,
								 const motor_electrical_angle_cfg_t *motor_electrical_angle_cfg)
{
	if ((motor_electrical_angle_h == NULL) || (motor_electrical_angle_cfg == NULL)) return false;
	if (motor_electrical_angle_cfg->motor_h == NULL) return false;
	if (motor_electrical_angle_cfg->motor_h->limits.pole_pairs == 0u) return false;

	/* Reset the measured electrical-angle helper state. */
	motor_electrical_angle_h->cfg = motor_electrical_angle_cfg;
	motor_electrical_angle_h->motor_h = motor_electrical_angle_cfg->motor_h;
	motor_electrical_angle_h->electrical_offset_u16 = motor_electrical_angle_cfg->electrical_offset_u16;
	motor_electrical_angle_h->is_initialized = true;

	return true;
}

bool motor_electrical_angle_update(motor_electrical_angle_handle_t *motor_electrical_angle_h,
								   uint16_t mechanical_angle_u16)
{
	uint16_t raw_electrical_angle_u16 = 0u;

	if (motor_electrical_angle_h == NULL) return false;
	if ((motor_electrical_angle_h->cfg == NULL) || (motor_electrical_angle_h->motor_h == NULL)) return false;
	if (motor_electrical_angle_h->is_initialized == false) return false;

	if (!motor_electrical_angle_compute_raw(motor_electrical_angle_h,
											mechanical_angle_u16,
											&raw_electrical_angle_u16))
	{
		return false;
	}

	/* measured_electrical = raw_electrical + electrical_offset. */
	uint32_t measured_electrical_angle_u32 =
			(uint32_t)raw_electrical_angle_u16 +
			(uint32_t)motor_electrical_angle_h->electrical_offset_u16;

	/* Publish the measured electrical angle for future sensored control paths. */
	motor_handle_t *motor_h = motor_electrical_angle_h->motor_h;
	motor_h->measurements.electrical_angle_u16 = (uint16_t)measured_electrical_angle_u32;
	motor_h->status.has_valid_electrical_angle = true;

	return true;
}

bool motor_electrical_angle_compute_raw(motor_electrical_angle_handle_t *motor_electrical_angle_h,
										uint16_t mechanical_angle_u16,
										uint16_t *electrical_angle_u16)
{
	if (motor_electrical_angle_h == NULL) return false;
	if (electrical_angle_u16 == NULL) return false;
	if ((motor_electrical_angle_h->cfg == NULL) || (motor_electrical_angle_h->motor_h == NULL)) return false;
	if (motor_electrical_angle_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_electrical_angle_h->motor_h;
	if (motor_h->limits.pole_pairs == 0u) return false;

	/* raw_electrical = mechanical_angle * pole_pairs, wrapped to uint16 full turn. */
	uint32_t raw_electrical_angle_u32 =
			(uint32_t)mechanical_angle_u16 * (uint32_t)motor_h->limits.pole_pairs;
	*electrical_angle_u16 = (uint16_t)raw_electrical_angle_u32;

	return true;
}

bool motor_electrical_angle_set_offset(motor_electrical_angle_handle_t *motor_electrical_angle_h,
									   uint16_t electrical_offset_u16)
{
	if (motor_electrical_angle_h == NULL) return false;
	if (motor_electrical_angle_h->is_initialized == false) return false;

	/* Store the electrical offset used by measured electrical-angle conversion. */
	motor_electrical_angle_h->electrical_offset_u16 = electrical_offset_u16;

	return true;
}
