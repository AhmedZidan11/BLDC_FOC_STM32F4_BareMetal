/**
 * @file as5600_analog.c
 * @brief Narrow AS5600 analog-angle acquisition helper.
 *
 */

#include "drivers/as5600_analog.h"
#include <stddef.h>

static uint16_t as5600_analog_raw_to_mechanical_angle_u16(uint16_t raw_averaged,
														   uint16_t adc_full_scale,
														   as5600_analog_direction_t direction)
{
	/* mechanical_angle_u16 = raw_averaged * 65535 / adc_full_scale. */
	uint16_t mechanical_angle_u16 =
			(uint16_t)(((uint32_t)raw_averaged * AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16) /
					  (uint32_t)adc_full_scale);

	/* Reverse sensor direction by wrapped negation in full-turn uint16 units. */
	if (direction == AS5600_ANALOG_DIRECTION_REVERSE)
	{
		mechanical_angle_u16 = (uint16_t)(0u - mechanical_angle_u16);
	}

	return mechanical_angle_u16;
}

bool as5600_analog_init(as5600_analog_handle_t *as5600_analog_h,
						const as5600_analog_cfg_t *as5600_analog_cfg)
{
	if ((as5600_analog_h == NULL) || (as5600_analog_cfg == NULL)) return false;
	if (as5600_analog_cfg->adc_h == NULL) return false;
	if (as5600_analog_cfg->adc_full_scale == 0u) return false;
	if (as5600_analog_cfg->raw_sample_period_us == 0u) return false;
	if (as5600_analog_cfg->angle_publish_period_us == 0u) return false;
	if (as5600_analog_cfg->angle_publish_period_us < as5600_analog_cfg->raw_sample_period_us) return false;

	/* Reset acquisition, averaging, and publication state. */
	as5600_analog_h->cfg = as5600_analog_cfg;
	as5600_analog_h->raw_accumulator = 0u;
	as5600_analog_h->raw_sample_count = 0u;
	as5600_analog_h->last_raw_averaged = 0u;
	as5600_analog_h->mechanical_angle_u16 = 0u;
	as5600_analog_h->last_raw_sample_time_us = 0u;
	as5600_analog_h->last_angle_publish_time_us = 0u;
	as5600_analog_h->has_new_angle_sample = false;
	as5600_analog_h->is_initialized = true;

	return true;
}

bool as5600_analog_update(as5600_analog_handle_t *as5600_analog_h,
						  uint64_t now_us)
{
	if (as5600_analog_h == NULL) return false;
	if (as5600_analog_h->cfg == NULL) return false;
	if (as5600_analog_h->cfg->adc_h == NULL) return false;
	if (as5600_analog_h->is_initialized == false) return false;

	/* Clear the publication flag until a new averaged angle is emitted. */
	as5600_analog_h->has_new_angle_sample = false;

	/* Decide whether the next raw ADC acquisition slot has elapsed. */
	if ((now_us - as5600_analog_h->last_raw_sample_time_us) >= as5600_analog_h->cfg->raw_sample_period_us)
	{
		uint16_t raw_sample;

		/* Read the completed conversion and accumulate raw ADC input. */
		if (adc_read(as5600_analog_h->cfg->adc_h, &raw_sample))
		{
			as5600_analog_h->raw_accumulator += raw_sample;
			as5600_analog_h->raw_sample_count++;
		}

		/* Trigger the next single-shot conversion for the following sample slot. */
		adc_start(as5600_analog_h->cfg->adc_h);
		as5600_analog_h->last_raw_sample_time_us += as5600_analog_h->cfg->raw_sample_period_us;
	}

	/* Decide whether the next averaged-angle publication slot has elapsed. */
	if ((now_us - as5600_analog_h->last_angle_publish_time_us) >= as5600_analog_h->cfg->angle_publish_period_us)
	{
		/* Publish one averaged mechanical angle sample when the window has data. */
		if (as5600_analog_h->raw_sample_count > 0u)
		{
			/* raw_avg = raw_sum / sample_count over the completed publish window. */
			as5600_analog_h->last_raw_averaged =
					(uint16_t)(as5600_analog_h->raw_accumulator / as5600_analog_h->raw_sample_count);
			/* Convert averaged raw ADC code into full-turn uint16 mechanical angle. */
			as5600_analog_h->mechanical_angle_u16 =
					as5600_analog_raw_to_mechanical_angle_u16(as5600_analog_h->last_raw_averaged,
															  as5600_analog_h->cfg->adc_full_scale,
															  as5600_analog_h->cfg->mechanical_angle_direction);
			/* Start a new accumulation window after publishing the averaged sample. */
			as5600_analog_h->raw_accumulator = 0u;
			as5600_analog_h->raw_sample_count = 0u;
			as5600_analog_h->has_new_angle_sample = true;
		}

		/* Advance the publication schedule by one configured publish interval. */
		as5600_analog_h->last_angle_publish_time_us += as5600_analog_h->cfg->angle_publish_period_us;
	}

	return true;
}
