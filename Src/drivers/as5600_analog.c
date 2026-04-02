/**
 * @file as5600_analog.c
 * @brief Narrow AS5600 analog-angle acquisition helper.
 *
 */

#include "drivers/as5600_analog.h"
#include <stddef.h>

static as5600_analog_handle_t *s_as5600_analog_h = NULL;

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
	if (as5600_analog_cfg->raw_samples_per_publish == 0u) return false;
	if ((s_as5600_analog_h != NULL) && (s_as5600_analog_h != as5600_analog_h)) return false;

	/* Reset acquisition, averaging, and publication state. */
	as5600_analog_h->cfg = as5600_analog_cfg;
	as5600_analog_h->raw_accumulator = 0u;
	as5600_analog_h->raw_sample_count = 0u;
	as5600_analog_h->last_raw_averaged = 0u;
	as5600_analog_h->mechanical_angle_u16 = 0u;
	as5600_analog_h->next_raw_sample_time_us = 0u;
	as5600_analog_h->raw_conversion_pending = false;
	as5600_analog_h->has_new_angle_sample = false;
	as5600_analog_h->is_initialized = true;
	s_as5600_analog_h = as5600_analog_h;

	return true;
}

bool as5600_analog_service(as5600_analog_handle_t *as5600_analog_h,
						   uint64_t now_us)
{
	if (as5600_analog_h == NULL) return false;
	if (as5600_analog_h->cfg == NULL) return false;
	if (as5600_analog_h->cfg->adc_h == NULL) return false;
	if (as5600_analog_h->is_initialized == false) return false;

	/* Seed the first SysTick-driven raw-sample slot on the first service call. */
	if (as5600_analog_h->next_raw_sample_time_us == 0u)
	{
		as5600_analog_h->next_raw_sample_time_us = now_us;
	}

	/* Start one raw ADC conversion only when the next SysTick slot has elapsed. */
	if ((as5600_analog_h->raw_conversion_pending == false) &&
		(now_us >= as5600_analog_h->next_raw_sample_time_us))
	{
		/* Advance the next raw-sample slot by one configured SysTick period. */
		as5600_analog_h->next_raw_sample_time_us += as5600_analog_h->cfg->raw_sample_period_us;

		/* Trigger one single-shot conversion for the current raw-sample slot. */
		adc_start(as5600_analog_h->cfg->adc_h);
		as5600_analog_h->raw_conversion_pending = true;
	}

	return true;
}

void as5600_analog_adc_irq_handler(void)
{
	if (s_as5600_analog_h == NULL) return;
	if (s_as5600_analog_h->cfg == NULL) return;
	if (s_as5600_analog_h->cfg->adc_h == NULL) return;
	if (s_as5600_analog_h->is_initialized == false) return;

	uint16_t raw_sample = 0u;

	/* Consume one completed ADC conversion from the driver-level IRQ path. */
	if (!adc_read(s_as5600_analog_h->cfg->adc_h, &raw_sample))
	{
		return;
	}

	/* Close the pending raw-sample slot and accumulate the new ADC code. */
	s_as5600_analog_h->raw_conversion_pending = false;
	s_as5600_analog_h->raw_accumulator += raw_sample;
	s_as5600_analog_h->raw_sample_count++;

	/* Publish one averaged angle after the configured raw-sample decimation. */
	if (s_as5600_analog_h->raw_sample_count >= s_as5600_analog_h->cfg->raw_samples_per_publish)
	{
		/* raw_avg = raw_sum / raw_samples_per_publish for the completed publish window. */
		s_as5600_analog_h->last_raw_averaged =
				(uint16_t)(s_as5600_analog_h->raw_accumulator / s_as5600_analog_h->raw_sample_count);

		/* Convert averaged raw ADC code into full-turn uint16 mechanical angle. */
		s_as5600_analog_h->mechanical_angle_u16 =
				as5600_analog_raw_to_mechanical_angle_u16(s_as5600_analog_h->last_raw_averaged,
														  s_as5600_analog_h->cfg->adc_full_scale,
														  s_as5600_analog_h->cfg->mechanical_angle_direction);

		/* Start a new raw accumulation window after publishing the averaged angle. */
		s_as5600_analog_h->raw_accumulator = 0u;
		s_as5600_analog_h->raw_sample_count = 0u;
		s_as5600_analog_h->has_new_angle_sample = true;
	}

}
