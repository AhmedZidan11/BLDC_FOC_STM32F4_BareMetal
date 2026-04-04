/**
 * @file as5600_analog.c
 * @brief Sensor-specific AS5600 analog angle acquisition helper.
 *
 * This module runs a fixed raw-sample start schedule from SysTick, takes
 * completed ADC samples in the ADC IRQ path, averages a fixed raw-sample
 * count, and publishes one mechanical angle sample for the main loop.
 */

#include "drivers/as5600_analog.h"
#include <stddef.h>

/* The current IRQ integration forwards ADC samples to one active AS5600 handle. */
static as5600_analog_handle_t *s_as5600_analog_h = NULL;

/**
 * @brief Convert one raw sample window into one mechanical angle.
 *
 * @param raw_accumulator Sum of raw ADC codes in the current publish window.
 * @param raw_sample_count Number of raw samples in the current publish window.
 * @param adc_full_scale ADC full-scale code for the AS5600 analog path.
 * @param direction Mechanical-angle direction for the published sample.
 * @return Mechanical angle in full-turn uint16 units.
 */
static uint16_t as5600_analog_raw_window_to_mechanical_angle_u16(uint32_t raw_accumulator,
																  uint16_t raw_sample_count,
																  uint16_t adc_full_scale,
																  as5600_analog_direction_t direction)
{
	uint64_t mechanical_angle_num = 0u;
	uint64_t mechanical_angle_den = 0u;

	/* Convert the full raw sum into angle_u16 and round to the nearest step. */
	mechanical_angle_num =
			((uint64_t)raw_accumulator * (uint64_t)AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16) +
			(((uint64_t)adc_full_scale * (uint64_t)raw_sample_count) / 2u);
	mechanical_angle_den =
			(uint64_t)adc_full_scale * (uint64_t)raw_sample_count;

	uint16_t mechanical_angle_u16 =
			(uint16_t)(mechanical_angle_num / mechanical_angle_den);

	/* Reverse the sensor direction with wrapped negation in full-turn uint16 units. */
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

	/* Reset sampling, averaging, and publish state. */
	as5600_analog_h->cfg = as5600_analog_cfg;
	as5600_analog_h->raw_accumulator = 0u;
	as5600_analog_h->raw_sample_count = 0u;
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

	/* Set the first raw sample slot on the first service call. */
	if (as5600_analog_h->next_raw_sample_time_us == 0u)
	{
		as5600_analog_h->next_raw_sample_time_us = now_us;
	}

	/* Clear one stale pending conversion so the next raw slot can recover. */
	if (as5600_analog_h->raw_conversion_pending == true)
	{
		/* stale_timeout = next_raw_sample_time_us + raw_sample_period_us. */
		uint64_t stale_timeout_us =
				as5600_analog_h->next_raw_sample_time_us +
				(uint64_t)as5600_analog_h->cfg->raw_sample_period_us;

		if (now_us >= stale_timeout_us)
		{
			as5600_analog_h->raw_conversion_pending = false;
		}
	}

	/* Start one raw ADC conversion only when the next sample slot is due. */
	if ((as5600_analog_h->raw_conversion_pending == false) &&
		(now_us >= as5600_analog_h->next_raw_sample_time_us))
	{
		/* Move the next raw sample slot by one configured period. */
		as5600_analog_h->next_raw_sample_time_us += as5600_analog_h->cfg->raw_sample_period_us;

		/* Start one single-shot conversion for the current raw sample slot. */
		adc_start(as5600_analog_h->cfg->adc_h);
		as5600_analog_h->raw_conversion_pending = true;
	}

	return true;
}

bool as5600_analog_consume_published_sample(as5600_analog_handle_t *as5600_analog_h,
											uint16_t *mechanical_angle_u16)
{
	if (as5600_analog_h == NULL) return false;
	if (mechanical_angle_u16 == NULL) return false;
	if (as5600_analog_h->cfg == NULL) return false;
	if (as5600_analog_h->is_initialized == false) return false;
	if (as5600_analog_h->has_new_angle_sample == false) return false;

	/* Take the latest published angle sample and clear the ready flag. */
	*mechanical_angle_u16 = as5600_analog_h->mechanical_angle_u16;
	as5600_analog_h->has_new_angle_sample = false;

	return true;
}

void as5600_analog_adc_irq_handler(void)
{
	if (s_as5600_analog_h == NULL) return;
	if (s_as5600_analog_h->cfg == NULL) return;
	if (s_as5600_analog_h->cfg->adc_h == NULL) return;
	if (s_as5600_analog_h->is_initialized == false) return;

	uint16_t raw_sample = 0u;

	/* Read one completed ADC conversion from the driver IRQ path. */
	if (!adc_read(s_as5600_analog_h->cfg->adc_h, &raw_sample))
	{
		return;
	}

	/* Close the pending raw slot and add the new ADC code to the window. */
	s_as5600_analog_h->raw_conversion_pending = false;
	s_as5600_analog_h->raw_accumulator += raw_sample;
	s_as5600_analog_h->raw_sample_count++;

	/* Publish one mechanical angle sample after the configured raw-sample count. */
	if (s_as5600_analog_h->raw_sample_count >= s_as5600_analog_h->cfg->raw_samples_per_publish)
	{
		/* Convert the full raw window into one rounded full-turn mechanical angle. */
		s_as5600_analog_h->mechanical_angle_u16 =
				as5600_analog_raw_window_to_mechanical_angle_u16(s_as5600_analog_h->raw_accumulator,
																 s_as5600_analog_h->raw_sample_count,
																 s_as5600_analog_h->cfg->adc_full_scale,
																 s_as5600_analog_h->cfg->mechanical_angle_direction);

		/* Start a new raw window after publishing the averaged angle. */
		s_as5600_analog_h->raw_accumulator = 0u;
		s_as5600_analog_h->raw_sample_count = 0u;
		/* The main loop consumes one published angle sample through has_new_angle_sample. */
		s_as5600_analog_h->has_new_angle_sample = true;
	}

}
