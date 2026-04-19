/**
 * @file as5600_analog.c
 * @brief Sensor-specific AS5600 analog angle acquisition helper.
 *
 * This module schedules fixed-rate raw ADC sampling, builds one published
 * mechanical-angle sample from a fixed raw window, and exposes one consumed
 * sample to the application layer.
 */

#include "drivers/as5600_analog.h"
#include <stddef.h>

/* The current IRQ integration forwards ADC samples to one active AS5600 handle. */
static as5600_analog_handle_t *s_as5600_analog_h = NULL;

/**
 * @brief Save IRQ state and enter a short critical section.
 *
 * @return Previous PRIMASK state.
 */
static uint32_t as5600_analog_irq_save(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

/**
 * @brief Restore IRQ state saved by as5600_analog_irq_save().
 *
 * @param primask Previous PRIMASK state.
 */
static void as5600_analog_irq_restore(uint32_t primask)
{
	if ((primask & 1u) == 0u)
	{
		__enable_irq();
	}
}

/**
 * @brief Convert one raw ADC sample into one mechanical angle sample.
 *
 * @param raw_sample Raw ADC sample from AS5600 analog output.
 * @param adc_full_scale ADC full-scale code for the AS5600 analog path.
 * @param direction Mechanical-angle direction for the published sample.
 * @return Mechanical angle in full-turn uint16 units.
 */
static uint16_t as5600_analog_raw_sample_to_mechanical_angle_u16(uint16_t raw_sample,
																  uint16_t adc_full_scale,
																  as5600_analog_direction_t direction)
{
	/* Convert one raw ADC code into one rounded full-turn uint16 mechanical angle. */
	uint64_t mechanical_angle_num =
			((uint64_t)raw_sample * (uint64_t)AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16) +
			((uint64_t)adc_full_scale / 2u);
	uint16_t mechanical_angle_u16 =
			(uint16_t)(mechanical_angle_num / (uint64_t)adc_full_scale);

	/* Reverse sensor polarity with wrapped negation in full-turn units. */
	if (direction == AS5600_ANALOG_DIRECTION_REVERSE)
	{
		mechanical_angle_u16 = (uint16_t)(0u - mechanical_angle_u16);
	}

	return mechanical_angle_u16;
}

/**
 * @brief Compute one signed wrapped delta between two uint16 angles.
 *
 * @param angle_u16 Current angle sample in full-turn uint16 units.
 * @param reference_angle_u16 Reference angle in full-turn uint16 units.
 * @return Signed shortest-path delta in angle counts.
 */
static int32_t as5600_analog_wrapped_delta_counts(uint16_t angle_u16,
												  uint16_t reference_angle_u16)
{
	return (int32_t)((int16_t)((uint16_t)(angle_u16 - reference_angle_u16)));
}

/**
 * @brief Divide one signed integer and round to nearest integer.
 *
 * @param numerator Signed numerator.
 * @param denominator Positive denominator.
 * @return Rounded signed quotient.
 */
static int32_t as5600_analog_divide_round_signed_i32(int32_t numerator,
													  uint16_t denominator)
{
	if (numerator >= 0)
	{
		return (int32_t)(((int64_t)numerator + ((int64_t)denominator / 2LL)) /
						 (int64_t)denominator);
	}

	return (int32_t)(((int64_t)numerator - ((int64_t)denominator / 2LL)) /
					 (int64_t)denominator);
}

/**
 * @brief Finalize one wrap-safe average angle over the active publish window.
 *
 * @param reference_angle_u16 First angle sample in the publish window.
 * @param delta_sum_counts Sum of wrapped signed deltas to the reference angle.
 * @param sample_count Number of samples in the active publish window.
 * @return Averaged mechanical angle in full-turn uint16 units.
 */
static uint16_t as5600_analog_finalize_window_average_angle_u16(uint16_t reference_angle_u16,
																 int32_t delta_sum_counts,
																 uint16_t sample_count)
{
	/* mean_delta = sum(relative_delta_i) / N, with relative_delta_0 = 0 at the reference sample. */
	int32_t mean_delta_counts = as5600_analog_divide_round_signed_i32(delta_sum_counts, sample_count);

	/* avg_angle = reference_angle + mean_delta, with normal uint16 wraparound. */
	return (uint16_t)((int32_t)reference_angle_u16 + mean_delta_counts);
}

/**
 * @brief Convert one sample angle into one corrected signed delta from last published angle.
 *
 * @param sample_angle_u16 Current sample angle in full-turn uint16 units.
 * @param last_published_angle_u16 Last published mechanical angle.
 * @param wrap_correction_threshold_counts Wrap correction threshold in counts.
 * @return Corrected signed delta in angle counts.
 */
static int32_t as5600_analog_compute_corrected_delta_from_last_published(uint16_t sample_angle_u16,
																		  uint16_t last_published_angle_u16,
																		  uint16_t wrap_correction_threshold_counts)
{
	int32_t corrected_delta_counts =
			(int32_t)sample_angle_u16 - (int32_t)last_published_angle_u16;
	const int32_t full_turn_counts = (int32_t)AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16 + 1;

	/* Apply one wrap correction when delta exceeds the configured half-turn threshold. */
	if (corrected_delta_counts > (int32_t)wrap_correction_threshold_counts)
	{
		corrected_delta_counts -= full_turn_counts;
	}
	else if (corrected_delta_counts < -(int32_t)wrap_correction_threshold_counts)
	{
		corrected_delta_counts += full_turn_counts;
	}

	return corrected_delta_counts;
}

/**
 * @brief Compute absolute value for one signed 32-bit value.
 *
 * @param value Signed value.
 * @return Absolute value.
 */
static int32_t as5600_analog_abs_i32(int32_t value)
{
	return (value < 0) ? (-value) : value;
}

/**
 * @brief Wrap one signed reconstructed angle into uint16 full-turn range.
 *
 * @param signed_angle_counts Signed reconstructed angle in full-turn counts.
 * @return Wrapped angle in uint16 full-turn units.
 */
static uint16_t as5600_analog_wrap_signed_angle_to_u16(int32_t signed_angle_counts)
{
	const int32_t full_turn_counts = (int32_t)AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16 + 1;
	int32_t wrapped_counts = signed_angle_counts % full_turn_counts;

	if (wrapped_counts < 0)
	{
		wrapped_counts += full_turn_counts;
	}

	return (uint16_t)wrapped_counts;
}

/**
 * @brief Find one nearest real sample index by absolute corrected delta.
 *
 * @param corrected_deltas_counts Corrected signed deltas for each window sample.
 * @param sample_count Number of valid samples.
 * @return Earliest sample index with the smallest absolute corrected delta.
 */
static uint16_t as5600_analog_find_nearest_sample_index(const int32_t *corrected_deltas_counts,
														uint16_t sample_count)
{
	uint16_t i = 0u;
	uint16_t nearest_index = 0u;
	int32_t nearest_abs_delta_counts = 0;

	if ((corrected_deltas_counts == NULL) || (sample_count == 0u))
	{
		return 0u;
	}

	nearest_abs_delta_counts = as5600_analog_abs_i32(corrected_deltas_counts[0u]);
	nearest_index = 0u;

	for (i = 1u; i < sample_count; i++)
	{
		int32_t candidate_abs_delta_counts =
				as5600_analog_abs_i32(corrected_deltas_counts[i]);

		if (candidate_abs_delta_counts < nearest_abs_delta_counts)
		{
			nearest_abs_delta_counts = candidate_abs_delta_counts;
			nearest_index = i;
		}
	}

	return nearest_index;
}

bool as5600_analog_init(as5600_analog_handle_t *as5600_analog_h,
						const as5600_analog_cfg_t *as5600_analog_cfg)
{
	uint16_t i = 0u;

	if ((as5600_analog_h == NULL) || (as5600_analog_cfg == NULL)) return false;
	if (as5600_analog_cfg->adc_h == NULL) return false;
	if (as5600_analog_cfg->adc_full_scale == 0u) return false;
	if (as5600_analog_cfg->raw_sample_period_us == 0u) return false;
	if (as5600_analog_cfg->raw_samples_per_publish == 0u) return false;
	if (as5600_analog_cfg->raw_samples_per_publish > AS5600_ANALOG_MAX_PUBLISH_WINDOW_SAMPLES) return false;
	if (as5600_analog_cfg->wrap_correction_threshold_counts == 0u) return false;
	if (as5600_analog_cfg->max_plausible_delta_per_publish_counts == 0u) return false;
	if ((s_as5600_analog_h != NULL) && (s_as5600_analog_h != as5600_analog_h)) return false;

	/* Reset scheduling and publish state. */
	as5600_analog_h->cfg = as5600_analog_cfg;
	as5600_analog_h->raw_sample_count = 0u;
	as5600_analog_h->window_reference_mechanical_angle_u16 = 0u;
	as5600_analog_h->window_delta_sum_counts = 0;
	as5600_analog_h->mechanical_angle_u16 = 0u;
	as5600_analog_h->has_published_angle = false;
	for (i = 0u; i < AS5600_ANALOG_MAX_PUBLISH_WINDOW_SAMPLES; i++)
	{
		as5600_analog_h->window_angle_samples_u16[i] = 0u;
	}
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
											as5600_analog_published_sample_t *published_sample)
{
	uint32_t primask = 0u;

	if (as5600_analog_h == NULL) return false;
	if (published_sample == NULL) return false;
	if (as5600_analog_h->cfg == NULL) return false;
	if (as5600_analog_h->is_initialized == false) return false;

	/* Take one coherent published-sample snapshot and clear the ready flag. */
	primask = as5600_analog_irq_save();
	if (as5600_analog_h->has_new_angle_sample == false)
	{
		as5600_analog_irq_restore(primask);
		return false;
	}

	published_sample->mechanical_angle_u16 = as5600_analog_h->mechanical_angle_u16;
	as5600_analog_h->has_new_angle_sample = false;
	as5600_analog_irq_restore(primask);

	return true;
}

void as5600_analog_adc_irq_handler(void)
{
	uint16_t window_sample_index = 0u;
	uint16_t current_sample_mechanical_angle_u16 = 0u;
	uint16_t published_mechanical_angle_u16 = 0u;
	uint16_t publish_raw_sample_count = 0u;
	bool has_last_published_angle = false;
	int32_t corrected_deltas_counts[AS5600_ANALOG_MAX_PUBLISH_WINDOW_SAMPLES] = {0};
	int32_t retained_corrected_delta_sum_counts = 0;
	uint16_t retained_sample_count = 0u;
	int32_t averaged_retained_corrected_delta_counts = 0;
	int32_t reconstructed_angle_counts = 0;
	uint16_t nearest_sample_index = 0u;
	uint16_t i = 0u;
	uint16_t raw_sample = 0u;

	if (s_as5600_analog_h == NULL) return;
	if (s_as5600_analog_h->cfg == NULL) return;
	if (s_as5600_analog_h->cfg->adc_h == NULL) return;
	if (s_as5600_analog_h->is_initialized == false) return;

	/* Read one completed ADC conversion from the driver IRQ path. */
	if (!adc_read(s_as5600_analog_h->cfg->adc_h, &raw_sample))
	{
		return;
	}

	/* Convert the new raw ADC sample into one mechanical angle sample. */
	current_sample_mechanical_angle_u16 =
			as5600_analog_raw_sample_to_mechanical_angle_u16(raw_sample,
															 s_as5600_analog_h->cfg->adc_full_scale,
															 s_as5600_analog_h->cfg->mechanical_angle_direction);

	/* Close the pending raw slot and append one angle sample into the current publish window. */
	window_sample_index = s_as5600_analog_h->raw_sample_count;
	s_as5600_analog_h->raw_conversion_pending = false;
	s_as5600_analog_h->window_angle_samples_u16[window_sample_index] = current_sample_mechanical_angle_u16;
	s_as5600_analog_h->raw_sample_count++;

	if (s_as5600_analog_h->raw_sample_count == 1u)
	{
		/* The first sample defines the wrap-safe reference for bootstrap averaging. */
		s_as5600_analog_h->window_reference_mechanical_angle_u16 = current_sample_mechanical_angle_u16;
		s_as5600_analog_h->window_delta_sum_counts = 0;
	}
	else
	{
		/* Accumulate wrapped signed deltas to avoid 0/360 boundary errors in bootstrap average. */
		s_as5600_analog_h->window_delta_sum_counts +=
				as5600_analog_wrapped_delta_counts(current_sample_mechanical_angle_u16,
												   s_as5600_analog_h->window_reference_mechanical_angle_u16);
	}

	/* Publish one mechanical angle sample after the configured raw-sample count. */
	if (s_as5600_analog_h->raw_sample_count >= s_as5600_analog_h->cfg->raw_samples_per_publish)
	{
		publish_raw_sample_count = s_as5600_analog_h->raw_sample_count;
		has_last_published_angle = s_as5600_analog_h->has_published_angle;

		if (has_last_published_angle == false)
		{
			/* Bootstrap publish uses wrap-safe window average because no last published anchor exists yet. */
			published_mechanical_angle_u16 =
					as5600_analog_finalize_window_average_angle_u16(
							s_as5600_analog_h->window_reference_mechanical_angle_u16,
							s_as5600_analog_h->window_delta_sum_counts,
							publish_raw_sample_count);
		}
		else
		{
			/* Convert each window sample into one wrap-corrected signed delta from last published angle. */
			retained_corrected_delta_sum_counts = 0;
			retained_sample_count = 0u;
			for (i = 0u; i < publish_raw_sample_count; i++)
			{
				corrected_deltas_counts[i] =
						as5600_analog_compute_corrected_delta_from_last_published(
								s_as5600_analog_h->window_angle_samples_u16[i],
								s_as5600_analog_h->mechanical_angle_u16,
								s_as5600_analog_h->cfg->wrap_correction_threshold_counts);

				/* Retain only samples whose corrected delta is plausible for one publish interval. */
				if (as5600_analog_abs_i32(corrected_deltas_counts[i]) <=
					(int32_t)s_as5600_analog_h->cfg->max_plausible_delta_per_publish_counts)
				{
					retained_corrected_delta_sum_counts += corrected_deltas_counts[i];
					retained_sample_count++;
				}
			}

			if (retained_sample_count > 0u)
			{
				/* Reconstruct one published angle from the retained corrected-delta average. */
				averaged_retained_corrected_delta_counts =
						as5600_analog_divide_round_signed_i32(
								retained_corrected_delta_sum_counts,
								retained_sample_count);
				reconstructed_angle_counts =
						(int32_t)s_as5600_analog_h->mechanical_angle_u16 +
						averaged_retained_corrected_delta_counts;
				published_mechanical_angle_u16 =
						as5600_analog_wrap_signed_angle_to_u16(reconstructed_angle_counts);
			}
			else
			{
				/* If no sample is plausible, publish one nearest real sample instead of holding last angle. */
				nearest_sample_index =
						as5600_analog_find_nearest_sample_index(
								corrected_deltas_counts,
								publish_raw_sample_count);
				published_mechanical_angle_u16 =
						s_as5600_analog_h->window_angle_samples_u16[nearest_sample_index];
			}
		}

		/* Publish one coherent angle sample for application consume. */
		s_as5600_analog_h->mechanical_angle_u16 = published_mechanical_angle_u16;
		s_as5600_analog_h->has_published_angle = true;

		/* Start a new raw window after publishing one angle sample. */
		s_as5600_analog_h->raw_sample_count = 0u;
		s_as5600_analog_h->window_reference_mechanical_angle_u16 = 0u;
		s_as5600_analog_h->window_delta_sum_counts = 0;
		for (i = 0u; i < AS5600_ANALOG_MAX_PUBLISH_WINDOW_SAMPLES; i++)
		{
			s_as5600_analog_h->window_angle_samples_u16[i] = 0u;
		}
		s_as5600_analog_h->has_new_angle_sample = true;
	}
}
