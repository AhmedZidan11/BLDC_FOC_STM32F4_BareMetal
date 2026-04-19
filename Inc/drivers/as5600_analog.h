#ifndef DRIVERS_AS5600_ANALOG_H
#define DRIVERS_AS5600_ANALOG_H

/**
 * @file as5600_analog.h
 * @brief Sensor-specific AS5600 analog angle acquisition helper.
 *
 * This module keeps the AS5600 analog publish path separate from the
 * generic ADC driver and provides one consumed mechanical-angle sample
 * at fixed cadence.
 *
 * Publish behavior:
 * - first completed window: wrap-safe average bootstrap
 * - later windows: continuity-gated corrected-delta average
 * - fallback when no sample passes gate: nearest real sample
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/adc.h"

#define AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16  65535u
#define AS5600_ANALOG_MAX_PUBLISH_WINDOW_SAMPLES 16u

/**
 * @brief AS5600 analog mechanical-angle direction.
 *
 */
typedef enum {
	AS5600_ANALOG_DIRECTION_FORWARD = 1,
	AS5600_ANALOG_DIRECTION_REVERSE = -1,
} as5600_analog_direction_t;

/**
 * @brief AS5600 analog configuration.
 *
 */
typedef struct {
	adc_handle_t *adc_h;
	uint16_t adc_full_scale;
	uint32_t raw_sample_period_us;      /* Time between raw ADC start requests from SysTick. */
	uint16_t raw_samples_per_publish;   /* Publish one angle after this many raw samples. */
	uint16_t wrap_correction_threshold_counts; /* Threshold used for one wrap correction in corrected-delta conversion. */
	uint16_t max_plausible_delta_per_publish_counts; /* Continuity-gate threshold in corrected-delta domain. */
	as5600_analog_direction_t mechanical_angle_direction;
} as5600_analog_cfg_t;

/**
 * @brief One published AS5600 angle sample.
 *
 */
typedef struct {
	uint16_t mechanical_angle_u16;
} as5600_analog_published_sample_t;

/**
 * @brief AS5600 analog runtime handle.
 *
 */
typedef struct {
	const as5600_analog_cfg_t *cfg;
	volatile uint16_t raw_sample_count; /* Number of raw ADC codes in the current publish window. */
	volatile uint16_t window_reference_mechanical_angle_u16; /* First angle sample in the current publish window. */
	/* Ordered angle samples of the active publish window. */
	volatile uint16_t window_angle_samples_u16[AS5600_ANALOG_MAX_PUBLISH_WINDOW_SAMPLES];
	volatile int32_t window_delta_sum_counts; /* Sum of wrapped signed deltas relative to the window reference. */
	volatile uint16_t mechanical_angle_u16;
	volatile bool has_published_angle;
	uint64_t next_raw_sample_time_us;
	volatile bool raw_conversion_pending;
	volatile bool has_new_angle_sample; /* True when one published angle sample is ready for the main loop. */
	bool is_initialized;
} as5600_analog_handle_t;

/**
 * @brief Initialize AS5600 analog handle.
 *
 * @param as5600_analog_h Pointer to AS5600 analog handle.
 * @param as5600_analog_cfg Pointer to AS5600 analog configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool as5600_analog_init(as5600_analog_handle_t *as5600_analog_h,
						const as5600_analog_cfg_t *as5600_analog_cfg);

/**
 * @brief Service SysTick-based raw ADC conversion scheduling.
 *
 * This function only checks when the next raw conversion should start.
 * The ADC IRQ path still handles completed conversions.
 *
 * @param as5600_analog_h Pointer to AS5600 analog handle.
 * @param now_us Current time in microseconds.
 * @return true if scheduling state is valid, false otherwise.
 */
bool as5600_analog_service(as5600_analog_handle_t *as5600_analog_h,
						   uint64_t now_us);

/**
 * @brief Consume one published angle sample.
 *
 * This function returns the latest published sample once and then clears
 * the ready flag.
 *
 * @param as5600_analog_h Pointer to AS5600 analog handle.
 * @param published_sample Pointer to one published sample.
 * @return true if one new published sample was consumed, false otherwise.
 */
bool as5600_analog_consume_published_sample(as5600_analog_handle_t *as5600_analog_h,
											as5600_analog_published_sample_t *published_sample);

/**
 * @brief Consume one completed ADC sample for the active AS5600 analog handle.
 *
 * Call this from the ADC IRQ path after adc_irq_handler() updates the ADC
 * driver state.
 */
void as5600_analog_adc_irq_handler(void);

#endif /* DRIVERS_AS5600_ANALOG_H */
