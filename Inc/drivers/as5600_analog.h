#ifndef DRIVERS_AS5600_ANALOG_H
#define DRIVERS_AS5600_ANALOG_H

/**
 * @file as5600_analog.h
 * @brief Sensor-specific AS5600 analog angle acquisition helper.
 *
 * This module keeps the AS5600 analog angle path separate from the
 * generic ADC driver.
 *
 * Responsibilities:
 * - start raw ADC conversions at a fixed period
 * - take completed raw ADC samples from the ADC IRQ path
 * - average a fixed raw-sample count
 * - convert the averaged raw data into mechanical angle
 * - apply sensor direction
 * - publish and consume mechanical angle samples
 *
 * @note Mechanical angle uses full-turn uint16 units
 *       (0..65535 <=> 0..1 mechanical revolution).
 * @note All timing uses microseconds.
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/adc.h"

#define AS5600_ANALOG_MECHANICAL_ANGLE_MAX_U16  65535u

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
	uint16_t raw_samples_per_publish;   /* Publish one averaged angle after this many raw samples. */
	as5600_analog_direction_t mechanical_angle_direction;
} as5600_analog_cfg_t;

/**
 * @brief AS5600 analog runtime handle.
 *
 */
typedef struct {
	const as5600_analog_cfg_t *cfg;
	volatile uint32_t raw_accumulator;  /* Sum of raw ADC codes in the current publish window. */
	volatile uint16_t raw_sample_count; /* Number of raw ADC codes in the current publish window. */
	volatile uint16_t mechanical_angle_u16;
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
 * @brief Consume one published averaged angle sample.
 *
 * This function returns the latest published sample once and then clears
 * the ready flag.
 *
 * @param as5600_analog_h Pointer to AS5600 analog handle.
 * @param mechanical_angle_u16 Pointer to the published mechanical angle output.
 * @return true if one new published sample was consumed, false otherwise.
 */
bool as5600_analog_consume_published_sample(as5600_analog_handle_t *as5600_analog_h,
											uint16_t *mechanical_angle_u16);

/**
 * @brief Consume one completed ADC sample for the active AS5600 analog handle.
 *
 * Call this from the ADC IRQ path after adc_irq_handler() updates the ADC
 * driver state.
 */
void as5600_analog_adc_irq_handler(void);

#endif /* DRIVERS_AS5600_ANALOG_H */
