#ifndef DRIVERS_AS5600_ANALOG_H
#define DRIVERS_AS5600_ANALOG_H

/**
 * @file as5600_analog.h
 * @brief Narrow AS5600 analog-angle acquisition helper.
 *
 * This module owns AS5600-specific analog acquisition details above the
 * existing ADC driver.
 *
 * Responsibilities:
 * - schedule raw ADC sample acquisition
 * - accumulate and average raw ADC samples
 * - convert averaged raw ADC data into mechanical angle
 * - apply sensor-direction mapping
 * - publish one averaged mechanical angle sample at a slower fixed interval
 *
 * @note Mechanical angle uses full-turn uint16 units
 *       (0..65535 <=> 0..1 mechanical revolution).
 * @note All timing uses microseconds.
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/adc.h"

#define AS5600_ANALOG_MECHANICAL_ANGLE_FULL_TURN_U16  65535u

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
	uint32_t raw_sample_period_us;
	uint32_t angle_publish_period_us;
	as5600_analog_direction_t mechanical_angle_direction;
} as5600_analog_cfg_t;

/**
 * @brief AS5600 analog runtime handle.
 *
 */
typedef struct {
	const as5600_analog_cfg_t *cfg;
	uint32_t raw_accumulator;
	uint16_t raw_sample_count;
	uint16_t last_raw_averaged;
	uint16_t mechanical_angle_u16;
	uint64_t last_raw_sample_time_us;
	uint64_t last_angle_publish_time_us;
	bool has_new_angle_sample;
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
 * @brief Update AS5600 analog acquisition and averaged angle publication.
 *
 * @param as5600_analog_h Pointer to AS5600 analog handle.
 * @param now_us Current time in microseconds.
 * @return true if update succeeded, false otherwise.
 */
bool as5600_analog_update(as5600_analog_handle_t *as5600_analog_h,
						  uint64_t now_us);

#endif /* DRIVERS_AS5600_ANALOG_H */
