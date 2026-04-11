#ifndef MOTOR_MOTOR_ELECTRICAL_ANGLE_H
#define MOTOR_MOTOR_ELECTRICAL_ANGLE_H

/**
 * @file motor_electrical_angle.h
 * @brief simple helper that converts measured mechanical angle into measured electrical angle.
 *
 * This module publishes the observed electrical angle used by feedback control.
 * It is separate from the open-loop commanded electrical angle.
 *
 * Responsibilities:
 * - store electrical-angle configuration
 * - connect to shared motor-domain state
 * - convert measured mechanical angle into measured electrical angle
 * - apply one configurable electrical offset
 * - update shared measured electrical-angle state
 *
 * @note Mechanical and electrical angles use full-turn uint16 units.
 */

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor.h"

/**
 * @brief Electrical-angle helper configuration.
 *
 */
typedef struct {
	motor_handle_t *motor_h;
	uint16_t electrical_offset_u16;
} motor_electrical_angle_cfg_t;

/**
 * @brief Electrical-angle helper runtime handle.
 *
 */
typedef struct {
	const motor_electrical_angle_cfg_t *cfg;
	motor_handle_t *motor_h;
	uint16_t electrical_offset_u16;
	bool is_initialized;
} motor_electrical_angle_handle_t;

/**
 * @brief Initialize the electrical-angle helper.
 *
 * @param motor_electrical_angle_h Pointer to electrical-angle handle.
 * @param motor_electrical_angle_cfg Pointer to electrical-angle configuration.
 * @return true if initialization succeeded, false otherwise.
 */
bool motor_electrical_angle_init(motor_electrical_angle_handle_t *motor_electrical_angle_h,
								 const motor_electrical_angle_cfg_t *motor_electrical_angle_cfg);

/**
 * @brief Update measured electrical angle from one measured mechanical angle sample.
 *
 * @param motor_electrical_angle_h Pointer to electrical-angle handle.
 * @param mechanical_angle_u16 Measured mechanical angle in full-turn uint16 units.
 * @return true if update succeeded, false otherwise.
 */
bool motor_electrical_angle_update(motor_electrical_angle_handle_t *motor_electrical_angle_h,
								   uint16_t mechanical_angle_u16);

/**
 * @brief Compute raw electrical angle from one measured mechanical angle sample.
 *
 * This conversion applies pole pairs only and does not apply electrical offset.
 *
 * @param motor_electrical_angle_h Pointer to electrical-angle handle.
 * @param mechanical_angle_u16 Measured mechanical angle in full-turn uint16 units.
 * @param electrical_angle_u16 Pointer to raw electrical angle output.
 * @return true if conversion succeeded, false otherwise.
 */
bool motor_electrical_angle_compute_raw(motor_electrical_angle_handle_t *motor_electrical_angle_h,
										uint16_t mechanical_angle_u16,
										uint16_t *electrical_angle_u16);

/**
 * @brief Set the electrical offset used for measured electrical angle conversion.
 *
 * @param motor_electrical_angle_h Pointer to electrical-angle handle.
 * @param electrical_offset_u16 Electrical offset in full-turn uint16 units.
 * @return true if the offset was updated, false otherwise.
 */
bool motor_electrical_angle_set_offset(motor_electrical_angle_handle_t *motor_electrical_angle_h,
									   uint16_t electrical_offset_u16);

#endif /* MOTOR_MOTOR_ELECTRICAL_ANGLE_H */
