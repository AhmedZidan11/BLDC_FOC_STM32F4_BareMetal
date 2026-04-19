#ifndef CONFIG_APP_MOTOR_TEST_CONFIG_H
#define CONFIG_APP_MOTOR_TEST_CONFIG_H

/**
 * @file app_motor_test_config.h
 * @brief Current motor test application configuration.
 *
 * This header stores the chosen setup values for the active sensored
 * closed-loop speed-control application with startup alignment.
 */

#define APP_MOTOR_TEST_POLE_PAIRS                                   7u
#define APP_MOTOR_TEST_STARTUP_OPENLOOP_TARGET_MECHANICAL_SPEED_MRPM 60000
/* Sensor polarity sign used only for angle reconstruction from AS5600 ADC. */
#define APP_MOTOR_TEST_SENSOR_DIRECTION                             (-1)
/* Phase-sequence sign for q-axis handedness in voltage-mode FOC actuation. */
#define APP_MOTOR_TEST_PHASE_SEQUENCE_SIGN                          (-1)
/* Control-positive mechanical speed sign used by speed control and speed logs. */
#define APP_MOTOR_TEST_CONTROL_DIRECTION_SIGN                       (-1)
#define APP_MOTOR_TEST_ALIGNMENT_AMPLITUDE_PERMYRIAD                1500u
#define APP_MOTOR_TEST_MAX_AMPLITUDE_PERMYRIAD                      10000u
#define APP_MOTOR_TEST_UPDATE_PERIOD_MS                             1u
#define APP_MOTOR_TEST_STARTUP_OPENLOOP_PHASE_INCREMENT_RAMP_STEP_U32 1u
#define APP_MOTOR_TEST_ALIGNMENT_DURATION_MS                        400u
#define APP_MOTOR_TEST_ALIGNMENT_ELECTRICAL_ANGLE_U16               0u
#define APP_MOTOR_TEST_ANGLE_ADC_SAMPLE_PERIOD_US                   100u
#define APP_MOTOR_TEST_ANGLE_PUBLISH_RAW_SAMPLE_COUNT               5u
/* Lightweight UART telemetry output interval for runtime tuning logs. */
#define APP_MOTOR_TEST_TELEMETRY_PERIOD_MS                          20u
#define APP_MOTOR_TEST_ANGLE_FULL_TURN_COUNTS                       65536u
/* Half-turn threshold used for one wrap correction in corrected-delta conversion. */
#define APP_MOTOR_TEST_ANGLE_HALF_TURN_COUNTS                       (APP_MOTOR_TEST_ANGLE_FULL_TURN_COUNTS / 2u)
/* Maximum plausible mechanical step per 500 us publish interval (~8 mechanical degrees). */
#define APP_MOTOR_TEST_ANGLE_MAX_PLAUSIBLE_DELTA_PER_PUBLISH_COUNTS ((APP_MOTOR_TEST_ANGLE_FULL_TURN_COUNTS * 8u) / 360u)
#define APP_MOTOR_TEST_FAULT_TRIGGER_ABS_SPEED_ERROR_MRPM          100000
/* Speed-feedback LPF time constant tau (larger tau => smoother, smaller tau => faster). */
#define APP_MOTOR_TEST_SPEED_FEEDBACK_FILTER_TIME_CONSTANT_MS       30u
/* Speed PI gains use Q15 fixed-point on mrpm input and permyriad output. */
#define APP_MOTOR_TEST_SPEED_PI_KP_Q15                              2048
/* Ki is zero temporarily so the first closed-loop test runs in P-only mode. */
#define APP_MOTOR_TEST_SPEED_PI_KI_PER_S_Q15                        0
#define APP_MOTOR_TEST_SPEED_PI_OUTPUT_LIMIT_PERMYRIAD              6000u
#define APP_MOTOR_TEST_SPEED_PI_UPDATE_PERIOD_MS                    1u
/* First tuning profile: unidirectional trapezoid 0 -> peak -> 0 (repeat). */
#define APP_MOTOR_TEST_SPEED_PROFILE_PEAK_MECHANICAL_SPEED_MRPM     300000
#define APP_MOTOR_TEST_SPEED_PROFILE_ACCELERATION_MRPM_PER_S        300000
#define APP_MOTOR_TEST_SPEED_PROFILE_ZERO_HOLD_MS                   500u
#define APP_MOTOR_TEST_SPEED_PROFILE_PEAK_HOLD_MS                   1000u
#define APP_MOTOR_TEST_SPEED_REFERENCE_ESTIMATOR_HISTORY_SAMPLE_COUNT 50u
#define APP_MOTOR_TEST_AS5600_ADC_FULL_SCALE                        4095u

#endif /* CONFIG_APP_MOTOR_TEST_CONFIG_H */
