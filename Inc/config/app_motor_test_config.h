#ifndef CONFIG_APP_MOTOR_TEST_CONFIG_H
#define CONFIG_APP_MOTOR_TEST_CONFIG_H

/**
 * @file app_motor_test_config.h
 * @brief Current motor test application configuration.
 *
 * This header stores the chosen setup values for the active open-loop motor
 * test application.
 */

#define APP_MOTOR_TEST_POLE_PAIRS                                   7u
#define APP_MOTOR_TEST_TARGET_MECHANICAL_SPEED_MRPM                 60000
/* Sensor polarity sign for mechanical angle reconstruction. */
#define APP_MOTOR_TEST_SENSOR_DIRECTION                             (-1)
/* Phase-sequence sign for q-axis direction: +1 => +90 deg, -1 => -90 deg. */
#define APP_MOTOR_TEST_PHASE_SEQUENCE_SIGN                          (-1)
/* Control-positive mechanical direction sign used by speed targets and logs. */
#define APP_MOTOR_TEST_CONTROL_DIRECTION_SIGN                       (-1)
#define APP_MOTOR_TEST_ALIGNMENT_AMPLITUDE_PERMYRIAD                1500u
#define APP_MOTOR_TEST_RUN_AMPLITUDE_PERMYRIAD                      2000u
#define APP_MOTOR_TEST_MAX_AMPLITUDE_PERMYRIAD                      10000u
#define APP_MOTOR_TEST_UPDATE_PERIOD_MS                             1u
#define APP_MOTOR_TEST_RAMP_DURATION_MS                             300u
#define APP_MOTOR_TEST_ALIGNMENT_DURATION_MS                        400u
#define APP_MOTOR_TEST_ALIGNMENT_ELECTRICAL_ANGLE_U16               0u
#define APP_MOTOR_TEST_FOC_UQ_TARGET_PERMYRIAD                      2000u
#define APP_MOTOR_TEST_FOC_UQ_RAMP_DURATION_MS                      300u
#define APP_MOTOR_TEST_ANGLE_ADC_SAMPLE_PERIOD_US                   100u
#define APP_MOTOR_TEST_ANGLE_PUBLISH_RAW_SAMPLE_COUNT               5u
#define APP_MOTOR_TEST_ANGLE_LOG_PERIOD_MS                          100u
/* Speed-feedback LPF time constant tau (larger tau => smoother, smaller tau => faster). */
#define APP_MOTOR_TEST_SPEED_FEEDBACK_FILTER_TIME_CONSTANT_MS       30u
/* Speed PI gains use Q15 fixed-point on mrpm input and permyriad output. */
#define APP_MOTOR_TEST_SPEED_PI_KP_Q15                              1024
/* Ki is zero temporarily so shadow PI behaves as a simpler P-only test. */
#define APP_MOTOR_TEST_SPEED_PI_KI_PER_S_Q15                        0
#define APP_MOTOR_TEST_SPEED_PI_OUTPUT_LIMIT_PERMYRIAD              3000u
#define APP_MOTOR_TEST_SPEED_PI_UPDATE_PERIOD_MS                    1u
/* Shadow target is kept near current runtime speed scale to avoid blind saturation. */
#define APP_MOTOR_TEST_SPEED_PI_SHADOW_TARGET_MECHANICAL_SPEED_MRPM 200000
#define APP_MOTOR_TEST_SPEED_REFERENCE_ESTIMATOR_HISTORY_SAMPLE_COUNT 50u
#define APP_MOTOR_TEST_AS5600_ADC_FULL_SCALE                        4095u

#endif /* CONFIG_APP_MOTOR_TEST_CONFIG_H */
