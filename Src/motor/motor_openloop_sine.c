/**
 * @file motor_openloop_sine.c
 * @brief Open-loop sine control layer above motor_3pwm.
 *
 */

#include "motor/motor_openloop_sine.h"
#include <stddef.h>

#define MOTOR_OPENLOOP_SINE_LUT_SIZE          256u
#define MOTOR_OPENLOOP_SINE_INDEX_SHIFT       8u
#define MOTOR_OPENLOOP_SINE_PHASE_SHIFT_120   21845u
#define MOTOR_OPENLOOP_SINE_PHASE_SHIFT_240   43690u
#define MOTOR_OPENLOOP_SINE_LUT_MAX           32767
#define MOTOR_OPENLOOP_SINE_DUTY_CENTER       5000
#define MOTOR_OPENLOOP_SINE_DUTY_MAX          10000
#define MOTOR_OPENLOOP_SINE_DUTY_SCALE_DENOM  (2 * MOTOR_OPENLOOP_SINE_LUT_MAX)

/**
 * @brief Full-wave sine lookup table in signed Q15-like units.
 *
 * One electrical revolution is stored across 256 samples.
 * Table values use the range [-32767, 32767], where 32767 corresponds to +1.0.
 */
static const int16_t motor_openloop_sine_lut[MOTOR_OPENLOOP_SINE_LUT_SIZE] = {
	     0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
	  6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
	 12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
	 18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
	 23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
	 27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
	 30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
	 32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
	 32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
	 32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
	 30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
	 27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
	 23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
	 18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
	 12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
	  6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
	     0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
	 -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
	-12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
	-18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
	-23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
	-27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
	-30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
	-32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
	-32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
	-32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
	-30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
	-27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
	-23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
	-18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
	-12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
	 -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804,
};

/**
 * @brief Read one sine sample from the full-wave LUT.
 *
 * The uint16 electrical angle uses one full turn across 0..65535.
 * The upper 8 bits select one of 256 full-wave LUT entries.
 *
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @return Signed normalized sine value in the range [-32767, 32767].
 */
static int16_t motor_openloop_sine_lut_get(uint16_t electrical_angle_u16)
{
	uint16_t index = electrical_angle_u16 >> MOTOR_OPENLOOP_SINE_INDEX_SHIFT;
	return motor_openloop_sine_lut[index];
}

/**
 * @brief Convert one LUT phase value to 3-PWM duty in permyriad.
 *
 * LUT values are signed Q15-like units in the range [-32767, 32767].
 * Duty is centered around 5000 permyriad and scaled by the requested amplitude.
 *
 * @param phase_q15 Signed normalized phase value from the sine LUT.
 * @param amplitude_permyriad Vector amplitude in permyriad units (0..10000).
 * @return Duty in permyriad units (0..10000).
 */
static uint16_t motor_openloop_sine_phase_to_duty_permyriad(int16_t phase_q15,
									 uint16_t amplitude_permyriad)
{
	int32_t scaled = (int32_t)phase_q15 * (int32_t)amplitude_permyriad;
	int32_t offset;

	if (scaled >= 0)
	{
		offset = (scaled + MOTOR_OPENLOOP_SINE_LUT_MAX) / MOTOR_OPENLOOP_SINE_DUTY_SCALE_DENOM;
	}
	else
	{
		offset = (scaled - MOTOR_OPENLOOP_SINE_LUT_MAX) / MOTOR_OPENLOOP_SINE_DUTY_SCALE_DENOM;
	}

	int32_t duty_permyriad = MOTOR_OPENLOOP_SINE_DUTY_CENTER + offset;

	if (duty_permyriad < 0)
	{
		duty_permyriad = 0;
	}
	else if (duty_permyriad > MOTOR_OPENLOOP_SINE_DUTY_MAX)
	{
		duty_permyriad = MOTOR_OPENLOOP_SINE_DUTY_MAX;
	}

	return (uint16_t)duty_permyriad;
}

bool motor_openloop_sine_init(motor_openloop_sine_handle_t *motor_openloop_sine_h,
							  const motor_openloop_sine_cfg_t *motor_openloop_sine_cfg)
{
	if ((motor_openloop_sine_h == NULL) || (motor_openloop_sine_cfg == NULL)) return false;
	if (motor_openloop_sine_cfg->motor_3pwm_h == NULL) return false;

	motor_openloop_sine_h->cfg = motor_openloop_sine_cfg;
	motor_openloop_sine_h->last_electrical_angle_u16 = 0u;
	motor_openloop_sine_h->last_amplitude_permyriad = 0u;
	motor_openloop_sine_h->is_initialized = true;

	return true;
}

bool motor_openloop_sine_apply(motor_openloop_sine_handle_t *motor_openloop_sine_h,
						   uint16_t electrical_angle_u16,
						   uint16_t amplitude_permyriad)
{
	if (motor_openloop_sine_h == NULL) return false;
	if ((motor_openloop_sine_h->cfg == NULL) || (motor_openloop_sine_h->cfg->motor_3pwm_h == NULL)) return false;
	if (motor_openloop_sine_h->is_initialized == false) return false;
	if (amplitude_permyriad > 10000u) return false;

	/* 120° and 240° shifts expressed in full-turn uint16 angle units. */
	uint16_t angle_a_u16 = electrical_angle_u16;
	uint16_t angle_b_u16 = (uint16_t)(electrical_angle_u16 + MOTOR_OPENLOOP_SINE_PHASE_SHIFT_120);
	uint16_t angle_c_u16 = (uint16_t)(electrical_angle_u16 + MOTOR_OPENLOOP_SINE_PHASE_SHIFT_240);

	uint16_t duty_a = motor_openloop_sine_phase_to_duty_permyriad(motor_openloop_sine_lut_get(angle_a_u16), amplitude_permyriad);
	uint16_t duty_b = motor_openloop_sine_phase_to_duty_permyriad(motor_openloop_sine_lut_get(angle_b_u16), amplitude_permyriad);
	uint16_t duty_c = motor_openloop_sine_phase_to_duty_permyriad(motor_openloop_sine_lut_get(angle_c_u16), amplitude_permyriad);

	if (!motor_3pwm_set_duty_abc(motor_openloop_sine_h->cfg->motor_3pwm_h, duty_a, duty_b, duty_c)) return false;

	motor_openloop_sine_h->last_electrical_angle_u16 = electrical_angle_u16;
	motor_openloop_sine_h->last_amplitude_permyriad = amplitude_permyriad;
	return true;
}
