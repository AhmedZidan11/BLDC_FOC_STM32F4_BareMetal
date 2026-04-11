/**
 * @file motor_sine_3pwm.c
 * @brief Shared sine modulation helper for motor-domain 3-PWM actuation.
 *
 */

#include "motor/motor_sine_3pwm.h"
#include <stddef.h>

#define MOTOR_SINE_3PWM_LUT_SIZE          256u
#define MOTOR_SINE_3PWM_INDEX_SHIFT       8u
#define MOTOR_SINE_3PWM_PHASE_SHIFT_120   21845u
#define MOTOR_SINE_3PWM_PHASE_SHIFT_240   43690u
#define MOTOR_SINE_3PWM_LUT_MAX           32767
#define MOTOR_SINE_3PWM_DUTY_CENTER       5000
#define MOTOR_SINE_3PWM_DUTY_MAX          10000
#define MOTOR_SINE_3PWM_DUTY_SCALE_DENOM  (2 * MOTOR_SINE_3PWM_LUT_MAX)

/**
 * @brief Full-wave sine lookup table in signed Q15-like units.
 *
 * One electrical revolution is stored across 256 samples.
 */
static const int16_t motor_sine_3pwm_lut[MOTOR_SINE_3PWM_LUT_SIZE] = {
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
 * @brief Read one sine-table sample from one full-turn electrical angle.
 *
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @return Signed Q15-like sine sample.
 */
static int16_t motor_sine_3pwm_lut_get(uint16_t electrical_angle_u16)
{
	/* index = electrical_angle_u16 / 256 for the 256-point full-wave LUT. */
	uint16_t index = electrical_angle_u16 >> MOTOR_SINE_3PWM_INDEX_SHIFT;
	return motor_sine_3pwm_lut[index];
}

/**
 * @brief Convert one signed sine phase sample into one centered duty.
 *
 * @param phase_q15 Signed Q15-like sample in the range [-32767, 32767].
 * @param amplitude_permyriad Requested amplitude in permyriad units.
 * @return Duty command in permyriad units centered around 5000.
 */
static uint16_t motor_sine_3pwm_phase_to_duty_permyriad(int16_t phase_q15,
														 uint16_t amplitude_permyriad)
{
	/* Multiply the sine sample by the requested amplitude. */
	int32_t scaled = (int32_t)phase_q15 * (int32_t)amplitude_permyriad;
	int32_t offset = 0;

	/* Round the signed duty offset to the nearest permyriad step. */
	if (scaled >= 0)
	{
		offset = (scaled + MOTOR_SINE_3PWM_LUT_MAX) / MOTOR_SINE_3PWM_DUTY_SCALE_DENOM;
	}
	else
	{
		offset = (scaled - MOTOR_SINE_3PWM_LUT_MAX) / MOTOR_SINE_3PWM_DUTY_SCALE_DENOM;
	}

	int32_t duty_permyriad = MOTOR_SINE_3PWM_DUTY_CENTER + offset;
	if (duty_permyriad < 0)
	{
		duty_permyriad = 0;
	}
	else if (duty_permyriad > MOTOR_SINE_3PWM_DUTY_MAX)
	{
		duty_permyriad = MOTOR_SINE_3PWM_DUTY_MAX;
	}

	return (uint16_t)duty_permyriad;
}

bool motor_sine_3pwm_apply(motor_3pwm_handle_t *motor_3pwm_h,
						   uint16_t electrical_angle_u16,
						   uint16_t amplitude_permyriad)
{
	if (motor_3pwm_h == NULL) return false;
	if (amplitude_permyriad > MOTOR_SINE_3PWM_MAX_AMPLITUDE_PERMYRIAD) return false;

	/* Shift phase B and C by 120 and 240 electrical degrees. */
	uint16_t angle_a_u16 = electrical_angle_u16;
	uint16_t angle_b_u16 = (uint16_t)(electrical_angle_u16 + MOTOR_SINE_3PWM_PHASE_SHIFT_120);
	uint16_t angle_c_u16 = (uint16_t)(electrical_angle_u16 + MOTOR_SINE_3PWM_PHASE_SHIFT_240);

	/* Convert the three phase samples into centered PWM duties. */
	uint16_t duty_a = motor_sine_3pwm_phase_to_duty_permyriad(
			motor_sine_3pwm_lut_get(angle_a_u16), amplitude_permyriad);
	uint16_t duty_b = motor_sine_3pwm_phase_to_duty_permyriad(
			motor_sine_3pwm_lut_get(angle_b_u16), amplitude_permyriad);
	uint16_t duty_c = motor_sine_3pwm_phase_to_duty_permyriad(
			motor_sine_3pwm_lut_get(angle_c_u16), amplitude_permyriad);

	return motor_3pwm_set_duty_abc(motor_3pwm_h, duty_a, duty_b, duty_c);
}
