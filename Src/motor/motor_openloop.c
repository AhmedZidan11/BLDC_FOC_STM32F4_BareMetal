/**
 * @file motor_openloop.c
 * @brief Combined open-loop actuation helper above motor_3pwm.
 *
 */

#include "motor/motor_openloop.h"
#include <stddef.h>

#define MOTOR_OPENLOOP_LUT_SIZE                 256u
#define MOTOR_OPENLOOP_INDEX_SHIFT              8u
#define MOTOR_OPENLOOP_PHASE_SHIFT_120          21845u
#define MOTOR_OPENLOOP_PHASE_SHIFT_240          43690u
#define MOTOR_OPENLOOP_LUT_MAX                  32767
#define MOTOR_OPENLOOP_DUTY_CENTER              5000
#define MOTOR_OPENLOOP_DUTY_MAX                 10000
#define MOTOR_OPENLOOP_DUTY_SCALE_DENOM         (2 * MOTOR_OPENLOOP_LUT_MAX)
#define MOTOR_OPENLOOP_MRPM_PER_RPM             1000LL
#define MOTOR_OPENLOOP_MAX_AMPLITUDE_PERMYRIAD  10000u

/**
 * @brief Full-wave sine lookup table in signed Q15-like units.
 *
 * One electrical revolution is stored across 256 samples.
 * Table values use the range [-32767, 32767], where 32767 corresponds to +1.0.
 */
static const int16_t motor_openloop_lut[MOTOR_OPENLOOP_LUT_SIZE] = {
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
 * @brief Read one sine-table sample from a full-turn electrical angle.
 *
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @return Signed Q15-like sine sample for the requested electrical angle.
 */
static int16_t motor_openloop_lut_get(uint16_t electrical_angle_u16)
{
	/* index = electrical_angle_u16 / 256 for the 256-point full-wave LUT. */
	uint16_t index = electrical_angle_u16 >> MOTOR_OPENLOOP_INDEX_SHIFT;
	return motor_openloop_lut[index];
}

/**
 * @brief Convert one signed sine phase sample into one centered duty command.
 *
 * @param phase_q15 Signed Q15-like LUT sample in the range [-32767, 32767].
 * @param amplitude_permyriad Open-loop amplitude command in permyriad units.
 * @return Duty command in permyriad units centered around 5000.
 */
static uint16_t motor_openloop_phase_to_duty_permyriad(int16_t phase_q15,
														uint16_t amplitude_permyriad)
{
	/* Multiply the sine sample by the requested amplitude. */
	int32_t scaled = (int32_t)phase_q15 * (int32_t)amplitude_permyriad;
	int32_t offset;

	/* Round the signed duty offset to the nearest permyriad step. */
	if (scaled >= 0)
	{
		offset = (scaled + MOTOR_OPENLOOP_LUT_MAX) / MOTOR_OPENLOOP_DUTY_SCALE_DENOM;
	}
	else
	{
		offset = (scaled - MOTOR_OPENLOOP_LUT_MAX) / MOTOR_OPENLOOP_DUTY_SCALE_DENOM;
	}

	int32_t duty_permyriad = MOTOR_OPENLOOP_DUTY_CENTER + offset;

	if (duty_permyriad < 0)
	{
		duty_permyriad = 0;
	}
	else if (duty_permyriad > MOTOR_OPENLOOP_DUTY_MAX)
	{
		duty_permyriad = MOTOR_OPENLOOP_DUTY_MAX;
	}

	return (uint16_t)duty_permyriad;
}

/**
 * @brief Apply one electrical angle through the 3-PWM stage.
 *
 * @param motor_openloop_h Pointer to open-loop handle.
 * @param electrical_angle_u16 Electrical angle in full-turn uint16 units.
 * @return true if the three duty commands were applied successfully.
 */
static bool motor_openloop_apply_electrical_angle(motor_openloop_handle_t *motor_openloop_h,
												  uint16_t electrical_angle_u16)
{
	if ((motor_openloop_h == NULL) || (motor_openloop_h->motor_h == NULL)) return false;
	if (motor_openloop_h->motor_3pwm_h == NULL) return false;
	if (motor_openloop_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_openloop_h->motor_h;
	uint16_t amplitude_permyriad = motor_h->targets.target_amplitude_permyriad;
	if (amplitude_permyriad > motor_h->limits.max_amplitude_permyriad) return false;

	/* Shift the other two phases by 120 and 240 electrical degrees. */
	uint16_t angle_a_u16 = electrical_angle_u16;
	uint16_t angle_b_u16 = (uint16_t)(electrical_angle_u16 + MOTOR_OPENLOOP_PHASE_SHIFT_120);
	uint16_t angle_c_u16 = (uint16_t)(electrical_angle_u16 + MOTOR_OPENLOOP_PHASE_SHIFT_240);

	/* Convert the three phase samples into centered PWM duties. */
	uint16_t duty_a = motor_openloop_phase_to_duty_permyriad(motor_openloop_lut_get(angle_a_u16), amplitude_permyriad);
	uint16_t duty_b = motor_openloop_phase_to_duty_permyriad(motor_openloop_lut_get(angle_b_u16), amplitude_permyriad);
	uint16_t duty_c = motor_openloop_phase_to_duty_permyriad(motor_openloop_lut_get(angle_c_u16), amplitude_permyriad);

	if (!motor_3pwm_set_duty_abc(motor_openloop_h->motor_3pwm_h, duty_a, duty_b, duty_c)) return false;

	/* Store only the last commanded electrical angle for the open-loop path. */
	motor_h->openloop.last_electrical_angle_u16 = electrical_angle_u16;
	return true;
}

bool motor_openloop_init(motor_openloop_handle_t *motor_openloop_h,
						 const motor_openloop_cfg_t *motor_openloop_cfg)
{
	if ((motor_openloop_h == NULL) || (motor_openloop_cfg == NULL)) return false;
	if (motor_openloop_cfg->motor_h == NULL) return false;
	if (motor_openloop_cfg->motor_3pwm_h == NULL) return false;
	if (motor_openloop_cfg->update_period_ms == 0u) return false;
	if (motor_openloop_cfg->phase_increment_ramp_step_u32 == 0u) return false;

	motor_handle_t *motor_h = motor_openloop_cfg->motor_h;
	if (motor_h->limits.pole_pairs == 0u)
	{
		motor_h->limits.pole_pairs = MOTOR_OPENLOOP_DEFAULT_TEST_POLE_PAIRS;
	}
	if (motor_h->limits.max_amplitude_permyriad == 0u)
	{
		motor_h->limits.max_amplitude_permyriad = MOTOR_OPENLOOP_MAX_AMPLITUDE_PERMYRIAD;
	}
	if (motor_h->limits.max_amplitude_permyriad > MOTOR_OPENLOOP_MAX_AMPLITUDE_PERMYRIAD) return false;
	if (motor_h->targets.target_amplitude_permyriad > motor_h->limits.max_amplitude_permyriad) return false;
	if (motor_h->targets.target_mechanical_speed_mrpm < 0) return false;

	/* max_speed = 60000 * 1000 / (pole_pairs * update_period_ms * min_updates_per_turn). */
	uint64_t max_speed_denom = (uint64_t)motor_h->limits.pole_pairs *
							   (uint64_t)motor_openloop_cfg->update_period_ms *
							   (uint64_t)MOTOR_OPENLOOP_MIN_UPDATES_PER_ELECTRICAL_TURN;
	motor_h->limits.max_target_mechanical_speed_mrpm =
			(int32_t)((MOTOR_OPENLOOP_MS_PER_MINUTE * MOTOR_OPENLOOP_MRPM_PER_RPM) / max_speed_denom);
	if (motor_h->targets.target_mechanical_speed_mrpm > motor_h->limits.max_target_mechanical_speed_mrpm) return false;

	/* phase_increment = speed_mrpm * pole_pairs * dt_ms * 2^32 / (60000 * 1000). */
	uint64_t phase_increment_num = (uint64_t)motor_h->targets.target_mechanical_speed_mrpm *
								   (uint64_t)motor_h->limits.pole_pairs *
								   (uint64_t)motor_openloop_cfg->update_period_ms *
								   MOTOR_OPENLOOP_PHASE_ACCUM_FULL_TURN_U32;
	uint32_t target_phase_increment_u32 = (uint32_t)((phase_increment_num +
													 ((MOTOR_OPENLOOP_MS_PER_MINUTE * MOTOR_OPENLOOP_MRPM_PER_RPM) / 2u)) /
													 (MOTOR_OPENLOOP_MS_PER_MINUTE * MOTOR_OPENLOOP_MRPM_PER_RPM));

	motor_openloop_h->cfg = motor_openloop_cfg;
	motor_openloop_h->motor_h = motor_h;
	motor_openloop_h->motor_3pwm_h = motor_openloop_cfg->motor_3pwm_h;
	motor_h->openloop.phase_accumulator_u32 = 0u;
	motor_h->openloop.current_phase_increment_u32 = 0u;
	motor_h->openloop.target_phase_increment_u32 = target_phase_increment_u32;
	motor_h->openloop.last_electrical_angle_u16 = 0u;
	motor_h->measurements.electrical_angle_u16 = 0u;
	motor_h->status.mode = MOTOR_MODE_INACTIVE;
	motor_h->status.is_initialized = true;
	motor_h->status.has_valid_electrical_angle = false;
	motor_openloop_h->is_initialized = true;

	return true;
}

bool motor_openloop_apply(motor_openloop_handle_t *motor_openloop_h,
						  uint16_t electrical_angle_u16)
{
	if (!motor_openloop_apply_electrical_angle(motor_openloop_h, electrical_angle_u16)) return false;

	motor_handle_t *motor_h = motor_openloop_h->motor_h;
	/* Put the applied electrical angle into the shared 32-bit phase accumulator. */
	motor_h->openloop.phase_accumulator_u32 =
			((uint32_t)electrical_angle_u16 << MOTOR_OPENLOOP_PHASE_ACCUM_ANGLE_SHIFT);
	motor_h->status.mode = MOTOR_MODE_OPENLOOP_STATIC;
	return true;
}

bool motor_openloop_update(motor_openloop_handle_t *motor_openloop_h)
{
	if (motor_openloop_h == NULL) return false;
	if ((motor_openloop_h->cfg == NULL) || (motor_openloop_h->motor_h == NULL)) return false;
	if (motor_openloop_h->motor_3pwm_h == NULL) return false;
	if (motor_openloop_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_openloop_h->motor_h;

	/* Move the active phase increment toward the target increment. */
	if (motor_h->openloop.current_phase_increment_u32 < motor_h->openloop.target_phase_increment_u32)
	{
		uint32_t increment_delta = motor_h->openloop.target_phase_increment_u32 -
								   motor_h->openloop.current_phase_increment_u32;
		if (increment_delta > motor_openloop_h->cfg->phase_increment_ramp_step_u32)
		{
			increment_delta = motor_openloop_h->cfg->phase_increment_ramp_step_u32;
		}

		motor_h->openloop.current_phase_increment_u32 += increment_delta;
	}
	else if (motor_h->openloop.current_phase_increment_u32 > motor_h->openloop.target_phase_increment_u32)
	{
		uint32_t increment_delta = motor_h->openloop.current_phase_increment_u32 -
								   motor_h->openloop.target_phase_increment_u32;
		if (increment_delta > motor_openloop_h->cfg->phase_increment_ramp_step_u32)
		{
			increment_delta = motor_openloop_h->cfg->phase_increment_ramp_step_u32;
		}

		motor_h->openloop.current_phase_increment_u32 -= increment_delta;
	}

	/* Advance the phase accumulator and use the upper 16 bits as electrical angle. */
	uint32_t next_phase_accumulator_u32 = motor_h->openloop.phase_accumulator_u32 +
										  motor_h->openloop.current_phase_increment_u32;
	uint16_t next_electrical_angle_u16 = (uint16_t)(next_phase_accumulator_u32 >> MOTOR_OPENLOOP_PHASE_ACCUM_ANGLE_SHIFT);

	if (!motor_openloop_apply_electrical_angle(motor_openloop_h, next_electrical_angle_u16)) return false;

	motor_h->openloop.phase_accumulator_u32 = next_phase_accumulator_u32;
	motor_h->status.mode = MOTOR_MODE_OPENLOOP_DRIVE;
	return true;
}
