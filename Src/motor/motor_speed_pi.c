/**
 * @file motor_speed_pi.c
 * @brief Shadow-mode digital PI controller for mechanical speed.
 *
 */

#include "motor/motor_speed_pi.h"
#include <stddef.h>

#define MOTOR_SPEED_PI_Q15_SCALE             32768LL
#define MOTOR_SPEED_PI_MS_PER_SECOND         1000LL

/**
 * @brief Divide one signed integer and round to nearest integer.
 *
 * @param numerator Signed numerator.
 * @param denominator Positive denominator.
 * @return Rounded signed integer quotient.
 */
static int64_t motor_speed_pi_divide_round_nearest(int64_t numerator,
												   int64_t denominator)
{
	if (numerator >= 0)
	{
		return (numerator + (denominator / 2LL)) / denominator;
	}

	return (numerator - (denominator / 2LL)) / denominator;
}

/**
 * @brief Clamp one signed integer into an inclusive range.
 *
 * @param value Input value.
 * @param minimum Minimum allowed value.
 * @param maximum Maximum allowed value.
 * @return Clamped value.
 */
static int32_t motor_speed_pi_clamp_i32(int64_t value,
										int32_t minimum,
										int32_t maximum)
{
	if (value < (int64_t)minimum)
	{
		return minimum;
	}

	if (value > (int64_t)maximum)
	{
		return maximum;
	}

	return (int32_t)value;
}

bool motor_speed_pi_init(motor_speed_pi_handle_t *motor_speed_pi_h,
						 const motor_speed_pi_cfg_t *motor_speed_pi_cfg)
{
	if ((motor_speed_pi_h == NULL) || (motor_speed_pi_cfg == NULL)) return false;
	if (motor_speed_pi_cfg->motor_h == NULL) return false;
	if (motor_speed_pi_cfg->update_period_ms == 0u) return false;
	if (motor_speed_pi_cfg->output_limit_permyriad == 0u) return false;

	motor_handle_t *motor_h = motor_speed_pi_cfg->motor_h;
	int64_t ki_dt_q15 =
			motor_speed_pi_divide_round_nearest(
					(int64_t)motor_speed_pi_cfg->ki_per_s_q15 *
					(int64_t)motor_speed_pi_cfg->update_period_ms,
					(int64_t)MOTOR_SPEED_PI_MS_PER_SECOND);

	/* Convert continuous-time Ki into one fixed-period discrete gain. */
	motor_speed_pi_h->cfg = motor_speed_pi_cfg;
	motor_speed_pi_h->motor_h = motor_h;
	motor_speed_pi_h->ki_dt_q15 = (int32_t)ki_dt_q15;
	motor_speed_pi_h->integrator_term_permyriad = 0;
	motor_speed_pi_h->is_initialized = true;

	motor_h->speed_pi.speed_error_mrpm = 0;
	motor_h->speed_pi.integrator_term_permyriad = 0;
	motor_h->speed_pi.shadow_uq_command_permyriad = 0;

	return true;
}

bool motor_speed_pi_reset(motor_speed_pi_handle_t *motor_speed_pi_h)
{
	if (motor_speed_pi_h == NULL) return false;
	if ((motor_speed_pi_h->cfg == NULL) || (motor_speed_pi_h->motor_h == NULL)) return false;
	if (motor_speed_pi_h->is_initialized == false) return false;

	/* Reset the PI runtime state and published shadow output. */
	motor_speed_pi_h->integrator_term_permyriad = 0;
	motor_speed_pi_h->motor_h->speed_pi.speed_error_mrpm = 0;
	motor_speed_pi_h->motor_h->speed_pi.integrator_term_permyriad = 0;
	motor_speed_pi_h->motor_h->speed_pi.shadow_uq_command_permyriad = 0;

	return true;
}

bool motor_speed_pi_update(motor_speed_pi_handle_t *motor_speed_pi_h,
						   int32_t target_mechanical_speed_mrpm,
						   int32_t measured_mechanical_speed_mrpm)
{
	if (motor_speed_pi_h == NULL) return false;
	if ((motor_speed_pi_h->cfg == NULL) || (motor_speed_pi_h->motor_h == NULL)) return false;
	if (motor_speed_pi_h->is_initialized == false) return false;

	motor_handle_t *motor_h = motor_speed_pi_h->motor_h;
	const int32_t output_limit = (int32_t)motor_speed_pi_h->cfg->output_limit_permyriad;
	int32_t speed_error_mrpm =
			target_mechanical_speed_mrpm - measured_mechanical_speed_mrpm;
	int64_t proportional_term_permyriad =
			motor_speed_pi_divide_round_nearest(
					(int64_t)motor_speed_pi_h->cfg->kp_q15 * (int64_t)speed_error_mrpm,
					(int64_t)MOTOR_SPEED_PI_Q15_SCALE);
	int64_t integrator_step_permyriad =
			motor_speed_pi_divide_round_nearest(
					(int64_t)motor_speed_pi_h->ki_dt_q15 * (int64_t)speed_error_mrpm,
					(int64_t)MOTOR_SPEED_PI_Q15_SCALE);
	int64_t integrator_candidate_permyriad =
			(int64_t)motor_speed_pi_h->integrator_term_permyriad +
			integrator_step_permyriad;
	int64_t output_candidate_permyriad =
			proportional_term_permyriad + integrator_candidate_permyriad;

	/* Apply conditional integration when saturation would grow in the same direction as error. */
	if (((output_candidate_permyriad > (int64_t)output_limit) && (speed_error_mrpm > 0)) ||
		((output_candidate_permyriad < (int64_t)(-output_limit)) && (speed_error_mrpm < 0)))
	{
		integrator_candidate_permyriad = (int64_t)motor_speed_pi_h->integrator_term_permyriad;
		output_candidate_permyriad = proportional_term_permyriad + integrator_candidate_permyriad;
	}

	/* Keep integrator clamping separate from final output clamping for clean anti-windup state. */
	int32_t clamped_integrator_permyriad =
			motor_speed_pi_clamp_i32(integrator_candidate_permyriad, -output_limit, output_limit);
	int32_t shadow_uq_command_permyriad =
			motor_speed_pi_clamp_i32(output_candidate_permyriad, -output_limit, output_limit);

	/* Publish the shadow PI state for observation without driving actuation. */
	motor_speed_pi_h->integrator_term_permyriad = clamped_integrator_permyriad;
	motor_h->speed_pi.speed_error_mrpm = speed_error_mrpm;
	motor_h->speed_pi.integrator_term_permyriad = clamped_integrator_permyriad;
	motor_h->speed_pi.shadow_uq_command_permyriad = shadow_uq_command_permyriad;

	return true;
}
