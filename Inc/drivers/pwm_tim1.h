#ifndef DRIVERS_PWM_TIM1_H_
#define DRIVERS_PWM_TIM1_H_

/**
 * @file pwm_tim1.h
 * @brief Minimal PWM module for STM32F4 (CMSIS only).
 *
 * Provides register-level configuration.
 * Assumes TIM1 outputs are configured in PWM mode 1.
 * This driver outputs 3 independent PWM signals intended for 3-PWM mode on the power stage.
 * No complementary/dead-time generation is used.
 *
 * Responsibilities:
 * - Configure TIM1 registers
 * - Set PWM duty-cycles
 *
 * @note Current limitation: center-aligned mode only (CMS=1..3). Edge-aligned support may be added later.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f4xx.h"
#include "drivers/gpio.h"

/**
 * @brief Alignment mode (maps to TIM1 CR1 CMS bits; edge-aligned uses CMS=0).
 */
typedef enum {
	PWM_ALIGN_EDGE = 0,
	PWM_ALIGN_CENTER_1 = 1,
	PWM_ALIGN_CENTER_2 = 2,
	PWM_ALIGN_CENTER_3 = 3,
} pwm_align_t;

/**
 * @brief TIM1 configuration (PWM mode)
 *
 * @pre pin_ch1,2,3 must describe correct AF mapping. Driver will call gpio_init_pin.
 * Each pin pointer may be NULL to disable the corresponding channel.
 * The pointed-to configs must remain valid for the lifetime of the driver.
 */
typedef struct {
	uint32_t tim_clk_hz;	// actual TIM1 counter clock
	uint32_t pwm_hz;
	pwm_align_t align;

	const gpio_pin_cfg_t* pin_ch1;
	const gpio_pin_cfg_t* pin_ch2;
	const gpio_pin_cfg_t* pin_ch3;
} pwm_tim1_cfg_t;

/**
 * @brief Handle for TIM1 (PWM mode).
 *
 */
typedef struct {
	uint16_t arr;		// Auto-reload value
} pwm_tim1_handle_t;

/**
 * @brief Configure TIM1 registers according to the given configuration instance.
 *
 * @param pwm_cfg Pointer to the TIM1 configuration instance (PWM mode).
 * @param pwm_h	Pointer to the TIM1 handle instance (PWM mode).
 * @return true if applied, false if parameters invalid.
 */
bool pwm_tim1_init(const pwm_tim1_cfg_t* pwm_cfg, pwm_tim1_handle_t* pwm_h);

/**
 * @brief Start counter for TIM1 and enable output gate.
 *
 * @param pwm_h Pointer to the TIM1 handle instance.
 * @return true if applied, false if parameters invalid.
 */
bool pwm_tim1_start(pwm_tim1_handle_t* pwm_h);

/**
 * @brief Stop counter for TIM1 and disable output gate.
 *
 * @param pwm_h Pointer to the TIM1 handle instance.
 * @return true if applied, false if parameters invalid.
 */
bool pwm_tim1_stop(pwm_tim1_handle_t* pwm_h);

/**
 * @brief Load TIM1_CCRx according to the channel number and duty value.
 *
 * @param pwm_h Pointer to the TIM1 handle instance.
 * @param ch Channel number.
 * @param duty compare value in timer ticks (0..ARR).
 * @return true if applied, false if parameters invalid.
 */
bool pwm_tim1_set_duty(pwm_tim1_handle_t* pwm_h, uint8_t ch, uint16_t duty);

#endif /* DRIVERS_PWM_TIM1_H_ */
