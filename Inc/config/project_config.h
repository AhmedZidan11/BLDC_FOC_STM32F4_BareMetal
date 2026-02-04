#ifndef DRIVERS_PROJECT_CONFIG_H
#define DRIVERS_PROJECT_CONFIG_H

/**
 * @file project_config.h
 * @brief Board-specific configuration and global instances (NUCLEO-F446RE).
 *
 * Declares peripheral configuration objects (cfg) and runtime handles (h)
 * used by the board/application layers.
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "drivers/gpio.h"
#include "drivers/exti.h"
#include "drivers/adc.h"
#include "drivers/usart2.h"
#include "drivers/pwm_tim1.h"

#define SYSCLK_HZ		16000000u
#define APB1_CLK_HZ		16000000U				// Adjust based on clock configuration
#define BAUDRATE		115200U

extern const gpio_pin_cfg_t LED_OUTPUT;			// User LED (PA5)
extern const gpio_pin_cfg_t PUSH_BUTTON;		// User Push-button (PC13)
extern const exti_cfg_t USER_BUTTON_EXTI;
extern const adc_cfg_t ADC1_IN0_CFG;			// Analog input (PA0)
extern adc_handle_t ADC1_IN0_H;
extern volatile bool user_button_on;			// Flag for user-button
extern const usart2_cfg_t USART2_CFG;			// USART2 config for PC logging (ST-LINK VCP)
extern usart2_handle_t USART2_H;
extern const pwm_tim1_cfg_t PWM_CFG;			// TIM1 3-channel PWM configuration
extern pwm_tim1_handle_t PWM_H;

#endif /* DRIVERS_PROJECT_CONFIG_H */
