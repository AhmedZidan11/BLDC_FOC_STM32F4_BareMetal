#ifndef BOARD_BOARD_H
#define BOARD_BOARD_H

/**
 * @file board.h
 * @brief Board-level API for NUCLEO-F446RE.
 *
 * Responsibilities:
 * - Initialize board drivers (GPIO/EXTI/ADC).
 * - Provide simple board actions (LED toggle).
 *
 */

#include "config/project_config.h"

/**
 * @brief Toggle the user LED.
 */
void toggle_led();

/**
 * @brief Initialize board peripherals (GPIO, EXTI, ADC, USART2, TIM1 PWM).
 */
void board_init();

/**
 * @brief Set PWM duty using permyriad units (0..10000 <=> 0.00% to 100.00%).
 *
 * Converts permyriad to timer ticks and updates TIM1_CCRx.
 *
 * @param pwm_h Pointer to PWM handle.
 * @param ch PWM channel number (1..3).
 * @param duty_permyriad Duty in permyriad (0..10000).
 */
void pwm_set_duty_permyriad(pwm_tim1_handle_t* pwm_h, uint8_t ch, uint16_t duty_permyriad);


#endif /* BOARD_BOARD_H_ */
