/**
 * @file board.c
 * @brief Board-level initialization and helpers for NUCLEO-F446RE (CMSIS only).
 *
 * Wires together project_config instances with driver init calls.
 */

#include "board/board.h"

void toggle_led()
{
	gpio_toggle(LED_OUTPUT.pin);
}

/**
 * @brief EXTI callback for the user button (PC13).
 *
 * Toggles the LED and raises a software flag for the application.
 */
static void user_button_callback( )
{
	toggle_led();
	user_button_on = true;
}

void board_init()
{
	/* Configure GPIO pins */
	gpio_init_pin(&LED_OUTPUT);
	gpio_init_pin(&PUSH_BUTTON);
	/* Configure EXTI lines */
	exti_register(13, user_button_callback, NULL);
	exti_init(&USER_BUTTON_EXTI);
	/* Configure ADC channels*/
	adc_init(&ADC1_IN0_H, &ADC1_IN0_CFG);
	usart2_init(&USART2_CFG, &USART2_H);
	pwm_tim1_init(&PWM_CFG, &PWM_H);
}

void pwm_set_duty_permyriad(pwm_tim1_handle_t* pwm_h, uint8_t ch, uint16_t duty_permyriad)
{
	if (pwm_h == NULL) return;
	if ((ch<1)||(ch>3)) return;
	if (duty_permyriad > 10000u) duty_permyriad = 10000u;
	uint16_t arr = pwm_h->arr;
	uint16_t duty = (uint16_t)(((uint32_t)duty_permyriad*((uint32_t)arr)) / 10000u);
	pwm_tim1_set_duty(pwm_h, ch, duty);
}
