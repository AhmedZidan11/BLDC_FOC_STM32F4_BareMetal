/**
 * @file project_config.c
 * @brief Definitions for global instances and variables.
 */

#include "config/project_config.h"

volatile bool user_button_on = false;  // Set by EXTI callback when the user button is pressed

/* GPIO configuration for user LED */
const gpio_pin_cfg_t LED_OUTPUT = {
		.pin = {GPIOA, 5, GPIO_PORTA},
		.mode = GPIO_MODE_OUTPUT,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_LOW,
		.af = 0
};

/* GPIO configuration for user push-button */
const gpio_pin_cfg_t PUSH_BUTTON = {
		.pin = {GPIOC, 13, GPIO_PORTC},
		.mode = GPIO_MODE_INPUT,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_UP,
		.speed = GPIO_SPEED_LOW,
		.af = 0
};

/* GPIO configuration for ADC input (PA0 / ADC1_IN0) */
const gpio_pin_cfg_t ADC_IN0 = {
		.pin = {GPIOA, 0, GPIO_PORTA},
		.mode = GPIO_MODE_ANALOG,
		.otype = GPIO_OTYPE_OPENDRAIN,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_LOW,
		.af = 0
};

/* EXTI configuration for user button (PC13 -> EXTI13) */
const exti_cfg_t USER_BUTTON_EXTI = {
		.gpio_cfg = &PUSH_BUTTON,
		.edge = FALLING_EDGE,
		.priority = 6,
};

/* ADC configuration for PA0 (channel 0) */
const adc_cfg_t ADC1_IN0_CFG = {
		.adc_channel = 0,
		.inst = ADC1,
		.mode = ADC_MODE_SINGLE,
		.pin_cfg = &ADC_IN0,
		.resolution = ADC_12_BIT,
		.sample_time = CYCLES_84,
		.irqn = ADC_IRQn,
		.irq_priority = 5,
};

/* ADC handle for PA0 (channel 0) */
adc_handle_t ADC1_IN0_H = {
		.cfg = & ADC1_IN0_CFG,
		.inst = ADC1,
		.adc_data_ready = false,
		.last_reading = 0,
};

/* GPIO configuration for USART2 (PA2 as Tx) */
const gpio_pin_cfg_t PIN_TX = {
		.pin = {GPIOA, 2, GPIO_PORTA},
		.mode = GPIO_MODE_AF,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_HIGH,
		.af = 7
};

/* GPIO configuration for USART2 (PA3 as Rx) */
const gpio_pin_cfg_t PIN_RX = {
		.pin = {GPIOA, 3, GPIO_PORTA},
		.mode = GPIO_MODE_AF,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_HIGH,
		.af = 7
};

/* USART2 configuration for PC connection (ST-LINK VCP) */
const usart2_cfg_t USART2_CFG = {
		.irq_priority = 6,
		.irqn = USART2_IRQn,
		.pin_cfg_rx = &PIN_RX,
		.pin_cfg_tx = &PIN_TX,
		.usart_baud = BAUDRATE,
		.usart_pclk_hz = APB1_CLK_HZ
};

/* USART2 RX/TX ring buffers (storage owned by this module and referenced by USART2_H) */
static ring_buffer_t usart2_rx_rb = {0};
static ring_buffer_t usart2_tx_rb = {0};

/* USART2 Handle */
usart2_handle_t USART2_H = {
		.err_fe_cnt = 0,
		.err_ne_cnt = 0,
		.err_ore_cnt = 0,
		.err_pe_cnt = 0,
		.rx_buffer = &usart2_rx_rb,
		.tx_buffer = &usart2_tx_rb,
};

/* GPIO configuration for PWM via TIM1 (PA8 as Ch1) */
const gpio_pin_cfg_t PWM_CH1 = {
		.pin = {GPIOA, 8, GPIO_PORTA},
		.mode = GPIO_MODE_AF,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_HIGH,
		.af = 1
};

/* GPIO configuration for PWM via TIM1 (PA9 as Ch2) */
const gpio_pin_cfg_t PWM_CH2 = {
		.pin = {GPIOA, 9, GPIO_PORTA},
		.mode = GPIO_MODE_AF,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_HIGH,
		.af = 1
};

/* GPIO configuration for PWM via TIM1 (PA10 as Ch3) */
const gpio_pin_cfg_t PWM_CH3 = {
		.pin = {GPIOA, 10, GPIO_PORTA},
		.mode = GPIO_MODE_AF,
		.otype = GPIO_OTYPE_PUSHPULL,
		.pull = GPIO_PULL_NONE,
		.speed = GPIO_SPEED_HIGH,
		.af = 1
};

/* TIM1 PWM configuration (frequency in Hz) */
const pwm_tim1_cfg_t PWM_CFG = {
  .tim_clk_hz = SYSCLK_HZ,
  .pwm_hz     = 1000,
  .align      = PWM_ALIGN_CENTER_1,
  .pin_ch1    = &PWM_CH1,
  .pin_ch2    = &PWM_CH2,
  .pin_ch3    = &PWM_CH3,
};

/* PWM Handle via TIM1 */
pwm_tim1_handle_t PWM_H = {
		.arr = 0,
};
