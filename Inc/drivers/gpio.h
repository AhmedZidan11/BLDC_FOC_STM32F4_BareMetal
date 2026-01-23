
#ifndef GPIO_H
#define GPIO_H

/**
 * @file gpio.h
 * @brief Minimal GPIO module for STM32F4 (CMSIS only).
 *
 * Provides registers configuration and simple I/O read/write tasks
 *
 * Responsibilities:
 * - Configure GPIO registers
 * - Read digital input
 * - Set/reset digital output
 * - toggle digital output
 *
 * @note For ADC and AF use this module to configure GPIO pins
 */

#include <stdint.h>
#include <stdbool.h>
#include<stddef.h>
#include "stm32f4xx.h"

/**
 * @brief Port identifier to simplify implementation of gpio_pin_t
 *
 * Notes: only 8 Ports (A to H) are considered in this driver
 *
 */
typedef enum {
    GPIO_PORTA  = 0,   /**< GPIO_PORTA */
	GPIO_PORTB  = 1,   /**< GPIO_PORTB */
	GPIO_PORTC  = 2,   /**< GPIO_PORTC */
	GPIO_PORTD  = 3,   /**< GPIO_PORTD */
	GPIO_PORTE  = 4,   /**< GPIO_PORTE */
	GPIO_PORTF  = 5,   /**< GPIO_PORTF */
	GPIO_PORTG  = 6,   /**< GPIO_PORTG */
	GPIO_PORTH  = 7,   /**< GPIO_PORTH */
} gpio_port_name_t;

/**
 * @brief Generic pin identifier
 *
 */
typedef struct {
    GPIO_TypeDef *port;   /**< GPIOA, GPIOB, ... */
    uint8_t       pin;    /**< 0..15 */
    gpio_port_name_t port_name; /**< extra port identifier */
} gpio_pin_t;

/**
 * @brief identifiers for GPIO modes in MODER
 *
 */
typedef enum {
    GPIO_MODE_INPUT  = 0, /**< MODER = 00 *//**< GPIO_MODE_INPUT */
    GPIO_MODE_OUTPUT = 1, /**< MODER = 01 *//**< GPIO_MODE_OUTPUT */
    GPIO_MODE_AF     = 2, /**< MODER = 10 *//**< GPIO_MODE_AF */
    GPIO_MODE_ANALOG = 3  /**< MODER = 11 *//**< GPIO_MODE_ANALOG */
} gpio_mode_t;

/**
 * @brief identifiers for output types in OTYPER
 *
 */
typedef enum {
    GPIO_OTYPE_PUSHPULL  = 0, /**< GPIO_OTYPE_PUSHPULL */
    GPIO_OTYPE_OPENDRAIN = 1  /**< GPIO_OTYPE_OPENDRAIN */
} gpio_otype_t;

/**
 * @brief identifiers for I/O pull-up/pull-down in PUPDR
 *
 */
typedef enum {
    GPIO_PULL_NONE = 0,/**< GPIO_PULL_NONE */
    GPIO_PULL_UP   = 1,/**< GPIO_PULL_UP */
    GPIO_PULL_DOWN = 2 /**< GPIO_PULL_DOWN */
} gpio_pull_t;

/**
 * @brief identifiers for output speed types in OSPEEDR
 *
 */
typedef enum {
    GPIO_SPEED_LOW  = 0,/**< GPIO_SPEED_LOW */
    GPIO_SPEED_MED  = 1,/**< GPIO_SPEED_MED */
    GPIO_SPEED_FAST = 2,/**< GPIO_SPEED_FAST */
    GPIO_SPEED_HIGH = 3 /**< GPIO_SPEED_HIGH */
} gpio_speed_t;

/**
 * @brief Generic pin configuration.
 *
 * Notes:
 * - For INPUT/OUTPUT/ANALOG: af is ignored.
 * - For AF mode: af must be 0..15.
 */
typedef struct {
    gpio_pin_t    pin;
    gpio_mode_t   mode;
    gpio_otype_t  otype;  /**< output/AF only */
    gpio_pull_t   pull;
    gpio_speed_t  speed;  /**< output/AF only */
    uint8_t       af;     /**< 0..15 for AF mode */
} gpio_pin_cfg_t;

/**
 * @brief Configure a GPIO pin according to the given instance.
 *
 * @param cfg Pointer to the pin configuration instance.
 * @return true if applied, false if parameters invalid.
 */
bool gpio_init_pin(const gpio_pin_cfg_t *cfg);

/**
 * @brief Set/Reset a defined output GPIO pin via BSRR
 *
 * @param pin contain port and pin information
 * @param level true for High, false for Low
 */
void gpio_write(gpio_pin_t pin, bool level);

/**
 * @brief toggle a defined output GPIO pin
 *
 * @param pin contain port and pin information
 */
void gpio_toggle(gpio_pin_t pin);

/**
 * @brief read the level of a defined input GPIO pin
 *
 * @param pin contain port and pin information
 * @return true for High, false for Low or invalid pin definition
 */
bool gpio_read(gpio_pin_t pin);


#endif /* GPIO_H */
