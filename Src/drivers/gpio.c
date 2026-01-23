
/**
 * @file gpio.c
 * @brief GPIO module implementation (STM32F4, CMSIS only).
 *
 *  GPIO configuration using direct register access (CMSIS)
 *  Implements: mode, pull-up/down, speed, output type, alternate function
 */

#include "drivers/gpio.h"


/* to check if pin number is valid */
static inline bool pin_ok(uint8_t pin) { return (pin < 16u); }


/* to define number of bit-shifts for MODER*/
static inline uint32_t moder_shift(uint8_t pin) { return ((uint32_t)pin * 2u); } // Bits 2*pin:2*pin+1


 /* to provide index of AFR for a corresponding pin number */
static inline uint32_t afr_index(uint8_t pin)   { return ((uint32_t)pin >> 3); } // pin = 0..7 => AFR[0], pin = 8:15 => AFR[1]


 /* to define number of bit-shifts for AFR */
static inline uint32_t afr_shift(uint8_t pin)   { return (((uint32_t)pin & 7u) * 4u); } // e.g pin = 5 => AFR[0], Bits 20:23


/**
 * @brief Apply a mask with:
 *
 * @param width number of bits
 * @param shift number of shifts
 * @return resulting mask
 */
static inline uint32_t field_mask(uint32_t width, uint32_t shift)
{
    return (((1u << width) - 1u) << shift);
}


/**
 * @brief Enable corresponding clock for the chosen GPIO Port
 *
 * @param port instant of GPIO_TypeDef from stm32f4xx.h
 */
static void gpio_clock_enable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; }
    else if (port == GPIOB) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; }
    else if (port == GPIOC) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; }
    else if (port == GPIOD) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; }
    else if (port == GPIOE) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; }
    else if (port == GPIOF) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN; }
    else if (port == GPIOG) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN; }
    else if (port == GPIOH) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; }
#if defined(GPIOI)
    else if (port == GPIOI) { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN; }
#endif
}


/* Configuration of GPIO registers for a given GPIO pin */
bool gpio_init_pin(const gpio_pin_cfg_t *cfg)
{
    if (cfg == NULL) return false;				// invalid cfg pointer
    if (cfg->pin.port == NULL) return false;	// invalid port pointer
    if (!pin_ok(cfg->pin.pin)) return false;	// invalid pin number

    gpio_clock_enable(cfg->pin.port);

    uint32_t pin = cfg->pin.pin;
    uint32_t sh2 = moder_shift((uint8_t)pin);

    /* MODER */
    cfg->pin.port->MODER &= ~field_mask(2u, sh2);
    cfg->pin.port->MODER |= (uint32_t)cfg->mode << sh2;

    /* PUPDR */
    cfg->pin.port->PUPDR &= ~field_mask(2u, sh2);
    cfg->pin.port->PUPDR |= (uint32_t)cfg->pull << sh2;

    /* OTYPER + OSPEEDR (output/AF only) */
    if (cfg->mode == GPIO_MODE_OUTPUT || cfg->mode == GPIO_MODE_AF)
    {
    	cfg->pin.port->OTYPER &= ~(1u << pin);
    	cfg->pin.port->OTYPER |= (uint32_t)cfg->otype << pin;

    	cfg->pin.port->OSPEEDR &= ~field_mask(2u, sh2);
    	cfg->pin.port->OSPEEDR |= (uint32_t)cfg->speed << sh2;
    }

    /* AFR (for AF mode only) */
    if (cfg->mode == GPIO_MODE_AF)
    {
        if (cfg->af > 15u) return false;

        uint32_t idx = afr_index((uint8_t)pin);
        uint32_t sh4 = afr_shift((uint8_t)pin);

        cfg->pin.port->AFR[idx] &= ~field_mask(4u, sh4);
        cfg->pin.port->AFR[idx] |= (uint32_t)cfg->af << sh4;
    }

    return true;
}


void gpio_write(gpio_pin_t pin, bool level)
{
    if (pin.port == NULL || !pin_ok(pin.pin)) return;

    if (level) {
        pin.port->BSRR = (1u << pin.pin);          /* set */
    }
    else {
        pin.port->BSRR = (1u << (pin.pin + 16u));  /* reset */
    }
}


void gpio_toggle(gpio_pin_t pin)
{
    if (pin.port == NULL || !pin_ok(pin.pin)) return;
    pin.port->ODR ^= (1u << pin.pin);
}


bool gpio_read(gpio_pin_t pin)
{
    if (pin.port == NULL || !pin_ok(pin.pin)) return false;
    return (pin.port->IDR & (1u << pin.pin)) != 0u;
}
