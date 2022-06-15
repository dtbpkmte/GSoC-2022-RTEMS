#ifndef LIBBSP_STM32F4_GPIO
#define LIBBSP_STM32F4_GPIO

#include <stm32f4xx.h>
#include <bsp/gpio2.h>

struct rtems_gpio_t {
    GPIO_TypeDef *port;
};

struct rtems_gpio_pin_t {
    uint16_t pin_mask;
};

struct rtems_gpio_config_t {
    GPIO_InitTypeDef *config;
};

#endif
