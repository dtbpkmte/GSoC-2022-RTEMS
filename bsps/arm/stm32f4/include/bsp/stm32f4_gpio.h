/*
 *  Copyright (c) 2022 Duc Doan <dtbpkmte at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
 
#ifndef LIBBSP_ARM_STM32F4_BSP_GPIO
#define LIBBSP_ARM_STM32F4_BSP_GPIO

#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_exti.h>
#include <bsp/gpio2.h>

/**
  * @brief STM32F4 BSP GPIO structure
  *
  */
typedef struct {
    rtems_gpio_t base;          /** Base rtems_gpio_t obj  **/
    uint32_t pin;               /** Pin index from 0 to 15 **/
    GPIO_TypeDef *port;         /** HAL GPIOx pointer      **/
} stm32f4_gpio_t;

#endif /* LIBBSP_ARM_STM32F4_BSP_GPIO */
