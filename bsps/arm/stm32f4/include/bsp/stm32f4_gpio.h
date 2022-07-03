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
    /**
      * @brief This member is a rtems_gpio object.
      */
    rtems_gpio base;
    /**
      *@brief This member is the pin number from 0 to 15.
      */
    uint32_t pin;
    /**
      * @brief This member is HAL GPIOx pointer.
      */
    GPIO_TypeDef *port;
} stm32f4_gpio;

#endif /* LIBBSP_ARM_STM32F4_BSP_GPIO */
