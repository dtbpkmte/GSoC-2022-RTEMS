/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2022 Duc Doan (dtbpkmte at gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/**
  * @brief Lock configuration of a pin.
  *
  * @param[in] base The pointer to the GPIO object.
  */
extern void stm32f4_gpio_lock_pin(
    rtems_gpio *base
);

/**
  * @brief Sets the alternate function for a pin.
  *
  * @param[in] base The pointer to the GPIO object.
  * @param alternate Alternate function, from 0-15.
  */
extern void stm32f4_gpio_set_af(
    rtems_gpio *base,
    uint32_t alternate
);

#endif /* LIBBSP_ARM_STM32F4_BSP_GPIO */
