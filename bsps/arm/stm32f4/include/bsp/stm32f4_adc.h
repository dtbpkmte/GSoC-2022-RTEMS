/* SPDX-License-Identifier: BSD-2-Clause */

/**
  * @file
  *
  * @ingroup stm32f4_adc
  *
  * STM32F4 BSP ADC API implementation.
  */

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

#ifndef LIBBSP_ARM_STM32F4_BSP_ADC
#define LIBBSP_ARM_STM32F4_BSP_ADC

#include <rtems/sysinit.h>
#include <bsp/adc.h>

#ifdef __ENABLE_ADC_API

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_adc.h>
#include <bsp/stm32f4_gpio.h>

#define STM32F4_ADC_DEFAULT_RESOLUTION          LL_ADC_RESOLUTION_10B
#define STM32F4_ADC_DEFAULT_ALIGNMENT           LL_ADC_DATA_ALIGN_RIGHT
#define STM32F4_ADC_DEFAULT_SAMPLINGTIME        LL_ADC_SAMPLINGTIME_3CYCLES

/**
  * Macros for simple locking of shared objects.
  * Structs must have a bool member named "locked"
  */
#define STM32F4_LOCK(obj)               \
    do {                                \
        ( obj )->locked = true;         \
    } while (0)

#define STM32F4_UNLOCK(obj)             \
    do {                                \
        ( obj )->locked = false;        \
    } while (0)

#define STM32F4_IS_LOCKED(obj)          \
    (( obj )->locked)

/**
  * @brief Wrapper of ADC_TypeDef with a simple lock.
  */
typedef struct {
    ADC_TypeDef *ADCx;
    bool locked;
} ADC_TypeDef_Protected;

/**
  * @brief Structure containing ADC configuration for an
  *        ADC pin.
  */
typedef struct {
    /**
      * @brief Locks ADC configuration change on this pin
      *        if set to true.
      */
    bool locked;
    /**
      * @brief STM32F4-defined ADCx.
      */
    ADC_TypeDef_Protected *ADCx;
    /**
      * @brief ADC channel of the pin.
      * This can be LL_ADC_CHANNEL_n defined by STM32F4 LL
      * driver, where 0 <= n <= 18. 
      */
    uint32_t channel;
    /**
      * @brief Resolution of the ADC pin.
      * This can be one of the following values:
      *     @ref LL_ADC_RESOLUTION_12B
      *     @ref LL_ADC_RESOLUTION_10B
      *     @ref LL_ADC_RESOLUTION_8B
      *     @ref LL_ADC_RESOLUTION_6B
      */
    uint32_t resolution;
    /**
      * @brief Data alignment of the ADC pin.
      * This can be one of the following values:
      *     @ref LL_ADC_DATA_ALIGN_RIGHT
      *     @ref LL_ADC_DATA_ALIGN_LEFT
      */
    uint32_t alignment;
} stm32f4_adc_config;

/**
  * @brief STM32F4 wrapper of peripherals API.
  */
typedef struct {
    rtems_adc_api base_api;
    stm32f4_adc_config *adc_config;
} stm32f4_adc;

const rtems_adc_handlers *stm32f4_get_adc_handlers(
    void
);

/**
  * @brief Get the HAL-defined ADCx pointer.
  * @note This function should not be called. Use 
  *       @see stm32f4_get_ADCx_protected() instead.
  */
ADC_TypeDef *stm32f4_get_ADCx(
    GPIO_TypeDef *gpiox
);

ADC_TypeDef_Protected *stm32f4_get_ADCx_protected(
    GPIO_TypeDef *gpiox
);

rtems_status_code stm32f4_get_LL_ADC_CHANNEL(
    stm32f4_gpio *gpio,
    uint32_t *channel
);

bool stm32f4_is_adc_pin(
    stm32f4_gpio *gpio
);

rtems_periph_api *stm32f4_adc_get(
    rtems_gpio *gpio
);

rtems_status_code stm32f4_adc_destroy(
    rtems_gpio *gpio
);

/***/
/**
  * @brief Perform initialization for STM32F4 ADC manager
  *
  * This function should be called in bspstarthook under
  * if ADC is enabled.
  */
extern void stm32f4_adc_start(
    void
);

/**
  * @brief Performs initialization for an ADC pin
  * @note This function needs to be registered with the
  *       peripherals API.
  */
void stm32f4_adc_init(
    rtems_gpio *base
);

rtems_status_code stm32f4_adc_read_raw(
    rtems_gpio *base,
    uint32_t *result,
    uint32_t timeout
);

rtems_status_code stm32f4_adc_start_read_raw_nb(
    rtems_gpio *base
);

rtems_adc_status stm32f4_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
);

rtems_status_code stm32f4_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
);

rtems_status_code stm32f4_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
);

rtems_status_code stm32f4_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
);

rtems_status_code stm32f4_adc_remove_interrupt(
    rtems_gpio *base
);

rtems_status_code stm32f4_adc_enable_interrupt(
    rtems_gpio *base
);

rtems_status_code stm32f4_adc_disable_interrupt(
    rtems_gpio *base
);

#endif /* __ENABLE_ADC_API */

#endif /* LIBBSP_ARM_STM32F4_BSP_ADC */
