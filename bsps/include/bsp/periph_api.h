/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup periph_api
 *
 * RTEMS Peripheral API
 *
 * This API acts as a base system for all peripheral APIs that operate on
 * a single GPIO pin. The main intention is to simplify user application
 * when dealing with additional functionality of GPIO pins.
 *
 * New APIs can be created based on this core API. In OOP language, they
 * just have to inherit the base class rtems_periph_api. In other words,
 * new APIs need to create a new structure that has a @ref rtems_periph_api
 * as the first member and implement some required functions.
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

#ifndef LIBBSP_BSP_PERIPH_API_H
#define LIBBSP_BSP_PERIPH_API_H

#include <bsp.h>
#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
  * @name Peripheral API data structures.
  *
  * @{
  */

/**
  * @brief Enumeration of supported peripheral APIs.
  *
  * New APIs should be added to this enum.
  */
typedef enum {
    RTEMS_PERIPH_API_TYPE_ADC,
    RTEMS_PERIPH_API_TYPE_BSP_SPECIFIC = 100,
    RTEMS_PERIPH_API_TYPE_APP_SPECIFIC = 200,
} rtems_periph_api_type;

/**
  * @brief The base structure of peripheral APIs.
  * All peripheral APIs must have this struct as their first member.
  *
  * @see rtems_adc_api for example implementation.
  */
typedef struct rtems_periph_api rtems_periph_api;

#include <bsp/gpio2.h>

/**
  * rtems_periph_api structure definition.
  */
struct rtems_periph_api {
    /**
      * @brief The type of this API.
      */
    rtems_periph_api_type api_type;
    /**
      * @brief A driver-specific function that performs initialization 
      *        for the API.
      */
    void (*init)(rtems_gpio *pin);
};

/** @} */

/**
  * @name Peripheral API operations.
  *
  * @{
  */

/**
  * @brief Register a handler to get API pointer.
  *
  * @param[in] get_api pointer to a handler to get an API with 
  *            specified type.
  * @param index the index of the GPIO controller.
  */
void rtems_periph_api_register_api(
    rtems_periph_api *(*get_api)(rtems_gpio *, rtems_periph_api_type),
    rtems_status_code (*remove_api)(rtems_gpio *),
    uint32_t index
);

/**
  * @brief Performs initialization for peripheral
  *        API system.
  * This function registers some variables that 
  * the peripheral API utilize from GPIO API.
  * 
  * @param get_ctrl_index Pointer to a helper
  *        that returns a GPIO controller index
  *        of a pin.
  * @param num_ctrl Pointer to a variable that
  *        tells the number of registered GPIO
  *        controllers.
  */
void rtems_periph_api_start(
    uint32_t (*get_ctrl_index)(uint32_t),
    uint32_t *num_ctrl
);

/**
  * @brief Assign an API to a GPIO object and 
  *        initialize it.
  *
  * @note This function may use malloc().
  * @note This function calls the handler init().
  *
  * @param pin The rtems_gpio object representing
  *            a pin.
  * @param type The peripheral API type.
  *
  * @retval RTEMS_SUCCESSFUL if an API is set
  *         correctly.
  * @retval RTEMS_UNSATISFIED if the API type is
  *         invalid for this pin.
  * @retval RTEMS_NO_MEMORY if memory cannot be
  *         allocated for API object.
  */
rtems_status_code rtems_periph_api_set_api(
    rtems_gpio *pin,
    rtems_periph_api_type type
);

/**
  * @brief Remove the API assigned to a pin by
  *        setting the pointer to NULL.
  *
  * @retval RTEMS_SUCCESSFUL
  */
rtems_status_code rtems_periph_api_remove_api(
    rtems_gpio *pin
);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_PERIPH_API_H */
