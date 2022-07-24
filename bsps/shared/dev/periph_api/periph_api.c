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

#include <bsp/periph_api.h>

/**
  * @brief An array to store handlers to get peripheral 
  *        API of each registered GPIO controller.
  */
static rtems_periph_api *(*get_api_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(rtems_gpio *, rtems_periph_api_type);

/**
  * @brief An array to store handlers to remove peripheral 
  *        API of each registered GPIO controller.
  */
static rtems_status_code (*remove_api_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(rtems_gpio *);

/**
  * Mirrors to GPIO private variables/
  */
static uint32_t (*get_ctrl_index_ptr)(uint32_t);
static uint32_t *num_ctrl_ptr;


void rtems_periph_api_start(
    uint32_t (*get_ctrl_index)(uint32_t),
    uint32_t *num_ctrl
)
{
    get_ctrl_index_ptr = get_ctrl_index;
    num_ctrl_ptr = num_ctrl;
}

void rtems_periph_api_register_api(
    rtems_periph_api *(*get_api)(rtems_gpio *, rtems_periph_api_type),
    rtems_status_code (*remove_api)(rtems_gpio *),
    uint32_t index
)
{
    get_api_table[index] = get_api;
    remove_api_table[index] = remove_api;
}

rtems_status_code rtems_periph_api_set_api(
    rtems_gpio *pin,
    rtems_periph_api_type type
)
{
    // Prevent memory-leak by removing the old API
    // first.
    if (pin->api != NULL)
        rtems_periph_api_remove_api(pin);

    uint32_t i = (*get_ctrl_index_ptr)(pin->virtual_pin);
    if (i >= *num_ctrl_ptr)
        return RTEMS_UNSATISFIED;

    pin->api = (*get_api_table[i])(pin, type);
    if (pin->api == NULL)
        return RTEMS_UNSATISFIED;

    // Initialize the API object
    pin->api->init(pin);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_periph_api_remove_api(
    rtems_gpio *pin
)
{
    uint32_t i = (*get_ctrl_index_ptr)(pin->virtual_pin);
    if (i >= *num_ctrl_ptr)
        return RTEMS_UNSATISFIED;

    return (*remove_api_table[i])(pin);
}
