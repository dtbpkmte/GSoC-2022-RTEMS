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

#include <bsp/gpio2.h>
#include <rtems/sysinit.h>

/**
  * @brief An array to store all registered GPIO controllers.
  */
static rtems_status_code (*get_gpio_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(uint32_t, rtems_gpio **);

/**
  * @brief An array to store all registered GPIO controllers.
  */
static rtems_status_code (*destroy_gpio_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(rtems_gpio *);

/**
  * @brief An array to store the boundaries of pin index of 
  *        GPIO controllers.
  *
  * Example with 2 16-pin controllers and 1 32-pin controller,
  * the pin_map will be:
  * { 0, 16, 32, 64}
  * Value 0 is always at index 0 for convenience of calculation.
  * The length of this array is always 1+(number of controllers).
  */
static uint32_t pin_map[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS+1] = {0};

/**
  * @brief The number of controllers registered.
  */
static uint32_t num_ctrl = 0;

static uint32_t get_ctrl_index(
    uint32_t virtual_pin
);

static uint32_t get_ctrl_index(
    uint32_t virtual_pin
)
{
    uint32_t i;
    // TODO: binary search
    for (i = 1; i <= num_ctrl; ++i) {
        if (virtual_pin < pin_map[i]) {
            break;
        }
    }
    return i-1;
}

rtems_status_code rtems_gpio_register(
    rtems_status_code (*get_gpio)(uint32_t, rtems_gpio **),
    rtems_status_code (*destroy_gpio)(rtems_gpio *),
    rtems_periph_api *(*get_api)(rtems_gpio *, rtems_periph_api_type),
    rtems_status_code (*remove_api)(rtems_gpio *),
    uint32_t pin_count
) 
{
//    rtems_interrupt_level level;

    if (num_ctrl == CONFIGURE_GPIO_MAXIMUM_CONTROLLERS)
        return RTEMS_TOO_MANY;

//    rtems_interrupt_disable(level);
    get_gpio_table[num_ctrl] = get_gpio;
    destroy_gpio_table[num_ctrl] = destroy_gpio;

    rtems_periph_api_register_api(get_api, remove_api, num_ctrl);

    pin_map[num_ctrl+1] = pin_map[num_ctrl] + pin_count;
    ++num_ctrl;
//    rtems_interrupt_enable(level);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_get(
    uint32_t virt_pin,
    rtems_gpio **out
) 
{
    uint32_t i = get_ctrl_index(virt_pin);
    if (i >= num_ctrl)
        return RTEMS_UNSATISFIED;

    uint32_t pin = virt_pin - pin_map[i];
    rtems_status_code sc = (*get_gpio_table[i])(pin, out);
    if (sc == RTEMS_SUCCESSFUL) {
        (*out)->virtual_pin = virt_pin;
    }
    return sc;
}

rtems_status_code rtems_gpio_destroy(
    rtems_gpio *base
)
{
    uint32_t i = get_ctrl_index(base->virtual_pin);
    if (i >= num_ctrl)
        return RTEMS_UNSATISFIED;
    return (*destroy_gpio_table[i])(base);
}

void rtems_gpio_start(
    void
)
{
    rtems_periph_api_start(get_ctrl_index, &num_ctrl);
    bsp_gpio_register_controllers();
}
RTEMS_SYSINIT_ITEM(
    rtems_gpio_start,
    RTEMS_SYSINIT_DEVICE_DRIVERS,
    RTEMS_SYSINIT_ORDER_LAST_BUT_1
);

rtems_status_code rtems_gpio_set_pin_mode(
    rtems_gpio *base, 
    rtems_gpio_pin_mode mode
) 
{
    return base->gpio_handlers->set_pin_mode(base, mode);
}

rtems_status_code rtems_gpio_set_pull(
    rtems_gpio *base, 
    rtems_gpio_pull pull
) 
{
    return base->gpio_handlers->set_pull(base, pull);
}

rtems_status_code rtems_gpio_configure_interrupt(
    rtems_gpio *base, 
    rtems_gpio_isr isr,
    void *arg,
    rtems_gpio_interrupt_trig trig,
    rtems_gpio_pull pull
) 
{
    return base->gpio_handlers->configure_interrupt(base, isr, arg, trig, pull);
}

rtems_status_code rtems_gpio_remove_interrupt(
    rtems_gpio *base
)
{
    return base->gpio_handlers->remove_interrupt(base);
}

rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio *base
)
{
    return base->gpio_handlers->enable_interrupt(base);
}

rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio *base
)
{
    return base->gpio_handlers->disable_interrupt(base);
}

rtems_status_code rtems_gpio_write(
    rtems_gpio *base, 
    rtems_gpio_pin_state value
) 
{
    return base->gpio_handlers->write(base, value);
}

rtems_status_code rtems_gpio_read(
    rtems_gpio *base, 
    rtems_gpio_pin_state *value
) 
{
    return base->gpio_handlers->read(base, value);
}

rtems_status_code rtems_gpio_toggle(
    rtems_gpio *base
) 
{
    return base->gpio_handlers->toggle(base);
}

