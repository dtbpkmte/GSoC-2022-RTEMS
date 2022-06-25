/**
 * @file
 *
 * @ingroup rtems_gpio
 *
 * @brief RTEMS GPIO API implementation.
 */

/*
 *  Copyright (c) 2022 Duc Doan <dtbpkmte at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/gpio2.h>

void rtems_gpio_register(
    rtems_gpio_ctrl_t *base, 
    const rtems_gpio_handlers_t *handlers
) 
{
    base->handlers = handlers;
}

rtems_status_code rtems_gpio_initialize(
    rtems_gpio_ctrl_t *base
)
{
    return base->handlers->initialize(base);
}

rtems_status_code rtems_gpio_deinitialize(
    rtems_gpio_ctrl_t *base
)
{
    return base->handlers->deinitialize(base);
}

rtems_status_code rtems_gpio_configure(
    rtems_gpio_ctrl_t *base, 
    void *pin,
    rtems_gpio_config_t *config
) 
{
    return base->handlers->configure(base, pin, config);
}

rtems_status_code rtems_gpio_set_pin_mode(
    rtems_gpio_ctrl_t *base, 
    void *pin,
    rtems_gpio_pin_mode mode
) 
{
    return base->handlers->set_pin_mode(base, pin, mode);
}

rtems_status_code rtems_gpio_set_pull(
    rtems_gpio_ctrl_t *base, 
    void *pin,
    rtems_gpio_pull pull
) 
{
    return base->handlers->set_pull(base, pin, pull);
}

rtems_status_code rtems_gpio_configure_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin,
    rtems_gpio_interrupt_config_t *int_conf
) 
{
    return base->handlers->configure_interrupt(base, pin, int_conf);
}

rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
)
{
    return base->handlers->enable_interrupt(base, pin, int_conf);
}

rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
)
{
    return base->handlers->disable_interrupt(base, pin, int_conf);
}

rtems_status_code rtems_gpio_write(
    rtems_gpio_ctrl_t *base, 
    void *pin,
    rtems_gpio_pin_state value
) 
{
    return base->handlers->write(base, pin, value);
}

rtems_status_code rtems_gpio_read(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_state *value
) 
{
    return base->handlers->read(base, pin, value);
}

rtems_status_code rtems_gpio_ctrl_toggle(
    rtems_gpio_ctrl_t *base,
    void *pin
) 
{
    return base->handlers->toggle(base, pin);
}

