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

/**
  * An array to store all registered GPIO controllers.
  */
static rtems_status_code (*get_gpio_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(uint32_t, rtems_gpio_t **);
static uint32_t pin_map[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS+1] = {0};
static uint32_t num_ctrl = 0;

void rtems_gpio_begin(
    void
)
{
    bsp_gpio_register_controllers();   
}

RTEMS_SYSINIT_ITEM(
    rtems_gpio_begin,
    RTEMS_SYSINIT_BSP_START,
    RTEMS_SYSINIT_ORDER_LAST
);


/**
  * @brief 
  *
  * @retval RTEMS_TOO_MANY if the maximum number of controllers are
  *         already registered
  */
rtems_status_code rtems_gpio_register(
    rtems_status_code (*get_gpio)(uint32_t, rtems_gpio_t **),
    uint32_t max_pin
) 
{
    rtems_interrupt_level level;

    if (num_ctrl == CONFIGURE_GPIO_MAXIMUM_CONTROLLERS)
        return RTEMS_TOO_MANY;

    rtems_interrupt_disable(level);
    get_gpio_table[num_ctrl] = get_gpio;
    pin_map[num_ctrl+1] = pin_map[num_ctrl] + max_pin;
    ++num_ctrl;
    rtems_interrupt_enable(level);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_get(
    uint32_t virt_pin,
    rtems_gpio_t **out
) 
{
    uint32_t i, pin;
    // TODO: binary search
    for (i = 1; i <= num_ctrl; ++i) {
        if (virt_pin < pin_map[i]) {
            pin = virt_pin - pin_map[i-1];
            break;
        }
    }
    if (i > num_ctrl)
        return RTEMS_UNSATISFIED;

    return (*get_gpio_table[i-1])(pin, out);
}

rtems_status_code rtems_gpio_begin(
    void
)
{
    return bsp_gpio_register_controllers();   
}

/**
  * Default implementation for bsp_gpio_register_controllers.
  */
__attribute__((weak)) rtems_status_code bsp_gpio_register_controllers(
    void
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code rtems_gpio_init(
    rtems_gpio_t *base
)
{
    return base->handlers->init(base);
}

rtems_status_code rtems_gpio_deinit(
    rtems_gpio_t *base
)
{
    return base->handlers->deinit(base);
}

rtems_status_code rtems_gpio_set_pin_mode(
    rtems_gpio_t *base, 
    rtems_gpio_pin_mode_t mode
) 
{
    return base->handlers->set_pin_mode(base, mode);
}

rtems_status_code rtems_gpio_set_pull(
    rtems_gpio_t *base, 
    rtems_gpio_pull_t pull
) 
{
    return base->handlers->set_pull(base, pull);
}

rtems_status_code rtems_gpio_configure_interrupt(
    rtems_gpio_t *base, 
    rtems_gpio_isr_t isr,
    void *arg,
    rtems_gpio_interrupt_trig_t trig,
    rtems_gpio_pull_t pull
) 
{
    return base->handlers->configure_interrupt(base, isr, arg, trig, pull);
}

rtems_status_code rtems_gpio_remove_interrupt(
    rtems_gpio_t *base, 
)
{
    return base->handlers->remove_interrupt(base);
}

rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio_t *base, 
)
{
    return base->handlers->enable_interrupt(base);
}

rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio_t *base, 
)
{
    return base->handlers->disable_interrupt(base);
}

rtems_status_code rtems_gpio_write(
    rtems_gpio_t *base, 
    rtems_gpio_pin_state_t value
) 
{
    return base->handlers->write(base, value);
}

rtems_status_code rtems_gpio_read(
    rtems_gpio_t *base, 
    rtems_gpio_pin_state *value
) 
{
    return base->handlers->read(base, value);
}

rtems_status_code rtems_gpio_toggle(
    rtems_gpio_t *base,
) 
{
    return base->handlers->toggle(base);
}

