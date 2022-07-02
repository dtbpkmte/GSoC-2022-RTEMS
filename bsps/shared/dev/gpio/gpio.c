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
#include <rtems/sysinit.h>

/**
  * An array to store all registered GPIO controllers.
  */
static rtems_status_code (*get_gpio_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(uint32_t, rtems_gpio **);

/**
  * An array to store all registered GPIO controllers.
  */
static rtems_status_code (*destroy_gpio_table[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS])(rtems_gpio *);

/**
  * An array to store the boundaries of pin index of controllers.
  * Example with 2 16-pin controllers and 1 32-pin controller,
  * the pin_map will be:
  * { 0, 16, 32, 64}
  * Value 0 is always at index 0 for convenience of calculation.
  * The length of this array is always 1+(number of controllers).
  */
static uint32_t pin_map[CONFIGURE_GPIO_MAXIMUM_CONTROLLERS+1] = {0};

/**
  * The number of controllers registered.
  */
static uint32_t num_ctrl = 0;

void rtems_gpio_begin(
    void
)
{
    bsp_gpio_register_controllers();   
}

/**
  * @brief 
  *
  * @retval RTEMS_TOO_MANY if the maximum number of controllers are
  *         already registered
  */
rtems_status_code rtems_gpio_register(
    rtems_status_code (*get_gpio)(uint32_t, rtems_gpio **),
    rtems_status_code (*destroy_gpio)(rtems_gpio *),
    uint32_t pin_count
) 
{
//    rtems_interrupt_level level;

    if (num_ctrl == CONFIGURE_GPIO_MAXIMUM_CONTROLLERS)
        return RTEMS_TOO_MANY;

//    rtems_interrupt_disable(level);
    get_gpio_table[num_ctrl] = get_gpio;
    destroy_gpio_table[num_ctrl] = destroy_gpio;
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

    rtems_status_code sc = (*get_gpio_table[i-1])(pin, out);
    if (sc == RTEMS_SUCCESSFUL) {
        (*out)->virtual_pin = virt_pin;
    }
    return sc;
}

rtems_status_code rtems_gpio_destroy(
    rtems_gpio *base
)
{
    uint32_t i;
    // TODO: binary search
    for (i = 1; i <= num_ctrl; ++i) {
        if (base->virtual_pin < pin_map[i]) {
            return (*destroy_gpio_table[i-1])(base);
        }
    }
    return RTEMS_UNSATISFIED;
}

rtems_status_code rtems_gpio_init(
    rtems_gpio *base
)
{
    return base->handlers->init(base);
}

rtems_status_code rtems_gpio_deinit(
    rtems_gpio *base
)
{
    return base->handlers->deinit(base);
}

rtems_status_code rtems_gpio_set_pin_mode(
    rtems_gpio *base, 
    rtems_gpio_pin_mode mode
) 
{
    return base->handlers->set_pin_mode(base, mode);
}

rtems_status_code rtems_gpio_set_pull(
    rtems_gpio *base, 
    rtems_gpio_pull pull
) 
{
    return base->handlers->set_pull(base, pull);
}

rtems_status_code rtems_gpio_configure_interrupt(
    rtems_gpio *base, 
    rtems_gpio_isr isr,
    void *arg,
    rtems_gpio_interrupt_trig trig,
    rtems_gpio_pull pull
) 
{
    return base->handlers->configure_interrupt(base, isr, arg, trig, pull);
}

rtems_status_code rtems_gpio_remove_interrupt(
    rtems_gpio *base
)
{
    return base->handlers->remove_interrupt(base);
}

rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio *base
)
{
    return base->handlers->enable_interrupt(base);
}

rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio *base
)
{
    return base->handlers->disable_interrupt(base);
}

rtems_status_code rtems_gpio_write(
    rtems_gpio *base, 
    rtems_gpio_pin_state value
) 
{
    return base->handlers->write(base, value);
}

rtems_status_code rtems_gpio_read(
    rtems_gpio *base, 
    rtems_gpio_pin_state *value
) 
{
    return base->handlers->read(base, value);
}

rtems_status_code rtems_gpio_toggle(
    rtems_gpio *base
) 
{
    return base->handlers->toggle(base);
}

