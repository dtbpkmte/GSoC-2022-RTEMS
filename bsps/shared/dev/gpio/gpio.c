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


__attribute__((weak)) rtems_status_code rtems_gpio_initialize(void) {
    return RTEMS_SUCCESSFUL;
}

RTEMS_SYSINIT_ITEM(
    rtems_gpio_initialize,
    RTEMS_SYSINIT_BSP_START,
    RTEMS_SYSINIT_ORDER_LAST
);

__attribute__((weak)) rtems_status_code rtems_gpio_write_pin_ex(rtems_gpio_t *gpiox, rtems_gpio_pin_state value) {
    return RTEMS_NOT_IMPLEMENTED;
}

__attribute__((weak)) rtems_status_code rtems_gpio_read_pin_ex(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value) {
    return RTEMS_NOT_IMPLEMENTED;
}

__attribute__((weak)) rtems_status_code rtems_gpio_toggle_pin_ex(rtems_gpio_t *gpiox) {
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state value) {
    if (gpiox->is_expander) {
        return rtems_gpio_write_pin_ex(gpiox, value);
    } else {
        return rtems_gpio_write_pin_default(gpiox, value);
    }
}

rtems_status_code rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value) {
    if (gpiox->is_expander) {
        return rtems_gpio_read_pin_ex(gpiox, value);
    } else {
        return rtems_gpio_read_pin_default(gpiox, value);
    }
}

rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox) {
    if (gpiox->is_expander) {
        return rtems_gpio_toggle_pin_ex(gpiox);
    } else {
        return rtems_gpio_toggle_pin_default(gpiox);
    }
}
