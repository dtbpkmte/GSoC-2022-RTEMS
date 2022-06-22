/**
  * @file
  *
  * @ingroup rtems_bsp/arm/stm32f4
  *
  * @brief RTEMS GPIO new API implementation for STM32F4.
  */

#include <bsp.h>
#include <rtems.h>
#include <bsp/stm32f4_gpio.h>

__attribute__((weak)) rtems_status_code rtems_gpio_initialize(void) {
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_pin_mode mode, rtems_gpio_pull pull, rtems_gpio_config_t *config) {
    GPIO_InitTypeDef init_struct;

    // Pin number
    init_struct.Pin = (uint16_t) (gpiox->pin_mask);
    
    // Pin mode
    switch (mode) {
        case RTEMS_GPIO_PINMODE_OUTPUT_PP:
            init_struct.Mode = GPIO_MODE_OUTPUT_PP;
            break;
        case RTEMS_GPIO_PINMODE_OUTPUT_OD:
            init_struct.Mode = GPIO_MODE_OUTPUT_OD;
            break;
        case RTEMS_GPIO_PINMODE_INPUT:
            init_struct.Mode = GPIO_MODE_INPUT;
            break;
        case RTEMS_GPIO_PINMODE_ANALOG:
            init_struct.Mode = GPIO_MODE_ANALOG;
            break;
        case RTEMS_GPIO_PINMODE_INTERRUPT:
            init_struct.Mode = config->interrupt_mode;
            break;
        case RTEMS_GPIO_PINMODE_BSP_SPECIFIC:
            if (config->is_event_mode == 1) {
                init_struct.Mode = config->event_mode;
            } else if (config->is_alternate_mode == 1) {
                init_struct.Mode = config->alternate_mode;
            } else {
                return RTEMS_UNSATISFIED;
            }
            break;
        default:
            /* illegal argument */
            return RTEMS_UNSATISFIED;
    }

    // Pin pull register
    switch (pull) {
        case RTEMS_GPIO_NOPULL:
            init_struct.Pull = GPIO_NOPULL;
            break;
        case RTEMS_GPIO_PULLUP:
            init_struct.Pull = GPIO_PULLUP;
            break;
        case RTEMS_GPIO_PULLDOWN:
            init_struct.Pull = GPIO_PULLDOWN;
            break;
        default:
            return RTEMS_UNSATISFIED;
    }
    
    // Pin speed
    init_struct.Speed = config->speed;

    // Pin alternate functionality
    if (mode == RTEMS_GPIO_PINMODE_BSP_SPECIFIC && 
            config->is_alternate_mode == 1) {
        init_struct.Alternate = config->alternate_fn;
    }

    // Call HAL to configure the GPIO pin
    HAL_GPIO_Init((GPIO_TypeDef *) (gpiox->port), &init_struct);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state value) {
    HAL_GPIO_WritePin((GPIO_TypeDef *) (gpiox->port), gpiox->pin_mask, value);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value) {
    *value = HAL_GPIO_ReadPin((GPIO_TypeDef *) (gpiox->port), gpiox->pin_mask);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox) {
    HAL_GPIO_TogglePin((GPIO_TypeDef *) (gpiox->port), gpiox->pin_mask);
    return RTEMS_SUCCESSFUL;
}
