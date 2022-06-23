/**
  * @file
  *
  * @ingroup rtems_bsp/arm/stm32f4
  *
  * @brief RTEMS GPIO new API implementation for STM32F4.
  *
  * @note RTEMS_GPIO_PINMODE_BSP_SPECIFIC is Alternate mode for STM32F4 BSP
  */

#include <bsp.h>
#include <rtems.h>
#include <bsp/stm32f4_gpio.h>

/**
  * @note Warning: only one pin can be passed as argument
  * @note If using interrupt mode, use rtems_gpio_configure_interrupt().
  * @note If using alternate mode, use rtems_gpio_configure().
  */
rtems_status_code rtems_gpio_set_pin_mode(rtems_gpio_t *gpiox, rtems_gpio_pin_mode mode) {
    uint32_t stm32f4_mode, stm32f4_output_type;
    switch (mode) {
    case RTEMS_GPIO_PINMODE_OUTPUT_PP:
        stm32f4_mode = LL_GPIO_MODE_OUTPUT;
        stm32f4_output_type = LL_GPIO_OUTPUT_PUSHPULL;
        break;
    case RTEMS_GPIO_PINMODE_OUTPUT_OD:
        stm32f4_mode = LL_GPIO_MODE_OUTPUT;
        stm32f4_output_type = LL_GPIO_OUTPUT_OPENDRAIN;
        break;
    case RTEMS_GPIO_PINMODE_INPUT:
        stm32f4_mode = LL_GPIO_MODE_INPUT;
        break;
    case RTEMS_GPIO_PINMODE_ANALOG:
        stm32f4_mode = LL_GPIO_MODE_ANALOG;
        break;
    case RTEMS_GPIO_PINMODE_BSP_SPECIFIC:
        /* use rtems_gpio_configure() instead */
        stm32f4_mode = LL_GPIO_MODE_ALTERNATE;
        break;
    default:
        /* illegal argument */
        return RTEMS_UNSATISFIED;
    }
    LL_GPIO_SetPinMode((GPIO_TypeDef *) gpiox->port, (uint32_t) (gpiox->pin_mask), stm32f4_mode);
    if (stm32f4_mode == LL_GPIO_MODE_OUTPUT) {
        LL_GPIO_SetPinOutputType((GPIO_TypeDef *) gpiox-> port, (uint32_t) (gpiox->pin_mask), stm32f4_output_type);
    }

    return RTEMS_SUCCESSFUL;
}

/**
  * @note Warning: only one pin can be passed as argument
  */
rtems_status_code rtems_gpio_set_pull(rtems_gpio_t *gpiox, rtems_gpio_pull pull) {
    uint32_t stm32f4_pull;
    switch (pull) {
    case RTEMS_GPIO_NOPULL:
        stm32f4_pull = LL_GPIO_PULL_NO;
        break;
    case RTEMS_GPIO_PULLUP:
        stm32f4_pull = LL_GPIO_PULL_UP;
        break;
    case RTEMS_GPIO_PULLDOWN:
        stm32f4_pull = LL_GPIO_PULL_DOWN;
        break;
    default:
        /* Illegal argument */
        return RTEMS_UNSATISFIED;
    }
    LL_GPIO_SetPinPull((GPIO_TypeDef *) gpiox->port, (uint32_t) (gpiox->pin_mask), stm32f4_pull);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_config_t *config) {
    GPIO_InitTypeDef init_struct;

    // Pin number
    init_struct.Pin = (uint16_t) (gpiox->pin_mask);
    
    // Pin mode
    switch (config->mode) {
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
    case RTEMS_GPIO_PINMODE_BSP_SPECIFIC:
        /* Alternate mode */
        init_struct.Mode = ((stm32f4_gpio_config_t *) config->bsp)->alternate_mode;
        break;
    default:
        /* illegal argument */
        return RTEMS_UNSATISFIED;
    }

    // Pin pull resistor
    switch (config->pull) {
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
    init_struct.Speed = ((stm32f4_gpio_config_t *) (config->bsp))->speed;

    // Pin alternate functionality
    if (config->mode == RTEMS_GPIO_PINMODE_BSP_SPECIFIC) {
        init_struct.Alternate = ((stm32f4_gpio_config_t *) (config->bsp))->alternate_fn;
    }

    // Call HAL to configure the GPIO pin
    HAL_GPIO_Init((GPIO_TypeDef *) gpiox->port, &init_struct);

    return RTEMS_SUCCESSFUL;
}

/**
  * TODO
  *
  * @note This function defaults to not using pull resistor.
  *       Use rtems_gpio_set_pull() afterwards to change.
  */
rtems_status_code rtems_gpio_configure_interrupt(rtems_gpio_t *gpiox, rtems_gpio_interrupt_config_t *int_conf) {
    GPIO_InitTypeDef hal_conf;

    switch (int_conf->interrupt_mode) {
    case RTEMS_GPIO_INT_MODE_NONE:
        return RTEMS_SUCCESSFUL;
    case RTEMS_GPIO_INT_MODE_FALLING:
        if (((stm32f4_gpio_interrupt_config_t *) int_conf->bsp)->is_event_mode) {
            hal_conf.Mode = GPIO_MODE_EVT_FALLING;
        } else {
            hal_conf.Mode = GPIO_MODE_IT_FALLING;
        }
        break;
    case RTEMS_GPIO_INT_MODE_RISING:
        if (((stm32f4_gpio_interrupt_config_t *) int_conf->bsp)->is_event_mode) {
            hal_conf.Mode = GPIO_MODE_EVT_RISING;
        } else {
            hal_conf.Mode = GPIO_MODE_IT_RISING;
        }
        break;
    case RTEMS_GPIO_INT_MODE_BOTH_EDGES:
        if (((stm32f4_gpio_interrupt_config_t *) int_conf->bsp)->is_event_mode) {
            hal_conf.Mode = GPIO_MODE_EVT_RISING_FALLING;
        } else {
            hal_conf.Mode = GPIO_MODE_IT_RISING_FALLING;
        }
        break;
    default:
        /* Invalid argument */
        return RTEMS_UNSATISFIED;
    }
    hal_conf.Pull = GPIO_NOPULL;
    hal_conf.Pin = gpiox->pin_mask;
    HAL_GPIO_Init((GPIO_TypeDef *) gpiox->port, &hal_conf);

    HAL_NVIC_SetPriority((IRQn_Type) int_conf->interrupt_number, int_conf->priority, ((stm32f4_gpio_interrupt_config_t *) int_conf->bsp)->subpriority);
    HAL_NVIC_EnableIRQ((IRQn_Type) int_conf->interrupt_number);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_write_pin_default(rtems_gpio_t *gpiox, rtems_gpio_pin_state value) {
    HAL_GPIO_WritePin((GPIO_TypeDef *) gpiox->port, gpiox->pin_mask, value);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_read_pin_default(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value) {
    *value = HAL_GPIO_ReadPin((GPIO_TypeDef *) gpiox->port, gpiox->pin_mask);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_toggle_pin_default(rtems_gpio_t *gpiox) {
    HAL_GPIO_TogglePin((GPIO_TypeDef *) gpiox->port, gpiox->pin_mask);
    return RTEMS_SUCCESSFUL;
}
