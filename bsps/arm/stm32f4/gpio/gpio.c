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

static rtems_status_code stm32f4_gpio_initialize(
    rtems_gpio_ctrl_t *base
);

static rtems_status_code stm32f4_gpio_deinitialize(
    rtems_gpio_ctrl_t *base
);

static rtems_status_code stm32f4_gpio_configure(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_config_t *config
);

static rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
);

static rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_interrupt_config_t *int_conf
);

static rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_interrupt_config_t *int_conf
);

static rtems_status_code stm32f4_gpio_set_pin_mode(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_pin_mode mode
);

static rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_pull pull
);

static rtems_status_code stm32f4_gpio_read(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_pin_state *value
);
static rtems_status_code stm32f4_gpio_write(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_pin_state value
);

static rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio_ctrl_t *base,
    void *pin
);

static const rtems_gpio_handlers_t stm32f4_gpio_handlers = {
    .initialize = stm32f4_gpio_initialize,
    .deinitialize = stm32f4_gpio_deinitialize,
    .configure = stm32f4_gpio_configure,
    .configure_interrupt = stm32f4_gpio_configure_interrupt,
    .enable_interrupt = stm32f4_gpio_enable_interrupt,
    .disable_interrupt = stm32f4_gpio_disable_interrupt,
    .set_pin_mode = stm32f4_gpio_set_pin_mode,
    .set_pull = stm32f4_gpio_set_pull,
    .read = stm32f4_gpio_read,
    .write = stm32f4_gpio_write,
    .toggle = stm32f4_gpio_toggle
};

stm32f4_gpio_ctrl_t gpioa_ctrl = { .port = GPIOA };
stm32f4_gpio_ctrl_t gpiob_ctrl = { .port = GPIOB };
stm32f4_gpio_ctrl_t gpioc_ctrl = { .port = GPIOC };
stm32f4_gpio_ctrl_t gpiod_ctrl = { .port = GPIOD };
stm32f4_gpio_ctrl_t gpioe_ctrl = { .port = GPIOE };
stm32f4_gpio_ctrl_t gpiof_ctrl = { .port = GPIOF };
stm32f4_gpio_ctrl_t gpiog_ctrl = { .port = GPIOG };
stm32f4_gpio_ctrl_t gpioh_ctrl = { .port = GPIOG };
stm32f4_gpio_ctrl_t gpioi_ctrl = { .port = GPIOI };
#ifdef STM32F429X
stm32f4_gpio_ctrl_t gpioj_ctrl = { .port = GPIOJ };
stm32f4_gpio_ctrl_t gpiok_ctrl = { .port = GPIOK };
#endif /* STM32F429X */

rtems_status_code stm32f4_gpio_get_ctrl(GPIO_TypeDef *port, rtems_gpio_ctrl_t **out) {
    switch ((uintptr_t) port) {
        case (uintptr_t) GPIOA:
            *out = (rtems_gpio_ctrl_t *) &gpioa_ctrl;
            break;
        case (uintptr_t) GPIOB:
            *out = (rtems_gpio_ctrl_t *) &gpiob_ctrl;
            break;
        case (uintptr_t) GPIOC:
            *out = (rtems_gpio_ctrl_t *) &gpioc_ctrl;
            break;
        case (uintptr_t) GPIOD:
            *out = (rtems_gpio_ctrl_t *) &gpiod_ctrl;
            break;
        case (uintptr_t) GPIOE:
            *out = (rtems_gpio_ctrl_t *) &gpioe_ctrl;
            break;
        case (uintptr_t) GPIOF:
            *out = (rtems_gpio_ctrl_t *) &gpiof_ctrl;
            break;
        case (uintptr_t) GPIOG:
            *out = (rtems_gpio_ctrl_t *) &gpiog_ctrl;
            break;
        case (uintptr_t) GPIOH:
            *out = (rtems_gpio_ctrl_t *) &gpiog_ctrl;
            break;
        case (uintptr_t) GPIOI:
            *out = (rtems_gpio_ctrl_t *) &gpioi_ctrl;
            break;
#ifdef STM32F429X
        case (uintptr_t) GPIOJ:
            *out = (rtems_gpio_ctrl_t *) &gpioj_ctrl;
            break;
        case (uintptr_t) GPIOK:
            *out = (rtems_gpio_ctrl_t *) &gpiok_ctrl;
            break;
#endif /* STM32F429X */
        default:
            return RTEMS_UNSATISFIED;
    }
    if (((stm32f4_gpio_ctrl_t *) (*out))->is_registered == false) {
        rtems_gpio_register(*out, &stm32f4_gpio_handlers);
        ((stm32f4_gpio_ctrl_t *) (*out))->is_registered = true;
    }
    return RTEMS_SUCCESSFUL;
}

static stm32f4_gpio_ctrl_t *get_ctrl_from_base(
    rtems_gpio_ctrl_t *base
) 
{
    return RTEMS_CONTAINER_OF(base, stm32f4_gpio_ctrl_t, base);
}

static stm32f4_gpio_config_t *get_config_from_base(
    rtems_gpio_config_t *config
) 
{
    return RTEMS_CONTAINER_OF(config, stm32f4_gpio_config_t, base);
}

static stm32f4_gpio_interrupt_config_t *get_interrupt_config_from_base(
    rtems_gpio_interrupt_config_t *int_conf
) 
{
    return RTEMS_CONTAINER_OF(int_conf, stm32f4_gpio_interrupt_config_t, base);
}

static rtems_status_code stm32f4_gpio_initialize(rtems_gpio_ctrl_t *base) {
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    
    switch ((uintptr_t) ctrl->port) {
        case (uintptr_t) GPIOA:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOB:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOC:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOD:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOE:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOF:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOG:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOH:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOI:
            __HAL_RCC_GPIOI_CLK_ENABLE();
            break;
#ifdef STM32F429X
        case (uintptr_t) GPIOJ:
            __HAL_RCC_GPIOJ_CLK_ENABLE();
            break;
        case (uintptr_t) GPIOK:
            __HAL_RCC_GPIOK_CLK_ENABLE();
            break;
#endif /* STM32F429X */
        default:
            return RTEMS_UNSATISFIED;
    }
    return RTEMS_SUCCESSFUL;
}

static rtems_status_code stm32f4_gpio_deinitialize(rtems_gpio_ctrl_t *base) {
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    
    switch ((uintptr_t) ctrl->port) {
        case (uintptr_t) GPIOA:
            __HAL_RCC_GPIOA_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOB:
            __HAL_RCC_GPIOB_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOC:
            __HAL_RCC_GPIOC_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOD:
            __HAL_RCC_GPIOD_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOE:
            __HAL_RCC_GPIOE_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOF:
            __HAL_RCC_GPIOF_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOG:
            __HAL_RCC_GPIOG_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOH:
            __HAL_RCC_GPIOH_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOI:
            __HAL_RCC_GPIOI_CLK_DISABLE();
            break;
#ifdef STM32F429X
        case (uintptr_t) GPIOJ:
            __HAL_RCC_GPIOJ_CLK_DISABLE();
            break;
        case (uintptr_t) GPIOK:
            __HAL_RCC_GPIOK_CLK_DISABLE();
            break;
#endif /* STM32F429X */
        default:
            return RTEMS_UNSATISFIED;
    }
    return RTEMS_SUCCESSFUL;
}

/**
  * @note Warning: only one pin can be passed as argument
  * @note If using interrupt mode, use rtems_gpio_configure_interrupt().
  * @note If using alternate mode, use rtems_gpio_configure().
  */
rtems_status_code stm32f4_gpio_set_pin_mode(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_mode mode
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    uint32_t pin_mask = *(uint32_t *)pin;

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
    LL_GPIO_SetPinMode((GPIO_TypeDef *) ctrl->port, pin_mask, stm32f4_mode);
    if (stm32f4_mode == LL_GPIO_MODE_OUTPUT) {
        LL_GPIO_SetPinOutputType((GPIO_TypeDef *) ctrl->port, pin_mask, stm32f4_output_type);
    }

    return RTEMS_SUCCESSFUL;
}

/**
  * @note Warning: only one pin can be passed as argument
  */
rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pull pull
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    uint32_t pin_mask = *(uint32_t *)pin;
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
    LL_GPIO_SetPinPull((GPIO_TypeDef *) ctrl->port, pin_mask, stm32f4_pull);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_configure(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_config_t *config
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    stm32f4_gpio_config_t *stm32_conf = get_config_from_base(config);
    uint32_t pin_mask = *(uint32_t *)pin;
    GPIO_InitTypeDef init_struct;

    // Pin number
    init_struct.Pin = pin_mask;
    
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
        init_struct.Mode = stm32_conf->alternate_mode;
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
    init_struct.Speed = stm32_conf->speed;

    // Pin alternate functionality
    if (config->mode == RTEMS_GPIO_PINMODE_BSP_SPECIFIC) {
        init_struct.Alternate = stm32_conf->alternate_fn;
    }

    // Call HAL to configure the GPIO pin
    HAL_GPIO_Init(ctrl->port, &init_struct);

    return RTEMS_SUCCESSFUL;
}

/**
  * TODO
  *
  * @note This function defaults to not using pull resistor.
  *       Use rtems_gpio_set_pull() afterwards to change.
  */
rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    stm32f4_gpio_interrupt_config_t *stm32_int_conf = get_interrupt_config_from_base(int_conf);
    uint32_t pin_mask = *(uint32_t *)pin;
    GPIO_InitTypeDef hal_conf;

    switch (int_conf->interrupt_mode) {
    case RTEMS_GPIO_INT_MODE_NONE:
        return RTEMS_SUCCESSFUL;
    case RTEMS_GPIO_INT_MODE_FALLING:
        if (stm32_int_conf->is_event_mode) {
            hal_conf.Mode = GPIO_MODE_EVT_FALLING;
        } else {
            hal_conf.Mode = GPIO_MODE_IT_FALLING;
        }
        break;
    case RTEMS_GPIO_INT_MODE_RISING:
        if (stm32_int_conf->is_event_mode) {
            hal_conf.Mode = GPIO_MODE_EVT_RISING;
        } else {
            hal_conf.Mode = GPIO_MODE_IT_RISING;
        }
        break;
    case RTEMS_GPIO_INT_MODE_BOTH_EDGES:
        if (stm32_int_conf->is_event_mode) {
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
    hal_conf.Pin = pin_mask;
    HAL_GPIO_Init(ctrl->port, &hal_conf);

    HAL_NVIC_SetPriority((IRQn_Type) int_conf->interrupt_number, int_conf->priority, stm32_int_conf->subpriority);

    return RTEMS_SUCCESSFUL;
}

static rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
) 
{
    HAL_NVIC_EnableIRQ((IRQn_Type) int_conf->interrupt_number);
    return RTEMS_SUCCESSFUL;
}

static rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
) 
{
    HAL_NVIC_DisableIRQ((IRQn_Type) int_conf->interrupt_number);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_write(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_state value
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    uint32_t pin_mask = *(uint32_t *)pin;

    HAL_GPIO_WritePin(ctrl->port, pin_mask, value);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_read(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_state *value
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    uint32_t pin_mask = *(uint32_t *)pin;

    *value = HAL_GPIO_ReadPin(ctrl->port, pin_mask);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio_ctrl_t *base, 
    void *pin
) 
{
    stm32f4_gpio_ctrl_t *ctrl = get_ctrl_from_base(base);
    uint32_t pin_mask = *(uint32_t *)pin;

    HAL_GPIO_TogglePin(ctrl->port, pin_mask);
    return RTEMS_SUCCESSFUL;
}
