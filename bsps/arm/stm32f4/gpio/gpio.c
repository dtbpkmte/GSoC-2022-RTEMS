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
  * GPIO API requirement.
  */
#define BSP_GPIO_NUM_CONTROLLERS        1

/**
  * Static functions prototypes. 
  * Implementing GPIO API.
  */
static rtems_status_code stm32f4_gpio_init(
    rtems_gpio_t *base
);

static rtems_status_code stm32f4_gpio_deinit(
    rtems_gpio_t *base
);

static rtems_status_code stm32f4_gpio_set_pin_mode(
    rtems_gpio_t *base,
    rtems_gpio_pin_mode_t mode
);

static rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio_t *base,
    rtems_gpio_pull_t pull
);

static rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio_t *base, 
    rtems_gpio_isr_t isr,
    rtems_gpio_interrupt_trig_t trig,
    rtems_gpio_pull_t pull
);

static rtems_status_code stm32f4_gpio_remove_interrupt(
    rtems_gpio_t *base
);

static rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio_t *base,
);

static rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio_t *base,
);

static rtems_status_code stm32f4_gpio_read(
    rtems_gpio_t *base,
    rtems_gpio_pin_state *value
);

static rtems_status_code stm32f4_gpio_write(
    rtems_gpio_t *base,
    rtems_gpio_pin_state value
);

static rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio_t *base,
);

/**
  * @brief STM32F4 GPIO handlers
  */
static const rtems_gpio_handlers_t stm32f4_gpio_handlers = {
    .init = stm32f4_gpio_init,
    .deinit = stm32f4_gpio_deinit,
    .set_pin_mode = stm32f4_gpio_set_pin_mode,
    .set_pull = stm32f4_gpio_set_pull,
    .configure_interrupt = stm32f4_gpio_configure_interrupt,
    .remove_interrupt = stm32f4_gpio_remove_interrupt,
    .enable_interrupt = stm32f4_gpio_enable_interrupt,
    .disable_interrupt = stm32f4_gpio_disable_interrupt,
    .read = stm32f4_gpio_read,
    .write = stm32f4_gpio_write,
    .toggle = stm32f4_gpio_toggle
};

static const GPIO_TypeDef *GPIOx[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
    GPIOF, GPIOG, GPIOH, GPIOI,
#ifdef STM32F429X
    GPIOJ, GPIOK
#endif /* STM32F429X */
};

#define STM32F4_GET_PORT(interm_pin) ( GPIOx[ ( interm_pin ) / 16 ] )

#define STM32F4_GET_PIN_0_15(interm_pin) ( ( interm_pin ) % 16 ) 

#define STM32F4_GET_HAL_GPIO_PIN(pin) ( GPIO_PIN_##pin )

#define STM32F4_GET_LL_EXTI_LINE(pin) ( LL_EXTI_LINE_##pin )

#define EXTI5_IRQn      EXTI9_5_IRQn
#define EXTI6_IRQn      EXTI9_5_IRQn
#define EXTI7_IRQn      EXTI9_5_IRQn
#define EXTI8_IRQn      EXTI9_5_IRQn
#define EXTI9_IRQn      EXTI9_5_IRQn
#define EXTI10_IRQn     EXTI15_10_IRQn
#define EXTI11_IRQn     EXTI15_10_IRQn
#define EXTI12_IRQn     EXTI15_10_IRQn
#define EXTI13_IRQn     EXTI15_10_IRQn
#define EXTI14_IRQn     EXTI15_10_IRQn
#define EXTI15_IRQn     EXTI15_10_IRQn

#define STM32F4_GET_EXTI_IRQn(pin) ( EXTI##pin##_IRQn )

/********** STM32F4 GPIO API functions ************/

rtems_status_code stm32f4_gpio_get(
    uint32_t interm_pin,
    rtems_gpio_t **out
)
{
    stm32f4_gpio_t *tmp = (stm32f4_gpio_t *) malloc(sizeof(stm32f4_gpio_t));
    if (tmp == NULL) {
        return RTEMS_NO_MEMORY;
    }
    tmp->base = { .handlers = &stm32f4_gpio_handlers };
    tmp->pin = STM32F4_GET_PIN_0_15(interm_pin);
    tmp->port = STM32F4_GET_PORT(interm_pin);
    
    *out = tmp;
    return RTEMS_SUCCESSFUL;
}

static stm32f4_gpio_t *get_gpio_from_base(
    rtems_gpio_t *base
) 
{
    return RTEMS_CONTAINER_OF(base, stm32f4_gpio_t, base);
}

static rtems_status_code stm32f4_gpio_init(rtems_gpio_t *base) {
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    
    switch ((uintptr_t) gpio->port) {
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

static rtems_status_code stm32f4_gpio_deinit(rtems_gpio_t *base) {
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    
    switch ((uintptr_t) gpio->port) {
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
    rtems_gpio_t *base, 
    rtems_gpio_pin_mode_t mode
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);

    uint32_t stm32f4_mode, stm32f4_output_type;
    switch (mode) {
    case RTEMS_GPIO_PINMODE_OUTPUT:
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
        stm32f4_mode = LL_GPIO_MODE_ALTERNATE;
        break;
    default:
        /* illegal argument */
        return RTEMS_UNSATISFIED;
    }
    LL_GPIO_SetPinMode(gpio->port, pin_mask, stm32f4_mode);
    if (stm32f4_mode == LL_GPIO_MODE_OUTPUT) {
        LL_GPIO_SetPinOutputType(gpio->port, pin_mask, stm32f4_output_type);
    }

    return RTEMS_SUCCESSFUL;
}

/**
  * @note Warning: only one pin can be passed as argument
  */
rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio_t *base, 
    rtems_gpio_pull_t pull
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);
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
    LL_GPIO_SetPinPull(gpio->port, gpio->pin, stm32f4_pull);
    return RTEMS_SUCCESSFUL;
}

/**
  * TODO
  *
  * @note This function defaults to not using pull resistor.
  *       Use rtems_gpio_set_pull() afterwards to change.
  */
rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio_t *base, 
    rtems_gpio_isr_t isr,
    void *arg,
    rtems_gpio_interrupt_trig_t trig,
    rtems_gpio_pull_t pull
) 
{
    // configure pin
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);
    GPIO_InitTypeDef hal_conf;

    switch (trig) {
    case RTEMS_GPIO_INT_TRIG_NONE:
        return RTEMS_SUCCESSFUL;
    case RTEMS_GPIO_INT_TRIG_FALLING:
        hal_conf.Mode = GPIO_MODE_IT_FALLING;
        break;
    case RTEMS_GPIO_INT_TRIG_RISING:
        hal_conf.Mode = GPIO_MODE_IT_RISING;
        break;
    case RTEMS_GPIO_INT_TRIG_BOTH_EDGES:
        hal_conf.Mode = GPIO_MODE_IT_RISING_FALLING;
        break;
    default:
        /* Invalid argument */
        return RTEMS_UNSATISFIED;
    }
    switch (pull) {
    case RTEMS_GPIO_NOPULL:
        hal_conf.Pull = GPIO_NOPULL;
        break;
    case RTEMS_GPIO_PULLUP:
        hal_conf.Pull = GPIO_PULLUP;
        break;
    case RTEMS_GPIO_PULLDOWN:
        hal_conf.Pull = GPIO_PULLDOWN;
        break;
    default:
        /* Illegal argument */
        return RTEMS_UNSATISFIED;
    }
    hal_conf.Pin = pin_mask;
    HAL_GPIO_Init(gpio->port, &hal_conf);

    // RTEMS interrupt config
    rtems_status_code sc = rtems_interrupt_handler_install(
            STM32F4_GET_EXTI_IRQn(gpio->pin), 
            NULL, 
            RTEMS_INTERRUPT_UNIQUE, 
            isr, 
            NULL);

    return sc;
}

static rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio_t *base
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(pin_mask);
    LL_EXTI_EnableIT_0_31(STM32F4_GET_LL_EXTI_LINE(gpio->pin));
    return RTEMS_SUCCESSFUL;
}

static rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio_t *base
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(pin_mask);
    LL_EXTI_DisableIT_0_31(STM32F4_GET_LL_EXTI_LINE(gpio->pin));
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_write(
    rtems_gpio_t *base, 
    rtems_gpio_pin_state value
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);

    HAL_GPIO_WritePin(gpio->port, pin_mask, value);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_read(
    rtems_gpio_t *base, 
    rtems_gpio_pin_state *value
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);

    *value = HAL_GPIO_ReadPin(gpio->port, pin_mask);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio_t *base, 
) 
{
    stm32f4_gpio_t *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);

    HAL_GPIO_TogglePin(gpio->port, pin_mask);
    return RTEMS_SUCCESSFUL;
}

//void exti0_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti0_isr(stm32_arg);
//    }
//}
//void exti1_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti1_isr(stm32_arg);
//    }
//}
//void exti2_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti2_isr(stm32_arg);
//    }
//}
//void exti3_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti3_isr(stm32_arg);
//    }
//}
//void exti4_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti4_isr(stm32_arg);
//    }
//}
//void exti5_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti5_isr(stm32_arg);
//    }
//}
//void exti6_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti6_isr(stm32_arg);
//    }
//}
//void exti7_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti7_isr(stm32_arg);
//    }
//}
//void exti8_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti8_isr(stm32_arg);
//    }
//}
//void exti9_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti9_isr(stm32_arg);
//    }
//}
//void exti10_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti10_isr(stm32_arg);
//    }
//}
//void exti11_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti11_isr(stm32_arg);
//    }
//}
//void exti12_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti12_isr(stm32_arg);
//    }
//}
//void exti13_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti13_isr(stm32_arg);
//    }
//}
//void exti14_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti14_isr(stm32_arg);
//    }
//}
//void exti15_handler(void *arg) {
//    stm32f4_interrupt_arg_t *stm32_arg;
//    if(__HAL_GPIO_EXTI_GET_IT(stm32_arg->pin) != RESET)
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(stm32_arg->pin);
//        stm32f4_isr_table->exti15_isr(stm32_arg);
//    }
//}
//void stm32f4_default_exti_isr(stm32f4_interrupt_arg_t * arg) {
//    (void) (0U);
//}
//
//typedef struct {
//    uint32_t pin;
//} stm32f4_interrupt_arg_t;
//
//typedef (void *) (stm32f4_interrupt_arg_t *) stm32f4_isr_f;
//
//struct {
//    stm32f4_isr_f exti0_isr;
//    stm32f4_isr_f exti1_isr;
//    stm32f4_isr_f exti2_isr;
//    stm32f4_isr_f exti3_isr;
//    stm32f4_isr_f exti4_isr;
//    stm32f4_isr_f exti5_isr;
//    stm32f4_isr_f exti6_isr;
//    stm32f4_isr_f exti7_isr;
//    stm32f4_isr_f exti8_isr;
//    stm32f4_isr_f exti9_isr;
//    stm32f4_isr_f exti10_isr;
//    stm32f4_isr_f exti11_isr;
//    stm32f4_isr_f exti12_isr;
//    stm32f4_isr_f exti13_isr;
//    stm32f4_isr_f exti14_isr;
//    stm32f4_isr_f exti15_isr;
//} stm32f4_isr_table;
