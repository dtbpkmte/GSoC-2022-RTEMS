/**
  * @file
  *
  * @ingroup rtems_bsp/arm/stm32f4
  *
  * @brief RTEMS GPIO new API implementation for STM32F4.
  *
  * @note RTEMS_GPIO_PINMODE_BSP_SPECIFIC is Alternate mode for STM32F4 BSP
  */

/*
 *  Copyright (c) 2022 Duc Doan <dtbpkmte at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp.h>
#include <rtems.h>
#include <bsp/stm32f4_gpio.h>
#include <stdlib.h>

/*********** Helpers *****************/
/**
  * @brief Macro to get stm32f4_gpio object from a base rtems_gpio
  *        object.
  *
  * This is a wrapper of RTEMS_CONTAINER_OF macro
  *
  * @param base The pointer to a rtems_gpio object
  * @retval The pointer to the stm32f4_gpio object owning
  *         the specified rtems_gpio object
  */
#define get_gpio_from_base(base) \
    RTEMS_CONTAINER_OF(base, stm32f4_gpio, base)

/*********** GPIO API ***************/
static rtems_status_code stm32f4_gpio_get(
    uint32_t interm_pin,
    rtems_gpio **out
);

static rtems_status_code stm32f4_gpio_destroy(
    rtems_gpio *base
);

static rtems_status_code stm32f4_gpio_init(
    rtems_gpio *base
);

static rtems_status_code stm32f4_gpio_deinit(
    rtems_gpio *base
);

static rtems_status_code stm32f4_gpio_set_pin_mode(
    rtems_gpio *base,
    rtems_gpio_pin_mode mode
);

static rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio *base,
    rtems_gpio_pull pull
);

static rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio *base, 
    rtems_gpio_isr isr,
    void *arg,
    rtems_gpio_interrupt_trig trig,
    rtems_gpio_pull pull
);

static rtems_status_code stm32f4_gpio_remove_interrupt(
    rtems_gpio *base
);

static rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio *base
);

static rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio *base
);

static rtems_status_code stm32f4_gpio_read(
    rtems_gpio *base,
    rtems_gpio_pin_state *value
);

static rtems_status_code stm32f4_gpio_write(
    rtems_gpio *base,
    rtems_gpio_pin_state value
);

static rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio *base
);

/*********************************************************/

/**
  * @brief STM32F4 GPIO handlers
  */
static const rtems_gpio_handlers stm32f4_gpio_handlers = {
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

static GPIO_TypeDef * const GPIOx[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
    GPIOF, GPIOG, GPIOH, GPIOI,
#ifdef STM32F429X
    GPIOJ, GPIOK
#endif /* STM32F429X */
};

static unsigned int const EXTIx_IRQn[] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn,
    EXTI15_10_IRQn
};

/**
  * @brief Converts intermediate pin number to port pointer.
  *
  * Intermediate pin number is a way of numerically labeling
  * pins. Pins are labeled incrementally across all ports.
  * Pins 0-15 from port A are 0-15. Pins 0-15 from port B are
  * 16-31. And so on.
  *
  * @param interm_pin is the intermediate pin number
  */
#define STM32F4_GET_PORT(interm_pin) (GPIOx[ ( interm_pin ) / 16 ])

/**
  * @brief Converts intermediate pin number to 0-15.
  *
  * Intermediate pin number is a way of numerically labeling
  * pins. Pins are labeled incrementally across all ports.
  * Pins 0-15 from port A are 0-15. Pins 0-15 from port B are
  * 16-31. And so on.
  *
  * @param interm_pin is the intermediate pin number
  */
#define STM32F4_GET_PIN_0_15(interm_pin) (( interm_pin ) % 16) 

/**
  * @brief Converts pin number from 0-15 to HAL pin mask.
  * @param pin is the pin number from 0-15
  */
#define STM32F4_GET_HAL_GPIO_PIN(pin) ((uint16_t) (1 << ( pin )))

/**
  * @brief Get EXTI Line from pin number 0-15
  * @param pin is the pin number from 0-15
  */
#define STM32F4_GET_LL_EXTI_LINE(pin) (0x1UL << ( pin ))

/**
  * @brief Get EXTI IRQ number from pin 0-15
  * @param pin is the pin number from 0-15
  */
#define STM32F4_GET_EXTI_IRQn(pin) (EXTIx_IRQn[( pin )])

/************** Interrupt manager *****************/
typedef struct {
    void *arg;
    stm32f4_gpio *gpio;
} stm32f4_interrupt_arg;

typedef struct {
    stm32f4_interrupt_arg arg;
    rtems_gpio_isr isr;
} stm32f4_interrupt;

static stm32f4_interrupt isr_table[16]; 

void exti_handler(void *arg);


/********** STM32F4 GPIO API functions ************/

rtems_status_code bsp_gpio_register_controllers(
    void
)
{
    return rtems_gpio_register(
            stm32f4_gpio_get,
            stm32f4_gpio_destroy,
            sizeof(GPIOx)/sizeof(GPIOx[0])*16
    );
}

rtems_status_code stm32f4_gpio_get(
    uint32_t interm_pin,
    rtems_gpio **out
)
{
    stm32f4_gpio *tmp = malloc(sizeof(stm32f4_gpio));
    if (tmp == NULL) {
        return RTEMS_NO_MEMORY;
    }
    tmp->base = (rtems_gpio) { .handlers = &stm32f4_gpio_handlers };
    tmp->pin = STM32F4_GET_PIN_0_15(interm_pin);
    tmp->port = STM32F4_GET_PORT(interm_pin);
    
    *out = (rtems_gpio *) tmp;
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_destroy(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    free(gpio);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_init(rtems_gpio *base) {
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    
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

rtems_status_code stm32f4_gpio_deinit(rtems_gpio *base) {
/*
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    
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
#endif 
        default:
            return RTEMS_UNSATISFIED;
    }
    return RTEMS_SUCCESSFUL;
*/
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_gpio_set_pin_mode(
    rtems_gpio *base, 
    rtems_gpio_pin_mode mode
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(gpio->pin);

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

rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio *base, 
    rtems_gpio_pull pull
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);
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
    LL_GPIO_SetPinPull(gpio->port, pin_mask, stm32f4_pull);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio *base, 
    rtems_gpio_isr isr,
    void *arg,
    rtems_gpio_interrupt_trig trig,
    rtems_gpio_pull pull
) 
{
    // configure pin
    stm32f4_gpio *gpio = get_gpio_from_base(base);
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
    isr_table[gpio->pin] = (stm32f4_interrupt){
        .arg = {
            .arg = arg,
            .gpio = gpio
        },
        .isr = isr
    };
    rtems_option opt = gpio->pin < 5 ? 
                        RTEMS_INTERRUPT_UNIQUE : 
                        RTEMS_INTERRUPT_SHARED;
    rtems_status_code sc = rtems_interrupt_handler_install(
            STM32F4_GET_EXTI_IRQn(gpio->pin), 
            NULL, 
            opt, 
            exti_handler, 
            &isr_table[gpio->pin].arg
    );

    return sc;
}

rtems_status_code stm32f4_gpio_remove_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    rtems_status_code sc = rtems_interrupt_handler_remove(
            STM32F4_GET_EXTI_IRQn(gpio->pin), 
            exti_handler, 
            &isr_table[gpio->pin].arg
    );
    if (sc == RTEMS_SUCCESSFUL) {
        isr_table[gpio->pin] = (stm32f4_interrupt){0};
    }
    return sc;
}

rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio *base
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    LL_EXTI_EnableIT_0_31(STM32F4_GET_LL_EXTI_LINE(gpio->pin));
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio *base
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);
    LL_EXTI_DisableIT_0_31(STM32F4_GET_LL_EXTI_LINE(gpio->pin));
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_write(
    rtems_gpio *base, 
    rtems_gpio_pin_state value
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);

    if (value)
        LL_GPIO_SetOutputPin(gpio->port, STM32F4_GET_HAL_GPIO_PIN(gpio->pin));
    else
        LL_GPIO_ResetOutputPin(gpio->port, STM32F4_GET_HAL_GPIO_PIN(gpio->pin));

    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_read(
    rtems_gpio *base, 
    rtems_gpio_pin_state *value
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);

    *value = LL_GPIO_IsInputPinSet(
            gpio->port, 
            STM32F4_GET_HAL_GPIO_PIN(gpio->pin)
    );
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio *base
) 
{
    stm32f4_gpio *gpio = get_gpio_from_base(base);

    LL_GPIO_TogglePin(
            gpio->port, 
            STM32F4_GET_HAL_GPIO_PIN(gpio->pin)
    );
    return RTEMS_SUCCESSFUL;
}

void exti_handler(void *arg) {
    stm32f4_interrupt_arg *stm32_arg = (stm32f4_interrupt_arg *) arg;
    uint32_t pin_mask = STM32F4_GET_HAL_GPIO_PIN(stm32_arg->gpio->pin);
    if(__HAL_GPIO_EXTI_GET_IT(pin_mask) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(pin_mask);
        (*isr_table[stm32_arg->gpio->pin].isr)(stm32_arg->arg);
    }
}

