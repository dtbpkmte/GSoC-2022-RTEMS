#ifndef LIBBSP_ARM_STM32F4_BSP_ADC
#define LIBBSP_ARM_STM32F4_BSP_ADC

#include <bsp/stm32f4_adc.h>

#if defined(ADC3)
#define NUM_ADC 3
#elif defined(ADC2)
#define NUM_ADC 2
#else
#define NUM_ADC 1
#endif

#define STM32F4_GET_ADCx_FROM_NUMBER(num)       (\
    ( num ) == 1                 ? ADC1 :        \
    ( num ) == 2 && NUM_ADC >= 2 ? ADC2 :        \
    ( num ) == 3 && NUM_ADC == 3 ? ADC3 :        \
                                   NULL)

/************** Interrupt manager *****************/
typedef struct {
    void *arg;
    stm32f4_gpio *gpio;
} stm32f4_interrupt_arg;

typedef struct {
    stm32f4_interrupt_arg arg;
    rtems_gpio_isr isr;
} stm32f4_interrupt;

static void adc_irq_handler(void *arg);

static stm32f4_interrupt isr_table[NUM_ADC];

/**
  * This tells if there is already an ISR registered at each
  * table index (element set to true).
  */
static bool isr_registered[NUM_ADC] = {0};

typedef struct {
    uint32_t adc_value;
    rtems_adc_status status;
} stm32f4_adc_data;
static stm32f4_adc_data adc_data[NUM_ADC] = {0};

static stm32f4_interrupt nb_interrupt = 
{
    .arg = {
        .arg = NULL,
        .gpio = NULL
    },
    .isr = adc_irq_handler
};


static void stm32f4_adc_select_channel(
    stm32f4_gpio *gpio
);

/***************/
ADC_TypeDef *stm32f4_get_ADCx(
    GPIO_TypeDef *gpio
)
{
    switch ((uintptr_t) gpio) {
#if defined(STM32F405xx) || defined(STM32F407xx)
        case (uintptr_t) GPIOA:
        case (uintptr_t) GPIOB:
        case (uintptr_t) GPIOC:
            return ADC1;
        case (uintptr_t) GPIOF:
            return ADC3;
#endif /* defined(STM32F405xx) || defined(STM32F407xx) */
        default:
            return NULL;
    }
}

rtems_status_code stm32f4_get_LL_ADC_CHANNEL(
    stm32f4_gpio *gpio,
    uint32_t *channel
)
{
    uintptr_t port = (uintptr_t) gpio->port;
#if defined(STM32F405xx) || defined(STM32F407xx)
    if (port == (uintptr_t) GPIOA) {
        switch (gpio->pin) {
            case 0:
                *channel = LL_ADC_CHANNEL_0;
                break;
            case 1:
                *channel = LL_ADC_CHANNEL_1;
                break;
            case 2:
                *channel = LL_ADC_CHANNEL_2;
                break;
            case 3:
                *channel = LL_ADC_CHANNEL_3;
                break;
            case 4:
                *channel = LL_ADC_CHANNEL_4;
                break;
            case 5:
                *channel = LL_ADC_CHANNEL_5;
                break;
            case 6:
                *channel = LL_ADC_CHANNEL_6;
                break;
            case 7:
                *channel = LL_ADC_CHANNEL_7;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
    }
    else if (port == (uintptr_t) GPIOB) {
        switch (gpio->pin) {
            case 0:
                *channel = LL_ADC_CHANNEL_8;
                break;
            case 1:
                *channel = LL_ADC_CHANNEL_9;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
    }
    else if (port == (uintptr_t) GPIOC) {
        switch (gpio->pin) {
            case 0:
                *channel = LL_ADC_CHANNEL_10;
                break;
            case 1:
                *channel = LL_ADC_CHANNEL_11;
                break;
            case 2:
                *channel = LL_ADC_CHANNEL_12;
                break;
            case 3:
                *channel = LL_ADC_CHANNEL_13;
                break;
            case 4:
                *channel = LL_ADC_CHANNEL_14;
                break;
            case 5:
                *channel = LL_ADC_CHANNEL_15;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
    }
    else if (port == (uintptr_t) GPIOF) {
        switch (gpio->pin) {
            case 3:
                *channel = LL_ADC_CHANNEL_9;
                break;
            case 4:
                *channel = LL_ADC_CHANNEL_14;
                break;
            case 5:
                *channel = LL_ADC_CHANNEL_15;
                break;
            case 6:
                *channel = LL_ADC_CHANNEL_4;
                break;
            case 7:
                *channel = LL_ADC_CHANNEL_5;
                break;
            case 8:
                *channel = LL_ADC_CHANNEL_6;
                break;
            case 9:
                *channel = LL_ADC_CHANNEL_7;
                break;
            case 10:
                *channel = LL_ADC_CHANNEL_8;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
    }
    else
        return RTEMS_UNSATISFIED;
#endif /* defined(STM32F405xx) || defined(STM32F407xx) */
    return RTEMS_SUCCESSFUL;
}

bool stm32f4_is_adc_pin(
    stm32f4_gpio *gpio
)
{
    uint32_t tmp;
    return stm32f4_get_LL_ADC_CHANNEL(gpio, &tmp) == RTEMS_SUCCESSFUL;
}

static const rtems_adc_handlers stm32f4_adc_handlers = {
    .read_raw = stm32f4_adc_read_raw,
    .start_read_raw_nb = stm32f4_adc_start_read_raw_nb,
    .read_raw_nb = stm32f4_adc_read_raw_nb,
    .set_resolution = stm32f4_adc_set_resolution,
    .set_alignment = stm32f4_adc_set_alignment,
    .configure_interrupt = stm32f4_adc_configure_interrupt,
    .remove_interrupt = stm32f4_adc_remove_interrupt,
    .enable_interrupt = stm32f4_adc_enable_interrupt,
    .disable_interrupt = stm32f4_adc_disable_interrupt
};

rtems_adc_handlers *stm32f4_get_adc_handlers(
    void
)
{
    return &stm32f4_adc_handlers;
}

static void stm32f4_adc_select_channel(
    stm32f4_gpio *gpio
)
{
    uint32_t adc_channel;
    stm32f4_get_LL_ADC_CHANNEL(gpio, &adc_channel);
    LL_ADC_REG_SetSequencerRanks(gpio->ADCx, LL_ADC_REG_RANK_1, adc_channel);
    LL_ADC_SetChannelSamplingTime(gpio->ADCx, adc_channel, STM32F4_ADC_DEFAULT_SAMPLINGTIME);
}

rtems_status_code stm32f4_adc_start(
    void
)
{
    // Install ISR for non-blocking read
    return rtems_interrupt_handler_install(
            ADC_IRQn,
            NULL,
            RTEMS_INTERRUPT_SHARED,
            adc_irq_handler,
            NULL
    );
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_init(
    stm32f4_gpio *gpio
)
{
    if (!LL_ADC_IsEnabled(gpio->ADCx)) {
        // Enable clock
        switch ((uintptr_t) gpio->ADCx) {
            case (uintptr_t) ADC1:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
                break;
            case (uintptr_t) ADC2:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
                break;
            case (uintptr_t) ADC3:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
                break;
            default:
                return RTEMS_UNSATISFIED;
        }

        // ADC common setup
        stm32f4_adc_set_resolution((rtems_gpio *) gpio, STM32F4_ADC_DEFAULT_RESOLUTION);
        stm32f4_adc_set_alignment((rtems_gpio *) gpio, STM32F4_ADC_DEFAULT_ALIGNMENT);
        LL_ADC_SetSequencersScanMode(gpio->ADCx, LL_ADC_SEQ_SCAN_DISABLE);

        LL_ADC_REG_SetTriggerSource(gpio->ADCx, LL_ADC_REG_TRIG_SOFTWARE);
        LL_ADC_REG_SetSequencerLength(gpio->ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);
        LL_ADC_REG_SetSequencerDiscont(gpio->ADCx, LL_ADC_REG_SEQ_DISCONT_DISABLE);
        LL_ADC_REG_SetContinuousMode(gpio->ADCx, LL_ADC_REG_CONV_SINGLE);
        LL_ADC_REG_SetDMATransfer(gpio->ADCx, LL_ADC_REG_DMA_TRANSFER_NONE);

        LL_ADC_REG_SetFlagEndOfConversion(gpio->ADCx, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);

        LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(gpio->ADCx), LL_ADC_CLOCK_SYNC_PCLK_DIV2);
        LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(gpio->ADCx), LL_ADC_MULTI_INDEPENDENT);

        LL_ADC_Enable(gpio->ADCx);
    }

    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_read_raw(
    rtems_gpio *base,
    uint32_t *result,
    uint32_t timeout
)
{
    uint32_t tickstart = 0U;
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    stm32f4_adc_select_channel(gpio);
    LL_ADC_REG_StartConversionSWStart(gpio->ADCx);
    while (!LL_ADC_IsActiveFlag_EOCS(gpio->ADCx)) {
        if (timeout != RTEMS_ADC_NO_TIMEOUT) {
            if (timeout == 0U || ((HAL_GetTick() - tickstart) > timeout)) {
                if (!LL_ADC_IsActiveFlag_EOCS(gpio->ADCx)) {
                    return RTEMS_TIMEOUT;
                }
            }
        }
    }
    *result = LL_ADC_REG_ReadConversionData32(gpio->ADCx);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_start_read_raw_nb(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_num = STM32F4_GET_ADC_NUMBER(stm32_arg->gpio->ADCx);
    if (adc_data[adc_num].status == RTEMS_ADC_NOT_STARTED) {
        adc_data[adc_num].status = RTEMS_ADC_NOT_READY;
        // start conversion here
        LL_ADC_EnableIT_EOCS(gpio->ADCx);
        LL_ADC_REG_StartConversionSWStart(gpio->ADCx);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

rtems_adc_status stm32f4_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_num = STM32F4_GET_ADC_NUMBER(stm32_arg->gpio->ADCx);
    rtems_adc_status ret = adc_data[adc_num].status;
    if (ret == RTEMS_ADC_READY) {
        *result = adc_data[adc_num].adc_value;
        adc_data[adc_num].status = RTEMS_ADC_NOT_STARTED;
    }
    return ret;
}

rtems_status_code stm32f4_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    uint32_t ll_res;
    switch (bits) {
        case 12:
            ll_res = LL_ADC_RESOLUTION_12B;
            break;
        case 10:
            ll_res = LL_ADC_RESOLUTION_10B;
            break;
        case 8:
            ll_res = LL_ADC_RESOLUTION_8B;
            break;
        case 6:
            ll_res = LL_ADC_RESOLUTION_6B;
            break;
        default:
            return RTEMS_UNSATISFIED;
    }
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    LL_ADC_SetResolution(gpio->ADCx, ll_res);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    uint32_t ll_align;
    switch (align) {
        case RTEMS_ADC_ALIGN_LEFT:
            ll_align = LL_ADC_DATA_ALIGN_LEFT;
            break;
        case RTEMS_ADC_ALIGN_RIGHT:
            ll_align = LL_ADC_DATA_ALIGN_RIGHT;
            break;
        default:
            return RTEMS_UNSATISFIED;
    }
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    LL_ADC_SetDataAlignment(gpio->ADCx, ll_align);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_num = STM32F4_GET_ADC_NUMBER(gpio->ADCx);
    if (!isr_registered[adc_num]) {
        isr_table[adc_num] = (stm32f4_interrupt){
            .arg = {
                .arg = arg,
                .gpio = gpio
            },
            .isr = isr
        };
        isr_registered[adc_num] = true;
        return rtems_interrupt_handler_install(
                ADC_IRQn,
                NULL,
                RTEMS_INTERRUPT_SHARED,
                adc_irq_handler,
                &isr_table[adc_num].arg
        );
    }
    return RTEMS_UNSATISFIED;
}

rtems_status_code stm32f4_adc_remove_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_num = STM32F4_GET_ADC_NUMBER(gpio->ADCx);
    rtems_status_code sc = rtems_interrupt_handler_remove(
            ADC_IRQn,
            adc_irq_handler,
            &isr_table[adc_num].arg
    );
    if (sc == RTEMS_SUCCESSFUL) {
        isr_registered[adc_num] = false;
    }
    return sc;
}

rtems_status_code stm32f4_adc_enable_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    if (isr_registered[STM32F4_GET_ADC_NUMBER(gpio->ADCx)]) {
        LL_ADC_EnableIT_EOCS(gpio->ADCx);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

rtems_status_code stm32f4_adc_disable_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    if (isr_registered[STM32F4_GET_ADC_NUMBER(gpio->ADCx)]) {
        LL_ADC_DisableIT_EOCS(gpio->ADCx);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

static void adc_irq_handler(void *arg) {
    unsigned int i;
    for (i = 0; i < NUM_ADC; ++i) {
        ADC_TypeDef *adcx = STM32F4_GET_ADCx_FROM_NUMBER(i);
        if (LL_ADC_IsActiveFlag_EOCS(adcx)) {
            // if the current IRQ has no ISR registered,
            // it means the IRQ happens from a non-blocking read
            if (!isr_registered[i]) {
                if (adc_data[i].status == RTEMS_ADC_NOT_READY) {
                    LL_ADC_DisableIT_EOCS(adcx);
                    adc_data[i].adc_value = LL_ADC_REG_ReadConversionData32(adcx);
                    adc_data[i].status = RTEMS_ADC_READY;
                }
            } 
            // if there is an ISR registered, call it
            else {
                LL_ADC_ClearFlag_EOCS(adcx);
                stm32f4_interrupt_arg *stm32_arg = (stm32f4_interrupt_arg *) arg;
                (*isr_table[i].isr)(stm32_arg->arg);
            }
            break;
        }
    }
}

#endif /* LIBBSP_ARM_STM32F4_BSP_ADC */
