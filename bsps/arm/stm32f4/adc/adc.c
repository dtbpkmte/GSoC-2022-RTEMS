#include <bsp/stm32f_adc.h>

static void stm32f4_adc_select_channel(
    stm32f4_gpio *gpio
);

/***************/
ADC_TypeDef *stm32f4_get_ADCx(
    GPIOTypeDef *gpiox
)
{
    switch ((uintptr_t) gpio->port) {
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

rtems_status_code get_LL_ADC_CHANNEL(
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
            case 1:
                *channel = LL_ADC_CHANNEL_1;
            case 2:
                *channel = LL_ADC_CHANNEL_2;
            case 3:
                *channel = LL_ADC_CHANNEL_3;
            case 4:
                *channel = LL_ADC_CHANNEL_4;
            case 5:
                *channel = LL_ADC_CHANNEL_5;
            case 6:
                *channel = LL_ADC_CHANNEL_6;
            case 7:
                *channel = LL_ADC_CHANNEL_7;
            default:
                return RTEMS_NOT_SATISFIED;
        }
    }
    else if (port == (uintptr_t) GPIOB) {
        switch (gpio->pin) {
            case 0:
                *channel = LL_ADC_CHANNEL_8;
            case 1:
                *channel = LL_ADC_CHANNEL_9;
            default:
                return RTEMS_NOT_SATISFIED;
        }
    }
    else if (port == (uintptr_t) GPIOC) {
        switch (gpio->pin) {
            case 0:
                *channel = LL_ADC_CHANNEL_10;
            case 1:
                *channel = LL_ADC_CHANNEL_11;
            case 2:
                *channel = LL_ADC_CHANNEL_12;
            case 3:
                *channel = LL_ADC_CHANNEL_13;
            case 4:
                *channel = LL_ADC_CHANNEL_14;
            case 5:
                *channel = LL_ADC_CHANNEL_15;
            default:
                return RTEMS_NOT_SATISFIED;
        }
    }
    else if (port == (uintptr_t) GPIOF) {
        switch (gpio->pin) {
            case 3:
                *channel = LL_ADC_CHANNEL_9;
            case 4:
                *channel = LL_ADC_CHANNEL_14;
            case 5:
                *channel = LL_ADC_CHANNEL_15;
            case 6:
                *channel = LL_ADC_CHANNEL_4;
            case 7:
                *channel = LL_ADC_CHANNEL_5;
            case 8:
                *channel = LL_ADC_CHANNEL_6;
            case 9:
                *channel = LL_ADC_CHANNEL_7;
            case 10:
                *channel = LL_ADC_CHANNEL_8;
            default:
                return RTEMS_NOT_SATISFIED;
        }
    }
    else
        return RTEMS_NOT_SATISFIED;
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
    .read_raw_nb = stm32f4_adc_read_raw_nb,
    .is_ready = stm32f4_adc_is_ready,
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

rtems_status_code stm32f4_adc_init(
    stm32f4_gpio *gpio
)
{
    if (!LL_ADC_IsEnabled(gpio->ADCx)) {
        // Enable clock
        switch ((uintptr_t) gpio->ADCx) {
            case (uintptr_t) ADC1:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
            case (uintptr_t) ADC2:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
            case (uintptr_t) ADC3:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
            default:
                return RTEMS_UNSATISFIED;
        }

        // ADC common setup
        // Default resolution: 10-bit
        ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_10B;
        // Default alignment: right
        ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
        ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
        LL_ADC_Init(gpio->ADCx, &ADC_InitStruct);
        ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
        ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
        ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
        ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
        ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
        LL_ADC_REG_Init(gpio->ADCx, &ADC_REG_InitStruct);
        LL_ADC_REG_SetFlagEndOfConversion(gpio->ADCx, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
        ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
        ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
        LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(gpio->ADCx), &ADC_CommonInitStruct);
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
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    stm32f4_adc_select_channel(gpio);
    LL_ADC_REG_StartConversionSWStart(gpio->ADCx);
    while (!LL_ADC_IsActiveFlag_EOCS(gpio->ADCx))
    *result = LL_ADC_REG_ReadConversionData32(gpio->ADCx);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_adc_status stm32f4_adc_is_ready(
    rtems_gpio *base
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_adc_remove_interrupt(
    rtems_gpio *base
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_adc_enable_interrupt(
    rtems_gpio *base
)
{
    return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code stm32f4_adc_disable_interrupt(
    rtems_gpio *base
)
{
    return RTEMS_NOT_IMPLEMENTED;
}
