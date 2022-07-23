#include <bsp/stm32f4_adc.h>
#include <stdlib.h>

#if defined(ADC3)
#define NUM_ADC 3
#elif defined(ADC2)
#define NUM_ADC 2
#else
#define NUM_ADC 1
#endif

/************** Interrupt manager *****************/
typedef struct {
    void *arg;
    stm32f4_gpio *gpio;
} stm32f4_interrupt_arg;

typedef struct {
    stm32f4_interrupt_arg arg;
    rtems_gpio_isr isr;
} stm32f4_interrupt;

void adc_irq_handler(void *arg);

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

static rtems_status_code stm32f4_adc_select_channel(
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

const rtems_adc_handlers *stm32f4_get_adc_handlers(
    void
)
{
    return &stm32f4_adc_handlers;
}

rtems_status_code stm32f4_adc_get(
    stm32f4_gpio *gpio
)
{
    // populate ADC config object
    gpio->adc_config = malloc(sizeof(stm32f4_adc_config));
    if (gpio->adc_config == NULL) {
        return RTEMS_NO_MEMORY;
    }
    STM32F4_LOCK(gpio->adc_config);
    STM32F4_LOCK(&gpio->adc_config->ADCx);
    gpio->adc_config->ADCx.ADCx = stm32f4_get_ADCx(gpio->port);
    stm32f4_get_LL_ADC_CHANNEL(gpio, &gpio->adc_config->channel);
    gpio->adc_config->resolution = STM32F4_ADC_DEFAULT_RESOLUTION;
    gpio->adc_config->alignment = STM32F4_ADC_DEFAULT_ALIGNMENT;
    STM32F4_UNLOCK(&gpio->adc_config->ADCx);
    STM32F4_UNLOCK(gpio->adc_config);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_destroy(
    stm32f4_gpio *gpio
)
{
    if (gpio->adc_config != NULL) {
        free(gpio->adc_config);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

static rtems_status_code stm32f4_adc_select_channel(
    stm32f4_gpio *gpio
)
{
    if (!STM32F4_IS_LOCKED(gpio->adc_config) &&
            !STM32F4_IS_LOCKED(&gpio->adc_config->ADCx)) {
        STM32F4_LOCK(gpio->adc_config);
        STM32F4_LOCK(&gpio->adc_config->ADCx);

        LL_ADC_Disable(gpio->adc_config->ADCx.ADCx);

        LL_ADC_SetResolution(gpio->adc_config->ADCx.ADCx, gpio->adc_config->resolution);
        LL_ADC_SetDataAlignment(gpio->adc_config->ADCx.ADCx, gpio->adc_config->alignment);
        LL_ADC_REG_SetSequencerRanks(
            gpio->adc_config->ADCx.ADCx, 
            LL_ADC_REG_RANK_1, 
            gpio->adc_config->channel
        );
        LL_ADC_SetChannelSamplingTime(
            gpio->adc_config->ADCx.ADCx, 
            gpio->adc_config->channel, 
            STM32F4_ADC_DEFAULT_SAMPLINGTIME
        );

        LL_ADC_Enable(gpio->adc_config->ADCx.ADCx);
        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        volatile uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
        while(counter != 0U)
        {
          counter--;
        }

        STM32F4_UNLOCK(gpio->adc_config);
        STM32F4_UNLOCK(&gpio->adc_config->ADCx);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

void stm32f4_adc_start(
    void
)
{
    // Install ISR for non-blocking read
    rtems_status_code sc = rtems_interrupt_handler_install(
            ADC_IRQn,
            NULL,
            RTEMS_INTERRUPT_SHARED,
            adc_irq_handler,
            NULL
    );
    while (sc != RTEMS_SUCCESSFUL);
}
RTEMS_SYSINIT_ITEM(
    stm32f4_adc_start,
    RTEMS_SYSINIT_DEVICE_DRIVERS,
    RTEMS_SYSINIT_ORDER_LAST
);

rtems_status_code stm32f4_adc_init(
    stm32f4_gpio *gpio
)
{
    if (!LL_ADC_IsEnabled(gpio->adc_config->ADCx.ADCx)) {
        // Enable clock
        switch ((uintptr_t) gpio->adc_config->ADCx.ADCx) {
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
        LL_ADC_SetSequencersScanMode(gpio->adc_config->ADCx.ADCx, LL_ADC_SEQ_SCAN_DISABLE);
        LL_ADC_REG_SetTriggerSource(gpio->adc_config->ADCx.ADCx, LL_ADC_REG_TRIG_SOFTWARE);
        LL_ADC_REG_SetSequencerLength(gpio->adc_config->ADCx.ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);
        LL_ADC_REG_SetSequencerDiscont(gpio->adc_config->ADCx.ADCx, LL_ADC_REG_SEQ_DISCONT_DISABLE);
        LL_ADC_REG_SetContinuousMode(gpio->adc_config->ADCx.ADCx, LL_ADC_REG_CONV_SINGLE);
        LL_ADC_REG_SetDMATransfer(gpio->adc_config->ADCx.ADCx, LL_ADC_REG_DMA_TRANSFER_NONE);

        LL_ADC_REG_SetFlagEndOfConversion(gpio->adc_config->ADCx.ADCx, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);

        LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(gpio->adc_config->ADCx.ADCx), LL_ADC_CLOCK_SYNC_PCLK_DIV2);
        LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(gpio->adc_config->ADCx.ADCx), LL_ADC_MULTI_INDEPENDENT);

        LL_ADC_Enable(gpio->adc_config->ADCx.ADCx);
        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        volatile uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
        while(counter != 0U)
        {
            counter--;
        }
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
    rtems_status_code sc = stm32f4_adc_select_channel(gpio);
    if (sc != RTEMS_SUCCESSFUL) {
        return sc;
    }
    STM32F4_LOCK(&gpio->adc_config->ADCx);
    STM32F4_LOCK(gpio->adc_config);
    LL_ADC_REG_StartConversionSWStart(gpio->adc_config->ADCx.ADCx);
    while (!LL_ADC_IsActiveFlag_EOCS(gpio->adc_config->ADCx.ADCx)) {
        if (timeout != RTEMS_ADC_NO_TIMEOUT) {
            if (timeout == 0U || ((HAL_GetTick() - tickstart) > timeout)) {
                if (!LL_ADC_IsActiveFlag_EOCS(gpio->adc_config->ADCx.ADCx)) {
                    return RTEMS_TIMEOUT;
                }
            }
        }
    }
    *result = LL_ADC_REG_ReadConversionData32(gpio->adc_config->ADCx.ADCx);
    STM32F4_UNLOCK(&gpio->adc_config->ADCx);
    STM32F4_UNLOCK(gpio->adc_config);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_start_read_raw_nb(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(gpio->adc_config->ADCx.ADCx) - 1;
    if (adc_data[adc_idx].status == RTEMS_ADC_NOT_STARTED) {
        // start conversion here
        rtems_status_code sc = stm32f4_adc_select_channel(gpio);
        if (sc != RTEMS_SUCCESSFUL) {
            return sc;
        }
        STM32F4_LOCK(gpio->adc_config);
        STM32F4_LOCK(&gpio->adc_config->ADCx);
        LL_ADC_EnableIT_EOCS(gpio->adc_config->ADCx.ADCx);
        LL_ADC_REG_StartConversionSWStart(gpio->adc_config->ADCx.ADCx);
        adc_data[adc_idx].status = RTEMS_ADC_NOT_READY;

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
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(gpio->adc_config->ADCx.ADCx) - 1;
    rtems_adc_status ret = adc_data[adc_idx].status;
    if (ret == RTEMS_ADC_READY) {
        *result = adc_data[adc_idx].adc_value;
        adc_data[adc_idx].status = RTEMS_ADC_NOT_STARTED;
        
        STM32F4_UNLOCK(&gpio->adc_config->ADCx);
        STM32F4_UNLOCK(gpio->adc_config);
    }
    return ret;
}

rtems_status_code stm32f4_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    if (!STM32F4_IS_LOCKED(gpio->adc_config)) {
        STM32F4_LOCK(gpio->adc_config);
        switch (bits) {
            case 12:
                gpio->adc_config->resolution = LL_ADC_RESOLUTION_12B;
                break;
            case 10:
                gpio->adc_config->resolution = LL_ADC_RESOLUTION_10B;
                break;
            case 8:
                gpio->adc_config->resolution = LL_ADC_RESOLUTION_8B;
                break;
            case 6:
                gpio->adc_config->resolution = LL_ADC_RESOLUTION_6B;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
        STM32F4_UNLOCK(gpio->adc_config);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_RESOURCE_IN_USE;
}

rtems_status_code stm32f4_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    if (!STM32F4_IS_LOCKED(gpio->adc_config)) {
        STM32F4_LOCK(gpio->adc_config);
        switch (align) {
            case RTEMS_ADC_ALIGN_LEFT:
                gpio->adc_config->alignment = LL_ADC_DATA_ALIGN_LEFT;
                break;
            case RTEMS_ADC_ALIGN_RIGHT:
                gpio->adc_config->alignment = LL_ADC_DATA_ALIGN_RIGHT;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
        STM32F4_UNLOCK(gpio->adc_config);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_RESOURCE_IN_USE;
}

rtems_status_code stm32f4_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(gpio->adc_config->ADCx.ADCx) - 1;
    if (!isr_registered[adc_idx]) {
        isr_table[adc_idx] = (stm32f4_interrupt){
            .arg = {
                .arg = arg,
                .gpio = gpio
            },
            .isr = isr
        };
        isr_registered[adc_idx] = true;
        return rtems_interrupt_handler_install(
                ADC_IRQn,
                NULL,
                RTEMS_INTERRUPT_SHARED,
                adc_irq_handler,
                &isr_table[adc_idx].arg
        );
    }
    return RTEMS_UNSATISFIED;
}

rtems_status_code stm32f4_adc_remove_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(gpio->adc_config->ADCx.ADCx) - 1;
    rtems_status_code sc = rtems_interrupt_handler_remove(
            ADC_IRQn,
            adc_irq_handler,
            &isr_table[adc_idx].arg
    );
    if (sc == RTEMS_SUCCESSFUL) {
        isr_registered[adc_idx] = false;
    }
    return sc;
}

rtems_status_code stm32f4_adc_enable_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    if (!STM32F4_IS_LOCKED(&gpio->adc_config->ADCx)) {
        if (isr_registered[STM32F4_GET_ADC_NUMBER(gpio->adc_config->ADCx.ADCx) - 1]) {
            LL_ADC_EnableIT_EOCS(gpio->adc_config->ADCx.ADCx);
            return RTEMS_SUCCESSFUL;
        }
        return RTEMS_UNSATISFIED;
    }
    return RTEMS_RESOURCE_IN_USE;
}

rtems_status_code stm32f4_adc_disable_interrupt(
    rtems_gpio *base
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(base);
    if (!STM32F4_IS_LOCKED(&gpio->adc_config->ADCx)) {
        if (isr_registered[STM32F4_GET_ADC_NUMBER(gpio->adc_config->ADCx.ADCx) - 1]) {
            LL_ADC_DisableIT_EOCS(gpio->adc_config->ADCx.ADCx);
            return RTEMS_SUCCESSFUL;
        }
        return RTEMS_UNSATISFIED;
    }
    return RTEMS_RESOURCE_IN_USE;
}

void adc_irq_handler(void *arg) {
    rtems_interrupt_level level;
    rtems_interrupt_disable( level );
    unsigned int i;
    for (i = 0; i < NUM_ADC; ++i) {
        ADC_TypeDef *adcx = STM32F4_GET_ADCx_FROM_NUMBER(i+1);
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
    rtems_interrupt_enable( level );
}

