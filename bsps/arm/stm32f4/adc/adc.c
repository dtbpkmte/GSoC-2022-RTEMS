/* SPDX-License-Identifier: BSD-2-Clause */

/**
  * @file
  *
  * @ingroup stm32f4_adc
  */

/*
 * Copyright (C) 2022 Duc Doan (dtbpkmte at gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bsp/stm32f4_adc.h>
#include <bsp/stm32f4_gpio.h>
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
    rtems_gpio *base
);

/***************/
#ifdef ADC1
static ADC_TypeDef_Protected _ADC1_protected = { ADC1, false };
ADC_TypeDef_Protected *const ADC1_protected = &_ADC1_protected;
#endif
#ifdef ADC2
static ADC_TypeDef_Protected _ADC2_protected = { ADC2, false };
ADC_TypeDef_Protected *const ADC2_protected = &_ADC2_protected;
#endif
#ifdef ADC3
static ADC_TypeDef_Protected _ADC3_protected = { ADC3, false };
ADC_TypeDef_Protected *const ADC3_protected = &_ADC3_protected;
#endif

/* Helpers */
#define STM32F4_GET_ADC_NUMBER(ADCx) \
    ( (uintptr_t) ( ADCx ) == (uintptr_t) ADC1 ? 1 : \
      (uintptr_t) ( ADCx ) == (uintptr_t) ADC2 ? 2 : \
      (uintptr_t) ( ADCx ) == (uintptr_t) ADC3 ? 3 : \
                                                 0 )
#define STM32F4_GET_ADCx_FROM_NUMBER(num)       (\
    ( num ) == 1                 ? ADC1 :        \
    ( num ) == 2 && NUM_ADC >= 2 ? ADC2 :        \
    ( num ) == 3 && NUM_ADC == 3 ? ADC3 :        \
                                   NULL)
#define STM32F4_GET_ADCx_PROTECTED_FROM_NUMBER(num)     (\
    ( num ) == 1                 ? ADC1_protected :      \
    ( num ) == 2 && NUM_ADC >= 2 ? ADC2_protected :      \
    ( num ) == 3 && NUM_ADC == 3 ? ADC3_protected :      \
                                   NULL)

// TODO: other variants
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

// TODO: other variants
ADC_TypeDef_Protected *stm32f4_get_ADCx_protected(
    GPIO_TypeDef *gpio
)
{
    switch ((uintptr_t) gpio) {
#if defined(STM32F405xx) || defined(STM32F407xx)
        case (uintptr_t) GPIOA:
        case (uintptr_t) GPIOB:
        case (uintptr_t) GPIOC:
            return ADC1_protected;
        case (uintptr_t) GPIOF:
            return ADC3_protected;
#endif /* defined(STM32F405xx) || defined(STM32F407xx) */
        default:
            return NULL;
    }
}

// TODO: other variants
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

static const rtems_adc_api stm32f4_adc_base_api = 
    RTEMS_ADC_BUILD_API(
        stm32f4_adc_init,
        stm32f4_adc_read_raw,
        stm32f4_adc_start_read_raw_nb,
        stm32f4_adc_read_raw_nb,
        stm32f4_adc_set_resolution,
        stm32f4_adc_set_alignment,
        stm32f4_adc_configure_interrupt,
        stm32f4_adc_remove_interrupt,
        stm32f4_adc_enable_interrupt,
        stm32f4_adc_disable_interrupt
    );

rtems_periph_api *stm32f4_adc_get(
    rtems_gpio *pin
)
{
    stm32f4_gpio *gpio = stm32f4_get_gpio_from_base(pin);
    uint32_t channel;
    // if the pin does not have ADC functionality
    if (stm32f4_get_LL_ADC_CHANNEL(gpio, &channel) != RTEMS_SUCCESSFUL) {
        return NULL;
    }

    // First allocate space for stm32f4_adc object
    stm32f4_adc *api = malloc(sizeof(stm32f4_adc));
    if (api == NULL) {
        return NULL;
    }
    api->base_api = stm32f4_adc_base_api;

    // Then allocate space for stm32f4_adc_config object
    api->adc_config = malloc(sizeof(stm32f4_adc_config));
    if (api->adc_config == NULL) {
        return NULL;
    }
    api->adc_config->ADCx = stm32f4_get_ADCx_protected(gpio->port);
    api->adc_config->channel = channel;
    api->adc_config->resolution = STM32F4_ADC_DEFAULT_RESOLUTION;
    api->adc_config->alignment = STM32F4_ADC_DEFAULT_ALIGNMENT;
    api->adc_config->locked = false;
    return (rtems_periph_api *) api;
}

rtems_status_code stm32f4_adc_destroy(
    rtems_gpio *pin
)
{
    if (pin->api != NULL) {
        free(((stm32f4_adc *) pin->api)->adc_config);
        free(pin->api);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

static rtems_status_code stm32f4_adc_select_channel(
    rtems_gpio *base
)
{
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    if (!STM32F4_IS_LOCKED(adc_config) &&
            !STM32F4_IS_LOCKED(adc_config->ADCx)) {
        STM32F4_LOCK(adc_config);
        STM32F4_LOCK(adc_config->ADCx);

        LL_ADC_Disable(adc_config->ADCx->ADCx);

        LL_ADC_SetResolution(adc_config->ADCx->ADCx, adc_config->resolution);
        LL_ADC_SetDataAlignment(adc_config->ADCx->ADCx, adc_config->alignment);
        LL_ADC_REG_SetSequencerRanks(
            adc_config->ADCx->ADCx, 
            LL_ADC_REG_RANK_1, 
            adc_config->channel
        );
        LL_ADC_SetChannelSamplingTime(
            adc_config->ADCx->ADCx, 
            adc_config->channel, 
            STM32F4_ADC_DEFAULT_SAMPLINGTIME
        );

        LL_ADC_Enable(adc_config->ADCx->ADCx);
        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        volatile uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
        while(counter != 0U)
        {
          counter--;
        }

        STM32F4_UNLOCK(adc_config);
        STM32F4_UNLOCK(adc_config->ADCx);
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

void stm32f4_adc_init(
    rtems_gpio *base
)
{
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    if (!LL_ADC_IsEnabled(adc_config->ADCx->ADCx)) {
        // Enable clock
        switch ((uintptr_t) adc_config->ADCx->ADCx) {
#ifdef ADC1
            case (uintptr_t) ADC1:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
                break;
#endif
#ifdef ADC2
            case (uintptr_t) ADC2:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
                break;
#endif
#ifdef ADC3
            case (uintptr_t) ADC3:
                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
                break;
#endif
            default:
                return;
        }

        // ADC common setup
        LL_ADC_SetSequencersScanMode(adc_config->ADCx->ADCx, LL_ADC_SEQ_SCAN_DISABLE);
        LL_ADC_REG_SetTriggerSource(adc_config->ADCx->ADCx, LL_ADC_REG_TRIG_SOFTWARE);
        LL_ADC_REG_SetSequencerLength(adc_config->ADCx->ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);
        LL_ADC_REG_SetSequencerDiscont(adc_config->ADCx->ADCx, LL_ADC_REG_SEQ_DISCONT_DISABLE);
        LL_ADC_REG_SetContinuousMode(adc_config->ADCx->ADCx, LL_ADC_REG_CONV_SINGLE);
        LL_ADC_REG_SetDMATransfer(adc_config->ADCx->ADCx, LL_ADC_REG_DMA_TRANSFER_NONE);

        LL_ADC_REG_SetFlagEndOfConversion(adc_config->ADCx->ADCx, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);

        LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc_config->ADCx->ADCx), LL_ADC_CLOCK_SYNC_PCLK_DIV2);
        LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(adc_config->ADCx->ADCx), LL_ADC_MULTI_INDEPENDENT);

        LL_ADC_Enable(adc_config->ADCx->ADCx);
        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        volatile uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
        while(counter != 0U)
        {
            counter--;
        }
    }
}

rtems_status_code stm32f4_adc_read_raw(
    rtems_gpio *base,
    uint32_t *result,
    uint32_t timeout
)
{
    uint32_t tickstart = 0U;
    rtems_status_code sc = stm32f4_adc_select_channel(base);
    if (sc != RTEMS_SUCCESSFUL) {
        return sc;
    }
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    STM32F4_LOCK(adc_config->ADCx);
    STM32F4_LOCK(adc_config);
    LL_ADC_REG_StartConversionSWStart(adc_config->ADCx->ADCx);
    while (!LL_ADC_IsActiveFlag_EOCS(adc_config->ADCx->ADCx)) {
        if (timeout != RTEMS_ADC_NO_TIMEOUT) {
            if (timeout == 0U || ((HAL_GetTick() - tickstart) > timeout)) {
                if (!LL_ADC_IsActiveFlag_EOCS(adc_config->ADCx->ADCx)) {
                    return RTEMS_TIMEOUT;
                }
            }
        }
    }
    *result = LL_ADC_REG_ReadConversionData32(adc_config->ADCx->ADCx);
    STM32F4_UNLOCK(adc_config->ADCx);
    STM32F4_UNLOCK(adc_config);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_start_read_raw_nb(
    rtems_gpio *base
)
{
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(adc_config->ADCx->ADCx) - 1;
    if (adc_data[adc_idx].status == RTEMS_ADC_NOT_STARTED) {
        // start conversion here
        rtems_status_code sc = stm32f4_adc_select_channel(base);
        if (sc != RTEMS_SUCCESSFUL) {
            return sc;
        }
        STM32F4_LOCK(adc_config);
        STM32F4_LOCK(adc_config->ADCx);
        LL_ADC_EnableIT_EOCS(adc_config->ADCx->ADCx);
        LL_ADC_REG_StartConversionSWStart(adc_config->ADCx->ADCx);
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
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(adc_config->ADCx->ADCx) - 1;
    rtems_adc_status ret = adc_data[adc_idx].status;
    if (ret == RTEMS_ADC_READY) {
        *result = adc_data[adc_idx].adc_value;
        adc_data[adc_idx].status = RTEMS_ADC_NOT_STARTED;
        
        STM32F4_UNLOCK(adc_config->ADCx);
        STM32F4_UNLOCK(adc_config);
    }
    return ret;
}

rtems_status_code stm32f4_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    if (!STM32F4_IS_LOCKED(adc_config)) {
        STM32F4_LOCK(adc_config);
        switch (bits) {
            case 12:
                adc_config->resolution = LL_ADC_RESOLUTION_12B;
                break;
            case 10:
                adc_config->resolution = LL_ADC_RESOLUTION_10B;
                break;
            case 8:
                adc_config->resolution = LL_ADC_RESOLUTION_8B;
                break;
            case 6:
                adc_config->resolution = LL_ADC_RESOLUTION_6B;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
        STM32F4_UNLOCK(adc_config);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_RESOURCE_IN_USE;
}

rtems_status_code stm32f4_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    if (!STM32F4_IS_LOCKED(adc_config)) {
        STM32F4_LOCK(adc_config);
        switch (align) {
            case RTEMS_ADC_ALIGN_LEFT:
                adc_config->alignment = LL_ADC_DATA_ALIGN_LEFT;
                break;
            case RTEMS_ADC_ALIGN_RIGHT:
                adc_config->alignment = LL_ADC_DATA_ALIGN_RIGHT;
                break;
            default:
                return RTEMS_UNSATISFIED;
        }
        STM32F4_UNLOCK(adc_config);
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
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(adc_config->ADCx->ADCx) - 1;
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
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    unsigned int adc_idx = STM32F4_GET_ADC_NUMBER(adc_config->ADCx->ADCx) - 1;
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
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    if (!STM32F4_IS_LOCKED(adc_config->ADCx)) {
        if (isr_registered[STM32F4_GET_ADC_NUMBER(adc_config->ADCx->ADCx) - 1]) {
            LL_ADC_EnableIT_EOCS(adc_config->ADCx->ADCx);
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
    stm32f4_adc_config *adc_config = ((stm32f4_adc *) base->api)->adc_config;
    if (!STM32F4_IS_LOCKED(adc_config->ADCx)) {
        if (isr_registered[STM32F4_GET_ADC_NUMBER(adc_config->ADCx->ADCx) - 1]) {
            LL_ADC_DisableIT_EOCS(adc_config->ADCx->ADCx);
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

