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
#include <stdlib.h>

#if defined(ADC3)
#define NUM_ADC 3
#elif defined(ADC2)
#define NUM_ADC 2
#else
#define NUM_ADC 1
#endif

/************** Helpers *****************/
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
#define stm32f4_adc_get_adc_from_base(_base) \
    RTEMS_CONTAINER_OF(_base, stm32f4_adc, base)

void adc_irq_handler(void *arg);

/**
  * @brief Data structure for non-blocking read.
  */
typedef struct {
    uint32_t adc_value;
    rtems_adc_status status;
} stm32f4_adc_data;
static stm32f4_adc_data adc_data[NUM_ADC] = {0};

/**
  * @brief Configure the ADC channel to be the specified one.
  * An ADC contains multiple channels, so before each operation, we should
  * select the wanted channel.
  */
static rtems_status_code stm32f4_adc_select_channel(
    rtems_adc *base
);

/***************/
/**
  * ADC objects that have simple lock for mutex.
  */
#ifdef ADC1
static ADC_TypeDef_Protected _ADC1_protected = { ADC1, false };
ADC_TypeDef_Protected *const ADC1_protected = &_ADC1_protected;

static rtems_status_code stm32f4_adc_get_adc1(uint32_t id, rtems_adc **out);
#endif
#ifdef ADC2
static ADC_TypeDef_Protected _ADC2_protected = { ADC2, false };
ADC_TypeDef_Protected *const ADC2_protected = &_ADC2_protected;

static rtems_status_code stm32f4_adc_get_adc2(uint32_t id, rtems_adc **out);
#endif
#ifdef ADC3
static ADC_TypeDef_Protected _ADC3_protected = { ADC3, false };
ADC_TypeDef_Protected *const ADC3_protected = &_ADC3_protected;

static rtems_status_code stm32f4_adc_get_adc3(uint32_t id, rtems_adc **out);
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
#define STM32F4_GET_LL_ADC_CHANNEL(num)        (\
    ( num ) == 0 ? LL_ADC_CHANNEL_0 :           \
    ( num ) == 1 ? LL_ADC_CHANNEL_1 :           \
    ( num ) == 2 ? LL_ADC_CHANNEL_2 :           \
    ( num ) == 3 ? LL_ADC_CHANNEL_3 :           \
    ( num ) == 4 ? LL_ADC_CHANNEL_4 :           \
    ( num ) == 5 ? LL_ADC_CHANNEL_5 :           \
    ( num ) == 6 ? LL_ADC_CHANNEL_6 :           \
    ( num ) == 7 ? LL_ADC_CHANNEL_7 :           \
    ( num ) == 8 ? LL_ADC_CHANNEL_8 :           \
    ( num ) == 9 ? LL_ADC_CHANNEL_9 :           \
    ( num ) == 10 ? LL_ADC_CHANNEL_10 :         \
    ( num ) == 11 ? LL_ADC_CHANNEL_11 :         \
    ( num ) == 12 ? LL_ADC_CHANNEL_12 :         \
    ( num ) == 13 ? LL_ADC_CHANNEL_13 :         \
    ( num ) == 14 ? LL_ADC_CHANNEL_14 :         \
    ( num ) == 15 ? LL_ADC_CHANNEL_15 :         \
                    0xffffffff)

static const rtems_adc_handlers stm32f4_adc_handlers = {
    stm32f4_adc_init,
    stm32f4_adc_read_raw,
    stm32f4_adc_start_read_raw_nb,
    stm32f4_adc_read_raw_nb,
    stm32f4_adc_set_channel,
    stm32f4_adc_set_resolution,
    stm32f4_adc_set_alignment
};

#ifdef ADC1
static rtems_status_code stm32f4_adc_get_adc1(
    uint32_t id, 
    rtems_adc **out
)
{
    return stm32f4_adc_get(1, id, out);
}
#endif /* ADC1 */
#ifdef ADC2
static rtems_status_code stm32f4_adc_get_adc2(
    uint32_t id, 
    rtems_adc **out
)
{
    return stm32f4_adc_get(2, id, out);
}
#endif /* ADC2 */
#ifdef ADC3
static rtems_status_code stm32f4_adc_get_adc3(
    uint32_t id, 
    rtems_adc **out
)
{
    return stm32f4_adc_get(3, id, out);
}
#endif /* ADC3 */

rtems_status_code stm32f4_adc_get(
    uint32_t adc_num,
    uint32_t id,
    rtems_adc **out
)
{
    // First allocate space for stm32f4_adc object
    stm32f4_adc *adc = malloc(sizeof(stm32f4_adc));
    if (adc == NULL) {
        return RTEMS_NO_MEMORY;
    }

    *(uint32_t *) &adc->base.id = id;
    *(const rtems_adc_handlers **) &adc->base.handlers = &stm32f4_adc_handlers;

    adc->adc_config.ADCx = STM32F4_GET_ADCx_PROTECTED_FROM_NUMBER(adc_num);
    adc->adc_config.resolution = STM32F4_ADC_DEFAULT_RESOLUTION;
    adc->adc_config.alignment = STM32F4_ADC_DEFAULT_ALIGNMENT;
    adc->adc_config.locked = false;
    *out = (rtems_adc *) adc;
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_destroy(
    rtems_adc *pin
)
{
    if (pin != NULL) {
        free(pin);
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_UNSATISFIED;
}

static rtems_status_code stm32f4_adc_select_channel(
    rtems_adc *base
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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
        volatile uint32_t counter = 
            (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
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

static void stm32f4_adc_start(
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

    // Registers with ADC API
#ifdef ADC1
    rtems_adc_register(stm32f4_adc_get_adc1, stm32f4_adc_destroy);
#endif /* ADC1 */
#ifdef ADC2
    rtems_adc_register(stm32f4_adc_get_adc2, stm32f4_adc_destroy);
#endif /* ADC2 */
#ifdef ADC3
    rtems_adc_register(stm32f4_adc_get_adc3, stm32f4_adc_destroy);
#endif /* ADC3 */
}
RTEMS_SYSINIT_ITEM(
    stm32f4_adc_start,
    RTEMS_SYSINIT_BSP_PRE_DRIVERS,
    RTEMS_SYSINIT_ORDER_LAST
);

void stm32f4_adc_init(
    rtems_adc *base
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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
    rtems_adc *base,
    uint32_t *result,
    uint32_t timeout
)
{
    uint32_t tickstart = 0U;
    rtems_status_code sc = stm32f4_adc_select_channel(base);
    if (sc != RTEMS_SUCCESSFUL) {
        return sc;
    }
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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
    rtems_adc *base
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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
    rtems_adc *base,
    uint32_t *result
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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

rtems_status_code stm32f4_adc_set_channel(
    rtems_adc *base,
    uint32_t channel
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    adc->adc_config.channel = STM32F4_GET_LL_ADC_CHANNEL(channel);
    if (adc->adc_config.channel == 0xffffffff)
        return RTEMS_UNSATISFIED;
    return RTEMS_SUCCESSFUL;
}

rtems_status_code stm32f4_adc_set_resolution(
    rtems_adc *base,
    unsigned int bits
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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
    rtems_adc *base,
    rtems_adc_align align
)
{
    stm32f4_adc *adc = stm32f4_adc_get_adc_from_base(base);
    stm32f4_adc_config *adc_config = &adc->adc_config;
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

void adc_irq_handler(void *arg) {
    rtems_interrupt_level level;
    rtems_interrupt_disable( level );
    unsigned int i;
    for (i = 0; i < NUM_ADC; ++i) {
        ADC_TypeDef *adcx = STM32F4_GET_ADCx_FROM_NUMBER(i+1);
        if (LL_ADC_IsActiveFlag_EOCS(adcx)) {
            if (adc_data[i].status == RTEMS_ADC_NOT_READY) {
                LL_ADC_DisableIT_EOCS(adcx);
                adc_data[i].adc_value = LL_ADC_REG_ReadConversionData32(adcx);
                adc_data[i].status = RTEMS_ADC_READY;
            }
            break;
        }
    }
    rtems_interrupt_enable( level );
}

RTEMS_ADC_LINK();
