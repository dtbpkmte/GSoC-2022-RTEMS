/* SPDX-License-Identifier: BSD-2-Clause */

/**
  * @file
  *
  * @ingroup adc
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

#include <bsp/adc.h>
#include <stdbool.h>
#include <rtems/sysinit.h>

/**
  * @brief Private struct to store get() and destroy() handlers of drivers.
  */
struct rtems_adc_ctrl {
    rtems_status_code (*get) (uint32_t, rtems_adc **);
    rtems_status_code (*destroy) (rtems_adc *);
};

static struct rtems_adc_ctrl get_destroy_table[CONFIGURE_ADC_MAXIMUM_CONTROLLERS];

static uint32_t num_ctrl = 0;

rtems_status_code rtems_adc_register(
    rtems_status_code (*get_adc) (uint32_t, rtems_adc **),
    rtems_status_code (*destroy_adc) (rtems_adc *)
)
{
    rtems_interrupt_level level;

    if (num_ctrl == CONFIGURE_GPIO_MAXIMUM_CONTROLLERS)
        return RTEMS_TOO_MANY;

    rtems_interrupt_disable(level);

    get_destroy_table[num_ctrl] = (struct rtems_adc_ctrl) { 
        get_adc, destroy_adc 
    };
    ++num_ctrl;

    rtems_interrupt_enable(level);

    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_adc_get(
    uint32_t id,
    rtems_adc **out
)
{
    if (id >= num_ctrl)
        return RTEMS_UNSATISFIED;
    rtems_status_code sc = (*get_destroy_table[id].get)(id, out);
    (*out)->tf = (*out)->tf_params = NULL;
    return sc;
}

rtems_status_code rtems_adc_destroy(
    rtems_adc *adc
)
{
    return (*get_destroy_table[adc->id].destroy)(adc);
}

void rtems_adc_init(
    rtems_adc *base
)
{
    base->handlers->init(base);
}

rtems_status_code rtems_adc_read_raw(
    rtems_adc *base,
    uint32_t *result
)
{
    return base->handlers->read_raw(base, result, RTEMS_ADC_NO_TIMEOUT);
}

rtems_status_code rtems_adc_read_raw_timeout(
    rtems_adc *base,
    uint32_t *result,
    uint32_t timeout
)
{
    return base->handlers->read_raw(base, result, timeout);
}

rtems_adc_status rtems_adc_read_raw_nb(
    rtems_adc *base,
    uint32_t *result
)
{
    
    return base->handlers->read_raw_nb(base, result);
}

rtems_status_code rtems_adc_assign_tf(
    rtems_adc *base,
    rtems_adc_tf tf,
    void *params
)
{
    base->tf = tf;
    base->tf_params = params;
    return RTEMS_SUCCESSFUL;
}
rtems_status_code rtems_adc_remove_tf(
    rtems_adc *base
)
{
    
    base->tf = NULL;
    base->tf_params = NULL;
    return RTEMS_SUCCESSFUL;
}
rtems_status_code rtems_adc_read(
    rtems_adc *base,
    double *result
)
{
    
    uint32_t raw;
    rtems_status_code sc = rtems_adc_read_raw(base, &raw);
    if (sc == RTEMS_SUCCESSFUL) {
        if (base->tf == NULL)
            *result = (double) raw;
        else
            *result = base->tf(base->tf_params, raw);
    }
    return sc;
}
rtems_status_code rtems_adc_read_timeout(
    rtems_adc *base,
    double *result,
    uint32_t timeout
)
{
    
    uint32_t raw;
    rtems_status_code sc = rtems_adc_read_raw_timeout(base, &raw, timeout);
    if (sc == RTEMS_SUCCESSFUL) {
        if (base->tf == NULL)
            *result = (double) raw;
        else
            *result = base->tf(base->tf_params, raw);
    }
    return sc;
}

rtems_status_code rtems_adc_start_read_nb(
    rtems_adc *base
)
{
    
    return base->handlers->start_read_raw_nb(base);
}
rtems_adc_status rtems_adc_read_nb(
    rtems_adc *base,
    double *result
)
{
    
    uint32_t raw;
    rtems_adc_status sc = rtems_adc_read_raw_nb(base, &raw);
    if (sc == RTEMS_ADC_READY) {
        if (base->tf == NULL)
            *result = (double) raw;
        else
            *result = base->tf(base->tf_params, raw);
    }
    return sc;
}

rtems_status_code rtems_adc_set_channel(
    rtems_adc *base,
    uint32_t channel
)
{
    rtems_status_code sc = base->handlers->set_channel(base, channel);
    if (sc == RTEMS_SUCCESSFUL)
        base->config.channel = channel;
    return sc;
}

rtems_status_code rtems_adc_set_resolution(
    rtems_adc *base,
    unsigned int bits
)
{
    
    rtems_status_code sc = base->handlers->set_resolution(base, bits);
    if (sc == RTEMS_SUCCESSFUL)
        base->config.resolution = bits;
    return sc;
}
rtems_status_code rtems_adc_set_alignment(
    rtems_adc *base,
    rtems_adc_align align
)
{
    
    rtems_status_code sc = base->handlers->set_alignment(base, align);
    if (sc == RTEMS_SUCCESSFUL)
        base->config.alignment = align;
    return sc;
}

/**
  * Force linking driver ADC source
  */
static void rtems_adc_start(
    void
)
{
    *adc_dummy = 1;
}
RTEMS_SYSINIT_ITEM(
    rtems_adc_start,
    RTEMS_SYSINIT_BSP_START,
    RTEMS_SYSINIT_ORDER_LAST
);

