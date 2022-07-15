/* SPDX-License-Identifier: BSD-2-Clause */

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

rtems_status_code rtems_adc_read_raw(
    rtems_gpio *base,
    uint32_t *result
)
{
    return base->adc_handlers->read_raw(base, result, RTEMS_ADC_NO_TIMEOUT);
}
rtems_status_code rtems_adc_read_raw_timeout(
    rtems_gpio *base,
    int32_t *result,
    uint32_t timeout
)
{
    return base->adc_handlers->read_raw(base, result, timeout);
}
rtems_status_code rtems_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
)
{
    return base->adc_handlers->read_raw_nb(base, result);
}

rtems_status_code rtems_adc_register_tf(
    uint32_t index,
    rtems_adc_tf tf,
    void *params
)
{
    return RTEMS_NOT_IMPLEMENTED;
}
rtems_status_code rtems_adc_remove_tf(
    uint32_t index
)
{
    return RTEMS_NOT_IMPLEMENTED;
}
rtems_status_code rtems_adc_read(
    rtems_gpio *base,
    uint32_t tf_index,
    double *result
)
{
    return RTEMS_NOT_IMPLEMENTED;
}
rtems_status_code rtems_adc_read_timeout(
    rtems_gpio *base,
    uint32_t tf_index,
    double *result,
    uint32_t timeout
)
{
    return RTEMS_NOT_IMPLEMENTED;
}
rtems_status_code rtems_adc_read_nb(
    rtems_gpio *base,
    uint32_t tf_index,
    double *result
)
{
    return RTEMS_NOT_IMPLEMENTED;
}
rtems_adc_status rtems_adc_is_ready(
    rtems_gpio *base
)
{
    return base->adc_handlers->is_ready(base);
}

rtems_status_code rtems_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    return base->adc_handlers->set_resolution(base, bits);
}
rtems_status_code rtems_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    return base->adc_handlers->set_alignment(base, align);
}

rtems_status_code rtems_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
)
{
    return base->adc_handlers->configure_interrupt(base, isr, arg);
}
rtems_status_code rtems_adc_enable_interrupt(
    rtems_gpio *base
)
{
    return base->adc_handlers->enable_interrupt(base);
}
rtems_status_code rtems_adc_disable_interrupt(
    rtems_gpio *base
)
{
    return base->adc_handlers->disable_interrupt(base);
}

