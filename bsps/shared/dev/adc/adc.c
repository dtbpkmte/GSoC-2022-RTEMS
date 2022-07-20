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
    if (base->is_adc_pin) {
        return base->adc_handlers->read_raw(base, result, RTEMS_ADC_NO_TIMEOUT);
    }
    return RTEMS_UNSATISFIED;
}
rtems_status_code rtems_adc_read_raw_timeout(
    rtems_gpio *base,
    uint32_t *result,
    uint32_t timeout
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->read_raw(base, result, timeout);
    }
    return RTEMS_UNSATISFIED;
}

rtems_adc_status rtems_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->read_raw_nb(base, result);
    }
    return RTEMS_UNSATISFIED;
}

rtems_status_code rtems_adc_assign_tf(
    rtems_gpio *base,
    rtems_adc_tf tf,
    void *params
)
{
    base->tf = tf;
    base->tf_params = params;
    return RTEMS_SUCCESSFUL;
}
rtems_status_code rtems_adc_remove_tf(
    rtems_gpio *base
)
{
    base->tf = NULL;
    base->tf_params = NULL;
    return RTEMS_SUCCESSFUL;
}
rtems_status_code rtems_adc_read(
    rtems_gpio *base,
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
    rtems_gpio *base,
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
    rtems_gpio *base
)
{
    if (base->is_adc_pin)
        return base->adc_handlers->start_read_raw_nb(base);
    return RTEMS_UNSATISFIED;
}
rtems_adc_status rtems_adc_read_nb(
    rtems_gpio *base,
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

rtems_status_code rtems_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->set_resolution(base, bits);
    }
    return RTEMS_UNSATISFIED;
}
rtems_status_code rtems_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->set_alignment(base, align);
    }
    return RTEMS_UNSATISFIED;
}

rtems_status_code rtems_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->configure_interrupt(base, isr, arg);
    }
    return RTEMS_UNSATISFIED;
}
rtems_status_code rtems_adc_remove_interrupt(
    rtems_gpio *base
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->remove_interrupt(base);
    }
    return RTEMS_UNSATISFIED;
}
rtems_status_code rtems_adc_enable_interrupt(
    rtems_gpio *base
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->enable_interrupt(base);
    }
    return RTEMS_UNSATISFIED;
}
rtems_status_code rtems_adc_disable_interrupt(
    rtems_gpio *base
)
{
    if (base->is_adc_pin) {
        return base->adc_handlers->disable_interrupt(base);
    }
    return RTEMS_UNSATISFIED;
}
