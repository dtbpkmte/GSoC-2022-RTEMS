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

#ifdef __ENABLE_ADC_API

#include <stdbool.h>

/**
  * @brief Macro to check if an API pointer is ADC API.
  * This only works if RTEMS_DEBUG is defined. This macro can
  * only be called from functions. 
  *
  * @param _api_ptr A pointer to rtems_periph_api.
  */ 
#ifdef RTEMS_DEBUG
#define RTEMS_ADC_API_TYPE_CHECK( _api_ptr )     \
    do {                                                \
        if ( *(rtems_periph_type *) ( _api_ptr ) != ( RTEMS_PERIPH_TYPE_ADC ) )               \
            return RTEMS_UNSATISFIED;                   \
    } while (0)
#else
#define RTEMS_ADC_API_TYPE_CHECK( _type )     \
    ((void) ( _type ))
#endif /* RTEMS_DEBUG */

rtems_status_code rtems_adc_read_raw(
    rtems_gpio *base,
    uint32_t *result
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->read_raw(base, result, RTEMS_ADC_NO_TIMEOUT);
}
rtems_status_code rtems_adc_read_raw_timeout(
    rtems_gpio *base,
    uint32_t *result,
    uint32_t timeout
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->read_raw(base, result, timeout);
}

rtems_adc_status rtems_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->read_raw_nb(base, result);
}

rtems_status_code rtems_adc_assign_tf(
    rtems_gpio *base,
    rtems_adc_tf tf,
    void *params
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    ((rtems_adc_api *) (base->api))->tf = tf;
    ((rtems_adc_api *) (base->api))->tf_params = params;
    return RTEMS_SUCCESSFUL;
}
rtems_status_code rtems_adc_remove_tf(
    rtems_gpio *base
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    ((rtems_adc_api *) (base->api))->tf = NULL;
    ((rtems_adc_api *) (base->api))->tf_params = NULL;
    return RTEMS_SUCCESSFUL;
}
rtems_status_code rtems_adc_read(
    rtems_gpio *base,
    double *result
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    uint32_t raw;
    rtems_status_code sc = rtems_adc_read_raw(base, &raw);
    if (sc == RTEMS_SUCCESSFUL) {
        if (((rtems_adc_api *) (base->api))->tf == NULL)
            *result = (double) raw;
        else
            *result = ((rtems_adc_api *) (base->api))->tf(((rtems_adc_api *) (base->api))->tf_params, raw);
    }
    return sc;
}
rtems_status_code rtems_adc_read_timeout(
    rtems_gpio *base,
    double *result,
    uint32_t timeout
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    uint32_t raw;
    rtems_status_code sc = rtems_adc_read_raw_timeout(base, &raw, timeout);
    if (sc == RTEMS_SUCCESSFUL) {
        if (((rtems_adc_api *) (base->api))->tf == NULL)
            *result = (double) raw;
        else
            *result = ((rtems_adc_api *) (base->api))->tf(((rtems_adc_api *) (base->api))->tf_params, raw);
    }
    return sc;
}

rtems_status_code rtems_adc_start_read_nb(
    rtems_gpio *base
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->start_read_raw_nb(base);
}
rtems_adc_status rtems_adc_read_nb(
    rtems_gpio *base,
    double *result
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    uint32_t raw;
    rtems_adc_status sc = rtems_adc_read_raw_nb(base, &raw);
    if (sc == RTEMS_ADC_READY) {
        if (((rtems_adc_api *) (base->api))->tf == NULL)
            *result = (double) raw;
        else
            *result = ((rtems_adc_api *) (base->api))->tf(((rtems_adc_api *) (base->api))->tf_params, raw);
    }
    return sc;
}

rtems_status_code rtems_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->set_resolution(base, bits);
}
rtems_status_code rtems_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->set_alignment(base, align);
}

rtems_status_code rtems_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->configure_interrupt(base, isr, arg);
}
rtems_status_code rtems_adc_remove_interrupt(
    rtems_gpio *base
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->remove_interrupt(base);
}
rtems_status_code rtems_adc_enable_interrupt(
    rtems_gpio *base
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->enable_interrupt(base);
}
rtems_status_code rtems_adc_disable_interrupt(
    rtems_gpio *base
)
{
    RTEMS_ADC_API_TYPE_CHECK( base->api );
    return ((rtems_adc_api *) (base->api))->disable_interrupt(base);
}

#endif /* __ENABLE_ADC_API */
