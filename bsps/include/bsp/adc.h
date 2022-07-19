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

#ifndef LIBBSP_BSP_ADC_H
#define LIBBSP_BSP_ADC_H

#include <bsp.h>
#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum {
    RTEMS_ADC_NOT_STARTED = 0,
    RTEMS_ADC_NOT_READY,
    RTEMS_ADC_READY
} rtems_adc_status;

typedef enum {
    RTEMS_ADC_ALIGN_LEFT,
    RTEMS_ADC_ALIGN_RIGHT
} rtems_adc_align;

typedef enum {
    RTEMS_ADC_NB_INTERRUPT,
    RTEMS_ADC_NB_DMA
} rtems_adc_nb_mode;

#define RTEMS_ADC_NO_TIMEOUT   0xFFFFFFFFU

typedef void (*rtems_adc_isr)(void *);
typedef double (*rtems_adc_tf) (void *params, uint32_t raw_value);
typedef struct rtems_adc_handlers rtems_adc_handlers;

#include <bsp/gpio2.h>

struct rtems_adc_handlers {
    rtems_status_code (*read_raw) (rtems_gpio *, uint32_t *, uint32_t);
    rtems_status_code (*start_read_raw_nb) (rtems_gpio *);
    rtems_adc_status (*read_raw_nb) (rtems_gpio *, uint32_t *);
    rtems_status_code (*set_resolution) (rtems_gpio *, unsigned int);
    rtems_status_code (*set_alignment) (rtems_gpio *, rtems_adc_align);
    rtems_status_code (*configure_interrupt) (rtems_gpio *, rtems_adc_isr, void *);
    rtems_status_code (*remove_interrupt) (rtems_gpio *);
    rtems_status_code (*enable_interrupt) (rtems_gpio *);
    rtems_status_code (*disable_interrupt) (rtems_gpio *);
};

/**
  * @brief Read raw ADC value with infinite timeout.
  *
  * @param base[in]
  * @param result[out]
  *
  * @retval
  */
extern rtems_status_code rtems_adc_read_raw(
    rtems_gpio *base, 
    uint32_t *result
);
extern rtems_status_code rtems_adc_read_raw_timeout(
    rtems_gpio *base, 
    uint32_t *result,
    uint32_t timeout
);

/**
  * @brief Starts a non-blocking ADC conversion.
  *
  * This function must be called before
  * rtems_adc_read_raw_nb() or rtems_adc_read_nb()
  *
  * @param base
  *
  * @retval
  */
extern rtems_adc_status rtems_adc_start_read_nb(
    rtems_gpio *base
);

extern rtems_status_code rtems_adc_read_raw_nb(
    rtems_gpio *base, 
    uint32_t *result
);

extern rtems_status_code rtems_adc_assign_tf(
    rtems_gpio *base,
    rtems_adc_tf tf, 
    void *params
);
extern rtems_status_code rtems_adc_remove_tf(
    rtems_gpio *base
);

/**
  * @brief Reads an ADC value with infinite timeout.
  *
  * If no transfer function assigned, this will
  * return the raw value via result pointer. Else, 
  * it returns the calculated value using transfer
  * function.
  *
  * @param base[in]
  * @param result[out]
  *
  * @retval
  */
extern rtems_status_code rtems_adc_read(
    rtems_gpio *base, 
    double *result
);
extern rtems_status_code rtems_adc_read_timeout(
    rtems_gpio *base, 
    double *result,
    uint32_t timeout
);
extern rtems_status_code rtems_adc_read_nb(
    rtems_gpio *base, 
    double *result
);

extern rtems_status_code rtems_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
);
extern rtems_status_code rtems_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
);

extern rtems_status_code rtems_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
);
extern rtems_status_code rtems_adc_remove_interrupt(
    rtems_gpio *base
);
extern rtems_status_code rtems_adc_enable_interrupt(
    rtems_gpio *base
);
extern rtems_status_code rtems_adc_disable_interrupt(
    rtems_gpio *base
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_ADC_H */
