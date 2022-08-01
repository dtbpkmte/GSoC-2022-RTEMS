/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup adc
 *
 * RTEMS ADC API.
 *
 * This API is created to improve portability and simplicity for ADC.
 *
 * This API extends the Peripheral API. Its type is RTEMS_PERIPH_API_TYPE_ADC.
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

#ifndef LIBBSP_BSP_ADC_H
#define LIBBSP_BSP_ADC_H

#include <bsp.h>
#include <rtems.h>
#include <bsp/periph_api.h>
#include <bsp/gpio2.h>


#ifdef __ENABLE_GPIO_API

#define __ENABLE_ADC_API

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

/**
  * @brief Enumeration of reference voltages.
  */
typedef enum {
    RTEMS_ADC_REF_DEFAULT = 0,
    RTEMS_ADC_REF_INTERNAL,
    RTEMS_ADC_REF_EXTERNAL
} rtems_adc_ref;

#define RTEMS_ADC_NO_TIMEOUT   0xFFFFFFFFU

typedef void (*rtems_adc_isr)(void *);
typedef double (*rtems_adc_tf) (void *params, uint32_t raw_value);
typedef struct rtems_adc_handlers rtems_adc_handlers;
typedef struct rtems_adc_api rtems_adc_api;

/**
  * @brief Macro to help creating a rtems_adc_api object.
  *
  * Each BSP/driver must define its own handlers and create an object 
  * of this struct with pointers to those handlers.
  */
#define RTEMS_ADC_BUILD_API(                                    \
        _init,                                                  \
        _read_raw,                                              \
        _start_read_raw_nb,                                     \
        _read_raw_nb,                                           \
        _set_resolution,                                        \
        _set_alignment,                                         \
        _configure_interrupt,                                   \
        _remove_interrupt,                                      \
        _enable_interrupt,                                      \
        _disable_interrupt)                                     \
    {                                                           \
        .base = {                                               \
            .api_type = RTEMS_PERIPH_API_TYPE_ADC,              \
            .init = _init                                       \
        },                                                      \
        .read_raw = ( _read_raw ),                              \
        .start_read_raw_nb = ( _start_read_raw_nb ),            \
        .read_raw_nb = ( _read_raw_nb ),                        \
        .set_resolution = ( _set_resolution ),                  \
        .set_alignment = ( _set_alignment ),                    \
        .configure_interrupt = ( _configure_interrupt ),        \
        .remove_interrupt = ( _remove_interrupt ),              \
        .enable_interrupt = ( _enable_interrupt ),              \
        .disable_interrupt = ( _disable_interrupt ),            \
    };

struct rtems_adc_api {
    /**
      * @brief Contain base structure rtems_periph_api.
      * @see rtems_periph_api
      */
    rtems_periph_api base;
    /**
      * @brief This member is a pointer to a transfer function
      *        that will be assigned to this pin.
      * If no transfer function assigned, it should remain NULL.
      */
    rtems_adc_tf tf;
    void *tf_params;
    /**
      * @brief Pointer to a function that reads raw ADC value.
      * This function is blocking and has a timeout parameter.
      */
    rtems_status_code (*read_raw) (rtems_gpio *, uint32_t *, uint32_t);
    /**
      * @brief Pointer to a function that starts ADC conversion
      *        in non-blocking style.
      */
    rtems_status_code (*start_read_raw_nb) (rtems_gpio *);
    /**
      * @brief Pointer to a function that gets a raw ADC value when
      *        available after a start_read_raw_nb() call. 
      * If data is not available, the function should return a status
      * to indicate that.
      */
    rtems_adc_status (*read_raw_nb) (rtems_gpio *, uint32_t *);
    /**
      * @brief Pointer to a function that sets the resolution of an
      *        ADC controller associated with a pin.
      *
      * @note If a controller contains multiple pins, the resolution
      *       setting may affect all of them.
      */
    rtems_status_code (*set_resolution) (rtems_gpio *, unsigned int);
    /**
      * @brief Pointer to a function that sets the alignment of an
      *        ADC controller associated with a pin.
      *
      * @note If a controller contains multiple pins, the alignment
      *       setting may affect all of them.
      */
    rtems_status_code (*set_alignment) (rtems_gpio *, rtems_adc_align);
    /**
      * @brief This member is the pointer to a handler for configuring
      *        interrupt of a pin.
      *
      * This handler should register ISR and its arguments.
      *
      * @note Interrupt may occur when ADC conversion is completed.
      * @note Enabling interrupt should be done in enable_interrupt()
      *       handler.
      */
    rtems_status_code (*configure_interrupt) (rtems_gpio *, rtems_adc_isr, void *);
    /**
      * @brief This member is the pointer to a handler for removing
      *        interrupt settings of a pin.
      */
    rtems_status_code (*remove_interrupt) (rtems_gpio *);
    /**
      * @brief This member is the pointer to a handler for enabling
      *        interrupt functionality of a pin.
      */
    rtems_status_code (*enable_interrupt) (rtems_gpio *);
    /**
      * @brief This member is the pointer to a handler for disabling
      *        interrupt of a pin.
      */
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
extern rtems_status_code rtems_adc_start_read_nb(
    rtems_gpio *base
);

/**
  * @brief Reads raw ADC value non-blocking.
  */
extern rtems_adc_status rtems_adc_read_raw_nb(
    rtems_gpio *base, 
    uint32_t *result
);

/**
  * @brief Assigns a transfer function with parameters
  *        to a pin.
  */
extern rtems_status_code rtems_adc_assign_tf(
    rtems_gpio *base,
    rtems_adc_tf tf, 
    void *params
);

/**
  * @brief Removes the assigned transfer function from
  *        a pin.
  */
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
extern rtems_adc_status rtems_adc_read_nb(
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

#endif /* __ENABLE_GPIO_API */

#endif /* LIBBSP_BSP_ADC_H */
