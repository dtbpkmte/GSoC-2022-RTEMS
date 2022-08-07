/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup adc
 *
 * RTEMS ADC API.
 *
 * This API is created to improve portability for ADC-related operations.
 *
 * Drivers need to register each ADC controller to the manager using 
 * @ref rtems_adc_register(). Then, users can get an ADC object of a controller
 * by specifying the index of that controller (starting from 0, the first 
 * registered ADC). 
 * 
 * Sometimes, ADCs are integrated into a GPIO pin (such as on-chip ADC). If so,
 * additional effort to set the pin mode to analog might be required.
 *
 * The general process to use ADC API:
 * - Get an ADC object using @ref rtems_adc_get()
 * - Configure ADC object with rtems_adc_set_* functions
 * - Call @ref rtems_adc_init()
 * - The ADC object should be ready by now
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
#include <bsp/gpio2.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern volatile int *adc_dummy;
/**
  * @brief Macro to link BSP source file.
  *
  * There might be a case that a BSP GPIO source file is not linked because
  * of no reference. Use this macro to create a dummy variable to link with
  * GPIO API. This macro should be placed outside of any function.
  */
#define RTEMS_ADC_LINK()                       \
    static int ___dummy___ = 0;                \
    volatile int *adc_dummy = &___dummy___;

/**
  * Configure the maximum number of ADC controllers used in
  * a application.
  *
  * The macro CONFIGURE_ADC_MAXIMUM_CONTROLLERS is a build option.
  * If it is not defined, it will default to BSP_ADC_NUM_CONTROLLERS. 
  * If BSP's number of controllers is not defined, it will default
  * to 0.
  */
#ifndef CONFIGURE_ADC_MAXIMUM_CONTROLLERS

#ifndef BSP_ADC_NUM_CONTROLLERS
#define CONFIGURE_ADC_MAXIMUM_CONTROLLERS 0
#else
#define CONFIGURE_ADC_MAXIMUM_CONTROLLERS BSP_ADC_NUM_CONTROLLERS
#endif /* BSP_ADC_NUM_CONTROLLERS */

#endif /* CONFIGURE_ADC_MAXIMUM_CONTROLLERS */

typedef enum {
    RTEMS_ADC_NOT_STARTED = 0,
    RTEMS_ADC_NOT_READY,
    RTEMS_ADC_READY
} rtems_adc_status;

/**
  * @brief Data alignment.
  */
typedef enum {
    RTEMS_ADC_ALIGN_LEFT,
    RTEMS_ADC_ALIGN_RIGHT
} rtems_adc_align;

/**
  * @brief Enumeration of reference voltages.
  */
typedef enum {
    RTEMS_ADC_REF_DEFAULT = 0,
    RTEMS_ADC_REF_INTERNAL,
    RTEMS_ADC_REF_EXTERNAL,
    RTEMS_ADC_REF_BSP_SPECIFIC = 100
} rtems_adc_ref;

#define RTEMS_ADC_NO_TIMEOUT   0xFFFFFFFFU

typedef void (*rtems_adc_isr)(void *);
typedef double (*rtems_adc_tf) (void *params, uint32_t raw_value);
typedef struct rtems_adc_handlers rtems_adc_handlers;
typedef struct rtems_adc_config rtems_adc_config;
typedef struct rtems_adc rtems_adc;

/**
  * @brief Driver-specific handlers.
  * Each driver need to create an object of this structure to supply to
  * the API.
  */
struct rtems_adc_handlers {
    void (*init) (rtems_adc *);
    /**
      * @brief Pointer to a function that reads raw ADC value.
      * This function is blocking and has a timeout parameter.
      */
    rtems_status_code (*read_raw) (rtems_adc *, uint32_t *, uint32_t);
    /**
      * @brief Pointer to a function that starts ADC conversion
      *        in non-blocking style.
      */
    rtems_status_code (*start_read_raw_nb) (rtems_adc *);
    /**
      * @brief Pointer to a function that gets a raw ADC value when
      *        available after a start_read_raw_nb() call. 
      * If data is not available, the function should return a status
      * to indicate that.
      */
    rtems_adc_status (*read_raw_nb) (rtems_adc *, uint32_t *);
    /**
      * @brief Sets the channel of an ADC.
      *
      */
    rtems_status_code (*set_channel) (rtems_adc *, uint32_t);
    /**
      * @brief Pointer to a function that sets the resolution of an
      *        ADC controller associated with a pin.
      *
      * @note If a controller contains multiple pins, the resolution
      *       setting may affect all of them.
      */
    rtems_status_code (*set_resolution) (rtems_adc *, unsigned int);
    /**
      * @brief Pointer to a function that sets the alignment of an
      *        ADC controller associated with a pin.
      *
      * @note If a controller contains multiple pins, the alignment
      *       setting may affect all of them.
      */
    rtems_status_code (*set_alignment) (rtems_adc *, rtems_adc_align);
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
    rtems_status_code (*configure_interrupt) (rtems_adc *, rtems_adc_isr, void *);
    /**
      * @brief This member is the pointer to a handler for removing
      *        interrupt settings of a pin.
      */
    rtems_status_code (*remove_interrupt) (rtems_adc *);
    /**
      * @brief This member is the pointer to a handler for enabling
      *        interrupt functionality of a pin.
      */
    rtems_status_code (*enable_interrupt) (rtems_adc *);
    /**
      * @brief This member is the pointer to a handler for disabling
      *        interrupt of a pin.
      */
    rtems_status_code (*disable_interrupt) (rtems_adc *);
};

/**
  * @brief ADC configuration structure.
  */
struct rtems_adc_config {
    uint32_t channel;
    rtems_adc_align alignment;
    uint32_t resolution;
};

/**
  * @brief ADC core object.
  */
struct rtems_adc {
    /**
      * @brief The index of an ADC controller.
      */
    const uint32_t id;
    /**
      * @brief Driver-specific handlers.
      */
    const rtems_adc_handlers *const handlers;
    /**
      * @brief ADC channel configuration.
      */
    rtems_adc_config config;
    /**
      * @brief A transfer function that will be assigned to this pin.
      * If no transfer function assigned, it should remain NULL.
      */
    rtems_adc_tf tf;
    void *tf_params;
};

/**
  * @brief Registers an ADC to the ADC manager.
  */
extern rtems_status_code rtems_adc_register(
    rtems_status_code (*get_adc) (uint32_t, rtems_adc **),
    rtems_status_code (*destroy_adc) (rtems_adc *)
);

/**
  * @brief Get the ADC object by ADC controller index.
  */
extern rtems_status_code rtems_adc_get(
    uint32_t id,
    rtems_adc **out
);

/**
  * @brief Frees an ADC object.
  */
extern rtems_status_code rtems_adc_destroy(
    rtems_adc *adc
);

/**
  * @brief Perform initialization for an ADC controller/channel.
  */
extern void rtems_adc_init(
    rtems_adc *base
);

/**
  * @brief Read raw ADC value with infinite timeout.
  *
  * @param base[in] Pointer to an ADC object.
  * @param result[out] Pointer to result variable.
  *
  * @retval
  */
extern rtems_status_code rtems_adc_read_raw(
    rtems_adc *base, 
    uint32_t *result
);

/**
  * @brief Read raw ADC value with specified timeout.
  *
  * @param base[in] Pointer to an ADC object.
  * @param result[out] Pointer to result variable.
  * @param timeout 
  *
  * @retval
  */
extern rtems_status_code rtems_adc_read_raw_timeout(
    rtems_adc *base, 
    uint32_t *result,
    uint32_t timeout
);

/**
  * @brief Starts a non-blocking ADC conversion.
  *
  * This function must be called before
  * rtems_adc_read_raw_nb() or rtems_adc_read_nb()
  *
  * @param base[in] Pointer to an ADC object.
  *
  * @retval
  */
extern rtems_status_code rtems_adc_start_read_nb(
    rtems_adc *base
);

/**
  * @brief Reads raw ADC value non-blocking.
  * User can query the status of the conversion by the returned status code.
  *
  * @param base[in] Pointer to an ADC object.
  * @param result[out] Pointer to result variable.
  *
  * @retval @see rtems_adc_status
  */
extern rtems_adc_status rtems_adc_read_raw_nb(
    rtems_adc *base, 
    uint32_t *result
);

/**
  * @brief Assigns a transfer function with parameters
  *        to a pin.
  */
extern rtems_status_code rtems_adc_assign_tf(
    rtems_adc *base,
    rtems_adc_tf tf, 
    void *params
);

/**
  * @brief Removes the assigned transfer function from
  *        a pin.
  */
extern rtems_status_code rtems_adc_remove_tf(
    rtems_adc *base
);

/**
  * @brief Reads an ADC value with infinite timeout.
  *
  * If no transfer function assigned, this will
  * return the raw value via result pointer. Else, 
  * it returns the calculated value using transfer
  * function.
  *
  * @param base[in] Pointer to an ADC object.
  * @param result[out] Pointer to result variable.
  *
  * @retval
  */
extern rtems_status_code rtems_adc_read(
    rtems_adc *base, 
    double *result
);

/**
  * @brief Reads an ADC value with specified timeout.
  *
  * If no transfer function assigned, this will
  * return the raw value via result pointer. Else, 
  * it returns the calculated value using transfer
  * function.
  *
  * @param base[in] Pointer to an ADC object.
  * @param result[out] Pointer to result variable.
  * @param timeout
  *
  * @retval
  */
extern rtems_status_code rtems_adc_read_timeout(
    rtems_adc *base, 
    double *result,
    uint32_t timeout
);

extern rtems_adc_status rtems_adc_read_nb(
    rtems_adc *base, 
    double *result
);

/**
  * @brief Set the channel of an ADC object.
  */
extern rtems_status_code rtems_adc_set_channel(
    rtems_adc *base,
    uint32_t channel
);

/**
  * @brief Set the resolution of an ADC object.
  */
extern rtems_status_code rtems_adc_set_resolution(
    rtems_adc *base,
    unsigned int bits
);

/**
  * @brief Set the alignment of an ADC object.
  */
extern rtems_status_code rtems_adc_set_alignment(
    rtems_adc *base,
    rtems_adc_align align
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_ADC_H */
