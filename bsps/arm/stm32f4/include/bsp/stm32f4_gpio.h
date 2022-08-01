/* SPDX-License-Identifier: BSD-2-Clause */

/**
  * @file
  * 
  * @ingroup stm32f4_gpio
  *
  * STM32F4 BSP implementation of GPIO.
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
 
#ifndef LIBBSP_ARM_STM32F4_BSP_GPIO
#define LIBBSP_ARM_STM32F4_BSP_GPIO

#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_exti.h>
#include <bsp/gpio2.h>

/*********** Helpers *****************/
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
#define stm32f4_get_gpio_from_base(_base) \
    RTEMS_CONTAINER_OF(_base, stm32f4_gpio, base)

/**
  * @brief STM32F4 BSP GPIO structure
  *
  */
typedef struct {
    /**
      * @brief This member is a rtems_gpio object.
      */
    rtems_gpio base;
    /**
      *@brief This member is the pin number from 0 to 15.
      */
    uint32_t pin;
    /**
      * @brief This member is HAL GPIOx pointer.
      */
    GPIO_TypeDef *port;
} stm32f4_gpio;

/**
  * @name STM32F4 GPIO functions
  *
  * @{
  */

/**
  * @name GPIO API implementation of STM32F4 BSP
  *
  * @{
  */


/**
  * @brief Sets the pin mode.
  *
  * @param[in] base The pointer to the GPIO object.
  * @param mode is a value of @see rtems_gpio_pin_mode
  *
  * @retval RTEMS_SUCCESSFUL if the pin mode is valid.
  * @retval RTEMS_UNSATISFIED if the pin mode is 
  *         invalid.
  */
extern rtems_status_code stm32f4_gpio_set_pin_mode(
    rtems_gpio *base,
    rtems_gpio_pin_mode mode
);

/**
  * @brief Sets pull resistor mode.
  *
  * @param[in] base The pointer to the GPIO object.
  * @param pull is a value of @see rtems_gpio_pull
  *
  * @retval RTEMS_SUCCESSFUL if the pull resistor 
  *         mode is valid.
  * @retval RTEMS_UNSATISFIED if the pull resistor
  *         mode is invalid.
  */
extern rtems_status_code stm32f4_gpio_set_pull(
    rtems_gpio *base,
    rtems_gpio_pull pull
);

/**
  * @brief Configures user-defined ISR for an EXTI.
  *
  * This function is used to register a custom ISR
  * with a GPIO pin. This API supports up to 1 ISR
  * for each EXTI line (total 16 ISRs) at a time.
  * 
  * @note If there is already an ISR registered with 
  *       a line, it needs to be removed to be able
  *       to register a new one.
  * @note This function does not enable interrupt.
  *       use @see rtems_gpio_enable_interrupt().
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL if interrupt 
  *         configuration is successful.
  * @retval RTEMS_UNSATISFIED if trigger mode/pull
  *         resistor mode is invalid or an ISR
  *         has already been registered for this 
  *         EXTI line.
  * @retval @see rtems_interrupt_handler_install()
  */
extern rtems_status_code stm32f4_gpio_configure_interrupt(
    rtems_gpio *base, 
    rtems_gpio_isr isr,
    void *arg,
    rtems_gpio_interrupt_trig trig,
    rtems_gpio_pull pull
);

/**
  * @brief Removes the registered ISR.
  *
  * @note This function does not disable interrupt.
  * @ref rtems_gpio_disable_interrupt()
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL if ISR removed successfully.
  * @retval RTEMS_UNSATISFIED if no ISR registered for 
  *         selected line.
  * @retval @see rtems_interrupt_handler_remove()
  */
extern rtems_status_code stm32f4_gpio_remove_interrupt(
    rtems_gpio *base
);

/**
  * @brief Enables EXTI for the line connected to this
  *        pin.
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL
  */
extern rtems_status_code stm32f4_gpio_enable_interrupt(
    rtems_gpio *base
);

/**
  * @brief Disables EXTI for the line connected to 
  *        this pin.
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL
  */
extern rtems_status_code stm32f4_gpio_disable_interrupt(
    rtems_gpio *base
);

/**
  * @brief Reads digital value into a variable.
  *
  * @param[in] base The pointer to the GPIO object.
  * @param[out] value The pointer to the output
  *             variable.
  *
  * @retval RTEMS_SUCCESSFUL
  */
extern rtems_status_code stm32f4_gpio_read(
    rtems_gpio *base,
    rtems_gpio_pin_state *value
);

/**
  * @brief Writes digital value to a pin.
  *
  * @param[in] base The pointer to the GPIO object.
  * @param value The output digital value.
  *
  * @retval RTEMS_SUCCESSFUL
  */
extern rtems_status_code stm32f4_gpio_write(
    rtems_gpio *base,
    rtems_gpio_pin_state value
);

/**
  * @brief Toggles the state of a pin.
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL
  */
extern rtems_status_code stm32f4_gpio_toggle(
    rtems_gpio *base
);

/** @} */

/**
  * @name Extra functionality of STM32F4 GPIO
  *
  * @{
  */

/**
  * @brief Initializes clock for the GPIO port
  *        owning this pin.
  *
  * @note This function is called in stm32f4_gpio_get().
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL if the port argument is
  *         valid
  * @retval RTEMS_UNSATISFIED if the port argument is
  *         invalid
  */
extern rtems_status_code stm32f4_gpio_init(
    rtems_gpio *base
);

/**
  * @brief Lock configuration of a pin.
  *
  * @param[in] base The pointer to the GPIO object.
  */
extern void stm32f4_gpio_lock_pin(
    rtems_gpio *base
);

/**
  * @brief Sets the alternate function for a pin.
  *
  * @param[in] base The pointer to the GPIO object.
  * @param alternate Alternate function, from 0-15.
  */
extern void stm32f4_gpio_set_af(
    rtems_gpio *base,
    uint32_t alternate
);

/** @} */

/** @} */

#endif /* LIBBSP_ARM_STM32F4_BSP_GPIO */
