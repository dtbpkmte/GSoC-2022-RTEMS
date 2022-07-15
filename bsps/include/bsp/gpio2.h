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

#ifndef LIBBSP_BSP_GPIO2_H
#define LIBBSP_BSP_GPIO2_H

#include <bsp.h>
#include <rtems.h>

/**
  * Configure the maximum number of GPIO controllers used in
  * a application.
  *
  * The macro CONFIGURE_GPIO_MAXIMUM_CONTROLLERS can be
  * defined in application code. If it is not defined,
  * it will default to BSP_GPIO_NUM_CONTROLLERS. If BSP's
  * number of controllers is not defined, it will default
  * to 1.
  */
#ifndef CONFIGURE_GPIO_MAXIMUM_CONTROLLERS

#ifndef BSP_GPIO_NUM_CONTROLLERS
#define CONFIGURE_GPIO_MAXIMUM_CONTROLLERS 1
#else
#define CONFIGURE_GPIO_MAXIMUM_CONTROLLERS BSP_GPIO_NUM_CONTROLLERS
#endif /* BSP_GPIO_NUM_CONTROLLERS */

#endif /* CONFIGURE_GPIO_MAXIMUM_CONTROLLERS */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define RTEMS_GPIO_BUILD_BASE(gpioh, adch) \
    (rtems_gpio) { .gpio_handlers = ( gpioh ), \
                   .adc_handlers = ( adch ) };

/**
  * @name GPIO data structures
  *
  * @{
  */

/**
  * @brief GPIO bit set and reset enumeration.
  */
typedef enum {
    RTEMS_GPIO_PIN_RESET = 0,
    RTEMS_GPIO_PIN_SET = 1
} rtems_gpio_pin_state;

/**
  * @brief GPIO pin modes. 
  */
typedef enum {
    RTEMS_GPIO_PINMODE_OUTPUT = 0,
    RTEMS_GPIO_PINMODE_OUTPUT_PP = 0,
    RTEMS_GPIO_PINMODE_OUTPUT_OD = 1,
    RTEMS_GPIO_PINMODE_INPUT = 2,
    RTEMS_GPIO_PINMODE_ANALOG = 3,
    RTEMS_GPIO_PINMODE_BSP_SPECIFIC = 4
} rtems_gpio_pin_mode;

/**
  * @brief GPIO pull resistor configuration. Defines pull-up or 
  *        pull-down activation.
  */
typedef enum {
    RTEMS_GPIO_NOPULL,
    RTEMS_GPIO_PULLUP,
    RTEMS_GPIO_PULLDOWN
} rtems_gpio_pull;

/**
  * @brief Interrupt modes enumeration.
  */
typedef enum {
    RTEMS_GPIO_INT_TRIG_NONE = 0,
    RTEMS_GPIO_INT_TRIG_FALLING,
    RTEMS_GPIO_INT_TRIG_RISING,
    RTEMS_GPIO_INT_TRIG_BOTH_EDGES,
    RTEMS_GPIO_INT_TRIG_LOW,
    RTEMS_GPIO_INT_TRIG_HIGH
} rtems_gpio_interrupt_trig;

typedef struct rtems_gpio_handlers rtems_gpio_handlers;
typedef struct rtems_gpio rtems_gpio;
/**
  * @brief Typedef of the function pointer of an ISR.
  */
typedef void (*rtems_gpio_isr)(void *);

#include <bsp/adc.h>

/**
  * @brief Structure containing pointers to handlers of a
  *        BSP/driver. Each BSP/driver must define its own 
  *        handlers and create an object of this structure
  *        with pointers to those handlers.
  */
struct rtems_gpio_handlers {
    /**
      * @brief This member is the pointer to an initialize handler. 
      *
      * This handler could be used to perform some set up steps for
      * a GPIO object (which means a pin or a port).
      */
    rtems_status_code (*init)(rtems_gpio *);

    /**
      * @brief This member is the pointer to a deinitialize handler. 
      *
      * This handler could be used to deinitialize a GPIO object.
      */
    rtems_status_code (*deinit)(rtems_gpio *);

    /**
      * @brief This member is the pointer to a handler for setting 
      *        pin mode. 
      *
      * Pin modes are from rtems_gpio_pin_mode enumeration.
      */
    rtems_status_code (*set_pin_mode)(rtems_gpio *, rtems_gpio_pin_mode);

    /**
      * @brief This member is the pointer to a handler for setting
      *        pull resistor mode. 
      *
      * Pull resistor modes are from rtems_gpio_pull enumeration.
      */
    rtems_status_code (*set_pull)(rtems_gpio *, rtems_gpio_pull);

    /**
      * @brief This member is the pointer to a handler for configuring
      *        interrupt of a pin. 
      * 
      * This handler should register ISR and its argument, interrupt
      * trigger mode, and pull resister mode for the pin.
      *
      * @note Enabling interrupt should be done in enable_interrupt()
      *       handler.
      */
    rtems_status_code (*configure_interrupt)(rtems_gpio *, rtems_gpio_isr, void *, rtems_gpio_interrupt_trig, rtems_gpio_pull);

    /**
      * @brief This member is the pointer to a handler for removing
      *        interrupt settings of a pin. 
      *
      * Interrupt settings can be ISR address, pin configuration, etc.
      */
    rtems_status_code (*remove_interrupt)(rtems_gpio *);

    /**
      * @brief This member is the pointer to a handler for enabling
      *        interrupt functionality of a pin.
      */
    rtems_status_code (*enable_interrupt)(rtems_gpio *);

    /**
      * @brief This member is the pointer to a handler for disabling
      *        interrupt of a pin. 
      */
    rtems_status_code (*disable_interrupt)(rtems_gpio *);

    /**
      * @brief This member is the pointer to a handler for reading
      *        the digital value of a pin. 
      *
      * The returned value should be in rtems_gpio_pin_state enum.
      */
    rtems_status_code (*read)(rtems_gpio *, rtems_gpio_pin_state *);

    /**
      * @brief This member is the pointer to a handler for writing 
      *        a digital value to a pin. 
      */
    rtems_status_code (*write)(rtems_gpio *, rtems_gpio_pin_state);

    /**
      * @brief This member is the pointer to a handler for toggling
      *        a pin. 
      *
      * It should change pin state from SET to RESET or vice versa.
      */
    rtems_status_code (*toggle)(rtems_gpio *);

};

/**
  * @brief Structure representing a GPIO object. 
  *
  * BSP/drivers need to define their own GPIO structures with
  * rtems_gpio being the first member.
  */
struct rtems_gpio {
    /**
      * @brief This member is a pointer to a structure containing
      *        pointers to handlers of a GPIO controller.
      */
    const rtems_gpio_handlers *gpio_handlers;
    /**
      * @brief This member is a pointer to a structure containing
      *        pointers to handlers of an ADC.
      */
    const rtems_adc_handlers *adc_handlers;

    /**
      * @brief This member is a virtual pin number, counting from
      *        0 (zero).
      */
    uint32_t virtual_pin;
};

/** @} */

/**
  * @name GPIO BSP functions
  *
  * BSP and drivers need to implement these.
  *
  * @{
  */

/**
  * @brief Wrapper to perform all BSP controllers registering
  *        with the GPIO manager.
  *
  * This function must be implemented by BSP. It should call
  * rtems_gpio_register() to register each integrated GPIO
  * controller.
  */
extern rtems_status_code bsp_gpio_register_controllers(
    void
);

/** @} */


/** @} */

/**
  * @name GPIO operations
  *
  * @{
  */

/**
  * @brief Get the GPIO object containing information
  *        about the specified pin.
  * 
  * This function maps the virtual pin to intermediate pin, 
  * and pass to the BSP/driver-specific function to get a
  * GPIO object.
  *
  * @note Warning: this function may use malloc(). When you 
  * are done with the GPIO object, call rtems_gpio_destroy()
  * to avoid memory leak.
  *
  * @param virt_pin The virtual pin number.
  * @param[out] out The pointer to the pointer to the output
  *             GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL
  * @retval RTEMS_UNSTISFIED if the virtual pin number
  *         is invalid (i.e. out of range)
  */
extern rtems_status_code rtems_gpio_get(
    uint32_t virt_pin,
    rtems_gpio **out
);

/**
  * @brief Destroy a GPIO object
  *
  * This function should be called on an GPIO object which is
  * no longer used to avoid memory leak. Internally it can 
  * use free().
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL
  * @retval RTEMS_UNSATISFIED
  */
extern rtems_status_code rtems_gpio_destroy(
    rtems_gpio *base
);

/**
  * @brief Registers a GPIO controller with GPIO manager.
  *
  * This function registers the pointer to BSP/driver-specific
  * get_gpio() and destroy_gpio() functions. Those two functions
  * are for creating and destroying GPIO objects. It also takes 
  * the number of pins of each BSP/driver for mapping into a
  * flat pin numbering system (virtual pin number).
  *
  * @param get_gpio The pointer to BSP/driver-specific get_gpio()
  * @param destroy_gpio The pointer to BSP/driver-specific 
  *        destroy_gpio()
  * @param pin_count The number of GPIO pins in the controller
  *
  * @retval RTEMS_SUCCESSFUL Controller registered successfully
  * @retval RTEMS_TOO_MANY if the maximum number of controllers are
  *         already registered
  */
extern rtems_status_code rtems_gpio_register(
    rtems_status_code (*get_gpio)(uint32_t, rtems_gpio **),
    rtems_status_code (*destroy_gpio)(rtems_gpio *),
    uint32_t pin_count
);

/**
  * @brief Initialize a GPIO object
  *
  * This function calls the registered BSP/driver-specific handler.
  * It can be used to initialize a GPIO object/pin/port. It should
  * be called once for each pin, before all non-setup operations.
  *
  * @param[in] base The pointer to a GPIO object
  *
  * @retval RTEMS_SUCCESSFUL GPIO object initialized successfully.
  * @retval RTEMS_UNSATISFIED Could not initialize a GPIO object.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_init(
    rtems_gpio *base
);

/**
  * @brief Deinitialize a GPIO object
  *
  * This function calls the registered BSP/driver-specific handler.
  * It can be used to deinitialize a GPIO object/pin/port. It 
  * should be called once for each pin, before all non-setup 
  * operations.
  *
  * @param[in] base The pointer to a GPIO object
  *
  * @retval RTEMS_SUCCESSFUL GPIO object deinitialized successfully.
  * @retval RTEMS_UNSATISFIED Could not deinitialize a GPIO object.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_deinit(
    rtems_gpio *base
);

/**
  * @brief Sets the pin mode of a pin.
  *
  * This function calls the registered BSP/driver-specific handler.
  * It can be used to set the pin mode for a pin. 
  *
  * @param[in] base The GPIO object to be configured.
  * @param mode The pin mode from the enumeration rtems_gpio_pin_mode
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not set the pin mode.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_set_pin_mode(
    rtems_gpio *base, 
    rtems_gpio_pin_mode mode
);

/**
  * @brief Sets the pin's pull resistor configuration.
  *
  * This function calls the registered BSP/driver-specific handler.
  * It can be used to set the pull resistor mode for a pin.
  *
  * @param[in] base The GPIO object to be configured.
  * @param mode The pull mode from the enumeration rtems_gpio_pull
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not set the pull resistor mode.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_set_pull(
    rtems_gpio *base, 
    rtems_gpio_pull pull
);

/**
  * @brief Configure interrupt on a GPIO pin.
  *
  * This function calls the registered BSP/driver-specific handler.
  * It can be used to set up a pin for interrupt mode.
  *
  * @note This only configures the interrupt but not enable it. Use
  *       rtems_gpio_enable_interrupt() to actually enable interrupt.
  *
  * @param[in] base The GPIO object to be configured.
  * @param isr The pointer to the ISR to be attached to this pin.
  * @param[in] arg The pointer to the argument of the ISR.
  * @param trig The trigger mode
  * @param pull The pull resistor mode
  *
  * @retval RTEMS_SUCCESSFUL Interrupt configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure interrupt.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_configure_interrupt(
    rtems_gpio *base, 
    rtems_gpio_isr isr,
    void *arg,
    rtems_gpio_interrupt_trig trig,
    rtems_gpio_pull pull
);

/**
  * @brief Remove interrupt settings for a pin.
  *
  * This function calls the registered BSP/driver-specific handler.
  * It can be used to remove interrupt configuration of a pin like
  * ISR, ISR argument, pin mode, etc.
  *
  * @note This only removes the interrupt but not disable it. Use
  *       rtems_gpio_disable_interrupt() to actually disable 
  *       interrupt.
  *
  * @param[in] base The GPIO object to be configured.
  *
  * @retval RTEMS_SUCCESSFUL Interrupt removed successfully.
  * @retval RTEMS_UNSATISFIED Could not remove interrupt.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_remove_interrupt(
    rtems_gpio *base
);

/**
  * @brief Enable interrupt on a GPIO pin.
  *
  * This function calls the registered BSP/driver-specific handler.
  *
  * @note This function only enables the interrupt (e.g. the vector)
  *       but not configure the pin. Use 
  *       rtems_gpio_configure_interrupt() for pin configuration.
  *
  * @param[in] base The GPIO object to be configured.
  *
  * @retval RTEMS_SUCCESSFUL Interrupt enabled successfully.
  * @retval RTEMS_UNSATISFIED Could not enable interrupt.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio *base
);

/**
  * @brief Disable interrupt on a GPIO pin.
  *
  * This function calls the registered BSP/driver-specific handler.
  *
  * @note This function only disables the interrupt (e.g. the vector)
  *       but not remove the pin's configurations. Use 
  *       rtems_gpio_remove_interrupt() for the latter purpose.
  *
  * @param[in] base The GPIO object to be configured.
  *
  * @retval RTEMS_SUCCESSFUL Interrupt disabled successfully.
  * @retval RTEMS_UNSATISFIED Could not disable interrupt.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio *base
);

/**
  * @brief Writes a digital value to a pin. 
  *
  * This function calls the registered BSP/driver-specific handler.
  *
  * @param[in] base The GPIO object that has information about the
  *                  pin.
  * @param value The state to be written to the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_write(
    rtems_gpio *base, 
    rtems_gpio_pin_state value
);

/**
  * @brief Reads the digital value of a pin. 
  *
  * This function calls the registered BSP/driver-specific handler.
  *
  * @param[in] base The GPIO object that has information about the
  *                  pin.
  *
  * @param[out] value The state of the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin succesfully read.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_read(
    rtems_gpio *base, 
    rtems_gpio_pin_state *value
);

/**
  * @brief Toggles the state of a GPIO pin.
  *
  * This function calls the registered BSP/driver-specific handler.
  *
  * @param[in] base The GPIO object that has information about the
  *                  pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully toggled.
  * @retval @see BSP/driver-specific function for other return codes.
  */
extern rtems_status_code rtems_gpio_toggle(
    rtems_gpio *base
);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_GPIO2_H */
