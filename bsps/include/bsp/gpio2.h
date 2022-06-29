/**
  * @file
  *
  * @ingroup rtems_gpio2
  *
  * @brief RTEMS GPIO new API definition.
  */

/*
*  Copyright (c) 2022 Duc Doan <dtbpkmte at gmail.com>
*
*  The license and distribution terms for this file may be
*  found in the file LICENSE in this distribution or at
*  http://www.rtems.org/license/LICENSE.
*/

#ifndef LIBBSP_BSP_GPIO2_H
#define LIBBSP_BSP_GPIO2_H

#include <bsp.h>
#include <rtems.h>

/**
  * 
  */
#ifndef CONFIGURE_GPIO_MAXIMUM_CONTROLLERS 

#ifndef BSP_GPIO_NUM_CONTROLLERS
#define CONFIGURE_GPIO_MAXIMUM_CONTROLLERS 1
#else
#define CONFIGURE_GPIO_MAXIMUM_CONTROLLERS BSP_GPIO_NUM_CONTROLLERS
#endif


#endif
/***********************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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
} rtems_gpio_pin_state_t;

/**
  * @brief GPIO pin modes. If a BSP has its other specific modes, 
  *        use RTEMS_GPIO_PINMODE_BSP_SPECIFIC and specify more 
  *        configuration details in the rtems_gpio_config_t.
  */
typedef enum {
    RTEMS_GPIO_PINMODE_OUTPUT = 0,
    RTEMS_GPIO_PINMODE_OUTPUT_PP = 0,
    RTEMS_GPIO_PINMODE_OUTPUT_OD = 1,
    RTEMS_GPIO_PINMODE_INPUT = 2,
    RTEMS_GPIO_PINMODE_ANALOG = 3,
    RTEMS_GPIO_PINMODE_BSP_SPECIFIC = 4
} rtems_gpio_pin_mode_t;

/**
  * @brief GPIO pull resistor configuration. Defines pull-up or 
  *        pull-down activation.
  */
typedef enum {
    RTEMS_GPIO_NOPULL,
    RTEMS_GPIO_PULLUP,
    RTEMS_GPIO_PULLDOWN
} rtems_gpio_pull_t;

/**
  * @brief Interrupt modes enumeration
  */
typedef enum {
    RTEMS_GPIO_INT_TRIG_NONE = 0,
    RTEMS_GPIO_INT_TRIG_FALLING,
    RTEMS_GPIO_INT_TRIG_RISING,
    RTEMS_GPIO_INT_TRIG_BOTH_EDGES,
    RTEMS_GPIO_INT_TRIG_LOW,
    RTEMS_GPIO_INT_TRIG_HIGH
} rtems_gpio_interrupt_trig_t;

typedef struct rtems_gpio_handlers rtems_gpio_handlers_t;
typedef struct rtems_gpio rtems_gpio_t;
typedef void (*rtems_gpio_isr_t)(void *);

/**
  * @brief Structure holding a set of GPIO handlers.
  */
struct rtems_gpio_handlers {
    rtems_status_code (*init)(rtems_gpio_t *);

    rtems_status_code (*deinit)(rtems_gpio_t *);

    rtems_status_code (*set_pin_mode)(rtems_gpio_t *, rtems_gpio_pin_mode_t);

    rtems_status_code (*set_pull)(rtems_gpio_t *, rtems_gpio_pull_t);

    rtems_status_code (*configure_interrupt)(rtems_gpio_t *, rtems_gpio_isr_t, void *, rtems_gpio_interrupt_trig_t, rtems_gpio_pull_t);

    rtems_status_code (*remove_interrupt)(rtems_gpio_t *);

    rtems_status_code (*enable_interrupt)(rtems_gpio_t *);

    rtems_status_code (*disable_interrupt)(rtems_gpio_t *);

    rtems_status_code (*read)(rtems_gpio_t *, rtems_gpio_pin_state_t *);

    rtems_status_code (*write)(rtems_gpio_t *, rtems_gpio_pin_state_t);

    rtems_status_code (*toggle)(rtems_gpio_t *);

};

/**
  * @brief Structure for a GPIO object. It holds information
  *        like port number and pin number/pin mask.
  * 
  */
struct rtems_gpio {
    const rtems_gpio_handlers_t *handlers;
    uint32_t virtual_pin;
};

/** @} */

/**
  * @name GPIO BSP functions
  *
  * BSP needs to implement these.
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
  * @note Warning: this function uses malloc(). When you are
  * done with the GPIO object, call rtems_gpio_destroy() to
  * avoid memory leak.
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
    rtems_gpio_t **out
);

/**
  * @brief Free a GPIO object
  *
  * This function should be called on an GPIO object which is
  * no longer used to avoid memory leak.
  *
  * @param[in] base The pointer to the GPIO object.
  *
  * @retval RTEMS_SUCCESSFUL
  * @retval RTEMS_UNSATISFIED
  */
extern rtems_status_code rtems_gpio_destroy(
    rtems_gpio_t *base
);

/**
  * @brief Registers a GPIO controller with GPIO manager.
  *
  * This function registers the pointer to BSP/driver-specific
  * get_gpio() and destroy_gpio() functions. Those two functions
  * are for creating and destroying GPIO objects.
  *
  * @param get_gpio The pointer to BSP/driver-specific get_gpio()
  * @param destroy_gpio The pointer to BSP/driver-specific 
  *        destroy_gpio()
  * @param pin_count The number of GPIO pins in the controller
  *
  * @retval RTEMS_SUCCESSFUL Controller registered successfully
  * @retval RTEMS_UNSATISFIED Controller cannot be registered
  */
extern rtems_status_code rtems_gpio_register(
    rtems_status_code (*get_gpio)(uint32_t, rtems_gpio_t **),
    rtems_status_code (*destroy_gpio)(rtems_gpio_t *),
    uint32_t pin_count
);

/**
  * @brief Performs setup for GPIO functionality. 
  *
  * This function calls bsp_gpio_register_controllers() and may 
  * perform additional initialization steps for GPIO functionality. 
  * It is called automatically when the BSP starts.
  *
  */
extern void rtems_gpio_begin(
    void
);


extern rtems_status_code rtems_gpio_init(
    rtems_gpio_t *base
);

extern rtems_status_code rtems_gpio_deinit(
    rtems_gpio_t *base
);

/**
  * @brief Sets the pin mode.
  *        To be implemented by BSP.
  *
  * @param[in] base The GPIO object to be configured.
  * @param mode The pin mode from the enumeration rtems_gpio_pin_mode
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure GPIO object.
  */
extern rtems_status_code rtems_gpio_set_pin_mode(
    rtems_gpio_t *base, 
    rtems_gpio_pin_mode_t mode
);

/**
  * @brief Sets the pin's pull resistor configuration.
  *        To be implemented by BSP.
  *
  * @param[in] base The GPIO object to be configured.
  * @param mode The pull mode from the enumeration rtems_gpio_pull_t
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure GPIO object.
  */
extern rtems_status_code rtems_gpio_set_pull(
    rtems_gpio_t *base, 
    rtems_gpio_pull_t pull
);

/**
  * @brief Configure interrupt on a GPIO pin.
  *        To be implemented by BSP.
  *
  * @param[in] base The GPIO object to be configured.
  * @param[in] int_conf The configuration object.
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure GPIO object.
  */
extern rtems_status_code rtems_gpio_configure_interrupt(
    rtems_gpio_t *base, 
    rtems_gpio_isr_t isr,
    void *arg,
    rtems_gpio_interrupt_trig_t trig,
    rtems_gpio_pull_t pull
);

extern rtems_status_code rtems_gpio_remove_interrupt(
    rtems_gpio_t *base
);

extern rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio_t *base
);

extern rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio_t *base
);

/**
  * @brief Writes a digital value to a pin. It selects between the
  *        default function (built-in GPIO) and GPIO expander
  *        function based on is_expander field of base.
  *
  * @param[in] base The GPIO object that has information about the
  *                  pin.
  * @param[in] value The state to be written to the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval * @see rtems_gpio_write_pin_default().
  * @retval * @see rtems_gpio_write_pin_ex().
  */
extern rtems_status_code rtems_gpio_write(
    rtems_gpio_t *base, 
    rtems_gpio_pin_state_t value
);

/**
  * @brief Reads the digital value of a pin. It selects between the
  *        default function (built-in GPIO) and GPIO expander
  *        function based on is_expander field of base.

  *
  * @param[in] base The GPIO object that has information about the
  *                  pin.
  *
  * @param[out] value The state of the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin succesfully read.
  * @retval * @see rtems_gpio_read_pin_default().
  * @retval * @see rtems_gpio_read_pin_ex().
  */
extern rtems_status_code rtems_gpio_read(
    rtems_gpio_t *base, 
    rtems_gpio_pin_state_t *value
);

/**
  * @brief Toggles the state of a GPIO pin. It selects between the
  *        default function (built-in GPIO) and GPIO expander
  *        function based on is_expander field of base.
  *
  *
  * @param[in] base The GPIO object that has information about the
  *                  pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully toggled.
  * @retval * @see rtems_gpio_toggle_pin_default().
  * @retval * @see rtems_gpio_toggle_pin_ex().
  */
extern rtems_status_code rtems_gpio_toggle(
    rtems_gpio_t *base
);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_GPIO2_H */
