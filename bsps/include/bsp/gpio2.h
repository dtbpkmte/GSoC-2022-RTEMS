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
} rtems_gpio_pin_state;

/**
  * @brief GPIO pin modes. If a BSP has its other specific modes, 
  *        use RTEMS_GPIO_PINMODE_BSP_SPECIFIC and specify more 
  *        configuration details in the rtems_gpio_config_t.
  */
typedef enum {
    RTEMS_GPIO_PINMODE_OUTPUT_PP,
    RTEMS_GPIO_PINMODE_OUTPUT_OD,
    RTEMS_GPIO_PINMODE_INPUT,
    RTEMS_GPIO_PINMODE_ANALOG,
    RTEMS_GPIO_PINMODE_BSP_SPECIFIC
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
  * @brief Interrupt modes enumeration
  */
typedef enum {
    RTEMS_GPIO_INT_MODE_NONE = 0,
    RTEMS_GPIO_INT_MODE_FALLING,
    RTEMS_GPIO_INT_MODE_RISING,
    RTEMS_GPIO_INT_MODE_BOTH_EDGES
} rtems_gpio_interrupt;

typedef struct rtems_gpio_handlers rtems_gpio_handlers_t;
typedef struct rtems_gpio_ctrl rtems_gpio_ctrl_t;
typedef struct rtems_gpio_config rtems_gpio_config_t;
typedef struct rtems_gpio_interrupt_config rtems_gpio_interrupt_config_t;

/**
  * @brief Structure holding a set of GPIO handlers.
  */
struct rtems_gpio_handlers {
    rtems_status_code (*initialize)(rtems_gpio_ctrl_t *);

    rtems_status_code (*deinitialize)(rtems_gpio_ctrl_t *);

    rtems_status_code (*configure)(rtems_gpio_ctrl_t *, void *, rtems_gpio_config_t *);

    rtems_status_code (*configure_interrupt)(rtems_gpio_ctrl_t *, void *, rtems_gpio_interrupt_config_t *);

    rtems_status_code (*enable_interrupt)(rtems_gpio_ctrl_t *, void *, rtems_gpio_interrupt_config_t *);

    rtems_status_code (*disable_interrupt)(rtems_gpio_ctrl_t *, void *, rtems_gpio_interrupt_config_t *);

    rtems_status_code (*set_pin_mode)(rtems_gpio_ctrl_t *, void *, rtems_gpio_pin_mode);

    rtems_status_code (*set_pull)(rtems_gpio_ctrl_t *, void *, rtems_gpio_pull);

    rtems_status_code (*read)(rtems_gpio_ctrl_t *, void *, rtems_gpio_pin_state *);

    rtems_status_code (*write)(rtems_gpio_ctrl_t *, void *, rtems_gpio_pin_state);

    rtems_status_code (*toggle)(rtems_gpio_ctrl_t *, void *);

};

/**
  * @brief Structure for a GPIO object. It holds information
  *        like port number and pin number/pin mask.
  * 
  */
struct rtems_gpio_ctrl {
    const rtems_gpio_handlers_t *handlers;
};

/**
  * @brief Structure for configuration of GPIO pins.
  *
  * It contains the common configuration of GPIO pins.
  */
struct rtems_gpio_config {
    rtems_gpio_pin_mode mode;   /* Pin mode */
    rtems_gpio_pull pull;       /* Pull resistor configuration */
};

/**
  * @brief Structure for configuration of an interrupt pin.
  */
struct rtems_gpio_interrupt_config {
    rtems_gpio_interrupt interrupt_mode;        /* Interrupt trigger mode */
    uint32_t interrupt_number;                  /* Interrupt number */
    uint32_t priority;                          /* Interrupt priority */
    void *bsp;                                  /* Pointer to BSP-specific config */
    void (*handler) (void *arg);                /* Pointer to the IRQ handler */
    void *arg;                                  /* Pointer to the arguments of IRQ handler */
};


/** @} */

/**
  * @name GPIO Functions
  *
  * @{
  */

extern void rtems_gpio_register(
    rtems_gpio_ctrl_t *base,
    const rtems_gpio_handlers_t *handlers
);

/**
  * @brief Initialization for GPIO. To be implemented by User Application.
  *        This function is called in bsp_start(), before Init task. It can
  *        be used, for example, to enable the GPIO clocks for STM32F4
  *        family.
  *
  * @retval RTEMS_SUCCESSFUL GPIO successfully initialized.
  * @retval RTEMS_UNSATISFIED Could not initialize GPIO object.
  */
extern rtems_status_code rtems_gpio_initialize(
    rtems_gpio_ctrl_t *base
);

extern rtems_status_code rtems_gpio_deinitialize(
    rtems_gpio_ctrl_t *base
);

/**
  * @brief Configures a GPIO object.
  *        To be implemented by BSP.
  *
  * @param[in] base The GPIO object to be configured.
  * @param[in] config The GPIO configuration object.
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure GPIO object.
  */
extern rtems_status_code rtems_gpio_configure(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_config_t *config
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
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_mode mode
);

/**
  * @brief Sets the pin's pull resistor configuration.
  *        To be implemented by BSP.
  *
  * @param[in] base The GPIO object to be configured.
  * @param mode The pull mode from the enumeration rtems_gpio_pull
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure GPIO object.
  */
extern rtems_status_code rtems_gpio_set_pull(
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pull pull
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
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_interrupt_config_t *int_conf
);

extern rtems_status_code rtems_gpio_enable_interrupt(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_interrupt_config_t *int_conf
);

extern rtems_status_code rtems_gpio_disable_interrupt(
    rtems_gpio_ctrl_t *base,
    void *pin,
    rtems_gpio_interrupt_config_t *int_conf
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
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_state value
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
    rtems_gpio_ctrl_t *base, 
    void *pin, 
    rtems_gpio_pin_state *value
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
  * @retval * @see rtems_gpio_ctrl_toggle_pin_default().
  * @retval * @see rtems_gpio_ctrl_toggle_pin_ex().
  */
extern rtems_status_code rtems_gpio_ctrl_toggle(
    rtems_gpio_ctrl_t *base, 
    void *pin
);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_GPIO2_H */
