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

/**
  * @brief Structure for a GPIO object. It holds information
  *        like port number and pin number/pin mask.
  *        To be implemented by BSP.
  * 
  */
typedef struct {
    void *port;                 /* Pointer to the GPIO port */
    uint32_t pin_mask;          /* The pin number or pin mask */
    bool is_expander;           /* If the GPIO is an expander, set to true.
                                   Else false. */
    uint32_t expander_id;       /* This field specifies which GPIO expander is
                                   in use. In case of using multiple expanders,
                                   this field is necessary to handle each. */
    void *bsp;                  /* Pointer to BSP-specific data */
} rtems_gpio_t;

/**
  * @brief Opaque type for configuration of a GPIO object.
  *        Specific to each BSP. To be implemented by BSP.
  */
typedef struct {
    rtems_gpio_pin_mode mode;   /* Pin mode */
    rtems_gpio_pull pull;       /* Pull resistor configuration */
    void *bsp;                  /* Pointer to BSP-specific config */
} rtems_gpio_config_t;

typedef struct {
    rtems_gpio_interrupt interrupt_mode;        /* Interrupt trigger mode */
    uint32_t interrupt_number;                  /* Interrupt number */
    uint32_t priority;                          /* Interrupt priority */
    void *bsp;                                  /* Pointer to BSP-specific config */
    void (*handler) (void *arg);                /* Pointer to the IRQ handler */
    void *arg;                                  /* Pointer to the arguments of IRQ handler */
} rtems_gpio_interrupt_config_t;

/** @} */

/**
  * @name GPIO Functions
  *
  * @{
  */

/**
  * @brief Initialization for GPIO. To be implemented by User Application.
  *        This function is called in bsp_start(), before Init task. It can
  *        be used, for example, to enable the GPIO clocks for STM32F4
  *        family.
  *
  * @retval RTEMS_SUCCESSFUL GPIO successfully initialized.
  * @retval RTEMS_UNSATISFIED Could not initialize GPIO object.
  */
extern rtems_status_code rtems_gpio_initialize(void);

/**
  * @brief Configures a GPIO object.
  *        To be implemented by BSP.
  *
  * @param[in] gpiox The GPIO object to be configured.
  * @param[in] config The GPIO configuration object.
  *
  * @retval RTEMS_SUCCESSFUL GPIO configured successfully.
  * @retval RTEMS_UNSATISFIED Could not configure GPIO object.
  */
extern rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_config_t *config);

extern rtems_status_code rtems_gpio_set_pin_mode(rtems_gpio_t *gpiox, rtems_gpio_pin_mode mode);

extern rtems_status_code rtems_gpio_set_pull(rtems_gpio_t *gpiox, rtems_gpio_pull pull);

extern rtems_status_code rtems_gpio_configure_interrupt(rtems_gpio_t *gpiox, rtems_gpio_interrupt_config_t *int_conf);

/**
  * @brief Writes a digital value to a pin. It selects between the
  *        default function (built-in GPIO) and GPIO expander
  *        function based on is_expander field of gpiox.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  * @param[in] value The state to be written to the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval * @see rtems_gpio_write_pin_default().
  * @retval * @see rtems_gpio_write_pin_ex().
  */
extern rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state value);

/**
  * @brief Writes a digital value to a pin using the microcontroller's
  *        built-in GPIO.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  * @param[in] value The state to be written to the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval RTEMS_UNSATISFIED Could not write to pin.
  */
extern rtems_status_code rtems_gpio_write_pin_default(rtems_gpio_t *gpiox, rtems_gpio_pin_state value);

/**
  * @brief Writes a digital value to a pin using GPIO expander.
  *        If using GPIO expander, the BSP should implement 
  *        this function. 
  *        In case of multiple expanders, this function should
  *        handle all of them. The field expander_id of gpiox
  *        might be helpful to differentiate among different
  *        expanders.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  * @param[in] value The state to be written to the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval If operation fails, return the status code from
  *         the internal function that performs this operation.
  *         If no such function exists, return RTEMS_UNSATISFIED.
  */
extern rtems_status_code rtems_gpio_write_pin_ex(rtems_gpio_t *gpiox, rtems_gpio_pin_state value);

/**
  * @brief Reads the digital value of a pin. It selects between the
  *        default function (built-in GPIO) and GPIO expander
  *        function based on is_expander field of gpiox.

  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  *
  * @param[out] value The state of the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin succesfully read.
  * @retval * @see rtems_gpio_read_pin_default().
  * @retval * @see rtems_gpio_read_pin_ex().
  */
extern rtems_status_code rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value);

/**
  * @brief Reads the digital value of a pin using the microcontroller's
  *        built-in GPIO.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  *
  * @param[out] value The state of the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin succesfully read.
  * @retval RTEMS_UNSATISFIED Could not read from pin.
  */
extern rtems_status_code rtems_gpio_read_pin_default(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value);

/**
  * @brief Reads the digital value of a pin using GPIO expander.
  *        If using GPIO expander, the BSP should implement 
  *        this function.
  *        In case of multiple expanders, this function should
  *        handle all of them. The field expander_id of gpiox
  *        might be helpful to differentiate among different
  *        expanders.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  *
  * @param[out] value The state of the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin succesfully read.
  * @retval If operation fails, return the status code from
  *         the internal function that performs this operation.
  *         If no such function exists, return RTEMS_UNSATISFIED.
  */
extern rtems_status_code rtems_gpio_read_pin_ex(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value);


/**
  * @brief Toggles the state of a GPIO pin. It selects between the
  *        default function (built-in GPIO) and GPIO expander
  *        function based on is_expander field of gpiox.
  *
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully toggled.
  * @retval * @see rtems_gpio_toggle_pin_default().
  * @retval * @see rtems_gpio_toggle_pin_ex().
  */
extern rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox);

/**
  * @brief Toggles the state of a GPIO pin using microcontroller's
  *        built-in GPIO.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully toggled.
  * @retval RTEMS_UNSATISFIED Could not toggle pin.
  */
extern rtems_status_code rtems_gpio_toggle_pin_default(rtems_gpio_t *gpiox);

/**
  * @brief Toggles the state of a GPIO pin using GPIO expander.
  *        If using GPIO expander, the BSP should implement 
  *        this function.
  *        In case of multiple expanders, this function should
  *        handle all of them. The field expander_id of gpiox
  *        might be helpful to differentiate among different
  *        expanders.
  *
  * @param[in] gpiox The GPIO object that has information about the
  *                  pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully toggled.
  * @retval If operation fails, return the status code from
  *         the internal function that performs this operation.
  *         If no such function exists, return RTEMS_UNSATISFIED.
  */
extern rtems_status_code rtems_gpio_toggle_pin_ex(rtems_gpio_t *gpiox);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_GPIO2_H */
