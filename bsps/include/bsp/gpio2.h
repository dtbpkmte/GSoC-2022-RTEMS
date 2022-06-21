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
    RTEMS_GPIO_PINMODE_INTERRUPT,
    RTEMS_GPIO_PINMODE_BSP_SPECIFIC
} rtems_gpio_pin_mode;

/**
  * @brief GPIO pull register configuration. Defines pull-up or 
  *        pull-down activation.
  */
typedef enum {
    RTEMS_GPIO_NOPULL,
    RTEMS_GPIO_PULLUP,
    RTEMS_GPIO_PULLDOWN
} rtems_gpio_pull;

/**
  * @brief Interrupt modes
  */
typedef enum {
    NONE = 0,
    FALLING,
    RISING,
    BOTH_EDGES
} rtems_gpio_interrupt;

/**
  * @brief Opaque type for a GPIO object. It holds information
  *        like port number and pin number.
  *        To be implemented by BSP.
  * 
  */
typedef struct rtems_gpio_t rtems_gpio_t;

/**
  * @brief Opaque type for configuration of a GPIO object.
  *        Specific to each BSP. To be implemented by BSP.
  */
typedef struct rtems_gpio_config_t rtems_gpio_config_t;

/** @} */

/**
  * @name GPIO Functions
  *
  * @{
  */

/**
  * @brief Initialization for GPIO. To be implemented by User Application.
  *
  * @retval RTEMS_SUCCESSFUL GPIO successfully initialized.
  * @retval RTEMS_UNSATISFIED Could not initialize GPIO object.
  */
rtems_status_code rtems_gpio_initialize(void);

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
extern rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_pin_mode mode, rtems_gpio_pull pull, rtems_gpio_config_t *config);

/**
  * @brief Writes a digital value to a pin.
  *
  * @param[in] gpiox The GPIO object that owns the pin.
  * @param[in] value The state to be written to the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval RTEMS_UNSATISFIED Could not write to pin.
  */
extern rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state value);

/**
  * @brief Reads the digital value of a pin.
  *
  * @param[in] gpiox The GPIO object that owns the pin.
  *
  * @param[out] value The state of the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin succesfully read.
  * @retval RTEMS_UNSATISFIED Could not read pin.
  */
extern rtems_status_code rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_state *value);

/**
  * @brief Toggles the state of a GPIO pin.
  *
  * @param[in] gpiox The GPIO object that owns the pin.
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully toggled.
  * @retval RTEMS_UNSATISFIED Could not toggle pin.
  */
extern rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_BSP_GPIO2_H */
