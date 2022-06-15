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

#ifndef LIBBSP_SHARED_GPIO2_H
#define LIBBSP_SHARED_GPIO2_H

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
    RTEMS_GPIO_PIN_SET = 0,
    RTEMS_GPIO_PIN_RESET
} rtems_gpio_pin_state;

/**
  * @brief Opaque type for a GPIO object.
  *        To be implemented by BSP.
  * 
  * @details This could represent the unit that owns GPIO pins. 
  *          For example, it would be a port for ARM Cortex-M.
  */
typedef struct rtems_gpio_t rtems_gpio_t;

/**
  * @brief Opaque type for a GPIO pin object.
  *        To be implemented by BSP.
  */
typedef struct rtems_gpio_pin_t rtems_gpio_pin_t;

/**
  * @brief Opaque type for configuration of a GPIO object.
  *        To be implemented by BSP.
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
extern rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_config_t *config);

/**
  * @brief Writes a digital value to a pin/pins.
  *
  * @param[in] gpiox The GPIO object that owns the pin(s).
  * @param[in] pin The GPIO pin number or pin mask to be written.
  * @param[in] value The state to be written to the pin(s).
  *
  * @retval RTEMS_SUCCESSFUL Pin successfully written.
  * @retval RTEMS_UNSATISFIED Could not write to pin(s).
  */
extern rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin, rtems_gpio_pin_state value);

/**
  * @brief Reads the digital value of a pin/pins.
  *
  * @param[in] gpiox The GPIO object that owns the pin(s).
  * @param[in] pin The GPIO pin(s) to be read.
  *
  * @retval The state of the pin(s).
  */
extern rtems_gpio_pin_state rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin);

/**
  * @brief Toggles the state of a GPIO pin/pins.
  *
  * @param[in] gpiox The GPIO object that owns the pin(s).
  * @param[in] pin The GPIO pin(s) to be toggled.
  *
  * @retval RTEMS_SUCCESSFUL Pin(s) successfully toggled.
  * @retval RTEMS_UNSATISFIED Could not toggle pin(s).
  */
extern rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_SHARED_GPIO2_H */
