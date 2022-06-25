/*
 *  Copyright (c) 2022 Duc Doan <dtbpkmte at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
 
#ifndef LIBBSP_ARM_STM32F4_BSP_GPIO
#define LIBBSP_ARM_STM32F4_BSP_GPIO

#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_ll_gpio.h>
#include <bsp/gpio2.h>

typedef struct {
    rtems_gpio_ctrl_t base;
    GPIO_TypeDef *port;
    bool is_registered;
} stm32f4_gpio_ctrl_t;

/**
  * @brief STM32F4-specific interrupt configuration structure.
  */
typedef struct {
    rtems_gpio_interrupt_config_t base;
    uint32_t subpriority;  /* Subpriority level of the IRQ */
    bool is_event_mode;     /* Set to true if using Event mode */
} stm32f4_gpio_interrupt_config_t;

/**
  * @note This structure should be initialized to 0 
  *
  * @brief This structure holds configuration options for STM32F4-specific
  *        GPIO. The 2 specific modes are Event and Alternate modes. Select
  *        a mode by setting the correct field (is_event_mode or is_alternate_mode)
  *        to 1.
  */
typedef struct {
    rtems_gpio_config_t base;   /* Base GPIO config object */

    uint32_t speed;             /* Speed of the pin. Must be specified */

    uint32_t alternate_mode;    /* Open drain or Push-pull mode
                                   Use if the pin is in Alternate mode */
    uint32_t alternate_fn;      /* The alternate function of the pin
                                   Use if the pin is in Alternate mode */
} stm32f4_gpio_config_t;

rtems_status_code stm32f4_gpio_get_ctrl(GPIO_TypeDef *port, rtems_gpio_ctrl_t **out);

#endif /* LIBBSP_ARM_STM32F4_BSP_GPIO */
