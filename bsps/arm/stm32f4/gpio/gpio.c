/**
  * @file
  *
  * @ingroup rtems_bsp/arm/stm32f4
  *
  * @brief RTEMS GPIO new API implementation for STM32F4.
  */

#include <bsp.h>
#include <rtems.h>
//#include <bsp/gpio2.h>
#include <bsp/stm32f4_gpio.h>

__attribute__((weak)) rtems_status_code rtems_gpio_initialize(void) {
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_config_t *config) {
    HAL_GPIO_Init((GPIO_TypeDef *) (gpiox->port), (GPIO_InitTypeDef *) (config->config));
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin, rtems_gpio_pin_state value) {
    HAL_GPIO_WritePin((GPIO_TypeDef *) (gpiox->port), pin->pin_mask, value);
    return RTEMS_SUCCESSFUL;
}

rtems_gpio_pin_state rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin) {
    return HAL_GPIO_ReadPin((GPIO_TypeDef *) (gpiox->port), pin->pin_mask);
}

rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin) {
    HAL_GPIO_TogglePin((GPIO_TypeDef *) (gpiox->port), pin->pin_mask);
    return RTEMS_SUCCESSFUL;
}
