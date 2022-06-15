#include <bsp.h>
#include <rtems.h>
#include <bsp/gpio2.h>
#include <stm32f4xx_hal.h>

struct rtems_gpio_t {
    GPIO_TypeDef *port;
};

struct rtems_gpio_pin_t {
    uint16_t pin_mask;  
};

struct rtems_gpio_config_t {
    GPIO_InitTypeDef *config;
};

__attribute__((weak)) rtems_status_code rtems_gpio_initialize(void) {
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_configure(rtems_gpio_t *gpiox, rtems_gpio_config_t *config) {
    HAL_GPIO_Init(gpiox->port, config->config);
    return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_write_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin, rtems_gpio_pin_state value) {
    HAL_GPIO_WritePin(gpiox->port, pin->pin_mask, value);
    return RTEMS_SUCCESSFUL;
}

rtems_gpio_pin_state rtems_gpio_read_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin) {
    return HAL_GPIO_ReadPin(gpiox->port, pin->pin_mask);
}

rtems_status_code rtems_gpio_toggle_pin(rtems_gpio_t *gpiox, rtems_gpio_pin_t *pin) {
    HAL_GPIO_TogglePin(gpiox->port, pin->pin_mask);
    return RTEMS_SUCCESSFUL;
}
