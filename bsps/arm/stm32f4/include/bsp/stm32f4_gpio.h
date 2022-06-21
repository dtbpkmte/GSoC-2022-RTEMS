#ifndef LIBBSP_STM32F4_GPIO
#define LIBBSP_STM32F4_GPIO

#include <stm32f4xx.h>
#include <bsp/gpio2.h>

struct rtems_gpio_t {
    GPIO_TypeDef *port;
    uint16_t pin;
};

/**
  * This structure should be initialized to 0 
  *
  * This structure holds configuration options for STM32F4-specific
  * GPIO. The 2 specific modes are Event and Alternate modes. Select
  * a mode by setting the correct field (is_event_mode or is_alternate_mode)
  * to 1.
  */
struct rtems_gpio_config_t {
    uint32_t speed;             /* Speed of the pin. Must be specified */
                                   
    uint32_t interrupt_mode;    /* The type of interrupt trigger of the pin 
                                   Use if the pin is in Interrupt mode */
                                   
    uint32_t is_event_mode;     /* Sets to 1 if the pin is in event mode,
                                   0 other wise.
                                   Use if the pin is in Event mode */
    uint32_t event_mode;        /* The type of event trigger of the pin 
                                   Use if the pin is in Event mode */
    
    uint32_t is_alternate_mode; /* Sets to 1 if the pin is in Alternate mode,
                                   0 other wise.
                                   Use if the pin is in Alternate mode */
    uint32_t alternate_mode;    /* Open drain or Push-pull mode
                                   Use if the pin is in Alternate mode */
    uint32_t alternate_fn;      /* The alternate function of the pin
                                   Use if the pin is in Alternate mode */
};

#endif
