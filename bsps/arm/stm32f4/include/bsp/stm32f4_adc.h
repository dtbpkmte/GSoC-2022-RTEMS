#include <bsp/adc.h>
#include <bsp/adc.h>
#include <bsp/stm32f4_gpio.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_adc.h>

#define STM32F4_ADC_DEFAULT_RESOLUTION          10 //bits
#define STM32F4_ADC_DEFAULT_ALIGNMENT           RTEMS_ADC_ALIGN_RIGHT
#define STM32F4_ADC_DEFAULT_SAMPLINGTIME       LL_ADC_SAMPLINGTIME_3CYCLES

const rtems_adc_handlers *stm32f4_get_adc_handlers(
    void
);

/* Helpers */
#define STM32F4_GET_ADC_NUMBER(ADCx) \
    ( (uintptr_t) ( ADCx ) == (uintptr_t) ADC1 ? 1 : \
      (uintptr_t) ( ADCx ) == (uintptr_t) ADC2 ? 2 : \
      (uintptr_t) ( ADCx ) == (uintptr_t) ADC3 ? 3 : \
                                                 0 )
#define STM32F4_GET_ADCx_FROM_NUMBER(num)       (\
    ( num ) == 1                 ? ADC1 :        \
    ( num ) == 2 && NUM_ADC >= 2 ? ADC2 :        \
    ( num ) == 3 && NUM_ADC == 3 ? ADC3 :        \
                                   NULL)

ADC_TypeDef *stm32f4_get_ADCx(
    GPIO_TypeDef *gpiox
);

rtems_status_code stm32f4_get_LL_ADC_CHANNEL(
    stm32f4_gpio *gpio,
    uint32_t *channel
);

bool stm32f4_is_adc_pin(
    stm32f4_gpio *gpio
);
/***/
/**
  * @brief Perform initialization for STM32F4 ADC manager
  *
  * This function should be called in bspstarthook under
  * if ADC is enabled.
  */
extern rtems_status_code stm32f4_adc_start(
    void
);

/**
  * @brief Performs initialization for an ADC pin
  */
rtems_status_code stm32f4_adc_init(
    stm32f4_gpio *gpio
);

rtems_status_code stm32f4_adc_read_raw(
    rtems_gpio *base,
    uint32_t *result,
    uint32_t timeout
);

rtems_status_code stm32f4_adc_start_read_raw_nb(
    rtems_gpio *base
);

rtems_adc_status stm32f4_adc_read_raw_nb(
    rtems_gpio *base,
    uint32_t *result
);

rtems_status_code stm32f4_adc_set_resolution(
    rtems_gpio *base,
    unsigned int bits
);

rtems_status_code stm32f4_adc_set_alignment(
    rtems_gpio *base,
    rtems_adc_align align
);

rtems_status_code stm32f4_adc_configure_interrupt(
    rtems_gpio *base,
    rtems_adc_isr isr,
    void *arg
);

rtems_status_code stm32f4_adc_remove_interrupt(
    rtems_gpio *base
);

rtems_status_code stm32f4_adc_enable_interrupt(
    rtems_gpio *base
);

rtems_status_code stm32f4_adc_disable_interrupt(
    rtems_gpio *base
);
