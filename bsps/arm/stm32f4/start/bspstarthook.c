/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp.h>
#include <bsp/start.h>

void BSP_START_TEXT_SECTION bsp_start_hook_0(void)
{
  /* Do nothing */
}

void BSP_START_TEXT_SECTION bsp_start_hook_1(void)
{
  bsp_start_copy_sections();
  bsp_start_clear_bss();

  /* At this point we can use objects outside the .start section */
  
#ifdef __rtems__

#if STM32F4_ENABLE_GENERIC_GPIO == 1
  bsp_gpio_register_controllers();
#endif /* STM32F4_ENABLE_GENERIC_GPIO */
#if BSP_ENABLE_ADC == 1
    stm32f4_adc_start();
#endif /* BSP_ENABLE_ADC */

#endif /* __rtems__ */
}
