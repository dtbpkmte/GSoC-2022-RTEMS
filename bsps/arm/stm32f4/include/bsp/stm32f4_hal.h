#ifndef LIBBSP_ARM_STM32F4_BSP_STM32F4HAL_H
#define LIBBSP_ARM_STM32F4_BSP_STM32F4HAL_H

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void stm32f4_clk_enable();
void stm32f4_clk_disable();

#ifdef __cplusplus
}
#endif

#endif
