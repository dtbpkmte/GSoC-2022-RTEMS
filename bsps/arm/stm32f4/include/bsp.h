/**
 * @file
 * @ingroup RTEMSBSPsARMSTM32F4
 * @brief Global BSP definitions.
 */

/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_BSP_H
#define LIBBSP_ARM_STM32F4_BSP_H

/**
 * @defgroup RTEMSBSPsARMSTM32F4 STM32F4
 *
 * @ingroup RTEMSBSPsARM
 *
 * @brief STM32F4 Board Support Package.
 *
 * @{
 */

#include <bspopts.h>
#include <bsp/default-initial-extension.h>

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BSP_FEATURE_IRQ_EXTENSION

#define BSP_ARMV7M_IRQ_PRIORITY_DEFAULT (13 << 4)

#define BSP_ARMV7M_SYSTICK_PRIORITY (14 << 4)

#define BSP_ARMV7M_SYSTICK_FREQUENCY STM32F4_HCLK

#ifdef __rtems__
/** 
  * GPIO API requirement. 
  */ 
#define BSP_GPIO_NUM_CONTROLLERS        1
#endif /* __rtems__ */

#ifdef __cplusplus
}
#endif /* __cplusplus */

/** @} */


#endif /* LIBBSP_ARM_STM32F4_BSP_H */
