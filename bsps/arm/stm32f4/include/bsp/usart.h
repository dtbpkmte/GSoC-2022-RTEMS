/**
 * @file
 * @ingroup stm32f4_usart
 * @brief USART (universal synchronous/asynchronous receiver/transmitter) support.
 */

/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_USART_H
#define LIBBSP_ARM_STM32F4_USART_H

#include <libchip/serial.h>

/**
 * @defgroup stm32f4_usart USART Support
 * @ingroup RTEMSBSPsARMSTM32F4
 * @brief USART Support
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern const console_fns stm32f4_usart_fns;
#ifdef __rtems__
extern console_tbl *stm32f4_default_console_tbl_ptr;
#endif /* __rtems__ */

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_STM32F4_USART_H */
