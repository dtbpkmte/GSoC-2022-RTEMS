/* SPDX-License-Identifier: BSD-2-Clause */

/**
  * @file
  *
  * @ingroup stm32f4_periph_api
  *
  * STM32F4 BSP implementation of Peripheral API.
  */

/*
 * Copyright (C) 2022 Duc Doan (dtbpkmte at gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIBBSP_BSP_ARM_STM32F4_PERIPH_H
#define LIBBSP_BSP_ARM_STM32F4_PERIPH_H

#include <bsp/periph_api.h>

/**
  * @brief STM32F4 BSP's function to get a
  *        peripheral API.
  */
rtems_periph_api *stm32f4_periph_get_api(
    rtems_gpio *pin,
    rtems_periph_api_type type
);

/**
  * @brief STM32F4 BSP's function to remove a
  *        peripheral API assigned to a pin.
  */
rtems_status_code stm32f4_periph_remove_api(
    rtems_gpio *pin
);

#endif /* LIBBSP_BSP_ARM_STM32F4_PERIPH_H */
