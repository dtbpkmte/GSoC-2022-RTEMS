/*
 * Copyright (c) 2012 Sebastian Huber.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <rtems.h>

#include <bsp/bootcard.h>

void bsp_reset(void)
{
  rtems_interrupt_level level;

  (void) level;
  rtems_interrupt_disable(level);

  while (1);
}
