/**
 * @file
 * 
 * @brief DIR Shell Command Implmentation
 */

/*
 * Copyright (c) 2001 Fernando Ruiz Casas <fruizcasas@gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <rtems/shell.h>
#include "internal.h"

rtems_shell_alias_t rtems_shell_DIR_Alias = {
  "ls",                      /* command */
  "dir"                      /* alias */
};
