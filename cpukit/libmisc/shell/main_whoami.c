/**
 * @file
 * 
 * @brief WHOAMI Shell Command Implmentation
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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <pwd.h>

#include <rtems.h>
#include <rtems/shell.h>
#include "internal.h"

static int rtems_shell_main_whoami(
  int   argc RTEMS_UNUSED,
  char *argv[] RTEMS_UNUSED
)
{
  struct passwd *pwd;

  pwd = getpwuid(geteuid());
  printf( "%s\n", (pwd) ? pwd->pw_name : "nobody");
  return 0;
}

rtems_shell_cmd_t rtems_shell_WHOAMI_Command = {
  "whoami",                                   /* name */
  "show current user",                        /* usage */
  "misc",                                     /* topic */
  rtems_shell_main_whoami,                    /* command */
  NULL,                                       /* alias */
  NULL                                        /* next */
};
