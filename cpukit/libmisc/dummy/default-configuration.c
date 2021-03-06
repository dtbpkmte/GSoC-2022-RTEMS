/* SPDX-License-Identifier: BSD-2-Clause */

/*
 *  Default configuration file
 *
 *  COPYRIGHT (c) 1989-2008.
 *  On-Line Applications Research Corporation (OAR).
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <string.h>

#include <rtems.h>

int main( int argc, char **argv );

static void Init( rtems_task_argument arg )
{
  const char *boot_cmdline = *((const char **) arg);
  char       *cmdline = NULL;
  int         argc = 0;
  char      **argv = NULL;
  int         result;

  if ( boot_cmdline != NULL ) {
    size_t n = strlen( boot_cmdline ) + 1;

    cmdline = malloc( n );
    if ( cmdline != NULL ) {
      char* command;

      memcpy( cmdline, boot_cmdline, n);

      command = cmdline;

      /*
       * Break the line up into arguments with "" being ignored.
       */
      while ( true ) {
        command = strtok( command, " \t\r\n" );
        if ( command == NULL )
          break;

        ++argc;
        command = '\0';
      }

      /*
       * If there are arguments, allocate enough memory for the argv
       * array to be passed into main().
       *
       * NOTE: If argc is 0, then argv will be NULL.
       */
      argv = calloc( argc, sizeof( *argv ) );
      if ( argv != NULL ) {
        int a;

        command = cmdline;
        argv[ 0 ] = command;

        for ( a = 1; a < argc; ++a ) {
          command += strlen( command ) + 1;
          argv[ a ] = command;
        }
      } else {
        argc = 0;
      }
    }
  }

  result = main( argc, argv );

  free( argv );
  free( cmdline );

  exit( result );
}

/* configuration information */

/* This is enough to get a basic main() up. */
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_STACK_CHECKER_ENABLED

/* on smaller architectures lower the number or resources */
#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_MAXIMUM_USER_EXTENSIONS 8
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 32

/* Include basic device drivers needed to call delays */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK

#define CONFIGURE_MAXIMUM_PROCESSORS CPU_MAXIMUM_PROCESSORS

#define CONFIGURE_DISABLE_BSP_SETTINGS

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
