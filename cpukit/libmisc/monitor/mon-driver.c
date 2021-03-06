/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @brief RTEMS monitor IO (device drivers) support
 *
 * There are 2 "driver" things the monitor knows about.
 *
 *    1. Regular RTEMS drivers.
 *         This is a table indexed by major device number and
 *         containing driver entry points only.
 *
 *    2. Driver name table.
 *         A separate table of names for drivers.
 *         The table converts driver names to a major number
 *         as index into the driver table and a minor number
 *         for an argument to driver.
 *
 *  Drivers are displayed with 'driver' command.
 *  Names are displayed with 'name' command.
 */

/*
 * COPYRIGHT (c) 1989-2022. On-Line Applications Research Corporation (OAR).
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

#include <rtems.h>
#include <rtems/ioimpl.h>
#include <rtems/monitor.h>

#include <stdio.h>
#include <stdlib.h>             /* strtoul() */
#include <inttypes.h>

#define DATACOL 15
#define CONTCOL DATACOL		/* continued col */


void
rtems_monitor_driver_canonical(
    rtems_monitor_driver_t *canonical_driver,
    const void             *driver_void
)
{
    const rtems_driver_address_table *d = (const rtems_driver_address_table *) driver_void;

    rtems_monitor_symbol_canonical_by_value(&canonical_driver->initialization,
                                            (void *) d->initialization_entry);

    rtems_monitor_symbol_canonical_by_value(&canonical_driver->open,
                                            (void *) d->open_entry);
    rtems_monitor_symbol_canonical_by_value(&canonical_driver->close,
                                            (void *) d->close_entry);
    rtems_monitor_symbol_canonical_by_value(&canonical_driver->read,
                                            (void *) d->read_entry);
    rtems_monitor_symbol_canonical_by_value(&canonical_driver->write,
                                            (void *) d->write_entry);
    rtems_monitor_symbol_canonical_by_value(&canonical_driver->control,
                                            (void *) d->control_entry);
}


const void *
rtems_monitor_driver_next(
    void                  *object_info RTEMS_UNUSED,
    rtems_monitor_driver_t *canonical_driver,
    rtems_id              *next_id
)
{
    uint32_t   n = rtems_object_id_get_index(*next_id);

    if (n >= _IO_Number_of_drivers)
        goto failed;

    _Objects_Allocator_lock();

    /*
     * dummy up a fake id and name for this item
     */

    canonical_driver->id = n;
    canonical_driver->name = rtems_build_name('-', '-', '-', '-');

    *next_id += 1;
    return (const void *) (&_IO_Driver_address_table[n]);

failed:
    *next_id = RTEMS_OBJECT_ID_FINAL;
    return 0;
}


void
rtems_monitor_driver_dump_header(
    bool verbose RTEMS_UNUSED
)
{
    fprintf(stdout,"\
  Major      Entry points\n");
/*23456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789
0         1         2         3         4         5         6         7       */
    rtems_monitor_separator();
}

void
rtems_monitor_driver_dump(
    rtems_monitor_driver_t *monitor_driver,
    bool                    verbose
)
{
    uint32_t            length = 0;

    length += fprintf(stdout,"  %" PRId32 "", monitor_driver->id);
    length += rtems_monitor_pad(13, length);
    length += fprintf(stdout,"init: ");
    length += rtems_monitor_symbol_dump(&monitor_driver->initialization, verbose);
    length += fprintf(stdout,";  control: ");
    length += rtems_monitor_symbol_dump(&monitor_driver->control, verbose);
    length += fprintf(stdout,"\n");
    length = 0;

    length += rtems_monitor_pad(13, length);

    length += fprintf(stdout,"open: ");
    length += rtems_monitor_symbol_dump(&monitor_driver->open, verbose);
    length += fprintf(stdout,";  close: ");
    length += rtems_monitor_symbol_dump(&monitor_driver->close, verbose);
    length += fprintf(stdout,"\n");
    length = 0;

    length += rtems_monitor_pad(13, length);

    length += fprintf(stdout,"read: ");
    length += rtems_monitor_symbol_dump(&monitor_driver->read, verbose);
    length += fprintf(stdout,";  write: ");
    length += rtems_monitor_symbol_dump(&monitor_driver->write, verbose);
    length += fprintf(stdout,"\n");
    length = 0;
}
