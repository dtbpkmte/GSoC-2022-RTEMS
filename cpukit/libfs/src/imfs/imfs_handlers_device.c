/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup IMFS
 *
 * @brief Device Operations Table
 */

/*
 *  COPYRIGHT (c) 1989-1999.
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

#include <rtems/imfs.h>

static int IMFS_stat_device(
  const rtems_filesystem_location_info_t *loc,
  struct stat *buf
)
{
  const IMFS_device_t *device = loc->node_access;

  buf->st_rdev = rtems_filesystem_make_dev_t( device->major, device->minor );

  return IMFS_stat( loc, buf );
}

static const rtems_filesystem_file_handlers_r IMFS_device_handlers = {
  .open_h = device_open,
  .close_h = device_close,
  .read_h = device_read,
  .write_h = device_write,
  .ioctl_h = device_ioctl,
  .lseek_h = rtems_filesystem_default_lseek_file,
  .fstat_h = IMFS_stat_device,
  .ftruncate_h = device_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .mmap_h = rtems_filesystem_default_mmap,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

static IMFS_jnode_t *IMFS_node_initialize_device(
  IMFS_jnode_t *node,
  void *arg
)
{
  IMFS_device_t *device = (IMFS_device_t *) node;
  dev_t *dev = arg;

  rtems_filesystem_split_dev_t( *dev, device->major, device->minor );

  return node;
}

const IMFS_mknod_control IMFS_mknod_control_device = {
  {
    .handlers = &IMFS_device_handlers,
    .node_initialize = IMFS_node_initialize_device,
    .node_remove = IMFS_node_remove_default,
    .node_destroy = IMFS_node_destroy_default
  },
  .node_size = sizeof( IMFS_device_t )
};
