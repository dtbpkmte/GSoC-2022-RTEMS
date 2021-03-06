/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup IMFS
 *
 * @brief IMFS Create a New Symbolic Link Node
 */

/*
 *  COPYRIGHT (c) 1989-2009.
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

#include <string.h>

static const IMFS_node_control IMFS_node_control_sym_link;

int IMFS_symlink(
  const rtems_filesystem_location_info_t *parentloc,
  const char *name,
  size_t namelen,
  const char *target
)
{
  IMFS_jnode_t *new_node;

  /*
   *  Create a new link node.
   */
  new_node = IMFS_create_node(
    parentloc,
    &IMFS_node_control_sym_link,
    sizeof( IMFS_sym_link_t ) + strlen( target ) + 1,
    name,
    namelen,
    ( S_IFLNK | ( S_IRWXU | S_IRWXG | S_IRWXO )),
    RTEMS_DECONST( char *, target )
  );

  if ( new_node == NULL ) {
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }

  return 0;
}

ssize_t IMFS_readlink(
  const rtems_filesystem_location_info_t *loc,
  char *buf,
  size_t bufsize
)
{
  IMFS_sym_link_t   *sym_link;
  ssize_t            i;

  sym_link = loc->node_access;

  for( i=0; ((i<bufsize) && (sym_link->name[i] != '\0')); i++ )
    buf[i] = sym_link->name[i];

  return i;
}

static int IMFS_stat_sym_link(
  const rtems_filesystem_location_info_t *loc,
  struct stat *buf
)
{
  const IMFS_sym_link_t *sym_link = loc->node_access;

  buf->st_size = strlen( sym_link->name );

  return IMFS_stat( loc, buf );
}

static const rtems_filesystem_file_handlers_r IMFS_link_handlers = {
  .open_h = rtems_filesystem_default_open,
  .close_h = rtems_filesystem_default_close,
  .read_h = rtems_filesystem_default_read,
  .write_h = rtems_filesystem_default_write,
  .ioctl_h = rtems_filesystem_default_ioctl,
  .lseek_h = rtems_filesystem_default_lseek,
  .fstat_h = IMFS_stat_sym_link,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .mmap_h = rtems_filesystem_default_mmap,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

static IMFS_jnode_t *IMFS_node_initialize_sym_link(
  IMFS_jnode_t *node,
  void *arg
)
{
  IMFS_sym_link_t *sym_link = (IMFS_sym_link_t *) node;

  sym_link->name = (char *) sym_link + sizeof( *sym_link );
  strcpy( sym_link->name, arg );

  return node;
}

static const IMFS_node_control IMFS_node_control_sym_link = {
  .handlers = &IMFS_link_handlers,
  .node_initialize = IMFS_node_initialize_sym_link,
  .node_remove = IMFS_node_remove_default,
  .node_destroy = IMFS_node_destroy_default
};
