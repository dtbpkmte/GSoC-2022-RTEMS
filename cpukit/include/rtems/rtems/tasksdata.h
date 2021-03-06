/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSImplClassicTask
 *
 * @brief This header file provides data structures used by the implementation
 *   and the @ref RTEMSImplApplConfig to define the
 *   ::_RTEMS_tasks_User_task_table, the ::_RTEMS_tasks_User_task_config,
 *   ::_RTEMS_tasks_Information, and ultimately ::Thread_Configured_control.
 */

/*
 * COPYRIGHT (c) 1989-2014.
 * On-Line Applications Research Corporation (OAR).
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

#ifndef _RTEMS_RTEMS_TASKSDATA_H
#define _RTEMS_RTEMS_TASKSDATA_H

#include <rtems/rtems/tasks.h>
#include <rtems/rtems/asrdata.h>
#include <rtems/rtems/eventdata.h>
#include <rtems/score/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup RTEMSImplClassicTask
 *
 * @{
 */

/**
 *  This is the API specific information required by each thread for
 *  the RTEMS API to function correctly.
 *
 */
typedef struct {
  /** This field contains the event control for this task. */
  Event_Control            Event;
  /** This field contains the system event control for this task. */
  Event_Control            System_event;
  /** This field contains the Classic API Signal information for this task. */
  ASR_Information          Signal;

  /**
   * @brief Signal post-switch action in case signals are pending.
   */
  Thread_Action            Signal_action;
}  RTEMS_API_Control;

/**
 * @brief Initialization table for the first user task.
 *
 * This table is used by _RTEMS_tasks_Initialize_user_task() and initialized
 * via <rtems/confdefs.h>.
 */
extern const rtems_initialization_tasks_table _RTEMS_tasks_User_task_table;

/**
 * @brief Creates and starts the Classic API initialization task using
 *   rtems_task_create() and the configuration provided by
 *   ::_RTEMS_tasks_User_task_table.
 */
void _RTEMS_tasks_Initialize_user_task( void );

/**
 * @brief This structure provides the configuration to construct and start the
 *   Classic API initialization task.
 */
typedef struct {
  /**
   * @brief This member provides the task configuration for
   *   rtems_task_construct().
   */
  rtems_task_config config;

  /**
   * @brief This member provides the task entry point for rtems_task_start().
   */
  rtems_task_entry entry_point;

  /**
   * @brief This member provides the task argument for rtems_task_start().
   */
  rtems_task_argument argument;
} RTEMS_tasks_User_task_config;

/**
 * @brief This structure provides the configuration of the Classic API
 *   initialization task.
 *
 * It is used by _RTEMS_tasks_Construct_user_task() and initialized via
 * <rtems/confdefs.h>, see also #CONFIGURE_INIT_TASK_CONSTRUCT_STORAGE_SIZE.
 */
extern const RTEMS_tasks_User_task_config _RTEMS_tasks_User_task_config;

/**
 * @brief Constructs and starts the Classic API initialization task using
 *   rtems_task_construct() and the configuration provided by
 *   ::_RTEMS_tasks_User_task_config.
 */
void _RTEMS_tasks_Construct_user_task( void );

/**
 *  The following instantiates the information control block used to
 *  manage this class of objects.
 */
extern Thread_Information _RTEMS_tasks_Information;

/** @} */

#ifdef __cplusplus
}
#endif

#endif
/* end of include file */
