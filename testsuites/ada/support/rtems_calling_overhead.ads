-- SPDX-License-Identifier: BSD-2-Clause

--
--  RTEMS_CALLING_OVERHEAD / SPECIFICATION
--
--  DESCRIPTION:
--
--  This package contains the invocation overhead for each 
--  of the RTEMS directives on the MC68020 Timing Platform.
--  This time is then subtracted from the execution time
--  of each directive as measured by the Timing Suite.
--
--  DEPENDENCIES: 
--
--  
--
--  COPYRIGHT (c) 1989-1997.
--  On-Line Applications Research Corporation (OAR).
--
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions
--  are met:
--  1. Redistributions of source code must retain the above copyright
--     notice, this list of conditions and the following disclaimer.
--  2. Redistributions in binary form must reproduce the above copyright
--     notice, this list of conditions and the following disclaimer in the
--     documentation and/or other materials provided with the distribution.
--
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
--  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
--  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
--  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
--  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
--  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
--  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
--  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
--  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--  POSSIBILITY OF SUCH DAMAGE.
--

with RTEMS;

package RTEMS_CALLING_OVERHEAD is

   INITIALIZE_EXECUTIVE      : constant RTEMS.UNSIGNED32 := 0;
   SHUTDOWN_EXECUTIVE        : constant RTEMS.UNSIGNED32 := 0;
   TASK_CREATE               : constant RTEMS.UNSIGNED32 := 0;
   TASK_IDENT                : constant RTEMS.UNSIGNED32 := 0;
   TASK_START                : constant RTEMS.UNSIGNED32 := 0;
   TASK_RESTART              : constant RTEMS.UNSIGNED32 := 0;
   TASK_DELETE               : constant RTEMS.UNSIGNED32 := 0;
   TASK_SUSPEND              : constant RTEMS.UNSIGNED32 := 0;
   TASK_RESUME               : constant RTEMS.UNSIGNED32 := 0;
   TASK_SET_PRIORITY         : constant RTEMS.UNSIGNED32 := 0;
   TASK_MODE                 : constant RTEMS.UNSIGNED32 := 0;
   TASK_GET_NOTE             : constant RTEMS.UNSIGNED32 := 0;
   TASK_SET_NOTE             : constant RTEMS.UNSIGNED32 := 0;
   TASK_WAKE_WHEN            : constant RTEMS.UNSIGNED32 := 0;
   TASK_WAKE_AFTER           : constant RTEMS.UNSIGNED32 := 0;
   INTERRUPT_CATCH           : constant RTEMS.UNSIGNED32 := 0;
   CLOCK_GET                 : constant RTEMS.UNSIGNED32 := 0;
   CLOCK_SET                 : constant RTEMS.UNSIGNED32 := 0;
   CLOCK_TICK                : constant RTEMS.UNSIGNED32 := 0;

   TIMER_CREATE              : constant RTEMS.UNSIGNED32 := 0;
   TIMER_DELETE              : constant RTEMS.UNSIGNED32 := 0;
   TIMER_IDENT               : constant RTEMS.UNSIGNED32 := 0;
   TIMER_FIRE_AFTER          : constant RTEMS.UNSIGNED32 := 0;
   TIMER_FIRE_WHEN           : constant RTEMS.UNSIGNED32 := 0;
   TIMER_RESET               : constant RTEMS.UNSIGNED32 := 0;
   TIMER_CANCEL              : constant RTEMS.UNSIGNED32 := 0;
   SEMAPHORE_CREATE          : constant RTEMS.UNSIGNED32 := 0;
   SEMAPHORE_DELETE          : constant RTEMS.UNSIGNED32 := 0;
   SEMAPHORE_IDENT           : constant RTEMS.UNSIGNED32 := 0;
   SEMAPHORE_OBTAIN          : constant RTEMS.UNSIGNED32 := 0;
   SEMAPHORE_RELEASE         : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_CREATE      : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_IDENT       : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_DELETE      : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_SEND        : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_URGENT      : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_BROADCAST   : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_RECEIVE     : constant RTEMS.UNSIGNED32 := 0;
   MESSAGE_QUEUE_FLUSH       : constant RTEMS.UNSIGNED32 := 0;

   EVENT_SEND                : constant RTEMS.UNSIGNED32 := 0;
   EVENT_RECEIVE             : constant RTEMS.UNSIGNED32 := 0;
   SIGNAL_CATCH              : constant RTEMS.UNSIGNED32 := 0;
   SIGNAL_SEND               : constant RTEMS.UNSIGNED32 := 0;
   PARTITION_CREATE          : constant RTEMS.UNSIGNED32 := 0;
   PARTITION_IDENT           : constant RTEMS.UNSIGNED32 := 0;
   PARTITION_DELETE          : constant RTEMS.UNSIGNED32 := 0;
   PARTITION_GET_BUFFER      : constant RTEMS.UNSIGNED32 := 0;
   PARTITION_RETURN_BUFFER   : constant RTEMS.UNSIGNED32 := 0;
   REGION_CREATE             : constant RTEMS.UNSIGNED32 := 0;
   REGION_IDENT              : constant RTEMS.UNSIGNED32 := 0;
   REGION_DELETE             : constant RTEMS.UNSIGNED32 := 0;
   REGION_GET_SEGMENT        : constant RTEMS.UNSIGNED32 := 0;
   REGION_RETURN_SEGMENT     : constant RTEMS.UNSIGNED32 := 0;
   PORT_CREATE               : constant RTEMS.UNSIGNED32 := 0;
   PORT_IDENT                : constant RTEMS.UNSIGNED32 := 0;
   PORT_DELETE               : constant RTEMS.UNSIGNED32 := 0;
   PORT_EXTERNAL_TO_INTERNAL : constant RTEMS.UNSIGNED32 := 0;
   PORT_INTERNAL_TO_EXTERNAL : constant RTEMS.UNSIGNED32 := 0;

   IO_INITIALIZE             : constant RTEMS.UNSIGNED32 := 0;
   IO_OPEN                   : constant RTEMS.UNSIGNED32 := 0;
   IO_CLOSE                  : constant RTEMS.UNSIGNED32 := 0;
   IO_READ                   : constant RTEMS.UNSIGNED32 := 0;
   IO_WRITE                  : constant RTEMS.UNSIGNED32 := 0;
   IO_CONTROL                : constant RTEMS.UNSIGNED32 := 0;
   FATAL_ERROR_OCCURRED      : constant RTEMS.UNSIGNED32 := 0;
   RATE_MONOTONIC_CREATE     : constant RTEMS.UNSIGNED32 := 0;
   RATE_MONOTONIC_IDENT      : constant RTEMS.UNSIGNED32 := 0;
   RATE_MONOTONIC_DELETE     : constant RTEMS.UNSIGNED32 := 0;
   RATE_MONOTONIC_CANCEL     : constant RTEMS.UNSIGNED32 := 0;
   RATE_MONOTONIC_PERIOD     : constant RTEMS.UNSIGNED32 := 0;
   MULTIPROCESSING_ANNOUNCE  : constant RTEMS.UNSIGNED32 := 0;

end RTEMS_CALLING_OVERHEAD;
