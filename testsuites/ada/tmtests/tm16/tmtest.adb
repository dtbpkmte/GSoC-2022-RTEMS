-- SPDX-License-Identifier: BSD-2-Clause

--
--  TMTEST / BODY
--
--  DESCRIPTION:
--
--  This package is the implementation of Test 16 of the RTEMS
--  Timing Test Suite.
--
--  DEPENDENCIES: 
--
--  
--
--  COPYRIGHT (c) 1989-2011.
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

with INTERFACES; use INTERFACES;
with RTEMS_CALLING_OVERHEAD;
with TEST_SUPPORT;
with TEXT_IO;
with TIMER_DRIVER;
with RTEMS.EVENT;

package body TMTEST is

-- 
--  INIT
--

   procedure INIT (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      ID     : RTEMS.ID;
      STATUS : RTEMS.STATUS_CODES;
   begin

      TEXT_IO.NEW_LINE( 2 );
      TEST_SUPPORT.ADA_TEST_BEGIN;

      RTEMS.TASKS.CREATE( 
         RTEMS.BUILD_NAME( 'T', 'E', 'S', 'T' ),
         251, 
         2048, 
         RTEMS.DEFAULT_MODES,
         RTEMS.DEFAULT_ATTRIBUTES,
         ID,
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE OF TEST INIT" );

      RTEMS.TASKS.START( 
         ID, 
         TMTEST.TEST_INIT'ACCESS, 
         0, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START OF TEST INIT" );

      RTEMS.TASKS.DELETE( RTEMS.SELF, STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_DELETE OF SELF" );

   end INIT;

-- 
--  TEST_INIT
--

   procedure TEST_INIT (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      PRIORITY   : RTEMS.TASKS.PRIORITY;
      TASK_ENTRY : RTEMS.TASKS.ENTRY_POINT;
      STATUS     : RTEMS.STATUS_CODES;
   begin

      PRIORITY := 250;

      for INDEX in 0 .. TIME_TEST_SUPPORT.OPERATION_COUNT
      loop

         RTEMS.TASKS.CREATE( 
            RTEMS.BUILD_NAME( 'M', 'I', 'D', ' ' ),
            PRIORITY, 
            1024, 
            RTEMS.DEFAULT_MODES,
            RTEMS.DEFAULT_ATTRIBUTES,
            TMTEST.TASK_ID( INDEX ), 
            STATUS
         );
         TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE LOOP" );

         if INDEX = TIME_TEST_SUPPORT.OPERATION_COUNT then
            TASK_ENTRY := TMTEST.HIGH_TASK'ACCESS;
         else
            TASK_ENTRY := TMTEST.MIDDLE_TASKS'ACCESS;
         end if;

         RTEMS.TASKS.START( 
            TMTEST.TASK_ID( INDEX ), 
            TASK_ENTRY, 
            0, 
            STATUS 
         );
         TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START LOOP" );

         PRIORITY := PRIORITY - 1;

      end loop;

      TMTEST.TASK_COUNT := 0;

      TIMER_DRIVER.INITIALIZE;                  -- starts the timer

      RTEMS.EVENT.SEND(                         -- preempts task
         TMTEST.TASK_ID( TMTEST.TASK_COUNT ), 
         RTEMS.EVENT_16, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "EVENT_SEND" );
      
   end TEST_INIT;

-- 
--  MIDDLE_TASKS
--

   procedure MIDDLE_TASKS (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      EVENT_OUT : RTEMS.EVENT_SET;
      STATUS    : RTEMS.STATUS_CODES;
   begin

      RTEMS.EVENT.RECEIVE(                      -- task blocks
         RTEMS.EVENT_16, 
         RTEMS.DEFAULT_OPTIONS,
         RTEMS.NO_TIMEOUT,
         EVENT_OUT,
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "EVENT_RECEIVE" );

      TMTEST.TASK_COUNT := TMTEST.TASK_COUNT + 1;

      RTEMS.EVENT.SEND(                         -- preempts task
         TMTEST.TASK_ID( TMTEST.TASK_COUNT ), 
         RTEMS.EVENT_16, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "EVENT_SEND" );
      
   end MIDDLE_TASKS;

-- 
--  HIGH_TASK
--

   procedure HIGH_TASK (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      EVENT_OUT : RTEMS.EVENT_SET;
      STATUS    : RTEMS.STATUS_CODES;
   begin

      RTEMS.EVENT.RECEIVE(                      -- task blocks
         RTEMS.EVENT_16, 
         RTEMS.DEFAULT_OPTIONS,
         RTEMS.NO_TIMEOUT,
         EVENT_OUT,
         STATUS
      );

      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;

      TIME_TEST_SUPPORT.PUT_TIME( 
         "EVENT_SEND (preemptive)", 
         TMTEST.END_TIME, 
         TIME_TEST_SUPPORT.OPERATION_COUNT, 
         0,
         RTEMS_CALLING_OVERHEAD.EVENT_SEND 
      );

      TEST_SUPPORT.ADA_TEST_END;
      RTEMS.SHUTDOWN_EXECUTIVE( 0 );

   end HIGH_TASK;

end TMTEST;
