-- SPDX-License-Identifier: BSD-2-Clause

--
--  TMTEST / BODY
--
--  DESCRIPTION:
--
--  This package is the implementation of Test 19 of the RTEMS
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

with RTEMS_CALLING_OVERHEAD;
with TEST_SUPPORT;
with TEXT_IO;
with TIME_TEST_SUPPORT;
with TIMER_DRIVER;
with RTEMS.SIGNAL;

package body TMTEST is

-- 
--  INIT
--

   procedure INIT (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      STATUS : RTEMS.STATUS_CODES;
   begin

      TEXT_IO.NEW_LINE( 2 );
      TEST_SUPPORT.ADA_TEST_BEGIN;

      RTEMS.TASKS.CREATE( 
         RTEMS.BUILD_NAME( 'T', 'I', 'M', 'E' ),
         128,
         1024, 
         RTEMS.DEFAULT_MODES,
         RTEMS.DEFAULT_ATTRIBUTES,
         TMTEST.TASK_ID( 1 ),
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE OF TASK1" );

      RTEMS.TASKS.START( 
         TMTEST.TASK_ID( 1 ),
         TMTEST.TASK_1'ACCESS, 
         0, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START OF TASK1" );

      RTEMS.TASKS.CREATE( 
         RTEMS.BUILD_NAME( 'T', 'I', 'M', 'E' ),
         127,
         1024, 
         RTEMS.DEFAULT_MODES,
         RTEMS.DEFAULT_ATTRIBUTES,
         TMTEST.TASK_ID( 2 ),
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE OF TASK2" );

      RTEMS.TASKS.START( 
         TMTEST.TASK_ID( 2 ),
         TMTEST.TASK_2'ACCESS, 
         0, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START OF TASK2" );

      RTEMS.TASKS.CREATE( 
         RTEMS.BUILD_NAME( 'T', 'I', 'M', 'E' ),
         126,
         1024, 
         RTEMS.DEFAULT_MODES,
         RTEMS.DEFAULT_ATTRIBUTES,
         TMTEST.TASK_ID( 3 ),
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE OF TASK3" );

      RTEMS.TASKS.START( 
         TMTEST.TASK_ID( 3 ),
         TMTEST.TASK_3'ACCESS, 
         0, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START OF TASK3" );

      RTEMS.TASKS.DELETE( RTEMS.SELF, STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_DELETE OF SELF" );

   end INIT;

-- 
--  PROCESS_ASR_FOR_PASS_1
--

   procedure PROCESS_ASR_FOR_PASS_1 (
      SIGNALS : in     RTEMS.SIGNAL_SET
   ) is
      pragma Unreferenced(SIGNALS);
   begin

      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;

      TIME_TEST_SUPPORT.PUT_TIME( 
         "SIGNAL_SEND (self)",
         TMTEST.END_TIME, 
         1,
         0,
         RTEMS_CALLING_OVERHEAD.SIGNAL_SEND 
      );

      TEXT_IO.PUT_LINE( "SIGNAL_ENTER (preemptive) na" );

      TIMER_DRIVER.INITIALIZE;

   end PROCESS_ASR_FOR_PASS_1;

-- 
--  PROCESS_ASR_FOR_PASS_2
--

   procedure PROCESS_ASR_FOR_PASS_2 (
      SIGNALS : in     RTEMS.SIGNAL_SET
   ) is
      pragma Unreferenced(SIGNALS);
      STATUS  : RTEMS.STATUS_CODES;
   begin

      TEXT_IO.PUT_LINE( "SIGNAL_ENTER (preemptive) na" );
     
      RTEMS.TASKS.RESUME( TMTEST.TASK_ID( 3 ), STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_RESUME" );

      TIMER_DRIVER.INITIALIZE;

   end PROCESS_ASR_FOR_PASS_2;

-- 
--  TASK_1
--

   procedure TASK_1 (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      STATUS  : RTEMS.STATUS_CODES;
   begin

      TIMER_DRIVER.INITIALIZE;
         RTEMS.SIGNAL.CATCH( 
            TMTEST.PROCESS_ASR_FOR_PASS_1'ACCESS, 
            RTEMS.DEFAULT_MODES,
            STATUS 
         );
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "SIGNAL_CATCH" );

      TIME_TEST_SUPPORT.PUT_TIME( 
         "SIGNAL_CATCH",
         TMTEST.END_TIME, 
         1,
         0,
         RTEMS_CALLING_OVERHEAD.SIGNAL_CATCH 
      );

      TIMER_DRIVER.INITIALIZE;
         RTEMS.SIGNAL.SEND( TMTEST.TASK_ID( 2 ), RTEMS.SIGNAL_1, STATUS );
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "SIGNAL_SEND" );

      TIME_TEST_SUPPORT.PUT_TIME( 
         "SIGNAL_SEND (non-preemptive)",
         TMTEST.END_TIME, 
         1,
         0,
         RTEMS_CALLING_OVERHEAD.SIGNAL_SEND 
      );

      TIMER_DRIVER.INITIALIZE;
         RTEMS.SIGNAL.SEND( RTEMS.SELF, RTEMS.SIGNAL_1, STATUS );
         TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "SIGNAL_SEND" );

      -- end time is done in ASR

      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;

      TIME_TEST_SUPPORT.PUT_TIME( 
         "SIGNAL_RETURN (non-preemptive)",
         TMTEST.END_TIME, 
         1,
         0,
         0
      );

      RTEMS.SIGNAL.CATCH( 
         TMTEST.PROCESS_ASR_FOR_PASS_2'ACCESS, 
         RTEMS.NO_PREEMPT,
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "SIGNAL_CATCH" );

      TIMER_DRIVER.INITIALIZE;
         RTEMS.SIGNAL.SEND( RTEMS.SELF, RTEMS.SIGNAL_1, STATUS );
         TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "SIGNAL_SEND" );

   end TASK_1;

-- 
--  PROCESS_ASR_FOR_TASK_2
--

   procedure PROCESS_ASR_FOR_TASK_2 (
      SIGNALS : in     RTEMS.SIGNAL_SET
   ) is
   begin

      NULL;

   end PROCESS_ASR_FOR_TASK_2;

-- 
--  TASK_2
--

   procedure TASK_2 (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      STATUS  : RTEMS.STATUS_CODES;
   begin

      RTEMS.SIGNAL.CATCH( 
         TMTEST.PROCESS_ASR_FOR_TASK_2'ACCESS, 
         RTEMS.DEFAULT_MODES,
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "SIGNAL_CATCH" );

      RTEMS.TASKS.SUSPEND( RTEMS.SELF, STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_SUSPEND" );

   end TASK_2;

-- 
--  TASK_3
--

   procedure TASK_3 (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      STATUS : RTEMS.STATUS_CODES;
   begin

      RTEMS.TASKS.SUSPEND( RTEMS.SELF, STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_SUSPEND" );

      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;

      TIME_TEST_SUPPORT.PUT_TIME( 
         "SIGNAL_RETURN (preemptive)",
         TMTEST.END_TIME, 
         1,
         0,
         0
     );

      TEST_SUPPORT.ADA_TEST_END;
     RTEMS.SHUTDOWN_EXECUTIVE( 0 );

   end TASK_3;

end TMTEST;
