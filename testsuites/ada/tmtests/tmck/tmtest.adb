-- SPDX-License-Identifier: BSD-2-Clause

--
--  TMTEST / BODY
--
--  DESCRIPTION:
--
--  This package is the implementation of Timer Check Test of the RTEMS
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

with TIMER_DRIVER;
with INTERFACES; use INTERFACES;
with TEST_SUPPORT;
with TEXT_IO;
with TIME_TEST_SUPPORT;
with UNSIGNED32_IO;

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

      TIMER_DRIVER.SET_FIND_AVERAGE_OVERHEAD( TRUE );

      TMTEST.TASK_NAME( 1 ) := RTEMS.BUILD_NAME(  'T', 'A', '1', ' ' );

      RTEMS.TASKS.CREATE( 
         TMTEST.TASK_NAME( 1 ), 
         1, 
         2048, 
         RTEMS.DEFAULT_MODES,
         RTEMS.DEFAULT_ATTRIBUTES,
         TMTEST.TASK_ID( 1 ),
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE OF TA1" );

      RTEMS.TASKS.START(
         TMTEST.TASK_ID( 1 ),
         TMTEST.TASK_1'ACCESS,
         0,
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START OF TA1" );

      RTEMS.TASKS.DELETE( RTEMS.SELF, STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_DELETE OF SELF" );

   end INIT;

-- 
--  TASK_1
--

   procedure TASK_1 (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
   begin

      TMTEST.CHECK_READ_TIMER;

TEST_SUPPORT.PAUSE;

      TIMER_DRIVER.INITIALIZE;
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TIME_TEST_SUPPORT.PUT_TIME( 
         "NULL timer stopped at", 
         TMTEST.END_TIME, 
         1, 
         0,
         0
      );

      TIMER_DRIVER.INITIALIZE;
         for INDEX in 0 .. 1000
         loop
            TIMER_DRIVER.EMPTY_FUNCTION;
         end loop;
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TIME_TEST_SUPPORT.PUT_TIME( 
         "LOOP (1000) timer stopped at", 
         TMTEST.END_TIME, 
         1, 
         0,
         0
      );

      TIMER_DRIVER.INITIALIZE;
         for INDEX in 0 .. 10000
         loop
            TIMER_DRIVER.EMPTY_FUNCTION;
         end loop;
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TIME_TEST_SUPPORT.PUT_TIME( 
         "LOOP (10000) timer stopped at", 
         TMTEST.END_TIME, 
         1, 
         0,
         0
      );

      TIMER_DRIVER.INITIALIZE;
         for INDEX in 0 .. 50000
         loop
            TIMER_DRIVER.EMPTY_FUNCTION;
         end loop;
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TIME_TEST_SUPPORT.PUT_TIME( 
         "LOOP (50000) timer stopped at", 
         TMTEST.END_TIME, 
         1, 
         0,
         0
      );

      TIMER_DRIVER.INITIALIZE;
         for INDEX in 0 .. 100000
         loop
            TIMER_DRIVER.EMPTY_FUNCTION;
         end loop;
      TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      TIME_TEST_SUPPORT.PUT_TIME( 
         "LOOP (100000) timer stopped at", 
         TMTEST.END_TIME, 
         1, 
         0,
         0
      );

      TEST_SUPPORT.ADA_TEST_END;
      RTEMS.SHUTDOWN_EXECUTIVE( 0 );
   
   end TASK_1;

-- 
--  CHECK_READ_TIMER
--

   procedure CHECK_READ_TIMER 
   is
      TIME  : RTEMS.UNSIGNED32;
   begin

      for INDEX in TMTEST.DISTRIBUTION'FIRST .. TMTEST.DISTRIBUTION'LAST
      loop
          TMTEST.DISTRIBUTION( INDEX ) := 0;
      end loop;

      for INDEX in 1 .. TMTEST.OPERATION_COUNT
      loop

         loop
            TIMER_DRIVER.INITIALIZE;
            TMTEST.END_TIME := TIMER_DRIVER.READ_TIMER;
      
            exit when 
               TMTEST.END_TIME <= RTEMS.UNSIGNED32( TMTEST.DISTRIBUTION'LAST );

            TEXT_IO.PUT( "TOO LONG (" );
            UNSIGNED32_IO.PUT( TMTEST.END_TIME );
            TEXT_IO.PUT_LINE( ")!!!" );
         end loop;

         TMTEST.DISTRIBUTION( TMTEST.END_TIME ) :=
             TMTEST.DISTRIBUTION( TMTEST.END_TIME ) + 1;

      end loop;

      TEXT_IO.PUT_LINE( 
         "Units may not be in microseconds for this test!!!"
      );

      TIME := 0;

      for INDEX in TMTEST.DISTRIBUTION'FIRST .. TMTEST.DISTRIBUTION'LAST
      loop
         if TMTEST.DISTRIBUTION( INDEX ) /= 0 then
            TIME := TIME + (TMTEST.DISTRIBUTION( INDEX ) * INDEX);
            UNSIGNED32_IO.PUT( INDEX );
            TEXT_IO.PUT( " " );
            UNSIGNED32_IO.PUT( TMTEST.DISTRIBUTION( INDEX ) );
            TEXT_IO.NEW_LINE;
         end if;
      end loop;

      TEXT_IO.PUT( "Total time = " );
      UNSIGNED32_IO.PUT( TIME );
      TEXT_IO.NEW_LINE;

   end CHECK_READ_TIMER;

end TMTEST;
