-- SPDX-License-Identifier: BSD-2-Clause

--
--  SPTEST / BODY
--
--  DESCRIPTION:
--
--  This package is the implementation of Test 8 of the RTEMS
--  Single Processor Test Suite.
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
with TEST_SUPPORT;
with TEXT_IO;
with UNSIGNED32_IO;

package body SPTEST is

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

      SPTEST.TASK_NAME( 1 ) := RTEMS.BUILD_NAME(  'T', 'A', '1', ' ' );

      RTEMS.TASKS.CREATE( 
         SPTEST.TASK_NAME( 1 ), 
         1, 
         2048, 
         RTEMS.DEFAULT_MODES,
         RTEMS.DEFAULT_ATTRIBUTES,
         SPTEST.TASK_ID( 1 ),
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_CREATE OF TA1" );

      RTEMS.TASKS.START(
         SPTEST.TASK_ID( 1 ),
         SPTEST.TASK_1'ACCESS,
         0,
         STATUS
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_START OF TA1" );

      RTEMS.TASKS.DELETE( RTEMS.SELF, STATUS );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_DELETE OF SELF" );

   end INIT;

-- 
--  PUT_MODE
--

   procedure PUT_MODE(
      COMMENT     : in    STRING;
      OUTPUT_MODE : in    RTEMS.MODE
   ) is
   begin

      TEXT_IO.PUT( COMMENT );
      UNSIGNED32_IO.PUT( OUTPUT_MODE, BASE => 16, WIDTH => 8 );
      TEXT_IO.NEW_LINE;

   end PUT_MODE;

-- 
--  TASK_1
--

   procedure TASK_1 (
      ARGUMENT : in     RTEMS.TASKS.ARGUMENT
   ) is
      pragma Unreferenced(ARGUMENT);
      PREVIOUS_MODE : RTEMS.MODE;
      STATUS : RTEMS.STATUS_CODES;
   begin

-- BEGINNING OF ASR

      RTEMS.TASKS.MODE( 
         RTEMS.ASR, 
         RTEMS.ASR_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - ASR                  - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.NO_ASR, 
         RTEMS.ASR_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - NO_ASR               - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.NO_ASR, 
         RTEMS.ASR_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - NO_ASR               - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.ASR, 
         RTEMS.ASR_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - ASR                  - previous mode: ", 
         PREVIOUS_MODE
      );
   
-- END OF ASR

-- BEGINNING OF TIMESLICE

      RTEMS.TASKS.MODE( 
         RTEMS.NO_TIMESLICE, 
         RTEMS.TIMESLICE_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - NO_TIMESLICE         - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.TIMESLICE, 
         RTEMS.TIMESLICE_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - TIMESLICE            - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.TIMESLICE, 
         RTEMS.TIMESLICE_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - TIMESLICE            - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.NO_TIMESLICE, 
         RTEMS.TIMESLICE_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - NO_TIMESLICE         - previous mode: ", 
         PREVIOUS_MODE
      );
   
-- END OF TIMESLICE

-- BEGINNING OF PREEMPT

      RTEMS.TASKS.MODE( 
         RTEMS.PREEMPT, 
         RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - PREEMPT              - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.NO_PREEMPT, 
         RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - NO_PREEMPT           - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.NO_PREEMPT, 
         RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - NO_PREEMPT           - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.PREEMPT, 
         RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - PREEMPT              - previous mode: ", 
         PREVIOUS_MODE
      );
   
-- END OF PREEMPT

-- BEGINNING OF INTERRUPT_LEVEL

      RTEMS.TASKS.MODE( 
         RTEMS.INTERRUPT_LEVEL( 3 ), 
         RTEMS.INTERRUPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - INTERRUPT_LEVEL( 3 ) - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.INTERRUPT_LEVEL( 5 ), 
         RTEMS.INTERRUPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - INTERRUPT_LEVEL( 5 ) - previous mode: ", 
         PREVIOUS_MODE
      );
   
-- END OF INTERRUPT_LEVEL

-- BEGINNING OF COMBINATIONS

      RTEMS.TASKS.MODE( 
         RTEMS.INTERRUPT_LEVEL( 3 ) + RTEMS.NO_ASR + 
            RTEMS.TIMESLICE + RTEMS.NO_PREEMPT, 
         RTEMS.INTERRUPT_MASK + RTEMS.ASR_MASK + 
            RTEMS.TIMESLICE_MASK + RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - set all modes        - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.INTERRUPT_LEVEL( 3 ) + RTEMS.NO_ASR + 
            RTEMS.TIMESLICE + RTEMS.NO_PREEMPT, 
         RTEMS.INTERRUPT_MASK + RTEMS.ASR_MASK + 
            RTEMS.TIMESLICE_MASK + RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - set all modes        - previous mode: ", 
         PREVIOUS_MODE
      );
   
      RTEMS.TASKS.MODE( 
         RTEMS.INTERRUPT_LEVEL( 0 ) + RTEMS.ASR + 
            RTEMS.NO_TIMESLICE + RTEMS.PREEMPT, 
         RTEMS.INTERRUPT_MASK + RTEMS.ASR_MASK + 
            RTEMS.TIMESLICE_MASK + RTEMS.PREEMPT_MASK, 
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - clear all modes      - previous mode: ", 
         PREVIOUS_MODE
      );
   
-- END OF COMBINATIONS

-- BEGINNING OF CURRENT MODE

      RTEMS.TASKS.MODE( 
         RTEMS.CURRENT_MODE,
         RTEMS.CURRENT_MODE,
         PREVIOUS_MODE, 
         STATUS 
      );
      TEST_SUPPORT.DIRECTIVE_FAILED( STATUS, "TASK_MODE" );
      SPTEST.PUT_MODE(
         "TA1 - task_mode - get current mode     - previous mode: ", 
         PREVIOUS_MODE
      );
   
-- END OF CURRENT MODE

      TEST_SUPPORT.ADA_TEST_END;
      RTEMS.SHUTDOWN_EXECUTIVE( 0 );
   end TASK_1;

end SPTEST;
