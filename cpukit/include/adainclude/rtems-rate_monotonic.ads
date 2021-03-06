-- SPDX-License-Identifier: BSD-2-Clause

--
--  RTEMS / Specification
--
--  DESCRIPTION:
--
--  This package provides the interface to the RTEMS API.
--
--  DEPENDENCIES:
--
--  NOTES:
--    RTEMS initialization and configuration are called from
--    the BSP side, therefore should never be called from ADA.
--
--  COPYRIGHT (c) 1997-2011.
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

package RTEMS.Rate_Monotonic is

   --
   --  The following type defines the status information returned
   --  about a period.
   --

   type Period_States is (
     Inactive,               -- off chain, never initialized
     Owner_Is_Blocking,      -- on chain, owner is blocking on it
     Active,                 -- on chain, running continuously
     Expired_While_Blocking, -- on chain, expired while owner was was blocking
     Expired                 -- off chain, will be reset by next
                             --   rtems_rate_monotonic_period
   );

   for Period_States'Size use 32;

   for Period_States use (
     Inactive                => 0,
     Owner_Is_Blocking       => 1,
     Active                  => 2,
     Expired_While_Blocking  => 3,
     Expired                 => 4
   );

   type Period_Status is
      record
         Owner                            : RTEMS.ID;
         State                            : RTEMS.Rate_Monotonic.Period_States;
         Ticks_Since_Last_Period          : RTEMS.Unsigned32;
         Ticks_Executed_Since_Last_Period : RTEMS.Unsigned32;
      end record;

   --
   --  Rate Monotonic Manager
   --

   procedure Create (
      Name   : in     RTEMS.Name;
      ID     :    out RTEMS.ID;
      Result :    out RTEMS.Status_Codes
   );

   procedure Ident (
      Name   : in     RTEMS.Name;
      ID     :    out RTEMS.ID;
      Result :    out RTEMS.Status_Codes
   );

   procedure Delete (
      ID     : in     RTEMS.ID;
      Result :    out RTEMS.Status_Codes
   );

   procedure Cancel (
      ID     : in     RTEMS.ID;
      Result :    out RTEMS.Status_Codes
   );

   procedure Period (
      ID      : in     RTEMS.ID;
      Length  : in     RTEMS.Interval;
      Result  :    out RTEMS.Status_Codes
   );

   procedure Get_Status (
      ID      : in     RTEMS.ID;
      Status  :    out RTEMS.Rate_Monotonic.Period_Status;
      Result  :    out RTEMS.Status_Codes
   );

   procedure Reset_Statistics (
      ID     : in     RTEMS.ID;
      Result :    out RTEMS.Status_Codes
   );

   procedure Reset_All_Statistics;
   pragma Import (
      C,
      Reset_All_Statistics,
      "rtems_rate_monotonic_reset_all_statistics"
   );

   procedure Report_Statistics;
   pragma Import (
      C,
      Report_Statistics,
      "rtems_rate_monotonic_report_statistics"
   );

end RTEMS.Rate_Monotonic;

