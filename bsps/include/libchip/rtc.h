/* SPDX-License-Identifier: BSD-2-Clause */

/*
 *  This file contains the Real-Time Clock definitions.
 *
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

#ifndef __LIBCHIP_RTC_h
#define __LIBCHIP_RTC_h

#include <stdbool.h>
#include <stdint.h>

#include <rtems.h>

/*
 *  Types for get and set register routines
 */

typedef uint32_t (*getRegister_f)(uintptr_t  port, uint8_t reg);
typedef void     (*setRegister_f)(uintptr_t  port, uint8_t reg, uint32_t value);

typedef struct _rtc_fns {
  void    (*deviceInitialize)(int minor);
  int     (*deviceGetTime)(int minor, rtems_time_of_day *time);
  int     (*deviceSetTime)(int minor, const rtems_time_of_day *time);
} rtc_fns;

typedef enum {
  RTC_M48T08,                  /* SGS-Thomsom M48T08 or M48T18 */
  RTC_ICM7170,                 /* Harris ICM-7170 */
  RTC_CUSTOM,                  /* BSP specific driver */
  RTC_MC146818A                /* Motorola MC146818A */
} rtc_devs;

/*
 * Each field is interpreted thus:
 *
 * sDeviceName  This is the name of the device.
 *
 * deviceType   This indicates the chip type.
 *
 * pDeviceFns   This is a pointer to the set of driver routines to use.
 *
 * pDeviceParams This contains either device specific data or a pointer to a
 *              device specific information table.
 *
 * ulCtrlPort1  This is the primary control port number for the device.
 *
 * ulDataPort   This is the port number for the data port of the device
 *
 * getRegister  This is the routine used to read register values.
 *
 * setRegister  This is the routine used to write register values.
 */

typedef struct _rtc_tbl {
  const char    *sDeviceName;
  rtc_devs       deviceType;
  const rtc_fns *pDeviceFns;
  bool           (*deviceProbe)(int minor);
  void          *pDeviceParams;
  uintptr_t      ulCtrlPort1;
  uintptr_t      ulDataPort;
  getRegister_f  getRegister;
  setRegister_f  setRegister;
} rtc_tbl;

extern rtc_tbl  RTC_Table[];
extern size_t   RTC_Count;


extern bool rtc_probe( int minor );

#endif
/* end of include file */
