/* SPDX-License-Identifier: BSD-2-Clause */

/*
 *  COPYRIGHT (c) 2012-2014 Chris Johns <chrisj@rtems.org>
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
/**
 * @file
 *
 * @ingroup rtems_rtl
 *
 * @brief RTEMS Run-Time Linker ELF Trace Support.
 */

#if !defined (_RTEMS_RTL_TRACE_H_)
#define _RTEMS_RTL_TRACE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

#include <rtems/printer.h>

/**
 * Set to 1 to build trace support in to the RTL code.
 */
#define RTEMS_RTL_TRACE 1

/**
 * The type of the mask.
 */
typedef uint32_t rtems_rtl_trace_mask;

/**
 * List of tracing bits for the various parts of the link editor.
 */
#define RTEMS_RTL_TRACE_DETAIL                 (1UL << 0)
#define RTEMS_RTL_TRACE_WARNING                (1UL << 1)
#define RTEMS_RTL_TRACE_LOAD                   (1UL << 2)
#define RTEMS_RTL_TRACE_UNLOAD                 (1UL << 3)
#define RTEMS_RTL_TRACE_SECTION                (1UL << 4)
#define RTEMS_RTL_TRACE_SYMBOL                 (1UL << 5)
#define RTEMS_RTL_TRACE_RELOC                  (1UL << 6)
#define RTEMS_RTL_TRACE_GLOBAL_SYM             (1UL << 7)
#define RTEMS_RTL_TRACE_LOAD_SECT              (1UL << 8)
#define RTEMS_RTL_TRACE_ALLOCATOR              (1UL << 9)
#define RTEMS_RTL_TRACE_UNRESOLVED             (1UL << 10)
#define RTEMS_RTL_TRACE_CACHE                  (1UL << 11)
#define RTEMS_RTL_TRACE_ARCHIVES               (1UL << 12)
#define RTEMS_RTL_TRACE_ARCHIVE_SYMS           (1UL << 13)
#define RTEMS_RTL_TRACE_DEPENDENCY             (1UL << 14)
#define RTEMS_RTL_TRACE_BIT_ALLOC              (1UL << 15)
#define RTEMS_RTL_TRACE_COMP                   (1UL << 16)
#define RTEMS_RTL_TRACE_ALL                    (0xffffffffUL & ~(RTEMS_RTL_TRACE_CACHE | \
                                                                 RTEMS_RTL_TRACE_COMP | \
                                                                 RTEMS_RTL_TRACE_GLOBAL_SYM | \
                                                                 RTEMS_RTL_TRACE_ARCHIVE_SYMS))

/**
 * Call to check if this part is bring traced. If RTEMS_RTL_TRACE is defined to
 * 0 the code is dead code elminiated when built with -Os, -O2, or higher.
 *
 * @param mask The part of the API to trace.
 * @retval true Tracing is active for the mask.
 * @retval false Do not trace.
 */
#if RTEMS_RTL_TRACE
bool rtems_rtl_trace (rtems_rtl_trace_mask mask);
#else
#define rtems_rtl_trace(_m) (0)
#endif

/**
 * Set the mask.
 *
 * @param mask The mask bits to set.
 * @return The previous mask.
 */
#if RTEMS_RTL_TRACE
rtems_rtl_trace_mask rtems_rtl_trace_set_mask (rtems_rtl_trace_mask mask);
#else
#define rtems_rtl_trace_set_mask(_m)
#endif

/**
 * Clear the mask.
 *
 * @param mask The mask bits to clear.
 * @return The previous mask.
 */
#if RTEMS_RTL_TRACE
rtems_rtl_trace_mask rtems_rtl_trace_clear_mask (rtems_rtl_trace_mask mask);
#else
#define rtems_rtl_trace_clear_mask(_m)
#endif

/**
 * Add shell trace shell command.
 */
#if RTEMS_RTL_TRACE
int rtems_rtl_trace_shell_command (const rtems_printer* printer,
				   int                  argc,
				   char*                argv[]);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
