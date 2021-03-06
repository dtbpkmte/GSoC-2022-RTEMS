/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 */

/*
 *  This file contains information pertaining to the OR1K processor.
 *
 *  COPYRIGHT (c) 2014 Hesham ALMatary <heshamelmatary@gmail.com>
 *
 *  Based on code with the following copyright...
 *  COPYRIGHT (c) 1989-1999, 2010.
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

#ifndef _RTEMS_SCORE_OR1K_H
#define _RTEMS_SCORE_OR1K_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  This file contains the information required to build
 *  RTEMS for a particular member of the OR1K family.
 *  It does this by setting variables to indicate which
 *  implementation dependent features are present in a particular
 *  member of the family.
 *
 *  This is a good place to list all the known CPU models
 *  that this port supports and which RTEMS CPU model they correspond
 *  to.
 */

 /*
 *  Define the name of the CPU family and specific model.
 */

#define CPU_NAME "OR1K"
#define CPU_MODEL_NAME "OR1200"

#ifdef __cplusplus
}
#endif

#endif /* _RTEMS_SCORE_OR1K_H */
