/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsMIPSMalta
 *
 * @brief Global BSP definitions.
 */

/*
 *  COPYRIGHT (c) 1989-2012.
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

#ifndef LIBBSP_MIPS_MALTA_BSP_H
#define LIBBSP_MIPS_MALTA_BSP_H

/**
 * @defgroup RTEMSBSPsMIPSMalta Malta
 *
 * @ingroup RTEMSBSPsMIPS
 *
 * @brief Malta Board Support Package.
 *
 * @{
 */

#ifndef ASM

#include <bspopts.h>
#include <bsp/default-initial-extension.h>

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_FEATURE_IRQ_EXTENSION
#define BSP_SHARED_HANDLER_SUPPORT      1

#define REVISION_REGISTER_ADDRESS  0x1fc00010
#define PRORV_MASK       0x0000000f   /* 4 bit Product Revision */
#define PROID_MASK       0x000000f0   /* 4 bit Product ID */
#define CORRV_MASK       0x00000300   /* 2 bit Core Board Revision */
#define CORID_MASK       0x0000fc00   /* 6 bit Core Board ID */
#define FPGRV_MASK       0x00ff0000   /* 8 bit CBUS FPGA Revision */
#define BSP_8259_BASE_ADDRESS    (0x18000000UL | 0xa0000000UL)
#define BSP_PCI_BASE_ADDRESS     (0x1be00000UL | 0xa0000000UL)
#define BSP_NIC_IO_BASE          (0x10000000UL | 0xa0000000UL)
#define PCI0_IO_BASE             (0x18000000UL | 0xa0000000UL)
#define BSP_NIC_MEM_BASE         (0x00000000UL | 0xa0000000UL)

/* functions */
#define WRITE_PROTECTED_UINT8( _addr, _value ) \
        do { \
          volatile uint8_t *_ptr = _addr | 0x80000000; \
          *_ptr = _value; \
        }
#define WRITE_PROTECTED_UINT16( _addr, _value ) \
        do { \
          volatile uint16_t *_ptr = _addr | 0x80000000; \
          *_ptr = _value; \
        }
#define WRITE_PROTECTED_UINT32( _addr, _value ) \
        do { \
          volatile uint32_t *_ptr = _addr | 0x80000000; \
          *_ptr = _value; \
        }
#define READ_PROTECTED_UINT8( _addr, _value ) \
        do { \
          volatile uint8_t *_ptr = _addr | 0x80000000; \
         _value = *_ptr; \
        }
#define READ_PROTECTED_UINT16( _addr, _value ) \
        do { \
          volatile uint16_t *_ptr = _addr | 0x80000000; \
         _value = *_ptr; \
        }
#define READ_PROTECTED_UINT32( _addr, _value ) \
        do { \
          volatile uint32_t *_ptr = _addr | 0x80000000; \
         _value = *_ptr; \
        }

#define READ_UINT8( _register_, _value_ ) \
        ((_value_) = *((volatile unsigned char *)(_register_)))

#define WRITE_UINT8( _register_, _value_ ) \
        (*((volatile unsigned char *)(_register_)) = (_value_))

#define READ_UINT16( _register_, _value_ ) \
     ((_value_) = *((volatile unsigned short *)(_register_)))

#define WRITE_UINT16( _register_, _value_ ) \
     (*((volatile unsigned short *)(_register_)) = (_value_))

void simple_out_32(uint32_t base, uint32_t addr, uint32_t val);
void simple_out_le32(uint32_t base, uint32_t addr, uint32_t val);
uint8_t simple_in_8( uint32_t base, uint32_t addr );
void simple_out_8( uint32_t base, uint32_t addr, uint8_t val );
int16_t simple_in_le16( uint32_t base, uint32_t addr );
int16_t simple_in_16( uint32_t base, uint32_t addr );
uint32_t simple_in_le32( uint32_t base, uint32_t addr );
uint32_t simple_in_32( uint32_t base, uint32_t addr );
void simple_out_le16( uint32_t base, uint32_t addr, uint16_t val );
void simple_out_16( uint32_t base, uint32_t addr, uint16_t val );

/*
 * Prototypes for methods called from .S for dependency tracking
 */
void init_tlb(void);
void resettlb(int i);

#ifdef __cplusplus
}
#endif

#endif /* !ASM */

/** @} */

#endif
