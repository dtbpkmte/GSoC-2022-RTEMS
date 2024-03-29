/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * RTEMS generic MPC83xx BSP
 *
 * This file contains the startup assembly code.
 */

/*
 * Copyright (c) 2007 embedded brains GmbH. All rights reserved.
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

#include <libcpu/powerpc-utility.h>
#include <rtems/powerpc/cache.h>
#include <bsp.h>
#include <mpc83xx/mpc83xx.h>

.macro SET_IMM_REGW base, reg2, offset, value
	LA \reg2, \value
	stw \reg2,\offset(\base)
.endm

#define REP8(l) l ; l; l; l; l; l; l; l;

.extern boot_card
.extern MBAR

#if defined(RESET_CONF_WRD_L)
.section ".resconf","ax"
PUBLIC_VAR (reset_conf_words)
reset_conf_words:
	REP8( .byte ((RESET_CONF_WRD_L >> 24) & 0xff))
	REP8( .byte ((RESET_CONF_WRD_L >> 16) & 0xff))
	REP8( .byte ((RESET_CONF_WRD_L >>  8) & 0xff))
	REP8( .byte ((RESET_CONF_WRD_L >>  0) & 0xff))

	REP8( .byte ((RESET_CONF_WRD_H >> 24) & 0xff))
	REP8( .byte ((RESET_CONF_WRD_H >> 16) & 0xff))
	REP8( .byte ((RESET_CONF_WRD_H >>  8) & 0xff))
	REP8( .byte ((RESET_CONF_WRD_H >>  0) & 0xff))
#endif

.section ".vectors","ax"
PUBLIC_VAR (reset_vec)
reset_vec:
	bl rom_entry

.section ".bsp_start_text", "ax"
PUBLIC_VAR (_start)
_start:
	/* Reset time base */
	li	r0, 0
	mtspr	TBWU, r0
	mtspr	TBWL, r0

#ifdef HAS_UBOOT
	mr	r14, r3
#endif /* HAS_UBOOT */

	/*
	 * basic CPU setup:
	 * init MSR
	 */
	mfmsr	r30
	SETBITS r30, r29, MSR_ME|MSR_RI
	CLRBITS r30, r29, MSR_IP|MSR_EE
	mtmsr	r30 				/* Set RI/ME, Clr EE in MSR */

	b start_rom_skip

PUBLIC_VAR (rom_entry)
rom_entry:
	/*
	 * basic CPU setup:
	 * init MSR
	 */
	mfmsr	r30
	SETBITS r30, r29, MSR_ME|MSR_RI
	CLRBITS r30, r29, MSR_IP|MSR_EE
	mtmsr	r30 				/* Set RI/ME, Clr EE in MSR */

	/*
	 * ROM startup: remap IMMR to 0xE0000000
	 * use special sequence from MPC8349EA RM Rev 1, 5.2.4.1.1 "Updating IMMRBAR"
	 */
	LWI  r30,IMMRBAR_DEFAULT
	LWI  r31,IMMRBAR
	lwz  r29,0(r30)
	stw  r31,0(r30)
#if 0
	lwz  r29,0(r28) /* read from ROM... */
#endif
	isync
	lwz  r29,0(r31) /* read from IMMRBAR... */
	isync
	/*
	 * NOTE:	 now r31 points to onchip registers
	*/
	/*
	 * we start from 0x100, so ROM is currently mapped to
	 * 0x00000000..
	 * in the next step, ROM will be remapped to its final location
	 * at 0xfe000000... (using LBLAWBAR1 with LBLAWBAR0 value)
	 * and we jump to that location.
	 * then we remove the ROM mapping to zero
	 */
#ifdef LBLAWBAR0_VAL
	SET_IMM_REGW r31,r30,LBLAWBAR1_OFF,LBLAWBAR0_VAL
#endif
#ifdef LBLAWAR0_VAL
	SET_IMM_REGW r31,r30,LBLAWAR1_OFF,LBLAWAR0_VAL
#endif


	/*
	 * ROM startup: jump to code final ROM location
	 */
	LA	r20, bsp_rom_start /* ROM-RAM reloc in r20 */
	LA	r29, start_code_in_rom /* get compile time addr of label */
	add	r29,r20,r29    /* compute exec address */
	mtlr	r29
	blr                     /* now further execution in upper ROM */

start_code_in_rom:

#ifdef LBLAWBAR0_VAL
	SET_IMM_REGW r31,r30,LBLAWBAR0_OFF,LBLAWBAR0_VAL
#endif
#ifdef LBLAWAR0_VAL
	SET_IMM_REGW r31,r30,LBLAWAR0_OFF,LBLAWAR0_VAL
#endif

/*
 * Local access window 1 is a special case since we used it for a temporary
 * mapping.  If we do not use it then restore the reset value.
 */
#ifdef LBLAWBAR1_VAL
	SET_IMM_REGW r31,r30,LBLAWBAR1_OFF,LBLAWBAR1_VAL
#else
	SET_IMM_REGW r31,r30,LBLAWBAR1_OFF,0
#endif
#ifdef LBLAWAR1_VAL
	SET_IMM_REGW r31,r30,LBLAWAR1_OFF,LBLAWAR1_VAL
#else
	SET_IMM_REGW r31,r30,LBLAWAR1_OFF,0
#endif

#ifdef LBLAWBAR2_VAL
	SET_IMM_REGW r31,r30,LBLAWBAR2_OFF,LBLAWBAR2_VAL
#endif
#ifdef LBLAWAR2_VAL
	SET_IMM_REGW r31,r30,LBLAWAR2_OFF,LBLAWAR2_VAL
#endif
#ifdef LBLAWBAR3_VAL
	SET_IMM_REGW r31,r30,LBLAWBAR3_OFF,LBLAWBAR3_VAL
#endif
#ifdef LBLAWAR3_VAL
	SET_IMM_REGW r31,r30,LBLAWAR3_OFF,LBLAWAR3_VAL
#endif
	/*
	 * ROM startup: init bus system
	 */
#ifdef BR0_VAL
	SET_IMM_REGW r31,r30,BR0_OFF,BR0_VAL
#endif
#ifdef OR0_VAL
	SET_IMM_REGW r31,r30,OR0_OFF,OR0_VAL
#endif
#ifdef BR1_VAL
	SET_IMM_REGW r31,r30,BR1_OFF,BR1_VAL
#endif
#ifdef OR1_VAL
	SET_IMM_REGW r31,r30,OR1_OFF,OR1_VAL
#endif
#ifdef BR2_VAL
	SET_IMM_REGW r31,r30,BR2_OFF,BR2_VAL
#endif
#ifdef OR2_VAL
	SET_IMM_REGW r31,r30,OR2_OFF,OR2_VAL
#endif
#ifdef BR3_VAL
	SET_IMM_REGW r31,r30,BR3_OFF,BR3_VAL
#endif
#ifdef OR3_VAL
	SET_IMM_REGW r31,r30,OR3_OFF,OR3_VAL
#endif
#ifdef BR4_VAL
	SET_IMM_REGW r31,r30,BR4_OFF,BR4_VAL
#endif
#ifdef OR4_VAL
	SET_IMM_REGW r31,r30,OR4_OFF,OR4_VAL
#endif
#ifdef BR5_VAL
	SET_IMM_REGW r31,r30,BR5_OFF,BR5_VAL
#endif
#ifdef OR5_VAL
	SET_IMM_REGW r31,r30,OR5_OFF,OR5_VAL
#endif
	/*
	 * ROM startup: init SDRAM access window
	 */
#ifdef DDRLAWBAR0_VAL
	SET_IMM_REGW r31,r30,DDRLAWBAR0_OFF,DDRLAWBAR0_VAL
#endif
#ifdef DDRLAWAR0_VAL
	SET_IMM_REGW r31,r30,DDRLAWAR0_OFF,DDRLAWAR0_VAL
#endif
#ifdef DDRLAWBAR1_VAL
	SET_IMM_REGW r31,r30,DDRLAWBAR1_OFF,DDRLAWBAR1_VAL
#endif
#ifdef DDRLAWAR1_VAL
	SET_IMM_REGW r31,r30,DDRLAWAR1_OFF,DDRLAWAR1_VAL
#endif
	/*
	 * ROM startup: init refresh interval
	 */
#ifdef MRPTR_VAL
	SET_IMM_REGW r31,r30,MRPTR_OFF,MRPTR_VAL
#endif
	/*
	 * ROM startup: init SDRAM
	 */
#ifdef LSRT_VAL
	SET_IMM_REGW r31,r30, LSRT_OFF, LSRT_VAL
#endif
#ifdef LSDMR_VAL
	SET_IMM_REGW r31,r30, LSDMR_OFF, LSDMR_VAL
#endif
#ifdef CS0_BNDS_VAL
	SET_IMM_REGW r31,r30,CS0_BNDS_OFF,CS0_BNDS_VAL
#endif
#ifdef CS1_BNDS_VAL
	SET_IMM_REGW r31,r30,CS1_BNDS_OFF,CS1_BNDS_VAL
#endif
#ifdef CS2_BNDS_VAL
	SET_IMM_REGW r31,r30,CS2_BNDS_OFF,CS2_BNDS_VAL
#endif
#ifdef CS3_BNDS_VAL
	SET_IMM_REGW r31,r30,CS3_BNDS_OFF,CS3_BNDS_VAL
#endif
#ifdef CS0_CONFIG_VAL
	SET_IMM_REGW r31,r30,CS0_CONFIG_OFF,CS0_CONFIG_VAL
#endif
#ifdef CS1_CONFIG_VAL
	SET_IMM_REGW r31,r30,CS1_CONFIG_OFF,CS1_CONFIG_VAL
#endif
#ifdef CS2_CONFIG_VAL
	SET_IMM_REGW r31,r30,CS2_CONFIG_OFF,CS2_CONFIG_VAL
#endif
#ifdef CS3_CONFIG_VAL
	SET_IMM_REGW r31,r30,CS3_CONFIG_OFF,CS3_CONFIG_VAL
#endif
#ifdef TIMING_CFG_3_VAL
	SET_IMM_REGW r31,r30,TIMING_CFG_3_OFF,TIMING_CFG_3_VAL
#endif
#ifdef TIMING_CFG_0_VAL
	SET_IMM_REGW r31,r30,TIMING_CFG_0_OFF,TIMING_CFG_0_VAL
#endif
#ifdef TIMING_CFG_1_VAL
	SET_IMM_REGW r31,r30,TIMING_CFG_1_OFF,TIMING_CFG_1_VAL
#endif
#ifdef TIMING_CFG_2_VAL
	SET_IMM_REGW r31,r30,TIMING_CFG_2_OFF,TIMING_CFG_2_VAL
#endif
#ifdef DDRCDR_VAL
	SET_IMM_REGW r31,r30,DDRCDR_OFF,DDRCDR_VAL
#endif
#ifdef DDR_SDRAM_CFG_2_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_CFG_2_OFF,DDR_SDRAM_CFG_2_VAL
#endif
#ifdef DDR_SDRAM_MODE_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_MODE_OFF,DDR_SDRAM_MODE_VAL
#endif
#ifdef DDR_SDRAM_MODE_2_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_MODE_2_OFF,DDR_SDRAM_MODE_2_VAL
#endif
#ifdef DDR_SDRAM_MD_CNTL_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_MD_CNTL_OFF,DDR_SDRAM_MD_CNTL_VAL
#endif
#ifdef DDR_SDRAM_MD_ITVL_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_MD_ITVL_OFF,DDR_SDRAM_MD_ITVL_VAL
#endif
#ifdef DDR_SDRAM_CLK_CNTL_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_CLK_CNTL_OFF,DDR_SDRAM_CLK_CNTL_VAL
#endif
#ifdef DDR_SDRAM_CFG_2_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_CFG_2_OFF,DDR_SDRAM_CFG_2_VAL|DDR_SDRAM_CFG_2_D_INIT
#endif

#ifdef DDR_ERR_DISABLE_VAL
	/*
	 * disable detect of RAM errors
	 */
	SET_IMM_REGW r31,r30,DDR_ERR_DISABLE_OFF,DDR_ERR_DISABLE_VAL
#endif
#ifdef DDR_SDRAM_DATA_INIT_VAL
	/*
	 * set this value to initialize memory
	 */
	SET_IMM_REGW r31,r30,DDR_SDRAM_DATA_INIT_OFF,DDR_SDRAM_DATA_INIT_VAL
#endif
#ifdef DDR_SDRAM_INIT_ADDR_VAL
	SET_IMM_REGW r31,r30,DDR_SDRAM_INIT_ADDR_OFF,DDR_SDRAM_INIT_ADDR_VAL
#endif
#ifdef DDR_SDRAM_CFG_VAL
	/*
	 * config DDR SDRAM
	 */
	SET_IMM_REGW r31,r30,DDR_SDRAM_CFG_OFF,DDR_SDRAM_CFG_VAL & ~DDR_SDRAM_CFG_MEM_EN
	/*
	 * FIXME: wait 200us
	 */
	/*
	 * enable  DDR SDRAM
	 */
	SET_IMM_REGW r31,r30,DDR_SDRAM_CFG_OFF,DDR_SDRAM_CFG_VAL | DDR_SDRAM_CFG_MEM_EN
	/*
	 * wait, until DDR_SDRAM_CFG_2_D_INIT is cleared
	 */
1:	lwz r30,DDR_SDRAM_CFG_2_OFF(r31)
	andi. r30,r30,DDR_SDRAM_CFG_2_D_INIT
	bne 1b
#endif
#ifdef DDR_ERR_DISABLE_VAL2
	/*
	 * enable detect of some RAM errors
	 */
	SET_IMM_REGW r31,r30,DDR_ERR_DISABLE_OFF,DDR_ERR_DISABLE_VAL2
#endif
#ifdef DDR_SDRAM_INTERVAL_VAL
	/*
	 * set the refresh interval
	 */
	SET_IMM_REGW r31,r30,DDR_SDRAM_INTERVAL_OFF,DDR_SDRAM_INTERVAL_VAL
#endif
start_rom_skip:
	/*
	 * determine current execution address offset
	 */
	bl start_rom_skip1
start_rom_skip1:
	mflr r20
	LA   r30,start_rom_skip1
	sub. r20,r20,r30
	/*
	 * execution address offset == 0?
	 * then do not relocate code and data
	 */
	beq  start_code_in_ram
	/*
	 * ROM or relocatable startup: copy startup code to SDRAM
	 */
	/* get start address of start section in RAM */
	LA	r29, bsp_section_start_begin
	/* get start address of start section in ROM (add reloc offset) */
	add	r30, r20, r29
	/* get size of startup code */
	LA	r28, bsp_section_start_end
	sub	28,r28,r29
	/* copy startup code from ROM to RAM location */
	bl	copy_image

	/*
	 * ROM startup: jump to code copy in  SDRAM
	 */
	/* get compile time address of label */
	LA	r29, copy_rest_of_text
	mtlr	r29
	blr                     /* now further execution RAM */
copy_rest_of_text:
	LWI  r31,IMMRBAR
#ifdef LCRR_VAL
	SET_IMM_REGW r31,r30,LCRR_OFF,LCRR_VAL
#endif
	/*
	 * ROM or relocatable startup: copy rest of code to SDRAM
	 */
	/* get start address of rest of loadable sections in RAM */
	LA	r29, bsp_section_text_begin
	/* get start address of loadable sections in ROM (add reloc offset) */
	add	r30, r20, r29
	/* get size of rest of loadable sections */
	LA	r28, bsp_section_data_end
	sub	r28,r28,r29
	bl	copy_image      /* copy text section from ROM to RAM location */

start_code_in_ram:

	/*
	 * ROM/RAM startup: clear bss in SDRAM
	 */
	LA	r3, bsp_section_sbss_begin  /* get start address of bss section */
	LA	r4, bsp_section_bss_end     /* get end address of bss section */
	sub	r4, r4, r3                  /* get size of bss section */
	bl	mpc83xx_zero_4              /* Clear the bss section */

#ifdef HAS_UBOOT
	mr	r3, r14
	bl	bsp_uboot_copy_board_info
#endif /* HAS_UBOOT */

	/* Read-only small data */
	LA r2, _SDA2_BASE_

	/* Read-write small data */
	LA r13, _SDA_BASE_

	/* Clear cmdline */
	li	r3, 0

	/*
	 * Initialize start stack.  The stacks are statically allocated and
	 * properly aligned.
	 */
	LA	r1, _ISR_Stack_area_end
	subi	r1, r1, PPC_DEFAULT_CACHE_LINE_SIZE
	stw	r3, 0(r1)

	/* Call the first C routine */
        bl      SYM (boot_card)

twiddle:
	/* We don't expect to return from boot_card but if we do */
	/* wait here for watchdog to kick us into hard reset     */
	b	twiddle

copy_image:
	cmpwi	r28, 0
	beqlr

	mr	r27, r28
	srwi	r28, r28, 2
	mtctr	r28

	slwi	r28, r28, 2
	sub     r27, r27, r28			/* maybe some residual bytes */
copy_image_word:
	lswi	r28, r30, 0x04

	stswi   r28, r29, 0x04			/* do word copy ROM -> RAM */


	addi    r30, r30, 0x04			/* increment source pointer */
	addi    r29, r29, 0x04			/* increment destination pointer */

	bdnz	copy_image_word			/* decrement ctr and branch if not 0 */

	cmpwi	r27, 0x00			/* copy image finished ? */
	beq	copy_image_end;
	mtctr   r27				/* reload counter for residual bytes */
copy_image_byte:
	lswi	r28, r30, 0x01

	stswi   r28, r29, 0x01			/* do byte copy ROM -> RAM */


	addi    r30, r30, 0x01			/* increment source pointer */
	addi    r29, r29, 0x01			/* increment destination pointer */

	bdnz	copy_image_byte			/* decrement ctr and branch if not 0 */

copy_image_end:
	blr


/**
 * @fn int mpc83xx_zero_4( void *dest, size_t n)
 *
 * @brief Zero all @a n bytes starting at @a dest with 4 byte writes.
 *
 * The address @a dest has to be aligned on 4 byte boundaries.  The size @a n
 * must be evenly divisible by 4.
 */
GLOBAL_FUNCTION mpc83xx_zero_4
	/* Create zero */
	xor	r0, r0, r0

	/* Set offset */
	xor	r5, r5, r5

	/* Loop counter for the first bytes up to 16 bytes */
	rlwinm.	r9, r4, 30, 30, 31
	beq	mpc83xx_zero_4_more
	mtctr	r9

mpc83xx_zero_4_head:

	stwx	r0, r3, r5
	addi	r5, r5, 4
	bdnz	mpc83xx_zero_4_head

mpc83xx_zero_4_more:

	/* More than 16 bytes? */
	srwi.	r9, r4, 4
	beqlr
	mtctr	r9

	/* Set offsets */
	addi	r6, r5, 4
	addi	r7, r5, 8
	addi	r8, r5, 12

mpc83xx_zero_4_tail:

	stwx	r0, r3, r5
	addi	r5, r5, 16
	stwx	r0, r3, r6
	addi	r6, r6, 16
	stwx	r0, r3, r7
	addi	r7, r7, 16
	stwx	r0, r3, r8
	addi	r8, r8, 16
	bdnz	mpc83xx_zero_4_tail

	/* Return */
	blr
