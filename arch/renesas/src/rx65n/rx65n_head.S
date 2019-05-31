/*****************************************************************************
 * arch/renesas/src/rx65n/rx65n_head.S
 *
 *
 *****************************************************************************/
 
 
/*****************************************************************************
 * Included Files
 *****************************************************************************/
#include <nuttx/config.h>		/* NuttX configuration settings */
#include <arch/board/board.h>	/* Board-specific settings */
#include <arch/irq.h>			/* IRQ definitons */
#include "chip.h"				/* Chip-specific settings */
#include "up_internal.h"
#include "up_arch.h"


/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/
/* This file holds the NuttX start logic that runs when the SH-1/US7032EVB1
 * is reset.  This logic must be located in SRAM at 0x0a00:2000.  On that
 * platform, the entire PROM and the first 8Kb of SRAM are reserved for CMON.
 */
 
 
/*****************************************************************************
 * External references
 *****************************************************************************/
/* Called functions */
	.globl	_up_lowsetup				/* Early initialization of UART */
#ifdef USE_EARLYSERIALINIT
	.globl	_up_earlyconsoleinit		/* Early initialization of console driver */
#endif
#ifdef CONFIG_ARCH_LEDS
	.globl	_board_autoled_initialize	/* Boot LED setup */
#endif
#ifdef CONFIG_DEBUG_FEATURES
	.globl	_up_lowputc					/* Low-level debug output */
#endif
	.globl	_nx_start					/* NuttX entry point */
/* Variables set up by the linker script */
	.globl _HardwareSetup  /*external Sub-routine to initialise Hardware */
	.globl _up_lowsetup
	.globl _data
	.globl _mdata
	.globl _ebss
	.globl _bss
	.globl _edata
	.globl _ustack
	.globl _istack
	
	
/*****************************************************************************
 * Macros
 *****************************************************************************/
/*****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status. This macro will
 *   modify r0, r1, r2 and r14
 *
 *****************************************************************************/
	.macro	showprogress, code
#ifdef CONFIG_DEBUG_FEATURES
	mov.l	#_up_lowputc, r13			/* Address of up_earlyconsoleinit */
	mov		#\code, r1
	jsr		r13							/* Call it */
#endif
	.endm
	
	
/************************************************************************************
 * Code
 ************************************************************************************/
	.section	.text
/************************************************************************************
 * Name: __start
 *
 * Description:
 *   After reset, program execution starts here.
 *
 ************************************************************************************/
	.text
	.globl	__start
	.globl	_PowerON_Reset
	.type	 __start, #function
	.type	_PowerON_Reset, #function
__start:
_PowerON_Reset:
	mvtc	#_ustack,USP			/* initialise user stack pointer				*/
	mvtc	#_istack,ISP			/* initialise interrupt stack pointer			*/
#ifdef __RXv2__
	mvtc	#_ExceptVectors, extb	/* setup exception vector (extb)				*/
#endif
	mvtc	#__vector_table, intb	/* setup interrupt vector (intb)				*/
	mvtc	#100h, fpsw				/* setup FPSW									*/
	/*
	 *	load data section from ROM to RAM
	 */
	mov		#_mdata, r2				/* src ROM address of data section in R2		*/
	mov		#_data, r1				/* dest start RAM address of data section in R1	*/
	mov		#_edata, r3				/* end RAM address of data section in R3		*/
	sub		r1, r3					/* size of data section in R3 (R3=R3-R1)		*/
	smovf							/* block copy R3 bytes from R2 to R1			*/
	/*
	 *	bss initialisation : zero out bss
	 */
	mov		#00h, r2				/* load R2 reg with zero						*/
	mov		#_ebss, r3				/* store the end address of bss in R3			*/
	mov		#_bss, r1				/* store the start address of bss in R1			*/
	sub		r1, r3					/* size of bss section in R3 (R3=R3-R1)			*/
	sstr.b
	/*
	 *	call the hardware initialiser
	 */
	bsr.a	_HardwareSetup
	nop
	mvtc	#30000h, psw			/*Set Ubit & Ibit for PSW						*/
	mov.l	#_up_lowsetup, r15
	jsr		r15						/* Call up_lowsetup								*/
	nop
#ifdef USE_EARLYSERIALINIT
	mov.l	#_up_earlyconsoleinit, r14
	jsr		r14						/* Call up_earlyconsoleinit						*/
	nop
#endif
	bsr.a	_nx_start
	nop
_exit:
	nop
_loop_here:
	bra		_loop_here
	nop

/************************************************************************************
 * Data
 ************************************************************************************/
	.section	.data
	.data
	.align	4

	.global	_g_idle_topstack
	.type	_g_idle_topstack, @object
_g_idle_topstack:
	.long	_ebss + 1024
	.size	_g_idle_topstack, .-_g_idle_topstack
    .end

/************************************************************************************
 * EOF
 ************************************************************************************/
