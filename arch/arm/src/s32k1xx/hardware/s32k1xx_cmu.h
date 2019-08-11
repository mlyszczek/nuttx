/************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_cmu.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/************************************************************************************
 * Included Files
 ************************************************************************************/

/* CMU Register Offsets *************************************************************/

#define S32K1XX_CMU_GCR_OFFSET   0x0000  /* Global Configuration Register */
#define S32K1XX_CMU_RCCR_OFFSET  0x0004  /* Reference Count Configuration Register */
#define S32K1XX_CMU_HTCR_OFFSET  0x0008  /* High Threshold Configuration Register */
#define S32K1XX_CMU_LTCR_OFFSET  0x000c  /* Low Threshold Configuration Register */
#define S32K1XX_CMU_SR_OFFSET    0x0010  /* Status Register */
#define S32K1XX_CMU_IER_OFFSET   0x0014  /* Interrupt Enable Register */

/* CMU Register Addresses ***********************************************************/

#define S32K1XX_CMU_GCR          (S32K1XX_CMU_BASE + S32K1XX_CMU_GCR_OFFSET)
#define S32K1XX_CMU_RCCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_RCCR_OFFSET)
#define S32K1XX_CMU_HTCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_HTCR_OFFSET)
#define S32K1XX_CMU_LTCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_LTCR_OFFSET)
#define S32K1XX_CMU_SR           (S32K1XX_CMU_BASE + S32K1XX_CMU_SR_OFFSET)
#define S32K1XX_CMU_IER          (S32K1XX_CMU_BASE + S32K1XX_CMU_IER_OFFSET)

/* CMU Register Bitfield Definitions ************************************************/

/* Global Configuration Register */
#define CMU_GCR_
/* Reference Count Configuration Register */
#define CMU_RCCR_
/* High Threshold Configuration Register */
#define CMU_HTCR_
/* Low Threshold Configuration Register */
#define CMU_LTCR_
/* Status Register */
#define CMU_SR_
/* Interrupt Enable Register */
#define CMU_IER_

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H */
