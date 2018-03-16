/************************************************************************************
 * arch/arm/src/imxrt/imxrt_ccm.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Janne Rosberg <janne@offcode.fi>
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

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_CCM_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_CCM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define IMXRT_CCM_CCR_OFFSET        0x0000  /* CCM Control Register */
                                 /* 0x0004  Reserved */
#define IMXRT_CCM_CSR_OFFSET        0x0008  /* CCM Status Register */
#define IMXRT_CCM_CCSR_OFFSET       0x000C  /* CCM Clock Switcher Register */
#define IMXRT_CCM_CACRR_OFFSET      0x0010  /* CCM Arm Clock Root Register */
#define IMXRT_CCM_CBCDR_OFFSET      0x0014  /* CCM Bus Clock Divider Register */
#define IMXRT_CCM_CBCMR_OFFSET      0x0018  /* CCM Bus Clock Multiplexer Register */
#define IMXRT_CCM_CSCMR1_OFFSET     0x001C  /* CCM Serial Clock Multiplexer Register 1 */
#define IMXRT_CCM_CSCMR2_OFFSET     0x0020  /* CCM Serial Clock Multiplexer Register 2 */
#define IMXRT_CCM_CSCDR1_OFFSET     0x0024  /* CCM Serial Clock Divider Register 1 */
#define IMXRT_CCM_CS1CDR_OFFSET     0x0028  /* CCM Clock Divider Register */
#define IMXRT_CCM_CS2CDR_OFFSET     0x002C  /* CCM Clock Divider Register */
#define IMXRT_CCM_CDCDR_OFFSET      0x0030  /* CCM D1 Clock Divider Register */
                                 /* 0x0034  Reserved */
#define IMXRT_CCM_CSCDR2_OFFSET     0x0038  /* CCM Serial Clock Divider Register 2 */
#define IMXRT_CCM_CSCDR3_OFFSET     0x003C  /* CCM Serial Clock Divider Register 3 */
                                 /* 0x0040  Reserved */
                                 /* 0x0044  Reserved */
#define IMXRT_CCM_CDHIPR_OFFSET     0x0048  /* CCM Divider Handshake In-Process Register */
                                 /* 0x004C  Reserved */
                                 /* 0x0050  Reserved */
#define IMXRT_CCM_CLPCR_OFFSET      0x0054  /* CCM Low Power Control Register */

#define IMXRT_CCM_CISR_OFFSET       0x0058  /* CCM Interrupt Status Register */
#define IMXRT_CCM_CIMR_OFFSET       0x005C  /* CCM Interrupt Mask Register */
#define IMXRT_CCM_CCOSR_OFFSET      0x0060  /* CCM Clock Output Source Register */
#define IMXRT_CCM_CGPR_OFFSET       0x0064  /* CCM General Purpose Register */
#define IMXRT_CCM_CCGR0_OFFSET      0x0068  /* CCM Clock Gating Register 0 */
#define IMXRT_CCM_CCGR1_OFFSET      0x006C  /* CCM Clock Gating Register 1 */
#define IMXRT_CCM_CCGR2_OFFSET      0x0070  /* CCM Clock Gating Register 2 */
#define IMXRT_CCM_CCGR3_OFFSET      0x0074  /* CCM Clock Gating Register 3 */
#define IMXRT_CCM_CCGR4_OFFSET      0x0078  /* CCM Clock Gating Register 4 */
#define IMXRT_CCM_CCGR5_OFFSET      0x007C  /* CCM Clock Gating Register 5 */
#define IMXRT_CCM_CCGR6_OFFSET      0x0080  /* CCM Clock Gating Register 6 */
                                 /* 0x0084  Reserved */
#define IMXRT_CCM_CMEOR_OFFSET      0x0088  /* CCM Module Enable Overide Register */

/* Register addresses ***************************************************************/

#define IMXRT_CCM_CCR               (IMXRT_CCM_BASE+IMXRT_CCM_CCR_OFFSET)
#define IMXRT_CCM_CSR               (IMXRT_CCM_BASE+IMXRT_CCM_CSR_OFFSET)
#define IMXRT_CCM_CCSR              (IMXRT_CCM_BASE+IMXRT_CCM_CCSR_OFFSET)
#define IMXRT_CCM_CACRR             (IMXRT_CCM_BASE+IMXRT_CCM_CACRR_OFFSET)
#define IMXRT_CCM_CBCDR             (IMXRT_CCM_BASE+IMXRT_CCM_CBCDR_OFFSET)
#define IMXRT_CCM_CBCMR             (IMXRT_CCM_BASE+IMXRT_CCM_CBCMR_OFFSET)
#define IMXRT_CCM_CSCMR1            (IMXRT_CCM_BASE+IMXRT_CCM_CSCMR1_OFFSET)
#define IMXRT_CCM_CSCMR2            (IMXRT_CCM_BASE+IMXRT_CCM_CSCMR2_OFFSET)
#define IMXRT_CCM_CSCDR1            (IMXRT_CCM_BASE+IMXRT_CCM_CSCDR1_OFFSET)
#define IMXRT_CCM_CS1CDR            (IMXRT_CCM_BASE+IMXRT_CCM_CS1CDR_OFFSET)
#define IMXRT_CCM_CS2CDR            (IMXRT_CCM_BASE+IMXRT_CCM_CS2CDR_OFFSET)
#define IMXRT_CCM_CDCDR             (IMXRT_CCM_BASE+IMXRT_CCM_CDCDR_OFFSET)
#define IMXRT_CCM_CSCDR2            (IMXRT_CCM_BASE+IMXRT_CCM_CSCDR2_OFFSET)
#define IMXRT_CCM_CSCDR3            (IMXRT_CCM_BASE+IMXRT_CCM_CSCDR3_OFFSET)
#define IMXRT_CCM_CDHIPR            (IMXRT_CCM_BASE+IMXRT_CCM_CDHIPR_OFFSET)
#define IMXRT_CCM_CLPCR             (IMXRT_CCM_BASE+IMXRT_CCM_CLPCR_OFFSET)
#define IMXRT_CCM_CISR              (IMXRT_CCM_BASE+IMXRT_CCM_CISR_OFFSET)
#define IMXRT_CCM_CIMR              (IMXRT_CCM_BASE+IMXRT_CCM_CIMR_OFFSET)
#define IMXRT_CCM_CCOSR             (IMXRT_CCM_BASE+IMXRT_CCM_CCOSR_OFFSET)
#define IMXRT_CCM_CGPR              (IMXRT_CCM_BASE+IMXRT_CCM_CGPR_OFFSET)
#define IMXRT_CCM_CCGR0             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR0_OFFSET)
#define IMXRT_CCM_CCGR1             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR1_OFFSET)
#define IMXRT_CCM_CCGR2             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR2_OFFSET)
#define IMXRT_CCM_CCGR3             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR3_OFFSET)
#define IMXRT_CCM_CCGR4             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR4_OFFSET)
#define IMXRT_CCM_CCGR5             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR5_OFFSET)
#define IMXRT_CCM_CCGR6             (IMXRT_CCM_BASE+IMXRT_CCM_CCGR6_OFFSET)
#define IMXRT_CCM_CMEOR             (IMXRT_CCM_BASE+IMXRT_CCM_CMEOR_OFFSET)

/* Register bit definitions *********************************************************/

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_CCM_H */

