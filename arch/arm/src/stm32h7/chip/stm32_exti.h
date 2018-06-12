/************************************************************************************
 * arch/arm/src/stm32h7/chip/stm32_exti.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_STM32H7_CHIP_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32H7_CHIP_STM32_EXTI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/* Content of this file requires verification before it is used with other
 * families
 */

#if defined(CONFIG_STM32H7_STM32H7X3XX)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define STM32_NEXTI                22
#define STM32_EXTI_MASK            0x003fffff
#define STM32_EXTI_BIT(n)          (1 << (n))

/* Register Offsets *****************************************************************/

#define STM32_EXTI_RTSR1_OFFSET    0x0000  /* Rising Trigger selection register 1 */
#define STM32_EXTI_FTSR1_OFFSET    0x0004  /* Falling Trigger selection register 1 */
#define STM32_EXTI_SWIER1_OFFSET   0x0008  /* Software interrupt event register 1 */
#define STM32_EXTI_D3PMR1_OFFSET   0x000c  /* D3 pending mask register 1 */
#define STM32_EXTI_D3PCR1L_OFFSET  0x0010  /* D3 pending clear selection register low 1 */
#define STM32_EXTI_D3PCR1H_OFFSET  0x0014  /* D3 pending clear selection register high 1 */

#define STM32_EXTI_RTSR2_OFFSET    0x0020  /* Rising Trigger selection register 2 */
#define STM32_EXTI_FTSR2_OFFSET    0x0024  /* Falling Trigger selection register 2 */
#define STM32_EXTI_WIER2_OFFSET    0x0028  /* Software interrupt event register 2 */
#define STM32_EXTI_D3PMR2_OFFSET   0x002c  /* D3 pending mask register 2 */
#define STM32_EXTI_D3PCR2L_OFFSET  0x0030  /* D3 pending clear selection register low 2 */
#define STM32_EXTI_D3PCR2H_OFFSET  0x0034  /* D3 pending clear selection register high 2 */

#define STM32_EXTI_RTSR3_OFFSET    0x0040  /* Rising Trigger selection register 3 */
#define STM32_EXTI_FTSR3_OFFSET    0x0044  /* Falling Trigger selection register 3 */
#define STM32_EXTI_WIER3_OFFSET    0x0048  /* Software interrupt event register 3 */
#define STM32_EXTI_D3PMR3_OFFSET   0x004c  /* D3 pending mask register 3 */
#define STM32_EXTI_D3PCR3L_OFFSET  0x0050  /* D3 pending clear selection register low 3 */
#define STM32_EXTI_D3PCR3H_OFFSET  0x0054  /* D3 pending clear selection register high 3 */

#define STM32_EXTI_CPUIMR1_OFFSET  0x0080  /* EXTI interrupt mask register 1 */
#define STM32_EXTI_CPUEMR1_OFFSET  0x0084  /* EXTI event mask register 1 */
#define STM32_EXTI_CPUPR1_OFFSET   0x0088  /* EXTI pending register 1 */

#define STM32_EXTI_CPUIMR2_OFFSET  0x0090  /* EXTI interrupt mask register 2 */
#define STM32_EXTI_CPUEMR2_OFFSET  0x0094  /* EXTI event mask register 2 */
#define STM32_EXTI_CPUPR2_OFFSET   0x0098  /* EXTI pending register 2 */

#define STM32_EXTI_CPUIMR3_OFFSET  0x00a0  /* EXTI interrupt mask register 3 */
#define STM32_EXTI_CPUEMR3_OFFSET  0x00a4  /* EXTI event mask register 3 */
#define STM32_EXTI_CPUPR3_OFFSET   0x00a8  /* EXTI pending register 3 */

/* Register Addresses ***************************************************************/

#define STM32_EXTI_RTSR1           (STM32_EXTI_BASE + STM32_EXTI_RTSR1_OFFSET)
#define STM32_EXTI_FTSR1           (STM32_EXTI_BASE + STM32_EXTI_FTSR1_OFFSET)
#define STM32_EXTI_SWIER1          (STM32_EXTI_BASE + STM32_EXTI_SWIER1_OFFSET)
#define STM32_EXTI_D3PMR1          (STM32_EXTI_BASE + STM32_EXTI_D3PMR1_OFFSET)
#define STM32_EXTI_D3PCR1L         (STM32_EXTI_BASE + STM32_EXTI_D3PCR1L_OFFSET)
#define STM32_EXTI_D3PCR1H         (STM32_EXTI_BASE + STM32_EXTI_D3PCR1H_OFFSET)

#define STM32_EXTI_RTSR2           (STM32_EXTI_BASE + STM32_EXTI_RTSR2_OFFSET)
#define STM32_EXTI_FTSR2           (STM32_EXTI_BASE + STM32_EXTI_FTSR2_OFFSET)
#define STM32_EXTI_WIER2           (STM32_EXTI_BASE + STM32_EXTI_WIER2_OFFSET)
#define STM32_EXTI_D3PMR2          (STM32_EXTI_BASE + STM32_EXTI_D3PMR2_OFFSET)
#define STM32_EXTI_D3PCR2L         (STM32_EXTI_BASE + STM32_EXTI_D3PCR2L_OFFSET)
#define STM32_EXTI_D3PCR2H         (STM32_EXTI_BASE + STM32_EXTI_D3PCR2H_OFFSET)

#define STM32_EXTI_RTSR3           (STM32_EXTI_BASE + STM32_EXTI_RTSR3_OFFSET)
#define STM32_EXTI_FTSR3           (STM32_EXTI_BASE + STM32_EXTI_FTSR3_OFFSET)
#define STM32_EXTI_WIER3           (STM32_EXTI_BASE + STM32_EXTI_WIER3_OFFSET)
#define STM32_EXTI_D3PMR3          (STM32_EXTI_BASE + STM32_EXTI_D3PMR3_OFFSET)
#define STM32_EXTI_D3PCR3L         (STM32_EXTI_BASE + STM32_EXTI_D3PCR3L_OFFSET)
#define STM32_EXTI_D3PCR3H         (STM32_EXTI_BASE + STM32_EXTI_D3PCR3H_OFFSET)

#define STM32_EXTI_CPUIMR1         (STM32_EXTI_BASE + STM32_EXTI_CPUIMR1_OFFSET)
#define STM32_EXTI_CPUEMR1         (STM32_EXTI_BASE + STM32_EXTI_CPUEMR1_OFFSET)
#define STM32_EXTI_CPUPR1          (STM32_EXTI_BASE + STM32_EXTI_CPUPR1_OFFSET)

#define STM32_EXTI_CPUIMR2         (STM32_EXTI_BASE + STM32_EXTI_CPUIMR2_OFFSET)
#define STM32_EXTI_CPUEMR2         (STM32_EXTI_BASE + STM32_EXTI_CPUEMR2_OFFSET)
#define STM32_EXTI_CPUPR2          (STM32_EXTI_BASE + STM32_EXTI_CPUPR2_OFFSET)

#define STM32_EXTI_CPUIMR3         (STM32_EXTI_BASE + STM32_EXTI_CPUIMR3_OFFSET)
#define STM32_EXTI_CPUEMR3         (STM32_EXTI_BASE + STM32_EXTI_CPUEMR3_OFFSET)
#define STM32_EXTI_CPUPR3          (STM32_EXTI_BASE + STM32_EXTI_CPUPR3_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Rising Trigger selection register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_RTSR1_

/* Falling Trigger selection register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_FTSR1_

/* Software interrupt event register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_SWIER1_

/* D3 pending mask register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PMR1_

/* D3 pending clear selection register low 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PCR1L_

/* D3 pending clear selection register high 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PCR1H_

/* Rising Trigger selection register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_RTSR2_

/* Falling Trigger selection register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_FTSR2_

/* Software interrupt event register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_WIER2_

/* D3 pending mask register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PMR2_

/* D3 pending clear selection register low 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PCR2L_

/* D3 pending clear selection register high 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PCR2H_

/* Rising Trigger selection register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_RTSR3_

/* Falling Trigger selection register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_FTSR3_

/* Software interrupt event register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_WIER3_

/* D3 pending mask register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PMR3_

/* D3 pending clear selection register low 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PCR3L_

/* D3 pending clear selection register high 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_D3PCR3H_

/* EXTI interrupt mask register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUIMR1_

/* EXTI event mask register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUEMR1_

/* EXTI pending register 1 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUPR1_

/* EXTI interrupt mask register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUIMR2_

/* EXTI event mask register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUEMR2_

/* EXTI pending register 2 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUPR2_

/* EXTI interrupt mask register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUIMR3_

/* EXTI event mask register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUEMR3_

/* EXTI pending register 3 */
/* REVISIT:  Mising bit-field definitions */
#define EXTI_CPUPR3_

#endif /* CONFIG_STM32H7_STM32H7X3XX */
#endif /* __ARCH_ARM_SRC_STM32H7_CHIP_STM32_EXTI_H */
