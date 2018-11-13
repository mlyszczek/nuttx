/************************************************************************************************
 * arch/arm/include/max326xx/max32620.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_MAX326XX_MAX32630_IRQ_H
#define __ARCH_ARM_INCLUDE_MAX326XX_MAX32630_IRQ_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* External interrupts (vectors >= 16) */

#define MAX326XX_IRQ_CRYPTO     (MAX326XX_IRQ_EXTINT + 0)    /*  0  Crypto Oscillator Stable */
#define MAX326XX_IRQ_SUPPLY     (MAX326XX_IRQ_EXTINT + 1)    /*  1  Power Supply Reset */
#define MAX326XX_IRQ_FLASHC     (MAX326XX_IRQ_EXTINT + 2)    /*  2  SRAM/FLASH Controller */
#define MAX326XX_IRQ_ALARM0     (MAX326XX_IRQ_EXTINT + 3)    /*  3  RTC Time-of-Day Alarm 0 */
#define MAX326XX_IRQ_ALARM1     (MAX326XX_IRQ_EXTINT + 4)    /*  4  RTC Time-of-Day Alarm 1 */
#define MAX326XX_IRQ_INTERVAL   (MAX326XX_IRQ_EXTINT + 5)    /*  5  RTC Interval Alarm */
#define MAX326XX_IRQ_RTCOVF     (MAX326XX_IRQ_EXTINT + 6)    /*  6  RTC Counter Overflow */
#define MAX326XX_IRQ_PMU        (MAX326XX_IRQ_EXTINT + 7)    /*  7  PMU */
#define MAX326XX_IRQ_USB        (MAX326XX_IRQ_EXTINT + 8)    /*  8  USB 2.0 */
#define MAX326XX_IRQ_AES        (MAX326XX_IRQ_EXTINT + 9)    /*  9  AES Interrupt */
#define MAX326XX_IRQ_MMA        (MAX326XX_IRQ_EXTINT + 10)   /* 10  MMA */
#define MAX326XX_IRQ_WDT0       (MAX326XX_IRQ_EXTINT + 11)   /* 11  Watchdog 0 Timeout Interrupt */
#define MAX326XX_IRQ_WDT0PRE    (MAX326XX_IRQ_EXTINT + 12)   /* 12  Watchdog 0 Pre-window Interrupt */
#define MAX326XX_IRQ_WDT1       (MAX326XX_IRQ_EXTINT + 13)   /* 13  Watchdog 1 Timeout Interrupt */
#define MAX326XX_IRQ_WDT1PRE    (MAX326XX_IRQ_EXTINT + 14)   /* 14  Watchdog 1 Pre-window Interrupt */
#define MAX326XX_IRQ_GPIO0      (MAX326XX_IRQ_EXTINT + 15)   /* 15  GPIO P0.1-7 External Interrupts */
#define MAX326XX_IRQ_GPIO1      (MAX326XX_IRQ_EXTINT + 16)   /* 16  GPIO P1.1-7 External Interrupts */
#define MAX326XX_IRQ_GPIO2      (MAX326XX_IRQ_EXTINT + 17)   /* 17  GPIO P2.1-7 External Interrupts */
#define MAX326XX_IRQ_GPIO3      (MAX326XX_IRQ_EXTINT + 18)   /* 18  GPIO P3.1-7 External Interrupts */
#define MAX326XX_IRQ_GPIO4      (MAX326XX_IRQ_EXTINT + 19)   /* 19  GPIO P4.1-7 External Interrupts */
#define MAX326XX_IRQ_GPIO5      (MAX326XX_IRQ_EXTINT + 20)   /* 20  GPIO P5.1-7 External Interrupts */
#define MAX326XX_IRQ_GPIO6      (MAX326XX_IRQ_EXTINT + 21)   /* 21  GPIO P6.1-7 External Interrupts */
#define MAX326XX_IRQ_TIM0_0     (MAX326XX_IRQ_EXTINT + 22)   /* 22  Timer 0 Interrupt 0 */
#define MAX326XX_IRQ_TIM0_1     (MAX326XX_IRQ_EXTINT + 23)   /* 23  Timer 0 Interrupt 1 */
#define MAX326XX_IRQ_TIM1_0     (MAX326XX_IRQ_EXTINT + 24)   /* 24  Timer 1 Interrupt 0 */
#define MAX326XX_IRQ_TIM1_1     (MAX326XX_IRQ_EXTINT + 25)   /* 25  Timer 1 Interrupt 1 */
#define MAX326XX_IRQ_TIM2_0     (MAX326XX_IRQ_EXTINT + 26)   /* 26  Timer 2 Interrupt 0 */
#define MAX326XX_IRQ_TIM2_1     (MAX326XX_IRQ_EXTINT + 27)   /* 27  Timer 2 Interrupt 1 */
#define MAX326XX_IRQ_TIM3_0     (MAX326XX_IRQ_EXTINT + 28)   /* 28  Timer 3 Interrupt 0 */
#define MAX326XX_IRQ_TIM3_1     (MAX326XX_IRQ_EXTINT + 29)   /* 29  Timer 3 Interrupt 1 */
#define MAX326XX_IRQ_TIM4_0     (MAX326XX_IRQ_EXTINT + 30)   /* 30  Timer 4 Interrupt 0 */
#define MAX326XX_IRQ_TIM4_1     (MAX326XX_IRQ_EXTINT + 31)   /* 31  Timer 4 Interrupt 1 */
#define MAX326XX_IRQ_TIM5_0     (MAX326XX_IRQ_EXTINT + 32)   /* 32  Timer 5 Interrupt 0 */
#define MAX326XX_IRQ_TIM5_1     (MAX326XX_IRQ_EXTINT + 33)   /* 33  Timer 5 Interrupt 1 */
#define MAX326XX_IRQ_UART0      (MAX326XX_IRQ_EXTINT + 34)   /* 34  UART 0 */
#define MAX326XX_IRQ_UART1      (MAX326XX_IRQ_EXTINT + 35)   /* 34  UART 1 */
#define MAX326XX_IRQ_UART2      (MAX326XX_IRQ_EXTINT + 36)   /* 36  UART 2 */
#define MAX326XX_IRQ_UART3      (MAX326XX_IRQ_EXTINT + 37)   /* 37  UART 3 */
#define MAX326XX_IRQ_PT         (MAX326XX_IRQ_EXTINT + 38)   /* 38  Pulse Train 0-15 */
#define MAX326XX_IRQ_I2CM0      (MAX326XX_IRQ_EXTINT + 39)   /* 39  I2C0 Master */
#define MAX326XX_IRQ_I2CM1      (MAX326XX_IRQ_EXTINT + 40)   /* 40  I2C1 Master */
#define MAX326XX_IRQ_I2CM2      (MAX326XX_IRQ_EXTINT + 41)   /* 41  I2C2 Master */
#define MAX326XX_IRQ_I2CS       (MAX326XX_IRQ_EXTINT + 42)   /* 42  I2CS MB 0-31 */
#define MAX326XX_IRQ_SPIM0      (MAX326XX_IRQ_EXTINT + 43)   /* 43  SPI Master 0 */
#define MAX326XX_IRQ_SPIM1      (MAX326XX_IRQ_EXTINT + 44)   /* 44  SPI Master 1 */
#define MAX326XX_IRQ_SPIM2      (MAX326XX_IRQ_EXTINT + 45)   /* 45  SPI Master 2 */
                                                             /* 46  Reserved */
#define MAX326XX_IRQ_1WIREM     (MAX326XX_IRQ_EXTINT + 47)   /* 47  1-Wire Master */
#define MAX326XX_IRQ_ADC        (MAX326XX_IRQ_EXTINT + 48)   /* 48  ADC */
#define MAX326XX_IRQ_SPIS       (MAX326XX_IRQ_EXTINT + 49)   /* 49  SPI Slave */

/* Number of external interrupts and total number of vectors */

#define MAX326XX_IRQ_NEXTINT    50
#define NR_IRQS                 (MAX326XX_IRQ_EXTINT + MAX326XX_IRQ_NEXTINT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Inline functions
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Function Prototypes
 ************************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_MAX326XX_MAX32630_IRQ_H */
