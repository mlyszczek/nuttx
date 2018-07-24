/************************************************************************************************
 * arch/arm/include/samd5e5/sam4l_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_SAMD5E5_SAM4L_IRQ_H
#define __ARCH_ARM_INCLUDE_SAMD5E5_SAM4L_IRQ_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* External interrupts (vectors >= 16) */

#define SAM_IRQ_PM             (SAM_IRQ_EXTINT+0)     /* 0   Power Manager: SLEEPRDY */
#define SAM_IRQ_MCLK           (SAM_IRQ_EXTINT+1)     /* 1   Main clock: CKRDY */
#define SAM_IRQ_XOSC0          (SAM_IRQ_EXTINT+2)     /* 2   XOSC0: Fail/Ready */
#define SAM_IRQ_XOSC1          (SAM_IRQ_EXTINT+3)     /* 3   XOSC1: Fail/Ready */
#define SAM_IRQ_DFLL           (SAM_IRQ_EXTINT+4)     /* 4   OSCCTRLD: FLLLOCKC, DFLLLOCKF,
                                                       *     DFLLOOB, DFLLRCS, DFLLRDY */
#define SAM_IRQ_DPLL0          (SAM_IRQ_EXTINT+5)     /* 5   DPLL0: DPLLLCKF, DPLLLCKR,
                                                       *     DPLLLDRTO, DPLLLTO */
#define SAM_IRQ_DPLL1          (SAM_IRQ_EXTINT+6)     /* 6   DPLL1: DPLLLCKF, DPLLLCKR,
                                                       *     DPLLLDRTO, DPLLLTO */
#define SAM_IRQ_OSC32K         (SAM_IRQ_EXTINT+7)     /* 7   OSC32KCTRL: OSC32KRDY,
                                                       *     XOSC32KFAIL, XOSC32KRDY */
#define SAM_IRQ_SUPCRDY        (SAM_IRQ_EXTINT+8)     /* 8   Supply Controller: BOD12RDY,
                                                       *     BOD33RDY, B12SRDY, B33SRDY,
                                                       *     VCORERDY, VREGRDY */
#define SAM_IRQ_SUPCDET        (SAM_IRQ_EXTINT+9)     /* 9   Supply Controller: BOD12DET,
                                                       *     BOD33DET */
#define SAM_IRQ_WDT            (SAM_IRQ_EXTINT+10)    /* 10  WDT: EW */
#define SAM_IRQ_RTC            (SAM_IRQ_EXTINT+11)    /* 11  RTC, CMPA0-3, PERA0-7, TAMPERA */
#define SAM_IRQ_EXTINT0        (SAM_IRQ_EXTINT+12)    /* 12  EIC: EXTINT0 */
#define SAM_IRQ_EXTINT1        (SAM_IRQ_EXTINT+13)    /* 13  EIC: EXTINT1 */
#define SAM_IRQ_EXTINT2        (SAM_IRQ_EXTINT+14)    /* 14  EIC: EXTINT2 */
#define SAM_IRQ_EXTINT3        (SAM_IRQ_EXTINT+15)    /* 15  EIC: EXTINT3 */
#define SAM_IRQ_EXTINT4        (SAM_IRQ_EXTINT+16)    /* 16  EIC: EXTINT4 */
#define SAM_IRQ_EXTINT5        (SAM_IRQ_EXTINT+17)    /* 17  EIC: EXTINT5 */
#define SAM_IRQ_EXTINT6        (SAM_IRQ_EXTINT+18)    /* 18  EIC: EXTINT6 */
#define SAM_IRQ_EXTINT7        (SAM_IRQ_EXTINT+19)    /* 19  EIC: EXTINT7 */
#define SAM_IRQ_EXTINT8        (SAM_IRQ_EXTINT+20)    /* 20  EIC: EXTINT8 */
#define SAM_IRQ_EXTINT9        (SAM_IRQ_EXTINT+21)    /* 21  EIC: EXTINT9 */
#define SAM_IRQ_EXTINT10       (SAM_IRQ_EXTINT+22)    /* 22  EIC: EXTINT10 */
#define SAM_IRQ_EXTINT11       (SAM_IRQ_EXTINT+23)    /* 23  EIC: EXTINT11 */
#define SAM_IRQ_EXTINT12       (SAM_IRQ_EXTINT+24)    /* 24  EIC: EXTINT12 */
#define SAM_IRQ_EXTINT13       (SAM_IRQ_EXTINT+25)    /* 25  EIC: EXTINT13 */
#define SAM_IRQ_EXTINT14       (SAM_IRQ_EXTINT+26)    /* 26  EIC: EXTINT14 */
#define SAM_IRQ_EXTINT15       (SAM_IRQ_EXTINT+27)    /* 27  EIC: EXTINT15 */
#define SAM_IRQ_FREQM          (SAM_IRQ_EXTINT+28)    /* 28  FREQM: Done */
#define SAM_IRQ_NVMCTRL0       (SAM_IRQ_EXTINT+29)    /* 29  NVMCTRL: 0-7 */
#define SAM_IRQ_NVMCTRL1       (SAM_IRQ_EXTINT+30)    /* 30  NVMCTRL: 8-10 */
#define SAM_IRQ_DMACH0         (SAM_IRQ_EXTINT+31)    /* 31  DMA Channel 0: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH1         (SAM_IRQ_EXTINT+32)    /* 32  DMA Channel 1: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH2         (SAM_IRQ_EXTINT+33)    /* 33  DMA Channel 2: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH3         (SAM_IRQ_EXTINT+34)    /* 34  DMA Channel 3: SUSP, TCMPL, TERR */
#define SAM_IRQ_DMACH4_31      (SAM_IRQ_EXTINT+35)    /* 35  DMA Channels 4-31: SUSP, TCMPL, TERR */
#define SAM_IRQ_EVSYS0         (SAM_IRQ_EXTINT+36)    /* 36  EVSYS Channel 0: EVD, OVR */
#define SAM_IRQ_EVSYS1         (SAM_IRQ_EXTINT+37)    /* 37  EVSYS Channel 1: EVD, OVR */
#define SAM_IRQ_EVSYS2         (SAM_IRQ_EXTINT+38)    /* 38  EVSYS Channel 2: EVD, OVR */
#define SAM_IRQ_EVSYS3         (SAM_IRQ_EXTINT+39)    /* 39  EVSYS Channel 3: EVD, OVR */
#define SAM_IRQ_EVSYS4_11      (SAM_IRQ_EXTINT+40)    /* 40  EVSYS Channels 4-11: EVD, OVR */
#define SAM_IRQ_PAC            (SAM_IRQ_EXTINT+41)    /* 41  PAC: ERR */
#define SAM_IRQ_RAMECC         (SAM_IRQ_EXTINT+45)    /* 45  RAM ECC: 0-1 */
#define SAM_IRQ_SERCOM0_0      (SAM_IRQ_EXTINT+46)    /* 46  SERCOM0: 0 */
#define SAM_IRQ_SERCOM0_1      (SAM_IRQ_EXTINT+47)    /* 47  SERCOM0: 1 */
#define SAM_IRQ_SERCOM0_2      (SAM_IRQ_EXTINT+48)    /* 48  SERCOM0: 2 */
#define SAM_IRQ_SERCOM0_46     (SAM_IRQ_EXTINT+49)    /* 49  SERCOM0: 4-6 */
#define SAM_IRQ_SERCOM1_0      (SAM_IRQ_EXTINT+50)    /* 50  SERCOM1: 0 */
#define SAM_IRQ_SERCOM1_1      (SAM_IRQ_EXTINT+51)    /* 51  SERCOM1: 1 */
#define SAM_IRQ_SERCOM1_2      (SAM_IRQ_EXTINT+52)    /* 52  SERCOM1: 2 */
#define SAM_IRQ_SERCOM1_46     (SAM_IRQ_EXTINT+53)    /* 53  SERCOM1: 4-6 */
#define SAM_IRQ_SERCOM2_0      (SAM_IRQ_EXTINT+54)    /* 54  SERCOM2: 0 */
#define SAM_IRQ_SERCOM2_1      (SAM_IRQ_EXTINT+55)    /* 55  SERCOM2: 1 */
#define SAM_IRQ_SERCOM2_2      (SAM_IRQ_EXTINT+56)    /* 56  SERCOM2: 2 */
#define SAM_IRQ_SERCOM2_46     (SAM_IRQ_EXTINT+57)    /* 57  SERCOM2: 4-6 */
#define SAM_IRQ_SERCOM3_0      (SAM_IRQ_EXTINT+58)    /* 58  SERCOM3: 0 */
#define SAM_IRQ_SERCOM3_1      (SAM_IRQ_EXTINT+59)    /* 59  SERCOM3: 1 */
#define SAM_IRQ_SERCOM3_2      (SAM_IRQ_EXTINT+60)    /* 60  SERCOM3: 2 */
#define SAM_IRQ_SERCOM3_46     (SAM_IRQ_EXTINT+61)    /* 61  SERCOM3: 4-6 */
#define SAM_IRQ_SERCOM4_0      (SAM_IRQ_EXTINT+62)    /* 62  SERCOM4: 0 */
#define SAM_IRQ_SERCOM4_1      (SAM_IRQ_EXTINT+63)    /* 63  SERCOM4: 1 */
#define SAM_IRQ_SERCOM4_2      (SAM_IRQ_EXTINT+64)    /* 64  SERCOM4: 2 */
#define SAM_IRQ_SERCOM4_46     (SAM_IRQ_EXTINT+65)    /* 65  SERCOM4: 4-6 */
#define SAM_IRQ_SERCOM5_0      (SAM_IRQ_EXTINT+66)    /* 66  SERCOM5: 0 */
#define SAM_IRQ_SERCOM5_1      (SAM_IRQ_EXTINT+67)    /* 67  SERCOM5: 1 */
#define SAM_IRQ_SERCOM5_2      (SAM_IRQ_EXTINT+68)    /* 68  SERCOM5: 2 */
#define SAM_IRQ_SERCOM5_46     (SAM_IRQ_EXTINT+69)    /* 69  SERCOM5: 4-6 */
#define SAM_IRQ_SERCOM6_0      (SAM_IRQ_EXTINT+70)    /* 70  SERCOM6: 0 */
#define SAM_IRQ_SERCOM6_1      (SAM_IRQ_EXTINT+71)    /* 71  SERCOM6: 1 */
#define SAM_IRQ_SERCOM6_2      (SAM_IRQ_EXTINT+72)    /* 72  SERCOM6: 2 */
#define SAM_IRQ_SERCOM6_46     (SAM_IRQ_EXTINT+73)    /* 73  SERCOM6: 4-6 */
#define SAM_IRQ_SERCOM7_0      (SAM_IRQ_EXTINT+74)    /* 74  SERCOM7: 0 */
#define SAM_IRQ_SERCOM7_1      (SAM_IRQ_EXTINT+75)    /* 75  SERCOM7: 1 */
#define SAM_IRQ_SERCOM7_2      (SAM_IRQ_EXTINT+76)    /* 76  SERCOM7: 2 */
#define SAM_IRQ_SERCOM7_46     (SAM_IRQ_EXTINT+77)    /* 77  SERCOM7: 4-6 */
#define SAM_IRQ_CAN0           (SAM_IRQ_EXTINT+78)    /* 78  CAN0: Line0, Line1 */
#define SAM_IRQ_CAN1           (SAM_IRQ_EXTINT+79)    /* 79  CAN1: Line0, Line1 */
#define SAM_IRQ_USB            (SAM_IRQ_EXTINT+80)    /* 80  USB: EORSM, DNRSM, EORST RST,
                                                       *     LPM DCONN, LPMSUSP DDISC, MSOF,
                                                       *     RAMACER, RXSTP TXSTP 0-7, STALL0
                                                       *     STALL 0-7, STALL1 0-7, SUSPEND,
                                                       *     TRFAIL0 TRFAIL 097, TRFAIL1 PERR
                                                       *      0..7, UPRSM, WAKEUP */
#define SAM_IRQ_USBSOF         (SAM_IRQ_EXTINT+81)    /* 81  USB: SOF HSOF */
#define SAM_IRQ_USBTRCPT0      (SAM_IRQ_EXTINT+82)    /* 82  USB: TRCPT0 0..7 */
#define SAM_IRQ_USBTRCPT1      (SAM_IRQ_EXTINT+83)    /* 83  USB: TRCPT0 0..7 */
#define SAM_IRQ_GMAL           (SAM_IRQ_EXTINT+84)    /* 84  GMAC:  GMAC, WOL */
#define SAM_IRQ_TCC0           (SAM_IRQ_EXTINT+85)    /* 85  TCC0: CNT A, DFS A, ERR A, FAULTA
                                                       *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                       *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC0MC0        (SAM_IRQ_EXTINT+86)    /* 86  TCC0: MC 0 */
#define SAM_IRQ_TCC0MC1        (SAM_IRQ_EXTINT+87)    /* 87  TCC0: MC 1 */
#define SAM_IRQ_TCC0MC2        (SAM_IRQ_EXTINT+88)    /* 88  TCC0: MC 2 */
#define SAM_IRQ_TCC0MC3        (SAM_IRQ_EXTINT+89)    /* 89  TCC0: MC 3 */
#define SAM_IRQ_TCC0MC4        (SAM_IRQ_EXTINT+90)    /* 90  TCC0: MC 4 */
#define SAM_IRQ_TCC0MC5        (SAM_IRQ_EXTINT+91)    /* 91  TCC0: MC 5 */
#define SAM_IRQ_TCC1           (SAM_IRQ_EXTINT+92)    /* 92  TCC1: CNT A, DFS A, ERR A, FAULTA
                                                       *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                       *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC1MC0        (SAM_IRQ_EXTINT+93)    /* 93  TCC1: MC 0 */
#define SAM_IRQ_TCC1MC1        (SAM_IRQ_EXTINT+94)    /* 94  TCC1: MC 1 */
#define SAM_IRQ_TCC1MC2        (SAM_IRQ_EXTINT+95)    /* 95  TCC1: MC 2 */
#define SAM_IRQ_TCC1MC3        (SAM_IRQ_EXTINT+96)    /* 96  TCC1: MC 3 */
#define SAM_IRQ_TCC2           (SAM_IRQ_EXTINT+97)    /* 97  TCC2: CNT A, DFS A, ERR A, FAULTA
                                                       *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                       *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC2MC0        (SAM_IRQ_EXTINT+98)    /* 98  TCC2: MC 0 */
#define SAM_IRQ_TCC2MC1        (SAM_IRQ_EXTINT+99)    /* 99  TCC2: MC 1 */
#define SAM_IRQ_TCC2MC2        (SAM_IRQ_EXTINT+100)   /* 100 TCC2: MC 2 */
#define SAM_IRQ_TCC3           (SAM_IRQ_EXTINT+101)   /* 101 TCC3: CNT A, DFS A, ERR A, FAULTA
                                                       *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                       *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC3MC0        (SAM_IRQ_EXTINT+102)   /* 102 TCC3: MC 0 */
#define SAM_IRQ_TCC3MC1        (SAM_IRQ_EXTINT+103)   /* 103 TCC3: MC 1 */
#define SAM_IRQ_TCC4           (SAM_IRQ_EXTINT+104)   /* 104 TCC4: CNT A, DFS A, ERR A, FAULTA
                                                       *     A, FAULTB A, FAULT0 A, FAULT1 A,
                                                       *     OVF, TRG, UFS A */
#define SAM_IRQ_TCC4MC0        (SAM_IRQ_EXTINT+105)   /* 105 TCC4: MC 0 */
#define SAM_IRQ_TCC4MC1        (SAM_IRQ_EXTINT+106)   /* 106 TCC4: MC 1 */
#define SAM_IRQ_TC0            (SAM_IRQ_EXTINT+107)   /* 107 TC0: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC1            (SAM_IRQ_EXTINT+108)   /* 108 TC1: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC2            (SAM_IRQ_EXTINT+109)   /* 109 TC2: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC3            (SAM_IRQ_EXTINT+110)   /* 110 TC3: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC4            (SAM_IRQ_EXTINT+111)   /* 111 TC4: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC5            (SAM_IRQ_EXTINT+112)   /* 112 TC5: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC6            (SAM_IRQ_EXTINT+113)   /* 113 TC6: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_TC7            (SAM_IRQ_EXTINT+114)   /* 114 TC7: ERR A, MC 0, MC 1, OVF */
#define SAM_IRQ_PDEC           (SAM_IRQ_EXTINT+115)   /* 115 PDEC: DIR A, ERR A, OVF, VLC A */
#define SAM_IRQ_PDECMC0        (SAM_IRQ_EXTINT+116)   /* 116 PDEC: MC 0 */
#define SAM_IRQ_PDECMC1        (SAM_IRQ_EXTINT+117)   /* 117 PDEC: MC 1 */
#define SAM_IRQ_ADC0           (SAM_IRQ_EXTINT+118)   /* 118 ADC0: OVERRUN, WINMON */
#define SAM_IRQ_ADC0RDY        (SAM_IRQ_EXTINT+119)   /* 119 ADC0: RESRDY */
#define SAM_IRQ_ADC1           (SAM_IRQ_EXTINT+120)   /* 120 ADC0: OVERRUN, WINMON */
#define SAM_IRQ_ADC1RDY        (SAM_IRQ_EXTINT+121)   /* 121 ADC0: RESRDY */
#define SAM_IRQ_AC             (SAM_IRQ_EXTINT+122)   /* 122 AC: COMP 0, COMP 1, WIN 0 */
#define SAM_IRQ_DACERR         (SAM_IRQ_EXTINT+123)   /* 123 DAC: OVERRUN A 0, OVERRUN A 1,
                                                       *     UNDERRUN A 0, UNDERRUN A 1 */
#define SAM_IRQ_DACEMPTY0      (SAM_IRQ_EXTINT+124)   /* 124 DAC: EMPTY 0 */
#define SAM_IRQ_DACEMPTY1      (SAM_IRQ_EXTINT+125)   /* 125 DAC: EMPTY 1 */
#define SAM_IRQ_DACRDY0        (SAM_IRQ_EXTINT+126)   /* 126 DAC: RESRDY 0 */
#define SAM_IRQ_DACRDY1        (SAM_IRQ_EXTINT+127)   /* 127 DAC: RESRDY 1 */
#define SAM_IRQ_I2S            (SAM_IRQ_EXTINT+128)   /* 128 I2S: RXOR 0, RXOR 1, RXRDY 0, RXRDY
                                                       *     1, TXRDY 0, TXRDY 1, TXUR 0, TXUR 1 */
#define SAM_IRQ_PCC            (SAM_IRQ_EXTINT+129)   /* 129 PCC: */
#define SAM_IRQ_AES            (SAM_IRQ_EXTINT+130)   /* 130 AES: ENCCMP, GFMCMP */
#define SAM_IRQ_TRNG           (SAM_IRQ_EXTINT+131)   /* 131 TRNG: IS0 */
#define SAM_IRQ_ICM            (SAM_IRQ_EXTINT+132)   /* 132 ICM: */
#define SAM_IRQ_PUKCC          (SAM_IRQ_EXTINT+133)   /* 133 PUKCC:  */
#define SAM_IRQ_QSPI           (SAM_IRQ_EXTINT+134)   /* 134 QSPI:  */
#define SAM_IRQ_SDHC0          (SAM_IRQ_EXTINT+135)   /* 135 SDHC0: SDHC0, TIMER */
#define SAM_IRQ_SDHC1          (SAM_IRQ_EXTINT+136)   /* 136 SDHC1: SDHC1, TIMER */

#define SAM_IRQ_NEXTINT        137                    /* Total number of external interrupt numbers */

#define SAM_IRQ_NIRQS          (SAM_IRQ_EXTINT+SAM_IRQ_NEXTINT) /* The number of real IRQs */

/* GPIO interrupts (derived from SAM_IRQ_PIOA/B/C) */

#ifdef CONFIG_SAMD5E5_GPIOA_IRQ
#  define SAM_IRQ_GPIOA_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT)
#  define SAM_IRQ_PA0         (SAM_IRQ_GPIOA_PINS+0)          /* GPIOA, PIN 0 */
#  define SAM_IRQ_PA1         (SAM_IRQ_GPIOA_PINS+1)          /* GPIOA, PIN 1 */
#  define SAM_IRQ_PA2         (SAM_IRQ_GPIOA_PINS+2)          /* GPIOA, PIN 2 */
#  define SAM_IRQ_PA3         (SAM_IRQ_GPIOA_PINS+3)          /* GPIOA, PIN 3 */
#  define SAM_IRQ_PA4         (SAM_IRQ_GPIOA_PINS+4)          /* GPIOA, PIN 4 */
#  define SAM_IRQ_PA5         (SAM_IRQ_GPIOA_PINS+5)          /* GPIOA, PIN 5 */
#  define SAM_IRQ_PA6         (SAM_IRQ_GPIOA_PINS+6)          /* GPIOA, PIN 6 */
#  define SAM_IRQ_PA7         (SAM_IRQ_GPIOA_PINS+7)          /* GPIOA, PIN 7 */
#  define SAM_IRQ_PA8         (SAM_IRQ_GPIOA_PINS+8)          /* GPIOA, PIN 8 */
#  define SAM_IRQ_PA9         (SAM_IRQ_GPIOA_PINS+9)          /* GPIOA, PIN 9 */
#  define SAM_IRQ_PA10        (SAM_IRQ_GPIOA_PINS+10)         /* GPIOA, PIN 10 */
#  define SAM_IRQ_PA11        (SAM_IRQ_GPIOA_PINS+11)         /* GPIOA, PIN 11 */
#  define SAM_IRQ_PA12        (SAM_IRQ_GPIOA_PINS+12)         /* GPIOA, PIN 12 */
#  define SAM_IRQ_PA13        (SAM_IRQ_GPIOA_PINS+13)         /* GPIOA, PIN 13 */
#  define SAM_IRQ_PA14        (SAM_IRQ_GPIOA_PINS+14)         /* GPIOA, PIN 14 */
#  define SAM_IRQ_PA15        (SAM_IRQ_GPIOA_PINS+15)         /* GPIOA, PIN 15 */
#  define SAM_IRQ_PA16        (SAM_IRQ_GPIOA_PINS+16)         /* GPIOA, PIN 16 */
#  define SAM_IRQ_PA17        (SAM_IRQ_GPIOA_PINS+17)         /* GPIOA, PIN 17 */
#  define SAM_IRQ_PA18        (SAM_IRQ_GPIOA_PINS+18)         /* GPIOA, PIN 18 */
#  define SAM_IRQ_PA19        (SAM_IRQ_GPIOA_PINS+19)         /* GPIOA, PIN 19 */
#  define SAM_IRQ_PA20        (SAM_IRQ_GPIOA_PINS+20)         /* GPIOA, PIN 20 */
#  define SAM_IRQ_PA21        (SAM_IRQ_GPIOA_PINS+21)         /* GPIOA, PIN 21 */
#  define SAM_IRQ_PA22        (SAM_IRQ_GPIOA_PINS+22)         /* GPIOA, PIN 22 */
#  define SAM_IRQ_PA23        (SAM_IRQ_GPIOA_PINS+23)         /* GPIOA, PIN 23 */
#  define SAM_IRQ_PA24        (SAM_IRQ_GPIOA_PINS+24)         /* GPIOA, PIN 24 */
#  define SAM_IRQ_PA25        (SAM_IRQ_GPIOA_PINS+25)         /* GPIOA, PIN 25 */
#  define SAM_IRQ_PA26        (SAM_IRQ_GPIOA_PINS+26)         /* GPIOA, PIN 26 */
#  define SAM_IRQ_PA27        (SAM_IRQ_GPIOA_PINS+27)         /* GPIOA, PIN 27 */
#  define SAM_IRQ_PA28        (SAM_IRQ_GPIOA_PINS+28)         /* GPIOA, PIN 28 */
#  define SAM_IRQ_PA29        (SAM_IRQ_GPIOA_PINS+29)         /* GPIOA, PIN 29 */
#  define SAM_IRQ_PA30        (SAM_IRQ_GPIOA_PINS+30)         /* GPIOA, PIN 30 */
#  define SAM_IRQ_PA31        (SAM_IRQ_GPIOA_PINS+31)         /* GPIOA, PIN 31 */
#  define SAM_NGPIOAIRQS      32
#else
#  define SAM_NGPIOAIRQS      0
#endif

#ifdef CONFIG_SAMD5E5_GPIOB_IRQ
#  define SAM_IRQ_GPIOB_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + SAM_NGPIOAIRQS)
#  define SAM_IRQ_PB0         (SAM_IRQ_GPIOB_PINS+0)          /* GPIOB, PIN 0 */
#  define SAM_IRQ_PB1         (SAM_IRQ_GPIOB_PINS+1)          /* GPIOB, PIN 1 */
#  define SAM_IRQ_PB2         (SAM_IRQ_GPIOB_PINS+2)          /* GPIOB, PIN 2 */
#  define SAM_IRQ_PB3         (SAM_IRQ_GPIOB_PINS+3)          /* GPIOB, PIN 3 */
#  define SAM_IRQ_PB4         (SAM_IRQ_GPIOB_PINS+4)          /* GPIOB, PIN 4 */
#  define SAM_IRQ_PB5         (SAM_IRQ_GPIOB_PINS+5)          /* GPIOB, PIN 5 */
#  define SAM_IRQ_PB6         (SAM_IRQ_GPIOB_PINS+6)          /* GPIOB, PIN 6 */
#  define SAM_IRQ_PB7         (SAM_IRQ_GPIOB_PINS+7)          /* GPIOB, PIN 7 */
#  define SAM_IRQ_PB8         (SAM_IRQ_GPIOB_PINS+8)          /* GPIOB, PIN 8 */
#  define SAM_IRQ_PB9         (SAM_IRQ_GPIOB_PINS+9)          /* GPIOB, PIN 9 */
#  define SAM_IRQ_PB10        (SAM_IRQ_GPIOB_PINS+10)         /* GPIOB, PIN 10 */
#  define SAM_IRQ_PB11        (SAM_IRQ_GPIOB_PINS+11)         /* GPIOB, PIN 11 */
#  define SAM_IRQ_PB12        (SAM_IRQ_GPIOB_PINS+12)         /* GPIOB, PIN 12 */
#  define SAM_IRQ_PB13        (SAM_IRQ_GPIOB_PINS+13)         /* GPIOB, PIN 13 */
#  define SAM_IRQ_PB14        (SAM_IRQ_GPIOB_PINS+14)         /* GPIOB, PIN 14 */
#  define SAM_IRQ_PB15        (SAM_IRQ_GPIOB_PINS+15)         /* GPIOB, PIN 15 */
#  define SAM_IRQ_PB16        (SAM_IRQ_GPIOB_PINS+16)         /* GPIOB, PIN 16 */
#  define SAM_IRQ_PB17        (SAM_IRQ_GPIOB_PINS+17)         /* GPIOB, PIN 17 */
#  define SAM_IRQ_PB18        (SAM_IRQ_GPIOB_PINS+18)         /* GPIOB, PIN 18 */
#  define SAM_IRQ_PB19        (SAM_IRQ_GPIOB_PINS+19)         /* GPIOB, PIN 19 */
#  define SAM_IRQ_PB20        (SAM_IRQ_GPIOB_PINS+20)         /* GPIOB, PIN 20 */
#  define SAM_IRQ_PB21        (SAM_IRQ_GPIOB_PINS+21)         /* GPIOB, PIN 21 */
#  define SAM_IRQ_PB22        (SAM_IRQ_GPIOB_PINS+22)         /* GPIOB, PIN 22 */
#  define SAM_IRQ_PB23        (SAM_IRQ_GPIOB_PINS+23)         /* GPIOB, PIN 23 */
#  define SAM_IRQ_PB24        (SAM_IRQ_GPIOB_PINS+24)         /* GPIOB, PIN 24 */
#  define SAM_IRQ_PB25        (SAM_IRQ_GPIOB_PINS+25)         /* GPIOB, PIN 25 */
#  define SAM_IRQ_PB26        (SAM_IRQ_GPIOB_PINS+26)         /* GPIOB, PIN 26 */
#  define SAM_IRQ_PB27        (SAM_IRQ_GPIOB_PINS+27)         /* GPIOB, PIN 27 */
#  define SAM_IRQ_PB28        (SAM_IRQ_GPIOB_PINS+28)         /* GPIOB, PIN 28 */
#  define SAM_IRQ_PB29        (SAM_IRQ_GPIOB_PINS+29)         /* GPIOB, PIN 29 */
#  define SAM_IRQ_PB30        (SAM_IRQ_GPIOB_PINS+30)         /* GPIOB, PIN 30 */
#  define SAM_IRQ_PB31        (SAM_IRQ_GPIOB_PINS+31)         /* GPIOB, PIN 31 */
#  define SAM_NGPIOBIRQS      32
#else
#  define SAM_NGPIOBIRQS      0
#endif

#ifdef CONFIG_SAMD5E5_GPIOC_IRQ
#  define SAM_IRQ_GPIOC_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS)
#  define SAM_IRQ_PC0         (SAM_IRQ_GPIOC_PINS+0)          /* GPIOC, PIN 0 */
#  define SAM_IRQ_PC1         (SAM_IRQ_GPIOC_PINS+1)          /* GPIOC, PIN 1 */
#  define SAM_IRQ_PC2         (SAM_IRQ_GPIOC_PINS+2)          /* GPIOC, PIN 2 */
#  define SAM_IRQ_PC3         (SAM_IRQ_GPIOC_PINS+3)          /* GPIOC, PIN 3 */
#  define SAM_IRQ_PC4         (SAM_IRQ_GPIOC_PINS+4)          /* GPIOC, PIN 4 */
#  define SAM_IRQ_PC5         (SAM_IRQ_GPIOC_PINS+5)          /* GPIOC, PIN 5 */
#  define SAM_IRQ_PC6         (SAM_IRQ_GPIOC_PINS+6)          /* GPIOC, PIN 6 */
#  define SAM_IRQ_PC7         (SAM_IRQ_GPIOC_PINS+7)          /* GPIOC, PIN 7 */
#  define SAM_IRQ_PC8         (SAM_IRQ_GPIOC_PINS+8)          /* GPIOC, PIN 8 */
#  define SAM_IRQ_PC9         (SAM_IRQ_GPIOC_PINS+9)          /* GPIOC, PIN 9 */
#  define SAM_IRQ_PC10        (SAM_IRQ_GPIOC_PINS+10)         /* GPIOC, PIN 10 */
#  define SAM_IRQ_PC11        (SAM_IRQ_GPIOC_PINS+11)         /* GPIOC, PIN 11 */
#  define SAM_IRQ_PC12        (SAM_IRQ_GPIOC_PINS+12)         /* GPIOC, PIN 12 */
#  define SAM_IRQ_PC13        (SAM_IRQ_GPIOC_PINS+13)         /* GPIOC, PIN 13 */
#  define SAM_IRQ_PC14        (SAM_IRQ_GPIOC_PINS+14)         /* GPIOC, PIN 14 */
#  define SAM_IRQ_PC15        (SAM_IRQ_GPIOC_PINS+15)         /* GPIOC, PIN 15 */
#  define SAM_IRQ_PC16        (SAM_IRQ_GPIOC_PINS+16)         /* GPIOC, PIN 16 */
#  define SAM_IRQ_PC17        (SAM_IRQ_GPIOC_PINS+17)         /* GPIOC, PIN 17 */
#  define SAM_IRQ_PC18        (SAM_IRQ_GPIOC_PINS+18)         /* GPIOC, PIN 18 */
#  define SAM_IRQ_PC19        (SAM_IRQ_GPIOC_PINS+19)         /* GPIOC, PIN 19 */
#  define SAM_IRQ_PC20        (SAM_IRQ_GPIOC_PINS+20)         /* GPIOC, PIN 20 */
#  define SAM_IRQ_PC21        (SAM_IRQ_GPIOC_PINS+21)         /* GPIOC, PIN 21 */
#  define SAM_IRQ_PC22        (SAM_IRQ_GPIOC_PINS+22)         /* GPIOC, PIN 22 */
#  define SAM_IRQ_PC23        (SAM_IRQ_GPIOC_PINS+23)         /* GPIOC, PIN 23 */
#  define SAM_IRQ_PC24        (SAM_IRQ_GPIOC_PINS+24)         /* GPIOC, PIN 24 */
#  define SAM_IRQ_PC25        (SAM_IRQ_GPIOC_PINS+25)         /* GPIOC, PIN 25 */
#  define SAM_IRQ_PC26        (SAM_IRQ_GPIOC_PINS+26)         /* GPIOC, PIN 26 */
#  define SAM_IRQ_PC27        (SAM_IRQ_GPIOC_PINS+27)         /* GPIOC, PIN 27 */
#  define SAM_IRQ_PC28        (SAM_IRQ_GPIOC_PINS+28)         /* GPIOC, PIN 28 */
#  define SAM_IRQ_PC29        (SAM_IRQ_GPIOC_PINS+29)         /* GPIOC, PIN 29 */
#  define SAM_IRQ_PC30        (SAM_IRQ_GPIOC_PINS+30)         /* GPIOC, PIN 30 */
#  define SAM_IRQ_PC31        (SAM_IRQ_GPIOC_PINS+31)         /* GPIOC, PIN 31 */
#  define SAM_NGPIOCIRQS      32
#else
#  define SAM_NGPIOCIRQS      0
#endif

#ifdef CONFIG_SAMD5E5_GPIOD_IRQ
#  define SAM_IRQ_GPIOD_PINS  (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS + SAM_NGPIOCIRQS)
#  define SAM_IRQ_PC0         (SAM_IRQ_GPIOD_PINS+0)          /* GPIOD, PIN 0 */
#  define SAM_IRQ_PC1         (SAM_IRQ_GPIOD_PINS+1)          /* GPIOD, PIN 1 */
#  define SAM_IRQ_PC2         (SAM_IRQ_GPIOD_PINS+2)          /* GPIOD, PIN 2 */
#  define SAM_IRQ_PC3         (SAM_IRQ_GPIOD_PINS+3)          /* GPIOD, PIN 3 */
#  define SAM_IRQ_PC4         (SAM_IRQ_GPIOD_PINS+4)          /* GPIOD, PIN 4 */
#  define SAM_IRQ_PC5         (SAM_IRQ_GPIOD_PINS+5)          /* GPIOD, PIN 5 */
#  define SAM_IRQ_PC6         (SAM_IRQ_GPIOD_PINS+6)          /* GPIOD, PIN 6 */
#  define SAM_IRQ_PC7         (SAM_IRQ_GPIOD_PINS+7)          /* GPIOD, PIN 7 */
#  define SAM_IRQ_PC8         (SAM_IRQ_GPIOD_PINS+8)          /* GPIOD, PIN 8 */
#  define SAM_IRQ_PC9         (SAM_IRQ_GPIOD_PINS+9)          /* GPIOD, PIN 9 */
#  define SAM_IRQ_PC10        (SAM_IRQ_GPIOD_PINS+10)         /* GPIOD, PIN 10 */
#  define SAM_IRQ_PC11        (SAM_IRQ_GPIOD_PINS+11)         /* GPIOD, PIN 11 */
#  define SAM_IRQ_PC12        (SAM_IRQ_GPIOD_PINS+12)         /* GPIOD, PIN 12 */
#  define SAM_IRQ_PC13        (SAM_IRQ_GPIOD_PINS+13)         /* GPIOD, PIN 13 */
#  define SAM_IRQ_PC14        (SAM_IRQ_GPIOD_PINS+14)         /* GPIOD, PIN 14 */
#  define SAM_IRQ_PC15        (SAM_IRQ_GPIOD_PINS+15)         /* GPIOD, PIN 15 */
#  define SAM_IRQ_PC16        (SAM_IRQ_GPIOD_PINS+16)         /* GPIOD, PIN 16 */
#  define SAM_IRQ_PC17        (SAM_IRQ_GPIOD_PINS+17)         /* GPIOD, PIN 17 */
#  define SAM_IRQ_PC18        (SAM_IRQ_GPIOD_PINS+18)         /* GPIOD, PIN 18 */
#  define SAM_IRQ_PC19        (SAM_IRQ_GPIOD_PINS+19)         /* GPIOD, PIN 19 */
#  define SAM_IRQ_PC20        (SAM_IRQ_GPIOD_PINS+20)         /* GPIOD, PIN 20 */
#  define SAM_IRQ_PC21        (SAM_IRQ_GPIOD_PINS+21)         /* GPIOD, PIN 21 */
#  define SAM_IRQ_PC22        (SAM_IRQ_GPIOD_PINS+22)         /* GPIOD, PIN 22 */
#  define SAM_IRQ_PC23        (SAM_IRQ_GPIOD_PINS+23)         /* GPIOD, PIN 23 */
#  define SAM_IRQ_PC24        (SAM_IRQ_GPIOD_PINS+24)         /* GPIOD, PIN 24 */
#  define SAM_IRQ_PC25        (SAM_IRQ_GPIOD_PINS+25)         /* GPIOD, PIN 25 */
#  define SAM_IRQ_PC26        (SAM_IRQ_GPIOD_PINS+26)         /* GPIOD, PIN 26 */
#  define SAM_IRQ_PC27        (SAM_IRQ_GPIOD_PINS+27)         /* GPIOD, PIN 27 */
#  define SAM_IRQ_PC28        (SAM_IRQ_GPIOD_PINS+28)         /* GPIOD, PIN 28 */
#  define SAM_IRQ_PC29        (SAM_IRQ_GPIOD_PINS+29)         /* GPIOD, PIN 29 */
#  define SAM_IRQ_PC30        (SAM_IRQ_GPIOD_PINS+30)         /* GPIOD, PIN 30 */
#  define SAM_IRQ_PC31        (SAM_IRQ_GPIOD_PINS+31)         /* GPIOD, PIN 31 */
#  define SAM_NGPIODIRQS      32
#else
#  define SAM_NGPIODIRQS      0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS               (SAM_IRQ_EXTINT + SAM_IRQ_NEXTINT + \
                               SAM_NGPIOAIRQS + SAM_NGPIOBIRQS + \
                               SAM_NGPIOCIRQS + SAM_NGPIODIRQS)

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

#endif /* __ARCH_ARM_INCLUDE_SAMD5E5_SAM4L_IRQ_H */
