/******************************************************************************
 *  Filename:       hw_rfc_pwr_h
 *  Revised:        2017-01-31 09:37:48 +0100 (Tue, 31 Jan 2017)
 *  Revision:       48345
 *
 * Copyright (c) 2015 - 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3) Neither the name NuttX nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_HW_RFC_PWR_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_HW_RFC_PWR_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * RFC_PWR component
 *
 ******************************************************************************
 * RF Core Power Management and Clock Enable
 */

#define RFC_PWR_PWMCLKEN_OFFSET                                     0x00000000

/******************************************************************************
 *
 * Register: RFC_PWR_PWMCLKEN
 *
 ******************************************************************************
 * Field:    [10] RFCTRC
 *
 * Enable clock to the RF Core Tracer (RFCTRC) module.
 */

#define RFC_PWR_PWMCLKEN_RFCTRC                                     0x00000400
#define RFC_PWR_PWMCLKEN_RFCTRC_MASK                                0x00000400
#define RFC_PWR_PWMCLKEN_RFCTRC_SHIFT                                       10

/* Field:     [9] FSCA
 *
 * Enable clock to the Frequency Synthesizer Calibration Accelerator (FSCA)
 * module.
 */

#define RFC_PWR_PWMCLKEN_FSCA                                       0x00000200
#define RFC_PWR_PWMCLKEN_FSCA_MASK                                  0x00000200
#define RFC_PWR_PWMCLKEN_FSCA_SHIFT                                          9

/* Field:     [8] PHA
 *
 * Enable clock to the Packet Handling Accelerator (PHA) module.
 */

#define RFC_PWR_PWMCLKEN_PHA                                        0x00000100
#define RFC_PWR_PWMCLKEN_PHA_MASK                                   0x00000100
#define RFC_PWR_PWMCLKEN_PHA_SHIFT                                           8

/* Field:     [7] RAT
 *
 * Enable clock to the Radio Timer (RAT) module.
 */

#define RFC_PWR_PWMCLKEN_RAT                                        0x00000080
#define RFC_PWR_PWMCLKEN_RAT_MASK                                   0x00000080
#define RFC_PWR_PWMCLKEN_RAT_SHIFT                                           7

/* Field:     [6] RFERAM
 *
 * Enable clock to the RF Engine RAM module.
 */

#define RFC_PWR_PWMCLKEN_RFERAM                                     0x00000040
#define RFC_PWR_PWMCLKEN_RFERAM_MASK                                0x00000040
#define RFC_PWR_PWMCLKEN_RFERAM_SHIFT                                        6

/* Field:     [5] RFE
 *
 * Enable clock to the RF Engine (RFE) module.
 */

#define RFC_PWR_PWMCLKEN_RFE                                        0x00000020
#define RFC_PWR_PWMCLKEN_RFE_MASK                                   0x00000020
#define RFC_PWR_PWMCLKEN_RFE_SHIFT                                           5

/* Field:     [4] MDMRAM
 *
 * Enable clock to the Modem RAM module.
 */

#define RFC_PWR_PWMCLKEN_MDMRAM                                     0x00000010
#define RFC_PWR_PWMCLKEN_MDMRAM_MASK                                0x00000010
#define RFC_PWR_PWMCLKEN_MDMRAM_SHIFT                                        4

/* Field:     [3] MDM
 *
 * Enable clock to the Modem (MDM) module.
 */

#define RFC_PWR_PWMCLKEN_MDM                                        0x00000008
#define RFC_PWR_PWMCLKEN_MDM_MASK                                   0x00000008
#define RFC_PWR_PWMCLKEN_MDM_SHIFT                                           3

/* Field:     [2] CPERAM
 *
 * Enable clock to the Command and Packet Engine (CPE) RAM module. As part of
 * RF Core initialization, set this bit together with CPE bit to enable CPE to
 * boot.
 */

#define RFC_PWR_PWMCLKEN_CPERAM                                     0x00000004
#define RFC_PWR_PWMCLKEN_CPERAM_MASK                                0x00000004
#define RFC_PWR_PWMCLKEN_CPERAM_SHIFT                                        2

/* Field:     [1] CPE
 *
 * Enable processor clock (hclk) to the Command and Packet Engine (CPE). As
 * part of RF Core initialization, set this bit together with CPERAM bit to
 * enable CPE to boot.
 */

#define RFC_PWR_PWMCLKEN_CPE                                        0x00000002
#define RFC_PWR_PWMCLKEN_CPE_MASK                                   0x00000002
#define RFC_PWR_PWMCLKEN_CPE_SHIFT                                           1

/* Field:     [0] RFC
 *
 * Enable essential clocks for the RF Core interface. This includes the
 * interconnect, the radio doorbell DBELL command interface, the power
 * management (PWR) clock control module, and bus clock (sclk) for the CPE. To
 * remove possibility of locking yourself out from the RF Core, this bit can
 * not be cleared. If you need to disable all clocks to the RF Core, see the
 * PRCM:RFCCLKG.CLK_EN register.
 */

#define RFC_PWR_PWMCLKEN_RFC                                        0x00000001
#define RFC_PWR_PWMCLKEN_RFC_MASK                                   0x00000001
#define RFC_PWR_PWMCLKEN_RFC_SHIFT                                           0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_HW_RFC_PWR_H */
