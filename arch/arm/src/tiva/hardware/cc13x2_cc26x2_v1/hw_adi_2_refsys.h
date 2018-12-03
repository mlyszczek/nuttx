/******************************************************************************
 *  Filename:       hw_adi_2_refsys_h
 *  Revised:        2017-01-10 11:54:43 +0100 (Tue, 10 Jan 2017)
 *  Revision:       48190
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_ADI_2_REFSYS_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_ADI_2_REFSYS_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * ADI_2_REFSYS component
 *
 ******************************************************************************
 * Internal
 */

#define ADI_2_REFSYS_REFSYSCTL0_OFFSET                              0x00000000

/* Internal */

#define ADI_2_REFSYS_SOCLDOCTL0_OFFSET                              0x00000002

/* Internal */

#define ADI_2_REFSYS_SOCLDOCTL1_OFFSET                              0x00000003

/* Internal */

#define ADI_2_REFSYS_SOCLDOCTL2_OFFSET                              0x00000004

/* Internal */

#define ADI_2_REFSYS_SOCLDOCTL3_OFFSET                              0x00000005

/* Internal */

#define ADI_2_REFSYS_SOCLDOCTL4_OFFSET                              0x00000006

/* Internal */

#define ADI_2_REFSYS_SOCLDOCTL5_OFFSET                              0x00000007

/* Internal */

#define ADI_2_REFSYS_HPOSCCTL0_OFFSET                               0x0000000a

/* Internal */

#define ADI_2_REFSYS_HPOSCCTL1_OFFSET                               0x0000000b

/* Internal */

#define ADI_2_REFSYS_HPOSCCTL2_OFFSET                               0x0000000c

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_REFSYSCTL0
 *
 ******************************************************************************
 * Field:   [4:0] TRIM_IREF
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_REFSYSCTL0_TRIM_IREF_MASK                      0x0000001f
#define ADI_2_REFSYS_REFSYSCTL0_TRIM_IREF_SHIFT                              0

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_SOCLDOCTL0
 *
 ******************************************************************************
 * Field:   [7:4] VTRIM_UDIG
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_MASK                     0x000000f0
#define ADI_2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_SHIFT                             4

/* Field:   [3:0] VTRIM_BOD
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL0_VTRIM_BOD_MASK                      0x0000000f
#define ADI_2_REFSYS_SOCLDOCTL0_VTRIM_BOD_SHIFT                              0

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_SOCLDOCTL1
 *
 ******************************************************************************
 * Field:   [7:4] VTRIM_COARSE
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL1_VTRIM_COARSE_MASK                   0x000000f0
#define ADI_2_REFSYS_SOCLDOCTL1_VTRIM_COARSE_SHIFT                           4

/* Field:   [3:0] VTRIM_DIG
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL1_VTRIM_DIG_MASK                      0x0000000f
#define ADI_2_REFSYS_SOCLDOCTL1_VTRIM_DIG_SHIFT                              0

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_SOCLDOCTL2
 *
 ******************************************************************************
 * Field:   [2:0] VTRIM_DELTA
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL2_VTRIM_DELTA_MASK                    0x00000007
#define ADI_2_REFSYS_SOCLDOCTL2_VTRIM_DELTA_SHIFT                            0

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_SOCLDOCTL3
 *
 ******************************************************************************
 * Field:   [7:6] ITRIM_DIGLDO_LOAD
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD_MASK              0x000000c0
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_LOAD_SHIFT                      6

/* Field:   [5:3] ITRIM_DIGLDO
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * BIAS_120P                Internal. Only to be used through TI provided API.
 * BIAS_100P                Internal. Only to be used through TI provided API.
 * BIAS_80P                 Internal. Only to be used through TI provided API.
 * BIAS_60P                 Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_MASK                   0x00000038
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_SHIFT                           3
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_120P              0x00000038
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_100P              0x00000028
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_80P               0x00000018
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_DIGLDO_BIAS_60P               0x00000000

/* Field:   [2:0] ITRIM_UDIGLDO
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO_MASK                  0x00000007
#define ADI_2_REFSYS_SOCLDOCTL3_ITRIM_UDIGLDO_SHIFT                          0

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_SOCLDOCTL4
 *
 ******************************************************************************
 * Field:   [6:5] UDIG_ITEST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN_MASK                  0x00000060
#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_ITEST_EN_SHIFT                          5

/* Field:   [4:2] DIG_ITEST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN_MASK                   0x0000001c
#define ADI_2_REFSYS_SOCLDOCTL4_DIG_ITEST_EN_SHIFT                           2

/* Field:     [1] BIAS_DIS
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL4_BIAS_DIS                            0x00000002
#define ADI_2_REFSYS_SOCLDOCTL4_BIAS_DIS_MASK                       0x00000002
#define ADI_2_REFSYS_SOCLDOCTL4_BIAS_DIS_SHIFT                               1

/* Field:     [0] UDIG_LDO_EN
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * EN                       Internal. Only to be used through TI provided API.
 * DIS                      Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_LDO_EN                         0x00000001
#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_LDO_EN_MASK                    0x00000001
#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_LDO_EN_SHIFT                            0
#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_LDO_EN_EN                      0x00000001
#define ADI_2_REFSYS_SOCLDOCTL4_UDIG_LDO_EN_DIS                     0x00000000

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_SOCLDOCTL5
 *
 ******************************************************************************
 * Field:     [3] IMON_ITEST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL5_IMON_ITEST_EN                       0x00000008
#define ADI_2_REFSYS_SOCLDOCTL5_IMON_ITEST_EN_MASK                  0x00000008
#define ADI_2_REFSYS_SOCLDOCTL5_IMON_ITEST_EN_SHIFT                          3

/* Field:   [2:0] TESTSEL
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * VDD_AON                  Internal. Only to be used through TI provided API.
 * VREF_AMP                 Internal. Only to be used through TI provided API.
 * ITEST                    Internal. Only to be used through TI provided API.
 * NC                       Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_SOCLDOCTL5_TESTSEL_MASK                        0x00000007
#define ADI_2_REFSYS_SOCLDOCTL5_TESTSEL_SHIFT                                0
#define ADI_2_REFSYS_SOCLDOCTL5_TESTSEL_VDD_AON                     0x00000004
#define ADI_2_REFSYS_SOCLDOCTL5_TESTSEL_VREF_AMP                    0x00000002
#define ADI_2_REFSYS_SOCLDOCTL5_TESTSEL_ITEST                       0x00000001
#define ADI_2_REFSYS_SOCLDOCTL5_TESTSEL_NC                          0x00000000

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_HPOSCCTL0
 *
 ******************************************************************************
 * Field:     [7] FILTER_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL0_FILTER_EN                            0x00000080
#define ADI_2_REFSYS_HPOSCCTL0_FILTER_EN_MASK                       0x00000080
#define ADI_2_REFSYS_HPOSCCTL0_FILTER_EN_SHIFT                               7

/* Field:   [6:5] BIAS_RECHARGE_DLY
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * MIN_DLY_X8               Internal. Only to be used through TI provided API.
 * MIN_DLY_X4               Internal. Only to be used through TI provided API.
 * MIN_DLY_X2               Internal. Only to be used through TI provided API.
 * MIN_DLY_X1               Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MASK               0x00000060
#define ADI_2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_SHIFT                       5
#define ADI_2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X8         0x00000060
#define ADI_2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X4         0x00000040
#define ADI_2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X2         0x00000020
#define ADI_2_REFSYS_HPOSCCTL0_BIAS_RECHARGE_DLY_MIN_DLY_X1         0x00000000

/* Field:   [4:3] TUNE_CAP
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * SHIFT_M108               Internal. Only to be used through TI provided API.
 * SHIFT_M70                Internal. Only to be used through TI provided API.
 * SHIFT_M35                Internal. Only to be used through TI provided API.
 * SHIFT_0                  Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL0_TUNE_CAP_MASK                        0x00000018
#define ADI_2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT                                3
#define ADI_2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_M108                  0x00000018
#define ADI_2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_M70                   0x00000010
#define ADI_2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_M35                   0x00000008
#define ADI_2_REFSYS_HPOSCCTL0_TUNE_CAP_SHIFT_0                     0x00000000

/* Field:   [2:1] SERIES_CAP
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL0_SERIES_CAP_MASK                      0x00000006
#define ADI_2_REFSYS_HPOSCCTL0_SERIES_CAP_SHIFT                              1

/* Field:     [0] DIV3_BYPASS
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * HPOSC_2520MHZ            Internal. Only to be used through TI provided API.
 * HPOSC_840MHZ             Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL0_DIV3_BYPASS                          0x00000001
#define ADI_2_REFSYS_HPOSCCTL0_DIV3_BYPASS_MASK                     0x00000001
#define ADI_2_REFSYS_HPOSCCTL0_DIV3_BYPASS_SHIFT                             0
#define ADI_2_REFSYS_HPOSCCTL0_DIV3_BYPASS_HPOSC_2520MHZ            0x00000001
#define ADI_2_REFSYS_HPOSCCTL0_DIV3_BYPASS_HPOSC_840MHZ             0x00000000

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_HPOSCCTL1
 *
 ******************************************************************************
 * Field:     [5] BIAS_DIS
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL1_BIAS_DIS                             0x00000020
#define ADI_2_REFSYS_HPOSCCTL1_BIAS_DIS_MASK                        0x00000020
#define ADI_2_REFSYS_HPOSCCTL1_BIAS_DIS_SHIFT                                5

/* Field:     [4] PWRDET_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL1_PWRDET_EN                            0x00000010
#define ADI_2_REFSYS_HPOSCCTL1_PWRDET_EN_MASK                       0x00000010
#define ADI_2_REFSYS_HPOSCCTL1_PWRDET_EN_SHIFT                               4

/* Field:   [3:0] BIAS_RES_SET
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL1_BIAS_RES_SET_MASK                    0x0000000f
#define ADI_2_REFSYS_HPOSCCTL1_BIAS_RES_SET_SHIFT                            0

/******************************************************************************
 *
 * Register: ADI_2_REFSYS_HPOSCCTL2
 *
 ******************************************************************************
 * Field:     [7] BIAS_HOLD_MODE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL2_BIAS_HOLD_MODE_EN                    0x00000080
#define ADI_2_REFSYS_HPOSCCTL2_BIAS_HOLD_MODE_EN_MASK               0x00000080
#define ADI_2_REFSYS_HPOSCCTL2_BIAS_HOLD_MODE_EN_SHIFT                       7

/* Field:     [6] TESTMUX_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL2_TESTMUX_EN                           0x00000040
#define ADI_2_REFSYS_HPOSCCTL2_TESTMUX_EN_MASK                      0x00000040
#define ADI_2_REFSYS_HPOSCCTL2_TESTMUX_EN_SHIFT                              6

/* Field:   [5:4] ATEST_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL2_ATEST_SEL_MASK                       0x00000030
#define ADI_2_REFSYS_HPOSCCTL2_ATEST_SEL_SHIFT                               4

/* Field:   [3:0] CURRMIRR_RATIO
 *
 * Internal. Only to be used through TI provided API.
 */

#define ADI_2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO_MASK                  0x0000000f
#define ADI_2_REFSYS_HPOSCCTL2_CURRMIRR_RATIO_SHIFT                          0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_ADI_2_REFSYS_H */
