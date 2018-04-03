/************************************************************************************
 * arch/arm/src/imxrt/imxrt_iomuxc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define IMXRT_IOMUXC_GPR_GPR0_OFFSET                    0x0000 /* GPR0 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR1_OFFSET                    0x0004 /* GPR1 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR2_OFFSET                    0x0008 /* GPR2 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR3_OFFSET                    0x000c /* GPR3 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR4_OFFSET                    0x0010 /* GPR4 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR5_OFFSET                    0x0014 /* GPR5 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR6_OFFSET                    0x0018 /* GPR6 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR7_OFFSET                    0x001c /* GPR7 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR8_OFFSET                    0x0020 /* GPR8 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR9_OFFSET                    0x0024 /* GPR9 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR10_OFFSET                   0x0028 /* GPR10 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR11_OFFSET                   0x002c /* GPR11 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR12_OFFSET                   0x0030 /* GPR12 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR13_OFFSET                   0x0034 /* GPR13 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR14_OFFSET                   0x0038 /* GPR14 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR15_OFFSET                   0x003c /* GPR15 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR16_OFFSET                   0x0040 /* GPR16 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR17_OFFSET                   0x0044 /* GPR17 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR18_OFFSET                   0x0048 /* GPR18 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR19_OFFSET                   0x004c /* GPR19 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR20_OFFSET                   0x0050 /* GPR20 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR21_OFFSET                   0x0054 /* GPR21 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR22_OFFSET                   0x0058 /* GPR22 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR23_OFFSET                   0x005c /* GPR23 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR24_OFFSET                   0x0060 /* GPR24 General Purpose Register*/
#define IMXRT_IOMUXC_GPR_GPR25_OFFSET                   0x0064 /* GPR25 General Purpose Register*/

#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEU_OFFSET          0x0000 /* SW_MUX_CTL_PAD_WAKEUP SW MUX Control Register */
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_ON_REQ_OFFSET    0x0004 /* SW_MUX_CTL_PAD_PMIC_ON_REQ SW MUX Control Register */
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_STBY_REQ_OFFSET  0x0008 /* SW_MUX_CTL_PAD_PMIC_STBY_REQ SW MUX Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_TEST_MODE_OFFSET      0x000c /* SW_PAD_CTL_PAD_TEST_MODE SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_POR_B_OFFSET          0x0010 /* SW_PAD_CTL_PAD_POR_B SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_ONOFF_OFFSET          0x0014 /* SW_PAD_CTL_PAD_ONOFF SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_OFFSET         0x0018 /* SW_PAD_CTL_PAD_WAKEUP SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_ON_REQ_OFFSET    0x001c /* SW_PAD_CTL_PAD_PMIC_ON_REQ SW PAD Control Register */
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_STBY_REQ_OFFSET  0x0020 /* SW_PAD_CTL_PAD_PMIC_STBY_REQ SW PAD Control Register */

#define IMXRT_IOMUXC_SNVS_GPR_GPR0_OFFSET                      0x0000 /* SNVC GPR0 General Purpose Register */
#define IMXRT_IOMUXC_SNVS_GPR_GPR1_OFFSET                      0x0004 /* SNVC GPR1 General Purpose Register */
#define IMXRT_IOMUXC_SNVS_GPR_GPR2_OFFSET                      0x0008 /* SNVC GPR2 General Purpose Register */
#define IMXRT_IOMUXC_SNVS_GPR_GPR3_OFFSET                      0x000c /* SNVC GPR3 General Purpose Register */

/* Pad Mux Registers */
/* Pad Mux Register Indices (used by software for table lookups) */

#define IMXRT_PADMUX_GPIO_EMC_00_INDEX              0
#define IMXRT_PADMUX_GPIO_EMC_01_INDEX              1
#define IMXRT_PADMUX_GPIO_EMC_02_INDEX              2
#define IMXRT_PADMUX_GPIO_EMC_03_INDEX              3
#define IMXRT_PADMUX_GPIO_EMC_04_INDEX              4
#define IMXRT_PADMUX_GPIO_EMC_05_INDEX              5
#define IMXRT_PADMUX_GPIO_EMC_06_INDEX              6
#define IMXRT_PADMUX_GPIO_EMC_07_INDEX              7
#define IMXRT_PADMUX_GPIO_EMC_08_INDEX              8
#define IMXRT_PADMUX_GPIO_EMC_09_INDEX              9
#define IMXRT_PADMUX_GPIO_EMC_10_INDEX              10
#define IMXRT_PADMUX_GPIO_EMC_11_INDEX              11
#define IMXRT_PADMUX_GPIO_EMC_12_INDEX              12
#define IMXRT_PADMUX_GPIO_EMC_13_INDEX              13
#define IMXRT_PADMUX_GPIO_EMC_14_INDEX              14
#define IMXRT_PADMUX_GPIO_EMC_15_INDEX              15
#define IMXRT_PADMUX_GPIO_EMC_16_INDEX              16
#define IMXRT_PADMUX_GPIO_EMC_17_INDEX              17
#define IMXRT_PADMUX_GPIO_EMC_18_INDEX              18
#define IMXRT_PADMUX_GPIO_EMC_19_INDEX              19
#define IMXRT_PADMUX_GPIO_EMC_20_INDEX              20
#define IMXRT_PADMUX_GPIO_EMC_21_INDEX              21
#define IMXRT_PADMUX_GPIO_EMC_22_INDEX              22
#define IMXRT_PADMUX_GPIO_EMC_23_INDEX              23
#define IMXRT_PADMUX_GPIO_EMC_24_INDEX              24
#define IMXRT_PADMUX_GPIO_EMC_25_INDEX              25
#define IMXRT_PADMUX_GPIO_EMC_26_INDEX              26
#define IMXRT_PADMUX_GPIO_EMC_27_INDEX              27
#define IMXRT_PADMUX_GPIO_EMC_28_INDEX              28
#define IMXRT_PADMUX_GPIO_EMC_29_INDEX              29
#define IMXRT_PADMUX_GPIO_EMC_30_INDEX              30
#define IMXRT_PADMUX_GPIO_EMC_31_INDEX              31
#define IMXRT_PADMUX_GPIO_EMC_32_INDEX              32
#define IMXRT_PADMUX_GPIO_EMC_33_INDEX              33
#define IMXRT_PADMUX_GPIO_EMC_34_INDEX              34
#define IMXRT_PADMUX_GPIO_EMC_35_INDEX              35
#define IMXRT_PADMUX_GPIO_EMC_36_INDEX              36
#define IMXRT_PADMUX_GPIO_EMC_37_INDEX              37
#define IMXRT_PADMUX_GPIO_EMC_38_INDEX              38
#define IMXRT_PADMUX_GPIO_EMC_39_INDEX              39
#define IMXRT_PADMUX_GPIO_EMC_40_INDEX              40
#define IMXRT_PADMUX_GPIO_EMC_41_INDEX              41
#define IMXRT_PADMUX_GPIO_AD_B0_00_INDEX            42
#define IMXRT_PADMUX_GPIO_AD_B0_01_INDEX            43
#define IMXRT_PADMUX_GPIO_AD_B0_02_INDEX            44
#define IMXRT_PADMUX_GPIO_AD_B0_03_INDEX            45
#define IMXRT_PADMUX_GPIO_AD_B0_04_INDEX            46
#define IMXRT_PADMUX_GPIO_AD_B0_05_INDEX            47
#define IMXRT_PADMUX_GPIO_AD_B0_06_INDEX            48
#define IMXRT_PADMUX_GPIO_AD_B0_07_INDEX            49
#define IMXRT_PADMUX_GPIO_AD_B0_08_INDEX            50
#define IMXRT_PADMUX_GPIO_AD_B0_09_INDEX            51
#define IMXRT_PADMUX_GPIO_AD_B0_10_INDEX            52
#define IMXRT_PADMUX_GPIO_AD_B0_11_INDEX            53
#define IMXRT_PADMUX_GPIO_AD_B0_12_INDEX            54
#define IMXRT_PADMUX_GPIO_AD_B0_13_INDEX            55
#define IMXRT_PADMUX_GPIO_AD_B0_14_INDEX            56
#define IMXRT_PADMUX_GPIO_AD_B0_15_INDEX            57
#define IMXRT_PADMUX_GPIO_AD_B1_00_INDEX            58
#define IMXRT_PADMUX_GPIO_AD_B1_01_INDEX            59
#define IMXRT_PADMUX_GPIO_AD_B1_02_INDEX            60
#define IMXRT_PADMUX_GPIO_AD_B1_03_INDEX            61
#define IMXRT_PADMUX_GPIO_AD_B1_04_INDEX            62
#define IMXRT_PADMUX_GPIO_AD_B1_05_INDEX            63
#define IMXRT_PADMUX_GPIO_AD_B1_06_INDEX            64
#define IMXRT_PADMUX_GPIO_AD_B1_07_INDEX            65
#define IMXRT_PADMUX_GPIO_AD_B1_08_INDEX            66
#define IMXRT_PADMUX_GPIO_AD_B1_09_INDEX            67
#define IMXRT_PADMUX_GPIO_AD_B1_10_INDEX            68
#define IMXRT_PADMUX_GPIO_AD_B1_11_INDEX            69
#define IMXRT_PADMUX_GPIO_AD_B1_12_INDEX            70
#define IMXRT_PADMUX_GPIO_AD_B1_13_INDEX            71
#define IMXRT_PADMUX_GPIO_AD_B1_14_INDEX            72
#define IMXRT_PADMUX_GPIO_AD_B1_15_INDEX            73
#define IMXRT_PADMUX_GPIO_B0_00_INDEX               74
#define IMXRT_PADMUX_GPIO_B0_01_INDEX               75
#define IMXRT_PADMUX_GPIO_B0_02_INDEX               76
#define IMXRT_PADMUX_GPIO_B0_03_INDEX               77
#define IMXRT_PADMUX_GPIO_B0_04_INDEX               78
#define IMXRT_PADMUX_GPIO_B0_05_INDEX               79
#define IMXRT_PADMUX_GPIO_B0_06_INDEX               80
#define IMXRT_PADMUX_GPIO_B0_07_INDEX               81
#define IMXRT_PADMUX_GPIO_B0_08_INDEX               82
#define IMXRT_PADMUX_GPIO_B0_09_INDEX               83
#define IMXRT_PADMUX_GPIO_B0_10_INDEX               84
#define IMXRT_PADMUX_GPIO_B0_11_INDEX               85
#define IMXRT_PADMUX_GPIO_B0_12_INDEX               86
#define IMXRT_PADMUX_GPIO_B0_13_INDEX               87
#define IMXRT_PADMUX_GPIO_B0_14_INDEX               88
#define IMXRT_PADMUX_GPIO_B0_15_INDEX               89
#define IMXRT_PADMUX_GPIO_B1_00_INDEX               90
#define IMXRT_PADMUX_GPIO_B1_01_INDEX               91
#define IMXRT_PADMUX_GPIO_B1_02_INDEX               92
#define IMXRT_PADMUX_GPIO_B1_03_INDEX               93
#define IMXRT_PADMUX_GPIO_B1_04_INDEX               94
#define IMXRT_PADMUX_GPIO_B1_05_INDEX               95
#define IMXRT_PADMUX_GPIO_B1_06_INDEX               96
#define IMXRT_PADMUX_GPIO_B1_07_INDEX               97
#define IMXRT_PADMUX_GPIO_B1_08_INDEX               98
#define IMXRT_PADMUX_GPIO_B1_09_INDEX               99
#define IMXRT_PADMUX_GPIO_B1_10_INDEX               100
#define IMXRT_PADMUX_GPIO_B1_11_INDEX               101
#define IMXRT_PADMUX_GPIO_B1_12_INDEX               102
#define IMXRT_PADMUX_GPIO_B1_13_INDEX               103
#define IMXRT_PADMUX_GPIO_B1_14_INDEX               104
#define IMXRT_PADMUX_GPIO_B1_15_INDEX               105
#define IMXRT_PADMUX_GPIO_SD_B0_00_INDEX            106
#define IMXRT_PADMUX_GPIO_SD_B0_01_INDEX            107
#define IMXRT_PADMUX_GPIO_SD_B0_02_INDEX            108
#define IMXRT_PADMUX_GPIO_SD_B0_03_INDEX            109
#define IMXRT_PADMUX_GPIO_SD_B0_04_INDEX            110
#define IMXRT_PADMUX_GPIO_SD_B0_05_INDEX            111
#define IMXRT_PADMUX_GPIO_SD_B1_00_INDEX            112
#define IMXRT_PADMUX_GPIO_SD_B1_01_INDEX            113
#define IMXRT_PADMUX_GPIO_SD_B1_02_INDEX            114
#define IMXRT_PADMUX_GPIO_SD_B1_03_INDEX            115
#define IMXRT_PADMUX_GPIO_SD_B1_04_INDEX            116
#define IMXRT_PADMUX_GPIO_SD_B1_05_INDEX            117
#define IMXRT_PADMUX_GPIO_SD_B1_06_INDEX            118
#define IMXRT_PADMUX_GPIO_SD_B1_07_INDEX            119
#define IMXRT_PADMUX_GPIO_SD_B1_08_INDEX            120
#define IMXRT_PADMUX_GPIO_SD_B1_09_INDEX            121
#define IMXRT_PADMUX_GPIO_SD_B1_10_INDEX            122
#define IMXRT_PADMUX_GPIO_SD_B1_11_INDEX            123

/* Pad Control Registers */
/* Pad Mux Register Indices (used by software for table lookups) */

#define IMX_PADCTL_SD2_DATA1_INDEX                  0
#define IMXRT_PADCTL_GPIO_EMC_00_INDEX              0
#define IMXRT_PADCTL_GPIO_EMC_01_INDEX              1
#define IMXRT_PADCTL_GPIO_EMC_02_INDEX              2
#define IMXRT_PADCTL_GPIO_EMC_03_INDEX              3
#define IMXRT_PADCTL_GPIO_EMC_04_INDEX              4
#define IMXRT_PADCTL_GPIO_EMC_05_INDEX              5
#define IMXRT_PADCTL_GPIO_EMC_06_INDEX              6
#define IMXRT_PADCTL_GPIO_EMC_07_INDEX              7
#define IMXRT_PADCTL_GPIO_EMC_08_INDEX              8
#define IMXRT_PADCTL_GPIO_EMC_09_INDEX              9
#define IMXRT_PADCTL_GPIO_EMC_10_INDEX              10
#define IMXRT_PADCTL_GPIO_EMC_11_INDEX              11
#define IMXRT_PADCTL_GPIO_EMC_12_INDEX              12
#define IMXRT_PADCTL_GPIO_EMC_13_INDEX              13
#define IMXRT_PADCTL_GPIO_EMC_14_INDEX              14
#define IMXRT_PADCTL_GPIO_EMC_15_INDEX              15
#define IMXRT_PADCTL_GPIO_EMC_16_INDEX              16
#define IMXRT_PADCTL_GPIO_EMC_17_INDEX              17
#define IMXRT_PADCTL_GPIO_EMC_18_INDEX              18
#define IMXRT_PADCTL_GPIO_EMC_19_INDEX              19
#define IMXRT_PADCTL_GPIO_EMC_20_INDEX              20
#define IMXRT_PADCTL_GPIO_EMC_21_INDEX              21
#define IMXRT_PADCTL_GPIO_EMC_22_INDEX              22
#define IMXRT_PADCTL_GPIO_EMC_23_INDEX              23
#define IMXRT_PADCTL_GPIO_EMC_24_INDEX              24
#define IMXRT_PADCTL_GPIO_EMC_25_INDEX              25
#define IMXRT_PADCTL_GPIO_EMC_26_INDEX              26
#define IMXRT_PADCTL_GPIO_EMC_27_INDEX              27
#define IMXRT_PADCTL_GPIO_EMC_28_INDEX              28
#define IMXRT_PADCTL_GPIO_EMC_29_INDEX              29
#define IMXRT_PADCTL_GPIO_EMC_30_INDEX              30
#define IMXRT_PADCTL_GPIO_EMC_31_INDEX              31
#define IMXRT_PADCTL_GPIO_EMC_32_INDEX              32
#define IMXRT_PADCTL_GPIO_EMC_33_INDEX              33
#define IMXRT_PADCTL_GPIO_EMC_34_INDEX              34
#define IMXRT_PADCTL_GPIO_EMC_35_INDEX              35
#define IMXRT_PADCTL_GPIO_EMC_36_INDEX              36
#define IMXRT_PADCTL_GPIO_EMC_37_INDEX              37
#define IMXRT_PADCTL_GPIO_EMC_38_INDEX              38
#define IMXRT_PADCTL_GPIO_EMC_39_INDEX              39
#define IMXRT_PADCTL_GPIO_EMC_40_INDEX              40
#define IMXRT_PADCTL_GPIO_EMC_41_INDEX              41
#define IMXRT_PADCTL_GPIO_AD_B0_00_INDEX            42
#define IMXRT_PADCTL_GPIO_AD_B0_01_INDEX            43
#define IMXRT_PADCTL_GPIO_AD_B0_02_INDEX            44
#define IMXRT_PADCTL_GPIO_AD_B0_03_INDEX            45
#define IMXRT_PADCTL_GPIO_AD_B0_04_INDEX            46
#define IMXRT_PADCTL_GPIO_AD_B0_05_INDEX            47
#define IMXRT_PADCTL_GPIO_AD_B0_06_INDEX            48
#define IMXRT_PADCTL_GPIO_AD_B0_07_INDEX            49
#define IMXRT_PADCTL_GPIO_AD_B0_08_INDEX            50
#define IMXRT_PADCTL_GPIO_AD_B0_09_INDEX            51
#define IMXRT_PADCTL_GPIO_AD_B0_10_INDEX            52
#define IMXRT_PADCTL_GPIO_AD_B0_11_INDEX            53
#define IMXRT_PADCTL_GPIO_AD_B0_12_INDEX            54
#define IMXRT_PADCTL_GPIO_AD_B0_13_INDEX            55
#define IMXRT_PADCTL_GPIO_AD_B0_14_INDEX            56
#define IMXRT_PADCTL_GPIO_AD_B0_15_INDEX            57
#define IMXRT_PADCTL_GPIO_AD_B1_00_INDEX            58
#define IMXRT_PADCTL_GPIO_AD_B1_01_INDEX            59
#define IMXRT_PADCTL_GPIO_AD_B1_02_INDEX            60
#define IMXRT_PADCTL_GPIO_AD_B1_03_INDEX            61
#define IMXRT_PADCTL_GPIO_AD_B1_04_INDEX            62
#define IMXRT_PADCTL_GPIO_AD_B1_05_INDEX            63
#define IMXRT_PADCTL_GPIO_AD_B1_06_INDEX            64
#define IMXRT_PADCTL_GPIO_AD_B1_07_INDEX            65
#define IMXRT_PADCTL_GPIO_AD_B1_08_INDEX            66
#define IMXRT_PADCTL_GPIO_AD_B1_09_INDEX            67
#define IMXRT_PADCTL_GPIO_AD_B1_10_INDEX            68
#define IMXRT_PADCTL_GPIO_AD_B1_11_INDEX            69
#define IMXRT_PADCTL_GPIO_AD_B1_12_INDEX            70
#define IMXRT_PADCTL_GPIO_AD_B1_13_INDEX            71
#define IMXRT_PADCTL_GPIO_AD_B1_14_INDEX            72
#define IMXRT_PADCTL_GPIO_AD_B1_15_INDEX            73
#define IMXRT_PADCTL_GPIO_B0_00_INDEX               74
#define IMXRT_PADCTL_GPIO_B0_01_INDEX               75
#define IMXRT_PADCTL_GPIO_B0_02_INDEX               76
#define IMXRT_PADCTL_GPIO_B0_03_INDEX               77
#define IMXRT_PADCTL_GPIO_B0_04_INDEX               78
#define IMXRT_PADCTL_GPIO_B0_05_INDEX               79
#define IMXRT_PADCTL_GPIO_B0_06_INDEX               80
#define IMXRT_PADCTL_GPIO_B0_07_INDEX               81
#define IMXRT_PADCTL_GPIO_B0_08_INDEX               82
#define IMXRT_PADCTL_GPIO_B0_09_INDEX               83
#define IMXRT_PADCTL_GPIO_B0_10_INDEX               84
#define IMXRT_PADCTL_GPIO_B0_11_INDEX               85
#define IMXRT_PADCTL_GPIO_B0_12_INDEX               86
#define IMXRT_PADCTL_GPIO_B0_13_INDEX               87
#define IMXRT_PADCTL_GPIO_B0_14_INDEX               88
#define IMXRT_PADCTL_GPIO_B0_15_INDEX               89
#define IMXRT_PADCTL_GPIO_B1_00_INDEX               90
#define IMXRT_PADCTL_GPIO_B1_01_INDEX               91
#define IMXRT_PADCTL_GPIO_B1_02_INDEX               92
#define IMXRT_PADCTL_GPIO_B1_03_INDEX               93
#define IMXRT_PADCTL_GPIO_B1_04_INDEX               94
#define IMXRT_PADCTL_GPIO_B1_05_INDEX               95
#define IMXRT_PADCTL_GPIO_B1_06_INDEX               96
#define IMXRT_PADCTL_GPIO_B1_07_INDEX               97
#define IMXRT_PADCTL_GPIO_B1_08_INDEX               98
#define IMXRT_PADCTL_GPIO_B1_09_INDEX               99
#define IMXRT_PADCTL_GPIO_B1_10_INDEX               100
#define IMXRT_PADCTL_GPIO_B1_11_INDEX               101
#define IMXRT_PADCTL_GPIO_B1_12_INDEX               102
#define IMXRT_PADCTL_GPIO_B1_13_INDEX               103
#define IMXRT_PADCTL_GPIO_B1_14_INDEX               104
#define IMXRT_PADCTL_GPIO_B1_15_INDEX               105
#define IMXRT_PADCTL_GPIO_SD_B0_00_INDEX            106
#define IMXRT_PADCTL_GPIO_SD_B0_01_INDEX            107
#define IMXRT_PADCTL_GPIO_SD_B0_02_INDEX            108
#define IMXRT_PADCTL_GPIO_SD_B0_03_INDEX            109
#define IMXRT_PADCTL_GPIO_SD_B0_04_INDEX            110
#define IMXRT_PADCTL_GPIO_SD_B0_05_INDEX            111
#define IMXRT_PADCTL_GPIO_SD_B1_00_INDEX            112
#define IMXRT_PADCTL_GPIO_SD_B1_01_INDEX            113
#define IMXRT_PADCTL_GPIO_SD_B1_02_INDEX            114
#define IMXRT_PADCTL_GPIO_SD_B1_03_INDEX            115
#define IMXRT_PADCTL_GPIO_SD_B1_04_INDEX            116
#define IMXRT_PADCTL_GPIO_SD_B1_05_INDEX            117
#define IMXRT_PADCTL_GPIO_SD_B1_06_INDEX            118
#define IMXRT_PADCTL_GPIO_SD_B1_07_INDEX            119
#define IMXRT_PADCTL_GPIO_SD_B1_08_INDEX            120
#define IMXRT_PADCTL_GPIO_SD_B1_09_INDEX            121
#define IMXRT_PADCTL_GPIO_SD_B1_10_INDEX            122
#define IMXRT_PADCTL_GPIO_SD_B1_11_INDEX            123

/* Register addresses ***************************************************************/

#define IMXRT_IOMUXC_GPR_GPR0            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR0_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR1            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR1_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR2            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR2_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR3            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR3_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR4            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR4_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR5            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR5_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR6            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR6_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR7            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR7_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR8            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR8_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR9            (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR9_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR10           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR10_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR11           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR11_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR12           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR12_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR13           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR13_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR14           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR14_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR15           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR15_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR16           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR16_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR17           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR17_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR18           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR18_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR19           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR19_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR20           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR20_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR21           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR21_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR22           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR22_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR23           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR23_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR24           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR24_OFFSET)
#define IMXRT_IOMUXC_GPR_GPR25           (IMXRT_IOMUXCGPR_BASE + IMXRT_IOMUXC_GPR_GPR25_OFFSET)

#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEU         (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEU_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_ON_REQ   (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_ON_REQ_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_STBY_REQ (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_MUX_CTL_PAD_PMIC_STBY_REQ_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_TEST_MODE     (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_TEST_MODE_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_POR_B         (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_POR_B_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_ONOFF         (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_ONOFF_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP        (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_ON_REQ   (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_ON_REQ_OFFSET)
#define IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_STBY_REQ (IMXRT_IOMUXCSNVS_BASE + IMXRT_IOMUXC_SNVS_SW_PAD_CTL_PAD_PMIC_STBY_REQ_OFFSET)

#define IMXRT_IOMUXC_SNVS_GPR_GPR0                     (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR0_OFFSET)
#define IMXRT_IOMUXC_SNVS_GPR_GPR1                     (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR1_OFFSET)
#define IMXRT_IOMUXC_SNVS_GPR_GPR2                     (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR2_OFFSET)
#define IMXRT_IOMUXC_SNVS_GPR_GPR3                     (IMXRT_IOMUXCSNVSGPR_BASE + IMXRT_IOMUXC_SNVS_GPR_GPR3_OFFSET)

/* Register bit definitions *********************************************************/


#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_IOMUXC_H */
