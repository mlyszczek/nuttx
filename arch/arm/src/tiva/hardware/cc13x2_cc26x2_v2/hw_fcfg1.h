/******************************************************************************
 *  Filename:       hw_fcfg1_h
 *  Revised:        2018-06-29 13:33:23 +0200 (Fri, 29 Jun 2018)
 *  Revision:       52241
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_FCFG1_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_FCFG1_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * FCFG1 component
 *
 ******************************************************************************
 * Misc configurations
 */

#define FCFG1_MISC_CONF_1_OFFSET                                    0x000000a0

/* Internal */

#define FCFG1_MISC_CONF_2_OFFSET                                    0x000000a4

/* Internal */

#define FCFG1_CONFIG_CC26_FE_OFFSET                                 0x000000c4

/* Internal */

#define FCFG1_CONFIG_CC13_FE_OFFSET                                 0x000000c8

/* Internal */

#define FCFG1_CONFIG_RF_COMMON_OFFSET                               0x000000cc

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_OFFSET                     0x000000d0

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_OFFSET                     0x000000d4

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_OFFSET                      0x000000d8

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_OFFSET                      0x000000dc

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV4_CC26_OFFSET                         0x000000e0

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV4_CC13_OFFSET                         0x000000e4

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV5_OFFSET                              0x000000e8

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV6_CC26_OFFSET                         0x000000ec

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV6_CC13_OFFSET                         0x000000f0

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV10_OFFSET                             0x000000f4

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV12_CC26_OFFSET                        0x000000f8

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV12_CC13_OFFSET                        0x000000fc

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV15_OFFSET                             0x00000100

/* Internal */

#define FCFG1_CONFIG_SYNTH_DIV30_OFFSET                             0x00000104

/* Flash information */

#define FCFG1_FLASH_NUMBER_OFFSET                                   0x00000164

/* Flash information */

#define FCFG1_FLASH_COORDINATE_OFFSET                               0x0000016c

/* Internal */

#define FCFG1_FLASH_E_P_OFFSET                                      0x00000170

/* Internal */

#define FCFG1_FLASH_C_E_P_R_OFFSET                                  0x00000174

/* Internal */

#define FCFG1_FLASH_P_R_PV_OFFSET                                   0x00000178

/* Internal */

#define FCFG1_FLASH_EH_SEQ_OFFSET                                   0x0000017c

/* Internal */

#define FCFG1_FLASH_VHV_E_OFFSET                                    0x00000180

/* Internal */

#define FCFG1_FLASH_PP_OFFSET                                       0x00000184

/* Internal */

#define FCFG1_FLASH_PROG_EP_OFFSET                                  0x00000188

/* Internal */

#define FCFG1_FLASH_ERA_PW_OFFSET                                   0x0000018c

/* Internal */

#define FCFG1_FLASH_VHV_OFFSET                                      0x00000190

/* Internal */

#define FCFG1_FLASH_VHV_PV_OFFSET                                   0x00000194

/* Internal */

#define FCFG1_FLASH_V_OFFSET                                        0x00000198

/* User Identification. */

#define FCFG1_USER_ID_OFFSET                                        0x00000294

/* Internal */

#define FCFG1_FLASH_OTP_DATA3_OFFSET                                0x000002b0

/* Internal */

#define FCFG1_ANA2_TRIM_OFFSET                                      0x000002b4

/* Internal */

#define FCFG1_LDO_TRIM_OFFSET                                       0x000002b8

/* MAC BLE Address 0 */

#define FCFG1_MAC_BLE_0_OFFSET                                      0x000002e8

/* MAC BLE Address 1 */

#define FCFG1_MAC_BLE_1_OFFSET                                      0x000002ec

/* MAC IEEE 802.15.4 Address 0 */

#define FCFG1_MAC_15_4_0_OFFSET                                     0x000002f0

/* MAC IEEE 802.15.4 Address 1 */

#define FCFG1_MAC_15_4_1_OFFSET                                     0x000002f4

/* Internal */

#define FCFG1_FLASH_OTP_DATA4_OFFSET                                0x00000308

/* Miscellaneous Trim  Parameters */

#define FCFG1_MISC_TRIM_OFFSET                                      0x0000030c

/* Internal */

#define FCFG1_RCOSC_HF_TEMPCOMP_OFFSET                              0x00000310

/* IcePick Device Identification */

#define FCFG1_ICEPICK_DEVICE_ID_OFFSET                              0x00000318

/* Factory Configuration (FCFG1) Revision */

#define FCFG1_FCFG1_REVISION_OFFSET                                 0x0000031c

/* Misc OTP Data */

#define FCFG1_MISC_OTP_DATA_OFFSET                                  0x00000320

/* IO Configuration */

#define FCFG1_IOCONF_OFFSET                                         0x00000344

/* Internal */

#define FCFG1_CONFIG_IF_ADC_OFFSET                                  0x0000034c

/* Internal */

#define FCFG1_CONFIG_OSC_TOP_OFFSET                                 0x00000350

/* AUX_ADC Gain in Absolute Reference Mode */

#define FCFG1_SOC_ADC_ABS_GAIN_OFFSET                               0x0000035c

/* AUX_ADC Gain in Relative Reference Mode */

#define FCFG1_SOC_ADC_REL_GAIN_OFFSET                               0x00000360

/* AUX_ADC Temperature Offsets in Absolute Reference Mode */

#define FCFG1_SOC_ADC_OFFSET_INT_OFFSET                             0x00000368

/* Internal */

#define FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_OFFSET                0x0000036c

/* Internal */

#define FCFG1_AMPCOMP_TH1_OFFSET                                    0x00000370

/* Internal */

#define FCFG1_AMPCOMP_TH2_OFFSET                                    0x00000374

/* Internal */

#define FCFG1_AMPCOMP_CTRL1_OFFSET                                  0x00000378

/* Internal */

#define FCFG1_ANABYPASS_VALUE2_OFFSET                               0x0000037c

/* Internal */

#define FCFG1_VOLT_TRIM_OFFSET                                      0x00000388

/* OSC Configuration */

#define FCFG1_OSC_CONF_OFFSET                                       0x0000038c

/* Internal */

#define FCFG1_FREQ_OFFSET_OFFSET                                    0x00000390

/* Internal */

#define FCFG1_MISC_OTP_DATA_1_OFFSET                                0x00000398

/* Power Down Current Control 20C */

#define FCFG1_PWD_CURR_20C_OFFSET                                   0x0000039c

/* Power Down Current Control 35C */

#define FCFG1_PWD_CURR_35C_OFFSET                                   0x000003a0

/* Power Down Current Control 50C */

#define FCFG1_PWD_CURR_50C_OFFSET                                   0x000003a4

/* Power Down Current Control 65C */

#define FCFG1_PWD_CURR_65C_OFFSET                                   0x000003a8

/* Power Down Current Control 80C */

#define FCFG1_PWD_CURR_80C_OFFSET                                   0x000003ac

/* Power Down Current Control 95C */

#define FCFG1_PWD_CURR_95C_OFFSET                                   0x000003b0

/* Power Down Current Control 110C */

#define FCFG1_PWD_CURR_110C_OFFSET                                  0x000003b4

/* Power Down Current Control 125C */

#define FCFG1_PWD_CURR_125C_OFFSET                                  0x000003b8

/* Shadow of EFUSE:DIE_ID_0 */

#define FCFG1_SHDW_DIE_ID_0_OFFSET                                  0x000003d0

/* Shadow of EFUSE:DIE_ID_1 */

#define FCFG1_SHDW_DIE_ID_1_OFFSET                                  0x000003d4

/* Shadow of EFUSE:DIE_ID_2 */

#define FCFG1_SHDW_DIE_ID_2_OFFSET                                  0x000003d8

/* Shadow of EFUSE:DIE_ID_3 */

#define FCFG1_SHDW_DIE_ID_3_OFFSET                                  0x000003dc

/* Internal */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_OFFSET                         0x000003f8

/* Internal */

#define FCFG1_SHDW_ANA_TRIM_OFFSET                                  0x000003fc

/* Internal */

#define FCFG1_DAC_BIAS_CNF_OFFSET                                   0x0000040c

/* Internal */

#define FCFG1_TFW_PROBE_OFFSET                                      0x00000418

/* Internal */

#define FCFG1_TFW_FT_OFFSET                                         0x0000041c

/* Internal */

#define FCFG1_DAC_CAL0_OFFSET                                       0x00000420

/* Internal */

#define FCFG1_DAC_CAL1_OFFSET                                       0x00000424

/* Internal */

#define FCFG1_DAC_CAL2_OFFSET                                       0x00000428

/* Internal */

#define FCFG1_DAC_CAL3_OFFSET                                       0x0000042c

/******************************************************************************
 *
 * Register: FCFG1_MISC_CONF_1
 *
 ******************************************************************************
 * Field:   [7:0] DEVICE_MINOR_REV
 *
 * HW minor revision number (a value of 0xff shall be treated equally to 0x00).
 * Any test of this field by SW should be implemented as a 'greater or equal'
 * comparison as signed integer.
 * Value may change without warning.
 */

#define FCFG1_MISC_CONF_1_DEVICE_MINOR_REV_MASK                     0x000000ff
#define FCFG1_MISC_CONF_1_DEVICE_MINOR_REV_SHIFT                             0

/******************************************************************************
 *
 * Register: FCFG1_MISC_CONF_2
 *
 ******************************************************************************
 * Field:   [7:0] HPOSC_COMP_P3
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_CONF_2_HPOSC_COMP_P3_MASK                        0x000000ff
#define FCFG1_MISC_CONF_2_HPOSC_COMP_P3_SHIFT                                0

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_CC26_FE
 *
 ******************************************************************************
 * Field: [31:28] IFAMP_IB
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_IFAMP_IB_MASK                          0xf0000000
#define FCFG1_CONFIG_CC26_FE_IFAMP_IB_SHIFT                                 28

/* Field: [27:24] LNA_IB
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_LNA_IB_MASK                            0x0f000000
#define FCFG1_CONFIG_CC26_FE_LNA_IB_SHIFT                                   24

/* Field: [23:19] IFAMP_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_IFAMP_TRIM_MASK                        0x00f80000
#define FCFG1_CONFIG_CC26_FE_IFAMP_TRIM_SHIFT                               19

/* Field: [18:14] CTL_PA0_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_CTL_PA0_TRIM_MASK                      0x0007c000
#define FCFG1_CONFIG_CC26_FE_CTL_PA0_TRIM_SHIFT                             14

/* Field:    [13] PATRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_PATRIMCOMPLETE_N                       0x00002000
#define FCFG1_CONFIG_CC26_FE_PATRIMCOMPLETE_N_MASK                  0x00002000
#define FCFG1_CONFIG_CC26_FE_PATRIMCOMPLETE_N_SHIFT                         13

/* Field:    [12] RSSITRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_RSSITRIMCOMPLETE_N                     0x00001000
#define FCFG1_CONFIG_CC26_FE_RSSITRIMCOMPLETE_N_MASK                0x00001000
#define FCFG1_CONFIG_CC26_FE_RSSITRIMCOMPLETE_N_SHIFT                       12

/* Field:   [7:0] RSSI_OFFSET
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC26_FE_RSSI_OFFSET_MASK                       0x000000ff
#define FCFG1_CONFIG_CC26_FE_RSSI_OFFSET_SHIFT                               0

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_CC13_FE
 *
 ******************************************************************************
 * Field: [31:28] IFAMP_IB
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_IFAMP_IB_MASK                          0xf0000000
#define FCFG1_CONFIG_CC13_FE_IFAMP_IB_SHIFT                                 28

/* Field: [27:24] LNA_IB
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_LNA_IB_MASK                            0x0f000000
#define FCFG1_CONFIG_CC13_FE_LNA_IB_SHIFT                                   24

/* Field: [23:19] IFAMP_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_IFAMP_TRIM_MASK                        0x00f80000
#define FCFG1_CONFIG_CC13_FE_IFAMP_TRIM_SHIFT                               19

/* Field: [18:14] CTL_PA0_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_CTL_PA0_TRIM_MASK                      0x0007c000
#define FCFG1_CONFIG_CC13_FE_CTL_PA0_TRIM_SHIFT                             14

/* Field:    [13] PATRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_PATRIMCOMPLETE_N                       0x00002000
#define FCFG1_CONFIG_CC13_FE_PATRIMCOMPLETE_N_MASK                  0x00002000
#define FCFG1_CONFIG_CC13_FE_PATRIMCOMPLETE_N_SHIFT                         13

/* Field:    [12] RSSITRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_RSSITRIMCOMPLETE_N                     0x00001000
#define FCFG1_CONFIG_CC13_FE_RSSITRIMCOMPLETE_N_MASK                0x00001000
#define FCFG1_CONFIG_CC13_FE_RSSITRIMCOMPLETE_N_SHIFT                       12

/* Field:   [7:0] RSSI_OFFSET
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_CC13_FE_RSSI_OFFSET_MASK                       0x000000ff
#define FCFG1_CONFIG_CC13_FE_RSSI_OFFSET_SHIFT                               0

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_RF_COMMON
 *
 ******************************************************************************
 * Field:    [31] DISABLE_CORNER_CAP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_DISABLE_CORNER_CAP                   0x80000000
#define FCFG1_CONFIG_RF_COMMON_DISABLE_CORNER_CAP_MASK              0x80000000
#define FCFG1_CONFIG_RF_COMMON_DISABLE_CORNER_CAP_SHIFT                     31

/* Field: [30:25] SLDO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_SLDO_TRIM_OUTPUT_MASK                0x7e000000
#define FCFG1_CONFIG_RF_COMMON_SLDO_TRIM_OUTPUT_SHIFT                       25

/* Field:    [21] PA20DBMTRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_PA20DBMTRIMCOMPLETE_N                0x00200000
#define FCFG1_CONFIG_RF_COMMON_PA20DBMTRIMCOMPLETE_N_MASK           0x00200000
#define FCFG1_CONFIG_RF_COMMON_PA20DBMTRIMCOMPLETE_N_SHIFT                  21

/* Field: [20:16] CTL_PA_20DBM_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_CTL_PA_20DBM_TRIM_MASK               0x001f0000
#define FCFG1_CONFIG_RF_COMMON_CTL_PA_20DBM_TRIM_SHIFT                      16

/* Field:  [15:9] RFLDO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_RFLDO_TRIM_OUTPUT_MASK               0x0000fe00
#define FCFG1_CONFIG_RF_COMMON_RFLDO_TRIM_OUTPUT_SHIFT                       9

/* Field:   [8:6] QUANTCTLTHRES
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_QUANTCTLTHRES_MASK                   0x000001c0
#define FCFG1_CONFIG_RF_COMMON_QUANTCTLTHRES_SHIFT                           6

/* Field:   [5:0] DACTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_RF_COMMON_DACTRIM_MASK                         0x0000003f
#define FCFG1_CONFIG_RF_COMMON_DACTRIM_SHIFT                                 0

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_MIN_ALLOWED_RTRIM_MASK     0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_MIN_ALLOWED_RTRIM_SHIFT            28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_RFC_MDM_DEMIQMC0_MASK      0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_RFC_MDM_DEMIQMC0_SHIFT             12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_LDOVCO_TRIM_OUTPUT_MASK    0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_LDOVCO_TRIM_OUTPUT_SHIFT            6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_2G4_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_MIN_ALLOWED_RTRIM_MASK     0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_MIN_ALLOWED_RTRIM_SHIFT            28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_RFC_MDM_DEMIQMC0_MASK      0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_RFC_MDM_DEMIQMC0_SHIFT             12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_LDOVCO_TRIM_OUTPUT_MASK    0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_LDOVCO_TRIM_OUTPUT_SHIFT            6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_2G4_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV2_CC26_1G
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_MIN_ALLOWED_RTRIM_MASK      0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_MIN_ALLOWED_RTRIM_SHIFT             28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_RFC_MDM_DEMIQMC0_MASK       0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_RFC_MDM_DEMIQMC0_SHIFT              12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_LDOVCO_TRIM_OUTPUT_MASK     0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_LDOVCO_TRIM_OUTPUT_SHIFT             6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC26_1G_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV2_CC13_1G
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_MIN_ALLOWED_RTRIM_MASK      0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_MIN_ALLOWED_RTRIM_SHIFT             28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_RFC_MDM_DEMIQMC0_MASK       0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_RFC_MDM_DEMIQMC0_SHIFT              12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_LDOVCO_TRIM_OUTPUT_MASK     0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_LDOVCO_TRIM_OUTPUT_SHIFT             6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV2_CC13_1G_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV4_CC26
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC26_MIN_ALLOWED_RTRIM_MASK         0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV4_CC26_MIN_ALLOWED_RTRIM_SHIFT                28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC26_RFC_MDM_DEMIQMC0_MASK          0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV4_CC26_RFC_MDM_DEMIQMC0_SHIFT                 12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC26_LDOVCO_TRIM_OUTPUT_MASK        0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV4_CC26_LDOVCO_TRIM_OUTPUT_SHIFT                6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV4_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV4_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV4_CC13
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC13_MIN_ALLOWED_RTRIM_MASK         0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV4_CC13_MIN_ALLOWED_RTRIM_SHIFT                28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC13_RFC_MDM_DEMIQMC0_MASK          0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV4_CC13_RFC_MDM_DEMIQMC0_SHIFT                 12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC13_LDOVCO_TRIM_OUTPUT_MASK        0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV4_CC13_LDOVCO_TRIM_OUTPUT_SHIFT                6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV4_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV4_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV4_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV5
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV5_MIN_ALLOWED_RTRIM_MASK              0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV5_MIN_ALLOWED_RTRIM_SHIFT                     28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV5_RFC_MDM_DEMIQMC0_MASK               0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV5_RFC_MDM_DEMIQMC0_SHIFT                      12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV5_LDOVCO_TRIM_OUTPUT_MASK             0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV5_LDOVCO_TRIM_OUTPUT_SHIFT                     6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV5_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV5_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV5_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV6_CC26
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC26_MIN_ALLOWED_RTRIM_MASK         0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV6_CC26_MIN_ALLOWED_RTRIM_SHIFT                28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC26_RFC_MDM_DEMIQMC0_MASK          0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV6_CC26_RFC_MDM_DEMIQMC0_SHIFT                 12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC26_LDOVCO_TRIM_OUTPUT_MASK        0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV6_CC26_LDOVCO_TRIM_OUTPUT_SHIFT                6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV6_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV6_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV6_CC13
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC13_MIN_ALLOWED_RTRIM_MASK         0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV6_CC13_MIN_ALLOWED_RTRIM_SHIFT                28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC13_RFC_MDM_DEMIQMC0_MASK          0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV6_CC13_RFC_MDM_DEMIQMC0_SHIFT                 12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC13_LDOVCO_TRIM_OUTPUT_MASK        0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV6_CC13_LDOVCO_TRIM_OUTPUT_SHIFT                6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV6_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV6_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV6_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV10
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV10_MIN_ALLOWED_RTRIM_MASK             0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV10_MIN_ALLOWED_RTRIM_SHIFT                    28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV10_RFC_MDM_DEMIQMC0_MASK              0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV10_RFC_MDM_DEMIQMC0_SHIFT                     12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV10_LDOVCO_TRIM_OUTPUT_MASK            0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV10_LDOVCO_TRIM_OUTPUT_SHIFT                    6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV10_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV10_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV10_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV12_CC26
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC26_MIN_ALLOWED_RTRIM_MASK        0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV12_CC26_MIN_ALLOWED_RTRIM_SHIFT               28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC26_RFC_MDM_DEMIQMC0_MASK         0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV12_CC26_RFC_MDM_DEMIQMC0_SHIFT                12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC26_LDOVCO_TRIM_OUTPUT_MASK       0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV12_CC26_LDOVCO_TRIM_OUTPUT_SHIFT               6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV12_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV12_CC26_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV12_CC13
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC13_MIN_ALLOWED_RTRIM_MASK        0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV12_CC13_MIN_ALLOWED_RTRIM_SHIFT               28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC13_RFC_MDM_DEMIQMC0_MASK         0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV12_CC13_RFC_MDM_DEMIQMC0_SHIFT                12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC13_LDOVCO_TRIM_OUTPUT_MASK       0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV12_CC13_LDOVCO_TRIM_OUTPUT_SHIFT               6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV12_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV12_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV12_CC13_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV15
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV15_MIN_ALLOWED_RTRIM_MASK             0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV15_MIN_ALLOWED_RTRIM_SHIFT                    28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV15_RFC_MDM_DEMIQMC0_MASK              0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV15_RFC_MDM_DEMIQMC0_SHIFT                     12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV15_LDOVCO_TRIM_OUTPUT_MASK            0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV15_LDOVCO_TRIM_OUTPUT_SHIFT                    6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV15_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV15_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV15_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_SYNTH_DIV30
 *
 ******************************************************************************
 * Field: [31:28] MIN_ALLOWED_RTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV30_MIN_ALLOWED_RTRIM_MASK             0xf0000000
#define FCFG1_CONFIG_SYNTH_DIV30_MIN_ALLOWED_RTRIM_SHIFT                    28

/* Field: [27:12] RFC_MDM_DEMIQMC0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV30_RFC_MDM_DEMIQMC0_MASK              0x0ffff000
#define FCFG1_CONFIG_SYNTH_DIV30_RFC_MDM_DEMIQMC0_SHIFT                     12

/* Field:  [11:6] LDOVCO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV30_LDOVCO_TRIM_OUTPUT_MASK            0x00000fc0
#define FCFG1_CONFIG_SYNTH_DIV30_LDOVCO_TRIM_OUTPUT_SHIFT                    6

/* Field:     [5] RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_SYNTH_DIV30_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N       0x00000020
#define FCFG1_CONFIG_SYNTH_DIV30_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_MASK  0x00000020
#define FCFG1_CONFIG_SYNTH_DIV30_RFC_MDM_DEMIQMC0_TRIMCOMPLETE_N_SHIFT 5

/******************************************************************************
 *
 * Register: FCFG1_FLASH_NUMBER
 *
 ******************************************************************************
 * Field:  [31:0] LOT_NUMBER
 *
 * Number of the manufacturing lot that produced this unit.
 */

#define FCFG1_FLASH_NUMBER_LOT_NUMBER_MASK                          0xffffffff
#define FCFG1_FLASH_NUMBER_LOT_NUMBER_SHIFT                                  0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_COORDINATE
 *
 ******************************************************************************
 * Field: [31:16] XCOORDINATE
 *
 * X coordinate of this unit on the wafer.
 */

#define FCFG1_FLASH_COORDINATE_XCOORDINATE_MASK                     0xffff0000
#define FCFG1_FLASH_COORDINATE_XCOORDINATE_SHIFT                            16

/* Field:  [15:0] YCOORDINATE
 *
 * Y coordinate of this unit on the wafer.
 */

#define FCFG1_FLASH_COORDINATE_YCOORDINATE_MASK                     0x0000ffff
#define FCFG1_FLASH_COORDINATE_YCOORDINATE_SHIFT                             0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_E_P
 *
 ******************************************************************************
 * Field: [31:24] PSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_E_P_PSU_MASK                                    0xff000000
#define FCFG1_FLASH_E_P_PSU_SHIFT                                           24

/* Field: [23:16] ESU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_E_P_ESU_MASK                                    0x00ff0000
#define FCFG1_FLASH_E_P_ESU_SHIFT                                           16

/* Field:  [15:8] PVSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_E_P_PVSU_MASK                                   0x0000ff00
#define FCFG1_FLASH_E_P_PVSU_SHIFT                                           8

/* Field:   [7:0] EVSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_E_P_EVSU_MASK                                   0x000000ff
#define FCFG1_FLASH_E_P_EVSU_SHIFT                                           0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_C_E_P_R
 *
 ******************************************************************************
 * Field: [31:24] RVSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_C_E_P_R_RVSU_MASK                               0xff000000
#define FCFG1_FLASH_C_E_P_R_RVSU_SHIFT                                      24

/* Field: [23:16] PV_ACCESS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_C_E_P_R_PV_ACCESS_MASK                          0x00ff0000
#define FCFG1_FLASH_C_E_P_R_PV_ACCESS_SHIFT                                 16

/* Field: [15:12] A_EXEZ_SETUP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_C_E_P_R_A_EXEZ_SETUP_MASK                       0x0000f000
#define FCFG1_FLASH_C_E_P_R_A_EXEZ_SETUP_SHIFT                              12

/* Field:  [11:0] CVSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_C_E_P_R_CVSU_MASK                               0x00000fff
#define FCFG1_FLASH_C_E_P_R_CVSU_SHIFT                                       0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_P_R_PV
 *
 ******************************************************************************
 * Field: [31:24] PH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_P_R_PV_PH_MASK                                  0xff000000
#define FCFG1_FLASH_P_R_PV_PH_SHIFT                                         24

/* Field: [23:16] RH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_P_R_PV_RH_MASK                                  0x00ff0000
#define FCFG1_FLASH_P_R_PV_RH_SHIFT                                         16

/* Field:  [15:8] PVH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_P_R_PV_PVH_MASK                                 0x0000ff00
#define FCFG1_FLASH_P_R_PV_PVH_SHIFT                                         8

/* Field:   [7:0] PVH2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_P_R_PV_PVH2_MASK                                0x000000ff
#define FCFG1_FLASH_P_R_PV_PVH2_SHIFT                                        0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_EH_SEQ
 *
 ******************************************************************************
 * Field: [31:24] EH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_EH_SEQ_EH_MASK                                  0xff000000
#define FCFG1_FLASH_EH_SEQ_EH_SHIFT                                         24

/* Field: [23:16] SEQ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_EH_SEQ_SEQ_MASK                                 0x00ff0000
#define FCFG1_FLASH_EH_SEQ_SEQ_SHIFT                                        16

/* Field: [15:12] VSTAT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_EH_SEQ_VSTAT_MASK                               0x0000f000
#define FCFG1_FLASH_EH_SEQ_VSTAT_SHIFT                                      12

/* Field:  [11:0] SM_FREQUENCY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_EH_SEQ_SM_FREQUENCY_MASK                        0x00000fff
#define FCFG1_FLASH_EH_SEQ_SM_FREQUENCY_SHIFT                                0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_VHV_E
 *
 ******************************************************************************
 * Field: [31:16] VHV_E_START
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_E_VHV_E_START_MASK                          0xffff0000
#define FCFG1_FLASH_VHV_E_VHV_E_START_SHIFT                                 16

/* Field:  [15:0] VHV_E_STEP_HIGHT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_E_VHV_E_STEP_HIGHT_MASK                     0x0000ffff
#define FCFG1_FLASH_VHV_E_VHV_E_STEP_HIGHT_SHIFT                             0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_PP
 *
 ******************************************************************************
 * Field: [31:24] PUMP_SU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_PP_PUMP_SU_MASK                                 0xff000000
#define FCFG1_FLASH_PP_PUMP_SU_SHIFT                                        24

/* Field: [23:16] TRIM3P4
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_PP_TRIM3P4_MASK                                 0x00ff0000
#define FCFG1_FLASH_PP_TRIM3P4_SHIFT                                        16

/* Field:  [15:0] MAX_PP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_PP_MAX_PP_MASK                                  0x0000ffff
#define FCFG1_FLASH_PP_MAX_PP_SHIFT                                          0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_PROG_EP
 *
 ******************************************************************************
 * Field: [31:16] MAX_EP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_PROG_EP_MAX_EP_MASK                             0xffff0000
#define FCFG1_FLASH_PROG_EP_MAX_EP_SHIFT                                    16

/* Field:  [15:0] PROGRAM_PW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_PROG_EP_PROGRAM_PW_MASK                         0x0000ffff
#define FCFG1_FLASH_PROG_EP_PROGRAM_PW_SHIFT                                 0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_ERA_PW
 *
 ******************************************************************************
 * Field:  [31:0] ERASE_PW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_ERA_PW_ERASE_PW_MASK                            0xffffffff
#define FCFG1_FLASH_ERA_PW_ERASE_PW_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_VHV
 *
 ******************************************************************************
 * Field: [27:24] TRIM13_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_TRIM13_P_MASK                               0x0f000000
#define FCFG1_FLASH_VHV_TRIM13_P_SHIFT                                      24

/* Field: [19:16] VHV_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_VHV_P_MASK                                  0x000f0000
#define FCFG1_FLASH_VHV_VHV_P_SHIFT                                         16

/* Field:  [11:8] TRIM13_E
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_TRIM13_E_MASK                               0x00000f00
#define FCFG1_FLASH_VHV_TRIM13_E_SHIFT                                       8

/* Field:   [3:0] VHV_E
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_VHV_E_MASK                                  0x0000000f
#define FCFG1_FLASH_VHV_VHV_E_SHIFT                                          0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_VHV_PV
 *
 ******************************************************************************
 * Field: [27:24] TRIM13_PV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_PV_TRIM13_PV_MASK                           0x0f000000
#define FCFG1_FLASH_VHV_PV_TRIM13_PV_SHIFT                                  24

/* Field: [19:16] VHV_PV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_PV_VHV_PV_MASK                              0x000f0000
#define FCFG1_FLASH_VHV_PV_VHV_PV_SHIFT                                     16

/* Field:  [15:8] VCG2P5
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_PV_VCG2P5_MASK                              0x0000ff00
#define FCFG1_FLASH_VHV_PV_VCG2P5_SHIFT                                      8

/* Field:   [7:0] VINH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_VHV_PV_VINH_MASK                                0x000000ff
#define FCFG1_FLASH_VHV_PV_VINH_SHIFT                                        0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_V
 *
 ******************************************************************************
 * Field: [31:24] VSL_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_V_VSL_P_MASK                                    0xff000000
#define FCFG1_FLASH_V_VSL_P_SHIFT                                           24

/* Field: [23:16] VWL_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_V_VWL_P_MASK                                    0x00ff0000
#define FCFG1_FLASH_V_VWL_P_SHIFT                                           16

/* Field:  [15:8] V_READ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_V_V_READ_MASK                                   0x0000ff00
#define FCFG1_FLASH_V_V_READ_SHIFT                                           8

/* Field:   [7:0] TRIM0P8
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_V_TRIM0P8_MASK                                  0x000000ff
#define FCFG1_FLASH_V_TRIM0P8_SHIFT                                          0

/******************************************************************************
 *
 * Register: FCFG1_USER_ID
 *
 ******************************************************************************
 * Field: [31:28] PG_REV
 *
 * Field used to distinguish revisions of the device
 */

#define FCFG1_USER_ID_PG_REV_MASK                                   0xf0000000
#define FCFG1_USER_ID_PG_REV_SHIFT                                          28

/* Field: [27:26] VER
 *
 * Version number.
 *
 * 0x0: Bits [25:12] of this register has the stated meaning.
 *
 * Any other setting indicate a different encoding of these bits.
 */

#define FCFG1_USER_ID_VER_MASK                                      0x0c000000
#define FCFG1_USER_ID_VER_SHIFT                                             26

/* Field:    [25] PA
 *
 * 0: Does not support 20dBm PA
 * 1: Supports 20dBM PA
 */

#define FCFG1_USER_ID_PA                                            0x02000000
#define FCFG1_USER_ID_PA_MASK                                       0x02000000
#define FCFG1_USER_ID_PA_SHIFT                                              25

/* Field:    [23] CC13
 *
 * 0: CC26xx device type
 * 1: CC13xx device type
 */

#define FCFG1_USER_ID_CC13                                          0x00800000
#define FCFG1_USER_ID_CC13_MASK                                     0x00800000
#define FCFG1_USER_ID_CC13_SHIFT                                            23

/* Field: [22:19] SEQUENCE
 *
 * Sequence.
 *
 * Used to differentiate between marketing/orderable product where other fields
 * of this register are the same (temp range, flash size, voltage range etc)
 */

#define FCFG1_USER_ID_SEQUENCE_MASK                                 0x00780000
#define FCFG1_USER_ID_SEQUENCE_SHIFT                                        19

/* Field: [18:16] PKG
 *
 * Package type.
 *
 * 0x0: 4x4mm QFN (RHB) package
 * 0x1: 5x5mm QFN (RSM) package
 * 0x2: 7x7mm QFN (RGZ) package
 * 0x3: Wafer sale package (naked die)
 * 0x4: WCSP (YFV)
 * 0x5: 7x7mm QFN package with Wettable Flanks
 *
 * Other values are reserved for future use.
 * Packages available for a specific device are shown in the device datasheet.
 */

#define FCFG1_USER_ID_PKG_MASK                                      0x00070000
#define FCFG1_USER_ID_PKG_SHIFT                                             16

/* Field: [15:12] PROTOCOL
 *
 * Protocols supported.
 *
 * 0x1: BLE
 * 0x2: RF4CE
 * 0x4: Zigbee/6lowpan
 * 0x8: Proprietary
 *
 * More than one protocol can be supported on same device - values above are
 * then combined.
 */

#define FCFG1_USER_ID_PROTOCOL_MASK                                 0x0000f000
#define FCFG1_USER_ID_PROTOCOL_SHIFT                                        12

/******************************************************************************
 *
 * Register: FCFG1_FLASH_OTP_DATA3
 *
 ******************************************************************************
 * Field: [31:23] EC_STEP_SIZE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA3_EC_STEP_SIZE_MASK                     0xff800000
#define FCFG1_FLASH_OTP_DATA3_EC_STEP_SIZE_SHIFT                            23

/* Field:    [22] DO_PRECOND
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA3_DO_PRECOND                            0x00400000
#define FCFG1_FLASH_OTP_DATA3_DO_PRECOND_MASK                       0x00400000
#define FCFG1_FLASH_OTP_DATA3_DO_PRECOND_SHIFT                              22

/* Field: [21:18] MAX_EC_LEVEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA3_MAX_EC_LEVEL_MASK                     0x003c0000
#define FCFG1_FLASH_OTP_DATA3_MAX_EC_LEVEL_SHIFT                            18

/* Field: [17:16] TRIM_1P7
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA3_TRIM_1P7_MASK                         0x00030000
#define FCFG1_FLASH_OTP_DATA3_TRIM_1P7_SHIFT                                16

/* Field:  [15:8] FLASH_SIZE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA3_FLASH_SIZE_MASK                       0x0000ff00
#define FCFG1_FLASH_OTP_DATA3_FLASH_SIZE_SHIFT                               8

/* Field:   [7:0] WAIT_SYSCODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA3_WAIT_SYSCODE_MASK                     0x000000ff
#define FCFG1_FLASH_OTP_DATA3_WAIT_SYSCODE_SHIFT                             0

/******************************************************************************
 *
 * Register: FCFG1_ANA2_TRIM
 *
 ******************************************************************************
 * Field:    [31] RCOSCHFCTRIMFRACT_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_RCOSCHFCTRIMFRACT_EN                        0x80000000
#define FCFG1_ANA2_TRIM_RCOSCHFCTRIMFRACT_EN_MASK                   0x80000000
#define FCFG1_ANA2_TRIM_RCOSCHFCTRIMFRACT_EN_SHIFT                          31

/* Field: [30:26] RCOSCHFCTRIMFRACT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_RCOSCHFCTRIMFRACT_MASK                      0x7c000000
#define FCFG1_ANA2_TRIM_RCOSCHFCTRIMFRACT_SHIFT                             26

/* Field: [24:23] SET_RCOSC_HF_FINE_RESISTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_SET_RCOSC_HF_FINE_RESISTOR_MASK             0x01800000
#define FCFG1_ANA2_TRIM_SET_RCOSC_HF_FINE_RESISTOR_SHIFT                    23

/* Field:    [22] ATESTLF_UDIGLDO_IBIAS_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_ATESTLF_UDIGLDO_IBIAS_TRIM                  0x00400000
#define FCFG1_ANA2_TRIM_ATESTLF_UDIGLDO_IBIAS_TRIM_MASK             0x00400000
#define FCFG1_ANA2_TRIM_ATESTLF_UDIGLDO_IBIAS_TRIM_SHIFT                    22

/* Field: [21:15] NANOAMP_RES_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_NANOAMP_RES_TRIM_MASK                       0x003f8000
#define FCFG1_ANA2_TRIM_NANOAMP_RES_TRIM_SHIFT                              15

/* Field:    [11] DITHER_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_DITHER_EN                                   0x00000800
#define FCFG1_ANA2_TRIM_DITHER_EN_MASK                              0x00000800
#define FCFG1_ANA2_TRIM_DITHER_EN_SHIFT                                     11

/* Field:  [10:8] DCDC_IPEAK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_DCDC_IPEAK_MASK                             0x00000700
#define FCFG1_ANA2_TRIM_DCDC_IPEAK_SHIFT                                     8

/* Field:   [7:6] DEAD_TIME_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_DEAD_TIME_TRIM_MASK                         0x000000c0
#define FCFG1_ANA2_TRIM_DEAD_TIME_TRIM_SHIFT                                 6

/* Field:   [5:3] DCDC_LOW_EN_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_DCDC_LOW_EN_SEL_MASK                        0x00000038
#define FCFG1_ANA2_TRIM_DCDC_LOW_EN_SEL_SHIFT                                3

/* Field:   [2:0] DCDC_HIGH_EN_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANA2_TRIM_DCDC_HIGH_EN_SEL_MASK                       0x00000007
#define FCFG1_ANA2_TRIM_DCDC_HIGH_EN_SEL_SHIFT                               0

/******************************************************************************
 *
 * Register: FCFG1_LDO_TRIM
 *
 ******************************************************************************
 * Field: [28:24] VDDR_TRIM_SLEEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_MASK                         0x1f000000
#define FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_SHIFT                                24

/* Field: [18:16] GLDO_CURSRC
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_LDO_TRIM_GLDO_CURSRC_MASK                             0x00070000
#define FCFG1_LDO_TRIM_GLDO_CURSRC_SHIFT                                    16

/* Field: [12:11] ITRIM_DIGLDO_LOAD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_LDO_TRIM_ITRIM_DIGLDO_LOAD_MASK                       0x00001800
#define FCFG1_LDO_TRIM_ITRIM_DIGLDO_LOAD_SHIFT                              11

/* Field:  [10:8] ITRIM_UDIGLDO
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_LDO_TRIM_ITRIM_UDIGLDO_MASK                           0x00000700
#define FCFG1_LDO_TRIM_ITRIM_UDIGLDO_SHIFT                                   8

/* Field:   [2:0] VTRIM_DELTA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_LDO_TRIM_VTRIM_DELTA_MASK                             0x00000007
#define FCFG1_LDO_TRIM_VTRIM_DELTA_SHIFT                                     0

/******************************************************************************
 *
 * Register: FCFG1_MAC_BLE_0
 *
 ******************************************************************************
 * Field:  [31:0] ADDR_0_31
 *
 * The first 32-bits of the 64-bit MAC BLE address
 */

#define FCFG1_MAC_BLE_0_ADDR_0_31_MASK                              0xffffffff
#define FCFG1_MAC_BLE_0_ADDR_0_31_SHIFT                                      0

/******************************************************************************
 *
 * Register: FCFG1_MAC_BLE_1
 *
 ******************************************************************************
 * Field:  [31:0] ADDR_32_63
 *
 * The last 32-bits of the 64-bit MAC BLE address
 */

#define FCFG1_MAC_BLE_1_ADDR_32_63_MASK                             0xffffffff
#define FCFG1_MAC_BLE_1_ADDR_32_63_SHIFT                                     0

/******************************************************************************
 *
 * Register: FCFG1_MAC_15_4_0
 *
 ******************************************************************************
 * Field:  [31:0] ADDR_0_31
 *
 * The first 32-bits of the 64-bit MAC 15.4 address
 */

#define FCFG1_MAC_15_4_0_ADDR_0_31_MASK                             0xffffffff
#define FCFG1_MAC_15_4_0_ADDR_0_31_SHIFT                                     0

/******************************************************************************
 *
 * Register: FCFG1_MAC_15_4_1
 *
 ******************************************************************************
 * Field:  [31:0] ADDR_32_63
 *
 * The last 32-bits of the 64-bit MAC 15.4 address
 */

#define FCFG1_MAC_15_4_1_ADDR_32_63_MASK                            0xffffffff
#define FCFG1_MAC_15_4_1_ADDR_32_63_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_FLASH_OTP_DATA4
 *
 ******************************************************************************
 * Field:    [31] STANDBY_MODE_SEL_INT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT              0x80000000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT_MASK         0x80000000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_WRT_SHIFT                31

/* Field: [30:29] STANDBY_PW_SEL_INT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_WRT_MASK           0x60000000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_WRT_SHIFT                  29

/* Field:    [28] DIS_STANDBY_INT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_WRT                   0x10000000
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_WRT_MASK              0x10000000
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_WRT_SHIFT                     28

/* Field:    [27] DIS_IDLE_INT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT                      0x08000000
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT_MASK                 0x08000000
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_WRT_SHIFT                        27

/* Field: [26:24] VIN_AT_X_INT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_WRT_MASK                 0x07000000
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_WRT_SHIFT                        24

/* Field:    [23] STANDBY_MODE_SEL_EXT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT              0x00800000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT_MASK         0x00800000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_WRT_SHIFT                23

/* Field: [22:21] STANDBY_PW_SEL_EXT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_WRT_MASK           0x00600000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_WRT_SHIFT                  21

/* Field:    [20] DIS_STANDBY_EXT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_WRT                   0x00100000
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_WRT_MASK              0x00100000
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_WRT_SHIFT                     20

/* Field:    [19] DIS_IDLE_EXT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT                      0x00080000
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT_MASK                 0x00080000
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_WRT_SHIFT                        19

/* Field: [18:16] VIN_AT_X_EXT_WRT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_WRT_MASK                 0x00070000
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_WRT_SHIFT                        16

/* Field:    [15] STANDBY_MODE_SEL_INT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD               0x00008000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD_MASK          0x00008000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_INT_RD_SHIFT                 15

/* Field: [14:13] STANDBY_PW_SEL_INT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_RD_MASK            0x00006000
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_INT_RD_SHIFT                   13

/* Field:    [12] DIS_STANDBY_INT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_RD                    0x00001000
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_RD_MASK               0x00001000
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_INT_RD_SHIFT                      12

/* Field:    [11] DIS_IDLE_INT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD                       0x00000800
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD_MASK                  0x00000800
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_INT_RD_SHIFT                         11

/* Field:  [10:8] VIN_AT_X_INT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_RD_MASK                  0x00000700
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_INT_RD_SHIFT                          8

/* Field:     [7] STANDBY_MODE_SEL_EXT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD               0x00000080
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD_MASK          0x00000080
#define FCFG1_FLASH_OTP_DATA4_STANDBY_MODE_SEL_EXT_RD_SHIFT                  7

/* Field:   [6:5] STANDBY_PW_SEL_EXT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_RD_MASK            0x00000060
#define FCFG1_FLASH_OTP_DATA4_STANDBY_PW_SEL_EXT_RD_SHIFT                    5

/* Field:     [4] DIS_STANDBY_EXT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_RD                    0x00000010
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_RD_MASK               0x00000010
#define FCFG1_FLASH_OTP_DATA4_DIS_STANDBY_EXT_RD_SHIFT                       4

/* Field:     [3] DIS_IDLE_EXT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD                       0x00000008
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD_MASK                  0x00000008
#define FCFG1_FLASH_OTP_DATA4_DIS_IDLE_EXT_RD_SHIFT                          3

/* Field:   [2:0] VIN_AT_X_EXT_RD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_RD_MASK                  0x00000007
#define FCFG1_FLASH_OTP_DATA4_VIN_AT_X_EXT_RD_SHIFT                          0

/******************************************************************************
 *
 * Register: FCFG1_MISC_TRIM
 *
 ******************************************************************************
 * Field: [16:12] TRIM_RECHARGE_COMP_OFFSET
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_TRIM_TRIM_RECHARGE_COMP_OFFSET_MASK              0x0001f000
#define FCFG1_MISC_TRIM_TRIM_RECHARGE_COMP_OFFSET_SHIFT                     12

/* Field:  [11:8] TRIM_RECHARGE_COMP_REFLEVEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_TRIM_TRIM_RECHARGE_COMP_REFLEVEL_MASK            0x00000f00
#define FCFG1_MISC_TRIM_TRIM_RECHARGE_COMP_REFLEVEL_SHIFT                    8

/* Field:   [7:0] TEMPVSLOPE
 *
 * Signed byte value representing the TEMP slope with battery voltage, in
 * degrees C / V, with four fractional bits.
 */

#define FCFG1_MISC_TRIM_TEMPVSLOPE_MASK                             0x000000ff
#define FCFG1_MISC_TRIM_TEMPVSLOPE_SHIFT                                     0

/******************************************************************************
 *
 * Register: FCFG1_RCOSC_HF_TEMPCOMP
 *
 ******************************************************************************
 * Field: [31:24] FINE_RESISTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_RCOSC_HF_TEMPCOMP_FINE_RESISTOR_MASK                  0xff000000
#define FCFG1_RCOSC_HF_TEMPCOMP_FINE_RESISTOR_SHIFT                         24

/* Field: [23:16] CTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_RCOSC_HF_TEMPCOMP_CTRIM_MASK                          0x00ff0000
#define FCFG1_RCOSC_HF_TEMPCOMP_CTRIM_SHIFT                                 16

/* Field:  [15:8] CTRIMFRACT_QUAD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_RCOSC_HF_TEMPCOMP_CTRIMFRACT_QUAD_MASK                0x0000ff00
#define FCFG1_RCOSC_HF_TEMPCOMP_CTRIMFRACT_QUAD_SHIFT                        8

/* Field:   [7:0] CTRIMFRACT_SLOPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_RCOSC_HF_TEMPCOMP_CTRIMFRACT_SLOPE_MASK               0x000000ff
#define FCFG1_RCOSC_HF_TEMPCOMP_CTRIMFRACT_SLOPE_SHIFT                       0

/******************************************************************************
 *
 * Register: FCFG1_ICEPICK_DEVICE_ID
 *
 ******************************************************************************
 * Field: [31:28] PG_REV
 *
 * Field used to distinguish revisions of the device.
 */

#define FCFG1_ICEPICK_DEVICE_ID_PG_REV_MASK                         0xf0000000
#define FCFG1_ICEPICK_DEVICE_ID_PG_REV_SHIFT                                28

/* Field: [27:12] WAFER_ID
 *
 * Field used to identify silicon die.
 */

#define FCFG1_ICEPICK_DEVICE_ID_WAFER_ID_MASK                       0x0ffff000
#define FCFG1_ICEPICK_DEVICE_ID_WAFER_ID_SHIFT                              12

/* Field:  [11:0] MANUFACTURER_ID
 *
 * Manufacturer code.
 *
 * 0x02f: Texas Instruments
 */

#define FCFG1_ICEPICK_DEVICE_ID_MANUFACTURER_ID_MASK                0x00000fff
#define FCFG1_ICEPICK_DEVICE_ID_MANUFACTURER_ID_SHIFT                        0

/******************************************************************************
 *
 * Register: FCFG1_FCFG1_REVISION
 *
 ******************************************************************************
 * Field:  [31:0] REV
 *
 * The revision number of the FCFG1 layout. This value will be read by
 * application SW in order to determine which FCFG1 parameters that have valid
 * values. This revision number must be incremented by 1 before any devices are
 * to be produced if the FCFG1 layout has changed since the previous production
 * of devices.
 * Value migth change without warning.
 */

#define FCFG1_FCFG1_REVISION_REV_MASK                               0xffffffff
#define FCFG1_FCFG1_REVISION_REV_SHIFT                                       0

/******************************************************************************
 *
 * Register: FCFG1_MISC_OTP_DATA
 *
 ******************************************************************************
 * Field: [31:28] RCOSC_HF_ITUNE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_RCOSC_HF_ITUNE_MASK                     0xf0000000
#define FCFG1_MISC_OTP_DATA_RCOSC_HF_ITUNE_SHIFT                            28

/* Field: [27:20] RCOSC_HF_CRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_RCOSC_HF_CRIM_MASK                      0x0ff00000
#define FCFG1_MISC_OTP_DATA_RCOSC_HF_CRIM_SHIFT                             20

/* Field: [19:15] PER_M
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_PER_M_MASK                              0x000f8000
#define FCFG1_MISC_OTP_DATA_PER_M_SHIFT                                     15

/* Field: [14:12] PER_E
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_PER_E_MASK                              0x00007000
#define FCFG1_MISC_OTP_DATA_PER_E_SHIFT                                     12

/* Field:   [7:0] TEST_PROGRAM_REV
 *
 * The revision of the test program used in the production process when FCFG1
 * was programmed.
 * Value migth change without warning.
 */

#define FCFG1_MISC_OTP_DATA_TEST_PROGRAM_REV_MASK                   0x000000ff
#define FCFG1_MISC_OTP_DATA_TEST_PROGRAM_REV_SHIFT                           0

/******************************************************************************
 *
 * Register: FCFG1_IOCONF
 *
 ******************************************************************************
 * Field:   [6:0] GPIO_CNT
 *
 * Number of available DIOs.
 */

#define FCFG1_IOCONF_GPIO_CNT_MASK                                  0x0000007f
#define FCFG1_IOCONF_GPIO_CNT_SHIFT                                          0

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_IF_ADC
 *
 ******************************************************************************
 * Field: [31:28] FF2ADJ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_FF2ADJ_MASK                             0xf0000000
#define FCFG1_CONFIG_IF_ADC_FF2ADJ_SHIFT                                    28

/* Field: [27:24] FF3ADJ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_FF3ADJ_MASK                             0x0f000000
#define FCFG1_CONFIG_IF_ADC_FF3ADJ_SHIFT                                    24

/* Field: [23:20] INT3ADJ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_INT3ADJ_MASK                            0x00f00000
#define FCFG1_CONFIG_IF_ADC_INT3ADJ_SHIFT                                   20

/* Field: [19:16] FF1ADJ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_FF1ADJ_MASK                             0x000f0000
#define FCFG1_CONFIG_IF_ADC_FF1ADJ_SHIFT                                    16

/* Field: [15:14] AAFCAP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_AAFCAP_MASK                             0x0000c000
#define FCFG1_CONFIG_IF_ADC_AAFCAP_SHIFT                                    14

/* Field: [13:10] INT2ADJ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_INT2ADJ_MASK                            0x00003c00
#define FCFG1_CONFIG_IF_ADC_INT2ADJ_SHIFT                                   10

/* Field:   [9:5] IFDIGLDO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_IFDIGLDO_TRIM_OUTPUT_MASK               0x000003e0
#define FCFG1_CONFIG_IF_ADC_IFDIGLDO_TRIM_OUTPUT_SHIFT                       5

/* Field:   [4:0] IFANALDO_TRIM_OUTPUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_IF_ADC_IFANALDO_TRIM_OUTPUT_MASK               0x0000001f
#define FCFG1_CONFIG_IF_ADC_IFANALDO_TRIM_OUTPUT_SHIFT                       0

/******************************************************************************
 *
 * Register: FCFG1_CONFIG_OSC_TOP
 *
 ******************************************************************************
 * Field: [29:26] XOSC_HF_ROW_Q12
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_MASK                   0x3c000000
#define FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_SHIFT                          26

/* Field: [25:10] XOSC_HF_COLUMN_Q12
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_MASK                0x03fffc00
#define FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_SHIFT                       10

/* Field:   [9:2] RCOSCLF_CTUNE_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_MASK                0x000003fc
#define FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_SHIFT                        2

/* Field:   [1:0] RCOSCLF_RTUNE_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_MASK                0x00000003
#define FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_SHIFT                        0

/******************************************************************************
 *
 * Register: FCFG1_SOC_ADC_ABS_GAIN
 *
 ******************************************************************************
 * Field:  [15:0] SOC_ADC_ABS_GAIN_TEMP1
 *
 * SOC_ADC gain in absolute reference mode at temperature 1 (30C). Calculated
 * in production test..
 */

#define FCFG1_SOC_ADC_ABS_GAIN_SOC_ADC_ABS_GAIN_TEMP1_MASK          0x0000ffff
#define FCFG1_SOC_ADC_ABS_GAIN_SOC_ADC_ABS_GAIN_TEMP1_SHIFT                  0

/******************************************************************************
 *
 * Register: FCFG1_SOC_ADC_REL_GAIN
 *
 ******************************************************************************
 * Field:  [15:0] SOC_ADC_REL_GAIN_TEMP1
 *
 * SOC_ADC gain in relative reference mode at temperature 1 (30C). Calculated
 * in production test..
 */

#define FCFG1_SOC_ADC_REL_GAIN_SOC_ADC_REL_GAIN_TEMP1_MASK          0x0000ffff
#define FCFG1_SOC_ADC_REL_GAIN_SOC_ADC_REL_GAIN_TEMP1_SHIFT                  0

/******************************************************************************
 *
 * Register: FCFG1_SOC_ADC_OFFSET_INT
 *
 ******************************************************************************
 * Field: [23:16] SOC_ADC_REL_OFFSET_TEMP1
 *
 * SOC_ADC offset in relative reference mode at temperature 1 (30C). Signed
 * 8-bit number. Calculated in production test..
 */

#define FCFG1_SOC_ADC_OFFSET_INT_SOC_ADC_REL_OFFSET_TEMP1_MASK      0x00ff0000
#define FCFG1_SOC_ADC_OFFSET_INT_SOC_ADC_REL_OFFSET_TEMP1_SHIFT             16

/* Field:   [7:0] SOC_ADC_ABS_OFFSET_TEMP1
 *
 * SOC_ADC offset in absolute reference mode at temperature 1 (30C). Signed
 * 8-bit number. Calculated in production test..
 */

#define FCFG1_SOC_ADC_OFFSET_INT_SOC_ADC_ABS_OFFSET_TEMP1_MASK      0x000000ff
#define FCFG1_SOC_ADC_OFFSET_INT_SOC_ADC_ABS_OFFSET_TEMP1_SHIFT              0

/******************************************************************************
 *
 * Register: FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_OFFSET
 *
 ******************************************************************************
 * Field:   [5:0] SOC_ADC_REF_VOLTAGE_TRIM_TEMP1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_SOC_ADC_REF_VOLTAGE_TRIM_TEMP1_MASK  0x0000003f
#define FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_SOC_ADC_REF_VOLTAGE_TRIM_TEMP1_SHIFT 0

/******************************************************************************
 *
 * Register: FCFG1_AMPCOMP_TH1
 *
 ******************************************************************************
 * Field: [23:18] HPMRAMP3_LTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_MASK                         0x00fc0000
#define FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_SHIFT                                18

/* Field: [15:10] HPMRAMP3_HTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_MASK                         0x0000fc00
#define FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_SHIFT                                10

/* Field:   [9:6] IBIASCAP_LPTOHP_OL_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_MASK               0x000003c0
#define FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT                       6

/* Field:   [5:0] HPMRAMP1_TH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_MASK                          0x0000003f
#define FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_SHIFT                                  0

/******************************************************************************
 *
 * Register: FCFG1_AMPCOMP_TH2
 *
 ******************************************************************************
 * Field: [31:26] LPMUPDATE_LTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_MASK                        0xfc000000
#define FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_SHIFT                               26

/* Field: [23:18] LPMUPDATE_HTM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_MASK                        0x00fc0000
#define FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_SHIFT                               18

/* Field: [15:10] ADC_COMP_AMPTH_LPM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_MASK                   0x0000fc00
#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_SHIFT                          10

/* Field:   [7:2] ADC_COMP_AMPTH_HPM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_MASK                   0x000000fc
#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_SHIFT                           2

/******************************************************************************
 *
 * Register: FCFG1_AMPCOMP_CTRL1
 *
 ******************************************************************************
 * Field:    [30] AMPCOMP_REQ_MODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE                        0x40000000
#define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_MASK                   0x40000000
#define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_SHIFT                          30

/* Field: [23:20] IBIAS_OFFSET
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_MASK                       0x00f00000
#define FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_SHIFT                              20

/* Field: [19:16] IBIAS_INIT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_MASK                         0x000f0000
#define FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_SHIFT                                16

/* Field:  [15:8] LPM_IBIAS_WAIT_CNT_FINAL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_MASK           0x0000ff00
#define FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT                   8

/* Field:   [7:4] CAP_STEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_CTRL1_CAP_STEP_MASK                           0x000000f0
#define FCFG1_AMPCOMP_CTRL1_CAP_STEP_SHIFT                                   4

/* Field:   [3:0] IBIASCAP_HPTOLP_OL_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_MASK             0x0000000f
#define FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_SHIFT                     0

/******************************************************************************
 *
 * Register: FCFG1_ANABYPASS_VALUE2
 *
 ******************************************************************************
 * Field:  [13:0] XOSC_HF_IBIASTHERM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_MASK              0x00003fff
#define FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_SHIFT                      0

/******************************************************************************
 *
 * Register: FCFG1_VOLT_TRIM
 *
 ******************************************************************************
 * Field: [28:24] VDDR_TRIM_HH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_VOLT_TRIM_VDDR_TRIM_HH_MASK                           0x1f000000
#define FCFG1_VOLT_TRIM_VDDR_TRIM_HH_SHIFT                                  24

/* Field: [20:16] VDDR_TRIM_H
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_VOLT_TRIM_VDDR_TRIM_H_MASK                            0x001f0000
#define FCFG1_VOLT_TRIM_VDDR_TRIM_H_SHIFT                                   16

/* Field:  [12:8] VDDR_TRIM_SLEEP_H
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_VOLT_TRIM_VDDR_TRIM_SLEEP_H_MASK                      0x00001f00
#define FCFG1_VOLT_TRIM_VDDR_TRIM_SLEEP_H_SHIFT                              8

/* Field:   [4:0] TRIMBOD_H
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_VOLT_TRIM_TRIMBOD_H_MASK                              0x0000001f
#define FCFG1_VOLT_TRIM_TRIMBOD_H_SHIFT                                      0

/******************************************************************************
 *
 * Register: FCFG1_OSC_CONF
 *
 ******************************************************************************
 * Field:    [29] ADC_SH_VBUF_EN
 *
 * Trim value for DDI_0_OSC:ADCDOUBLERNANOAMPCTL.ADC_SH_VBUF_EN.
 */

#define FCFG1_OSC_CONF_ADC_SH_VBUF_EN                               0x20000000
#define FCFG1_OSC_CONF_ADC_SH_VBUF_EN_MASK                          0x20000000
#define FCFG1_OSC_CONF_ADC_SH_VBUF_EN_SHIFT                                 29

/* Field:    [28] ADC_SH_MODE_EN
 *
 * Trim value for DDI_0_OSC:ADCDOUBLERNANOAMPCTL.ADC_SH_MODE_EN.
 */

#define FCFG1_OSC_CONF_ADC_SH_MODE_EN                               0x10000000
#define FCFG1_OSC_CONF_ADC_SH_MODE_EN_MASK                          0x10000000
#define FCFG1_OSC_CONF_ADC_SH_MODE_EN_SHIFT                                 28

/* Field:    [27] ATESTLF_RCOSCLF_IBIAS_TRIM
 *
 * Trim value for DDI_0_OSC:ATESTCTL.ATESTLF_RCOSCLF_IBIAS_TRIM.
 */

#define FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM                   0x08000000
#define FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM_MASK              0x08000000
#define FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM_SHIFT                     27

/* Field: [26:25] XOSCLF_REGULATOR_TRIM
 *
 * Trim value for DDI_0_OSC:LFOSCCTL.XOSCLF_REGULATOR_TRIM.
 */

#define FCFG1_OSC_CONF_XOSCLF_REGULATOR_TRIM_MASK                   0x06000000
#define FCFG1_OSC_CONF_XOSCLF_REGULATOR_TRIM_SHIFT                          25

/* Field: [24:21] XOSCLF_CMIRRWR_RATIO
 *
 * Trim value for DDI_0_OSC:LFOSCCTL.XOSCLF_CMIRRWR_RATIO.
 */

#define FCFG1_OSC_CONF_XOSCLF_CMIRRWR_RATIO_MASK                    0x01e00000
#define FCFG1_OSC_CONF_XOSCLF_CMIRRWR_RATIO_SHIFT                           21

/* Field: [20:19] XOSC_HF_FAST_START
 *
 * Trim value for DDI_0_OSC:CTL1.XOSC_HF_FAST_START.
 */

#define FCFG1_OSC_CONF_XOSC_HF_FAST_START_MASK                      0x00180000
#define FCFG1_OSC_CONF_XOSC_HF_FAST_START_SHIFT                             19

/* Field:    [18] XOSC_OPTION
 *
 * 0: XOSC_HF unavailable (may not be bonded out)
 * 1: XOSC_HF available (default)
 */

#define FCFG1_OSC_CONF_XOSC_OPTION                                  0x00040000
#define FCFG1_OSC_CONF_XOSC_OPTION_MASK                             0x00040000
#define FCFG1_OSC_CONF_XOSC_OPTION_SHIFT                                    18

/* Field:    [17] HPOSC_OPTION
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_OPTION                                 0x00020000
#define FCFG1_OSC_CONF_HPOSC_OPTION_MASK                            0x00020000
#define FCFG1_OSC_CONF_HPOSC_OPTION_SHIFT                                   17

/* Field:    [16] HPOSC_BIAS_HOLD_MODE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_BIAS_HOLD_MODE_EN                      0x00010000
#define FCFG1_OSC_CONF_HPOSC_BIAS_HOLD_MODE_EN_MASK                 0x00010000
#define FCFG1_OSC_CONF_HPOSC_BIAS_HOLD_MODE_EN_SHIFT                        16

/* Field: [15:12] HPOSC_CURRMIRR_RATIO
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_CURRMIRR_RATIO_MASK                    0x0000f000
#define FCFG1_OSC_CONF_HPOSC_CURRMIRR_RATIO_SHIFT                           12

/* Field:  [11:8] HPOSC_BIAS_RES_SET
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_BIAS_RES_SET_MASK                      0x00000f00
#define FCFG1_OSC_CONF_HPOSC_BIAS_RES_SET_SHIFT                              8

/* Field:     [7] HPOSC_FILTER_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_FILTER_EN                              0x00000080
#define FCFG1_OSC_CONF_HPOSC_FILTER_EN_MASK                         0x00000080
#define FCFG1_OSC_CONF_HPOSC_FILTER_EN_SHIFT                                 7

/* Field:   [6:5] HPOSC_BIAS_RECHARGE_DELAY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_BIAS_RECHARGE_DELAY_MASK               0x00000060
#define FCFG1_OSC_CONF_HPOSC_BIAS_RECHARGE_DELAY_SHIFT                       5

/* Field:   [2:1] HPOSC_SERIES_CAP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_SERIES_CAP_MASK                        0x00000006
#define FCFG1_OSC_CONF_HPOSC_SERIES_CAP_SHIFT                                1

/* Field:     [0] HPOSC_DIV3_BYPASS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_OSC_CONF_HPOSC_DIV3_BYPASS                            0x00000001
#define FCFG1_OSC_CONF_HPOSC_DIV3_BYPASS_MASK                       0x00000001
#define FCFG1_OSC_CONF_HPOSC_DIV3_BYPASS_SHIFT                               0

/******************************************************************************
 *
 * Register: FCFG1_FREQ_OFFSET
 *
 ******************************************************************************
 * Field: [31:16] HPOSC_COMP_P0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FREQ_OFFSET_HPOSC_COMP_P0_MASK                        0xffff0000
#define FCFG1_FREQ_OFFSET_HPOSC_COMP_P0_SHIFT                               16

/* Field:  [15:8] HPOSC_COMP_P1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FREQ_OFFSET_HPOSC_COMP_P1_MASK                        0x0000ff00
#define FCFG1_FREQ_OFFSET_HPOSC_COMP_P1_SHIFT                                8

/* Field:   [7:0] HPOSC_COMP_P2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_FREQ_OFFSET_HPOSC_COMP_P2_MASK                        0x000000ff
#define FCFG1_FREQ_OFFSET_HPOSC_COMP_P2_SHIFT                                0

/******************************************************************************
 *
 * Register: FCFG1_MISC_OTP_DATA_1
 *
 ******************************************************************************
 * Field: [28:27] PEAK_DET_ITRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_PEAK_DET_ITRIM_MASK                   0x18000000
#define FCFG1_MISC_OTP_DATA_1_PEAK_DET_ITRIM_SHIFT                          27

/* Field: [26:24] HP_BUF_ITRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_HP_BUF_ITRIM_MASK                     0x07000000
#define FCFG1_MISC_OTP_DATA_1_HP_BUF_ITRIM_SHIFT                            24

/* Field: [23:22] LP_BUF_ITRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_LP_BUF_ITRIM_MASK                     0x00c00000
#define FCFG1_MISC_OTP_DATA_1_LP_BUF_ITRIM_SHIFT                            22

/* Field: [21:20] DBLR_LOOP_FILTER_RESET_VOLTAGE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_DBLR_LOOP_FILTER_RESET_VOLTAGE_MASK   0x00300000
#define FCFG1_MISC_OTP_DATA_1_DBLR_LOOP_FILTER_RESET_VOLTAGE_SHIFT          20

/* Field: [19:10] HPM_IBIAS_WAIT_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_HPM_IBIAS_WAIT_CNT_MASK               0x000ffc00
#define FCFG1_MISC_OTP_DATA_1_HPM_IBIAS_WAIT_CNT_SHIFT                      10

/* Field:   [9:4] LPM_IBIAS_WAIT_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_LPM_IBIAS_WAIT_CNT_MASK               0x000003f0
#define FCFG1_MISC_OTP_DATA_1_LPM_IBIAS_WAIT_CNT_SHIFT                       4

/* Field:   [3:0] IDAC_STEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_MISC_OTP_DATA_1_IDAC_STEP_MASK                        0x0000000f
#define FCFG1_MISC_OTP_DATA_1_IDAC_STEP_SHIFT                                0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_20C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_20C_DELTA_CACHE_REF_MASK                     0xff000000
#define FCFG1_PWD_CURR_20C_DELTA_CACHE_REF_SHIFT                            24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_20C_DELTA_RFMEM_RET_MASK                     0x00ff0000
#define FCFG1_PWD_CURR_20C_DELTA_RFMEM_RET_SHIFT                            16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_20C_DELTA_XOSC_LPM_MASK                      0x0000ff00
#define FCFG1_PWD_CURR_20C_DELTA_XOSC_LPM_SHIFT                              8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_20C_BASELINE_MASK                            0x000000ff
#define FCFG1_PWD_CURR_20C_BASELINE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_35C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_35C_DELTA_CACHE_REF_MASK                     0xff000000
#define FCFG1_PWD_CURR_35C_DELTA_CACHE_REF_SHIFT                            24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_35C_DELTA_RFMEM_RET_MASK                     0x00ff0000
#define FCFG1_PWD_CURR_35C_DELTA_RFMEM_RET_SHIFT                            16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_35C_DELTA_XOSC_LPM_MASK                      0x0000ff00
#define FCFG1_PWD_CURR_35C_DELTA_XOSC_LPM_SHIFT                              8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_35C_BASELINE_MASK                            0x000000ff
#define FCFG1_PWD_CURR_35C_BASELINE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_50C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_50C_DELTA_CACHE_REF_MASK                     0xff000000
#define FCFG1_PWD_CURR_50C_DELTA_CACHE_REF_SHIFT                            24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_50C_DELTA_RFMEM_RET_MASK                     0x00ff0000
#define FCFG1_PWD_CURR_50C_DELTA_RFMEM_RET_SHIFT                            16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_50C_DELTA_XOSC_LPM_MASK                      0x0000ff00
#define FCFG1_PWD_CURR_50C_DELTA_XOSC_LPM_SHIFT                              8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_50C_BASELINE_MASK                            0x000000ff
#define FCFG1_PWD_CURR_50C_BASELINE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_65C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_65C_DELTA_CACHE_REF_MASK                     0xff000000
#define FCFG1_PWD_CURR_65C_DELTA_CACHE_REF_SHIFT                            24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_65C_DELTA_RFMEM_RET_MASK                     0x00ff0000
#define FCFG1_PWD_CURR_65C_DELTA_RFMEM_RET_SHIFT                            16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_65C_DELTA_XOSC_LPM_MASK                      0x0000ff00
#define FCFG1_PWD_CURR_65C_DELTA_XOSC_LPM_SHIFT                              8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_65C_BASELINE_MASK                            0x000000ff
#define FCFG1_PWD_CURR_65C_BASELINE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_80C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_80C_DELTA_CACHE_REF_MASK                     0xff000000
#define FCFG1_PWD_CURR_80C_DELTA_CACHE_REF_SHIFT                            24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_80C_DELTA_RFMEM_RET_MASK                     0x00ff0000
#define FCFG1_PWD_CURR_80C_DELTA_RFMEM_RET_SHIFT                            16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_80C_DELTA_XOSC_LPM_MASK                      0x0000ff00
#define FCFG1_PWD_CURR_80C_DELTA_XOSC_LPM_SHIFT                              8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_80C_BASELINE_MASK                            0x000000ff
#define FCFG1_PWD_CURR_80C_BASELINE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_95C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_95C_DELTA_CACHE_REF_MASK                     0xff000000
#define FCFG1_PWD_CURR_95C_DELTA_CACHE_REF_SHIFT                            24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_95C_DELTA_RFMEM_RET_MASK                     0x00ff0000
#define FCFG1_PWD_CURR_95C_DELTA_RFMEM_RET_SHIFT                            16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_95C_DELTA_XOSC_LPM_MASK                      0x0000ff00
#define FCFG1_PWD_CURR_95C_DELTA_XOSC_LPM_SHIFT                              8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_95C_BASELINE_MASK                            0x000000ff
#define FCFG1_PWD_CURR_95C_BASELINE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_110C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_110C_DELTA_CACHE_REF_MASK                    0xff000000
#define FCFG1_PWD_CURR_110C_DELTA_CACHE_REF_SHIFT                           24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_110C_DELTA_RFMEM_RET_MASK                    0x00ff0000
#define FCFG1_PWD_CURR_110C_DELTA_RFMEM_RET_SHIFT                           16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_110C_DELTA_XOSC_LPM_MASK                     0x0000ff00
#define FCFG1_PWD_CURR_110C_DELTA_XOSC_LPM_SHIFT                             8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_110C_BASELINE_MASK                           0x000000ff
#define FCFG1_PWD_CURR_110C_BASELINE_SHIFT                                   0

/******************************************************************************
 *
 * Register: FCFG1_PWD_CURR_125C
 *
 ******************************************************************************
 * Field: [31:24] DELTA_CACHE_REF
 *
 * Additional maximum current, in units of 1uA, with cache retention
 */

#define FCFG1_PWD_CURR_125C_DELTA_CACHE_REF_MASK                    0xff000000
#define FCFG1_PWD_CURR_125C_DELTA_CACHE_REF_SHIFT                           24

/* Field: [23:16] DELTA_RFMEM_RET
 *
 * Additional maximum current, in 1uA units, with RF memory retention
 */

#define FCFG1_PWD_CURR_125C_DELTA_RFMEM_RET_MASK                    0x00ff0000
#define FCFG1_PWD_CURR_125C_DELTA_RFMEM_RET_SHIFT                           16

/* Field:  [15:8] DELTA_XOSC_LPM
 *
 * Additional maximum current, in units of 1uA, with XOSC_HF on in low-power
 * mode
 */

#define FCFG1_PWD_CURR_125C_DELTA_XOSC_LPM_MASK                     0x0000ff00
#define FCFG1_PWD_CURR_125C_DELTA_XOSC_LPM_SHIFT                             8

/* Field:   [7:0] BASELINE
 *
 * Worst-case baseline maximum powerdown current, in units of 0.5uA
 */

#define FCFG1_PWD_CURR_125C_BASELINE_MASK                           0x000000ff
#define FCFG1_PWD_CURR_125C_BASELINE_SHIFT                                   0

/******************************************************************************
 *
 * Register: FCFG1_SHDW_DIE_ID_0
 *
 ******************************************************************************
 * Field:  [31:0] ID_31_0
 *
 * Shadow of DIE_ID_0 register in eFuse row number 5
 */

#define FCFG1_SHDW_DIE_ID_0_ID_31_0_MASK                            0xffffffff
#define FCFG1_SHDW_DIE_ID_0_ID_31_0_SHIFT                                    0

/******************************************************************************
 *
 * Register: FCFG1_SHDW_DIE_ID_1
 *
 ******************************************************************************
 * Field:  [31:0] ID_63_32
 *
 * Shadow of DIE_ID_1 register in eFuse row number 6
 */

#define FCFG1_SHDW_DIE_ID_1_ID_63_32_MASK                           0xffffffff
#define FCFG1_SHDW_DIE_ID_1_ID_63_32_SHIFT                                   0

/******************************************************************************
 *
 * Register: FCFG1_SHDW_DIE_ID_2
 *
 ******************************************************************************
 * Field:  [31:0] ID_95_64
 *
 * Shadow of DIE_ID_2 register in eFuse row number 7
 */

#define FCFG1_SHDW_DIE_ID_2_ID_95_64_MASK                           0xffffffff
#define FCFG1_SHDW_DIE_ID_2_ID_95_64_SHIFT                                   0

/******************************************************************************
 *
 * Register: FCFG1_SHDW_DIE_ID_3
 *
 ******************************************************************************
 * Field:  [31:0] ID_127_96
 *
 * Shadow of DIE_ID_3 register in eFuse row number 8
 */

#define FCFG1_SHDW_DIE_ID_3_ID_127_96_MASK                          0xffffffff
#define FCFG1_SHDW_DIE_ID_3_ID_127_96_SHIFT                                  0

/******************************************************************************
 *
 * Register: FCFG1_SHDW_OSC_BIAS_LDO_TRIM
 *
 ******************************************************************************
 * Field: [26:23] TRIMMAG
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_TRIMMAG_MASK                   0x07800000
#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_TRIMMAG_SHIFT                          23

/* Field: [22:18] TRIMIREF
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_TRIMIREF_MASK                  0x007c0000
#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_TRIMIREF_SHIFT                         18

/* Field: [17:16] ITRIM_DIG_LDO
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_ITRIM_DIG_LDO_MASK             0x00030000
#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_ITRIM_DIG_LDO_SHIFT                    16

/* Field: [15:12] VTRIM_DIG
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_VTRIM_DIG_MASK                 0x0000f000
#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_VTRIM_DIG_SHIFT                        12

/* Field:  [11:8] VTRIM_COARSE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_VTRIM_COARSE_MASK              0x00000f00
#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_VTRIM_COARSE_SHIFT                      8

/* Field:   [7:0] RCOSCHF_CTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_RCOSCHF_CTRIM_MASK             0x000000ff
#define FCFG1_SHDW_OSC_BIAS_LDO_TRIM_RCOSCHF_CTRIM_SHIFT                     0

/******************************************************************************
 *
 * Register: FCFG1_SHDW_ANA_TRIM
 *
 ******************************************************************************
 * Field: [26:25] BOD_BANDGAP_TRIM_CNF
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_BOD_BANDGAP_TRIM_CNF_MASK               0x06000000
#define FCFG1_SHDW_ANA_TRIM_BOD_BANDGAP_TRIM_CNF_SHIFT                      25

/* Field:    [24] VDDR_ENABLE_PG1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_VDDR_ENABLE_PG1                         0x01000000
#define FCFG1_SHDW_ANA_TRIM_VDDR_ENABLE_PG1_MASK                    0x01000000
#define FCFG1_SHDW_ANA_TRIM_VDDR_ENABLE_PG1_SHIFT                           24

/* Field:    [23] VDDR_OK_HYS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_VDDR_OK_HYS                             0x00800000
#define FCFG1_SHDW_ANA_TRIM_VDDR_OK_HYS_MASK                        0x00800000
#define FCFG1_SHDW_ANA_TRIM_VDDR_OK_HYS_SHIFT                               23

/* Field: [22:21] IPTAT_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_IPTAT_TRIM_MASK                         0x00600000
#define FCFG1_SHDW_ANA_TRIM_IPTAT_TRIM_SHIFT                                21

/* Field: [20:16] VDDR_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_VDDR_TRIM_MASK                          0x001f0000
#define FCFG1_SHDW_ANA_TRIM_VDDR_TRIM_SHIFT                                 16

/* Field: [15:11] TRIMBOD_INTMODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_TRIMBOD_INTMODE_MASK                    0x0000f800
#define FCFG1_SHDW_ANA_TRIM_TRIMBOD_INTMODE_SHIFT                           11

/* Field:  [10:6] TRIMBOD_EXTMODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_TRIMBOD_EXTMODE_MASK                    0x000007c0
#define FCFG1_SHDW_ANA_TRIM_TRIMBOD_EXTMODE_SHIFT                            6

/* Field:   [5:0] TRIMTEMP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_SHDW_ANA_TRIM_TRIMTEMP_MASK                           0x0000003f
#define FCFG1_SHDW_ANA_TRIM_TRIMTEMP_SHIFT                                   0

/******************************************************************************
 *
 * Register: FCFG1_DAC_BIAS_CNF
 *
 ******************************************************************************
 * Field: [17:12] LPM_TRIM_IOUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_BIAS_CNF_LPM_TRIM_IOUT_MASK                       0x0003f000
#define FCFG1_DAC_BIAS_CNF_LPM_TRIM_IOUT_SHIFT                              12

/* Field:  [11:9] LPM_BIAS_WIDTH_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_BIAS_CNF_LPM_BIAS_WIDTH_TRIM_MASK                 0x00000e00
#define FCFG1_DAC_BIAS_CNF_LPM_BIAS_WIDTH_TRIM_SHIFT                         9

/* Field:     [8] LPM_BIAS_BACKUP_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_BIAS_CNF_LPM_BIAS_BACKUP_EN                       0x00000100
#define FCFG1_DAC_BIAS_CNF_LPM_BIAS_BACKUP_EN_MASK                  0x00000100
#define FCFG1_DAC_BIAS_CNF_LPM_BIAS_BACKUP_EN_SHIFT                          8

/******************************************************************************
 *
 * Register: FCFG1_TFW_PROBE
 *
 ******************************************************************************
 * Field:  [31:0] REV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_TFW_PROBE_REV_MASK                                    0xffffffff
#define FCFG1_TFW_PROBE_REV_SHIFT                                            0

/******************************************************************************
 *
 * Register: FCFG1_TFW_FT
 *
 ******************************************************************************
 * Field:  [31:0] REV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_TFW_FT_REV_MASK                                       0xffffffff
#define FCFG1_TFW_FT_REV_SHIFT                                               0

/******************************************************************************
 *
 * Register: FCFG1_DAC_CAL0
 *
 ******************************************************************************
 * Field: [31:16] SOC_DAC_VOUT_CAL_DECOUPLE_C2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL0_SOC_DAC_VOUT_CAL_DECOUPLE_C2_MASK            0xffff0000
#define FCFG1_DAC_CAL0_SOC_DAC_VOUT_CAL_DECOUPLE_C2_SHIFT                   16

/* Field:  [15:0] SOC_DAC_VOUT_CAL_DECOUPLE_C1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL0_SOC_DAC_VOUT_CAL_DECOUPLE_C1_MASK            0x0000ffff
#define FCFG1_DAC_CAL0_SOC_DAC_VOUT_CAL_DECOUPLE_C1_SHIFT                    0

/******************************************************************************
 *
 * Register: FCFG1_DAC_CAL1
 *
 ******************************************************************************
 * Field: [31:16] SOC_DAC_VOUT_CAL_PRECH_C2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL1_SOC_DAC_VOUT_CAL_PRECH_C2_MASK               0xffff0000
#define FCFG1_DAC_CAL1_SOC_DAC_VOUT_CAL_PRECH_C2_SHIFT                      16

/* Field:  [15:0] SOC_DAC_VOUT_CAL_PRECH_C1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL1_SOC_DAC_VOUT_CAL_PRECH_C1_MASK               0x0000ffff
#define FCFG1_DAC_CAL1_SOC_DAC_VOUT_CAL_PRECH_C1_SHIFT                       0

/******************************************************************************
 *
 * Register: FCFG1_DAC_CAL2
 *
 ******************************************************************************
 * Field: [31:16] SOC_DAC_VOUT_CAL_ADCREF_C2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL2_SOC_DAC_VOUT_CAL_ADCREF_C2_MASK              0xffff0000
#define FCFG1_DAC_CAL2_SOC_DAC_VOUT_CAL_ADCREF_C2_SHIFT                     16

/* Field:  [15:0] SOC_DAC_VOUT_CAL_ADCREF_C1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL2_SOC_DAC_VOUT_CAL_ADCREF_C1_MASK              0x0000ffff
#define FCFG1_DAC_CAL2_SOC_DAC_VOUT_CAL_ADCREF_C1_SHIFT                      0

/******************************************************************************
 *
 * Register: FCFG1_DAC_CAL3
 *
 ******************************************************************************
 * Field: [31:16] SOC_DAC_VOUT_CAL_VDDS_C2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL3_SOC_DAC_VOUT_CAL_VDDS_C2_MASK                0xffff0000
#define FCFG1_DAC_CAL3_SOC_DAC_VOUT_CAL_VDDS_C2_SHIFT                       16

/* Field:  [15:0] SOC_DAC_VOUT_CAL_VDDS_C1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FCFG1_DAC_CAL3_SOC_DAC_VOUT_CAL_VDDS_C1_MASK                0x0000ffff
#define FCFG1_DAC_CAL3_SOC_DAC_VOUT_CAL_VDDS_C1_SHIFT                        0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_FCFG1_H */
