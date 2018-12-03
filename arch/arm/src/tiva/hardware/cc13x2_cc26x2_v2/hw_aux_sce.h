/******************************************************************************
 *  Filename:       hw_aux_sce_h
 *  Revised:        2018-05-14 12:24:52 +0200 (Mon, 14 May 2018)
 *  Revision:       51990
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_AUX_SCE_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_AUX_SCE_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * AUX_SCE component
 *
 ******************************************************************************
 * Internal
 */

#define AUX_SCE_CTL_OFFSET                                          0x00000000

/* Internal */

#define AUX_SCE_FETCHSTAT_OFFSET                                    0x00000004

/* Internal */

#define AUX_SCE_CPUSTAT_OFFSET                                      0x00000008

/* Internal */

#define AUX_SCE_WUSTAT_OFFSET                                       0x0000000c

/* Internal */

#define AUX_SCE_REG1_0_OFFSET                                       0x00000010

/* Internal */

#define AUX_SCE_REG3_2_OFFSET                                       0x00000014

/* Internal */

#define AUX_SCE_REG5_4_OFFSET                                       0x00000018

/* Internal */

#define AUX_SCE_REG7_6_OFFSET                                       0x0000001c

/* Internal */

#define AUX_SCE_LOOPADDR_OFFSET                                     0x00000020

/* Internal */

#define AUX_SCE_LOOPCNT_OFFSET                                      0x00000024

/******************************************************************************
 *
 * Register: AUX_SCE_CTL
 *
 ******************************************************************************
 * Field: [31:24] FORCE_EV_LOW
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_FORCE_EV_LOW_MASK                               0xff000000
#define AUX_SCE_CTL_FORCE_EV_LOW_SHIFT                                      24

/* Field: [23:16] FORCE_EV_HIGH
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_FORCE_EV_HIGH_MASK                              0x00ff0000
#define AUX_SCE_CTL_FORCE_EV_HIGH_SHIFT                                     16

/* Field:  [15:8] RESET_VECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_RESET_VECTOR_MASK                               0x0000ff00
#define AUX_SCE_CTL_RESET_VECTOR_SHIFT                                       8

/* Field:     [6] DBG_FREEZE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_DBG_FREEZE_EN                                   0x00000040
#define AUX_SCE_CTL_DBG_FREEZE_EN_MASK                              0x00000040
#define AUX_SCE_CTL_DBG_FREEZE_EN_SHIFT                                      6

/* Field:     [5] FORCE_WU_LOW
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_FORCE_WU_LOW                                    0x00000020
#define AUX_SCE_CTL_FORCE_WU_LOW_MASK                               0x00000020
#define AUX_SCE_CTL_FORCE_WU_LOW_SHIFT                                       5

/* Field:     [4] FORCE_WU_HIGH
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_FORCE_WU_HIGH                                   0x00000010
#define AUX_SCE_CTL_FORCE_WU_HIGH_MASK                              0x00000010
#define AUX_SCE_CTL_FORCE_WU_HIGH_SHIFT                                      4

/* Field:     [3] RESTART
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_RESTART                                         0x00000008
#define AUX_SCE_CTL_RESTART_MASK                                    0x00000008
#define AUX_SCE_CTL_RESTART_SHIFT                                            3

/* Field:     [2] SINGLE_STEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_SINGLE_STEP                                     0x00000004
#define AUX_SCE_CTL_SINGLE_STEP_MASK                                0x00000004
#define AUX_SCE_CTL_SINGLE_STEP_SHIFT                                        2

/* Field:     [1] SUSPEND
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_SUSPEND                                         0x00000002
#define AUX_SCE_CTL_SUSPEND_MASK                                    0x00000002
#define AUX_SCE_CTL_SUSPEND_SHIFT                                            1

/* Field:     [0] CLK_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CTL_CLK_EN                                          0x00000001
#define AUX_SCE_CTL_CLK_EN_MASK                                     0x00000001
#define AUX_SCE_CTL_CLK_EN_SHIFT                                             0

/******************************************************************************
 *
 * Register: AUX_SCE_FETCHSTAT
 *
 ******************************************************************************
 * Field: [31:16] OPCODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_FETCHSTAT_OPCODE_MASK                               0xffff0000
#define AUX_SCE_FETCHSTAT_OPCODE_SHIFT                                      16

/* Field:  [15:0] PC
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_FETCHSTAT_PC_MASK                                   0x0000ffff
#define AUX_SCE_FETCHSTAT_PC_SHIFT                                           0

/******************************************************************************
 *
 * Register: AUX_SCE_CPUSTAT
 *
 ******************************************************************************
 * Field:    [11] BUS_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_BUS_ERROR                                   0x00000800
#define AUX_SCE_CPUSTAT_BUS_ERROR_MASK                              0x00000800
#define AUX_SCE_CPUSTAT_BUS_ERROR_SHIFT                                     11

/* Field:    [10] SLEEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_SLEEP                                       0x00000400
#define AUX_SCE_CPUSTAT_SLEEP_MASK                                  0x00000400
#define AUX_SCE_CPUSTAT_SLEEP_SHIFT                                         10

/* Field:     [9] WEV
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_WEV                                         0x00000200
#define AUX_SCE_CPUSTAT_WEV_MASK                                    0x00000200
#define AUX_SCE_CPUSTAT_WEV_SHIFT                                            9

/* Field:     [8] HALTED
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_HALTED                                      0x00000100
#define AUX_SCE_CPUSTAT_HALTED_MASK                                 0x00000100
#define AUX_SCE_CPUSTAT_HALTED_SHIFT                                         8

/* Field:     [3] V_FLAG
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_V_FLAG                                      0x00000008
#define AUX_SCE_CPUSTAT_V_FLAG_MASK                                 0x00000008
#define AUX_SCE_CPUSTAT_V_FLAG_SHIFT                                         3

/* Field:     [2] C_FLAG
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_C_FLAG                                      0x00000004
#define AUX_SCE_CPUSTAT_C_FLAG_MASK                                 0x00000004
#define AUX_SCE_CPUSTAT_C_FLAG_SHIFT                                         2

/* Field:     [1] N_FLAG
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_N_FLAG                                      0x00000002
#define AUX_SCE_CPUSTAT_N_FLAG_MASK                                 0x00000002
#define AUX_SCE_CPUSTAT_N_FLAG_SHIFT                                         1

/* Field:     [0] Z_FLAG
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_CPUSTAT_Z_FLAG                                      0x00000001
#define AUX_SCE_CPUSTAT_Z_FLAG_MASK                                 0x00000001
#define AUX_SCE_CPUSTAT_Z_FLAG_SHIFT                                         0

/******************************************************************************
 *
 * Register: AUX_SCE_WUSTAT
 *
 ******************************************************************************
 * Field: [18:16] EXC_VECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_WUSTAT_EXC_VECTOR_MASK                              0x00070000
#define AUX_SCE_WUSTAT_EXC_VECTOR_SHIFT                                     16

/* Field:     [8] WU_SIGNAL
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_WUSTAT_WU_SIGNAL                                    0x00000100
#define AUX_SCE_WUSTAT_WU_SIGNAL_MASK                               0x00000100
#define AUX_SCE_WUSTAT_WU_SIGNAL_SHIFT                                       8

/* Field:   [7:0] EV_SIGNALS
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * SCEWEV_PROG              Internal. Only to be used through TI provided API.
 * AUX_ADC_FIFO_NOT_EMPTY   Internal. Only to be used through TI provided API.
 * AUX_TIMER1_EV_OR_IDLE    Internal. Only to be used through TI provided API.
 * AUX_TIMER0_EV_OR_IDLE    Internal. Only to be used through TI provided API.
 * AUX_TDC_DONE             Internal. Only to be used through TI provided API.
 * AUX_COMPB                Internal. Only to be used through TI provided API.
 * AUX_COMPA                Internal. Only to be used through TI provided API.
 * AUX_PROG_DLY_IDLE        Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_WUSTAT_EV_SIGNALS_MASK                              0x000000ff
#define AUX_SCE_WUSTAT_EV_SIGNALS_SHIFT                                      0
#define AUX_SCE_WUSTAT_EV_SIGNALS_SCEWEV_PROG                       0x00000080
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_ADC_FIFO_NOT_EMPTY            0x00000040
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_TIMER1_EV_OR_IDLE             0x00000020
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_TIMER0_EV_OR_IDLE             0x00000010
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_TDC_DONE                      0x00000008
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_COMPB                         0x00000004
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_COMPA                         0x00000002
#define AUX_SCE_WUSTAT_EV_SIGNALS_AUX_PROG_DLY_IDLE                 0x00000001

/******************************************************************************
 *
 * Register: AUX_SCE_REG1_0
 *
 ******************************************************************************
 * Field: [31:16] REG1
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG1_0_REG1_MASK                                    0xffff0000
#define AUX_SCE_REG1_0_REG1_SHIFT                                           16

/* Field:  [15:0] REG0
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG1_0_REG0_MASK                                    0x0000ffff
#define AUX_SCE_REG1_0_REG0_SHIFT                                            0

/******************************************************************************
 *
 * Register: AUX_SCE_REG3_2
 *
 ******************************************************************************
 * Field: [31:16] REG3
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG3_2_REG3_MASK                                    0xffff0000
#define AUX_SCE_REG3_2_REG3_SHIFT                                           16

/* Field:  [15:0] REG2
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG3_2_REG2_MASK                                    0x0000ffff
#define AUX_SCE_REG3_2_REG2_SHIFT                                            0

/******************************************************************************
 *
 * Register: AUX_SCE_REG5_4
 *
 ******************************************************************************
 * Field: [31:16] REG5
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG5_4_REG5_MASK                                    0xffff0000
#define AUX_SCE_REG5_4_REG5_SHIFT                                           16

/* Field:  [15:0] REG4
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG5_4_REG4_MASK                                    0x0000ffff
#define AUX_SCE_REG5_4_REG4_SHIFT                                            0

/******************************************************************************
 *
 * Register: AUX_SCE_REG7_6
 *
 ******************************************************************************
 * Field: [31:16] REG7
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG7_6_REG7_MASK                                    0xffff0000
#define AUX_SCE_REG7_6_REG7_SHIFT                                           16

/* Field:  [15:0] REG6
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_REG7_6_REG6_MASK                                    0x0000ffff
#define AUX_SCE_REG7_6_REG6_SHIFT                                            0

/******************************************************************************
 *
 * Register: AUX_SCE_LOOPADDR
 *
 ******************************************************************************
 * Field: [31:16] STOP
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_LOOPADDR_STOP_MASK                                  0xffff0000
#define AUX_SCE_LOOPADDR_STOP_SHIFT                                         16

/* Field:  [15:0] START
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_LOOPADDR_START_MASK                                 0x0000ffff
#define AUX_SCE_LOOPADDR_START_SHIFT                                         0

/******************************************************************************
 *
 * Register: AUX_SCE_LOOPCNT
 *
 ******************************************************************************
 * Field:   [7:0] ITER_LEFT
 *
 * Internal. Only to be used through TI provided API.
 */

#define AUX_SCE_LOOPCNT_ITER_LEFT_MASK                              0x000000ff
#define AUX_SCE_LOOPCNT_ITER_LEFT_SHIFT                                      0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_AUX_SCE_H */
