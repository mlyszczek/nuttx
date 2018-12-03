/******************************************************************************
 *  Filename:       hw_flash_h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_HW_FLASH_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_HW_FLASH_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * FLASH component
 *
 ******************************************************************************
 * FMC and Efuse Status
 */

#define FLASH_STAT_OFFSET                                           0x0000001c

/* Internal */

#define FLASH_CFG_OFFSET                                            0x00000024

/* Internal */

#define FLASH_SYSCODE_START_OFFSET                                  0x00000028

/* Internal */

#define FLASH_FLASH_SIZE_OFFSET                                     0x0000002c

/* Internal */

#define FLASH_FWLOCK_OFFSET                                         0x0000003c

/* Internal */

#define FLASH_FWFLAG_OFFSET                                         0x00000040

/* Internal */

#define FLASH_EFUSE_OFFSET                                          0x00001000

/* Internal */

#define FLASH_EFUSEADDR_OFFSET                                      0x00001004

/* Internal */

#define FLASH_DATAUPPER_OFFSET                                      0x00001008

/* Internal */

#define FLASH_DATALOWER_OFFSET                                      0x0000100c

/* Internal */

#define FLASH_EFUSECFG_OFFSET                                       0x00001010

/* Internal */

#define FLASH_EFUSESTAT_OFFSET                                      0x00001014

/* Internal */

#define FLASH_ACC_OFFSET                                            0x00001018

/* Internal */

#define FLASH_BOUNDARY_OFFSET                                       0x0000101c

/* Internal */

#define FLASH_EFUSEFLAG_OFFSET                                      0x00001020

/* Internal */

#define FLASH_EFUSEKEY_OFFSET                                       0x00001024

/* Internal */

#define FLASH_EFUSERELEASE_OFFSET                                   0x00001028

/* Internal */

#define FLASH_EFUSEPINS_OFFSET                                      0x0000102c

/* Internal */

#define FLASH_EFUSECRA_OFFSET                                       0x00001030

/* Internal */

#define FLASH_EFUSEREAD_OFFSET                                      0x00001034

/* Internal */

#define FLASH_EFUSEPROGRAM_OFFSET                                   0x00001038

/* Internal */

#define FLASH_EFUSEERROR_OFFSET                                     0x0000103c

/* Internal */

#define FLASH_SINGLEBIT_OFFSET                                      0x00001040

/* Internal */

#define FLASH_TWOBIT_OFFSET                                         0x00001044

/* Internal */

#define FLASH_SELFTESTCYC_OFFSET                                    0x00001048

/* Internal */

#define FLASH_SELFTESTSIGN_OFFSET                                   0x0000104c

/* Internal */

#define FLASH_FRDCTL_OFFSET                                         0x00002000

/* Internal */

#define FLASH_FSPRD_OFFSET                                          0x00002004

/* Internal */

#define FLASH_FEDACCTL1_OFFSET                                      0x00002008

/* Internal */

#define FLASH_FEDACSTAT_OFFSET                                      0x0000201c

/* Internal */

#define FLASH_FBPROT_OFFSET                                         0x00002030

/* Internal */

#define FLASH_FBSE_OFFSET                                           0x00002034

/* Internal */

#define FLASH_FBBUSY_OFFSET                                         0x00002038

/* Internal */

#define FLASH_FBAC_OFFSET                                           0x0000203c

/* Internal */

#define FLASH_FBFALLBACK_OFFSET                                     0x00002040

/* Internal */

#define FLASH_FBPRDY_OFFSET                                         0x00002044

/* Internal */

#define FLASH_FPAC1_OFFSET                                          0x00002048

/* Internal */

#define FLASH_FPAC2_OFFSET                                          0x0000204c

/* Internal */

#define FLASH_FMAC_OFFSET                                           0x00002050

/* Internal */

#define FLASH_FMSTAT_OFFSET                                         0x00002054

/* Internal */

#define FLASH_FLOCK_OFFSET                                          0x00002064

/* Internal */

#define FLASH_FVREADCT_OFFSET                                       0x00002080

/* Internal */

#define FLASH_FVHVCT1_OFFSET                                        0x00002084

/* Internal */

#define FLASH_FVHVCT2_OFFSET                                        0x00002088

/* Internal */

#define FLASH_FVHVCT3_OFFSET                                        0x0000208c

/* Internal */

#define FLASH_FVNVCT_OFFSET                                         0x00002090

/* Internal */

#define FLASH_FVSLP_OFFSET                                          0x00002094

/* Internal */

#define FLASH_FVWLCT_OFFSET                                         0x00002098

/* Internal */

#define FLASH_FEFUSECTL_OFFSET                                      0x0000209c

/* Internal */

#define FLASH_FEFUSESTAT_OFFSET                                     0x000020a0

/* Internal */

#define FLASH_FEFUSEDATA_OFFSET                                     0x000020a4

/* Internal */

#define FLASH_FSEQPMP_OFFSET                                        0x000020a8

/* Internal */

#define FLASH_FBSTROBES_OFFSET                                      0x00002100

/* Internal */

#define FLASH_FPSTROBES_OFFSET                                      0x00002104

/* Internal */

#define FLASH_FBMODE_OFFSET                                         0x00002108

/* Internal */

#define FLASH_FTCR_OFFSET                                           0x0000210c

/* Internal */

#define FLASH_FADDR_OFFSET                                          0x00002110

/* Internal */

#define FLASH_FTCTL_OFFSET                                          0x0000211c

/* Internal */

#define FLASH_FWPWRITE0_OFFSET                                      0x00002120

/* Internal */

#define FLASH_FWPWRITE1_OFFSET                                      0x00002124

/* Internal */

#define FLASH_FWPWRITE2_OFFSET                                      0x00002128

/* Internal */

#define FLASH_FWPWRITE3_OFFSET                                      0x0000212c

/* Internal */

#define FLASH_FWPWRITE4_OFFSET                                      0x00002130

/* Internal */

#define FLASH_FWPWRITE5_OFFSET                                      0x00002134

/* Internal */

#define FLASH_FWPWRITE6_OFFSET                                      0x00002138

/* Internal */

#define FLASH_FWPWRITE7_OFFSET                                      0x0000213c

/* Internal */

#define FLASH_FWPWRITE_ECC_OFFSET                                   0x00002140

/* Internal */

#define FLASH_FSWSTAT_OFFSET                                        0x00002144

/* Internal */

#define FLASH_FSM_GLBCTL_OFFSET                                     0x00002200

/* Internal */

#define FLASH_FSM_STATE_OFFSET                                      0x00002204

/* Internal */

#define FLASH_FSM_STAT_OFFSET                                       0x00002208

/* Internal */

#define FLASH_FSM_CMD_OFFSET                                        0x0000220c

/* Internal */

#define FLASH_FSM_PE_OSU_OFFSET                                     0x00002210

/* Internal */

#define FLASH_FSM_VSTAT_OFFSET                                      0x00002214

/* Internal */

#define FLASH_FSM_PE_VSU_OFFSET                                     0x00002218

/* Internal */

#define FLASH_FSM_CMP_VSU_OFFSET                                    0x0000221c

/* Internal */

#define FLASH_FSM_EX_VAL_OFFSET                                     0x00002220

/* Internal */

#define FLASH_FSM_RD_H_OFFSET                                       0x00002224

/* Internal */

#define FLASH_FSM_P_OH_OFFSET                                       0x00002228

/* Internal */

#define FLASH_FSM_ERA_OH_OFFSET                                     0x0000222c

/* Internal */

#define FLASH_FSM_SAV_PPUL_OFFSET                                   0x00002230

/* Internal */

#define FLASH_FSM_PE_VH_OFFSET                                      0x00002234

/* Internal */

#define FLASH_FSM_PRG_PW_OFFSET                                     0x00002240

/* Internal */

#define FLASH_FSM_ERA_PW_OFFSET                                     0x00002244

/* Internal */

#define FLASH_FSM_SAV_ERA_PUL_OFFSET                                0x00002254

/* Internal */

#define FLASH_FSM_TIMER_OFFSET                                      0x00002258

/* Internal */

#define FLASH_FSM_MODE_OFFSET                                       0x0000225c

/* Internal */

#define FLASH_FSM_PGM_OFFSET                                        0x00002260

/* Internal */

#define FLASH_FSM_ERA_OFFSET                                        0x00002264

/* Internal */

#define FLASH_FSM_PRG_PUL_OFFSET                                    0x00002268

/* Internal */

#define FLASH_FSM_ERA_PUL_OFFSET                                    0x0000226c

/* Internal */

#define FLASH_FSM_STEP_SIZE_OFFSET                                  0x00002270

/* Internal */

#define FLASH_FSM_PUL_CNTR_OFFSET                                   0x00002274

/* Internal */

#define FLASH_FSM_EC_STEP_HEIGHT_OFFSET                             0x00002278

/* Internal */

#define FLASH_FSM_ST_MACHINE_OFFSET                                 0x0000227c

/* Internal */

#define FLASH_FSM_FLES_OFFSET                                       0x00002280

/* Internal */

#define FLASH_FSM_WR_ENA_OFFSET                                     0x00002288

/* Internal */

#define FLASH_FSM_ACC_PP_OFFSET                                     0x0000228c

/* Internal */

#define FLASH_FSM_ACC_EP_OFFSET                                     0x00002290

/* Internal */

#define FLASH_FSM_ADDR_OFFSET                                       0x000022a0

/* Internal */

#define FLASH_FSM_SECTOR_OFFSET                                     0x000022a4

/* Internal */

#define FLASH_FMC_REV_ID_OFFSET                                     0x000022a8

/* Internal */

#define FLASH_FSM_ERR_ADDR_OFFSET                                   0x000022ac

/* Internal */

#define FLASH_FSM_PGM_MAXPUL_OFFSET                                 0x000022b0

/* Internal */

#define FLASH_FSM_EXECUTE_OFFSET                                    0x000022b4

/* Internal */

#define FLASH_FSM_SECTOR1_OFFSET                                    0x000022c0

/* Internal */

#define FLASH_FSM_SECTOR2_OFFSET                                    0x000022c4

/* Internal */

#define FLASH_FSM_BSLE0_OFFSET                                      0x000022e0

/* Internal */

#define FLASH_FSM_BSLE1_OFFSET                                      0x000022e4

/* Internal */

#define FLASH_FSM_BSLP0_OFFSET                                      0x000022f0

/* Internal */

#define FLASH_FSM_BSLP1_OFFSET                                      0x000022f4

/* Internal */

#define FLASH_FCFG_BANK_OFFSET                                      0x00002400

/* Internal */

#define FLASH_FCFG_WRAPPER_OFFSET                                   0x00002404

/* Internal */

#define FLASH_FCFG_BNK_TYPE_OFFSET                                  0x00002408

/* Internal */

#define FLASH_FCFG_B0_START_OFFSET                                  0x00002410

/* Internal */

#define FLASH_FCFG_B1_START_OFFSET                                  0x00002414

/* Internal */

#define FLASH_FCFG_B2_START_OFFSET                                  0x00002418

/* Internal */

#define FLASH_FCFG_B3_START_OFFSET                                  0x0000241c

/* Internal */

#define FLASH_FCFG_B4_START_OFFSET                                  0x00002420

/* Internal */

#define FLASH_FCFG_B5_START_OFFSET                                  0x00002424

/* Internal */

#define FLASH_FCFG_B6_START_OFFSET                                  0x00002428

/* Internal */

#define FLASH_FCFG_B7_START_OFFSET                                  0x0000242c

/* Internal */

#define FLASH_FCFG_B0_SSIZE0_OFFSET                                 0x00002430

/******************************************************************************
 *
 * Register: FLASH_STAT
 *
 ******************************************************************************
 * Field:    [15] EFUSE_BLANK
 *
 * Efuse scanning detected if fuse ROM is blank:
 * 0 : Not blank
 * 1 : Blank
 */

#define FLASH_STAT_EFUSE_BLANK                                      0x00008000
#define FLASH_STAT_EFUSE_BLANK_MASK                                 0x00008000
#define FLASH_STAT_EFUSE_BLANK_SHIFT                                        15

/* Field:    [14] EFUSE_TIMEOUT
 *
 * Efuse scanning resulted in timeout error.
 * 0 : No Timeout error
 * 1 : Timeout Error
 */

#define FLASH_STAT_EFUSE_TIMEOUT                                    0x00004000
#define FLASH_STAT_EFUSE_TIMEOUT_MASK                               0x00004000
#define FLASH_STAT_EFUSE_TIMEOUT_SHIFT                                      14

/* Field:    [13] EFUSE_CRC_ERROR
 *
 * Efuse scanning resulted in scan chain CRC error.
 * 0 : No CRC error
 * 1 : CRC Error
 */

#define FLASH_STAT_EFUSE_CRC_ERROR                                  0x00002000
#define FLASH_STAT_EFUSE_CRC_ERROR_MASK                             0x00002000
#define FLASH_STAT_EFUSE_CRC_ERROR_SHIFT                                    13

/* Field:  [12:8] EFUSE_ERRCODE
 *
 * Same as EFUSEERROR.CODE
 */

#define FLASH_STAT_EFUSE_ERRCODE_MASK                               0x00001f00
#define FLASH_STAT_EFUSE_ERRCODE_SHIFT                                       8

/* Field:     [2] SAMHOLD_DIS
 *
 * Status indicator of flash sample and hold sequencing logic. This bit will go
 * to 1 some delay after CFG.DIS_IDLE is set to 1.
 * 0: Not disabled
 * 1: Sample and hold disabled and stable
 */

#define FLASH_STAT_SAMHOLD_DIS                                      0x00000004
#define FLASH_STAT_SAMHOLD_DIS_MASK                                 0x00000004
#define FLASH_STAT_SAMHOLD_DIS_SHIFT                                         2

/* Field:     [1] BUSY
 *
 * Fast version of the FMC FMSTAT.BUSY bit.
 * This flag is valid immediately after the operation setting it (FMSTAT.BUSY
 * is delayed some cycles)
 * 0 : Not busy
 * 1 : Busy
 */

#define FLASH_STAT_BUSY                                             0x00000002
#define FLASH_STAT_BUSY_MASK                                        0x00000002
#define FLASH_STAT_BUSY_SHIFT                                                1

/* Field:     [0] POWER_MODE
 *
 * Power state of the flash sub-system.
 * 0 : Active
 * 1 : Low power
 */

#define FLASH_STAT_POWER_MODE                                       0x00000001
#define FLASH_STAT_POWER_MODE_MASK                                  0x00000001
#define FLASH_STAT_POWER_MODE_SHIFT                                          0

/******************************************************************************
 *
 * Register: FLASH_CFG
 *
 ******************************************************************************
 * Field:     [8] STANDBY_MODE_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_STANDBY_MODE_SEL                                  0x00000100
#define FLASH_CFG_STANDBY_MODE_SEL_MASK                             0x00000100
#define FLASH_CFG_STANDBY_MODE_SEL_SHIFT                                     8

/* Field:   [7:6] STANDBY_PW_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_STANDBY_PW_SEL_MASK                               0x000000c0
#define FLASH_CFG_STANDBY_PW_SEL_SHIFT                                       6

/* Field:     [5] DIS_EFUSECLK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_DIS_EFUSECLK                                      0x00000020
#define FLASH_CFG_DIS_EFUSECLK_MASK                                 0x00000020
#define FLASH_CFG_DIS_EFUSECLK_SHIFT                                         5

/* Field:     [4] DIS_READACCESS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_DIS_READACCESS                                    0x00000010
#define FLASH_CFG_DIS_READACCESS_MASK                               0x00000010
#define FLASH_CFG_DIS_READACCESS_SHIFT                                       4

/* Field:     [3] ENABLE_SWINTF
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_ENABLE_SWINTF                                     0x00000008
#define FLASH_CFG_ENABLE_SWINTF_MASK                                0x00000008
#define FLASH_CFG_ENABLE_SWINTF_SHIFT                                        3

/* Field:     [1] DIS_STANDBY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_DIS_STANDBY                                       0x00000002
#define FLASH_CFG_DIS_STANDBY_MASK                                  0x00000002
#define FLASH_CFG_DIS_STANDBY_SHIFT                                          1

/* Field:     [0] DIS_IDLE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_CFG_DIS_IDLE                                          0x00000001
#define FLASH_CFG_DIS_IDLE_MASK                                     0x00000001
#define FLASH_CFG_DIS_IDLE_SHIFT                                             0

/******************************************************************************
 *
 * Register: FLASH_SYSCODE_START
 *
 ******************************************************************************
 * Field:   [4:0] SYSCODE_START
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_SYSCODE_START_SYSCODE_START_MASK                      0x0000001f
#define FLASH_SYSCODE_START_SYSCODE_START_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FLASH_SIZE
 *
 ******************************************************************************
 * Field:   [7:0] SECTORS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FLASH_SIZE_SECTORS_MASK                               0x000000ff
#define FLASH_FLASH_SIZE_SECTORS_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FWLOCK
 *
 ******************************************************************************
 * Field:   [2:0] FWLOCK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWLOCK_FWLOCK_MASK                                    0x00000007
#define FLASH_FWLOCK_FWLOCK_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_FWFLAG
 *
 ******************************************************************************
 * Field:   [2:0] FWFLAG
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWFLAG_FWFLAG_MASK                                    0x00000007
#define FLASH_FWFLAG_FWFLAG_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_EFUSE
 *
 ******************************************************************************
 * Field: [28:24] INSTRUCTION
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSE_INSTRUCTION_MASK                                0x1f000000
#define FLASH_EFUSE_INSTRUCTION_SHIFT                                       24

/* Field:  [15:0] DUMPWORD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSE_DUMPWORD_MASK                                   0x0000ffff
#define FLASH_EFUSE_DUMPWORD_SHIFT                                           0

/******************************************************************************
 *
 * Register: FLASH_EFUSEADDR
 *
 ******************************************************************************
 * Field: [15:11] BLOCK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEADDR_BLOCK_MASK                                  0x0000f800
#define FLASH_EFUSEADDR_BLOCK_SHIFT                                         11

/* Field:  [10:0] ROW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEADDR_ROW_MASK                                    0x000007ff
#define FLASH_EFUSEADDR_ROW_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_DATAUPPER
 *
 ******************************************************************************
 * Field:   [7:3] SPARE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_DATAUPPER_SPARE_MASK                                  0x000000f8
#define FLASH_DATAUPPER_SPARE_SHIFT                                          3

/* Field:     [2] P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_DATAUPPER_P                                           0x00000004
#define FLASH_DATAUPPER_P_MASK                                      0x00000004
#define FLASH_DATAUPPER_P_SHIFT                                              2

/* Field:     [1] R
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_DATAUPPER_R                                           0x00000002
#define FLASH_DATAUPPER_R_MASK                                      0x00000002
#define FLASH_DATAUPPER_R_SHIFT                                              1

/* Field:     [0] EEN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_DATAUPPER_EEN                                         0x00000001
#define FLASH_DATAUPPER_EEN_MASK                                    0x00000001
#define FLASH_DATAUPPER_EEN_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_DATALOWER
 *
 ******************************************************************************
 * Field:  [31:0] DATA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_DATALOWER_DATA_MASK                                   0xffffffff
#define FLASH_DATALOWER_DATA_SHIFT                                           0

/******************************************************************************
 *
 * Register: FLASH_EFUSECFG
 *
 ******************************************************************************
 * Field:     [8] IDLEGATING
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSECFG_IDLEGATING                                   0x00000100
#define FLASH_EFUSECFG_IDLEGATING_MASK                              0x00000100
#define FLASH_EFUSECFG_IDLEGATING_SHIFT                                      8

/* Field:   [4:3] SLAVEPOWER
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSECFG_SLAVEPOWER_MASK                              0x00000018
#define FLASH_EFUSECFG_SLAVEPOWER_SHIFT                                      3

/* Field:     [0] GATING
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSECFG_GATING                                       0x00000001
#define FLASH_EFUSECFG_GATING_MASK                                  0x00000001
#define FLASH_EFUSECFG_GATING_SHIFT                                          0

/******************************************************************************
 *
 * Register: FLASH_EFUSESTAT
 *
 ******************************************************************************
 * Field:     [0] RESETDONE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSESTAT_RESETDONE                                   0x00000001
#define FLASH_EFUSESTAT_RESETDONE_MASK                              0x00000001
#define FLASH_EFUSESTAT_RESETDONE_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_ACC
 *
 ******************************************************************************
 * Field:  [23:0] ACCUMULATOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_ACC_ACCUMULATOR_MASK                                  0x00ffffff
#define FLASH_ACC_ACCUMULATOR_SHIFT                                          0

/******************************************************************************
 *
 * Register: FLASH_BOUNDARY
 *
 ******************************************************************************
 * Field:    [23] DISROW0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_DISROW0                                      0x00800000
#define FLASH_BOUNDARY_DISROW0_MASK                                 0x00800000
#define FLASH_BOUNDARY_DISROW0_SHIFT                                        23

/* Field:    [22] SPARE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_SPARE                                        0x00400000
#define FLASH_BOUNDARY_SPARE_MASK                                   0x00400000
#define FLASH_BOUNDARY_SPARE_SHIFT                                          22

/* Field:    [21] EFC_SELF_TEST_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_EFC_SELF_TEST_ERROR                          0x00200000
#define FLASH_BOUNDARY_EFC_SELF_TEST_ERROR_MASK                     0x00200000
#define FLASH_BOUNDARY_EFC_SELF_TEST_ERROR_SHIFT                            21

/* Field:    [20] EFC_INSTRUCTION_INFO
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_EFC_INSTRUCTION_INFO                         0x00100000
#define FLASH_BOUNDARY_EFC_INSTRUCTION_INFO_MASK                    0x00100000
#define FLASH_BOUNDARY_EFC_INSTRUCTION_INFO_SHIFT                           20

/* Field:    [19] EFC_INSTRUCTION_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_EFC_INSTRUCTION_ERROR                        0x00080000
#define FLASH_BOUNDARY_EFC_INSTRUCTION_ERROR_MASK                   0x00080000
#define FLASH_BOUNDARY_EFC_INSTRUCTION_ERROR_SHIFT                          19

/* Field:    [18] EFC_AUTOLOAD_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_EFC_AUTOLOAD_ERROR                           0x00040000
#define FLASH_BOUNDARY_EFC_AUTOLOAD_ERROR_MASK                      0x00040000
#define FLASH_BOUNDARY_EFC_AUTOLOAD_ERROR_SHIFT                             18

/* Field: [17:14] OUTPUTENABLE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_OUTPUTENABLE_MASK                            0x0003c000
#define FLASH_BOUNDARY_OUTPUTENABLE_SHIFT                                   14

/* Field:    [13] SYS_ECC_SELF_TEST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_SYS_ECC_SELF_TEST_EN                         0x00002000
#define FLASH_BOUNDARY_SYS_ECC_SELF_TEST_EN_MASK                    0x00002000
#define FLASH_BOUNDARY_SYS_ECC_SELF_TEST_EN_SHIFT                           13

/* Field:    [12] SYS_ECC_OVERRIDE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_SYS_ECC_OVERRIDE_EN                          0x00001000
#define FLASH_BOUNDARY_SYS_ECC_OVERRIDE_EN_MASK                     0x00001000
#define FLASH_BOUNDARY_SYS_ECC_OVERRIDE_EN_SHIFT                            12

/* Field:    [11] EFC_FDI
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_EFC_FDI                                      0x00000800
#define FLASH_BOUNDARY_EFC_FDI_MASK                                 0x00000800
#define FLASH_BOUNDARY_EFC_FDI_SHIFT                                        11

/* Field:    [10] SYS_DIEID_AUTOLOAD_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_SYS_DIEID_AUTOLOAD_EN                        0x00000400
#define FLASH_BOUNDARY_SYS_DIEID_AUTOLOAD_EN_MASK                   0x00000400
#define FLASH_BOUNDARY_SYS_DIEID_AUTOLOAD_EN_SHIFT                          10

/* Field:   [9:8] SYS_REPAIR_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_SYS_REPAIR_EN_MASK                           0x00000300
#define FLASH_BOUNDARY_SYS_REPAIR_EN_SHIFT                                   8

/* Field:   [7:4] SYS_WS_READ_STATES
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_SYS_WS_READ_STATES_MASK                      0x000000f0
#define FLASH_BOUNDARY_SYS_WS_READ_STATES_SHIFT                              4

/* Field:   [3:0] INPUTENABLE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_BOUNDARY_INPUTENABLE_MASK                             0x0000000f
#define FLASH_BOUNDARY_INPUTENABLE_SHIFT                                     0

/******************************************************************************
 *
 * Register: FLASH_EFUSEFLAG
 *
 ******************************************************************************
 * Field:     [0] KEY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEFLAG_KEY                                         0x00000001
#define FLASH_EFUSEFLAG_KEY_MASK                                    0x00000001
#define FLASH_EFUSEFLAG_KEY_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_EFUSEKEY
 *
 ******************************************************************************
 * Field:  [31:0] CODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEKEY_CODE_MASK                                    0xffffffff
#define FLASH_EFUSEKEY_CODE_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_EFUSERELEASE
 *
 ******************************************************************************
 * Field: [31:25] ODPYEAR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSERELEASE_ODPYEAR_MASK                             0xfe000000
#define FLASH_EFUSERELEASE_ODPYEAR_SHIFT                                    25

/* Field: [24:21] ODPMONTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSERELEASE_ODPMONTH_MASK                            0x01e00000
#define FLASH_EFUSERELEASE_ODPMONTH_SHIFT                                   21

/* Field: [20:16] ODPDAY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSERELEASE_ODPDAY_MASK                              0x001f0000
#define FLASH_EFUSERELEASE_ODPDAY_SHIFT                                     16

/* Field:  [15:9] EFUSEYEAR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSERELEASE_EFUSEYEAR_MASK                           0x0000fe00
#define FLASH_EFUSERELEASE_EFUSEYEAR_SHIFT                                   9

/* Field:   [8:5] EFUSEMONTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSERELEASE_EFUSEMONTH_MASK                          0x000001e0
#define FLASH_EFUSERELEASE_EFUSEMONTH_SHIFT                                  5

/* Field:   [4:0] EFUSEDAY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSERELEASE_EFUSEDAY_MASK                            0x0000001f
#define FLASH_EFUSERELEASE_EFUSEDAY_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_EFUSEPINS
 *
 ******************************************************************************
 * Field:    [15] EFC_SELF_TEST_DONE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_SELF_TEST_DONE                          0x00008000
#define FLASH_EFUSEPINS_EFC_SELF_TEST_DONE_MASK                     0x00008000
#define FLASH_EFUSEPINS_EFC_SELF_TEST_DONE_SHIFT                            15

/* Field:    [14] EFC_SELF_TEST_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_SELF_TEST_ERROR                         0x00004000
#define FLASH_EFUSEPINS_EFC_SELF_TEST_ERROR_MASK                    0x00004000
#define FLASH_EFUSEPINS_EFC_SELF_TEST_ERROR_SHIFT                           14

/* Field:    [13] SYS_ECC_SELF_TEST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_SYS_ECC_SELF_TEST_EN                        0x00002000
#define FLASH_EFUSEPINS_SYS_ECC_SELF_TEST_EN_MASK                   0x00002000
#define FLASH_EFUSEPINS_SYS_ECC_SELF_TEST_EN_SHIFT                          13

/* Field:    [12] EFC_INSTRUCTION_INFO
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_INSTRUCTION_INFO                        0x00001000
#define FLASH_EFUSEPINS_EFC_INSTRUCTION_INFO_MASK                   0x00001000
#define FLASH_EFUSEPINS_EFC_INSTRUCTION_INFO_SHIFT                          12

/* Field:    [11] EFC_INSTRUCTION_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_INSTRUCTION_ERROR                       0x00000800
#define FLASH_EFUSEPINS_EFC_INSTRUCTION_ERROR_MASK                  0x00000800
#define FLASH_EFUSEPINS_EFC_INSTRUCTION_ERROR_SHIFT                         11

/* Field:    [10] EFC_AUTOLOAD_ERROR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_AUTOLOAD_ERROR                          0x00000400
#define FLASH_EFUSEPINS_EFC_AUTOLOAD_ERROR_MASK                     0x00000400
#define FLASH_EFUSEPINS_EFC_AUTOLOAD_ERROR_SHIFT                            10

/* Field:     [9] SYS_ECC_OVERRIDE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_SYS_ECC_OVERRIDE_EN                         0x00000200
#define FLASH_EFUSEPINS_SYS_ECC_OVERRIDE_EN_MASK                    0x00000200
#define FLASH_EFUSEPINS_SYS_ECC_OVERRIDE_EN_SHIFT                            9

/* Field:     [8] EFC_READY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_READY                                   0x00000100
#define FLASH_EFUSEPINS_EFC_READY_MASK                              0x00000100
#define FLASH_EFUSEPINS_EFC_READY_SHIFT                                      8

/* Field:     [7] EFC_FCLRZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_EFC_FCLRZ                                   0x00000080
#define FLASH_EFUSEPINS_EFC_FCLRZ_MASK                              0x00000080
#define FLASH_EFUSEPINS_EFC_FCLRZ_SHIFT                                      7

/* Field:     [6] SYS_DIEID_AUTOLOAD_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_SYS_DIEID_AUTOLOAD_EN                       0x00000040
#define FLASH_EFUSEPINS_SYS_DIEID_AUTOLOAD_EN_MASK                  0x00000040
#define FLASH_EFUSEPINS_SYS_DIEID_AUTOLOAD_EN_SHIFT                          6

/* Field:   [5:4] SYS_REPAIR_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_SYS_REPAIR_EN_MASK                          0x00000030
#define FLASH_EFUSEPINS_SYS_REPAIR_EN_SHIFT                                  4

/* Field:   [3:0] SYS_WS_READ_STATES
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPINS_SYS_WS_READ_STATES_MASK                     0x0000000f
#define FLASH_EFUSEPINS_SYS_WS_READ_STATES_SHIFT                             0

/******************************************************************************
 *
 * Register: FLASH_EFUSECRA
 *
 ******************************************************************************
 * Field:   [5:0] DATA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSECRA_DATA_MASK                                    0x0000003f
#define FLASH_EFUSECRA_DATA_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_EFUSEREAD
 *
 ******************************************************************************
 * Field:   [9:8] DATABIT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEREAD_DATABIT_MASK                                0x00000300
#define FLASH_EFUSEREAD_DATABIT_SHIFT                                        8

/* Field:   [7:4] READCLOCK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEREAD_READCLOCK_MASK                              0x000000f0
#define FLASH_EFUSEREAD_READCLOCK_SHIFT                                      4

/* Field:     [3] DEBUG
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEREAD_DEBUG                                       0x00000008
#define FLASH_EFUSEREAD_DEBUG_MASK                                  0x00000008
#define FLASH_EFUSEREAD_DEBUG_SHIFT                                          3

/* Field:     [2] SPARE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEREAD_SPARE                                       0x00000004
#define FLASH_EFUSEREAD_SPARE_MASK                                  0x00000004
#define FLASH_EFUSEREAD_SPARE_SHIFT                                          2

/* Field:   [1:0] MARGIN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEREAD_MARGIN_MASK                                 0x00000003
#define FLASH_EFUSEREAD_MARGIN_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_EFUSEPROGRAM
 *
 ******************************************************************************
 * Field:    [30] COMPAREDISABLE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPROGRAM_COMPAREDISABLE                           0x40000000
#define FLASH_EFUSEPROGRAM_COMPAREDISABLE_MASK                      0x40000000
#define FLASH_EFUSEPROGRAM_COMPAREDISABLE_SHIFT                             30

/* Field: [29:14] CLOCKSTALL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPROGRAM_CLOCKSTALL_MASK                          0x3fffc000
#define FLASH_EFUSEPROGRAM_CLOCKSTALL_SHIFT                                 14

/* Field:    [13] VPPTOVDD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPROGRAM_VPPTOVDD                                 0x00002000
#define FLASH_EFUSEPROGRAM_VPPTOVDD_MASK                            0x00002000
#define FLASH_EFUSEPROGRAM_VPPTOVDD_SHIFT                                   13

/* Field:  [12:9] ITERATIONS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPROGRAM_ITERATIONS_MASK                          0x00001e00
#define FLASH_EFUSEPROGRAM_ITERATIONS_SHIFT                                  9

/* Field:   [8:0] WRITECLOCK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEPROGRAM_WRITECLOCK_MASK                          0x000001ff
#define FLASH_EFUSEPROGRAM_WRITECLOCK_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_EFUSEERROR
 *
 ******************************************************************************
 * Field:     [5] DONE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEERROR_DONE                                       0x00000020
#define FLASH_EFUSEERROR_DONE_MASK                                  0x00000020
#define FLASH_EFUSEERROR_DONE_SHIFT                                          5

/* Field:   [4:0] CODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_EFUSEERROR_CODE_MASK                                  0x0000001f
#define FLASH_EFUSEERROR_CODE_SHIFT                                          0

/******************************************************************************
 *
 * Register: FLASH_SINGLEBIT
 *
 ******************************************************************************
 * Field:  [31:1] FROMN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_SINGLEBIT_FROMN_MASK                                  0xfffffffe
#define FLASH_SINGLEBIT_FROMN_SHIFT                                          1

/* Field:     [0] FROM0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_SINGLEBIT_FROM0                                       0x00000001
#define FLASH_SINGLEBIT_FROM0_MASK                                  0x00000001
#define FLASH_SINGLEBIT_FROM0_SHIFT                                          0

/******************************************************************************
 *
 * Register: FLASH_TWOBIT
 *
 ******************************************************************************
 * Field:  [31:1] FROMN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_TWOBIT_FROMN_MASK                                     0xfffffffe
#define FLASH_TWOBIT_FROMN_SHIFT                                             1

/* Field:     [0] FROM0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_TWOBIT_FROM0                                          0x00000001
#define FLASH_TWOBIT_FROM0_MASK                                     0x00000001
#define FLASH_TWOBIT_FROM0_SHIFT                                             0

/******************************************************************************
 *
 * Register: FLASH_SELFTESTCYC
 *
 ******************************************************************************
 * Field:  [31:0] CYCLES
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_SELFTESTCYC_CYCLES_MASK                               0xffffffff
#define FLASH_SELFTESTCYC_CYCLES_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_SELFTESTSIGN
 *
 ******************************************************************************
 * Field:  [31:0] SIGNATURE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_SELFTESTSIGN_SIGNATURE_MASK                           0xffffffff
#define FLASH_SELFTESTSIGN_SIGNATURE_SHIFT                                   0

/******************************************************************************
 *
 * Register: FLASH_FRDCTL
 *
 ******************************************************************************
 * Field:  [11:8] RWAIT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FRDCTL_RWAIT_MASK                                     0x00000f00
#define FLASH_FRDCTL_RWAIT_SHIFT                                             8

/******************************************************************************
 *
 * Register: FLASH_FSPRD
 *
 ******************************************************************************
 * Field:  [15:8] RMBSEM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSPRD_RMBSEM_MASK                                     0x0000ff00
#define FLASH_FSPRD_RMBSEM_SHIFT                                             8

/* Field:     [1] RM1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSPRD_RM1                                             0x00000002
#define FLASH_FSPRD_RM1_MASK                                        0x00000002
#define FLASH_FSPRD_RM1_SHIFT                                                1

/* Field:     [0] RM0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSPRD_RM0                                             0x00000001
#define FLASH_FSPRD_RM0_MASK                                        0x00000001
#define FLASH_FSPRD_RM0_SHIFT                                                0

/******************************************************************************
 *
 * Register: FLASH_FEDACCTL1
 *
 ******************************************************************************
 * Field:    [24] SUSP_IGNR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEDACCTL1_SUSP_IGNR                                   0x01000000
#define FLASH_FEDACCTL1_SUSP_IGNR_MASK                              0x01000000
#define FLASH_FEDACCTL1_SUSP_IGNR_SHIFT                                     24

/******************************************************************************
 *
 * Register: FLASH_FEDACSTAT
 *
 ******************************************************************************
 * Field:    [25] RVF_INT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEDACSTAT_RVF_INT                                     0x02000000
#define FLASH_FEDACSTAT_RVF_INT_MASK                                0x02000000
#define FLASH_FEDACSTAT_RVF_INT_SHIFT                                       25

/* Field:    [24] FSM_DONE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEDACSTAT_FSM_DONE                                    0x01000000
#define FLASH_FEDACSTAT_FSM_DONE_MASK                               0x01000000
#define FLASH_FEDACSTAT_FSM_DONE_SHIFT                                      24

/******************************************************************************
 *
 * Register: FLASH_FBPROT
 *
 ******************************************************************************
 * Field:     [0] PROTL1DIS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBPROT_PROTL1DIS                                      0x00000001
#define FLASH_FBPROT_PROTL1DIS_MASK                                 0x00000001
#define FLASH_FBPROT_PROTL1DIS_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FBSE
 *
 ******************************************************************************
 * Field:  [15:0] BSE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSE_BSE_MASK                                         0x0000ffff
#define FLASH_FBSE_BSE_SHIFT                                                 0

/******************************************************************************
 *
 * Register: FLASH_FBBUSY
 *
 ******************************************************************************
 * Field:   [7:0] BUSY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBBUSY_BUSY_MASK                                      0x000000ff
#define FLASH_FBBUSY_BUSY_SHIFT                                              0

/******************************************************************************
 *
 * Register: FLASH_FBAC
 *
 ******************************************************************************
 * Field:    [16] OTPPROTDIS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBAC_OTPPROTDIS                                       0x00010000
#define FLASH_FBAC_OTPPROTDIS_MASK                                  0x00010000
#define FLASH_FBAC_OTPPROTDIS_SHIFT                                         16

/* Field:  [15:8] BAGP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBAC_BAGP_MASK                                        0x0000ff00
#define FLASH_FBAC_BAGP_SHIFT                                                8

/* Field:   [7:0] VREADS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBAC_VREADS_MASK                                      0x000000ff
#define FLASH_FBAC_VREADS_SHIFT                                              0

/******************************************************************************
 *
 * Register: FLASH_FBFALLBACK
 *
 ******************************************************************************
 * Field: [27:24] FSM_PWRSAV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_FSM_PWRSAV_MASK                            0x0f000000
#define FLASH_FBFALLBACK_FSM_PWRSAV_SHIFT                                   24

/* Field: [19:16] REG_PWRSAV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_REG_PWRSAV_MASK                            0x000f0000
#define FLASH_FBFALLBACK_REG_PWRSAV_SHIFT                                   16

/* Field: [15:14] BANKPWR7
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR7_MASK                              0x0000c000
#define FLASH_FBFALLBACK_BANKPWR7_SHIFT                                     14

/* Field: [13:12] BANKPWR6
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR6_MASK                              0x00003000
#define FLASH_FBFALLBACK_BANKPWR6_SHIFT                                     12

/* Field: [11:10] BANKPWR5
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR5_MASK                              0x00000c00
#define FLASH_FBFALLBACK_BANKPWR5_SHIFT                                     10

/* Field:   [9:8] BANKPWR4
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR4_MASK                              0x00000300
#define FLASH_FBFALLBACK_BANKPWR4_SHIFT                                      8

/* Field:   [7:6] BANKPWR3
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR3_MASK                              0x000000c0
#define FLASH_FBFALLBACK_BANKPWR3_SHIFT                                      6

/* Field:   [5:4] BANKPWR2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR2_MASK                              0x00000030
#define FLASH_FBFALLBACK_BANKPWR2_SHIFT                                      4

/* Field:   [3:2] BANKPWR1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR1_MASK                              0x0000000c
#define FLASH_FBFALLBACK_BANKPWR1_SHIFT                                      2

/* Field:   [1:0] BANKPWR0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBFALLBACK_BANKPWR0_MASK                              0x00000003
#define FLASH_FBFALLBACK_BANKPWR0_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FBPRDY
 *
 ******************************************************************************
 * Field:    [16] BANKBUSY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBPRDY_BANKBUSY                                       0x00010000
#define FLASH_FBPRDY_BANKBUSY_MASK                                  0x00010000
#define FLASH_FBPRDY_BANKBUSY_SHIFT                                         16

/* Field:    [15] PUMPRDY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBPRDY_PUMPRDY                                        0x00008000
#define FLASH_FBPRDY_PUMPRDY_MASK                                   0x00008000
#define FLASH_FBPRDY_PUMPRDY_SHIFT                                          15

/* Field:     [0] BANKRDY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBPRDY_BANKRDY                                        0x00000001
#define FLASH_FBPRDY_BANKRDY_MASK                                   0x00000001
#define FLASH_FBPRDY_BANKRDY_SHIFT                                           0

/******************************************************************************
 *
 * Register: FLASH_FPAC1
 *
 ******************************************************************************
 * Field: [27:16] PSLEEPTDIS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPAC1_PSLEEPTDIS_MASK                                 0x0fff0000
#define FLASH_FPAC1_PSLEEPTDIS_SHIFT                                        16

/* Field:  [15:4] PUMPRESET_PW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPAC1_PUMPRESET_PW_MASK                               0x0000fff0
#define FLASH_FPAC1_PUMPRESET_PW_SHIFT                                       4

/* Field:   [1:0] PUMPPWR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPAC1_PUMPPWR_MASK                                    0x00000003
#define FLASH_FPAC1_PUMPPWR_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_FPAC2
 *
 ******************************************************************************
 * Field:  [15:0] PAGP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPAC2_PAGP_MASK                                       0x0000ffff
#define FLASH_FPAC2_PAGP_SHIFT                                               0

/******************************************************************************
 *
 * Register: FLASH_FMAC
 *
 ******************************************************************************
 * Field:   [2:0] BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMAC_BANK_MASK                                        0x00000007
#define FLASH_FMAC_BANK_SHIFT                                                0

/******************************************************************************
 *
 * Register: FLASH_FMSTAT
 *
 ******************************************************************************
 * Field:    [17] RVSUSP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_RVSUSP                                         0x00020000
#define FLASH_FMSTAT_RVSUSP_MASK                                    0x00020000
#define FLASH_FMSTAT_RVSUSP_SHIFT                                           17

/* Field:    [16] RDVER
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_RDVER                                          0x00010000
#define FLASH_FMSTAT_RDVER_MASK                                     0x00010000
#define FLASH_FMSTAT_RDVER_SHIFT                                            16

/* Field:    [15] RVF
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_RVF                                            0x00008000
#define FLASH_FMSTAT_RVF_MASK                                       0x00008000
#define FLASH_FMSTAT_RVF_SHIFT                                              15

/* Field:    [14] ILA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_ILA                                            0x00004000
#define FLASH_FMSTAT_ILA_MASK                                       0x00004000
#define FLASH_FMSTAT_ILA_SHIFT                                              14

/* Field:    [13] DBF
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_DBF                                            0x00002000
#define FLASH_FMSTAT_DBF_MASK                                       0x00002000
#define FLASH_FMSTAT_DBF_SHIFT                                              13

/* Field:    [12] PGV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_PGV                                            0x00001000
#define FLASH_FMSTAT_PGV_MASK                                       0x00001000
#define FLASH_FMSTAT_PGV_SHIFT                                              12

/* Field:    [11] PCV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_PCV                                            0x00000800
#define FLASH_FMSTAT_PCV_MASK                                       0x00000800
#define FLASH_FMSTAT_PCV_SHIFT                                              11

/* Field:    [10] EV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_EV                                             0x00000400
#define FLASH_FMSTAT_EV_MASK                                        0x00000400
#define FLASH_FMSTAT_EV_SHIFT                                               10

/* Field:     [9] CV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_CV                                             0x00000200
#define FLASH_FMSTAT_CV_MASK                                        0x00000200
#define FLASH_FMSTAT_CV_SHIFT                                                9

/* Field:     [8] BUSY
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_BUSY                                           0x00000100
#define FLASH_FMSTAT_BUSY_MASK                                      0x00000100
#define FLASH_FMSTAT_BUSY_SHIFT                                              8

/* Field:     [7] ERS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_ERS                                            0x00000080
#define FLASH_FMSTAT_ERS_MASK                                       0x00000080
#define FLASH_FMSTAT_ERS_SHIFT                                               7

/* Field:     [6] PGM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_PGM                                            0x00000040
#define FLASH_FMSTAT_PGM_MASK                                       0x00000040
#define FLASH_FMSTAT_PGM_SHIFT                                               6

/* Field:     [5] INVDAT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_INVDAT                                         0x00000020
#define FLASH_FMSTAT_INVDAT_MASK                                    0x00000020
#define FLASH_FMSTAT_INVDAT_SHIFT                                            5

/* Field:     [4] CSTAT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_CSTAT                                          0x00000010
#define FLASH_FMSTAT_CSTAT_MASK                                     0x00000010
#define FLASH_FMSTAT_CSTAT_SHIFT                                             4

/* Field:     [3] VOLSTAT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_VOLSTAT                                        0x00000008
#define FLASH_FMSTAT_VOLSTAT_MASK                                   0x00000008
#define FLASH_FMSTAT_VOLSTAT_SHIFT                                           3

/* Field:     [2] ESUSP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_ESUSP                                          0x00000004
#define FLASH_FMSTAT_ESUSP_MASK                                     0x00000004
#define FLASH_FMSTAT_ESUSP_SHIFT                                             2

/* Field:     [1] PSUSP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_PSUSP                                          0x00000002
#define FLASH_FMSTAT_PSUSP_MASK                                     0x00000002
#define FLASH_FMSTAT_PSUSP_SHIFT                                             1

/* Field:     [0] SLOCK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMSTAT_SLOCK                                          0x00000001
#define FLASH_FMSTAT_SLOCK_MASK                                     0x00000001
#define FLASH_FMSTAT_SLOCK_SHIFT                                             0

/******************************************************************************
 *
 * Register: FLASH_FLOCK
 *
 ******************************************************************************
 * Field:  [15:0] ENCOM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FLOCK_ENCOM_MASK                                      0x0000ffff
#define FLASH_FLOCK_ENCOM_SHIFT                                              0

/******************************************************************************
 *
 * Register: FLASH_FVREADCT
 *
 ******************************************************************************
 * Field:   [3:0] VREADCT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVREADCT_VREADCT_MASK                                 0x0000000f
#define FLASH_FVREADCT_VREADCT_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FVHVCT1
 *
 ******************************************************************************
 * Field: [23:20] TRIM13_E
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT1_TRIM13_E_MASK                                 0x00f00000
#define FLASH_FVHVCT1_TRIM13_E_SHIFT                                        20

/* Field: [19:16] VHVCT_E
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT1_VHVCT_E_MASK                                  0x000f0000
#define FLASH_FVHVCT1_VHVCT_E_SHIFT                                         16

/* Field:   [7:4] TRIM13_PV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT1_TRIM13_PV_MASK                                0x000000f0
#define FLASH_FVHVCT1_TRIM13_PV_SHIFT                                        4

/* Field:   [3:0] VHVCT_PV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT1_VHVCT_PV_MASK                                 0x0000000f
#define FLASH_FVHVCT1_VHVCT_PV_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FVHVCT2
 *
 ******************************************************************************
 * Field: [23:20] TRIM13_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT2_TRIM13_P_MASK                                 0x00f00000
#define FLASH_FVHVCT2_TRIM13_P_SHIFT                                        20

/* Field: [19:16] VHVCT_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT2_VHVCT_P_MASK                                  0x000f0000
#define FLASH_FVHVCT2_VHVCT_P_SHIFT                                         16

/******************************************************************************
 *
 * Register: FLASH_FVHVCT3
 *
 ******************************************************************************
 * Field: [19:16] WCT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT3_WCT_MASK                                      0x000f0000
#define FLASH_FVHVCT3_WCT_SHIFT                                             16

/* Field:   [3:0] VHVCT_READ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVHVCT3_VHVCT_READ_MASK                               0x0000000f
#define FLASH_FVHVCT3_VHVCT_READ_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FVNVCT
 *
 ******************************************************************************
 * Field:  [12:8] VCG2P5CT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVNVCT_VCG2P5CT_MASK                                  0x00001f00
#define FLASH_FVNVCT_VCG2P5CT_SHIFT                                          8

/* Field:   [4:0] VIN_CT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVNVCT_VIN_CT_MASK                                    0x0000001f
#define FLASH_FVNVCT_VIN_CT_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_FVSLP
 *
 ******************************************************************************
 * Field: [15:12] VSL_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVSLP_VSL_P_MASK                                      0x0000f000
#define FLASH_FVSLP_VSL_P_SHIFT                                             12

/******************************************************************************
 *
 * Register: FLASH_FVWLCT
 *
 ******************************************************************************
 * Field:   [4:0] VWLCT_P
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FVWLCT_VWLCT_P_MASK                                   0x0000001f
#define FLASH_FVWLCT_VWLCT_P_SHIFT                                           0

/******************************************************************************
 *
 * Register: FLASH_FEFUSECTL
 *
 ******************************************************************************
 * Field: [26:24] CHAIN_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSECTL_CHAIN_SEL_MASK                              0x07000000
#define FLASH_FEFUSECTL_CHAIN_SEL_SHIFT                                     24

/* Field:    [17] WRITE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSECTL_WRITE_EN                                    0x00020000
#define FLASH_FEFUSECTL_WRITE_EN_MASK                               0x00020000
#define FLASH_FEFUSECTL_WRITE_EN_SHIFT                                      17

/* Field:    [16] BP_SEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSECTL_BP_SEL                                      0x00010000
#define FLASH_FEFUSECTL_BP_SEL_MASK                                 0x00010000
#define FLASH_FEFUSECTL_BP_SEL_SHIFT                                        16

/* Field:     [8] EF_CLRZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSECTL_EF_CLRZ                                     0x00000100
#define FLASH_FEFUSECTL_EF_CLRZ_MASK                                0x00000100
#define FLASH_FEFUSECTL_EF_CLRZ_SHIFT                                        8

/* Field:     [4] EF_TEST
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSECTL_EF_TEST                                     0x00000010
#define FLASH_FEFUSECTL_EF_TEST_MASK                                0x00000010
#define FLASH_FEFUSECTL_EF_TEST_SHIFT                                        4

/* Field:   [3:0] EFUSE_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSECTL_EFUSE_EN_MASK                               0x0000000f
#define FLASH_FEFUSECTL_EFUSE_EN_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FEFUSESTAT
 *
 ******************************************************************************
 * Field:     [0] SHIFT_DONE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSESTAT_SHIFT_DONE                                 0x00000001
#define FLASH_FEFUSESTAT_SHIFT_DONE_MASK                            0x00000001
#define FLASH_FEFUSESTAT_SHIFT_DONE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FEFUSEDATA
 *
 ******************************************************************************
 * Field:  [31:0] FEFUSEDATA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FEFUSEDATA_FEFUSEDATA_MASK                            0xffffffff
#define FLASH_FEFUSEDATA_FEFUSEDATA_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FSEQPMP
 *
 ******************************************************************************
 * Field: [27:24] TRIM_3P4
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSEQPMP_TRIM_3P4_MASK                                 0x0f000000
#define FLASH_FSEQPMP_TRIM_3P4_SHIFT                                        24

/* Field: [21:20] TRIM_1P7
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSEQPMP_TRIM_1P7_MASK                                 0x00300000
#define FLASH_FSEQPMP_TRIM_1P7_SHIFT                                        20

/* Field: [19:16] TRIM_0P8
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSEQPMP_TRIM_0P8_MASK                                 0x000f0000
#define FLASH_FSEQPMP_TRIM_0P8_SHIFT                                        16

/* Field: [14:12] VIN_AT_X
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSEQPMP_VIN_AT_X_MASK                                 0x00007000
#define FLASH_FSEQPMP_VIN_AT_X_SHIFT                                        12

/* Field:     [8] VIN_BY_PASS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSEQPMP_VIN_BY_PASS                                   0x00000100
#define FLASH_FSEQPMP_VIN_BY_PASS_MASK                              0x00000100
#define FLASH_FSEQPMP_VIN_BY_PASS_SHIFT                                      8

/******************************************************************************
 *
 * Register: FLASH_FBSTROBES
 *
 ******************************************************************************
 * Field:    [24] ECBIT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_ECBIT                                       0x01000000
#define FLASH_FBSTROBES_ECBIT_MASK                                  0x01000000
#define FLASH_FBSTROBES_ECBIT_SHIFT                                         24

/* Field:    [18] RWAIT2_FLCLK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_RWAIT2_FLCLK                                0x00040000
#define FLASH_FBSTROBES_RWAIT2_FLCLK_MASK                           0x00040000
#define FLASH_FBSTROBES_RWAIT2_FLCLK_SHIFT                                  18

/* Field:    [17] RWAIT_FLCLK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_RWAIT_FLCLK                                 0x00020000
#define FLASH_FBSTROBES_RWAIT_FLCLK_MASK                            0x00020000
#define FLASH_FBSTROBES_RWAIT_FLCLK_SHIFT                                   17

/* Field:    [16] FLCLKEN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_FLCLKEN                                     0x00010000
#define FLASH_FBSTROBES_FLCLKEN_MASK                                0x00010000
#define FLASH_FBSTROBES_FLCLKEN_SHIFT                                       16

/* Field:     [8] CTRLENZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_CTRLENZ                                     0x00000100
#define FLASH_FBSTROBES_CTRLENZ_MASK                                0x00000100
#define FLASH_FBSTROBES_CTRLENZ_SHIFT                                        8

/* Field:     [6] NOCOLRED
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_NOCOLRED                                    0x00000040
#define FLASH_FBSTROBES_NOCOLRED_MASK                               0x00000040
#define FLASH_FBSTROBES_NOCOLRED_SHIFT                                       6

/* Field:     [5] PRECOL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_PRECOL                                      0x00000020
#define FLASH_FBSTROBES_PRECOL_MASK                                 0x00000020
#define FLASH_FBSTROBES_PRECOL_SHIFT                                         5

/* Field:     [4] TI_OTP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_TI_OTP                                      0x00000010
#define FLASH_FBSTROBES_TI_OTP_MASK                                 0x00000010
#define FLASH_FBSTROBES_TI_OTP_SHIFT                                         4

/* Field:     [3] OTP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_OTP                                         0x00000008
#define FLASH_FBSTROBES_OTP_MASK                                    0x00000008
#define FLASH_FBSTROBES_OTP_SHIFT                                            3

/* Field:     [2] TEZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBSTROBES_TEZ                                         0x00000004
#define FLASH_FBSTROBES_TEZ_MASK                                    0x00000004
#define FLASH_FBSTROBES_TEZ_SHIFT                                            2

/******************************************************************************
 *
 * Register: FLASH_FPSTROBES
 *
 ******************************************************************************
 * Field:     [8] EXECUTEZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPSTROBES_EXECUTEZ                                    0x00000100
#define FLASH_FPSTROBES_EXECUTEZ_MASK                               0x00000100
#define FLASH_FPSTROBES_EXECUTEZ_SHIFT                                       8

/* Field:     [1] V3PWRDNZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPSTROBES_V3PWRDNZ                                    0x00000002
#define FLASH_FPSTROBES_V3PWRDNZ_MASK                               0x00000002
#define FLASH_FPSTROBES_V3PWRDNZ_SHIFT                                       1

/* Field:     [0] V5PWRDNZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FPSTROBES_V5PWRDNZ                                    0x00000001
#define FLASH_FPSTROBES_V5PWRDNZ_MASK                               0x00000001
#define FLASH_FPSTROBES_V5PWRDNZ_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FBMODE
 *
 ******************************************************************************
 * Field:   [2:0] MODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FBMODE_MODE_MASK                                      0x00000007
#define FLASH_FBMODE_MODE_SHIFT                                              0

/******************************************************************************
 *
 * Register: FLASH_FTCR
 *
 ******************************************************************************
 * Field:   [6:0] TCR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FTCR_TCR_MASK                                         0x0000007f
#define FLASH_FTCR_TCR_SHIFT                                                 0

/******************************************************************************
 *
 * Register: FLASH_FADDR
 *
 ******************************************************************************
 * Field:  [31:0] FADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FADDR_FADDR_MASK                                      0xffffffff
#define FLASH_FADDR_FADDR_SHIFT                                              0

/******************************************************************************
 *
 * Register: FLASH_FTCTL
 *
 ******************************************************************************
 * Field:    [16] WDATA_BLK_CLR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FTCTL_WDATA_BLK_CLR                                   0x00010000
#define FLASH_FTCTL_WDATA_BLK_CLR_MASK                              0x00010000
#define FLASH_FTCTL_WDATA_BLK_CLR_SHIFT                                     16

/* Field:     [1] TEST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FTCTL_TEST_EN                                         0x00000002
#define FLASH_FTCTL_TEST_EN_MASK                                    0x00000002
#define FLASH_FTCTL_TEST_EN_SHIFT                                            1

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE0
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE0_FWPWRITE0_MASK                              0xffffffff
#define FLASH_FWPWRITE0_FWPWRITE0_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE1
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE1_FWPWRITE1_MASK                              0xffffffff
#define FLASH_FWPWRITE1_FWPWRITE1_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE2
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE2_FWPWRITE2_MASK                              0xffffffff
#define FLASH_FWPWRITE2_FWPWRITE2_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE3
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE3
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE3_FWPWRITE3_MASK                              0xffffffff
#define FLASH_FWPWRITE3_FWPWRITE3_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE4
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE4
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE4_FWPWRITE4_MASK                              0xffffffff
#define FLASH_FWPWRITE4_FWPWRITE4_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE5
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE5
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE5_FWPWRITE5_MASK                              0xffffffff
#define FLASH_FWPWRITE5_FWPWRITE5_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE6
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE6
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE6_FWPWRITE6_MASK                              0xffffffff
#define FLASH_FWPWRITE6_FWPWRITE6_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE7
 *
 ******************************************************************************
 * Field:  [31:0] FWPWRITE7
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE7_FWPWRITE7_MASK                              0xffffffff
#define FLASH_FWPWRITE7_FWPWRITE7_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FWPWRITE_ECC
 *
 ******************************************************************************
 * Field: [31:24] ECCBYTES07_00
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE_ECC_ECCBYTES07_00_MASK                       0xff000000
#define FLASH_FWPWRITE_ECC_ECCBYTES07_00_SHIFT                              24

/* Field: [23:16] ECCBYTES15_08
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE_ECC_ECCBYTES15_08_MASK                       0x00ff0000
#define FLASH_FWPWRITE_ECC_ECCBYTES15_08_SHIFT                              16

/* Field:  [15:8] ECCBYTES23_16
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE_ECC_ECCBYTES23_16_MASK                       0x0000ff00
#define FLASH_FWPWRITE_ECC_ECCBYTES23_16_SHIFT                               8

/* Field:   [7:0] ECCBYTES31_24
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FWPWRITE_ECC_ECCBYTES31_24_MASK                       0x000000ff
#define FLASH_FWPWRITE_ECC_ECCBYTES31_24_SHIFT                               0

/******************************************************************************
 *
 * Register: FLASH_FSWSTAT
 *
 ******************************************************************************
 * Field:     [0] SAFELV
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSWSTAT_SAFELV                                        0x00000001
#define FLASH_FSWSTAT_SAFELV_MASK                                   0x00000001
#define FLASH_FSWSTAT_SAFELV_SHIFT                                           0

/******************************************************************************
 *
 * Register: FLASH_FSM_GLBCTL
 *
 ******************************************************************************
 * Field:     [0] CLKSEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_GLBCTL_CLKSEL                                     0x00000001
#define FLASH_FSM_GLBCTL_CLKSEL_MASK                                0x00000001
#define FLASH_FSM_GLBCTL_CLKSEL_SHIFT                                        0

/******************************************************************************
 *
 * Register: FLASH_FSM_STATE
 *
 ******************************************************************************
 * Field:    [11] CTRLENZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STATE_CTRLENZ                                     0x00000800
#define FLASH_FSM_STATE_CTRLENZ_MASK                                0x00000800
#define FLASH_FSM_STATE_CTRLENZ_SHIFT                                       11

/* Field:    [10] EXECUTEZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STATE_EXECUTEZ                                    0x00000400
#define FLASH_FSM_STATE_EXECUTEZ_MASK                               0x00000400
#define FLASH_FSM_STATE_EXECUTEZ_SHIFT                                      10

/* Field:     [8] FSM_ACT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STATE_FSM_ACT                                     0x00000100
#define FLASH_FSM_STATE_FSM_ACT_MASK                                0x00000100
#define FLASH_FSM_STATE_FSM_ACT_SHIFT                                        8

/* Field:     [7] TIOTP_ACT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STATE_TIOTP_ACT                                   0x00000080
#define FLASH_FSM_STATE_TIOTP_ACT_MASK                              0x00000080
#define FLASH_FSM_STATE_TIOTP_ACT_SHIFT                                      7

/* Field:     [6] OTP_ACT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STATE_OTP_ACT                                     0x00000040
#define FLASH_FSM_STATE_OTP_ACT_MASK                                0x00000040
#define FLASH_FSM_STATE_OTP_ACT_SHIFT                                        6

/******************************************************************************
 *
 * Register: FLASH_FSM_STAT
 *
 ******************************************************************************
 * Field:     [2] NON_OP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STAT_NON_OP                                       0x00000004
#define FLASH_FSM_STAT_NON_OP_MASK                                  0x00000004
#define FLASH_FSM_STAT_NON_OP_SHIFT                                          2

/* Field:     [1] OVR_PUL_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STAT_OVR_PUL_CNT                                  0x00000002
#define FLASH_FSM_STAT_OVR_PUL_CNT_MASK                             0x00000002
#define FLASH_FSM_STAT_OVR_PUL_CNT_SHIFT                                     1

/* Field:     [0] INV_DAT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STAT_INV_DAT                                      0x00000001
#define FLASH_FSM_STAT_INV_DAT_MASK                                 0x00000001
#define FLASH_FSM_STAT_INV_DAT_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FSM_CMD
 *
 ******************************************************************************
 * Field:   [5:0] FSMCMD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_CMD_FSMCMD_MASK                                   0x0000003f
#define FLASH_FSM_CMD_FSMCMD_SHIFT                                           0

/******************************************************************************
 *
 * Register: FLASH_FSM_PE_OSU
 *
 ******************************************************************************
 * Field:  [15:8] PGM_OSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PE_OSU_PGM_OSU_MASK                               0x0000ff00
#define FLASH_FSM_PE_OSU_PGM_OSU_SHIFT                                       8

/* Field:   [7:0] ERA_OSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PE_OSU_ERA_OSU_MASK                               0x000000ff
#define FLASH_FSM_PE_OSU_ERA_OSU_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FSM_VSTAT
 *
 ******************************************************************************
 * Field: [15:12] VSTAT_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_VSTAT_VSTAT_CNT_MASK                              0x0000f000
#define FLASH_FSM_VSTAT_VSTAT_CNT_SHIFT                                     12

/******************************************************************************
 *
 * Register: FLASH_FSM_PE_VSU
 *
 ******************************************************************************
 * Field:  [15:8] PGM_VSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PE_VSU_PGM_VSU_MASK                               0x0000ff00
#define FLASH_FSM_PE_VSU_PGM_VSU_SHIFT                                       8

/* Field:   [7:0] ERA_VSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PE_VSU_ERA_VSU_MASK                               0x000000ff
#define FLASH_FSM_PE_VSU_ERA_VSU_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FSM_CMP_VSU
 *
 ******************************************************************************
 * Field: [15:12] ADD_EXZ
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_CMP_VSU_ADD_EXZ_MASK                              0x0000f000
#define FLASH_FSM_CMP_VSU_ADD_EXZ_SHIFT                                     12

/******************************************************************************
 *
 * Register: FLASH_FSM_EX_VAL
 *
 ******************************************************************************
 * Field:  [15:8] REP_VSU
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_EX_VAL_REP_VSU_MASK                               0x0000ff00
#define FLASH_FSM_EX_VAL_REP_VSU_SHIFT                                       8

/* Field:   [7:0] EXE_VALD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_EX_VAL_EXE_VALD_MASK                              0x000000ff
#define FLASH_FSM_EX_VAL_EXE_VALD_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FSM_RD_H
 *
 ******************************************************************************
 * Field:   [7:0] RD_H
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_RD_H_RD_H_MASK                                    0x000000ff
#define FLASH_FSM_RD_H_RD_H_SHIFT                                            0

/******************************************************************************
 *
 * Register: FLASH_FSM_P_OH
 *
 ******************************************************************************
 * Field:  [15:8] PGM_OH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_P_OH_PGM_OH_MASK                                  0x0000ff00
#define FLASH_FSM_P_OH_PGM_OH_SHIFT                                          8

/******************************************************************************
 *
 * Register: FLASH_FSM_ERA_OH
 *
 ******************************************************************************
 * Field:  [15:0] ERA_OH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERA_OH_ERA_OH_MASK                                0x0000ffff
#define FLASH_FSM_ERA_OH_ERA_OH_SHIFT                                        0

/******************************************************************************
 *
 * Register: FLASH_FSM_SAV_PPUL
 *
 ******************************************************************************
 * Field:  [11:0] SAV_P_PUL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SAV_PPUL_SAV_P_PUL_MASK                           0x00000fff
#define FLASH_FSM_SAV_PPUL_SAV_P_PUL_SHIFT                                   0

/******************************************************************************
 *
 * Register: FLASH_FSM_PE_VH
 *
 ******************************************************************************
 * Field:  [15:8] PGM_VH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PE_VH_PGM_VH_MASK                                 0x0000ff00
#define FLASH_FSM_PE_VH_PGM_VH_SHIFT                                         8

/******************************************************************************
 *
 * Register: FLASH_FSM_PRG_PW
 *
 ******************************************************************************
 * Field:  [15:0] PROG_PUL_WIDTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_MASK                        0x0000ffff
#define FLASH_FSM_PRG_PW_PROG_PUL_WIDTH_SHIFT                                0

/******************************************************************************
 *
 * Register: FLASH_FSM_ERA_PW
 *
 ******************************************************************************
 * Field:  [31:0] FSM_ERA_PW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERA_PW_FSM_ERA_PW_MASK                            0xffffffff
#define FLASH_FSM_ERA_PW_FSM_ERA_PW_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FSM_SAV_ERA_PUL
 *
 ******************************************************************************
 * Field:  [11:0] SAV_ERA_PUL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SAV_ERA_PUL_SAV_ERA_PUL_MASK                      0x00000fff
#define FLASH_FSM_SAV_ERA_PUL_SAV_ERA_PUL_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FSM_TIMER
 *
 ******************************************************************************
 * Field:  [31:0] FSM_TIMER
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_TIMER_FSM_TIMER_MASK                              0xffffffff
#define FLASH_FSM_TIMER_FSM_TIMER_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FSM_MODE
 *
 ******************************************************************************
 * Field: [19:18] RDV_SUBMODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_RDV_SUBMODE_MASK                             0x000c0000
#define FLASH_FSM_MODE_RDV_SUBMODE_SHIFT                                    18

/* Field: [17:16] PGM_SUBMODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_PGM_SUBMODE_MASK                             0x00030000
#define FLASH_FSM_MODE_PGM_SUBMODE_SHIFT                                    16

/* Field: [15:14] ERA_SUBMODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_ERA_SUBMODE_MASK                             0x0000c000
#define FLASH_FSM_MODE_ERA_SUBMODE_SHIFT                                    14

/* Field: [13:12] SUBMODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_SUBMODE_MASK                                 0x00003000
#define FLASH_FSM_MODE_SUBMODE_SHIFT                                        12

/* Field:  [11:9] SAV_PGM_CMD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_SAV_PGM_CMD_MASK                             0x00000e00
#define FLASH_FSM_MODE_SAV_PGM_CMD_SHIFT                                     9

/* Field:   [8:6] SAV_ERA_MODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_SAV_ERA_MODE_MASK                            0x000001c0
#define FLASH_FSM_MODE_SAV_ERA_MODE_SHIFT                                    6

/* Field:   [5:3] MODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_MODE_MASK                                    0x00000038
#define FLASH_FSM_MODE_MODE_SHIFT                                            3

/* Field:   [2:0] CMD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_MODE_CMD_MASK                                     0x00000007
#define FLASH_FSM_MODE_CMD_SHIFT                                             0

/******************************************************************************
 *
 * Register: FLASH_FSM_PGM
 *
 ******************************************************************************
 * Field: [25:23] PGM_BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PGM_PGM_BANK_MASK                                 0x03800000
#define FLASH_FSM_PGM_PGM_BANK_SHIFT                                        23

/* Field:  [22:0] PGM_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PGM_PGM_ADDR_MASK                                 0x007fffff
#define FLASH_FSM_PGM_PGM_ADDR_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FSM_ERA
 *
 ******************************************************************************
 * Field: [25:23] ERA_BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERA_ERA_BANK_MASK                                 0x03800000
#define FLASH_FSM_ERA_ERA_BANK_SHIFT                                        23

/* Field:  [22:0] ERA_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERA_ERA_ADDR_MASK                                 0x007fffff
#define FLASH_FSM_ERA_ERA_ADDR_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FSM_PRG_PUL
 *
 ******************************************************************************
 * Field: [19:16] BEG_EC_LEVEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_MASK                         0x000f0000
#define FLASH_FSM_PRG_PUL_BEG_EC_LEVEL_SHIFT                                16

/* Field:  [11:0] MAX_PRG_PUL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PRG_PUL_MAX_PRG_PUL_MASK                          0x00000fff
#define FLASH_FSM_PRG_PUL_MAX_PRG_PUL_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_FSM_ERA_PUL
 *
 ******************************************************************************
 * Field: [19:16] MAX_EC_LEVEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_MASK                         0x000f0000
#define FLASH_FSM_ERA_PUL_MAX_EC_LEVEL_SHIFT                                16

/* Field:  [11:0] MAX_ERA_PUL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERA_PUL_MAX_ERA_PUL_MASK                          0x00000fff
#define FLASH_FSM_ERA_PUL_MAX_ERA_PUL_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_FSM_STEP_SIZE
 *
 ******************************************************************************
 * Field: [24:16] EC_STEP_SIZE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_MASK                       0x01ff0000
#define FLASH_FSM_STEP_SIZE_EC_STEP_SIZE_SHIFT                              16

/******************************************************************************
 *
 * Register: FLASH_FSM_PUL_CNTR
 *
 ******************************************************************************
 * Field: [24:16] CUR_EC_LEVEL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PUL_CNTR_CUR_EC_LEVEL_MASK                        0x01ff0000
#define FLASH_FSM_PUL_CNTR_CUR_EC_LEVEL_SHIFT                               16

/* Field:  [11:0] PUL_CNTR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PUL_CNTR_PUL_CNTR_MASK                            0x00000fff
#define FLASH_FSM_PUL_CNTR_PUL_CNTR_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FSM_EC_STEP_HEIGHT
 *
 ******************************************************************************
 * Field:   [3:0] EC_STEP_HEIGHT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_EC_STEP_HEIGHT_EC_STEP_HEIGHT_MASK                0x0000000f
#define FLASH_FSM_EC_STEP_HEIGHT_EC_STEP_HEIGHT_SHIFT                        0

/******************************************************************************
 *
 * Register: FLASH_FSM_ST_MACHINE
 *
 ******************************************************************************
 * Field:    [23] DO_PRECOND
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_DO_PRECOND                             0x00800000
#define FLASH_FSM_ST_MACHINE_DO_PRECOND_MASK                        0x00800000
#define FLASH_FSM_ST_MACHINE_DO_PRECOND_SHIFT                               23

/* Field:    [22] FSM_INT_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_FSM_INT_EN                             0x00400000
#define FLASH_FSM_ST_MACHINE_FSM_INT_EN_MASK                        0x00400000
#define FLASH_FSM_ST_MACHINE_FSM_INT_EN_SHIFT                               22

/* Field:    [21] ALL_BANKS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_ALL_BANKS                              0x00200000
#define FLASH_FSM_ST_MACHINE_ALL_BANKS_MASK                         0x00200000
#define FLASH_FSM_ST_MACHINE_ALL_BANKS_SHIFT                                21

/* Field:    [20] CMPV_ALLOWED
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_CMPV_ALLOWED                           0x00100000
#define FLASH_FSM_ST_MACHINE_CMPV_ALLOWED_MASK                      0x00100000
#define FLASH_FSM_ST_MACHINE_CMPV_ALLOWED_SHIFT                             20

/* Field:    [19] RANDOM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_RANDOM                                 0x00080000
#define FLASH_FSM_ST_MACHINE_RANDOM_MASK                            0x00080000
#define FLASH_FSM_ST_MACHINE_RANDOM_SHIFT                                   19

/* Field:    [18] RV_SEC_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_RV_SEC_EN                              0x00040000
#define FLASH_FSM_ST_MACHINE_RV_SEC_EN_MASK                         0x00040000
#define FLASH_FSM_ST_MACHINE_RV_SEC_EN_SHIFT                                18

/* Field:    [17] RV_RES
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_RV_RES                                 0x00020000
#define FLASH_FSM_ST_MACHINE_RV_RES_MASK                            0x00020000
#define FLASH_FSM_ST_MACHINE_RV_RES_SHIFT                                   17

/* Field:    [16] RV_INT_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_RV_INT_EN                              0x00010000
#define FLASH_FSM_ST_MACHINE_RV_INT_EN_MASK                         0x00010000
#define FLASH_FSM_ST_MACHINE_RV_INT_EN_SHIFT                                16

/* Field:    [14] ONE_TIME_GOOD
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_ONE_TIME_GOOD                          0x00004000
#define FLASH_FSM_ST_MACHINE_ONE_TIME_GOOD_MASK                     0x00004000
#define FLASH_FSM_ST_MACHINE_ONE_TIME_GOOD_SHIFT                            14

/* Field:    [11] DO_REDU_COL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_DO_REDU_COL                            0x00000800
#define FLASH_FSM_ST_MACHINE_DO_REDU_COL_MASK                       0x00000800
#define FLASH_FSM_ST_MACHINE_DO_REDU_COL_SHIFT                              11

/* Field:  [10:7] DBG_SHORT_ROW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_DBG_SHORT_ROW_MASK                     0x00000780
#define FLASH_FSM_ST_MACHINE_DBG_SHORT_ROW_SHIFT                             7

/* Field:     [5] PGM_SEC_COF_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_PGM_SEC_COF_EN                         0x00000020
#define FLASH_FSM_ST_MACHINE_PGM_SEC_COF_EN_MASK                    0x00000020
#define FLASH_FSM_ST_MACHINE_PGM_SEC_COF_EN_SHIFT                            5

/* Field:     [4] PREC_STOP_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_PREC_STOP_EN                           0x00000010
#define FLASH_FSM_ST_MACHINE_PREC_STOP_EN_MASK                      0x00000010
#define FLASH_FSM_ST_MACHINE_PREC_STOP_EN_SHIFT                              4

/* Field:     [3] DIS_TST_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_DIS_TST_EN                             0x00000008
#define FLASH_FSM_ST_MACHINE_DIS_TST_EN_MASK                        0x00000008
#define FLASH_FSM_ST_MACHINE_DIS_TST_EN_SHIFT                                3

/* Field:     [2] CMD_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_CMD_EN                                 0x00000004
#define FLASH_FSM_ST_MACHINE_CMD_EN_MASK                            0x00000004
#define FLASH_FSM_ST_MACHINE_CMD_EN_SHIFT                                    2

/* Field:     [1] INV_DATA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_INV_DATA                               0x00000002
#define FLASH_FSM_ST_MACHINE_INV_DATA_MASK                          0x00000002
#define FLASH_FSM_ST_MACHINE_INV_DATA_SHIFT                                  1

/* Field:     [0] OVERRIDE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ST_MACHINE_OVERRIDE                               0x00000001
#define FLASH_FSM_ST_MACHINE_OVERRIDE_MASK                          0x00000001
#define FLASH_FSM_ST_MACHINE_OVERRIDE_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_FSM_FLES
 *
 ******************************************************************************
 * Field:  [11:8] BLK_TIOTP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_FLES_BLK_TIOTP_MASK                               0x00000f00
#define FLASH_FSM_FLES_BLK_TIOTP_SHIFT                                       8

/* Field:   [7:0] BLK_OTP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_FLES_BLK_OTP_MASK                                 0x000000ff
#define FLASH_FSM_FLES_BLK_OTP_SHIFT                                         0

/******************************************************************************
 *
 * Register: FLASH_FSM_WR_ENA
 *
 ******************************************************************************
 * Field:   [2:0] WR_ENA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_WR_ENA_WR_ENA_MASK                                0x00000007
#define FLASH_FSM_WR_ENA_WR_ENA_SHIFT                                        0

/******************************************************************************
 *
 * Register: FLASH_FSM_ACC_PP
 *
 ******************************************************************************
 * Field:  [31:0] FSM_ACC_PP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ACC_PP_FSM_ACC_PP_MASK                            0xffffffff
#define FLASH_FSM_ACC_PP_FSM_ACC_PP_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FSM_ACC_EP
 *
 ******************************************************************************
 * Field:  [15:0] ACC_EP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ACC_EP_ACC_EP_MASK                                0x0000ffff
#define FLASH_FSM_ACC_EP_ACC_EP_SHIFT                                        0

/******************************************************************************
 *
 * Register: FLASH_FSM_ADDR
 *
 ******************************************************************************
 * Field: [30:28] BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ADDR_BANK_MASK                                    0x70000000
#define FLASH_FSM_ADDR_BANK_SHIFT                                           28

/* Field:  [27:0] CUR_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ADDR_CUR_ADDR_MASK                                0x0fffffff
#define FLASH_FSM_ADDR_CUR_ADDR_SHIFT                                        0

/******************************************************************************
 *
 * Register: FLASH_FSM_SECTOR
 *
 ******************************************************************************
 * Field: [31:16] SECT_ERASED
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SECTOR_SECT_ERASED_MASK                           0xffff0000
#define FLASH_FSM_SECTOR_SECT_ERASED_SHIFT                                  16

/* Field:  [15:8] FSM_SECTOR_EXTENSION
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SECTOR_FSM_SECTOR_EXTENSION_MASK                  0x0000ff00
#define FLASH_FSM_SECTOR_FSM_SECTOR_EXTENSION_SHIFT                          8

/* Field:   [7:4] SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SECTOR_SECTOR_MASK                                0x000000f0
#define FLASH_FSM_SECTOR_SECTOR_SHIFT                                        4

/* Field:   [3:0] SEC_OUT
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SECTOR_SEC_OUT_MASK                               0x0000000f
#define FLASH_FSM_SECTOR_SEC_OUT_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FMC_REV_ID
 *
 ******************************************************************************
 * Field: [31:12] MOD_VERSION
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMC_REV_ID_MOD_VERSION_MASK                           0xfffff000
#define FLASH_FMC_REV_ID_MOD_VERSION_SHIFT                                  12

/* Field:  [11:0] CONFIG_CRC
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FMC_REV_ID_CONFIG_CRC_MASK                            0x00000fff
#define FLASH_FMC_REV_ID_CONFIG_CRC_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FSM_ERR_ADDR
 *
 ******************************************************************************
 * Field:  [31:8] FSM_ERR_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERR_ADDR_FSM_ERR_ADDR_MASK                        0xffffff00
#define FLASH_FSM_ERR_ADDR_FSM_ERR_ADDR_SHIFT                                8

/* Field:   [3:0] FSM_ERR_BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_ERR_ADDR_FSM_ERR_BANK_MASK                        0x0000000f
#define FLASH_FSM_ERR_ADDR_FSM_ERR_BANK_SHIFT                                0

/******************************************************************************
 *
 * Register: FLASH_FSM_PGM_MAXPUL
 *
 ******************************************************************************
 * Field:  [11:0] FSM_PGM_MAXPUL
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_PGM_MAXPUL_FSM_PGM_MAXPUL_MASK                    0x00000fff
#define FLASH_FSM_PGM_MAXPUL_FSM_PGM_MAXPUL_SHIFT                            0

/******************************************************************************
 *
 * Register: FLASH_FSM_EXECUTE
 *
 ******************************************************************************
 * Field: [19:16] SUSPEND_NOW
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_EXECUTE_SUSPEND_NOW_MASK                          0x000f0000
#define FLASH_FSM_EXECUTE_SUSPEND_NOW_SHIFT                                 16

/* Field:   [4:0] FSMEXECUTE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_EXECUTE_FSMEXECUTE_MASK                           0x0000001f
#define FLASH_FSM_EXECUTE_FSMEXECUTE_SHIFT                                   0

/******************************************************************************
 *
 * Register: FLASH_FSM_SECTOR1
 *
 ******************************************************************************
 * Field:  [31:0] FSM_SECTOR1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SECTOR1_FSM_SECTOR1_MASK                          0xffffffff
#define FLASH_FSM_SECTOR1_FSM_SECTOR1_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_FSM_SECTOR2
 *
 ******************************************************************************
 * Field:  [31:0] FSM_SECTOR2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_SECTOR2_FSM_SECTOR2_MASK                          0xffffffff
#define FLASH_FSM_SECTOR2_FSM_SECTOR2_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_FSM_BSLE0
 *
 ******************************************************************************
 * Field:  [31:0] FSM_BSLE0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_BSLE0_FSM_BSLE0_MASK                              0xffffffff
#define FLASH_FSM_BSLE0_FSM_BSLE0_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FSM_BSLE1
 *
 ******************************************************************************
 * Field:  [31:0] FSM_BSL1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_BSLE1_FSM_BSL1_MASK                               0xffffffff
#define FLASH_FSM_BSLE1_FSM_BSL1_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FSM_BSLP0
 *
 ******************************************************************************
 * Field:  [31:0] FSM_BSLP0
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_BSLP0_FSM_BSLP0_MASK                              0xffffffff
#define FLASH_FSM_BSLP0_FSM_BSLP0_SHIFT                                      0

/******************************************************************************
 *
 * Register: FLASH_FSM_BSLP1
 *
 ******************************************************************************
 * Field:  [31:0] FSM_BSL1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FSM_BSLP1_FSM_BSL1_MASK                               0xffffffff
#define FLASH_FSM_BSLP1_FSM_BSL1_SHIFT                                       0

/******************************************************************************
 *
 * Register: FLASH_FCFG_BANK
 *
 ******************************************************************************
 * Field: [31:20] EE_BANK_WIDTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BANK_EE_BANK_WIDTH_MASK                          0xfff00000
#define FLASH_FCFG_BANK_EE_BANK_WIDTH_SHIFT                                 20

/* Field: [19:16] EE_NUM_BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BANK_EE_NUM_BANK_MASK                            0x000f0000
#define FLASH_FCFG_BANK_EE_NUM_BANK_SHIFT                                   16

/* Field:  [15:4] MAIN_BANK_WIDTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BANK_MAIN_BANK_WIDTH_MASK                        0x0000fff0
#define FLASH_FCFG_BANK_MAIN_BANK_WIDTH_SHIFT                                4

/* Field:   [3:0] MAIN_NUM_BANK
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BANK_MAIN_NUM_BANK_MASK                          0x0000000f
#define FLASH_FCFG_BANK_MAIN_NUM_BANK_SHIFT                                  0

/******************************************************************************
 *
 * Register: FLASH_FCFG_WRAPPER
 *
 ******************************************************************************
 * Field: [31:24] FAMILY_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_FAMILY_TYPE_MASK                         0xff000000
#define FLASH_FCFG_WRAPPER_FAMILY_TYPE_SHIFT                                24

/* Field:    [20] MEM_MAP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_MEM_MAP                                  0x00100000
#define FLASH_FCFG_WRAPPER_MEM_MAP_MASK                             0x00100000
#define FLASH_FCFG_WRAPPER_MEM_MAP_SHIFT                                    20

/* Field: [19:16] CPU2
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_CPU2_MASK                                0x000f0000
#define FLASH_FCFG_WRAPPER_CPU2_SHIFT                                       16

/* Field: [15:12] EE_IN_MAIN
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_EE_IN_MAIN_MASK                          0x0000f000
#define FLASH_FCFG_WRAPPER_EE_IN_MAIN_SHIFT                                 12

/* Field:    [11] ROM
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_ROM                                      0x00000800
#define FLASH_FCFG_WRAPPER_ROM_MASK                                 0x00000800
#define FLASH_FCFG_WRAPPER_ROM_SHIFT                                        11

/* Field:    [10] IFLUSH
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_IFLUSH                                   0x00000400
#define FLASH_FCFG_WRAPPER_IFLUSH_MASK                              0x00000400
#define FLASH_FCFG_WRAPPER_IFLUSH_SHIFT                                     10

/* Field:     [9] SIL3
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_SIL3                                     0x00000200
#define FLASH_FCFG_WRAPPER_SIL3_MASK                                0x00000200
#define FLASH_FCFG_WRAPPER_SIL3_SHIFT                                        9

/* Field:     [8] ECCA
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_ECCA                                     0x00000100
#define FLASH_FCFG_WRAPPER_ECCA_MASK                                0x00000100
#define FLASH_FCFG_WRAPPER_ECCA_SHIFT                                        8

/* Field:   [7:6] AUTO_SUSP
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_AUTO_SUSP_MASK                           0x000000c0
#define FLASH_FCFG_WRAPPER_AUTO_SUSP_SHIFT                                   6

/* Field:   [5:4] UERR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_UERR_MASK                                0x00000030
#define FLASH_FCFG_WRAPPER_UERR_SHIFT                                        4

/* Field:   [3:0] CPU_TYPE1
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_WRAPPER_CPU_TYPE1_MASK                           0x0000000f
#define FLASH_FCFG_WRAPPER_CPU_TYPE1_SHIFT                                   0

/******************************************************************************
 *
 * Register: FLASH_FCFG_BNK_TYPE
 *
 ******************************************************************************
 * Field: [31:28] B7_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B7_TYPE_MASK                            0xf0000000
#define FLASH_FCFG_BNK_TYPE_B7_TYPE_SHIFT                                   28

/* Field: [27:24] B6_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B6_TYPE_MASK                            0x0f000000
#define FLASH_FCFG_BNK_TYPE_B6_TYPE_SHIFT                                   24

/* Field: [23:20] B5_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B5_TYPE_MASK                            0x00f00000
#define FLASH_FCFG_BNK_TYPE_B5_TYPE_SHIFT                                   20

/* Field: [19:16] B4_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B4_TYPE_MASK                            0x000f0000
#define FLASH_FCFG_BNK_TYPE_B4_TYPE_SHIFT                                   16

/* Field: [15:12] B3_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B3_TYPE_MASK                            0x0000f000
#define FLASH_FCFG_BNK_TYPE_B3_TYPE_SHIFT                                   12

/* Field:  [11:8] B2_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B2_TYPE_MASK                            0x00000f00
#define FLASH_FCFG_BNK_TYPE_B2_TYPE_SHIFT                                    8

/* Field:   [7:4] B1_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B1_TYPE_MASK                            0x000000f0
#define FLASH_FCFG_BNK_TYPE_B1_TYPE_SHIFT                                    4

/* Field:   [3:0] B0_TYPE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_BNK_TYPE_B0_TYPE_MASK                            0x0000000f
#define FLASH_FCFG_BNK_TYPE_B0_TYPE_SHIFT                                    0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B0_START
 *
 ******************************************************************************
 * Field: [31:28] B0_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B0_START_B0_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B0_START_B0_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B0_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B0_START_B0_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B0_START_B0_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B0_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B0_START_B0_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B0_START_B0_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B1_START
 *
 ******************************************************************************
 * Field: [31:28] B1_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B1_START_B1_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B1_START_B1_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B1_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B1_START_B1_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B1_START_B1_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B1_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B1_START_B1_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B1_START_B1_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B2_START
 *
 ******************************************************************************
 * Field: [31:28] B2_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B2_START_B2_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B2_START_B2_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B2_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B2_START_B2_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B2_START_B2_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B2_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B2_START_B2_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B2_START_B2_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B3_START
 *
 ******************************************************************************
 * Field: [31:28] B3_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B3_START_B3_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B3_START_B3_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B3_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B3_START_B3_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B3_START_B3_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B3_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B3_START_B3_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B3_START_B3_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B4_START
 *
 ******************************************************************************
 * Field: [31:28] B4_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B4_START_B4_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B4_START_B4_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B4_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B4_START_B4_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B4_START_B4_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B4_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B4_START_B4_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B4_START_B4_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B5_START
 *
 ******************************************************************************
 * Field: [31:28] B5_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B5_START_B5_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B5_START_B5_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B5_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B5_START_B5_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B5_START_B5_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B5_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B5_START_B5_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B5_START_B5_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B6_START
 *
 ******************************************************************************
 * Field: [31:28] B6_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B6_START_B6_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B6_START_B6_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B6_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B6_START_B6_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B6_START_B6_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B6_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B6_START_B6_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B6_START_B6_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B7_START
 *
 ******************************************************************************
 * Field: [31:28] B7_MAX_SECTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B7_START_B7_MAX_SECTOR_MASK                      0xf0000000
#define FLASH_FCFG_B7_START_B7_MAX_SECTOR_SHIFT                             28

/* Field: [27:24] B7_MUX_FACTOR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B7_START_B7_MUX_FACTOR_MASK                      0x0f000000
#define FLASH_FCFG_B7_START_B7_MUX_FACTOR_SHIFT                             24

/* Field:  [23:0] B7_START_ADDR
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B7_START_B7_START_ADDR_MASK                      0x00ffffff
#define FLASH_FCFG_B7_START_B7_START_ADDR_SHIFT                              0

/******************************************************************************
 *
 * Register: FLASH_FCFG_B0_SSIZE0
 *
 ******************************************************************************
 * Field: [27:16] B0_NUM_SECTORS
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B0_SSIZE0_B0_NUM_SECTORS_MASK                    0x0fff0000
#define FLASH_FCFG_B0_SSIZE0_B0_NUM_SECTORS_SHIFT                           16

/* Field:   [3:0] B0_SECT_SIZE
 *
 * Internal. Only to be used through TI provided API.
 */

#define FLASH_FCFG_B0_SSIZE0_B0_SECT_SIZE_MASK                      0x0000000f
#define FLASH_FCFG_B0_SSIZE0_B0_SECT_SIZE_SHIFT                              0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_C13X0_HW_FLASH_H */
