/******************************************************************************
 *  Filename:       hw_pka_h
 *  Revised:        2017-09-14 10:33:07 +0200 (Thu, 14 Sep 2017)
 *  Revision:       49733
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_PKA_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_PKA_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * PKA component
 *
 ******************************************************************************
 * PKA Vector A Address
 */

#define PKA_APTR_OFFSET                                             0x00000000

/* PKA Vector B Address */

#define PKA_BPTR_OFFSET                                             0x00000004

/* PKA Vector C Address */

#define PKA_CPTR_OFFSET                                             0x00000008

/* PKA Vector D Address */

#define PKA_DPTR_OFFSET                                             0x0000000c

/* PKA Vector A Length */

#define PKA_ALENGTH_OFFSET                                          0x00000010

/* PKA Vector B Length */

#define PKA_BLENGTH_OFFSET                                          0x00000014

/* PKA Bit Shift Value */

#define PKA_SHIFT_OFFSET                                            0x00000018

/* PKA Function */

#define PKA_FUNCTION_OFFSET                                         0x0000001c

/* PKA compare result */

#define PKA_COMPARE_OFFSET                                          0x00000020

/* PKA most-significant-word of result vector */

#define PKA_MSW_OFFSET                                              0x00000024

/* PKA most-significant-word of divide remainder */

#define PKA_DIVMSW_OFFSET                                           0x00000028

/* PKA sequencer control and status register */

#define PKA_SEQCTRL_OFFSET                                          0x000000c8

/* PKA hardware options register */

#define PKA_OPTIONS_OFFSET                                          0x000000f4

/* PKA firmware revision and capabilities register */

#define PKA_FWREV_OFFSET                                            0x000000f8

/* PKA hardware revision register */

#define PKA_HWREV_OFFSET                                            0x000000fc

/******************************************************************************
 *
 * Register: PKA_APTR
 *
 ******************************************************************************
 * Field:  [10:0] APTR
 *
 * This register specifies the location of vector A within the PKA RAM. Vectors
 * are identified through the location of their least-significant 32-bit word.
 * Note that bit [0] must be zero to ensure that the vector starts at an 8-byte
 * boundary.
 */

#define PKA_APTR_APTR_MASK                                          0x000007ff
#define PKA_APTR_APTR_SHIFT                                                  0

/******************************************************************************
 *
 * Register: PKA_BPTR
 *
 ******************************************************************************
 * Field:  [10:0] BPTR
 *
 * This register specifies the location of vector B within the PKA RAM. Vectors
 * are identified through the location of their least-significant 32-bit word.
 * Note that bit [0] must be zero to ensure that the vector starts at an 8-byte
 * boundary.
 */

#define PKA_BPTR_BPTR_MASK                                          0x000007ff
#define PKA_BPTR_BPTR_SHIFT                                                  0

/******************************************************************************
 *
 * Register: PKA_CPTR
 *
 ******************************************************************************
 * Field:  [10:0] CPTR
 *
 * This register specifies the location of vector C within the PKA RAM. Vectors
 * are identified through the location of their least-significant 32-bit word.
 * Note that bit [0] must be zero to ensure that the vector starts at an 8-byte
 * boundary.
 */

#define PKA_CPTR_CPTR_MASK                                          0x000007ff
#define PKA_CPTR_CPTR_SHIFT                                                  0

/******************************************************************************
 *
 * Register: PKA_DPTR
 *
 ******************************************************************************
 * Field:  [10:0] DPTR
 *
 * This register specifies the location of vector D within the PKA RAM. Vectors
 * are identified through the location of their least-significant 32-bit word.
 * Note that bit [0] must be zero to ensure that the vector starts at an 8-byte
 * boundary.
 */

#define PKA_DPTR_DPTR_MASK                                          0x000007ff
#define PKA_DPTR_DPTR_SHIFT                                                  0

/******************************************************************************
 *
 * Register: PKA_ALENGTH
 *
 ******************************************************************************
 * Field:   [8:0] ALENGTH
 *
 * This register specifies the length (in 32-bit words) of Vector A.
 */

#define PKA_ALENGTH_ALENGTH_MASK                                    0x000001ff
#define PKA_ALENGTH_ALENGTH_SHIFT                                            0

/******************************************************************************
 *
 * Register: PKA_BLENGTH
 *
 ******************************************************************************
 * Field:   [8:0] BLENGTH
 *
 * This register specifies the length (in 32-bit words) of Vector B.
 */

#define PKA_BLENGTH_BLENGTH_MASK                                    0x000001ff
#define PKA_BLENGTH_BLENGTH_SHIFT                                            0

/******************************************************************************
 *
 * Register: PKA_SHIFT
 *
 ******************************************************************************
 * Field:   [4:0] NUM_BITS_TO_SHIFT
 *
 * This register specifies the number of bits to shift the input vector (in the
 * range 0-31) during a Rshift or Lshift operation.
 */

#define PKA_SHIFT_NUM_BITS_TO_SHIFT_MASK                            0x0000001f
#define PKA_SHIFT_NUM_BITS_TO_SHIFT_SHIFT                                    0

/******************************************************************************
 *
 * Register: PKA_FUNCTION
 *
 ******************************************************************************
 * Field:    [24] STALL_RESULT
 *
 * When written with a 1b, updating of the COMPARE bit, MSW and DIVMSW
 * registers, as well as resetting the run bit is stalled beyond the point that
 * a running operation is actually finished. Use this to allow software enough
 * time to read results from a previous operation when the newly started
 * operation is known to take only a short amount of time. If a result is
 * waiting, the result registers is updated and the run bit is reset in the
 * clock cycle following writing the stall result bit back to 0b. The Stall
 * result function may only be used for basic PKCP operations.
 */

#define PKA_FUNCTION_STALL_RESULT                                   0x01000000
#define PKA_FUNCTION_STALL_RESULT_MASK                              0x01000000
#define PKA_FUNCTION_STALL_RESULT_SHIFT                                     24

/* Field:    [15] RUN
 *
 * The host sets this bit to instruct the PKA module to begin processing the
 * basic PKCP or complex sequencer operation. This bit is reset low
 * automatically when the operation is complete.
 * After a reset, the run bit is always set to 1b. Depending on the option,
 * program ROM or program RAM, the following applies:
 * Program ROM - The first sequencer instruction sets the bit to 0b. This is
 * done immediately after the hardware reset is released.
 * Program RAM - The sequencer must set the bit to 0b. As a valid firmware may
 * not have been loaded, the sequencer is held in software reset after the
 * hardware reset is released (the SEQCTRL.RESET bit is set to 1b). After the
 * FW image is loaded and the Reset bit is cleared, the sequencer starts to
 * execute the FW. The first instruction clears the run bit.
 * In both cases a few clock cycles are needed before the first instruction is
 * executed and the run bit state has been propagated.
 */

#define PKA_FUNCTION_RUN                                            0x00008000
#define PKA_FUNCTION_RUN_MASK                                       0x00008000
#define PKA_FUNCTION_RUN_SHIFT                                              15

/* Field: [14:12] SEQUENCER_OPERATIONS
 *
 * These bits select the complex sequencer operation to perform:
 * 0x0: None
 * 0x1: ExpMod-CRT
 * 0x2: ECmontMUL
 * 0x3: ECC-ADD (if available in firmware, otherwise reserved)
 * 0x4: ExpMod-ACT2
 * 0x5: ECC-MUL (if available in firmware, otherwise reserved)
 * 0x6: ExpMod-variable
 * 0x7: ModInv (if available in firmware, otherwise reserved)
 * The encoding of these operations is determined by sequencer firmware.
 */

#define PKA_FUNCTION_SEQUENCER_OPERATIONS_MASK                      0x00007000
#define PKA_FUNCTION_SEQUENCER_OPERATIONS_SHIFT                             12

/* Field:    [11] COPY
 *
 * Perform copy operation
 */

#define PKA_FUNCTION_COPY                                           0x00000800
#define PKA_FUNCTION_COPY_MASK                                      0x00000800
#define PKA_FUNCTION_COPY_SHIFT                                             11

/* Field:    [10] COMPARE
 *
 * Perform compare operation
 */

#define PKA_FUNCTION_COMPARE                                        0x00000400
#define PKA_FUNCTION_COMPARE_MASK                                   0x00000400
#define PKA_FUNCTION_COMPARE_SHIFT                                          10

/* Field:     [9] MODULO
 *
 * Perform modulo operation
 */

#define PKA_FUNCTION_MODULO                                         0x00000200
#define PKA_FUNCTION_MODULO_MASK                                    0x00000200
#define PKA_FUNCTION_MODULO_SHIFT                                            9

/* Field:     [8] DIVIDE
 *
 * Perform divide operation
 */

#define PKA_FUNCTION_DIVIDE                                         0x00000100
#define PKA_FUNCTION_DIVIDE_MASK                                    0x00000100
#define PKA_FUNCTION_DIVIDE_SHIFT                                            8

/* Field:     [7] LSHIFT
 *
 * Perform left shift operation
 */

#define PKA_FUNCTION_LSHIFT                                         0x00000080
#define PKA_FUNCTION_LSHIFT_MASK                                    0x00000080
#define PKA_FUNCTION_LSHIFT_SHIFT                                            7

/* Field:     [6] RSHIFT
 *
 * Perform right shift operation
 */

#define PKA_FUNCTION_RSHIFT                                         0x00000040
#define PKA_FUNCTION_RSHIFT_MASK                                    0x00000040
#define PKA_FUNCTION_RSHIFT_SHIFT                                            6

/* Field:     [5] SUBTRACT
 *
 * Perform subtract operation
 */

#define PKA_FUNCTION_SUBTRACT                                       0x00000020
#define PKA_FUNCTION_SUBTRACT_MASK                                  0x00000020
#define PKA_FUNCTION_SUBTRACT_SHIFT                                          5

/* Field:     [4] ADD
 *
 * Perform add operation
 */

#define PKA_FUNCTION_ADD                                            0x00000010
#define PKA_FUNCTION_ADD_MASK                                       0x00000010
#define PKA_FUNCTION_ADD_SHIFT                                               4

/* Field:     [3] MS_ONE
 *
 * Loads the location of the Most Significant one bit within the result word
 * indicated in the MSW register into bits [4:0] of the DIVMSW.MSW_ADDRESS
 * register - can only be used with basic PKCP operations, except for Divide,
 * Modulo and Compare.
 */

#define PKA_FUNCTION_MS_ONE                                         0x00000008
#define PKA_FUNCTION_MS_ONE_MASK                                    0x00000008
#define PKA_FUNCTION_MS_ONE_SHIFT                                            3

/* Field:     [1] ADDSUB
 *
 * Perform combined add/subtract operation
 */

#define PKA_FUNCTION_ADDSUB                                         0x00000002
#define PKA_FUNCTION_ADDSUB_MASK                                    0x00000002
#define PKA_FUNCTION_ADDSUB_SHIFT                                            1

/* Field:     [0] MULTIPLY
 *
 * Perform multiply operation
 */

#define PKA_FUNCTION_MULTIPLY                                       0x00000001
#define PKA_FUNCTION_MULTIPLY_MASK                                  0x00000001
#define PKA_FUNCTION_MULTIPLY_SHIFT                                          0

/******************************************************************************
 *
 * Register: PKA_COMPARE
 *
 ******************************************************************************
 * Field:     [2] A_GREATER_THAN_B
 *
 * Vector_A is greater than Vector_B
 */

#define PKA_COMPARE_A_GREATER_THAN_B                                0x00000004
#define PKA_COMPARE_A_GREATER_THAN_B_MASK                           0x00000004
#define PKA_COMPARE_A_GREATER_THAN_B_SHIFT                                   2

/* Field:     [1] A_LESS_THAN_B
 *
 * Vector_A is less than Vector_B
 */

#define PKA_COMPARE_A_LESS_THAN_B                                   0x00000002
#define PKA_COMPARE_A_LESS_THAN_B_MASK                              0x00000002
#define PKA_COMPARE_A_LESS_THAN_B_SHIFT                                      1

/* Field:     [0] A_EQUALS_B
 *
 * Vector_A is equal to Vector_B
 */

#define PKA_COMPARE_A_EQUALS_B                                      0x00000001
#define PKA_COMPARE_A_EQUALS_B_MASK                                 0x00000001
#define PKA_COMPARE_A_EQUALS_B_SHIFT                                         0

/******************************************************************************
 *
 * Register: PKA_MSW
 *
 ******************************************************************************
 * Field:    [15] RESULT_IS_ZERO
 *
 * The result vector is all zeroes, ignore the address returned in bits [10:0]
 */

#define PKA_MSW_RESULT_IS_ZERO                                      0x00008000
#define PKA_MSW_RESULT_IS_ZERO_MASK                                 0x00008000
#define PKA_MSW_RESULT_IS_ZERO_SHIFT                                        15

/* Field:  [10:0] MSW_ADDRESS
 *
 * Address of the most-significant nonzero 32-bit word of the result vector in
 * PKA RAM
 */

#define PKA_MSW_MSW_ADDRESS_MASK                                    0x000007ff
#define PKA_MSW_MSW_ADDRESS_SHIFT                                            0

/******************************************************************************
 *
 * Register: PKA_DIVMSW
 *
 ******************************************************************************
 * Field:    [15] RESULT_IS_ZERO
 *
 * The result vector is all zeroes, ignore the address returned in bits [10:0]
 */

#define PKA_DIVMSW_RESULT_IS_ZERO                                   0x00008000
#define PKA_DIVMSW_RESULT_IS_ZERO_MASK                              0x00008000
#define PKA_DIVMSW_RESULT_IS_ZERO_SHIFT                                     15

/* Field:  [10:0] MSW_ADDRESS
 *
 * Address of the most significant nonzero 32-bit word of the remainder result
 * vector in PKA RAM
 */

#define PKA_DIVMSW_MSW_ADDRESS_MASK                                 0x000007ff
#define PKA_DIVMSW_MSW_ADDRESS_SHIFT                                         0

/******************************************************************************
 *
 * Register: PKA_SEQCTRL
 *
 ******************************************************************************
 * Field:    [31] RESET
 *
 * Option program ROM: Reset value = 0. Read/Write, reset value 0b (ZERO).
 * Writing 1b resets the sequencer, write to 0b to restart operations again. As
 * the reset value is 0b, the sequencer will automatically start operations
 * executing from program ROM. This bit should always be written with zero and
 * ignored when reading this register.
 *
 * Option Program RAM: Reset value =1. Read/Write, reset value 1b (ONE). When
 * 1b, the sequencer is held in a reset state and the PKA_PROGRAM area is
 * accessible for loading the sequencer program (while the PKA_DATA_RAM is
 * inaccessible), write to 0b to (re)start sequencer operations and disable
 * PKA_PROGRAM area accessibility (also enables the PKA_DATA_RAM accesses).
 * Resetting the sequencer (in order to load other firmware) should only be
 * done when the PKA Engine is not performing any operations (i.e. the
 * FUNCTION.RUN bit should be zero).
 */

#define PKA_SEQCTRL_RESET                                           0x80000000
#define PKA_SEQCTRL_RESET_MASK                                      0x80000000
#define PKA_SEQCTRL_RESET_SHIFT                                             31

/* Field:  [15:8] SEQUENCER_STAT
 *
 * These read-only bits can be used by the sequencer to communicate status to
 * the outside world. Bit [8] is also used as sequencer interrupt, with the
 * complement of this bit ORed into the FUNCTION.RUN bit. This field should
 * always be written with zeroes and ignored when reading this register.
 */

#define PKA_SEQCTRL_SEQUENCER_STAT_MASK                             0x0000ff00
#define PKA_SEQCTRL_SEQUENCER_STAT_SHIFT                                     8

/* Field:   [7:0] SW_CONTROL_STAT
 *
 * These bits can be used by software to trigger sequencer operations. External
 * logic can set these bits by writing 1b, cannot reset them by writing 0b. The
 * sequencer can reset these bits by writing 0b, cannot set them by writing 1b.
 * Setting the FUNCTION.RUN bit together with a nonzero sequencer operations
 * field automatically sets bit [0] here. This field should always be written
 * with zeroes and ignored when reading this register.
 */

#define PKA_SEQCTRL_SW_CONTROL_STAT_MASK                            0x000000ff
#define PKA_SEQCTRL_SW_CONTROL_STAT_SHIFT                                    0

/******************************************************************************
 *
 * Register: PKA_OPTIONS
 *
 ******************************************************************************
 * Field:    [11] INT_MASKING
 *
 * Interrupt Masking
 *    0x0:  indicates that the main interrupt output (bit [1] of the interrupts
 * output bus) is the direct complement of the run bit in the PKA_CONTROL
 * register,                                                0x1 : indicates
 * that interrupt masking logic is present for this output.
 * Note: Reset value is undefined
 */

#define PKA_OPTIONS_INT_MASKING                                     0x00000800
#define PKA_OPTIONS_INT_MASKING_MASK                                0x00000800
#define PKA_OPTIONS_INT_MASKING_SHIFT                                       11

/* Field:  [10:8] PROTECTION_OPTION
 *
 * Protection Option
 *     0x0: indicates no additional protection against side channel attacks,
 *
 * 0x1: indicates the SCAP option
 *   0x2: Reserved
 *       0x3: indicates the PROT option;
 * Note: Reset value is undefined
 */

#define PKA_OPTIONS_PROTECTION_OPTION_MASK                          0x00000700
#define PKA_OPTIONS_PROTECTION_OPTION_SHIFT                                  8

/* Field:     [7] PROGRAM_RAM
 *
 * Program RAM
 *  0x1: indicates sequencer program storage in RAM,               0x0:
 * indicates sequencer program storage  in ROM.
 * Note: Reset value is undefined
 */

#define PKA_OPTIONS_PROGRAM_RAM                                     0x00000080
#define PKA_OPTIONS_PROGRAM_RAM_MASK                                0x00000080
#define PKA_OPTIONS_PROGRAM_RAM_SHIFT                                        7

/* Field:   [6:5] SEQUENCER_CONFIGURATION
 *
 * Sequencer Configuration
 * 0x0: Reserved
 *          0x1 : Indicates a standard sequencer
 *   0x2: Reserved
 *             0x3: Reserved
 */

#define PKA_OPTIONS_SEQUENCER_CONFIGURATION_MASK                    0x00000060
#define PKA_OPTIONS_SEQUENCER_CONFIGURATION_SHIFT                            5

/* Field:   [1:0] PKCP_CONFIGURATION
 *
 * PKCP Configuration                                                       0x0
 * : Reserved
 *      0x1 : Indicates a PKCP with a 16x16 multiplier,                  0x2:
 * indicates a PKCP with a 32x32 multiplier,                     0x3 : Reserved
 * Note: Reset value is undefined.
 */

#define PKA_OPTIONS_PKCP_CONFIGURATION_MASK                         0x00000003
#define PKA_OPTIONS_PKCP_CONFIGURATION_SHIFT                                 0

/******************************************************************************
 *
 * Register: PKA_FWREV
 *
 ******************************************************************************
 * Field: [31:28] FW_CAPABILITIES
 *
 * Firmware Capabilities
 *
 *                    4-bit binary encoding for the functionality implemented
 * in the firmware.
 *            0x0: indicates basic ModExp with/without CRT.               0x1:
 * adds Modular Inversion,                                          0x2: value
 * 2 adds Modular Inversion and ECC operations.
 *                                       0x3-0xf : Reserved.
 */

#define PKA_FWREV_FW_CAPABILITIES_MASK                              0xf0000000
#define PKA_FWREV_FW_CAPABILITIES_SHIFT                                     28

/* Field: [27:24] MAJOR_FW_REVISION
 *
 * 4-bit binary encoding of the major firmware revision number
 */

#define PKA_FWREV_MAJOR_FW_REVISION_MASK                            0x0f000000
#define PKA_FWREV_MAJOR_FW_REVISION_SHIFT                                   24

/* Field: [23:20] MINOR_FW_REVISION
 *
 * 4-bit binary encoding of the minor firmware revision number
 */

#define PKA_FWREV_MINOR_FW_REVISION_MASK                            0x00f00000
#define PKA_FWREV_MINOR_FW_REVISION_SHIFT                                   20

/* Field: [19:16] FW_PATCH_LEVEL
 *
 * 4-bit binary encoding of the firmware patch level, initial release will
 * carry value zero
 * Patches are used to remove bugs without changing the functionality or
 * interface of a module.
 */

#define PKA_FWREV_FW_PATCH_LEVEL_MASK                               0x000f0000
#define PKA_FWREV_FW_PATCH_LEVEL_SHIFT                                      16

/******************************************************************************
 *
 * Register: PKA_HWREV
 *
 ******************************************************************************
 * Field: [27:24] MAJOR_HW_REVISION
 *
 * 4-bit binary encoding of the major hardware revision number
 */

#define PKA_HWREV_MAJOR_HW_REVISION_MASK                            0x0f000000
#define PKA_HWREV_MAJOR_HW_REVISION_SHIFT                                   24

/* Field: [23:20] MINOR_HW_REVISION
 *
 * 4-bit binary encoding of the minor hardware revision number
 */

#define PKA_HWREV_MINOR_HW_REVISION_MASK                            0x00f00000
#define PKA_HWREV_MINOR_HW_REVISION_SHIFT                                   20

/* Field: [19:16] HW_PATCH_LEVEL
 *
 * 4-bit binary encoding of the hardware patch level, initial release will
 * carry value zero
 * Patches are used to remove bugs without changing the functionality or
 * interface of a module.
 */

#define PKA_HWREV_HW_PATCH_LEVEL_MASK                               0x000f0000
#define PKA_HWREV_HW_PATCH_LEVEL_SHIFT                                      16

/* Field:  [15:8] COMPLEMENT_OF_BASIC_EIP_NUMBER
 *
 * Bit-by-bit logic complement of bits [7:0], EIP-28 gives 0xe3
 */

#define PKA_HWREV_COMPLEMENT_OF_BASIC_EIP_NUMBER_MASK               0x0000ff00
#define PKA_HWREV_COMPLEMENT_OF_BASIC_EIP_NUMBER_SHIFT                       8

/* Field:   [7:0] BASIC_EIP_NUMBER
 *
 * 8-bit binary encoding of the EIP number, EIP-28 gives 0x1c
 */

#define PKA_HWREV_BASIC_EIP_NUMBER_MASK                             0x000000ff
#define PKA_HWREV_BASIC_EIP_NUMBER_SHIFT                                     0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_PKA_H */
