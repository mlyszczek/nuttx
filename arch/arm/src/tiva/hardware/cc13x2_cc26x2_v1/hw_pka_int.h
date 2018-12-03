/******************************************************************************
 *  Filename:       hw_pka_int_h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_PKA_INT_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_PKA_INT_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * PKA_INT component
 *
 ******************************************************************************
 * PKA Options register
 */

#define PKA_INT_OPTIONS_OFFSET                                      0x00000ff8

/* PKA hardware revision register */

#define PKA_INT_REVISION_OFFSET                                     0x00000ffc

/******************************************************************************
 *
 * Register: PKA_INT_OPTIONS
 *
 ******************************************************************************
 * Field:    [10] AIC_PRESENT
 *
 * When set to '1', indicates that an EIP201 AIC  is included in the EIP150
 */

#define PKA_INT_OPTIONS_AIC_PRESENT                                 0x00000400
#define PKA_INT_OPTIONS_AIC_PRESENT_MASK                            0x00000400
#define PKA_INT_OPTIONS_AIC_PRESENT_SHIFT                                   10

/* Field:     [9] EIP76_PRESENT
 *
 * When set to '1', indicates that the EIP76 TRNG  is included in the EIP150
 */

#define PKA_INT_OPTIONS_EIP76_PRESENT                               0x00000200
#define PKA_INT_OPTIONS_EIP76_PRESENT_MASK                          0x00000200
#define PKA_INT_OPTIONS_EIP76_PRESENT_SHIFT                                  9

/* Field:     [8] EIP28_PRESENT
 *
 * When set to '1', indicates that the EIP28 PKA is included in the EIP150
 */

#define PKA_INT_OPTIONS_EIP28_PRESENT                               0x00000100
#define PKA_INT_OPTIONS_EIP28_PRESENT_MASK                          0x00000100
#define PKA_INT_OPTIONS_EIP28_PRESENT_SHIFT                                  8

/* Field:     [3] AXI_INTERFACE
 *
 * When set to '1', indicates that the EIP150 is equipped with a AXI interface
 */

#define PKA_INT_OPTIONS_AXI_INTERFACE                               0x00000008
#define PKA_INT_OPTIONS_AXI_INTERFACE_MASK                          0x00000008
#define PKA_INT_OPTIONS_AXI_INTERFACE_SHIFT                                  3

/* Field:     [2] AHB_IS_ASYNC
 *
 * When set to '1', indicates that AHB interface is asynchronous  Only
 * applicable when AHB_INTERFACE is 1
 */

#define PKA_INT_OPTIONS_AHB_IS_ASYNC                                0x00000004
#define PKA_INT_OPTIONS_AHB_IS_ASYNC_MASK                           0x00000004
#define PKA_INT_OPTIONS_AHB_IS_ASYNC_SHIFT                                   2

/* Field:     [1] AHB_INTERFACE
 *
 * When set to '1', indicates that the EIP150 is equipped with a AHB interface
 */

#define PKA_INT_OPTIONS_AHB_INTERFACE                               0x00000002
#define PKA_INT_OPTIONS_AHB_INTERFACE_MASK                          0x00000002
#define PKA_INT_OPTIONS_AHB_INTERFACE_SHIFT                                  1

/* Field:     [0] PLB_INTERFACE
 *
 * When set to '1', indicates that the EIP150 is equipped with a PLB interface
 */

#define PKA_INT_OPTIONS_PLB_INTERFACE                               0x00000001
#define PKA_INT_OPTIONS_PLB_INTERFACE_MASK                          0x00000001
#define PKA_INT_OPTIONS_PLB_INTERFACE_SHIFT                                  0

/******************************************************************************
 *
 * Register: PKA_INT_REVISION
 *
 ******************************************************************************
 * Field: [27:24] MAJOR_REVISION
 *
 * These bits encode the major version number for this module
 */

#define PKA_INT_REVISION_MAJOR_REVISION_MASK                        0x0f000000
#define PKA_INT_REVISION_MAJOR_REVISION_SHIFT                               24

/* Field: [23:20] MINOR_REVISION
 *
 * These bits encode the minor version number for this module
 */

#define PKA_INT_REVISION_MINOR_REVISION_MASK                        0x00f00000
#define PKA_INT_REVISION_MINOR_REVISION_SHIFT                               20

/* Field: [19:16] PATCH_LEVEL
 *
 * These bits encode the hardware patch level for this module they start at
 * value 0 on the first release
 */

#define PKA_INT_REVISION_PATCH_LEVEL_MASK                           0x000f0000
#define PKA_INT_REVISION_PATCH_LEVEL_SHIFT                                  16

/* Field:  [15:8] COMP_EIP_NUM
 *
 * These bits simply contain the complement of bits [7:0], used by a driver to
 * ascertain that the EIP150 revision register is indeed read
 */

#define PKA_INT_REVISION_COMP_EIP_NUM_MASK                          0x0000ff00
#define PKA_INT_REVISION_COMP_EIP_NUM_SHIFT                                  8

/* Field:   [7:0] EIP_NUM
 *
 * These bits encode the AuthenTec EIP number for the EIP150
 */

#define PKA_INT_REVISION_EIP_NUM_MASK                               0x000000ff
#define PKA_INT_REVISION_EIP_NUM_SHIFT                                       0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V1_PKA_INT_H */
