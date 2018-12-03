/******************************************************************************
 *  Filename:       hw_aon_ioc_h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_HW_AON_IOC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_HW_AON_IOC_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * AON_IOC component
 *
 ******************************************************************************
 * Internal
 */

#define AON_IOC_IOSTRMIN_OFFSET                                     0x00000000

/* Internal */

#define AON_IOC_IOSTRMED_OFFSET                                     0x00000004

/* Internal */

#define AON_IOC_IOSTRMAX_OFFSET                                     0x00000008

/* IO Latch Control */

#define AON_IOC_IOCLATCH_OFFSET                                     0x0000000c

/* SCLK_LF External Output Control */

#define AON_IOC_CLK32KCTL_OFFSET                                    0x00000010

/******************************************************************************
 *
 * Register: AON_IOC_IOSTRMIN
 *
 ******************************************************************************
 * Field:   [2:0] GRAY_CODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define AON_IOC_IOSTRMIN_GRAY_CODE_MASK                             0x00000007
#define AON_IOC_IOSTRMIN_GRAY_CODE_SHIFT                                     0

/******************************************************************************
 *
 * Register: AON_IOC_IOSTRMED
 *
 ******************************************************************************
 * Field:   [2:0] GRAY_CODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define AON_IOC_IOSTRMED_GRAY_CODE_MASK                             0x00000007
#define AON_IOC_IOSTRMED_GRAY_CODE_SHIFT                                     0

/******************************************************************************
 *
 * Register: AON_IOC_IOSTRMAX
 *
 ******************************************************************************
 * Field:   [2:0] GRAY_CODE
 *
 * Internal. Only to be used through TI provided API.
 */

#define AON_IOC_IOSTRMAX_GRAY_CODE_MASK                             0x00000007
#define AON_IOC_IOSTRMAX_GRAY_CODE_SHIFT                                     0

/******************************************************************************
 *
 * Register: AON_IOC_IOCLATCH
 *
 ******************************************************************************
 * Field:     [0] EN
 *
 * Controls latches between MCU IOC and AON_IOC.
 *
 * The latches are transparent by default.
 *
 * They must be closed prior to power off the domain(s) controlling the IOs in
 * order to preserve IO values on external pins.
 * ENUMs:
 * TRANSP                   Latches are transparent, meaning the value of the
 *                          IO is directly controlled by the GPIO or
 *                          peripheral value
 * STATIC                   Latches are static, meaning the current value on
 *                          the IO pin is frozen by latches and kept even
 *                          if GPIO module or a peripheral module is turned
 *                          off
 */

#define AON_IOC_IOCLATCH_EN                                         0x00000001
#define AON_IOC_IOCLATCH_EN_MASK                                    0x00000001
#define AON_IOC_IOCLATCH_EN_SHIFT                                            0
#define AON_IOC_IOCLATCH_EN_TRANSP                                  0x00000001
#define AON_IOC_IOCLATCH_EN_STATIC                                  0x00000000

/******************************************************************************
 *
 * Register: AON_IOC_CLK32KCTL
 *
 ******************************************************************************
 * Field:     [0] OE_N
 *
 * 0: Output enable active. SCLK_LF output on IO pin that has PORT_ID (e.g.
 * IOC:IOCFG0.PORT_ID) set to AON_CLK32K.
 * 1: Output enable not active
 */

#define AON_IOC_CLK32KCTL_OE_N                                      0x00000001
#define AON_IOC_CLK32KCTL_OE_N_MASK                                 0x00000001
#define AON_IOC_CLK32KCTL_OE_N_SHIFT                                         0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_HW_AON_IOC_H */
