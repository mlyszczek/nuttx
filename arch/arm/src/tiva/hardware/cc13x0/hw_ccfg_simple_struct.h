/******************************************************************************
 *  Filename:       hw_ccfg_simple_struct_h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_HW_CCFG_SIMPLE_STRUCT_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_HW_CCFG_SIMPLE_STRUCT_H

/******************************************************************************
 * Public Types
 ******************************************************************************/

/******************************************************************************
 *
 * Customer configuration (ccfg) typedef.
 * The implementation of this struct is required by device ROM boot code
 * and must be placed at the end of flash. Do not modify this struct!
 *
 ******************************************************************************/

typedef struct
{                                            /*  Mapped to address */
  uint32_t   CCFG_EXT_LF_CLK               ; /* 0x50003fa8 */
  uint32_t   CCFG_MODE_CONF_1              ; /* 0x50003fac */
  uint32_t   CCFG_SIZE_AND_DIS_FLAGS       ; /* 0x50003fb0 */
  uint32_t   CCFG_MODE_CONF                ; /* 0x50003fb4 */
  uint32_t   CCFG_VOLT_LOAD_0              ; /* 0x50003fb8 */
  uint32_t   CCFG_VOLT_LOAD_1              ; /* 0x50003fbc */
  uint32_t   CCFG_RTC_OFFSET               ; /* 0x50003fc0 */
  uint32_t   CCFG_FREQ_OFFSET              ; /* 0x50003fc4 */
  uint32_t   CCFG_IEEE_MAC_0               ; /* 0x50003fc8 */
  uint32_t   CCFG_IEEE_MAC_1               ; /* 0x50003fcc */
  uint32_t   CCFG_IEEE_BLE_0               ; /* 0x50003fd0 */
  uint32_t   CCFG_IEEE_BLE_1               ; /* 0x50003fd4 */
  uint32_t   CCFG_BL_CONFIG                ; /* 0x50003fd8 */
  uint32_t   CCFG_ERASE_CONF               ; /* 0x50003fdc */
  uint32_t   CCFG_CCFG_TI_OPTIONS          ; /* 0x50003fe0 */
  uint32_t   CCFG_CCFG_TAP_DAP_0           ; /* 0x50003fe4 */
  uint32_t   CCFG_CCFG_TAP_DAP_1           ; /* 0x50003fe8 */
  uint32_t   CCFG_IMAGE_VALID_CONF         ; /* 0x50003fec */
  uint32_t   CCFG_CCFG_PROT_31_0           ; /* 0x50003ff0 */
  uint32_t   CCFG_CCFG_PROT_63_32          ; /* 0x50003ff4 */
  uint32_t   CCFG_CCFG_PROT_95_64          ; /* 0x50003ff8 */
  uint32_t   CCFG_CCFG_PROT_127_96         ; /* 0x50003ffc */
} ccfg_t;

/******************************************************************************
 *
 * Define the extern ccfg structure (__ccfg)
 *
 ******************************************************************************/

extern const ccfg_t __ccfg;

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_HW_CCFG_SIMPLE_STRUCT_H */
