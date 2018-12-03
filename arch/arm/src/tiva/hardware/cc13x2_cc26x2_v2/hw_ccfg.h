/******************************************************************************
 *  Filename:       hw_ccfg_h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_CCFG_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_CCFG_H

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/******************************************************************************
 *
 * This section defines the register offsets of
 * CCFG component
 *
 ******************************************************************************
 * Extern LF clock configuration
 */

#define CCFG_EXT_LF_CLK_OFFSET                                      0x00001fa8

/* Mode Configuration 1 */

#define CCFG_MODE_CONF_1_OFFSET                                     0x00001fac

/* CCFG Size and Disable Flags */

#define CCFG_SIZE_AND_DIS_FLAGS_OFFSET                              0x00001fb0

/* Mode Configuration 0 */

#define CCFG_MODE_CONF_OFFSET                                       0x00001fb4

/* Voltage Load 0 */

#define CCFG_VOLT_LOAD_0_OFFSET                                     0x00001fb8

/* Voltage Load 1 */

#define CCFG_VOLT_LOAD_1_OFFSET                                     0x00001fbc

/* Real Time Clock Offset */

#define CCFG_RTC_OFFSET_OFFSET                                      0x00001fc0

/* Frequency Offset */

#define CCFG_FREQ_OFFSET_OFFSET                                     0x00001fc4

/* IEEE MAC Address 0 */

#define CCFG_IEEE_MAC_0_OFFSET                                      0x00001fc8

/* IEEE MAC Address 1 */

#define CCFG_IEEE_MAC_1_OFFSET                                      0x00001fcc

/* IEEE BLE Address 0 */

#define CCFG_IEEE_BLE_0_OFFSET                                      0x00001fd0

/* IEEE BLE Address 1 */

#define CCFG_IEEE_BLE_1_OFFSET                                      0x00001fd4

/* Bootloader Configuration */

#define CCFG_BL_CONFIG_OFFSET                                       0x00001fd8

/* Erase Configuration */

#define CCFG_ERASE_CONF_OFFSET                                      0x00001fdc

/* TI Options */

#define CCFG_CCFG_TI_OPTIONS_OFFSET                                 0x00001fe0

/* Test Access Points Enable 0 */

#define CCFG_CCFG_TAP_DAP_0_OFFSET                                  0x00001fe4

/* Test Access Points Enable 1 */

#define CCFG_CCFG_TAP_DAP_1_OFFSET                                  0x00001fe8

/* Image Valid */

#define CCFG_IMAGE_VALID_CONF_OFFSET                                0x00001fec

/* Protect Sectors 0-31 */

#define CCFG_CCFG_PROT_31_0_OFFSET                                  0x00001ff0

/* Protect Sectors 32-63 */

#define CCFG_CCFG_PROT_63_32_OFFSET                                 0x00001ff4

/* Protect Sectors 64-95 */

#define CCFG_CCFG_PROT_95_64_OFFSET                                 0x00001ff8

/* Protect Sectors 96-127 */

#define CCFG_CCFG_PROT_127_96_OFFSET                                0x00001ffc

/******************************************************************************
 *
 * Register: CCFG_EXT_LF_CLK
 *
 ******************************************************************************
 * Field: [31:24] DIO
 *
 * Unsigned integer, selecting the DIO to supply external 32kHz clock as
 * SCLK_LF when MODE_CONF.SCLK_LF_OPTION is set to EXTERNAL. The selected DIO
 * will be marked as reserved by the pin driver (TI-RTOS environment) and hence
 * not selectable for other usage.
 */

#define CCFG_EXT_LF_CLK_DIO_MASK                                    0xff000000
#define CCFG_EXT_LF_CLK_DIO_SHIFT                                           24

/* Field:  [23:0] RTC_INCREMENT
 *
 * Unsigned integer, defining the input frequency of the external clock and is
 * written to AON_RTC:SUBSECINC.VALUEINC. Defined as follows:
 * EXT_LF_CLK.RTC_INCREMENT = 2^38/InputClockFrequency in Hertz (e.g.:
 * RTC_INCREMENT=0x800000 for InputClockFrequency=32768 Hz)
 */

#define CCFG_EXT_LF_CLK_RTC_INCREMENT_MASK                          0x00ffffff
#define CCFG_EXT_LF_CLK_RTC_INCREMENT_SHIFT                                  0

/******************************************************************************
 *
 * Register: CCFG_MODE_CONF_1
 *
 ******************************************************************************
 * Field: [23:20] ALT_DCDC_VMIN
 *
 * Minimum voltage for when DC/DC should be used if alternate DC/DC setting is
 * enabled (SIZE_AND_DIS_FLAGS.DIS_ALT_DCDC_SETTING=0).
 * Voltage = (28 + ALT_DCDC_VMIN) / 16.
 * 0: 1.75V
 * 1: 1.8125V
 * ...
 * 14: 2.625V
 * 15: 2.6875V
 *
 * NOTE! The DriverLib function SysCtrl_DCDC_VoltageConditionalControl() must
 * be called regularly to apply this field (handled automatically if using TI
 * RTOS!).
 */

#define CCFG_MODE_CONF_1_ALT_DCDC_VMIN_MASK                         0x00f00000
#define CCFG_MODE_CONF_1_ALT_DCDC_VMIN_SHIFT                                20

/* Field:    [19] ALT_DCDC_DITHER_EN
 *
 * Enable DC/DC dithering if alternate DC/DC setting is enabled
 * (SIZE_AND_DIS_FLAGS.DIS_ALT_DCDC_SETTING=0).
 * 0: Dither disable
 * 1: Dither enable
 */

#define CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN                         0x00080000
#define CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN_MASK                    0x00080000
#define CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN_SHIFT                           19

/* Field: [18:16] ALT_DCDC_IPEAK
 *
 * Inductor peak current if alternate DC/DC setting is enabled
 * (SIZE_AND_DIS_FLAGS.DIS_ALT_DCDC_SETTING=0). Assuming 10uH external
 * inductor!
 * Peak current = 31 + ( 4 * ALT_DCDC_IPEAK ) :
 * 0: 31mA (min)
 * ...
 * 4: 47mA
 * ...
 * 7: 59mA (max)
 */

#define CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_MASK                        0x00070000
#define CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_SHIFT                               16

/* Field: [15:12] DELTA_IBIAS_INIT
 *
 * Signed delta value for IBIAS_INIT. Delta value only applies if
 * SIZE_AND_DIS_FLAGS.DIS_XOSC_OVR=0.
 * See FCFG1:AMPCOMP_CTRL1.IBIAS_INIT
 */

#define CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_MASK                      0x0000f000
#define CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_SHIFT                             12

/* Field:  [11:8] DELTA_IBIAS_OFFSET
 *
 * Signed delta value for IBIAS_OFFSET. Delta value only applies if
 * SIZE_AND_DIS_FLAGS.DIS_XOSC_OVR=0.
 * See FCFG1:AMPCOMP_CTRL1.IBIAS_OFFSET
 */

#define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_MASK                    0x00000f00
#define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_SHIFT                            8

/* Field:   [7:0] XOSC_MAX_START
 *
 * Unsigned value of maximum XOSC startup time (worst case) in units of 100us.
 * Value only applies if SIZE_AND_DIS_FLAGS.DIS_XOSC_OVR=0.
 */

#define CCFG_MODE_CONF_1_XOSC_MAX_START_MASK                        0x000000ff
#define CCFG_MODE_CONF_1_XOSC_MAX_START_SHIFT                                0

/******************************************************************************
 *
 * Register: CCFG_SIZE_AND_DIS_FLAGS
 *
 ******************************************************************************
 * Field: [31:16] SIZE_OF_CCFG
 *
 * Total size of CCFG in bytes.
 */

#define CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_MASK                   0xffff0000
#define CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_SHIFT                          16

/* Field:  [15:4] DISABLE_FLAGS
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_MASK                  0x0000fff0
#define CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_SHIFT                          4

/* Field:     [3] DIS_TCXO
 *
 * Disable TCXO.
 * 0: TCXO functionality enabled.
 * 1: TCXO functionality disabled.
 * Note:
 * An external TCXO is required if DIS_TCXO = 0.
 */

#define CCFG_SIZE_AND_DIS_FLAGS_DIS_TCXO                            0x00000008
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_TCXO_MASK                       0x00000008
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_TCXO_SHIFT                               3

/* Field:     [2] DIS_GPRAM
 *
 * Disable GPRAM (or use the 8K VIMS RAM as CACHE RAM).
 * 0: GPRAM is enabled and hence CACHE disabled.
 * 1: GPRAM is disabled and instead CACHE is enabled (default).
 * Notes:
 * - Disabling CACHE will reduce CPU execution speed (up to 60%).
 * - GPRAM is 8 K-bytes in size and located at 0x11000000-0x11001fff if
 * enabled.
 * See:
 * VIMS:CTL.MODE
 */

#define CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM                           0x00000004
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM_MASK                      0x00000004
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM_SHIFT                              2

/* Field:     [1] DIS_ALT_DCDC_SETTING
 *
 * Disable alternate DC/DC settings.
 * 0: Enable alternate DC/DC settings.
 * 1: Disable alternate DC/DC settings.
 * See:
 * MODE_CONF_1.ALT_DCDC_VMIN
 * MODE_CONF_1.ALT_DCDC_DITHER_EN
 * MODE_CONF_1.ALT_DCDC_IPEAK
 *
 * NOTE! The DriverLib function SysCtrl_DCDC_VoltageConditionalControl() must
 * be called regularly to apply this field (handled automatically if using TI
 * RTOS!).
 */

#define CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING                0x00000002
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING_MASK           0x00000002
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING_SHIFT                   1

/* Field:     [0] DIS_XOSC_OVR
 *
 * Disable XOSC override functionality.
 * 0: Enable XOSC override functionality.
 * 1: Disable XOSC override functionality.
 * See:
 * MODE_CONF_1.DELTA_IBIAS_INIT
 * MODE_CONF_1.DELTA_IBIAS_OFFSET
 * MODE_CONF_1.XOSC_MAX_START
 */

#define CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR                        0x00000001
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_MASK                   0x00000001
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_SHIFT                           0

/******************************************************************************
 *
 * Register: CCFG_MODE_CONF
 *
 ******************************************************************************
 * Field: [31:28] VDDR_TRIM_SLEEP_DELTA
 *
 * Signed delta value to apply to the
 * VDDR_TRIM_SLEEP target, minus one. See FCFG1:VOLT_TRIM.VDDR_TRIM_SLEEP_H.
 * 0x8 (-8) : Delta = -7
 * ...
 * 0xf (-1) : Delta = 0
 * 0x0 (0) : Delta = +1
 * ...
 * 0x7 (7) : Delta = +8
 */

#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_MASK                   0xf0000000
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_SHIFT                          28

/* Field:    [27] DCDC_RECHARGE
 *
 * DC/DC during recharge in powerdown.
 * 0: Use the DC/DC during recharge in powerdown.
 * 1: Do not use the DC/DC during recharge in powerdown (default).
 *
 * NOTE! The DriverLib function SysCtrl_DCDC_VoltageConditionalControl() must
 * be called regularly to apply this field (handled automatically if using TI
 * RTOS!).
 */

#define CCFG_MODE_CONF_DCDC_RECHARGE                                0x08000000
#define CCFG_MODE_CONF_DCDC_RECHARGE_MASK                           0x08000000
#define CCFG_MODE_CONF_DCDC_RECHARGE_SHIFT                                  27

/* Field:    [26] DCDC_ACTIVE
 *
 * DC/DC in active mode.
 * 0: Use the DC/DC during active mode.
 * 1: Do not use the DC/DC during active mode (default).
 *
 * NOTE! The DriverLib function SysCtrl_DCDC_VoltageConditionalControl() must
 * be called regularly to apply this field (handled automatically if using TI
 * RTOS!).
 */

#define CCFG_MODE_CONF_DCDC_ACTIVE                                  0x04000000
#define CCFG_MODE_CONF_DCDC_ACTIVE_MASK                             0x04000000
#define CCFG_MODE_CONF_DCDC_ACTIVE_SHIFT                                    26

/* Field:    [25] VDDR_EXT_LOAD
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_MODE_CONF_VDDR_EXT_LOAD                                0x02000000
#define CCFG_MODE_CONF_VDDR_EXT_LOAD_MASK                           0x02000000
#define CCFG_MODE_CONF_VDDR_EXT_LOAD_SHIFT                                  25

/* Field:    [24] VDDS_BOD_LEVEL
 *
 * VDDS BOD level.
 * 0: VDDS BOD level is 2.0V (necessary for external load mode, or for maximum
 * PA output power on CC13xx).
 * 1: VDDS BOD level is 1.8V (or 1.65V for external regulator mode) (default).
 */

#define CCFG_MODE_CONF_VDDS_BOD_LEVEL                               0x01000000
#define CCFG_MODE_CONF_VDDS_BOD_LEVEL_MASK                          0x01000000
#define CCFG_MODE_CONF_VDDS_BOD_LEVEL_SHIFT                                 24

/* Field: [23:22] SCLK_LF_OPTION
 *
 * Select source for SCLK_LF.
 * ENUMs:
 * RCOSC_LF                 Low frequency RCOSC (default)
 * XOSC_LF                  32.768kHz low frequency XOSC
 * EXTERNAL_LF              External low frequency clock on DIO defined by
 *                          EXT_LF_CLK.DIO. The RTC tick speed
 *                          AON_RTC:SUBSECINC is updated to
 *                          EXT_LF_CLK.RTC_INCREMENT (done in the
 *                          trimDevice() xxWare boot function). External
 *                          clock must always be running when the chip is
 *                          in standby for VDDR recharge timing.
 * XOSC_HF_DLF              31.25kHz clock derived from 24MHz XOSC (dividing
 *                          by 768 in HW). The RTC tick speed
 *                          [AON_RTC.SUBSECINC.*] is updated to 0x8637bd,
 *                          corresponding to a 31.25kHz clock (done in the
 *                          trimDevice() xxWare boot function). Standby
 *                          power mode is not supported when using this
 *                          clock source.
 */

#define CCFG_MODE_CONF_SCLK_LF_OPTION_MASK                          0x00c00000
#define CCFG_MODE_CONF_SCLK_LF_OPTION_SHIFT                                 22
#define CCFG_MODE_CONF_SCLK_LF_OPTION_RCOSC_LF                      0x00c00000
#define CCFG_MODE_CONF_SCLK_LF_OPTION_XOSC_LF                       0x00800000
#define CCFG_MODE_CONF_SCLK_LF_OPTION_EXTERNAL_LF                   0x00400000
#define CCFG_MODE_CONF_SCLK_LF_OPTION_XOSC_HF_DLF                   0x00000000

/* Field:    [21] VDDR_TRIM_SLEEP_TC
 *
 * 0x1: VDDR_TRIM_SLEEP_DELTA is not temperature compensated
 * 0x0: RTOS/driver temperature compensates VDDR_TRIM_SLEEP_DELTA every time
 * standby mode is entered. This improves low-temperature RCOSC_LF frequency
 * stability in standby mode.
 *
 * When temperature compensation is performed, the delta is calculates this
 * way:
 * Delta = max (delta, min(8, floor(62-temp)/8))
 * Here, delta is given by VDDR_TRIM_SLEEP_DELTA, and temp is the current
 * temperature in degrees C.
 */

#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC                           0x00200000
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC_MASK                      0x00200000
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_TC_SHIFT                             21

/* Field:    [20] RTC_COMP
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_MODE_CONF_RTC_COMP                                     0x00100000
#define CCFG_MODE_CONF_RTC_COMP_MASK                                0x00100000
#define CCFG_MODE_CONF_RTC_COMP_SHIFT                                       20

/* Field: [19:18] XOSC_FREQ
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 * ENUMs:
 * 24M                      24 MHz XOSC_HF
 * 48M                      48 MHz XOSC_HF
 * HPOSC                    HPOSC
 */

#define CCFG_MODE_CONF_XOSC_FREQ_MASK                               0x000c0000
#define CCFG_MODE_CONF_XOSC_FREQ_SHIFT                                      18
#define CCFG_MODE_CONF_XOSC_FREQ_24M                                0x000c0000
#define CCFG_MODE_CONF_XOSC_FREQ_48M                                0x00080000
#define CCFG_MODE_CONF_XOSC_FREQ_HPOSC                              0x00040000

/* Field:    [17] XOSC_CAP_MOD
 *
 * Enable modification (delta) to XOSC cap-array. Value specified in
 * XOSC_CAPARRAY_DELTA.
 * 0: Apply cap-array delta
 * 1: Do not apply cap-array delta (default)
 */

#define CCFG_MODE_CONF_XOSC_CAP_MOD                                 0x00020000
#define CCFG_MODE_CONF_XOSC_CAP_MOD_MASK                            0x00020000
#define CCFG_MODE_CONF_XOSC_CAP_MOD_SHIFT                                   17

/* Field:    [16] HF_COMP
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_MODE_CONF_HF_COMP                                      0x00010000
#define CCFG_MODE_CONF_HF_COMP_MASK                                 0x00010000
#define CCFG_MODE_CONF_HF_COMP_SHIFT                                        16

/* Field:  [15:8] XOSC_CAPARRAY_DELTA
 *
 * Signed 8-bit value, directly modifying trimmed XOSC cap-array step value.
 * Enabled by XOSC_CAP_MOD.
 */

#define CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_MASK                     0x0000ff00
#define CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_SHIFT                             8

/* Field:   [7:0] VDDR_CAP
 *
 * Unsigned 8-bit integer, representing the minimum decoupling capacitance
 * (worst case) on VDDR, in units of 100nF. This should take into account
 * capacitor tolerance and voltage dependent capacitance variation. This bit
 * affects the recharge period calculation when going into powerdown or
 * standby.
 *
 * NOTE! If using the following functions this field must be configured (used
 * by TI RTOS):
 * SysCtrlSetRechargeBeforePowerDown() SysCtrlAdjustRechargeAfterPowerDown()
 */

#define CCFG_MODE_CONF_VDDR_CAP_MASK                                0x000000ff
#define CCFG_MODE_CONF_VDDR_CAP_SHIFT                                        0

/******************************************************************************
 *
 * Register: CCFG_VOLT_LOAD_0
 *
 ******************************************************************************
 * Field: [31:24] VDDR_EXT_TP45
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_MASK                         0xff000000
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_SHIFT                                24

/* Field: [23:16] VDDR_EXT_TP25
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_MASK                         0x00ff0000
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_SHIFT                                16

/* Field:  [15:8] VDDR_EXT_TP5
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_MASK                          0x0000ff00
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_SHIFT                                  8

/* Field:   [7:0] VDDR_EXT_TM15
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_MASK                         0x000000ff
#define CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_SHIFT                                 0

/******************************************************************************
 *
 * Register: CCFG_VOLT_LOAD_1
 *
 ******************************************************************************
 * Field: [31:24] VDDR_EXT_TP125
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_MASK                        0xff000000
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_SHIFT                               24

/* Field: [23:16] VDDR_EXT_TP105
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_MASK                        0x00ff0000
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_SHIFT                               16

/* Field:  [15:8] VDDR_EXT_TP85
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_MASK                         0x0000ff00
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_SHIFT                                 8

/* Field:   [7:0] VDDR_EXT_TP65
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_MASK                         0x000000ff
#define CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_SHIFT                                 0

/******************************************************************************
 *
 * Register: CCFG_RTC_OFFSET
 *
 ******************************************************************************
 * Field: [31:16] RTC_COMP_P0
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_RTC_OFFSET_RTC_COMP_P0_MASK                            0xffff0000
#define CCFG_RTC_OFFSET_RTC_COMP_P0_SHIFT                                   16

/* Field:  [15:8] RTC_COMP_P1
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_RTC_OFFSET_RTC_COMP_P1_MASK                            0x0000ff00
#define CCFG_RTC_OFFSET_RTC_COMP_P1_SHIFT                                    8

/* Field:   [7:0] RTC_COMP_P2
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_RTC_OFFSET_RTC_COMP_P2_MASK                            0x000000ff
#define CCFG_RTC_OFFSET_RTC_COMP_P2_SHIFT                                    0

/******************************************************************************
 *
 * Register: CCFG_FREQ_OFFSET
 *
 ******************************************************************************
 * Field: [31:16] HF_COMP_P0
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_FREQ_OFFSET_HF_COMP_P0_MASK                            0xffff0000
#define CCFG_FREQ_OFFSET_HF_COMP_P0_SHIFT                                   16

/* Field:  [15:8] HF_COMP_P1
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_FREQ_OFFSET_HF_COMP_P1_MASK                            0x0000ff00
#define CCFG_FREQ_OFFSET_HF_COMP_P1_SHIFT                                    8

/* Field:   [7:0] HF_COMP_P2
 *
 * Reserved for future use. Software should not rely on the value of a
 * reserved. Writing any other value than the reset/default value may result in
 * undefined behavior.
 */

#define CCFG_FREQ_OFFSET_HF_COMP_P2_MASK                            0x000000ff
#define CCFG_FREQ_OFFSET_HF_COMP_P2_SHIFT                                    0

/******************************************************************************
 *
 * Register: CCFG_IEEE_MAC_0
 *
 ******************************************************************************
 * Field:  [31:0] ADDR
 *
 * Bits[31:0] of the 64-bits custom IEEE MAC address.
 * If different from 0xffffffff then the value of this field is applied;
 * otherwise use value from FCFG.
 */

#define CCFG_IEEE_MAC_0_ADDR_MASK                                   0xffffffff
#define CCFG_IEEE_MAC_0_ADDR_SHIFT                                           0

/******************************************************************************
 *
 * Register: CCFG_IEEE_MAC_1
 *
 ******************************************************************************
 * Field:  [31:0] ADDR
 *
 * Bits[63:32] of the 64-bits custom IEEE MAC address.
 * If different from 0xffffffff then the value of this field is applied;
 * otherwise use value from FCFG.
 */

#define CCFG_IEEE_MAC_1_ADDR_MASK                                   0xffffffff
#define CCFG_IEEE_MAC_1_ADDR_SHIFT                                           0

/******************************************************************************
 *
 * Register: CCFG_IEEE_BLE_0
 *
 ******************************************************************************
 * Field:  [31:0] ADDR
 *
 * Bits[31:0] of the 64-bits custom IEEE BLE address.
 * If different from 0xffffffff then the value of this field is applied;
 * otherwise use value from FCFG.
 */

#define CCFG_IEEE_BLE_0_ADDR_MASK                                   0xffffffff
#define CCFG_IEEE_BLE_0_ADDR_SHIFT                                           0

/******************************************************************************
 *
 * Register: CCFG_IEEE_BLE_1
 *
 ******************************************************************************
 * Field:  [31:0] ADDR
 *
 * Bits[63:32] of the 64-bits custom IEEE BLE address.
 * If different from 0xffffffff then the value of this field is applied;
 * otherwise use value from FCFG.
 */

#define CCFG_IEEE_BLE_1_ADDR_MASK                                   0xffffffff
#define CCFG_IEEE_BLE_1_ADDR_SHIFT                                           0

/******************************************************************************
 *
 * Register: CCFG_BL_CONFIG
 *
 ******************************************************************************
 * Field: [31:24] BOOTLOADER_ENABLE
 *
 * Bootloader enable. Boot loader can be accessed if
 * IMAGE_VALID_CONF.IMAGE_VALID is non-zero or BL_ENABLE is enabled (and
 * conditions for boot loader backdoor are met).
 * 0xc5: Boot loader is enabled.
 * Any other value: Boot loader is disabled.
 */

#define CCFG_BL_CONFIG_BOOTLOADER_ENABLE_MASK                       0xff000000
#define CCFG_BL_CONFIG_BOOTLOADER_ENABLE_SHIFT                              24

/* Field:    [16] BL_LEVEL
 *
 * Sets the active level of the selected DIO number BL_PIN_NUMBER if boot
 * loader backdoor is enabled by the BL_ENABLE field.
 * 0: Active low.
 * 1: Active high.
 */

#define CCFG_BL_CONFIG_BL_LEVEL                                     0x00010000
#define CCFG_BL_CONFIG_BL_LEVEL_MASK                                0x00010000
#define CCFG_BL_CONFIG_BL_LEVEL_SHIFT                                       16

/* Field:  [15:8] BL_PIN_NUMBER
 *
 * DIO number that is level checked if the boot loader backdoor is enabled by
 * the BL_ENABLE field.
 */

#define CCFG_BL_CONFIG_BL_PIN_NUMBER_MASK                           0x0000ff00
#define CCFG_BL_CONFIG_BL_PIN_NUMBER_SHIFT                                   8

/* Field:   [7:0] BL_ENABLE
 *
 * Enables the boot loader backdoor.
 * 0xc5: Boot loader backdoor is enabled.
 * Any other value: Boot loader backdoor is disabled.
 *
 * NOTE! Boot loader must be enabled (see BOOTLOADER_ENABLE) if boot loader
 * backdoor is enabled.
 */

#define CCFG_BL_CONFIG_BL_ENABLE_MASK                               0x000000ff
#define CCFG_BL_CONFIG_BL_ENABLE_SHIFT                                       0

/******************************************************************************
 *
 * Register: CCFG_ERASE_CONF
 *
 ******************************************************************************
 * Field:     [8] CHIP_ERASE_DIS_N
 *
 * Chip erase.
 * This bit controls if a chip erase requested through the JTAG WUC TAP will be
 * ignored in a following boot caused by a reset of the MCU VD.
 * A successful chip erase operation will force the content of the flash main
 * bank back to the state as it was when delivered by TI.
 * 0: Disable. Any chip erase request detected during boot will be ignored.
 * 1: Enable. Any chip erase request detected during boot will be performed by
 * the boot FW.
 */

#define CCFG_ERASE_CONF_CHIP_ERASE_DIS_N                            0x00000100
#define CCFG_ERASE_CONF_CHIP_ERASE_DIS_N_MASK                       0x00000100
#define CCFG_ERASE_CONF_CHIP_ERASE_DIS_N_SHIFT                               8

/* Field:     [0] BANK_ERASE_DIS_N
 *
 * Bank erase.
 * This bit controls if the ROM serial boot loader will accept a received Bank
 * Erase command (COMMAND_BANK_ERASE).
 * A successful Bank Erase operation will erase all main bank sectors not
 * protected by write protect configuration bits in CCFG.
 * 0: Disable the boot loader bank erase function.
 * 1: Enable the boot loader bank erase function.
 */

#define CCFG_ERASE_CONF_BANK_ERASE_DIS_N                            0x00000001
#define CCFG_ERASE_CONF_BANK_ERASE_DIS_N_MASK                       0x00000001
#define CCFG_ERASE_CONF_BANK_ERASE_DIS_N_SHIFT                               0

/******************************************************************************
 *
 * Register: CCFG_CCFG_TI_OPTIONS
 *
 ******************************************************************************
 * Field:   [7:0] TI_FA_ENABLE
 *
 * TI Failure Analysis.
 * 0xc5: Enable the functionality of unlocking the TI FA (TI Failure Analysis)
 * option with the unlock code.
 * All other values: Disable the functionality of unlocking the TI FA option
 * with the unlock code.
 */

#define CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_MASK                      0x000000ff
#define CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_SHIFT                              0

/******************************************************************************
 *
 * Register: CCFG_CCFG_TAP_DAP_0
 *
 ******************************************************************************
 * Field: [23:16] CPU_DAP_ENABLE
 *
 * Enable CPU DAP.
 * 0xc5: Main CPU DAP access is enabled during power-up/system-reset by ROM
 * boot FW.
 * Any other value: Main CPU DAP access will remain disabled out of
 * power-up/system-reset.
 */

#define CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_MASK                     0x00ff0000
#define CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_SHIFT                            16

/* Field:  [15:8] PWRPROF_TAP_ENABLE
 *
 * Enable PWRPROF TAP.
 * 0xc5: PWRPROF TAP access is enabled during power-up/system-reset by ROM boot
 * FW if enabled by corresponding configuration value in FCFG1 defined by TI.
 * Any other value: PWRPROF TAP access will remain disabled out of
 * power-up/system-reset.
 */

#define CCFG_CCFG_TAP_DAP_0_PWRPROF_TAP_ENABLE_MASK                 0x0000ff00
#define CCFG_CCFG_TAP_DAP_0_PWRPROF_TAP_ENABLE_SHIFT                         8

/* Field:   [7:0] TEST_TAP_ENABLE
 *
 * Enable Test TAP.
 * 0xc5: TEST TAP access is enabled during power-up/system-reset by ROM boot FW
 * if enabled by corresponding configuration value in FCFG1 defined by TI.
 * Any other value: TEST TAP access will remain disabled out of
 * power-up/system-reset.
 */

#define CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_MASK                    0x000000ff
#define CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_SHIFT                            0

/******************************************************************************
 *
 * Register: CCFG_CCFG_TAP_DAP_1
 *
 ******************************************************************************
 * Field: [23:16] PBIST2_TAP_ENABLE
 *
 * Enable PBIST2 TAP.
 * 0xc5: PBIST2 TAP access is enabled during power-up/system-reset by ROM boot
 * FW if enabled by corresponding configuration value in FCFG1 defined by TI.
 * Any other value: PBIST2 TAP access will remain disabled out of
 * power-up/system-reset.
 */

#define CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_MASK                  0x00ff0000
#define CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_SHIFT                         16

/* Field:  [15:8] PBIST1_TAP_ENABLE
 *
 * Enable PBIST1 TAP.
 * 0xc5: PBIST1 TAP access is enabled during power-up/system-reset by ROM boot
 * FW if enabled by corresponding configuration value in FCFG1 defined by TI.
 * Any other value: PBIST1 TAP access will remain disabled out of
 * power-up/system-reset.
 */

#define CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_MASK                  0x0000ff00
#define CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_SHIFT                          8

/* Field:   [7:0] AON_TAP_ENABLE
 *
 * Enable AON TAP
 * 0xc5: AON TAP access is enabled during power-up/system-reset by ROM boot FW
 * if enabled by corresponding configuration value in FCFG1 defined by TI.
 * Any other value: AON TAP access will remain disabled out of
 * power-up/system-reset.
 */

#define CCFG_CCFG_TAP_DAP_1_AON_TAP_ENABLE_MASK                     0x000000ff
#define CCFG_CCFG_TAP_DAP_1_AON_TAP_ENABLE_SHIFT                             0

/******************************************************************************
 *
 * Register: CCFG_IMAGE_VALID_CONF
 *
 ******************************************************************************
 * Field:  [31:0] IMAGE_VALID
 *
 * This field must have the address value of the start of the flash vector
 * table in order to enable the boot FW in ROM to transfer control to a flash
 * image.
 * Any illegal vector table start address value will force the boot FW in ROM
 * to transfer control to the serial boot loader in ROM.
 */

#define CCFG_IMAGE_VALID_CONF_IMAGE_VALID_MASK                      0xffffffff
#define CCFG_IMAGE_VALID_CONF_IMAGE_VALID_SHIFT                              0

/******************************************************************************
 *
 * Register: CCFG_CCFG_PROT_31_0
 *
 ******************************************************************************
 * Field:    [31] WRT_PROT_SEC_31
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_31                         0x80000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_31_MASK                    0x80000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_31_SHIFT                           31

/* Field:    [30] WRT_PROT_SEC_30
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_30                         0x40000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_30_MASK                    0x40000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_30_SHIFT                           30

/* Field:    [29] WRT_PROT_SEC_29
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_29                         0x20000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_29_MASK                    0x20000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_29_SHIFT                           29

/* Field:    [28] WRT_PROT_SEC_28
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_28                         0x10000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_28_MASK                    0x10000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_28_SHIFT                           28

/* Field:    [27] WRT_PROT_SEC_27
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_27                         0x08000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_27_MASK                    0x08000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_27_SHIFT                           27

/* Field:    [26] WRT_PROT_SEC_26
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_26                         0x04000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_26_MASK                    0x04000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_26_SHIFT                           26

/* Field:    [25] WRT_PROT_SEC_25
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_25                         0x02000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_25_MASK                    0x02000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_25_SHIFT                           25

/* Field:    [24] WRT_PROT_SEC_24
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_24                         0x01000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_24_MASK                    0x01000000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_24_SHIFT                           24

/* Field:    [23] WRT_PROT_SEC_23
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_23                         0x00800000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_23_MASK                    0x00800000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_23_SHIFT                           23

/* Field:    [22] WRT_PROT_SEC_22
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_22                         0x00400000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_22_MASK                    0x00400000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_22_SHIFT                           22

/* Field:    [21] WRT_PROT_SEC_21
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_21                         0x00200000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_21_MASK                    0x00200000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_21_SHIFT                           21

/* Field:    [20] WRT_PROT_SEC_20
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_20                         0x00100000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_20_MASK                    0x00100000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_20_SHIFT                           20

/* Field:    [19] WRT_PROT_SEC_19
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_19                         0x00080000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_19_MASK                    0x00080000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_19_SHIFT                           19

/* Field:    [18] WRT_PROT_SEC_18
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_18                         0x00040000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_18_MASK                    0x00040000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_18_SHIFT                           18

/* Field:    [17] WRT_PROT_SEC_17
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_17                         0x00020000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_17_MASK                    0x00020000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_17_SHIFT                           17

/* Field:    [16] WRT_PROT_SEC_16
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_16                         0x00010000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_16_MASK                    0x00010000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_16_SHIFT                           16

/* Field:    [15] WRT_PROT_SEC_15
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_15                         0x00008000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_15_MASK                    0x00008000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_15_SHIFT                           15

/* Field:    [14] WRT_PROT_SEC_14
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_14                         0x00004000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_14_MASK                    0x00004000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_14_SHIFT                           14

/* Field:    [13] WRT_PROT_SEC_13
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_13                         0x00002000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_13_MASK                    0x00002000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_13_SHIFT                           13

/* Field:    [12] WRT_PROT_SEC_12
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_12                         0x00001000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_12_MASK                    0x00001000
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_12_SHIFT                           12

/* Field:    [11] WRT_PROT_SEC_11
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_11                         0x00000800
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_11_MASK                    0x00000800
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_11_SHIFT                           11

/* Field:    [10] WRT_PROT_SEC_10
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_10                         0x00000400
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_10_MASK                    0x00000400
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_10_SHIFT                           10

/* Field:     [9] WRT_PROT_SEC_9
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_9                          0x00000200
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_9_MASK                     0x00000200
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_9_SHIFT                             9

/* Field:     [8] WRT_PROT_SEC_8
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_8                          0x00000100
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_8_MASK                     0x00000100
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_8_SHIFT                             8

/* Field:     [7] WRT_PROT_SEC_7
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_7                          0x00000080
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_7_MASK                     0x00000080
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_7_SHIFT                             7

/* Field:     [6] WRT_PROT_SEC_6
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_6                          0x00000040
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_6_MASK                     0x00000040
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_6_SHIFT                             6

/* Field:     [5] WRT_PROT_SEC_5
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_5                          0x00000020
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_5_MASK                     0x00000020
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_5_SHIFT                             5

/* Field:     [4] WRT_PROT_SEC_4
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_4                          0x00000010
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_4_MASK                     0x00000010
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_4_SHIFT                             4

/* Field:     [3] WRT_PROT_SEC_3
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_3                          0x00000008
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_3_MASK                     0x00000008
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_3_SHIFT                             3

/* Field:     [2] WRT_PROT_SEC_2
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_2                          0x00000004
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_2_MASK                     0x00000004
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_2_SHIFT                             2

/* Field:     [1] WRT_PROT_SEC_1
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_1                          0x00000002
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_1_MASK                     0x00000002
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_1_SHIFT                             1

/* Field:     [0] WRT_PROT_SEC_0
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_0                          0x00000001
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_0_MASK                     0x00000001
#define CCFG_CCFG_PROT_31_0_WRT_PROT_SEC_0_SHIFT                             0

/******************************************************************************
 *
 * Register: CCFG_CCFG_PROT_63_32
 *
 ******************************************************************************
 * Field:    [31] WRT_PROT_SEC_63
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_63                        0x80000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_63_MASK                   0x80000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_63_SHIFT                          31

/* Field:    [30] WRT_PROT_SEC_62
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_62                        0x40000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_62_MASK                   0x40000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_62_SHIFT                          30

/* Field:    [29] WRT_PROT_SEC_61
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_61                        0x20000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_61_MASK                   0x20000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_61_SHIFT                          29

/* Field:    [28] WRT_PROT_SEC_60
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_60                        0x10000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_60_MASK                   0x10000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_60_SHIFT                          28

/* Field:    [27] WRT_PROT_SEC_59
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_59                        0x08000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_59_MASK                   0x08000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_59_SHIFT                          27

/* Field:    [26] WRT_PROT_SEC_58
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_58                        0x04000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_58_MASK                   0x04000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_58_SHIFT                          26

/* Field:    [25] WRT_PROT_SEC_57
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_57                        0x02000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_57_MASK                   0x02000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_57_SHIFT                          25

/* Field:    [24] WRT_PROT_SEC_56
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_56                        0x01000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_56_MASK                   0x01000000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_56_SHIFT                          24

/* Field:    [23] WRT_PROT_SEC_55
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_55                        0x00800000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_55_MASK                   0x00800000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_55_SHIFT                          23

/* Field:    [22] WRT_PROT_SEC_54
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_54                        0x00400000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_54_MASK                   0x00400000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_54_SHIFT                          22

/* Field:    [21] WRT_PROT_SEC_53
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_53                        0x00200000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_53_MASK                   0x00200000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_53_SHIFT                          21

/* Field:    [20] WRT_PROT_SEC_52
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_52                        0x00100000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_52_MASK                   0x00100000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_52_SHIFT                          20

/* Field:    [19] WRT_PROT_SEC_51
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_51                        0x00080000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_51_MASK                   0x00080000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_51_SHIFT                          19

/* Field:    [18] WRT_PROT_SEC_50
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_50                        0x00040000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_50_MASK                   0x00040000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_50_SHIFT                          18

/* Field:    [17] WRT_PROT_SEC_49
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_49                        0x00020000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_49_MASK                   0x00020000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_49_SHIFT                          17

/* Field:    [16] WRT_PROT_SEC_48
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_48                        0x00010000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_48_MASK                   0x00010000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_48_SHIFT                          16

/* Field:    [15] WRT_PROT_SEC_47
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_47                        0x00008000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_47_MASK                   0x00008000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_47_SHIFT                          15

/* Field:    [14] WRT_PROT_SEC_46
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_46                        0x00004000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_46_MASK                   0x00004000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_46_SHIFT                          14

/* Field:    [13] WRT_PROT_SEC_45
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_45                        0x00002000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_45_MASK                   0x00002000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_45_SHIFT                          13

/* Field:    [12] WRT_PROT_SEC_44
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_44                        0x00001000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_44_MASK                   0x00001000
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_44_SHIFT                          12

/* Field:    [11] WRT_PROT_SEC_43
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_43                        0x00000800
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_43_MASK                   0x00000800
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_43_SHIFT                          11

/* Field:    [10] WRT_PROT_SEC_42
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_42                        0x00000400
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_42_MASK                   0x00000400
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_42_SHIFT                          10

/* Field:     [9] WRT_PROT_SEC_41
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_41                        0x00000200
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_41_MASK                   0x00000200
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_41_SHIFT                           9

/* Field:     [8] WRT_PROT_SEC_40
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_40                        0x00000100
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_40_MASK                   0x00000100
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_40_SHIFT                           8

/* Field:     [7] WRT_PROT_SEC_39
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_39                        0x00000080
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_39_MASK                   0x00000080
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_39_SHIFT                           7

/* Field:     [6] WRT_PROT_SEC_38
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_38                        0x00000040
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_38_MASK                   0x00000040
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_38_SHIFT                           6

/* Field:     [5] WRT_PROT_SEC_37
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_37                        0x00000020
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_37_MASK                   0x00000020
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_37_SHIFT                           5

/* Field:     [4] WRT_PROT_SEC_36
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_36                        0x00000010
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_36_MASK                   0x00000010
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_36_SHIFT                           4

/* Field:     [3] WRT_PROT_SEC_35
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_35                        0x00000008
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_35_MASK                   0x00000008
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_35_SHIFT                           3

/* Field:     [2] WRT_PROT_SEC_34
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_34                        0x00000004
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_34_MASK                   0x00000004
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_34_SHIFT                           2

/* Field:     [1] WRT_PROT_SEC_33
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_33                        0x00000002
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_33_MASK                   0x00000002
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_33_SHIFT                           1

/* Field:     [0] WRT_PROT_SEC_32
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_32                        0x00000001
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_32_MASK                   0x00000001
#define CCFG_CCFG_PROT_63_32_WRT_PROT_SEC_32_SHIFT                           0

/******************************************************************************
 *
 * Register: CCFG_CCFG_PROT_95_64
 *
 ******************************************************************************
 * Field:    [31] WRT_PROT_SEC_95
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_95                        0x80000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_95_MASK                   0x80000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_95_SHIFT                          31

/* Field:    [30] WRT_PROT_SEC_94
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_94                        0x40000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_94_MASK                   0x40000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_94_SHIFT                          30

/* Field:    [29] WRT_PROT_SEC_93
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_93                        0x20000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_93_MASK                   0x20000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_93_SHIFT                          29

/* Field:    [28] WRT_PROT_SEC_92
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_92                        0x10000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_92_MASK                   0x10000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_92_SHIFT                          28

/* Field:    [27] WRT_PROT_SEC_91
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_91                        0x08000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_91_MASK                   0x08000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_91_SHIFT                          27

/* Field:    [26] WRT_PROT_SEC_90
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_90                        0x04000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_90_MASK                   0x04000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_90_SHIFT                          26

/* Field:    [25] WRT_PROT_SEC_89
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_89                        0x02000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_89_MASK                   0x02000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_89_SHIFT                          25

/* Field:    [24] WRT_PROT_SEC_88
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_88                        0x01000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_88_MASK                   0x01000000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_88_SHIFT                          24

/* Field:    [23] WRT_PROT_SEC_87
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_87                        0x00800000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_87_MASK                   0x00800000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_87_SHIFT                          23

/* Field:    [22] WRT_PROT_SEC_86
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_86                        0x00400000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_86_MASK                   0x00400000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_86_SHIFT                          22

/* Field:    [21] WRT_PROT_SEC_85
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_85                        0x00200000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_85_MASK                   0x00200000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_85_SHIFT                          21

/* Field:    [20] WRT_PROT_SEC_84
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_84                        0x00100000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_84_MASK                   0x00100000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_84_SHIFT                          20

/* Field:    [19] WRT_PROT_SEC_83
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_83                        0x00080000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_83_MASK                   0x00080000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_83_SHIFT                          19

/* Field:    [18] WRT_PROT_SEC_82
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_82                        0x00040000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_82_MASK                   0x00040000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_82_SHIFT                          18

/* Field:    [17] WRT_PROT_SEC_81
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_81                        0x00020000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_81_MASK                   0x00020000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_81_SHIFT                          17

/* Field:    [16] WRT_PROT_SEC_80
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_80                        0x00010000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_80_MASK                   0x00010000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_80_SHIFT                          16

/* Field:    [15] WRT_PROT_SEC_79
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_79                        0x00008000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_79_MASK                   0x00008000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_79_SHIFT                          15

/* Field:    [14] WRT_PROT_SEC_78
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_78                        0x00004000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_78_MASK                   0x00004000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_78_SHIFT                          14

/* Field:    [13] WRT_PROT_SEC_77
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_77                        0x00002000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_77_MASK                   0x00002000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_77_SHIFT                          13

/* Field:    [12] WRT_PROT_SEC_76
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_76                        0x00001000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_76_MASK                   0x00001000
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_76_SHIFT                          12

/* Field:    [11] WRT_PROT_SEC_75
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_75                        0x00000800
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_75_MASK                   0x00000800
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_75_SHIFT                          11

/* Field:    [10] WRT_PROT_SEC_74
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_74                        0x00000400
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_74_MASK                   0x00000400
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_74_SHIFT                          10

/* Field:     [9] WRT_PROT_SEC_73
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_73                        0x00000200
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_73_MASK                   0x00000200
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_73_SHIFT                           9

/* Field:     [8] WRT_PROT_SEC_72
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_72                        0x00000100
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_72_MASK                   0x00000100
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_72_SHIFT                           8

/* Field:     [7] WRT_PROT_SEC_71
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_71                        0x00000080
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_71_MASK                   0x00000080
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_71_SHIFT                           7

/* Field:     [6] WRT_PROT_SEC_70
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_70                        0x00000040
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_70_MASK                   0x00000040
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_70_SHIFT                           6

/* Field:     [5] WRT_PROT_SEC_69
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_69                        0x00000020
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_69_MASK                   0x00000020
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_69_SHIFT                           5

/* Field:     [4] WRT_PROT_SEC_68
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_68                        0x00000010
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_68_MASK                   0x00000010
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_68_SHIFT                           4

/* Field:     [3] WRT_PROT_SEC_67
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_67                        0x00000008
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_67_MASK                   0x00000008
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_67_SHIFT                           3

/* Field:     [2] WRT_PROT_SEC_66
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_66                        0x00000004
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_66_MASK                   0x00000004
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_66_SHIFT                           2

/* Field:     [1] WRT_PROT_SEC_65
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_65                        0x00000002
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_65_MASK                   0x00000002
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_65_SHIFT                           1

/* Field:     [0] WRT_PROT_SEC_64
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_64                        0x00000001
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_64_MASK                   0x00000001
#define CCFG_CCFG_PROT_95_64_WRT_PROT_SEC_64_SHIFT                           0

/******************************************************************************
 *
 * Register: CCFG_CCFG_PROT_127_96
 *
 ******************************************************************************
 * Field:    [31] WRT_PROT_SEC_127
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_127                      0x80000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_127_MASK                 0x80000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_127_SHIFT                        31

/* Field:    [30] WRT_PROT_SEC_126
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_126                      0x40000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_126_MASK                 0x40000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_126_SHIFT                        30

/* Field:    [29] WRT_PROT_SEC_125
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_125                      0x20000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_125_MASK                 0x20000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_125_SHIFT                        29

/* Field:    [28] WRT_PROT_SEC_124
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_124                      0x10000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_124_MASK                 0x10000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_124_SHIFT                        28

/* Field:    [27] WRT_PROT_SEC_123
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_123                      0x08000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_123_MASK                 0x08000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_123_SHIFT                        27

/* Field:    [26] WRT_PROT_SEC_122
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_122                      0x04000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_122_MASK                 0x04000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_122_SHIFT                        26

/* Field:    [25] WRT_PROT_SEC_121
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_121                      0x02000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_121_MASK                 0x02000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_121_SHIFT                        25

/* Field:    [24] WRT_PROT_SEC_120
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_120                      0x01000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_120_MASK                 0x01000000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_120_SHIFT                        24

/* Field:    [23] WRT_PROT_SEC_119
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_119                      0x00800000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_119_MASK                 0x00800000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_119_SHIFT                        23

/* Field:    [22] WRT_PROT_SEC_118
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_118                      0x00400000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_118_MASK                 0x00400000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_118_SHIFT                        22

/* Field:    [21] WRT_PROT_SEC_117
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_117                      0x00200000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_117_MASK                 0x00200000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_117_SHIFT                        21

/* Field:    [20] WRT_PROT_SEC_116
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_116                      0x00100000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_116_MASK                 0x00100000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_116_SHIFT                        20

/* Field:    [19] WRT_PROT_SEC_115
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_115                      0x00080000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_115_MASK                 0x00080000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_115_SHIFT                        19

/* Field:    [18] WRT_PROT_SEC_114
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_114                      0x00040000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_114_MASK                 0x00040000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_114_SHIFT                        18

/* Field:    [17] WRT_PROT_SEC_113
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_113                      0x00020000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_113_MASK                 0x00020000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_113_SHIFT                        17

/* Field:    [16] WRT_PROT_SEC_112
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_112                      0x00010000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_112_MASK                 0x00010000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_112_SHIFT                        16

/* Field:    [15] WRT_PROT_SEC_111
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_111                      0x00008000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_111_MASK                 0x00008000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_111_SHIFT                        15

/* Field:    [14] WRT_PROT_SEC_110
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_110                      0x00004000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_110_MASK                 0x00004000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_110_SHIFT                        14

/* Field:    [13] WRT_PROT_SEC_109
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_109                      0x00002000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_109_MASK                 0x00002000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_109_SHIFT                        13

/* Field:    [12] WRT_PROT_SEC_108
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_108                      0x00001000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_108_MASK                 0x00001000
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_108_SHIFT                        12

/* Field:    [11] WRT_PROT_SEC_107
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_107                      0x00000800
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_107_MASK                 0x00000800
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_107_SHIFT                        11

/* Field:    [10] WRT_PROT_SEC_106
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_106                      0x00000400
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_106_MASK                 0x00000400
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_106_SHIFT                        10

/* Field:     [9] WRT_PROT_SEC_105
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_105                      0x00000200
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_105_MASK                 0x00000200
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_105_SHIFT                         9

/* Field:     [8] WRT_PROT_SEC_104
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_104                      0x00000100
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_104_MASK                 0x00000100
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_104_SHIFT                         8

/* Field:     [7] WRT_PROT_SEC_103
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_103                      0x00000080
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_103_MASK                 0x00000080
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_103_SHIFT                         7

/* Field:     [6] WRT_PROT_SEC_102
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_102                      0x00000040
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_102_MASK                 0x00000040
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_102_SHIFT                         6

/* Field:     [5] WRT_PROT_SEC_101
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_101                      0x00000020
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_101_MASK                 0x00000020
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_101_SHIFT                         5

/* Field:     [4] WRT_PROT_SEC_100
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_100                      0x00000010
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_100_MASK                 0x00000010
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_100_SHIFT                         4

/* Field:     [3] WRT_PROT_SEC_99
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_99                       0x00000008
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_99_MASK                  0x00000008
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_99_SHIFT                          3

/* Field:     [2] WRT_PROT_SEC_98
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_98                       0x00000004
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_98_MASK                  0x00000004
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_98_SHIFT                          2

/* Field:     [1] WRT_PROT_SEC_97
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_97                       0x00000002
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_97_MASK                  0x00000002
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_97_SHIFT                          1

/* Field:     [0] WRT_PROT_SEC_96
 *
 * 0: Sector protected
 */

#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_96                       0x00000001
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_96_MASK                  0x00000001
#define CCFG_CCFG_PROT_127_96_WRT_PROT_SEC_96_SHIFT                          0

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_V2_CCFG_H */
