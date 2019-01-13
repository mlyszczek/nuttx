/**************************************************************************************
 * drivers/lcd/tda19988.h
 * Definitions for the TDA19988.  The TDA19988 is a very low power and very small
 * size High-Definition Multimedia Interface (HDMI) 1.4a transmitter
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 **************************************************************************************/

#ifndef __DRIVERS_LCD_TDA19988_H
#define __DRIVERS_LCD_TDA19988_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* CEC Registers **********************************************************************/
/* The device has two I2C interfaces CEC (0x34) and HDMI (0x70). */

#define CEC_STATUS_REG             0xfe
#  define CEC_STATUS_CONNECTED     (1 << 1)

#define CEC_ENABLE_REG             0xff
#  define CEC_ENABLE_ALL           0x87

/* HDMI Memory Pages ******************************************************************/
/* HDMI Memory is accessed via page and address.  The page must first be selected, then
 * only the address is sent in order accessing memory locations within the selected
 * page.
 */
 
#define HDMI_CTRL_PAGE             0x00  /* General control page */
#define HDMI_PLL_PAGE              0x02  /* PLL settings page */
#define HDMI_EDID_PAGE             0x09  /* EDID control page */
#define HDMI_INFO_PAGE             0x10  /* Information frames and packets page */
#define HDMI_AUDIO_PAGE            0x11  /* Audio settings and content info packets page */
#define HDMI_HDCPOTP_PAGE          0x12  /* HDCP (TDA19988AHN and TDA19988AET only) and OTP */
#define HDMI_GAMUT_PAGE            0x13  /* Gamut-related metadata packets page */

/* The page select register does not lie within the above pages. The value of 0xff is
 * used for this access.
 */

#define HDMI_NO_PAGE               0xff

/* General Control Page Registers and Bit Definitions */

#define HDMI_CTRL_REV_LO_REG       0x00
#define HDMI_CTRL_REV_HI_REG       0x02
#  define HDMI_REV_TDA19988        0x0331

#define HDMI_CTRL_RESET_REG        0x0a
#  define HDMI_CTRL_RESET_DDC      (1 << 1)

#define HDMI_CTRL_DDC_CTRL_REG     0x0b
#  define HDMI_CTRL_DDC_EN         0x00

#define HDMI_CTRL_DDC_CLK_REG      0x0c
#  define HDMI_CTRL_DDC_CLK_EN     (1 << 0)

#define HDMI_CTRL_INTR_CTRL_REG    0x0f
#  define HDMI_CTRL_INTR_EN_GLO    (1 << 2)

#define HDMI_CTRL_INT_REG          0x11
#  define HDMI_CTRL_INT_EDID       (1 << 1)

/* EDID Control Page Registers and Bit Definitions */

#define HDMI_EDID_DATA_REG         0x00

#define HDMI_EDID_DEV_ADDR_REG     0xfb
#  define HDMI_EDID_DEV_ADDR       0xa0

#define HDMI_EDID_OFFSET_REG       0xfc
#  define HDMI_EDID_OFFSET         0x00

#define HDMI_EDID_SEG_PTR_ADDR_REG 0xfc
#define HDMI_EDID_SEG_PTR_ADDR     0x00

#define HDMI_EDID_SEG_ADDR_REG     0xfe
#  define HDMI_EDID_SEG_ADDR 0x00

#define HDMI_EDID_REQ_REG          0xfa
#  define HDMI_EDID_REQ_READ       (1 << 0)

/* HDCP (TDA19988AHN and TDA19988AET only) and OTP Page Registers and Bit
 * Definitions.
 */

#define HDMI_HDCP_OTP_DDC_CLK_REG  0x9a
#  define HDMI_HDCP_OTP_DDC_CLK    0x27

#define HDMI_HDCP_OTP_SOME_REG     0x9b
#  define HDMI_HDCP_OTP_SOME       0x02

/* Page Select Register (no page) */

#define HDMI_PAGE_SELECT_REG       0xff

#endif /* __DRIVERS_LCD_TDA19988_H */
