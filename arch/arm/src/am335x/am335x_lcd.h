/****************************************************************************
 * arch/arm/src/am335x/am335x_lcd.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The LCD driver derives from the LPC54xx LCD driver but also includes
 * information from the FreeBSD AM335x LCD driver which was released under
 * a two-clause BSD license:
 *
 *   Copyright 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
 *   All rights reserved.
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
 ****************************************************************************/

/* The LPC54 LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_LCD_H
#define __ARCH_ARM_SRC_AM335X_AM335X_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/am335x_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

/* Base address of the video RAM frame buffer */

#ifndef CONFIG_AM335X_LCD_VRAMBASE
#  define CONFIG_AM335X_LCD_VRAMBASE ((uint32_t)AM335X_EXTDRAM_CS0 + 0x00010000)
#endif

/* LCD refresh rate */

#ifndef CONFIG_AM335X_LCD_REFRESH_FREQ
#  define CONFIG_AM335X_LCD_REFRESH_FREQ (50) /* Hz */
#endif

/* Bits per pixel / color format */

#undef AM335X_COLOR_FMT
#if defined(CONFIG_AM335X_LCD_BPP1)
#  define AM335X_BPP                    1
#  define AM335X_COLOR_FMT              FB_FMT_Y1
#elif defined(CONFIG_AM335X_LCD_BPP2)
#  define AM335X_BPP                    2
#  define AM335X_COLOR_FMT              FB_FMT_Y2
#elif defined(CONFIG_AM335X_LCD_BPP4)
#  define AM335X_BPP                    4
#  define AM335X_COLOR_FMT              FB_FMT_Y4
#elif defined(CONFIG_AM335X_LCD_BPP8)
#  define AM335X_BPP                    8
#  define AM335X_COLOR_FMT              FB_FMT_Y8
#elif defined(CONFIG_AM335X_LCD_BPP12_444)
#  define AM335X_BPP       1            12
#  define AM335X_COLOR_FMT              FB_FMT_RGB12_444
#elif defined(CONFIG_AM335X_LCD_BPP16)
#  define AM335X_BPP                    16
#  define AM335X_COLOR_FMT              FB_FMT_Y16
#elif defined(CONFIG_AM335X_LCD_BPP16_565)
#  define AM335X_BPP                    16
#  define AM335X_COLOR_FMT              FB_FMT_RGB16_565
#elif defined(CONFIG_AM335X_LCD_BPP24)
#  define AM335X_BPP                    32  /* Only 24 of 32 bits used for RGB */
#  define AM335X_COLOR_FMT              FB_FMT_RGB24
#  ifndef CONFIG_AM335X_LCD_TFTPANEL
#    error "24 BPP is only available for a TFT panel"
#  endif
#else
#  ifndef CONFIG_AM335X_LCD_TFTPANEL
#    warning "Assuming 24 BPP"
#    define AM335X_BPP                  24
#    define CONFIG_AM335X_LCD_BPP24     1
#    define AM335X_COLOR_FMT            FB_FMT_RGB24
#  else
#    warning "Assuming 16 BPP 5:6:5"
#    define AM335X_BPP                  16
#    define CONFIG_AM335X_LCD_BPP16_565 1
#    define AM335X_COLOR_FMT            FB_FMT_RGB16_565
#  endif
#endif

/* Background color */

#ifndef CONFIG_AM335X_LCD_BACKCOLOR
#  define CONFIG_AM335X_LCD_BACKCOLOR 0  /* Initial background color */
#endif

/* Horizontal video characteristics */

#ifndef CONFIG_AM335X_LCD_HWIDTH
#  define CONFIG_AM335X_LCD_HWIDTH 480 /* Width in pixels */
#endif

#ifndef CONFIG_AM335X_LCD_HPULSE
#  define CONFIG_AM335X_LCD_HPULSE 2
#endif

#ifndef CONFIG_AM335X_LCD_HFRONTPORCH
#  define CONFIG_AM335X_LCD_HFRONTPORCH 5
#endif

#ifndef CONFIG_AM335X_LCD_HBACKPORCH
#  define CONFIG_AM335X_LCD_HBACKPORCH 40
#endif

/* Vertical video characteristics */

#ifndef CONFIG_AM335X_LCD_VHEIGHT
#  define CONFIG_AM335X_LCD_VHEIGHT 272 /* Height in rows */
#endif

#ifndef CONFIG_AM335X_LCD_VPULSE
#  define CONFIG_AM335X_LCD_VPULSE 2
#endif

#ifndef CONFIG_AM335X_LCD_VFRONTPORCH
#  define CONFIG_AM335X_LCD_VFRONTPORCH 8
#endif

#ifndef CONFIG_AM335X_LCD_VBACKPORCH
#  define CONFIG_AM335X_LCD_VBACKPORCH 8
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the LPC54xx.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void am335x_lcdclear(nxgl_mxpixel_t color);

/****************************************************************************
 * Name: am335x_backlight
 *
 * Description:
 *   If CONFIG_AM335X_LCD_BACKLIGHT is defined, then the board-specific logic
 *   must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_AM335X_LCD_BACKLIGHT
void am335x_backlight(bool blon);
#endif

#endif /* __ARCH_ARM_SRC_AM335X_AM335X_LCD_H */
