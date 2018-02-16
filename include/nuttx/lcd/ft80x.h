/****************************************************************************
 * include/nuttx/lcd/ft80x.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *  - Document No.: FT_000792, "FT800 Embedded Video Engine", Datasheet
 *    Version 1.1, Clearance No.: FTDI# 334, Future Technology Devices
 *    International Ltd.
 *  - Document No.: FT_000986, "FT801 Embedded Video Engine Datasheet",
 *    Version 1.0, Clearance No.: FTDI#376, Future Technology Devices
 *    International Ltd.
 *  - Application Note AN_240AN_240, "FT800 From the Ground Up", Version
 *    1.1, Issue Date: 2014-06-09, Future Technology Devices International
 *    Ltd.
 *  - Some definitions derive from FTDI sample code.
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

#ifndef __INCLUDE_NUTTX_LCD_FT80X_H
#define __INCLUDE_NUTTX_LCD_FT80X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/lcd_ioctl.h>

#ifdef CONFIG_LCD_FT80X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FT80x IOCTL commands:
 *
 * FT80XIOC_DISPLYLIST:
 *   Description:  Send display list
 *   Argument:     A reference to a display list structure instance.  See
 *                 struct ft80x_displaylist_s below.
 *   Returns:      Depends on the nature of the commands in the display list
 *                 (usually nothing)
 */

#define FT80XIOC_DISPLYLIST        _LCDIOC(FT80X_NIOCTL_BASE)

/* Host commands */

#define FT80X_CMD_ACTIVE           0x00        /* Switch from Standby/Sleep modes to active mode */
#define FT80X_CMD_STANDBY          0x41        /* Put FT80x core to standby mode */
#define FT80X_CMD_SLEEP            0x42        /* Put FT80x core to sleep mode */
#define FT80X_CMD_PWRDOWN          0x50        /* Switch off 1.2V internal regulator */
#define FT80X_CMD_CLKEXT           0x44        /* Enable PLL input from oscillator or external clock */
#define FT80X_CMD_CLK48M           0x62        /* Switch PLL output clock to 48MHz (default). */
#define FT80X_CMD_CLK36M           0x61        /* Switch PLL output clock to 36MHz */
#define FT80X_CMD_CORERST          0x68        /* Send reset pulse to FT800 core */

/* FT80x Coprocessor commands */

#define FT80X_CMD_APPEND           0xffffff1e  /* Append memory to a display list */
#define FT80X_CMD_BGCOLOR          0xffffff09  /* Set the background color */
#define FT80X_CMD_BITMAP_TRANSFORM 0xffffff21
#define FT80X_CMD_BUTTON           0xffffff0d  /* Draw a button */
#define FT80X_CMD_CALIBRATE        0xffffff15  /* Execute touchscreen calibration routine */
#define FT80X_CMD_CLOCK            0xffffff14  /* Draw an analog clock */
#define FT80X_CMD_COLDSTART        0xffffff32  /* Set co-processor engine state to default values */
#define FT80X_CMD_CRC              0xffffff03  /* ? */
#define FT80X_CMD_DIAL             0xffffff2d  /* Draw a rotary dial control */
#define FT80X_CMD_DLSTART          0xffffff00  /* Start a new display list */
#define FT80X_CMD_EXECUTE          0xffffff07  /* ? */
#define FT80X_CMD_FGCOLOR          0xffffff0a  /* Set the foreground color */
#define FT80X_CMD_GAUGE            0xffffff13  /* Draw a gauge */
#define FT80X_CMD_GETMATRIX        0xffffff33  /* Retrieves the current matrix coefficients */
#define FT80X_CMD_GETPOINT         0xffffff08  /* ? */
#define FT80X_CMD_GETPROPS         0xffffff25
#define FT80X_CMD_GETPTR           0xffffff23
#define FT80X_CMD_GRADCOLOR        0xffffff34  /* Set 3D effects for BUTTON and KEYS highlight colors */
#define FT80X_CMD_GRADIENT         0xffffff0b  /* Draw a smooth color gradient */
#define FT80X_CMD_HAMMERAUX        0xffffff04  /* ? */
#define FT80X_CMD_IDCT             0xffffff06  /* ? */
#define FT80X_CMD_INFLATE          0xffffff22  /* Decompress data into memory */
#define FT80X_CMD_INTERRUPT        0xffffff02  /* Trigger interrupt INT_CMDFLAG */
#define FT80X_CMD_KEYS             0xffffff0e  /* Draw a row of keys */
#define FT80X_CMD_LOADIDENTITY     0xffffff26  /* Set the current matrix to identity */
#define FT80X_CMD_LOADIMAGE        0xffffff24  /* Load a JPEG image */
#define FT80X_CMD_LOGO             0xffffff31  /* Play a device logo animation */
#define FT80X_CMD_MARCH            0xffffff05  /* ? */
#define FT80X_CMD_MEMCPY           0xffffff1d  /* Copy a block of memory */
#define FT80X_CMD_MEMCRC           0xffffff18  /* Compute a CRC for memory */
#define FT80X_CMD_MEMSET           0xffffff1b  /* Fill memory with a byte value */
#define FT80X_CMD_MEMWRITE         0xffffff1a  /* Write bytes into memory */
#define FT80X_CMD_MEMZERO          0xffffff1c  /* Write zero to a block of memory */
#define FT80X_CMD_NUMBER           0xffffff2e  /* Draw a decimal number */
#define FT80X_CMD_PROGRESS         0xffffff0f  /* Draw a progress bar */
#define FT80X_CMD_REGREAD          0xffffff19  /* Read a register value */
#define FT80X_CMD_ROTATE           0xffffff29  /* Apply a rotation to the current matrix */
#define FT80X_CMD_SCALE            0xffffff28  /* Apply a scale to the current matrix */
#define FT80X_CMD_SCREENSAVER      0xffffff2f
#define FT80X_CMD_SCROLLBAR        0xffffff11  /* Draw a scroll bar */
#define FT80X_CMD_SETFONT          0xffffff2b
#define FT80X_CMD_SETMATRIX        0xffffff2a  /* Write current matrix as a bitmap transform */
#define FT80X_CMD_SKETCH           0xffffff30  /* Start a continuous sketch update */
#define FT80X_CMD_SLIDER           0xffffff10  /* Draw a slider */
#define FT80X_CMD_SNAPSHOT         0xffffff1f  /* Take a snapshot of the current screen */
#define FT80X_CMD_SPINNER          0xffffff16  /* Start an animated spinner */
#define FT80X_CMD_STOP             0xffffff17  /* Stop any spinner, screensave, or sketch */
#define FT80X_CMD_SWAP             0xffffff01  /* Swap the current display list */
#define FT80X_CMD_TEXT             0xffffff0c  /* Draw text */
#define FT80X_CMD_TOGGLE           0xffffff12  /* Draw a toggle switch */
#define FT80X_CMD_TOUCH_TRANSFORM  0xffffff20  /* ? */
#define FT80X_CMD_TRACK            0xffffff2c
#define FT80X_CMD_TRANSLATE        0xffffff27  /* Apply a translation to the current matrix */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Pins relevant to software control.  The FT80X is a 48-pin part.  Most of
 * the pins are associated with the TFT panel and other board-related
 * support.  A few a relevant to software control of the part.  Those are
 * listed here:
 *
 * FT80X PIN  DIR DESCRIPTION
 *  3          I  SPI: SCLK, I2C: SCL
 *  4          I  SPI: MISO, I2C: SDA
 *  5         I/O SPI: MOSI
 *  6          I  SPI: nCS
 *  11        OD  nINT Host interrupt
 *  12         *  nPD  Power down input
 *
 * SCL/SDA, SCLK/MISO/MOSI/nCS are handled by generic I2C or SPI logic. nInt
 * and nPD are directly managed by this interface.
 */

/* A reference to a structure of this type must be passed to the FT80X
 * driver.  This structure provides information about the configuration
 * of the FT80X and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.  The
 * memory may be read-only.
 */

struct ft80x_config_s
{
  /* Device characterization */

  uint32_t int_frequency;  /* I2C/SPI initialization frequency */
  uint32_t op_frequency;   /* I2C/SPI operational frequency */
#ifdef CONFIG_LCD_FT80X_I2C
  uint8_t address;         /* 7-bit I2C address */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the FT80X driver from differences in GPIO
   * interrupt handling by varying boards and MCUs. Interrupts should be
   * configured on the falling edge of nINT.
   *
   *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
   *   enable  - Enable or disable the GPIO interrupt
   *   clear   - Acknowledge/clear any pending GPIO interrupt
   *   pwrdown - Power down the FT80X
   */

  int  (*attach)(FAR const struct ft80x_config_s *state, xcpt_t isr,
                 FAR void *arg);
  void (*enable)(FAR const struct ft80x_config_s *state, bool enable);
  void (*clear)(FAR const struct ft80x_config_s *state);
  bool (*pwrdown)(FAR const struct ft80x_config_s *state, bool pwrdown);
};

/* This structure describes one generic display list command */

struct ft80x_dlcmd_s
{
  uint32_t len;     /* Size of the command structure instance */
  uint32_t cmd;     /* Display list command.  See FT80X_CMD_* definitions */
  uint8_t  data[1]; /* Data associated with the command */
};

/* Specific display list command structures */
/* FT80X_CMD_APPEND:  Append memory to a display list */

struct ft80x_data_append_s
{
  uint32_t ptr,
  uint32_t num;
};

struct ft80x_cmd_append_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_append_s) */
  uint32_t cmd;     /* FT80X_CMD_APPEND */
  struct ft80x_data_append_s data;
};

/* FT80X_CMD_BGCOLOR:  Set the background color */

struct ft80x_data_bgcolor_s
{
  uint32_t c;
};

struct ft80x_cmd_bgcolor_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_bgcolor_s) */
  uint32_t cmd;     /* FT80X_CMD_BGCOLOR */
  struct ft80x_data_bgcolor_s data;
};

struct ft80x_data_bitmaptransfer_s
{
  int32_t x0;
  int32_t y0;
  int32_t x1;
  int32_t y1;
  int32_t x2;
  int32_t y2;
  int32_t tx0;
  int32_t ty0;
  int32_t tx1;
  int32_t ty1;
  int32_t tx2;
  int32_t ty2
  uint16_t result;
};

struct ft80x_cmd_bitmaptransfer_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_bitmaptransfer_s) */
  uint32_t cmd;     /* FT80X_CMD_BITMAP_TRANSFORM */
  struct ft80x_data_bitmaptransfer_s data;
};

/* FT80X_CMD_BUTTON:  Draw a button */

struct ft80x_data_button_s
{
   int16_t x;
   int16_t y;
   int16_t w;
   int16_t h;
   int16_t font,
   uint16_t options;
   FAR const char *s;
};

struct ft80x_cmd_button_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_button_s) */
  uint32_t cmd;     /* FT80X_CMD_BUTTON */
  struct ft80x_data_button_s data;
};

/* FT80X_CMD_CALIBRATE:  Execute touchscreen calibration routine */

struct ft80x_data_calibrate_s
{
  uint32_t result;
};

struct ft80x_cmd_calibrate_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_calibrate_s) */
  uint32_t cmd;     /* FT80X_CMD_CALIBRATE */
  struct ft80x_data_calibrate_s data;
};

/* FT80X_CMD_CLOCK:  Draw an analog clock */

struct ft80x_data_clock_s
{
  int16_t x;
  int16_t y;
  int16_t r;
  uint16_t options;
  uint16_t h;
  uint16_t m;
  uint16_t s;
  uint16_t ms;
};

struct ft80x_cmd_clock_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_clock_s) */
  uint32_t cmd;     /* FT80X_CMD_CLOCK */
  struct ft80x_data_clock_s data;
};

/* FT80X_CMD_COLDSTART:  Set co-processor engine state to default values */

struct ft80x_cmd_coldstart_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_clock_s) */
  uint32_t cmd;     /* FT80X_CMD_COLDSTART */
};

/* FT80X_CMD_DIAL:  Draw a rotary dial control */

struct ft80x_data_dial_s
{
  int16_t x;
  int16_t y;
  int16_t r;
  uint16_t options;
  uint16_t val;
};

struct ft80x_cmd_dial_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_dial_s) */
  uint32_t cmd;     /* FT80X_CMD_DIAL */
  struct ft80x_data_dial_s data;
};

/* FT80X_CMD_DLSTART: Start a new display list */

struct ft80x_cmd_dlstart_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_dlstart_s) */
  uint32_t cmd;     /* FT80X_CMD_DLSTART */
};

/* FT80X_CMD_FGCOLOR:  Set the foreground color */

struct ft80x_data_fgcolor_s
{
  uint32_t c;
};

struct ft80x_cmd_fgcolor_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_fgcolor_s) */
  uint32_t cmd;     /* FT80X_CMD_FGCOLOR */
  struct ft80x_data_fgcolor_s data;
};

/* FT80X_CMD_GAUGE: Draw a gauge */

struct ft80x_data_gauge_s
{
  int16_t x;
  int16_t y;
  int16_t r;
  uint16_t options;
  uint16_t major;
  uint16_t minor;
  uint16_t val;
  uint16_t range;
};

struct ft80x_cmd_gauge_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_gauge_s) */
  uint32_t cmd;     /* FT80X_CMD_GAUGE */
  struct ft80x_data_gauge_s data;
};

/* FT80X_CMD_GETMATRIX: Retrieves the current matrix coefficients */

struct ft80x_data_getmatrix_s
{
 int32_t a;
 int32_t b;
 int32_t c;
 int32_t d;
 int32_t e;
 int32_t f;
};

struct ft80x_cmd_getmatrix_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_getmatrix_s) */
  uint32_t cmd;     /* FT80X_CMD_GETMATRIX */
  struct ft80x_data_getmatrix_s data;
};

/* FT80X_CMD_GETPROPS */

struct ft80x_data_getprops_s
{
  uint32_t ptr;
  uint32_t w;
  uint32_t h;
};

struct ft80x_cmd_getprops_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_getprops_s) */
  uint32_t cmd;     /* FT80X_CMD_GETPROPS */
  struct ft80x_data_getprops_s data;
};

/* FT80X_CMD_GETPTR */

struct ft80x_data_getptr_s
{
  uint32_t result;
};

struct ft80x_cmd_getptr_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_getptr_s) */
  uint32_t cmd;     /* FT80X_CMD_GETPTR */
  struct ft80x_data_getptr_s data;
};

/* FT80X_CMD_GRADCOLOR: Set 3D effects for BUTTON and KEYS highlight colors */

struct ft80x_data_gradcolor_s
{
  uint32_t c;
};

struct ft80x_cmd_gradcolor_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_gradcolor_s) */
  uint32_t cmd;     /* FT80X_CMD_GRADCOLOR */
  struct ft80x_data_gradcolor_s data;
};

/* FT80X_CMD_GRADIENT:  Draw a smooth color gradient */

struct ft80x_data_gradient_s
{
  int16_t x0;
  int16_t y0;
  uint32_t rgb0;
  int16_t x1;
  int16_t y1;
  uint32_t rgb1;
};

struct ft80x_cmd_gradient_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_gradient_s) */
  uint32_t cmd;     /* FT80X_CMD_GRADIENT */
  struct ft80x_data_gradient_s data;
};

/* FT80X_CMD_INFLATE:  Decompress data into memory */

struct ft80x_data_inflate_s
{
  uint32_t ptr;
  FAR uint8_t *compressed;
};

struct ft80x_cmd_inflate_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_inflate_s) */
  uint32_t cmd;     /* FT80X_CMD_INFLATE */
  struct ft80x_data_inflate_s data;
};

/* FT80X_CMD_INTERRUPT:  Trigger interrupt INT_CMDFLAG */

struct ft80x_data_interrupt_s
{
  uint32_t ms;
};

struct ft80x_cmd_interrupt_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_interrupt_s) */
  uint32_t cmd;     /* FT80X_CMD_INTERRUPT */
  struct ft80x_data_interrupt_s data;
};

/* FT80X_CMD_KEYS:  Draw a row of keys */

struct ft80x_data_keys_s
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  int16_t font;
  uint16_t options;
  FAR const char *s;
};

struct ft80x_cmd_keys_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_keys_s) */
  uint32_t cmd;     /* FT80X_CMD_KEYS */
  struct ft80x_data_keys_s data;
};

/* FT80X_CMD_LOADIDENTITY:  Set the current matrix to identity */

struct ft80x_cmd_loadidentity_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_loadidentity_s) */
  uint32_t cmd;     /* FT80X_CMD_LOADIDENTITY */
};

/* FT80X_CMD_LOADIMAGE:  Load a JPEG image */

struct ft80x_data_loadimage_s
{
  uint32_t ptr;
  uint32_t options;
};

struct ft80x_cmd_loadimage_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_loadimage_s) */
  uint32_t cmd;     /* FT80X_CMD_LOADIMAGE */
  struct ft80x_data_loadimage_s data;
};

/* FT80X_CMD_LOGO:  Play a device logo animation */

struct ft80x_cmd_logo_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_logo_s) */
  uint32_t cmd;     /* FT80X_CMD_LOGO */
};

/* FT80X_CMD_MEMCPY:  Copy a block of memory */

struct ft80x_data_memcpy_s
{
   uint32_t dest;
   uint32_t src;
   uint32_t num;
};

struct ft80x_cmd_memcpy_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_memcpy_s) */
  uint32_t cmd;     /* FT80X_CMD_MEMCPY */
  struct ft80x_data_memcpy_s data;
};

/* FT80X_CMD_MEMCRC:  Compute a CRC for memory */

struct ft80x_data_memcrc_s
{
  uint32_t ptr;
  uint32_t num;
  uint32_t result;
};

struct ft80x_cmd_memcrc_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_memcrc_s) */
  uint32_t cmd;     /* FT80X_CMD_MEMCRC */
  struct ft80x_data_memcrc_s data;
};

/* FT80X_CMD_MEMSET:  Fill memory with a byte value */

struct ft80x_data_memset_s
{
  uint32_t ptr;
  uint32_t value;
  uint32_t num;
};

struct ft80x_cmd_memset_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_memset_s) */
  uint32_t cmd;     /* FT80X_CMD_MEMSET */
  struct ft80x_data_memset_s data;
};

/* FT80X_CMD_MEMWRITE:  Write bytes into memory */

struct ft80x_data_memwrite_s
{
  uint32_t ptr;
  uint32_t num;
  FAR uint8_t *src;
};

struct ft80x_cmd_memwrite_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_memwrite_s) */
  uint32_t cmd;     /* FT80X_CMD_MEMWRITE */
  struct ft80x_data_memwrite_s data;
};

/* FT80X_CMD_MEMZERO:  Write zero to a block of memory */

struct ft80x_data_memzero_s
{
  uint32_t ptr;
  uint32_t num;
};

struct ft80x_cmd_memzero_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_memzero_s) */
  uint32_t cmd;     /* FT80X_CMD_MEMZERO */
  struct ft80x_data_memzero_s data;
};

/* FT80X_CMD_NUMBER:  Draw a decimal number */

struct ft80x_data_number_s
{
  int16_t x;
  int16_t y;
  int16_t font;
  uint16_t options;
  int32_t n;
};

struct ft80x_cmd_number_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_number_s) */
  uint32_t cmd;     /* FT80X_CMD_NUMBER */
  struct ft80x_data_number_s data;
};

/* FT80X_CMD_PROGRESS:  Draw a progress bar */

struct ft80x_data_progress_s
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  uint16_t options;
  uint16_t val;
  uint16_t range;
};

struct ft80x_cmd_progress_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_progress_s) */
  uint32_t cmd;     /* FT80X_CMD_PROGRESS */
  struct ft80x_data_progress_s data;
};

/* FT80X_CMD_REGREAD:  Read a register value */

struct ft80x_data_regread_s
{
  uint32_t ptr;
  uint32_t result;
};

struct ft80x_cmd_regread_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_regread_s) */
  uint32_t cmd;     /* FT80X_CMD_REGREAD */
  struct ft80x_data_regread_s data;
};

/* FT80X_CMD_ROTATE:  Apply a rotation to the current matrix */

struct ft80x_data_rotate_s
{
  int32_t a;
};

struct ft80x_cmd_rotate_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_rotate_s) */
  uint32_t cmd;     /* FT80X_CMD_ROTATE */
  struct ft80x_data_rotate_s data;
};

/* FT80X_CMD_SCALE:  Apply a scale to the current matrix */

struct ft80x_data_scale_s
{
  int32_t sx;
  int32_t sy;
};

struct ft80x_cmd_scale_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_scale_s) */
  uint32_t cmd;     /* FT80X_CMD_SCALE */
  struct ft80x_data_scale_s data;
};

/* FT80X_CMD_SCREENSAVER */

struct ft80x_cmd_screensaver_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_screensaver_s) */
  uint32_t cmd;     /* FT80X_CMD_SCREENSAVER */
};

/* FT80X_CMD_SCROLLBAR:  Draw a scroll bar */

struct ft80x_data_scrollbar_s
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  uint16_t options;
  uint16_t val;
  uint16_t size;
  uint16_t range;
};

struct ft80x_cmd_scrollbar_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_scrollbar_s) */
  uint32_t cmd;     /* FT80X_CMD_SCROLLBAR */
  struct ft80x_data_scrollbar_s data;
};

/* FT80X_CMD_SETFONT */

struct ft80x_data_setfont_s
{
  uint32_t font;
  uint32_t ptr;
};

struct ft80x_cmd_setfont_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_setfont_s) */
  uint32_t cmd;     /* FT80X_CMD_SETFONT */
  struct ft80x_data_setfont_s data;
};

/* FT80X_CMD_SETMATRIX: Write current matrix as a bitmap transform */

struct ft80x_cmd_setmatrix_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_setmatrix_s) */
  uint32_t cmd;     /* FT80X_CMD_SETMATRIX */
};

/* FT80X_CMD_SKETCH:  Start a continuous sketch update */

struct ft80x_data_sketch_s
{
  int16_t x;
  int16_t y;
  uint16_t w;
  uint16_t h;
  uint32_t ptr;
  uint16_t format;
};

struct ft80x_cmd_sketch_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_sketch_s) */
  uint32_t cmd;     /* FT80X_CMD_SKETCH */
  struct ft80x_data_sketch_s data;
};

/* FT80X_CMD_SLIDER:  Draw a slider */

struct ft80x_data_slider_s
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  uint16_t options;
  uint16_t val;
  uint16_t range;
};

struct ft80x_cmd_slider_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_slider_s) */
  uint32_t cmd;     /* FT80X_CMD_SLIDER */
  struct ft80x_data_slider_s data;
};

/* FT80X_CMD_SNAPSHOT:  Take a snapshot of the current screen */

struct ft80x_data_snapshot_s
{
  uint32_t ptr;
};

struct ft80x_cmd_snapshot_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_snapshot_s) */
  uint32_t cmd;     /* FT80X_CMD_SNAPSHOT */
  struct ft80x_data_snapshot_s data;
};

/* FT80X_CMD_SPINNER: Start an animated spinner */

struct ft80x_data_spinner_s
{
  int16_t x;
  int16_t y;
  uint16_t style;
  uint16_t scale;
};

struct ft80x_cmd_spinner_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_spinner_s) */
  uint32_t cmd;     /* FT80X_CMD_SPINNER */
  struct ft80x_data_spinner_s data;
};

/* FT80X_CMD_STOP:  Stop any spinner, screensave, or sketch */

struct ft80x_cmd_stop_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_stop_s) */
  uint32_t cmd;     /* FT80X_CMD_STOP */
};

/* FT80X_CMD_SWAP: Swap the current display list */

struct ft80x_cmd_swap_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_swap_s) */
  uint32_t cmd;     /* FT80X_CMD_SWAP */
};

/* FT80X_CMD_TEXT: Draw text */

struct ft80x_data_text_s
{
  int16_t x;
  int16_t y;
  int16_t font;
  uint16_t options;
  FAR const char *s;
};

struct ft80x_cmd_text_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_text_s) */
  uint32_t cmd;     /* FT80X_CMD_TEXT */
  struct ft80x_data_text_s data;
};

/* FT80X_CMD_TOGGLE:  Draw a toggle switch */

struct ft80x_data_toggle_s
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t font;
  uint16_t options;
  uint16_t state
  FAR const char *s;
};

struct ft80x_cmd_toggle_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_toggle_s) */
  uint32_t cmd;     /* FT80X_CMD_TOGGLE */
  struct ft80x_data_toggle_s data;
};

/* FT80X_CMD_TRACK */

struct ft80x_data_track_s
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  int16_t tag;
};

struct ft80x_cmd_track_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_track_s) */
  uint32_t cmd;     /* FT80X_CMD_TRACK */
  struct ft80x_data_track_s data;
};

/* FT80X_CMD_TRANSLATE:  Apply a translation to the current matrix */

struct ft80x_data_translate_s
{
  int32_t tx;
  int32_t ty;
};

struct ft80x_cmd_translate_s
{
  uint32_t len;     /* sizeof(struct ft80x_cmd_translate_s) */
  uint32_t cmd;     /* FT80X_CMD_TRANSLATE */
  struct ft80x_data_translate_s data;
};

/* This structure then defines a list of display commands */

struct ft80x_displaylist_s
{
  unsigned int ncmds;       /* Number of commands in the display list */
  struct ft80x_dlcmd_s cmd; /* First command in the display list */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ft80x_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
 *   will register the driver as /dev/ft80x.
 *
 * Input Parameters:
 *   spi     - An SPI driver instance
 *   i2c     - An I2C master driver instance
 *   lower   - Persistent board configuration data / lower half interface
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_LCD_FT80X_SPI)
int ft80x_register(FAR struct spi_dev_s *spi,
                   FAR const struct ft80x_config_s *lower);
#elif defined(CONFIG_LCD_FT80X_I2C)
int ft80x_register(FAR struct i2c_master_s *i2c,
                   FAR const struct ft80x_config_s *lower);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_FT80X */
#endif /* __INCLUDE_NUTTX_LCD_FT80X_H */
