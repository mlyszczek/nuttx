/**************************************************************************************
 * drivers/lcd/ft80x_base.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *  - Document No.: FT_000792, "FT800 Embedded Video Engine", Datasheet Version 1.1,
 *    Clearance No.: FTDI# 334, Future Technology Devices International Ltd.
 *  - Document No.: FT_000986, "FT801 Embedded Video Engine Datasheet", Version 1.0,
 *    Clearance No.: FTDI#376, Future Technology Devices International Ltd.
 *  - Application Note AN_240AN_240, "FT800 From the Ground Up", Version 1.1,
 *    Issue Date: 2014-06-09, Future Technology Devices International Ltd.
 *  - "FT800 Series Programmer Guide Guide", Version 2.1, Issue Date: 2016-09-19,
 *    Future Technology Devices International Ltd.
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

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ft80x.h>

#include <arch/irq.h>

#include "ft80x.h"

#ifdef CONFIG_LCD_FT80X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LCD_FT800
#  define DEVNAME "/dev/ft800"
#ifdef CONFIG_LCD_FT801
#  define DEVNAME "/dev/ft801"
#else
#  error No FT80x device configured
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Coprocessor display list commands */

static void ft80x_cmd_append(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_append_s *data);
static void ft80x_cmd_bgcolor(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_bgcolor_s *data);
static void ft80x_cmd_bitmaptransform(FAR struct ft80x_dev_s *priv,
              uint32_t dladdr, FAR struct ft80x_data_bitmaptransform_s *data);
static void ft80x_cmd_button(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_button_s *data);
static void ft80x_cmd_calibrate(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_calibrate_s *data);
static void ft80x_cmd_clock(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_clock_s *data);
static void ft80x_cmd_coldstart(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_dial(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_dial_s *data);
static void ft80x_cmd_dlstart(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_fgcolor(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_fgcolor_s *data);
static void ft80x_cmd_gauge(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_gauge_s *data);
static void ft80x_cmd_getmatrix(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_getmatrix_s *data);
static void ft80x_cmd_getprops(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_getprops_s *data);
static void ft80x_cmd_getptr(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_getptr_s *data);
static void ft80x_cmd_gradcolor(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_gradcolor_s *data);
static void ft80x_cmd_gradient(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_gradient_s *data);
static void ft80x_cmd_inflate(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_inflate_s *data);
static void ft80x_cmd_interrupt(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_interrupt_s *data);
static void ft80x_cmd_keys(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_keys_s *data);
static void ft80x_cmd_loadidentity(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_loadimage(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_loadimage_s *data);
static void ft80x_cmd_logo(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_memcpy(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_memcpy_s *data);
static void ft80x_cmd_memcrc(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_memcrc_s *data);
static void ft80x_cmd_memset(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_memset_s *data);
static void ft80x_cmd_memwrite(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_memwrite_s *data);
static void ft80x_cmd_memzero(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_memzero_s *data);
static void ft80x_cmd_number(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_number_s *data);
static void ft80x_cmd_progress(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_progress_s *data);
static void ft80x_cmd_regread(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_regread_s *data);
static void ft80x_cmd_rotate(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_rotate_s *data);
static void ft80x_cmd_scale(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_scale_s *data);
static void ft80x_cmd_screensaver(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_scrollbar(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_scrollbar_s *data);
static void ft80x_cmd_setfont(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_setfont_s *data);
static void ft80x_cmd_setmatrix(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_sketch(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_sketch_s *data);
static void ft80x_cmd_slider(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_slider_s *data);
static void ft80x_cmd_snapshot(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_snapshot_s *data);
static void ft80x_cmd_spinner(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_spinner_s *data);
static void ft80x_cmd_stop(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_swap(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_text(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_text_s *data);
static void ft80x_cmd_toggle(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_toggle_s *data);
static void ft80x_cmd_track(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_track_s *data);
static void ft80x_cmd_translate(FAR struct ft80x_dev_s *priv, uint32_t dladdr,
              FAR struct ft80x_data_translate_s *data);

static int  ft80x_displaylist(FAR struct ft80x_dev_s *priv,
             FAR struct ft80x_displaylist_s *dl);

/* Character driver methods */

static int  ft80x_open(FAR struct file *filep);
static int  ft80x_close(FAR struct file *filep);

static ssize_t ft80x_read(FAR struct file *filep, FAR char *buffer,
              size_t buflen);
static ssize_t ft80x_write(FAR struct file *filep, FAR const char *buffer,
              size_t buflen);
static int  ft80x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int  ft80x_poll(FAR struct file *filep, FAR struct pollfd *fds,
              bool setup);
#endif

/* Initialization */

static int  ft80x_initialize(FAR struct ft80x_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations ft80x_fops =
{
  ft80x_open,    /* open */
  ft80x_close,   /* close */
  ft80x_read,    /* read */
  ft80x_write,   /* write */
  NULL,          /* seek */
  ft80x_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , ft80x_poll   /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft80x_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int ft80x_displaylist(FAR struct ft80x_dev_s *priv,
                             FAR struct ft80x_displaylist_s *dl)
{
  FAR struct ft80x_dlcmd_s *cmd;
  uint32_t dladdr;
  unsigned int ncmds;
  unsigned int i;
  int ret = OK;

  DEBUGASSERT(priv != NULL && dl != NULL);

  ncmds  = dl->ncmds;
  cmd    = &dl->cmd;
  dladdr = FT80X_RAM_DL;

  /* Mark the start of the display list */

  (void)ft80x_cmd_dlstart(priv);
  dladdr += 4;

  for (i = 0; i < ncmds && ret == OK; i++)
    {
      /* Process the command.
       *
       * Commands are identified by the MS bits of the cmd.  32-bit display
       * commands have 2-8 bit MS endings (none of which take the value 0xff).
       * Multi-word graphics engine co-processor commands all begin with
       * 0xffffff--
       */

      if (cmd->cmd & 0xffffff00 != 0xffffff00)
        {
          /* Add a simple 32-bit command to the display list */

          DEBUGASSERT(cmd->len == 4);
          ret = ft80x_write_word(priv, dladdr, cmd->cmd);
        }
      else
        {
          switch (cmd->cmd)
            {
              case FT80X_CMD_APPEND:  /* Append memory to a display list */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_append_s));
                ret = ft80x_cmd_append(priv, dladdr,
                        (FAR struct ft80x_data_append_s *)&cmd->data);
                break;

              case FT80X_CMD_BGCOLOR:  /* Set the background color */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_bgcolor_s));
                ret = ft80x_cmd_bgcolor(priv, dladdr,
                        (FAR struct ft80x_data_bgcolor_s *)&cmd->data);
                break;

              case FT80X_CMD_BITMAP_TRANSFORM:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_bitmaptransform_s));
                ret = ft80x_cmd_bitmaptransform(priv, dladdr,
                        (FAR struct ft80x_data_bitmaptransform_s *)&cmd->data);
                break;

              case FT80X_CMD_BUTTON:  /* Draw a button */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_button_s));
                ret = ft80x_cmd_button(priv, dladdr,
                        (FAR struct ft80x_data_button_s *)&cmd->data);
                break;

              case FT80X_CMD_CALIBRATE:  /* Execute touchscreen calibration routine */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_calibrate_s));
                ret = ft80x_cmd_calibrate(priv, dladdr,
                        (FAR struct ft80x_data_calibrate_s *)&cmd->data);
                break;

              case FT80X_CMD_CLOCK:  /* Draw an analog clock */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_clock_s));
                ret = ft80x_cmd_clock(priv, dladdr,
                        (FAR struct ft80x_data_clock_s *)&cmd->data);
                break;

              case FT80X_CMD_COLDSTART:  /* Set co-processor engine state to default values */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_coldstart_s));
                ret = ft80x_cmd_coldstart(priv);
                break;

              case FT80X_CMD_DIAL:  /* Draw a rotary dial control */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_dial_s));
                ret = ft80x_cmd_dial(priv, dladdr,
                        (FAR struct ft80x_data_dial_s *)&cmd->data);
                break;

              case FT80X_CMD_DLSTART:  /* Start a new display list */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_dlstart_s));
                ret = ft80x_cmd_dlstart(priv);
                break;

              case FT80X_CMD_FGCOLOR:  /* Set the foreground color */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_fgcolor_s));
                ret = ft80x_cmd_fgcolor(priv, dladdr,
                        (FAR struct ft80x_data_fgcolor_s *)&cmd->data);
                break;

              case FT80X_CMD_GAUGE:  /* Draw a gauge */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_gauge_s));
                ret = ft80x_cmd_gauge(priv, dladdr,
                        (FAR struct ft80x_data_gauge_s *)&cmd->data);
                break;

              case FT80X_CMD_GETMATRIX:  /* Retrieves the current matrix coefficients */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_getmatrix_s));
                ret = ft80x_cmd_getmatrix(priv, dladdr,
                        (FAR struct ft80x_data_getmatrix_s *)&cmd->data);
                break;

              case FT80X_CMD_GETPROPS:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_getprops_s));
                ret = ft80x_cmd_getprops(priv, dladdr,
                        (FAR struct ft80x_data_getprops_s *)&cmd->data);
                break;

              case FT80X_CMD_GETPTR:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_getptr_s));
                ret = ft80x_cmd_getptr(priv, dladdr,
                        (FAR struct ft80x_data_getptr_s *)&cmd->data);
                break;

              case FT80X_CMD_GRADCOLOR:  /* Set 3D effects for BUTTON and KEYS highlight colors */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_gradcolor_s));
                ret = ft80x_cmd_gradcolor(priv, dladdr,
                        (FAR struct ft80x_data_gradcolor_s *)&cmd->data);
                break;

              case FT80X_CMD_GRADIENT:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_gradient_s));
                ret = ft80x_cmd_gradient(priv, dladdr,
                        (FAR struct ft80x_data_gradient_s *)&cmd->data);
                break;

              case FT80X_CMD_INFLATE:  /* Decompress data into memory */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_inflate_s));
                ret = ft80x_cmd_inflate(priv, dladdr,
                        (FAR struct ft80x_data_inflate_s *)&cmd->data);
                break;

              case FT80X_CMD_INTERRUPT:  /* Trigger interrupt INT_CMDFLAG */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_interrupt_s));
                ret = ft80x_cmd_interrupt(priv, dladdr,
                        (FAR struct ft80x_data_interrupt_s *)&cmd->data);
                break;

              case FT80X_CMD_KEYS:  /* Draw a row of keys */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_keys_s));
                ret = ft80x_cmd_keys(priv, dladdr,
                        (FAR struct ft80x_data_keys_s *)&cmd->data);
                break;

              case FT80X_CMD_LOADIDENTITY:  /* Set the current matrix to identity */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_loadidentity_s));
                ret = ft80x_cmd_loadidentity(priv);
                break;

              case FT80X_CMD_LOADIMAGE:  /* Load a JPEG image */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_loadimage_s));
                ret = ft80x_cmd_loadimage(priv, dladdr,
                        (FAR struct ft80x_data_loadimage_s *)&cmd->data);
                break;

              case FT80X_CMD_LOGO:  /* Play a device logo animation */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_logo_s));
                ret = ft80x_cmd_logo(priv);
                break;

              case FT80X_CMD_MEMCPY:  /* Copy a block of memory */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_memcpy_s));
                ret = ft80x_cmd_memcpy(priv, dladdr,
                        (FAR struct ft80x_data_memcpy_s *)&cmd->data);
                break;

              case FT80X_CMD_MEMCRC:  /* Compute a CRC for memory */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_memcrc_s));
                ret = ft80x_cmd_memcrc(priv, dladdr,
                        (FAR struct ft80x_data_memcrc_s *)&cmd->data);
                break;

              case FT80X_CMD_MEMSET:  /* Fill memory with a byte value */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_memset_s));
                ret = ft80x_cmd_memset(priv, dladdr,
                        (FAR struct ft80x_data_memset_s *)&cmd->data);
                break;

              case FT80X_CMD_MEMWRITE:  /* Write bytes into memory */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_memwrite_s));
                ret = ft80x_cmd_memwrite(priv, dladdr,
                        (FAR struct ft80x_data_memwrite_s *)&cmd->data);
                break;

              case FT80X_CMD_MEMZERO:  /* Write zero to a block of memory */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_memzero_s));
                ret = ft80x_cmd_memzero(priv, dladdr,
                        (FAR struct ft80x_data_memzero_s *)&cmd->data);
                break;

              case FT80X_CMD_NUMBER:  /* Draw a decimal number */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_number_s));
                ret = ft80x_cmd_number(priv, dladdr,
                        (FAR struct ft80x_data_number_s *)&cmd->data);
                break;

              case FT80X_CMD_PROGRESS:  /* Draw a progress bar */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_progress_s));
                ret = ft80x_cmd_progress(priv, dladdr,
                        (FAR struct ft80x_data_progress_s *)&cmd->data);
                break;

              case FT80X_CMD_REGREAD:  /* Read a register value */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_regread_s));
                ret = ft80x_cmd_regread(priv, dladdr,
                        (FAR struct ft80x_data_regread_s *)&cmd->data);
                break;

              case FT80X_CMD_ROTATE:  /* Apply a rotation to the current matrix */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_rotate_s));
                ret = ft80x_cmd_rotate(priv, dladdr,
                        (FAR struct ft80x_data_rotate_s *)&cmd->data);
                break;

              case FT80X_CMD_SCALE:  /* Apply a scale to the current matrix */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_scale_s));
                ret = ft80x_cmd_scale(priv, dladdr,
                        (FAR struct ft80x_data_scale_s *)&cmd->data);
                break;

              case FT80X_CMD_SCREENSAVER:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_screensaver_s));
                ret = ft80x_cmd_screensaver(priv, dladdr,
                        (FAR struct ft80x_data_screensaver_s *)&cmd->data);
                break;

              case FT80X_CMD_SCROLLBAR:  /* Draw a scroll bar */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_scrollbar_s));
                ret = ft80x_cmd_scrollbar(priv, dladdr,
                        (FAR struct ft80x_data_scrollbar_s *)&cmd->data);
                break;

              case FT80X_CMD_SETFONT:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_setfont_s));
                ret = ft80x_cmd_setfont(priv, dladdr,
                        (FAR struct ft80x_data_setfont_s *)&cmd->data);
                break;

              case FT80X_CMD_SETMATRIX:  /* Write current matrix as a bitmap transform */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_setmatrix_s));
                ret = ft80x_cmd_setmatrix(priv, dladdr,
                        (FAR struct ft80x_data_setmatrix_s *)&cmd->data);
                break;

              case FT80X_CMD_SKETCH:  /* Start a continuous sketch update */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_sketch_s));
                ret = ft80x_cmd_sketch(priv, dladdr,
                        (FAR struct ft80x_data_sketch_s *)&cmd->data);
                break;

              case FT80X_CMD_SLIDER:  /* Draw a slider */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_slider_s));
                ret = ft80x_cmd_slider(priv, dladdr,
                        (FAR struct ft80x_data_slider_s *)&cmd->data);
                break;

              case FT80X_CMD_SNAPSHOT:  /* Take a snapshot of the current screen */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_snapshot_s));
                ret = ft80x_cmd_snapshot(priv, dladdr,
                        (FAR struct ft80x_data_snapshot_s *)&cmd->data);
                break;

              case FT80X_CMD_SPINNER:  /* Start an animated spinner */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_spinner_s));
                ret = ft80x_cmd_spinner(priv, dladdr,
                        (FAR struct ft80x_data_spinner_s *)&cmd->data);
                break;

              case FT80X_CMD_STOP:  /* Stop any spinner, screensaver, or sketch */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_stop_s));
                ret = ft80x_cmd_stop(priv);
                break;

              case FT80X_CMD_SWAP:  /* Swap the current display list */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_swap_s));
                ret = ft80x_cmd_swap(priv);
                break;

              case FT80X_CMD_TEXT:  /* Draw text */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_text_s));
                ret = ft80x_cmd_text(priv, dladdr,
                        (FAR struct ft80x_data_text_s *)&cmd->data);
                break;

              case FT80X_CMD_TOGGLE:  /* Draw a toggle switch */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_toggle_s));
                ret = ft80x_cmd_toggle(priv, dladdr,
                        (FAR struct ft80x_data_toggle_s *)&cmd->data);
                break;

              case FT80X_CMD_TRACK:
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_track_s));
                ret = ft80x_cmd_track(priv, dladdr,
                        (FAR struct ft80x_data_track_s *)&cmd->data);
                break;

              case FT80X_CMD_TRANSLATE:  /* Apply a translation to the current matrix */
                DEBUGASSERT(priv->len == sizeof(struct ft80x_cmd_translate_s));
                ret = ft80x_cmd_translate(priv, dladdr,
                        (FAR struct ft80x_data_translate_s *)&cmd->data);
                break;

              default:
                lcderr("ERROR: Unrecognized command in display list: 0x8lx\n",
                       (unsigned long)cmd->cmd);
                return -EINVAL;
            }
        }

      /* Point to the next command in the display list */

      dladdr += cmd->len;
      cmd     = (FAR struct ft80x_dlcmd_s *)((uintptr_t)cmd + cmd->len);
    }

  /* Check for errors */
  if (ret >= 0)
    {
      /* Finish the display list and swap to the new one */

      ft80x_write_word(priv, dladdr, FT80X_DISPLAY());
      ret = ft80x_cmd_swap(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: ft80x_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int ft80x_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  uint8_t tmp;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  ft80xinfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count */

  priv->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&priv->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: ft80x_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int ft80x_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  ft80xinfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrement the references to the driver. */

  if (priv->crefs > 1)
    {
      priv->crefs--;
    }

  ret = OK;
  nxsem_post(&priv->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: ft80x_read
 ****************************************************************************/

static ssize_t ft80x_read(FAR struct file *filep, FAR char *buffer,
                          size_t len)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  ft80xinfo("buffer: %p len %lu\n", buffer, (unsigned long)len);

#warning Missing logic
  return len;
}

/****************************************************************************
 * Name: ft80x_write
 ****************************************************************************/

static ssize_t ft80x_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;

  ft80xinfo("buffer: %p len %lu\n", buffer, (unsigned long)len);

#warning Missing logic
  return len;
}

/****************************************************************************
 * Name: ft80x_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ****************************************************************************/

static int ft80x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct ft80x_dev_s *priv;
  FAR struct ft80x_config_s *lower;
  int ret;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv  = inode->i_private;
  lower = priv->lower;
  DEBUGASSET(lower != NULL);

  ft80xinfo("cmd: %d arg: %lu\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* FT80XIOC_DISPLYLIST:
       *   Description:  Send display list
       *   Argument:     A reference to a display list structure instance.
       *                 See struct ft80x_displaylist_s below.
       *   Returns:      Depends on the nature of the commands in the display
       *                 list (usually nothing)
       */

      case FT80XIOC_DISPLYLIST
        {
          FAR struct ft80x_displaylist_s *dl =
            (FAR struct ft80x_displaylist_s *)((uintptr_t)arg)

          if (dl != NULL)
            {
              ret = ft80x_displaylist(priv, dl);
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        lcderr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: ft80x_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int ft80x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
#warning Missing logic
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: ft80x_initialize
 *
 * Description:
 *  Initialize the FT80x
 *
 ****************************************************************************/

static int ft80x_initialize(FAR struct ft80x_dev_s *priv)
{
#if 0
  uint8_t regval8;
#endif

  /* To configure the display, load the timing control registers with values
   * for the particular display. These registers control horizontal timing:
   *
   *   - FT80X_PCLK
   *   - FT80X_PCLK_POL
   *   - FT80X_HCYCLE
   *   - FT80X_HOFFSET
   *   - FT80X_HSIZE
   *   - FT80X_HSYNC0
   *   - FT80X_HSYNC1
   *
   * These registers control vertical timing:
   *
   *   - FT80X_VCYCLE
   *   - FT80X_VOFFSET
   *   - FT80X_VSIZE
   *   - FT80X_VSYNC0
   *   - FT80X_VSYNC1
   *
   * And the FT80X_CSPREAD register changes color clock timing to reduce system
   * noise.
   *
   * GPIO bit 7 is used for the display enable pin of the LCD module. By
   * setting the direction of the GPIO bit to out direction, the display can
   * be enabled by writing value of 1 into GPIO bit 7 or the display can be
   * disabled by writing a value of 0 into GPIO bit 7. By default GPIO bit 7
   * direction is output and the value is 0.
   */

  /* Initialization Sequence from Power Down using PD_N pin:
   *
   * 1. Drive the PD_N pin high
   * 2. Wait for at least 20ms
   * 3. Execute "Initialization Sequence during the Boot up" from steps 1 to 9
   *
   * Initialization Sequence from Sleep Mode:
   *
   * 1. Send Host command "ACTIVE" to enable clock to FT800
   * 2. Wait for at least 20ms
   * 3. Execute "Initialization Sequence during Boot Up" from steps 5 to 8
   *
   * Initialization sequence from standby mode:
   *
   * Execute all the steps mentioned in "Initialization Sequence from Sleep
   * Mode" except waiting for at least 20ms in step 2.
   */

   DEBUGASSERT(priv->lower != NULL && priv->lower->pwrdown != NULL);
   priv->lower->pwrdown(priv->lower, false);
   up_mdelay(20);

  /* Initialization Sequence during the boot up:
   *
   * 1. Use MCU SPI clock not more than 11MHz
   * 2. Send Host command CLKEXT to FT800
   * 3. Send Host command ACTIVE to enable clock to FT800.
   * 4. Configure video timing registers, except FT80X_PCLK
   * 5. Write first display list
   * 6. Write FT80X_DLSWAP, FT800 swaps display list immediately
   * 7. Enable back light control for display
   * 8. Write FT80X_PCLK, video output begins with the first display list
   * 9. Use MCU SPI clock not more than 30MHz
   */

  /* 1. Select the initial SPI frequency */

  DEBUGASSERT(priv->lower->init_frequency <= 11000000);
  priv->frequency = priv->lower->init_frequency;

  /* 2. Send Host command CLKEXT to FT800
   * 3. Send Host command ACTIVE to enable clock to FT800.
   */

  ft80x_host_command(priv, FT80X_CMD_CLKEXT);
  ft80x_host_command(priv, FT80X_CMD_ACTIVE);

  /* Verify the chip ID */
#warning Missing logic

  /* 4. Configure video timing registers, except FT80X_PCLK
   *
   * Once the FT800 is awake and the internal clock set and Device ID
   * checked, the next task is to configure the LCD display parameters for
   * the chosen display with the values determined in Section 2.3.3 above.
   *
   * a. Set FT80X_PCLK to zero - This disables the pixel clock output while
   *    the LCD and other system parameters are configured
   * b. Set the following registers with values for the chosen display.
   *    Typical WQVGA and QVGA values are shown:
   *
   *    Register        Description                      WQVGA    QVGA 320 x 240
   *                                                     480x272  320x240
   *    FT80X_PCLK_POL  Pixel Clock Polarity             1        0
   *    FT80X_HSIZE     Image width in pixels            480      320
   *    FT80X_HCYCLE    Total number of clocks per line  548      408
   *    FT80X_HOFFSET   Horizontal image start           43       70
   *                    (pixels from left)
   *    FT80X_HSYNC0    Start of HSYNC pulse             0        0
   *                    (falling edge)
   *    FT80X_HSYNC1    End of HSYNC pulse               41       10
   *                    (rising edge)
   *    FT80X_VSIZE     Image height in pixels           272      240
   *    FT80X_VCYCLE    Total number of lines per screen 292      263
   *    FT80X_VOFFSET   Vertical image start             12       13
   *                    (lines from top)
   *    FT80X_VSYNC0    Start of VSYNC pulse             0        0
   *                    (falling edge)
   *    FT80X_VSYNC1    End of VSYNC pulse               10       2
   *                    (rising edge)
   *
   * c. Enable or disable FT80X_CSPREAD with a value of 01h or 00h,
   *    respectively.  Enabling FT80X_CSPREAD will offset the R, G and B
   *    output bits so all they do not all change at the same time.
   */

  ft80x_write_byte(priv, FT80X_PCLK, 0);

#if defined(CONFIG_LCD_FT80X_WQVGA)
  ft80x_write_hword(priv, FT80X_HCYCLE, 548);
  ft80x_write_hword(priv, FT80X_HOFFSET, 43);
  ft80x_write_hword(priv, FT80X_HSYNC0, 0);
  ft80x_write_hword(priv, FT80X_HSYNC1, 41);
  ft80x_write_hword(priv, FT80X_VCYCLE, 292);
  ft80x_write_hword(priv, FT80X_VOFFSET, 12);
  ft80x_write_hword(priv, FT80X_VSYNC0, 0);
  ft80x_write_hword(priv, FT80X_VSYNC1, 10);
  ft80x_write_byte(priv, FT80X_SWIZZLE, 0);
  ft80x_write_byte(priv, FT80X_PCLK_POL, 1);
  ft80x_write_byte(priv, FT80X_CSPREAD, 1);
  ft80x_write_hword(priv, FT80X_HSIZE, 480);
  ft80x_write_hword(priv, FT80X_VSIZE, 272);

#elif defined(CONFIG_LCD_FT80X_QVGA)
  ft80x_write_hword(priv, FT80X_HCYCLE, 408);
  ft80x_write_hword(priv, FT80X_HOFFSET, 70);
  ft80x_write_hword(priv, FT80X_HSYNC0, 0);
  ft80x_write_hword(priv, FT80X_HSYNC1, 10);
  ft80x_write_hword(priv, FT80X_VCYCLE, 263);
  ft80x_write_hword(priv, FT80X_VOFFSET, 13);
  ft80x_write_hword(priv, FT80X_VSYNC0, 0);
  ft80x_write_hword(priv, FT80X_VSYNC1, 2);
  ft80x_write_byte(priv, FT80X_SWIZZLE, 0);  /* REVISIT */
  ft80x_write_byte(priv, FT80X_PCLK_POL, 0);
  ft80x_write_byte(priv, FT80X_CSPREAD, 1);
  ft80x_write_hword(priv, FT80X_HSIZE, 320);
  ft80x_write_hword(priv, FT80X_VSIZE, 240);

#else
#  error Unknown display size
#endif

  /* 5. Write first display list */

  ft80x_write_word(priv, FT80X_RAM_DL + 0, FT80X_CLEAR_COLOR_RGB(0,0,0));
  ft80x_write_word(priv, FT80X_RAM_DL + 4, FT80X_CLEAR(1,1,1));
  ft80x_write_word(priv, FT80X_RAM_DL + 8, FT80X_DISPLAY());

  /* 6. Write FT80X_DLSWAP, FT800 swaps display list immediately */

  ft80x_write_byte(priv, FT80X_DLSWAP, DLSWAP_FRAME);

  /* GPIO bit 7 is used for the display enable pin of the LCD module. By
   * setting the direction of the GPIO bit to out direction, the display can
   * be enabled by writing value of 1 into GPIO bit 7 or the display can be
   * disabled by writing a value of 0 into GPIO bit 7. By default GPIO bit 7
   * direction is output and the value is 0.
   */

  regval8  = ft80x_read_byte(priv, FT80X_GPIO_DIR);
  regval8 |= (1 << 7);
  ft80x_write_byte(priv, FT80X_GPIO_DIR, regval8);

  regval8  = ft80x_read_byte(priv, FT80X_GPIO);
  regval8 |= (1 << 7);
  ft80x_write_byte(priv, FT80X_GPIO, regval8);

  /* 7. Enable back light control for display */
#warning Missing logic

  /* 8. Write FT80X_PCLK, video output begins with the first display list */

  ft80x_write_byte(priv, FT80X_PCLK, 5);

  /* 9. Use MCU SPI clock not more than 30MHz */

  DEBUGASSERT(priv->lower->op_frequency <= 30000000);
  priv->frequency = priv->lower->op_frequency;

  return OK;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

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
{
  FAR struct ft80x_dev_s *priv;

#if defined(CONFIG_LCD_FT80X_SPI)
  DEBUGASSERT(spi != NULL && lower != NULL);
#elif defined(CONFIG_LCD_FT80X_I2C)
  DEBUGASSERT(i2c != NULL && lower != NULL);
#endif

  /* Allocate the driver state structure */

  priv = (FAR struct ft80x_dev_s *)kmm_zalloc(sizeof(struct ft80x_dev_s));
  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate state structure\n");
      return -ENOMEM;
    }

  /* Save the lower level interface and configuration information */

  priv->lower = lower;

#ifdef CONFIG_LCD_FT80X_SPI
  /* Remember the SPI configuration */

  priv->spi = spi;
#else
  /* Remember the I2C configuration */

  priv->i2c = i2c;
#endif

  /* Initialize the mutual exclusion semaphore */

  sem_init(&priv->exclsem, 0, 1);

  /* Initialize the FT80x */

  ret = ft80x_initialize(priv);
  if (ret < 0)
    {
      goto errout_with_sem;
    }

  /* Register the FT80x character driver */

  ret = register_driver(DEVNAME, &ft80x_fops, 0666, priv);
  if (ret < 0)
    {
      goto errout_with_sem;
    }

  return OK;

errout_with_sem:
  sem_destory(&priv->exclsem);
  return ret;
}

#endif /* CONFIG_LCD_FT80X */
