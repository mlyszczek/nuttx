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

/* Coprocessor commands */

static void ft80x_cmd_append(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_append_s *data);
static void ft80x_cmd_bgcolor(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_bgcolor_s *data);
static void ft80x_cmd_bitmaptransform(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_bitmaptransform_s *data);
static void ft80x_cmd_button(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_button_s *data);
static void ft80x_cmd_calibrate(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_calibrate_s *data);
static void ft80x_cmd_clock(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_clock_s *data);
static void ft80x_cmd_coldstart(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_dial(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_dial_s *data);
static void ft80x_cmd_dlstart(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_fgcolor(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_fgcolor_s *data);
static void ft80x_cmd_gauge(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_gauge_s *data);
static void ft80x_cmd_getmatrix(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_getmatrix_s *data);
static void ft80x_cmd_getprops(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_getprops_s *data);
static void ft80x_cmd_getptr(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_getptr_s *data);
static void ft80x_cmd_gradcolor(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_gradcolor_s *data);
static void ft80x_cmd_gradient(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_gradient_s *data);
static void ft80x_cmd_inflate(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_inflate_s *data);
static void ft80x_cmd_interrupt(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_interrupt_s *data);
static void ft80x_cmd_keys(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_keys_s *data);
static void ft80x_cmd_loadidentity(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_loadimage(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_loadimage_s *data);
static void ft80x_cmd_logo(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_memcpy(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_memcpy_s *data);
static void ft80x_cmd_memcrc(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_memcrc_s *data);
static void ft80x_cmd_memset(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_memset_s *data);
static void ft80x_cmd_memwrite(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_memwrite_s *data);
static void ft80x_cmd_memzero(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_memzero_s *data);
static void ft80x_cmd_number(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_number_s *data);
static void ft80x_cmd_progress(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_progress_s *data);
static void ft80x_cmd_regread(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_regread_s *data);
static void ft80x_cmd_rotate(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_rotate_s *data);
static void ft80x_cmd_scale(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_scale_s *data);
static void ft80x_cmd_screensaver(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_scrollbar(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_scrollbar_s *data);
static void ft80x_cmd_setfont(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_setfont_s *data);
static void ft80x_cmd_setmatrix(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_sketch(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_sketch_s *data);
static void ft80x_cmd_slider(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_slider_s *data);
static void ft80x_cmd_snapshot(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_snapshot_s *data);
static void ft80x_cmd_spinner(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_spinner_s *data);
static void ft80x_cmd_stop(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_swap(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_text(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_text_s *data);
static void ft80x_cmd_toggle(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_toggle_s *data);
static void ft80x_cmd_track(FAR struct ft80x_dev_s *priv,
              FAR struct ft80x_data_track_s *data);
static void ft80x_cmd_translate(FAR struct ft80x_dev_s *priv,
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

/* This is the FT80x driver instance (only a single device is supported for now) */

static const struct ft80x_dev_s g_ft80x_lcddev;

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
  unsigned int ncmds;
  unsigned int i;
  int ret = OK;

  DEBUGASSERT(priv != NULL && dl != NULL);

  ncmds = dl->ncmds;
  cmd   = &dl->cmd;

  for (i = 0; i < ncmds; i++)
    {
      /* Process the command */

      switch (cmd->cmd)
        {
          case FT80X_CMD_APPEND:  /* Append memory to a display list */
            ret = ft80x_cmd_append(priv,
                    (FAR struct ft80x_data_append_s *)&cmd->data);
            break;

          case FT80X_CMD_BGCOLOR:  /* Set the background color */
            ret = ft80x_cmd_bgcolor(priv,
                    (FAR struct ft80x_data_bgcolor_s *)&cmd->data);
            break;

          case FT80X_CMD_BITMAP_TRANSFORM:
            ret = ft80x_cmd_bitmaptransform(priv,
                    (FAR struct ft80x_data_bitmaptransform_s *)&cmd->data);
            break;

          case FT80X_CMD_BUTTON:  /* Draw a button */
            ret = ft80x_cmd_button(priv,
                    (FAR struct ft80x_data_button_s *)&cmd->data);
            break;

          case FT80X_CMD_CALIBRATE:  /* Execute touchscreen calibration routine */
            ret = ft80x_cmd_calibrate(priv,
                    (FAR struct ft80x_data_calibrate_s *)&cmd->data);
            break;

          case FT80X_CMD_CLOCK:  /* Draw an analog clock */
            ret = ft80x_cmd_clock(priv,
                    (FAR struct ft80x_data_clock_s *)&cmd->data);
            break;

          case FT80X_CMD_COLDSTART:  /* Set co-processor engine state to default values */
            ret = ft80x_cmd_coldstart(priv);
            break;

          case FT80X_CMD_DIAL:  /* Draw a rotary dial control */
            ret = ft80x_cmd_dial(priv,
                    (FAR struct ft80x_data_dial_s *)&cmd->data);
            break;

          case FT80X_CMD_DLSTART:  /* Start a new display list */
            ret = ft80x_cmd_dlstart(priv);
            break;

          case FT80X_CMD_FGCOLOR:  /* Set the foreground color */
            ret = ft80x_cmd_fgcolor(priv,
                    (FAR struct ft80x_data_fgcolor_s *)&cmd->data);
            break;

          case FT80X_CMD_GAUGE:  /* Draw a gauge */
            ret = ft80x_cmd_gauge(priv,
                    (FAR struct ft80x_data_gauge_s *)&cmd->data);
            break;

          case FT80X_CMD_GETMATRIX:  /* Retrieves the current matrix coefficients */
            ret = ft80x_cmd_getmatrix(priv,
                    (FAR struct ft80x_data_getmatrix_s *)&cmd->data);
            break;

          case FT80X_CMD_GETPROPS:
            ret = ft80x_cmd_getprops(priv,
                    (FAR struct ft80x_data_getprops_s *)&cmd->data);
            break;

          case FT80X_CMD_GETPTR:
            ret = ft80x_cmd_getptr(priv,
                    (FAR struct ft80x_data_getptr_s *)&cmd->data);
            break;

          case FT80X_CMD_GRADCOLOR:  /* Set 3D effects for BUTTON and KEYS highlight colors */
            ret = ft80x_cmd_gradcolor(priv,
                    (FAR struct ft80x_data_gradcolor_s *)&cmd->data);
            break;

          case FT80X_CMD_GRADIENT:
            ret = ft80x_cmd_gradient(priv,
                    (FAR struct ft80x_data_gradient_s *)&cmd->data);
            break;

          case FT80X_CMD_HAMMERAUX:
            ret = ft80x_cmd_hammeraux(priv,
                    (FAR struct ft80x_data_hammeraux_s *)&cmd->data);
            break;

          case FT80X_CMD_IDCT:
            ret = ft80x_cmd_idct(priv,
                    (FAR struct ft80x_data_idct_s *)&cmd->data);
            break;

          case FT80X_CMD_INFLATE:  /* Decompress data into memory */
            ret = ft80x_cmd_inflate(priv,
                    (FAR struct ft80x_data_inflate_s *)&cmd->data);
            break;

          case FT80X_CMD_INTERRUPT:  /* Trigger interrupt INT_CMDFLAG */
            ret = ft80x_cmd_interrupt(priv,
                    (FAR struct ft80x_data_interrupt_s *)&cmd->data);
            break;

          case FT80X_CMD_KEYS:  /* Draw a row of keys */
            ret = ft80x_cmd_keys(priv,
                    (FAR struct ft80x_data_keys_s *)&cmd->data);
            break;

          case FT80X_CMD_LOADIDENTITY:  /* Set the current matrix to identity */
            ret = ft80x_cmd_loadidentity(priv);
            break;

          case FT80X_CMD_LOADIMAGE:  /* Load a JPEG image */
            ret = ft80x_cmd_loadimage(priv,
                    (FAR struct ft80x_data_loadimage_s *)&cmd->data);
            break;

          case FT80X_CMD_LOGO:  /* Play a device logo animation */
            ret = ft80x_cmd_logo(priv);
            break;

          case FT80X_CMD_MEMCPY:  /* Copy a block of memory */
            ret = ft80x_cmd_memcpy(priv,
                    (FAR struct ft80x_data_memcpy_s *)&cmd->data);
            break;

          case FT80X_CMD_MEMCRC:  /* Compute a CRC for memory */
            ret = ft80x_cmd_memcrc(priv,
                    (FAR struct ft80x_data_memcrc_s *)&cmd->data);
            break;

          case FT80X_CMD_MEMSET:  /* Fill memory with a byte value */
            ret = ft80x_cmd_memset(priv,
                    (FAR struct ft80x_data_memset_s *)&cmd->data);
            break;

          case FT80X_CMD_MEMWRITE:  /* Write bytes into memory */
            ret = ft80x_cmd_memwrite(priv,
                    (FAR struct ft80x_data_memwrite_s *)&cmd->data);
            break;

          case FT80X_CMD_MEMZERO:  /* Write zero to a block of memory */
            ret = ft80x_cmd_memzero(priv,
                    (FAR struct ft80x_data_memzero_s *)&cmd->data);
            break;

          case FT80X_CMD_NUMBER:  /* Draw a decimal number */
            ret = ft80x_cmd_number(priv,
                    (FAR struct ft80x_data_number_s *)&cmd->data);
            break;

          case FT80X_CMD_PROGRESS:  /* Draw a progress bar */
            ret = ft80x_cmd_progress(priv,
                    (FAR struct ft80x_data_progress_s *)&cmd->data);
            break;

          case FT80X_CMD_REGREAD:  /* Read a register value */
            ret = ft80x_cmd_regread(priv,
                    (FAR struct ft80x_data_regread_s *)&cmd->data);
            break;

          case FT80X_CMD_ROTATE:  /* Apply a rotation to the current matrix */
            ret = ft80x_cmd_rotate(priv,
                    (FAR struct ft80x_data_rotate_s *)&cmd->data);
            break;

          case FT80X_CMD_SCALE:  /* Apply a scale to the current matrix */
            ret = ft80x_cmd_scale(priv,
                    (FAR struct ft80x_data_scale_s *)&cmd->data);
            break;

          case FT80X_CMD_SCREENSAVER:
            ret = ft80x_cmd_screensaver(priv,
                    (FAR struct ft80x_data_screensaver_s *)&cmd->data);
            break;

          case FT80X_CMD_SCROLLBAR:  /* Draw a scroll bar */
            ret = ft80x_cmd_scrollbar(priv,
                    (FAR struct ft80x_data_scrollbar_s *)&cmd->data);
            break;

          case FT80X_CMD_SETFONT:
            ret = ft80x_cmd_setfont(priv,
                    (FAR struct ft80x_data_setfont_s *)&cmd->data);
            break;

          case FT80X_CMD_SETMATRIX:  /* Write current matrix as a bitmap transform */
            ret = ft80x_cmd_setmatrix(priv,
                    (FAR struct ft80x_data_setmatrix_s *)&cmd->data);
            break;

          case FT80X_CMD_SKETCH:  /* Start a continuous sketch update */
            ret = ft80x_cmd_sketch(priv,
                    (FAR struct ft80x_data_sketch_s *)&cmd->data);
            break;

          case FT80X_CMD_SLIDER:  /* Draw a slider */
            ret = ft80x_cmd_slider(priv,
                    (FAR struct ft80x_data_slider_s *)&cmd->data);
            break;

          case FT80X_CMD_SNAPSHOT:  /* Take a snapshot of the current screen */
            ret = ft80x_cmd_snapshot(priv,
                    (FAR struct ft80x_data_snapshot_s *)&cmd->data);
            break;

          case FT80X_CMD_SPINNER:  /* Start an animated spinner */
            ret = ft80x_cmd_spinner(priv,
                    (FAR struct ft80x_data_spinner_s *)&cmd->data);
            break;

          case FT80X_CMD_STOP:  /* Stop any spinner, screensave, or sketch */
            ret = ft80x_cmd_stop(priv);
            break;

          case FT80X_CMD_SWAP:  /* Swap the current display list */
            ret = ft80x_cmd_swap(priv);
            break;

          case FT80X_CMD_TEXT:  /* Draw text */
            ret = ft80x_cmd_text(priv,
                    (FAR struct ft80x_data_text_s *)&cmd->data);
            break;

          case FT80X_CMD_TOGGLE:  /* Draw a toggle switch */
            ret = ft80x_cmd_toggle(priv,
                    (FAR struct ft80x_data_toggle_s *)&cmd->data);
            break;

          case FT80X_CMD_TRACK:
            ret = ft80x_cmd_track(priv,
                    (FAR struct ft80x_data_track_s *)&cmd->data);
            break;

          case FT80X_CMD_TRANSLATE:  /* Apply a translation to the current matrix */
            ret = ft80x_cmd_translate(priv,
                    (FAR struct ft80x_data_translate_s *)&cmd->data);
            break;

          default:
            lcderr("ERROR: Unrecognized command in display list: 0x8lx\n",
                   (unsigned long)cmd->cmd);
            return -EINVAL;
        }
      /* Point to the next command in the display list */

      cmd = (FAR struct ft80x_dlcmd_s *)((uintptr_t)cmd + cmd->len);
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
  FAR struct ft80x_dev_s *priv = &g_ft80x_lcddev;

#if defined(CONFIG_LCD_FT80X_SPI)
  DEBUGASSERT(spi != NULL && lower != NULL);
#elif defined(CONFIG_LCD_FT80X_I2C)
  DEBUGASSERT(i2c != NULL && lower != NULL);
#endif

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

  /* Register the FT80x character driver */

  ret = register_driver(DEVNAME, &ft80x_fops, 0666, priv);
  if (ret < 0)
    {
      sem_destory(&priv->exclsem);
    }

  return ret;
  return &priv->dev;
}

#endif /* CONFIG_LCD_FT80X */
