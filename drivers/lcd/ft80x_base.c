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

static void ft80x_cmd_text(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t font, uint16_t options,
              FAR const ft_char8_t *s);
static void ft80x_cmd_number(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t font, uint16_t options, int32_t n);
static void ft80x_cmd_loadidentity(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_toggle(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t font, uint16_t options,
              uint16_t state, const ft_char8_t *s);
static void ft80x_cmd_gauge(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t r, uint16_t options, uint16_t major,
              uint16_t minor, uint16_t val, uint16_t range);
static void ft80x_cmd_regread(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t result);
static void ft80x_cmd_getprops(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t w, uint32_t h);
static void ft80x_cmd_memcpy(FAR struct ft80x_dev_s *priv, uint32_t dest,
              uint32_t src, uint32_t num);
static void ft80x_cmd_spinner(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, uint16_t style, uint16_t scale);
static void ft80x_cmd_bgcolor(FAR struct ft80x_dev_s *priv, uint32_t c);
static void ft80x_cmd_swap(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_inflate(FAR struct ft80x_dev_s *priv, uint32_t ptr);
static void ft80x_cmd_translate(FAR struct ft80x_dev_s *priv, int32_t tx,
              int32_t ty);
static void ft80x_cmd_stop(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_slider(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t h, uint16_t options,
              uint16_t val, uint16_t range);
static void ft80x_cmd_interrupt(FAR struct ft80x_dev_s *priv, uint32_t ms);
static void ft80x_cmd_fgcolor(FAR struct ft80x_dev_s *priv, uint32_t c);
static void ft80x_cmd_rotate(FAR struct ft80x_dev_s *priv, int32_t a);
static void ft80x_cmd_button(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t h, int16_t font,
              uint16_t options, FAR const ft_char8_t *s);
static void ft80x_cmd_memwrite(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t num);
static void ft80x_cmd_Scrollbar(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t h, uint16_t options,
              uint16_t val, uint16_t size, uint16_t range);
static void ft80x_cmd_getmatrix(FAR struct ft80x_dev_s *priv, int32_t a,
              int32_t b, int32_t c, int32_t d, int32_t e, int32_t f);
static void ft80x_cmd_sketch(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, uint16_t w, uint16_t h, uint32_t ptr,
              uint16_t format);
static void ft80x_cmd_csketch(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, uint16_t w, uint16_t h, uint32_t ptr,
              uint16_t format, uint16_t freq);
static void ft80x_cmd_memset(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t value, uint32_t num);
static void ft80x_cmd_calibrate(FAR struct ft80x_dev_s *priv,
              uint32_t result);
static void ft80x_cmd_setfont(FAR struct ft80x_dev_s *priv, uint32_t font,
              uint32_t ptr);
static void ft80x_cmd_bitmap_transform(FAR struct ft80x_dev_s *priv,
              int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2,
              int32_t y2, int32_t tx0, int32_t ty0, int32_t tx1,
              int32_t ty1, int32_t tx2, int32_t ty2, uint16_t result);
static void ft80x_cmd_gradcolor(FAR struct ft80x_dev_s *priv, uint32_t c);
static void ft80x_cmd_append(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t num);
static void ft80x_cmd_memzero(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t num);
static void ft80x_cmd_scale(FAR struct ft80x_dev_s *priv, int32_t sx,
              int32_t sy);
static void ft80x_cmd_clock(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t r, uint16_t options, uint16_t h,
              uint16_t m, uint16_t s, uint16_t ms);
static void ft80x_cmd_gradient(FAR struct ft80x_dev_s *priv, int16_t x0,
              int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1,
              uint32_t rgb1);
static void ft80x_cmd_setmatrix(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_track(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t h, int16_t tag);
static void ft80x_cmd_getptr(FAR struct ft80x_dev_s *priv, uint32_t result);
static void ft80x_cmd_progress(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t h, uint16_t options,
              uint16_t val, uint16_t range);
static void ft80x_cmd_coldstart(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_keys(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t w, int16_t h, int16_t font,
              uint16_t options, FAR const ft_char8_t *s);
static void ft80x_cmd_dial(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t r, uint16_t options, uint16_t val);
static void ft80x_cmd_loadimage(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t options);
static void ft80x_cmd_dlstart(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_snapshot(FAR struct ft80x_dev_s *priv, uint32_t ptr);
static void ft80x_cmd_screensaver(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_memcrc(FAR struct ft80x_dev_s *priv, uint32_t ptr,
              uint32_t num, uint32_t result);
static void ft80x_cmd_logo(FAR struct ft80x_dev_s *priv);
static void ft80x_cmd_calibrate(FAR struct ft80x_dev_s *priv,
              uint32_t result);
static void ft80x_cmd_text(FAR struct ft80x_dev_s *priv, int16_t x,
              int16_t y, int16_t font, uint16_t options,\
              FAR const ft_char8_t *s);

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
