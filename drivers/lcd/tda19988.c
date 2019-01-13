/****************************************************************************
 * drivers/lcd/tda19988.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/lcd/tda19988.h>

#include "tda19988.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one TDA19988 driver instance */

struct tda1988_dev_s
{
  /* The contained lower half driver instance */

  FAR const struct tda19088_lower_s *lower;

  /* Upper half driver state */

  uint8_t page;  /* Currently selected page */
}

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int     tda19988_select_page(FAR struct tda1988_dev_s *priv,
                                    uint8_t page);
static int     tda19988_hdmi_getreg(FAR struct tda1988_dev_s *priv,
                 uint8_t page, uint8_t regaddr, FAR uint8_t *regval);
static int     tda19988_hdmi_putreg(FAR struct tda1988_dev_s *priv,
                 uint8_t page, uint8_t regaddr, uint8_t regval);
static int     tda19988_hdmi_modifyreg(FAR struct tda1988_dev_s *priv,
                 uint8_t page, uint8_t regaddr, uint8_t clrbits,
                 uint8_t setbits)

/* Character driver methods */

static ssize_t tda19988_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t tda19988_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int     tda19988_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations tda19988_fops =
{
  NULL,           /* open */
  NULL,           /* close */
  tda19988_read,  /* read */
  tda19988_write, /* write */
  NULL,           /* seek */
  NULL            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , tda19988_poll /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tda19988_select_page
 *
 * Description:
 *   Select the HDMI page (if not already selected)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_select_page(FAR struct tda1988_dev_s *priv, uint8_t page)
{
  uint8_t buffer[2];
  int ret = OK;

  /* Check if we need to select a new page for this transfer */

  if (page != HDMI_NO_PAGE && page != priv->page)
    {
      buffer[0] = HDMI_PAGE_SELECT_REG;
      buffer[1] = page;

      ret = i2c_write(priv->lower->hdmi.i2c, &priv->lower->hdmi.config,
                      buffer, 2);
      if (ret >= 0)
        {
          priv->page = page;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tda19988_hdmi_getreg
 *
 * Description:
 *   Read the value from one TDA19988 register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_getreg(FAR struct tda1988_dev_s *priv, uint8_t page,
                                uint8_t regaddr, FAR uint8_t *regval)
{
  uint8_t buffer[1];
  int ret;

  DEBUGASSERT(priv != NULL && regval != NULL);

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcdwarn("WARNING: Failed to select page %02x: %d\n", page, ret);
      return ret;
    }

  /* Write the register address and read the register value */

  buffer[0] = regaddr;
  ret = i2c_writeread(priv->lower->hdmi.i2c, &priv->lower->hdmi.config,
                      buffer, 1, regval, 1);
  if (ret < 0)
    {
      lcdwarn("WARNING: i2c_writeread() failed: %d\n", ret);
      return -1;
    }

  lcdinfo("Read: %02x:%02x->%02x\n", page, regaddr, *regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_hdmi_putreg
 *
 * Description:
 *   Write a value to one TDA19988 register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_putreg(FAR struct tda1988_dev_s *priv,
                                uint8_t page, uint8_t regaddr,
                                uint8_t regval)
{
  uint8_t buffer[2];
  int ret;

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcdwarn("WARNING: Failed to select page %02x: %d\n", page, ret);
      return ret;
    }

  /* Write the register address and the register value */

  buffer[0] = regaddr;
  buffer[1] = regval;

  ret = i2c_write(priv->lower->hdmi.i2c, &priv->lower->hdmi.config,
                  buffer, 2);
  if (ret < 0)
    {
      lcdwarn("WARNING: i2c_write() failed: %d\n", ret);
      return ret;
    }

  lcdinfo("Read: %02x:%02x<-%02x\n", page, regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: tda19988_hdmi_putreg
 *
 * Description:
 *   Write a value to one TDA19988 register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int tda19988_hdmi_modifyreg(FAR struct tda1988_dev_s *priv,
                                   uint8_t page, uint8_t regaddr,
                                   uint8_t clrbits, uint8_t setbits)
{
  uint8_t regval;
  int ret;

  /* Select the HDMI page */

  ret = tda19988_select_page(priv, page);
  if (ret < 0)
    {
      lcdwarn("WARNING: Failed to select page %02x: %d\n", page, ret);
      return ret;
    }

  /* Read the register contents */

  ret = tda19988_hdmi_getreg(priv, page, regaddr, &regval);
  if (ret < 0)
    {
      lcdwarn("WARNING: tda19988_hdmi_getreg failed: %d\n", ret);
      return ret;
    }

  /* Modify the register content */

  regval &= ~clrbits;
  regval |= setbits;

  /* Write the modified regiser content */

  ret = tda19988_hdmi_putreg(priv, page, regaddr, regval);
  if (ret < 0)
    {
      lcdwarn("WARNING: tda19988_hdmi_putreg failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: tda19988_read
 ****************************************************************************/

static ssize_t tda19988_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: tda19988_write
 ****************************************************************************/

static ssize_t tda19988_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: tda19988_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int tda19988_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tda19988_register
 *
 * Description:
 *   Initialize and register the the TDA19988 driver at 'devpath'
 *
 * Input Parameters:
 *   devpath - The location to register the TDA19988 driver instance
 *   lower   - The interface to the the TDA19988 lower half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int tda19988_register(FAR const char *devpath,
                      FAR const struct tda19088_lower_s *lower);
{
  FAR struct tda1988_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && lower != NULL);

  /* Allocate an instance of the TDA19988 driver */

  priv = (FAR struct tda1988_dev_s *)kmm_zalloc(sizeof(struct tda1988_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the driver structure */

  priv->lower = lower;
  priv->page  = HDMI_NO_PAGE;

  /* Register the driver */

  ret = register_driver(devpath, &tda19988_fops, 0666, NULL);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  return OK;
}
