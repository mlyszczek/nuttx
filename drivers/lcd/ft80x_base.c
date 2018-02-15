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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ft80x.h>

#include <arch/irq.h>

#include "ft80x.h"

#ifdef CONFIG_LCD_FT80X

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/

/* LCD Data Transfer Methods */

static int ft80x_putrun(fb_coord_t row, fb_coord_t col,
                        FAR const uint8_t *buffer, size_t npixels);
static int ft80x_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                        size_t npixels);

/* LCD Configuration */

static int ft80x_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo);
static int ft80x_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int ft80x_getpower(struct lcd_dev_s *dev);
static int ft80x_setpower(struct lcd_dev_s *dev, int power);
static int ft80x_getcontrast(struct lcd_dev_s *dev);
static int ft80x_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefore be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint8_t g_runbuffer[FT80X_DEV_ROWSIZE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = FT80X_DEV_COLORFMT,  /* Color format: B&W */
  .xres    = FT80X_DEV_XRES,      /* Horizontal resolution in pixel columns */
  .yres    = FT80X_DEV_YRES,      /* Vertical resolution in pixel rows */
  .nplanes = 1,                   /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = ft80x_putrun,             /* Put a run into LCD memory */
  .getrun = ft80x_getrun,             /* Get a run from LCD memory */
  .buffer = (FAR uint8_t *)g_runbuffer, /* Run scratch buffer */
  .bpp    = FT80X_DEV_BPP,            /* Bits-per-pixel */
};

/* This is the OLED driver instance (only a single device is supported for now) */

static const struct lcd_dev_s g_ft80x_lcddev;

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ft80x_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 * Input Parameters:
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ft80x_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                          size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)&g_ft80x_lcddev;

#warning Missing logic
  return OK;
}
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ft80x_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD.
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ft80x_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ft80x_dev_s *priv = &g_ft80x_lcddev;

#warning Missing logic
  return OK;
}
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ft80x_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ft80x_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev != NULL && vinfo != NULL);

  lcdinfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);

  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ft80x_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ft80x_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo != NULL&& planeno == 0);

  lcdinfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);

  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ft80x_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ft80x_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

#warning Missing logic
  return 0;
}

/**************************************************************************************
 * Name:  ft80x_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ft80x_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)dev;
  DEBUGASSERT(priv != NULL && (unsigned)power <= CONFIG_LCD_MAXPOWER);

  lcdinfo("power: %d [%d]\n", power, priv->on ? CONFIG_LCD_MAXPOWER : 0);

  if (power <= 0)
    {
      /* Turn the display off */
#warning Missing logic
    }
  else
    {
      /* Configure display and turn the display on */
#warning Missing logic
    }

  return OK;
}

/**************************************************************************************
 * Name:  ft80x_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ft80x_getcontrast(struct lcd_dev_s *dev)
{
  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  lcdinfo("contrast: %d\n", priv->contrast);

#warning Missing logic
  return 0;
}

/**************************************************************************************
 * Name:  ft80x_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ft80x_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  FAR struct ft80x_dev_s *priv = (FAR struct ft80x_dev_s *)dev;

  lcdinfo("contrast: %d\n", contrast);
  DEBUGASSERT(priv != NULL && contrast <= CONFIG_LCD_MAXCONTRAST);

#warning Missing logic
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
  FAR struct ft80x_dev_s *priv = &g_ft80x_lcddev;

#if defined(CONFIG_LCD_FT80X_SPI)
  DEBUGASSERT(spi != NULL && lower != NULL);
#elif defined(CONFIG_LCD_FT80X_I2C)
  DEBUGASSERT(i2c != NULL && lower != NULL);
#endif

  /* Initialize the device state structure */

  priv->dev.getvideoinfo = ft80x_getvideoinfo;
  priv->dev.getplaneinfo = ft80x_getplaneinfo;
  priv->dev.getpower     = ft80x_getpower;
  priv->dev.setpower     = ft80x_setpower;
  priv->dev.getcontrast  = ft80x_getcontrast;
  priv->dev.setcontrast  = ft80x_setcontrast;

  /* Save the lower level interface and configuration information */

  priv->lower            = lower;

#ifdef CONFIG_LCD_FT80X_SPI
  /* Remember the SPI configuration */

  priv->spi = dev;

  /* Configure the SPI */

  ft80x_configspi(priv->spi);

#else
  /* Remember the I2C configuration */

  priv->i2c  = dev;
#endif

  /* Power on and configure display */

  ft80x_setpower(&priv->dev, true);
  return &priv->dev;
}

#endif /* CONFIG_LCD_FT80X */
