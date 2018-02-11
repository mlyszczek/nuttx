/****************************************************************************
 * include/nuttx/lcd/ft800.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Touch Screen Controller, ADS7843," Burr-Brown Products from Texas
 *    Instruments, SBAS090B, September 2000, Revised May 2002"
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

#ifndef __INCLUDE_NUTTX_LCD_FT800_H
#define __INCLUDE_NUTTX_LCD_FT800_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_LCD_FT800

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Pins relevant to software control.  The FT800 is a 48-pin part.  Most of
 * the pins are associated with the TFT panel and other board-related
 * support.  A few a relevant to software control of the part.  Those are
 * listed here:
 *
 * FT800 PIN  DIR DESCRIPTION
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

/* A reference to a structure of this type must be passed to the FT800
 * driver.  This structure provides information about the configuration
 * of the FT800 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.  The
 * memory may be read-only.
 */

struct ft800_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* I2C/SPI frequency */
#ifdef CONFIG_FT800_I2C
  uint8_t address;     /* 7-bit I2C address */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the FT800 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs. Interrupts should be
   * configured on the falling edge of nINT.
   *
   *   attach  - Attach the ADS7843E interrupt handler to the GPIO interrupt
   *   enable  - Enable or disable the GPIO interrupt
   *   clear   - Acknowledge/clear any pending GPIO interrupt
   *   pwrdown - Power down the FT800
   */

  int  (*attach)(FAR const struct ft800_config_s *state, xcpt_t isr,
                 FAR void *arg);
  void (*enable)(FAR const struct ft800_config_s *state, bool enable);
  void (*clear)(FAR const struct ft800_config_s *state);
  bool (*pwrdown)(FAR const struct ft800_config_s *state, bool pwrdown);
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
 * Name: ft800_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
 *   will register the driver as /dev/ft800.
 *
 * Input Parameters:
 *   spi     - An SPI driver instance
 *   i2c     - An I2C master driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_FT800_SPI)
int ft800_register(FAR struct spi_dev_s *spi,
                   FAR const struct ft800_config_s *config);
#elif defined(CONFIG_FT800_I2C)
int ft800_register(FAR struct i2c_master_s *i2c,
                   FAR const struct ft800_config_s *config);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_FT800 */
#endif /* __INCLUDE_NUTTX_LCD_FT800_H */
