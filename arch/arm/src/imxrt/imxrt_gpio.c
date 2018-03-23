/****************************************************************************
 * arch/arm/src/imxrt/imxrt_gpio.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "up_arch.h"
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_PADMUX_INVALID    255

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_gpio1_padmux[IMXRT_GPIO_NPINS] =
{
#warning REVISIT: Need PADMUX INDEX definitions in chip/imxrt_iomuxc.h
};

static const uint8_t g_gpio2_padmux[IMXRT_GPIO_NPINS] =
{
#warning REVISIT: Need PADMUX INDEX definitions in chip/imxrt_iomuxc.h
};

static const uint8_t g_gpio3_padmux[IMXRT_GPIO_NPINS] =
{
#warning REVISIT: Need PADMUX INDEX definitions in chip/imxrt_iomuxc.h
};

static const uint8_t g_gpio4_padmux[IMXRT_GPIO_NPINS] =
{
#warning REVISIT: Need PADMUX INDEX definitions in chip/imxrt_iomuxc.h
};

static FAR const uint8_t *g_gpio_padmux[IMXRT_GPIO_NPORTS + 1] =
{
  g_gpio1_padmux,                    /* GPIO1 */
  g_gpio2_padmux,                    /* GPIO2 */
  g_gpio3_padmux,                    /* GPIO3 */
  g_gpio4_padmux,                    /* GPIO4 */
  NULL                               /* End of list */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_gpio_dirout
 ****************************************************************************/

static inline void imxrt_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_GDIR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imxrt_gpio_dirin
 ****************************************************************************/

static inline void imxrt_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_GDIR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imxrt_gpio_setoutput
 ****************************************************************************/

static void imxrt_gpio_setoutput(int port, int pin, bool value)
{
  uintptr_t regaddr = IMXRT_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  if (value)
    {
      regval |= GPIO_PIN(pin);
    }
  else
    {
      regval &= ~GPIO_PIN(pin);
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: imxrt_gpio_getinput
 ****************************************************************************/

static inline bool imxrt_gpio_getinput(int port, int pin)
{
  uintptr_t regaddr = IMXRT_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imxrt_gpio_configinput
 ****************************************************************************/

static int imxrt_gpio_configinput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  FAR const uint8_t *table;
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  unsigned int index;

  /* Configure pin as in input */

  imxrt_gpio_dirin(port, pin);

  /* Configure pin as a GPIO */

  table = g_gpio_padmux[port];
  if (table == NULL)
    {
      return -EINVAL;
    }

  index = (unsigned int)table[pin];
  if (index >= IMXRT_PADMUX_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = IMXRT_PADMUX_ADDRESS(index);
  putreg32(PADMUX_MUXMODE_ALT5, regaddr);

  /* Configure pin pad settings */

  index = imxrt_padmux_map(index);
  if (index >= IMXRT_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = IMXRT_PADCTL_ADDRESS(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imxrt_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Name: imxrt_gpio_configoutput
 ****************************************************************************/

static inline int imxrt_gpio_configoutput(gpio_pinset_t pinset)
{
  int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  /* Set the output value */

  imxrt_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  imxrt_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpio_configperiph
 ****************************************************************************/

static inline int imxrt_gpio_configperiph(gpio_pinset_t pinset)
{
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t value;
  unsigned int index;

  /* Configure pin as a peripheral */

  index = ((pinset & GPIO_PADMUX_MASK) >> GPIO_PADMUX_SHIFT);
  regaddr = IMXRT_PADMUX_ADDRESS(index);

  value = ((pinset & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT);
  regval = (value << PADMUX_MUXMODE_SHIFT);

  putreg32(regval, regaddr);

  /* Configure pin pad settings */

  index = imxrt_padmux_map(index);
  if (index >= IMXRT_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = IMXRT_PADCTL_ADDRESS(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imxrt_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int ret;

  /* Configure the pin as an input initially to avoid any spurious outputs */

  flags = enter_critical_section();

  /* Configure based upon the pin mode */

  switch (pinset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        {
          /* Configure the pin as a GPIO input */

          ret = imxrt_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = imxrt_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = imxrt_gpio_configoutput(pinset);
            }
        }
        break;

      case GPIO_PERIPH:
        {
          /* Configure the pin as a peripheral */

          ret = imxrt_gpio_configperiph(pinset);
        }
        break;

#ifdef CONFIG_IMXRT_GPIO_IRQ
      case GPIO_INTERRUPT:
        {
          /* Configure the pin as a GPIO input */

          ret = imxrt_gpio_configinput(pinset);
          if (ret == OK)
            {
              ret = imxrt_gpioirq_configure(pinset);
            }
        }
        break;
#endif

      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/************************************************************************************
 * Name: imxrt_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  flags = enter_critical_section();
  imxrt_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/************************************************************************************
 * Name: imxrt_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  flags = enter_critical_section();
  value = imxrt_gpio_getinput(port, pin);
  leave_critical_section(flags);
  return value;
}
