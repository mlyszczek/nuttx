/****************************************************************************
 * arch/arm/src/imxrt/imxrt_irq.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include "up_arch.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table is indexed by the Pad Mux register index and provides the index
 * to the corresponding Pad Control register.
 *
 * REVISIT:  This could be greatly simplified:  The Pad Control registers
 * map 1-to-1 with the Pad Mux registers except for two regions where
 * there are no corresponding Pad Mux registers.  The entire table could be
 * replaced to two range checks and the appropriate offset added to the Pad
 * Mux Register index.
 */

static const uint8_t g_mux2ctl_map[IMX_PADMUX_NREGISTERS] =
{
  /* The first mappings are simple 1-to-1 mappings.  This may be a little wasteful */
#warning REVISIT: Need PADMUX CTRL definitions in chip/imxrt_iomuxc.h


  /* There is then a group of Pad Control registers with no Pad Mux register counterpart */


  /* The mapping is again 1-to-1 with an offset for the above registers that
   * have no Pad Mux register counterpart.
   */


  /* There is a second group of Pad Control registers with no Pad Mux register counterpart */

                                    /* IMX_PADCTL_JTAG_TMS_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_MOD_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TRSTB_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TDI_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TCK_INDEX - No counterpart */
                                    /* IMX_PADCTL_JTAG_TDO_INDEX - No counterpart */

  /* The mapping is again 1-to-1 with an offset for the above registers that
   * have no Pad Mux register counterpart.
   */

};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_padmux_map
 *
 * Description:
 *   This function map a Pad Mux register index to the corresponding Pad
 *   Control register index.
 *
 ****************************************************************************/

unsigned int imxrt_padmux_map(unsigned int padmux)
{
  DEBUGASSERT(padmux < IMX_PADMUX_NREGISTERS);
  return (unsigned int)g_mux2ctl_map[padmux];
}

/****************************************************************************
 * Name: imxrt_iomux_configure
 *
 * Description:
 *   This function writes the encoded pad configuration to the Pad Control
 *   register.
 *
 ****************************************************************************/

int imxrt_iomux_configure(uintptr_t padctl, iomux_pinset_t ioset)
{
  uint32_t regval = 0;
  uint32_t value;

  /* Select CMOS input or Schmitt Trigger input */

  if ((ioset & IOMUX_SCHMITT_TRIGGER) != 0)
    {
      regval |= PADCTL_SRE;
    }

  /* Select drive strength */

  value = (ioset & IOMUX_DRIVE_MASK) >> IOMUX_DRIVE_SHIFT;
  regval |= PADCTL_DSE(value);

  /* Select spped */

  value = (ioset & IOMUX_SPEED_MASK) >> IOMUX_SPEED_SHIFT;
  regval |= PADCTL_SPEED(value);

  /* Select CMOS output or Open Drain outpout */

  if ((ioset & IOMUX_OPENDRAIN) != 0)
    {
      regval |= PADCTL_ODE;
    }

  /* Handle pull/keep selection */

  switch (ioset & _IOMUX_PULLTYPE_MASK)
    {
      default:
      case _IOMUX_PULL_NONE:
        break;

      case _IOMUX_PULL_KEEP:
        {
          regval |= PADCTL_PKE;
        }
        break;

      case _IOMUX_PULL_ENABLE:
        {
          regval |= (PADCTL_PKE | PADCTL_PUE);

          value   = (ioset & _IOMUX_PULLDESC_MASK) >> _IOMUX_PULLDESC_SHIFT;
          regval |= PADCTL_PUS(value);
        }
        break;
    }

  /* Select slow/fast slew rate */

  if ((ioset & IOMUX_SLEW_FAST) != 0)
    {
      regval |= PADCTL_HYS;
    }

  /* Write the result to the specified Pad Control register */

  putreg32(regval, padctl);
  return OK;
}
