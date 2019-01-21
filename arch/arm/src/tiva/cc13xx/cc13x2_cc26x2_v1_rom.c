/************************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x2_cc26x2_v1_rom.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's setup_rom.c file which has a fully compatible BSD license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

/* Hardware headers */

#include "../inc/hw_types.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_adi.h"
#include "../inc/hw_adi_2_refsys.h"
#include "../inc/hw_adi_3_refsys.h"
#include "../inc/hw_adi_4_aux.h"
#include "../inc/hw_aon_batmon.h"
#include "../inc/hw_aux_sysif.h"
#include "../inc/hw_ccfg.h"
#include "../inc/hw_ddi_0_osc.h"
#include "../inc/hw_fcfg1.h"

/* Driverlib headers */

#include "ddi.h"
#include "ioc.h"
#include "osc.h"
#include "sys_ctrl.h"
#include "setup_rom.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: rom_setup_stepvaddrtrimto
 ************************************************************************************/

void rom_setup_stepvaddrtrimto(uint32_t toCode)
{
  uint32_t pmctlResetctl_reg;
  int32_t targetTrim;
  int32_t currentTrim;

  targetTrim =
    SetupSignExtendVddrTrimValue(toCode &
                                 (ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_M >>
                                  ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_S));
  currentTrim =
    SetupSignExtendVddrTrimValue((HWREGB(ADI3_BASE + ADI_3_REFSYS_O_DCDCCTL0) &
                                  ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_M) >>
                                 ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_S);

  if (targetTrim != currentTrim)
    {
      pmctlResetctl_reg =
        (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_RESETCTL) &
         ~AON_PMCTL_RESETCTL_MCU_WARM_RESET_M);
      if (pmctlResetctl_reg & AON_PMCTL_RESETCTL_VDDR_LOSS_EN_M)
        {
          HWREG(AON_PMCTL_BASE + AON_PMCTL_O_RESETCTL) =
            (pmctlResetctl_reg & ~AON_PMCTL_RESETCTL_VDDR_LOSS_EN_M);
          HWREG(AON_RTC_BASE + AON_RTC_O_SYNC); /* Wait for VDDR_LOSS_EN
                                                 * setting to propagate */

        }

      while (targetTrim != currentTrim)
        {
          HWREG(AON_RTC_BASE + AON_RTC_O_SYNCLF);       /* Wait for next edge
                                                         * on SCLK_LF (positive
                                                         * or negative) */

          if (targetTrim > currentTrim)
            currentTrim++;
          else
            currentTrim--;

          HWREGB(ADI3_BASE + ADI_3_REFSYS_O_DCDCCTL0) = ((HWREGB
                                                          (ADI3_BASE +
                                                           ADI_3_REFSYS_O_DCDCCTL0)
                                                          &
                                                          ~ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_M)
                                                         |
                                                         ((((uint32_t)
                                                            currentTrim) <<
                                                           ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_S)
                                                          &
                                                          ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_M));
        }

      HWREG(AON_RTC_BASE + AON_RTC_O_SYNCLF);   /* Wait for next edge on
                                                 * SCLK_LF (positive or
                                                 * negative) */

      if (pmctlResetctl_reg & AON_PMCTL_RESETCTL_VDDR_LOSS_EN_M)
        {
          HWREG(AON_RTC_BASE + AON_RTC_O_SYNCLF);       /* Wait for next edge
                                                         * on SCLK_LF (positive
                                                         * or negative) */

          HWREG(AON_RTC_BASE + AON_RTC_O_SYNCLF);       /* Wait for next edge
                                                         * on SCLK_LF (positive
                                                         * or negative) */

          HWREG(AON_PMCTL_BASE + AON_PMCTL_O_RESETCTL) = pmctlResetctl_reg;
          HWREG(AON_RTC_BASE + AON_RTC_O_SYNC); /* And finally wait for
                                                 * VDDR_LOSS_EN setting to
                                                 * propagate */

        }
    }
}

/************************************************************************************
 * Name: rom_setup_coldreset_from_shutdown_cfg1
 ************************************************************************************/

void rom_setup_coldreset_from_shutdown_cfg1(uint32_t ccfg_modeconf)
{
  /* Check for CC1352 boost mode The combination VDDR_EXT_LOAD=0 and
   * VDDS_BOD_LEVEL=1 is defined to select boost mode */

  if (((ccfg_modeconf & CCFG_MODE_CONF_VDDR_EXT_LOAD) == 0) &&
      ((ccfg_modeconf & CCFG_MODE_CONF_VDDS_BOD_LEVEL) != 0))
    {
      /* Set VDDS_BOD trim - using masked write {MASK8:DATA8} - TRIM_VDDS_BOD
       * is bits[7:3] of ADI3..REFSYSCTL1 - Needs a positive transition on
       * BOD_BG_TRIM_EN (bit[7] of REFSYSCTL3) to latch new VDDS BOD. Set to 0
       * first to guarantee a positive transition. */

      HWREGB(ADI3_BASE + ADI_O_CLR + ADI_3_REFSYS_O_REFSYSCTL3) =
        ADI_3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN;

      *VDDS_BOD_LEVEL = 1 means that boost mode is selected
        * -Max out the VDDS_BOD trim( = VDDS_BOD_POS_31)
        * /HWREGH(ADI3_BASE + ADI_O_MASK8B + (ADI_3_REFSYS_O_REFSYSCTL1 * 2)) =
        (ADI_3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_M << 8) |
        (ADI_3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_31);
      HWREGB(ADI3_BASE + ADI_O_SET + ADI_3_REFSYS_O_REFSYSCTL3) =
        ADI_3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN;

      rom_setup_stepvaddrtrimto((HWREG(FCFG1_BASE + FCFG1_O_VOLT_TRIM) &
                           FCFG1_VOLT_TRIM_VDDR_TRIM_HH_M) >>
                          FCFG1_VOLT_TRIM_VDDR_TRIM_HH_S);
    }

  /* 1. Do not allow DCDC to be enabled if in external regulator mode.
   * Preventing this by setting both the RECHARGE and the ACTIVE bits bit in
   * the CCFG_MODE_CONF copy register (ccfg_modeconf). 2. Adjusted battery
   * monitor low limit in internal regulator mode. This is done by setting
   * AON_BATMON_FLASHPUMPP0_LOWLIM=0 in internal regulator mode. */

  if (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_PWRCTL) &
      AON_PMCTL_PWRCTL_EXT_REG_MODE)
    {
      ccfg_modeconf |=
        (CCFG_MODE_CONF_DCDC_RECHARGE_M | CCFG_MODE_CONF_DCDC_ACTIVE_M);
    }
  else
    {
      HWREGBITW(AON_BATMON_BASE + AON_BATMON_O_FLASHPUMPP0,
                AON_BATMON_FLASHPUMPP0_LOWLIM_BITN) = 0;
    }

  /* set the RECHARGE source based upon CCFG:MODE_CONF:DCDC_RECHARGE Note:
   * Inverse polarity */

  HWREGBITW(AON_PMCTL_BASE + AON_PMCTL_O_PWRCTL,
            AON_PMCTL_PWRCTL_DCDC_EN_BITN) =
    (((ccfg_modeconf >> CCFG_MODE_CONF_DCDC_RECHARGE_S) & 1) ^ 1);

  /* set the ACTIVE source based upon CCFG:MODE_CONF:DCDC_ACTIVE Note: Inverse
   * polarity */

  HWREGBITW(AON_PMCTL_BASE + AON_PMCTL_O_PWRCTL,
            AON_PMCTL_PWRCTL_DCDC_ACTIVE_BITN) =
    (((ccfg_modeconf >> CCFG_MODE_CONF_DCDC_ACTIVE_S) & 1) ^ 1);
}
