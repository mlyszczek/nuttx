/************************************************************************************
 * arch/arm/src/imxrt/imxrt_lpsrtc.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/imxrt_snvs.h"
#include "imxrt_hprtc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_HPRTC
struct rtc_state_s
{
#warning Missing logic
};
#endif /* CONFIG_IMXRT_SNVS_HPRTC */

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_HPRTC
/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_rtc_alarmcb;
#endif
#endif /* CONFIG_IMXRT_SNVS_HPRTC */

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_HPRTC

/************************************************************************************
 * Name: imxrt_rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int imxrt_rtc_interrupt(int irq, void *context, FAR void *arg)
{
#warning Missing logic
  return 0;
}
#endif

#endif /* CONFIG_IMXRT_SNVS_HPRTC */

/************************************************************************************
 * Name: imxrt_hprtc_alarmenable
 *
 * Description:
 *    Enable alarm interrupts.  This is currently only used internally at the time
 *    that alarm interrupts are enabled.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
static void imxrt_hprtc_alarmenable(void)
{
  uint32_t regval;

  /* Enable the alarm */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval |= SNVS_HPCR_HPTAEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  /* Enable alarm interrupts at the NVIC */

  up_enable_irq(IMXRT_IRQ_SNVS);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Functions used only for HPRTC
 ************************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_HPRTC

/************************************************************************************
 * Name: imxrt_hprtc_irqinitialize
 *
 * Description:
 *   Initialize IRQs for RTC, not possible during up_rtc_initialize because
 *   up_irqinitialize is called later.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int imxrt_hprtc_irqinitialize(void)
{
#warning Missing logic
  return OK;
}

/************************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but neither
 *   CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ************************************************************************************/

time_t up_rtc_time(void)
{
  /* Delegate to imxrt_hprtc_time() */

  return imxrt_hprtc_time();
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
#warning Missing logic
  return OK;
}

#endif /* CONFIG_IMXRT_SNVS_HPRTC */

/************************************************************************************
 * Logic Common to LPSRTC and HPRTC
 ************************************************************************************/

/************************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_initialize(void)
{
  /* Perform HPRTC initialization */
#warning Missing logic

#ifdef CONFIG_IMXRT_SNVS_LPSRTC
  /* Perform LPSRTC initialization */

  return imxrt_lpsrtc_initialize();
#else
  return OK;
#endif
}

/************************************************************************************
 * Name: imxrt_hprtc_synchronize
 *
 * Description:
 *   Synchronize the HPRTC to the LPSRTC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_LPSRTC
void imxrt_hprtc_synchronize(void)
{
  uint32_t regval;
  uint32_t hpcr;

  /* Disable the RTC */

  hpcr    = getreg32(IMXRT_SNVS_HPCR);
  regval  = hpcr;
  regval &= ~SNVS_HPCR_RTCEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_RTCEN) != 0)
    {
    }

  /* Synchronize to the LPSRTC */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval |= SNVS_HPCR_HPTS;
  putreg32(regval, IMXRT_SNVS_HPCR);

  /* Re-enable the HPRTC if it was enabled on entry. */

  if ((hpcr & SNVS_HPCR_RTCEN) != 0)
    {
      imxrt_hprtc_enable();
    }
}
#endif

/************************************************************************************
 * Name: imxrt_hprtc_enable
 *
 * Description:
 *   Enable/start the HPRTC time counter.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void imxrt_hprtc_enable(void)
{
  uint32_t regval;

  /* Enable the HPRTC */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval |= SNVS_HPCR_RTCEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_RTCEN) == 0)
    {
    }
}

/************************************************************************************
 * Name: imxrt_hprtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is the underlying implementation of the
 *   up_rtc_time() function that is used by the RTOS during initialization to set up
 *   the system time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ************************************************************************************/

uint32_t imxrt_hprtc_time(void)
{
  uint32_t seconds;
  uint32_t verify = 0;

  /* Do consecutive reads until value is correct */

  do
    {
      /* IMXRT_SNVS_HPRTCMR: Bits 9-14 = 15-bit MSB of counter.
       * IMXRT_SNVS_HPRTCLR: 32-bit LSB of counter.
       *
       * REVISIT:  This could be modified to support CONFIG_RTC_HI_RES
       */

      seconds = verify;
      verify  = (getreg32(IMXRT_SNVS_HPRTCMR) << 17) |
                (getreg32(IMXRT_SNVS_HPRTCLR) >> 15);
    }
  while (tmp != seconds);

  return seconds;
}

/************************************************************************************
 * Name: imxrt_hprtc_getalarm
 *
 * Description:
 *   Get the current alarm setting in seconds.  This is only used by the lower half
 *   RTC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
uint32_t imxrt_hprtc_getalarm(void)
{
  /* Return the alarm setting in seconds
   *
   * IMXRT_SNVS_HPTAMR Bits 9-14 = 15-bit MSB of alarm setting.
   * IMXRT_SNVS_HPTALR 32-bit LSB of alarm setting.
   */

  return (getreg32(IMXRT_SNVS_HPTAMR) << 17) |
         (getreg32(IMXRT_SNVS_HPTALR) >> 15);
}
#endif

/************************************************************************************
 * Name: imxrt_hprtc_setalarm
 *
 * Description:
 *   Set the alarm (in seconds) and enable alarm interrupts.  This is only used by
 *   the lower half RTC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
int imxrt_hprtc_setalarm(uint32_t sec)
{
  irqstate_t flags;
  uint32_t regval;
  uint32_t now;

  /* Disable interrupts so that the following sequence of events will not be
   * interrupted or preempted.
   */

  flags = spin_lock_irqsave();

  now = imxrt_hprtc_time();

  /* Return error if the alarm time has passed.
   * NOTE:  This will fail, of course, when the number of seconds since epoch
   * wraps.
   */

  if (sec < now)
    {
      return -EINVAL;
    }

  /* Disable the RTC alarm interrupt */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval  = hcpr & ~SNVS_HPCR_HPTAEN;
  putreg32(regval, IMXRT_SNVS_HPCR);

  while ((getreg32(IMXRT_SNVS_HPCR) & SNVS_HPCR_HPTAEN) != 0)
    {
    }

  /* Set alarm in seconds
   *
   * IMXRT_SNVS_HPTAMR Bits 9-14 = 15-bit MSB of alarm setting.
   * IMXRT_SNVS_HPTALR 32-bit LSB of alarm setting.
   */

  putreg32(sec >> 17, IMXRT_SNVS_HPTAMR);
  putreg32(sec << 15, IMXRT_SNVS_HPTALR);

  /* Unconditionally enable the RTC alarm interrupt */

  imxrt_hprtc_alarmenable();
  spin_unlock_irqrestore(flags);
  return OK;
}
#endif

/************************************************************************************
 * Name: imxrt_hprtc_alarmdisable
 *
 * Description:
 *    Disable alarm interrupts.  Used internally after the receipt of the alarm
 *    interrupt.  Also called by the lower-half RTC driver in order to cancel an
 *    alarm.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ************************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
void imxrt_hprtc_alarmdisable(SNVS_Type *base, uint32_t mask)
{
  /* Disable alarm interrupts at the NVIC */

  up_disable_irq(IMXRT_IRQ_SNVS);

  /* Enable the alarm */

  regval  = getreg32(IMXRT_SNVS_HPCR);
  regval &= ~SNVS_HPCR_HPTAEN;
  putreg32(regval, IMXRT_SNVS_HPCR);
}
#endif

#endif /* CONFIG_IMXRT_SNVS_HPRTC */
