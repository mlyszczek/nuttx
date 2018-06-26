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
#include "imxrt_lpsrtc.h"

#ifdef CONFIG_IMXRT_SNVS_LPSRTC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct lpsrtc_state_s
{
#warning Missing logic
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_lpsrtc_alarmcb;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: imxrt_lpsrtc_interrupt
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
static int imxrt_lpsrtc_interrupt(int irq, void *context, FAR void *arg)
{
#warning Missing logic
  return 0;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: mxrt_lpsrtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.  This function is called
 *   via up_rtc_initialize (see imxrt_hprtc.c).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int mxrt_lpsrtc_initialize(void)
{
#warning Missing logic
  return OK;
}

/************************************************************************************
 * Name: imxrt_lpsrtc_irqinitialize
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

int imxrt_lpsrtc_irqinitialize(void)
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

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  /* Delegate to imxrt_hprtc_time() */

  return imxrt_hprtc_time();
}
#endif

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

#endif /* CONFIG_IMXRT_SNVS_LPSRTC */
