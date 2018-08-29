/****************************************************************************
 * sched/sched/sched_suspend.c
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

#include <sys/types.h>
#include <sched.h>
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_suspend
 *
 * Description:
 *   Suspend/pause the specified thread.  This is normally calling indirectly
 *   via group_suspendchildren();
 *
 ****************************************************************************/

void sched_suspend(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL);

  flags = enter_critical_section();

  /* Check the previous state of the task */

  if (tcb->prev_state >= FIRST_BLOCKED_STATE &&
      tcb->prev_state <= LAST_BLOCKED_STATE)
    {
      tstate_t prev_state;

      /* REVISIT:  What if the task is waiting for a resource, receives,
       * that received while, suspended.  How do we account for that
       * state change while suspended?  We will be putting the TCB back
       * into the wrong list!
       */
#warning Missing logic

      /* Move the TCB from the g_stoppedtasks list to the blocked task list. */

      sched_removeblocked(tcb);
      tcb->prev_state = tcb->prev_state;
      sched_addblocked(tcb, TSTATE_TASK_STOPPED)
    }
  else
    {
      /* The task was running or runnable before being stopped.  Simply
       * restart it.  If pre-emption is not disabled this action could
       * block this task here!
       */

      tcb->prev_state = tcb->task_state;
      prev_state      = TSTATE_TASK_STOPPED;
      up_block_task(tcb);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sched_resume_task
 *
 * Description:
 *   Resume a suspended task
 *
 ****************************************************************************/

void sched_resume_task(FAR struct tcb_s *tcb)
{
??? TODO ???
#ifdef HAVE_GROUP_MEMBERS
  /* Stop all of the children of the task */

??? TODO ???
#endif
}
