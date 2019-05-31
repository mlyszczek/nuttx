/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_sigdeliver.c
 *
 *   
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "sched/sched.h"
#include "up_internal.h"
#include "up_arch.h"


#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void up_sigdeliver(void)
{
#ifndef CONFIG_DISABLE_SIGNALS
  struct tcb_s *rtcb = this_task();
  uint8_t regs[22];
  sig_deliver_t sigdeliver;

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = rtcb->pterrno;

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the real return state on the stack. */

  up_copystate(regs, rtcb->xcp.regs);
  regs[REG_PC]   = rtcb->xcp.saved_pc;
  

  /* Get a local copy of the sigdeliver function pointer.  We do this so
   * that we can nullify the sigdeliver function pointer in the TCB and
   * accept more signal deliveries while processing the current pending
   * signals.
   */

  sigdeliver           = rtcb->xcp.sigdeliver;
  rtcb->xcp.sigdeliver = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signals */

  sigdeliver(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   */

  sinfo("Resuming\n");
  (void)up_irq_save();
  rtcb->pterrno = saved_errno;

  /* Then restore the correct state for this thread of
   * execution.
   */

  board_autoled_off(LED_SIGNAL);
  up_fullcontextrestore(regs);
#endif
}

#endif /* !CONFIG_DISABLE_SIGNALS */

