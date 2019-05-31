/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_irq.c
 *
 *   
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

volatile uint32_t *g_current_regs; /* Actually a pointer to the beginning of a uint8_t array */

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_disable_irq(int irq)
{
  /* There are no ez80 interrupt controller settings to disable IRQs */
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* There are no ez80 interrupt controller settings to enable IRQs */
}

#endif /* CONFIG_ARCH_NOINTC */
