/************************************************************************************
 * arch/renesas/src/rx65n/rx_65n.h
 *
 *
 *
 ************************************************************************************/

#ifndef __ARCH_RENESAS_SRC_RX_65N_H
#define __ARCH_RENESAS_SRC_RX_65N_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "iodefine.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory-mapped register addresses *************************************************/

/* Serial Communications interface (SCI) */

enum E_RX_SCI {
	RX_SCI_SMR_OFFSET = 0,	/* Serial Mode Register (8-bits wide) */
	RX_SCI_BRR_OFFSET,		/* Bit Rate Register (8-bits wide) */
	RX_SCI_SCR_OFFSET,		/* Serial Control Register (8-bits wide) */
	RX_SCI_TDR_OFFSET,		/* Transmit Data Register (8-bits wide) */
	RX_SCI_SSR_OFFSET,		/* Serial Status Register (8-bits wide) */
	RX_SCI_RDR_OFFSET,		/* Receive Data Register (8-bits wide) */
	RX_SCI_SCMR_OFFSET,
	RX_SCI_SEMR_OFFSET,
	RX_SCI_SNFR_OFFSET,
	RX_SCI_SIMR1_OFFSET,
	RX_SCI_SIMR2_OFFSET,
	RX_SCI_SIMR3_OFFSET,
	RX_SCI_SISR_OFFSET,
	RX_SCI_SPMR_OFFSET,
	RX_SCI_THRHL_OFFSET,
	RX_SCI_RDRHL_OFFSET,
	RX_SCI_MDDR_OFFSET
};

#if 1
#define RX65N_SCI0_BASE       (uint32_t)&SCI0
#define RX65N_SCI1_BASE       (uint32_t)&SCI1
#define RX65N_SCI2_BASE       (uint32_t)&SCI2
#define RX65N_SCI3_BASE       (uint32_t)&SCI3
#define RX65N_SCI4_BASE       (uint32_t)&SCI4
#define RX65N_SCI5_BASE       (uint32_t)&SCI5
#define RX65N_SCI6_BASE       (uint32_t)&SCI6
#define RX65N_SCI7_BASE       (uint32_t)&SCI7
#define RX65N_SCI8_BASE       (uint32_t)&SCI8
#else
#define RX_SCI2_BASE       (0x0008A040)
#endif

#define RX_SCI2_SMR        (RX65N_SCI2_BASE+RX_SCI_SMR_OFFSET)
#define RX_SCI2_BRR        (RX65N_SCI2_BASE+RX_SCI_BRR_OFFSET)
#define RX_SCI2_SCR        (RX65N_SCI2_BASE+RX_SCI_SCR_OFFSET)
#define RX_SCI2_TDR        (RX65N_SCI2_BASE+RX_SCI_TDR_OFFSET)
#define RX_SCI2_SSR        (RX65N_SCI2_BASE+RX_SCI_SSR_OFFSET)
#define RX_SCI2_RDR        (RX65N_SCI2_BASE+RX_SCI_RDR_OFFSET)




/* Serial Communications interface (SCI) */

#define RX_SCISMR_CKSMASK  (0x03)        /* Bit 0-1: Internal clock source */
#define RX_SCISMR_DIV1     (0x00)        /*   System clock (phi) */
#define RX_SCISMR_DIV4     (0x01)        /*   phi/4 */
#define RX_SCISMR_DIV16    (0x02)        /*   phi/16 */
#define RX_SCISMR_DIV64    (0x03)        /*   phi/64 */
#define RX_SCISMR_MP       (0x04)        /* Bit 2: Multiprocessor select */
#define RX_SCISMR_STOP     (0x08)        /* Bit 3: 0:One stop bit, 1:Two stop bits */
#define RX_SCISMR_OE       (0x10)        /* Bit 4: 0:Even parity, 1:Odd parity */
#define RX_SCISMR_PE       (0x20)        /* Bit 5: Parity enable */
#define RX_SCISMR_CHR      (0x40)        /* Bit 6: 0:8-bit data, 1:7-bit data */
#define RX_SCISMR_CA       (0x80)        /* Bit 7: 0:Asynchronous, 1:clocked synchronous */

#define RX_SCISCR_CKEMASK  (0x03)        /* Bit 0-1: Internal clock source */
										  /* Asynchronous mode: */
#define RX_SCISCR_AISIN    (0x00)        /*   Internal clock, SCK pin used for input pin */
#define RX_SCISCR_AISOUT   (0x01)        /*   Internal clock, SCK pin used for clock output */
#define RX_SCISCR_AXSIN1   (0x02)        /*   External clock, SCK pin used for clock input */
#define RX_SCISCR_AXSIN2   (0x03)        /*   External clock, SCK pin used for clock input */
										  /* Synchronous mode: */
#define RX_SCISCR_SISOUT1  (0x00)        /*   Internal clock, SCK pin used for input pin */
#define RX_SCISCR_SISOUT2  (0x01)        /*   Internal clock, SCK pin used for clock output */
#define RX_SCISCR_SXSIN1   (0x02)        /*   External clock, SCK pin used for clock input */
#define RX_SCISCR_SXSIN2   (0x03)        /*   External clock, SCK pin used for clock input */
#define RX_SCISCR_TEIE     (0x04)        /* Bit 2: 1=Transmit end interrupt enable */
#define RX_SCISCR_MPIE     (0x08)        /* Bit 3: 1=Multiprocessor interrupt enable */
#define RX_SCISCR_RE       (0x10)        /* Bit 4: 1=Receiver enable */
#define RX_SCISCR_TE       (0x20)        /* Bit 5: 1=Transmitter enable */
#define RX_SCISCR_RIE      (0x40)        /* Bit 6: 1=Recieve-data-full interrupt enable */
#define RX_SCISCR_TIE      (0x80)        /* Bit 7: 1=Transmit-data-empty interrupt enable */
#define RX_SCISCR_ALLINTS  (0xcc)

#define RX_SCISSR_MPBT     (0x01)        /* Bit 0: Multi-processor Bit in Transmit data */
#define RX_SCISSR_MPB      (0x02)        /* Bit 1: Multi-processor Bit in receive data */
#define RX_SCISSR_TEND     (0x04)        /* Bit 2: End of transmission */
#define RX_SCISSR_PER      (0x08)        /* Bit 3: Receive parity error */
#define RX_SCISSR_FER      (0x10)        /* Bit 4: Receive framing error */
#define RX_SCISSR_ORER     (0x20)        /* Bit 5: Receive overrun error */
#define RX_SCISSR_RDRF     (0x40)        /* Bit 6: RDR contains valid received data */
#define RX_SCISSR_TDRE     (0x80)        /* Bit 7: TDR does not contain valid transmit data */


#endif /* __ARCH_RENESAS_SRC_RX_65N_H */













