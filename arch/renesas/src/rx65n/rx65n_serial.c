/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_serial.c
 *
 *
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include "rx65n_macrodriver.h"
#include "iodefine.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "rx_65n.h"
#include "rx65n_sci.h"
#include "rx65n/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/


/* Is there a serial console? */

#if   defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI0)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI1)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI2)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI3)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI4)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI5)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI6)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI7)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && defined(CONFIG_RX65N_SCI8)
#  define HAVE_CONSOLE 1
#else
#  undef HAVE_CONSOLE
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#  undef CONFIG_SCI2_SERIAL_CONSOLE
#  undef CONFIG_SCI3_SERIAL_CONSOLE
#  undef CONFIG_SCI4_SERIAL_CONSOLE
#  undef CONFIG_SCI5_SERIAL_CONSOLE
#  undef CONFIG_SCI6_SERIAL_CONSOLE
#  undef CONFIG_SCI7_SERIAL_CONSOLE
#  undef CONFIG_SCI8_SERIAL_CONSOLE
#endif


#ifdef USE_SERIALDRIVER

#if 0

/* Which SCI with be tty0/console and which tty1? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci0port		/* SCI0 is console */
#  define	TTYS0_DEV		g_sci0port		/* SCI0 is tty0 */
#  ifdef CONFIG_RX65_SCI1
#    define	TTYS1_DEV		g_sci1port		/* SCI1 is tty1 */
#  else
#    undef	TTYS1_DEV
#  endif
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci1port		/* SCI1 is console */
#  define	TTYS0_DEV		g_sci1port		/* SCI1 is tty0 */
#  ifdef CONFIG_RX65N_SCI0
#    define	TTYS1_DEV		g_sci0port		/* SCI0 is tty1 */
#  else
#    undef	TTYS1_DEV						/* No tty1 */
#  endif
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci2port		/* SCI1 is console */
#  define	TTYS0_DEV		g_sci2port		/* SCI1 is tty0 */
#  ifdef CONFIG_RX65N_SCI0
#    define	TTYS2_DEV		g_sci0port		/* SCI0 is tty1 */
#  else
#    undef	TTYS2_DEV						/* No tty2 */
#  endif
#else
#  undef	CONSOLE_DEV						/* No console */
#  define	TTYS0_DEV		g_sci1port		/* SCI1 is tty0 */
#  undef	TTYS1_DEV						/* No tty1 */
#  undef	TTYS2_DEV						/* No tty2 */
#endif

#else

/* Which SCI with be tty0/console and which tty1? */

#ifdef CONFIG_RX65N_SCI0
#  define TTYS0_DEV     g_sci0port       /* SCI0 is tty0 */
#else
#  undef  TTYS0_DEV                      /* No tty0 */
#endif
#ifdef CONFIG_RX65N_SCI1
#  define TTYS1_DEV     g_sci1port       /* SCI1 is tty1 */
#else
#  undef  TTYS1_DEV                      /* No tty1 */
#endif
#ifdef CONFIG_RX65N_SCI2
#  define TTYS2_DEV     g_sci2port       /* SCI2 is tty2 */
#else
#  undef  TTYS2_DEV                      /* No tty2 */
#endif
#ifdef CONFIG_RX65N_SCI3
#  define TTYS3_DEV     g_sci3port       /* SCI3 is tty3 */
#else
#  undef  TTYS3_DEV                      /* No tty3 */
#endif
#ifdef CONFIG_RX65N_SCI4
#  define TTYS4_DEV     g_sci4port       /* SCI4 is tty4 */
#else
#  undef  TTYS4_DEV                      /* No tty4 */
#endif
#ifdef CONFIG_RX65N_SCI5
#  define TTYS5_DEV     g_sci5port       /* SCI5 is tty5 */
#else
#  undef  TTYS5_DEV                      /* No tty5 */
#endif
#ifdef CONFIG_RX65N_SCI6
#  define TTYS6_DEV     g_sci6port       /* SCI6 is tty6 */
#else
#  undef  TTYS6_DEV                      /* No tty6 */
#endif
#ifdef CONFIG_RX65N_SCI7
#  define TTYS7_DEV     g_sci7port       /* SCI7 is tty7 */
#else
#  undef  TTYS7_DEV                      /* No tty7 */
#endif
#ifdef CONFIG_RX65N_SCI8
#  define TTYS8_DEV     g_sci8port       /* SCI8 is tty8 */
#else
#  undef  TTYS8_DEV                      /* No tty8 */
#endif

#if   defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci0port		/* SCI0 is console */
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci1port		/* SCI1 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS1_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci2port		/* SCI2 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS2_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci3port		/* SCI3 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS3_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci4port		/* SCI4 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS4_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci5port		/* SCI5 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS5_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci6port		/* SCI6 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS6_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci7port		/* SCI7 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS7_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE)
#  define	CONSOLE_DEV		g_sci8port		/* SCI8 is console */
#  ifdef	TTYS0_DEV
#    define	TTYS8_DEV		TTYS0_DEV
#  endif // TTYS0_DEV
#  define	TTYS0_DEV		CONSOLE_DEV
#else
#  undef	CONSOLE_DEV						/* No console */
#endif

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
		  uint32_t scibase;   /* Base address of SCI registers */
		  uint32_t baud;      /* Configured baud */
 volatile  uint8_t scr;       /* Saved SCR value */
 volatile  uint8_t ssr;       /* Saved SR value (only used during interrupt processing) */
		   uint8_t irq;       /* Base IRQ associated with this SCI */
		   uint8_t parity;    /* 0=none, 1=odd, 2=even */
		   uint8_t bits;      /* Number of bits (7 or 8) */
			  bool stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, FAR void *arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I/O buffers */

#ifdef CONFIG_RX65N_SCI0
static char g_sci0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_sci0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI1
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI2
static char g_sci2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_sci2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI3
static char g_sci3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_sci3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI4
static char g_sci4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_sci4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI5
static char g_sci5rxbuffer[CONFIG_SCI5_RXBUFSIZE];
static char g_sci5txbuffer[CONFIG_SCI5_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI6
static char g_sci6rxbuffer[CONFIG_SCI6_RXBUFSIZE];
static char g_sci6txbuffer[CONFIG_SCI6_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI7
static char g_sci7rxbuffer[CONFIG_SCI7_RXBUFSIZE];
static char g_sci7txbuffer[CONFIG_SCI7_TXBUFSIZE];
#endif
#ifdef CONFIG_RX65N_SCI8
static char g_sci8rxbuffer[CONFIG_SCI8_RXBUFSIZE];
static char g_sci8txbuffer[CONFIG_SCI8_TXBUFSIZE];
#endif


struct uart_ops_s g_sci_ops =
{
	.setup			= up_setup,
	.shutdown		= up_shutdown,
	.attach			= up_attach,
	.detach			= up_detach,
	.receive		= up_receive,
	.rxint			= up_rxint,
	.rxavailable	= up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
	.rxflowcontrol	= NULL,
#endif
	.send			= up_send,
	.txint			= up_txint,
	.txready		= up_txready,
	.txempty		= up_txready,
};

#ifdef CONFIG_RX65N_SCI0
static struct up_dev_s g_sci0priv =
{
	.scibase		= RX65N_SCI0_BASE,
	.baud			= CONFIG_SCI0_BAUD,
	.irq			= RX65N_SCI0_IRQBASE,
	.parity			= CONFIG_SCI0_PARITY,
	.bits			= CONFIG_SCI0_BITS,
	.stopbits2		= CONFIG_SCI0_2STOP,
};

static uart_dev_t g_sci0port =
{
	.recv			= {
		.size   = CONFIG_SCI0_RXBUFSIZE,
		.buffer = g_sci0rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI0_TXBUFSIZE,
		.buffer	= g_sci0txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci0priv,
};
#endif

#ifdef CONFIG_RX65N_SCI1
static struct up_dev_s g_sci1priv =
{
	.scibase		= RX65N_SCI1_BASE,
	.baud			= CONFIG_SCI1_BAUD,
	.irq			= RX65N_SCI1_IRQBASE,
	.parity			= CONFIG_SCI1_PARITY,
	.bits			= CONFIG_SCI1_BITS,
	.stopbits2		= CONFIG_SCI1_2STOP,
};

static uart_dev_t g_sci1port =
{
	.recv			= {
		.size   = CONFIG_SCI1_RXBUFSIZE,
		.buffer = g_sci1rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI1_TXBUFSIZE,
		.buffer	= g_sci1txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci1priv,
};
#endif

#ifdef CONFIG_RX65N_SCI2
static struct up_dev_s g_sci2priv =
{
	.scibase		= RX65N_SCI2_BASE,
	.baud			= CONFIG_SCI2_BAUD,
	.irq			= RX65N_SCI2_IRQBASE,
	.parity			= CONFIG_SCI2_PARITY,
	.bits			= CONFIG_SCI2_BITS,
	.stopbits2		= CONFIG_SCI2_2STOP,
};

static uart_dev_t g_sci2port =
{
	.recv			= {
		.size   = CONFIG_SCI2_RXBUFSIZE,
		.buffer = g_sci2rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI2_TXBUFSIZE,
		.buffer	= g_sci2txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci2priv,
};
#endif

#ifdef CONFIG_RX65N_SCI3
static struct up_dev_s g_sci3priv =
{
	.scibase		= RX65N_SCI3_BASE,
	.baud			= CONFIG_SCI3_BAUD,
	.irq			= RX65N_SCI3_IRQBASE,
	.parity			= CONFIG_SCI3_PARITY,
	.bits			= CONFIG_SCI3_BITS,
	.stopbits2		= CONFIG_SCI3_2STOP,
};

static uart_dev_t g_sci3port =
{
	.recv			= {
		.size   = CONFIG_SCI3_RXBUFSIZE,
		.buffer = g_sci3rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI3_TXBUFSIZE,
		.buffer	= g_sci3txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci3priv,
};
#endif

#ifdef CONFIG_RX65N_SCI4
static struct up_dev_s g_sci4priv =
{
	.scibase		= RX65N_SCI4_BASE,
	.baud			= CONFIG_SCI4_BAUD,
	.irq			= RX65N_SCI4_IRQBASE,
	.parity			= CONFIG_SCI4_PARITY,
	.bits			= CONFIG_SCI4_BITS,
	.stopbits2		= CONFIG_SCI4_2STOP,
};

static uart_dev_t g_sci4port =
{
	.recv			= {
		.size   = CONFIG_SCI4_RXBUFSIZE,
		.buffer = g_sci4rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI4_TXBUFSIZE,
		.buffer	= g_sci4txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci4priv,
};
#endif

#ifdef CONFIG_RX65N_SCI5
static struct up_dev_s g_sci5priv =
{
	.scibase		= RX65N_SCI5_BASE,
	.baud			= CONFIG_SCI5_BAUD,
	.irq			= RX65N_SCI5_IRQBASE,
	.parity			= CONFIG_SCI5_PARITY,
	.bits			= CONFIG_SCI5_BITS,
	.stopbits2		= CONFIG_SCI5_2STOP,
};

static uart_dev_t g_sci5port =
{
	.recv			= {
		.size   = CONFIG_SCI5_RXBUFSIZE,
		.buffer = g_sci5rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI5_TXBUFSIZE,
		.buffer	= g_sci5txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci5priv,
};
#endif

#ifdef CONFIG_RX65N_SCI6
static struct up_dev_s g_sci6priv =
{
	.scibase		= RX65N_SCI6_BASE,
	.baud			= CONFIG_SCI6_BAUD,
	.irq			= RX65N_SCI6_IRQBASE,
	.parity			= CONFIG_SCI6_PARITY,
	.bits			= CONFIG_SCI6_BITS,
	.stopbits2		= CONFIG_SCI6_2STOP,
};

static uart_dev_t g_sci6port =
{
	.recv			= {
		.size   = CONFIG_SCI6_RXBUFSIZE,
		.buffer = g_sci6rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI6_TXBUFSIZE,
		.buffer	= g_sci6txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci6priv,
};
#endif

#ifdef CONFIG_RX65N_SCI7
static struct up_dev_s g_sci7priv =
{
	.scibase		= RX65N_SCI7_BASE,
	.baud			= CONFIG_SCI7_BAUD,
	.irq			= RX65N_SCI7_IRQBASE,
	.parity			= CONFIG_SCI7_PARITY,
	.bits			= CONFIG_SCI7_BITS,
	.stopbits2		= CONFIG_SCI7_2STOP,
};

static uart_dev_t g_sci7port =
{
	.recv			= {
		.size   = CONFIG_SCI7_RXBUFSIZE,
		.buffer = g_sci7rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI7_TXBUFSIZE,
		.buffer	= g_sci7txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci7priv,
};
#endif

#ifdef CONFIG_RX65N_SCI8
static struct up_dev_s g_sci8priv =
{
	.scibase		= RX65N_SCI8_BASE,
	.baud			= CONFIG_SCI8_BAUD,
	.irq			= RX65N_SCI8_IRQBASE,
	.parity			= CONFIG_SCI8_PARITY,
	.bits			= CONFIG_SCI8_BITS,
	.stopbits2		= CONFIG_SCI8_2STOP,
};

static uart_dev_t g_sci8port =
{
	.recv			= {
		.size   = CONFIG_SCI8_RXBUFSIZE,
		.buffer = g_sci8rxbuffer,
	},
	.xmit			= {
		.size	= CONFIG_SCI8_TXBUFSIZE,
		.buffer	= g_sci8txbuffer,
	},
	.ops		= &g_sci_ops,
	.priv		= &g_sci8priv,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/
static inline
uint8_t		up_serialin(struct up_dev_s *priv, int offset)
{
	return getreg8(priv->scibase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/
static inline
void		up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
	putreg8(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_disablesciint
 ****************************************************************************/
static inline
void		up_disablesciint(struct up_dev_s *priv, uint8_t *scr)
{
	if(scr) {
		*scr = priv->scr;
	}
	/* The disable all interrupts */
	priv->scr &= ~RX_SCISCR_ALLINTS;
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_restoresciint
 ****************************************************************************/
static inline
void		up_restoresciint(struct up_dev_s *priv, uint8_t scr)
{
	/* Set the interrupt bits in the scr value */
	priv->scr  &= ~RX_SCISCR_ALLINTS;
	priv->scr |= (scr & RX_SCISCR_ALLINTS);
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/
#ifdef HAVE_CONSOLE
static inline
void		up_waittxready(struct up_dev_s *priv)
{
	int		tmp;

	/* Limit how long we will wait for the TDR empty condition */
	for(tmp = 1000 ; tmp > 0 ; tmp--) {
		/* Check if the TDR is empty.  The TDR becomes empty when:  (1) the
		 * the chip is reset or enters standby mode, (2) the TE bit in the SCR
		 * is cleared, or (3) the current TDR contents are loaded in the TSR so
		 * that new data can be written in the TDR.
		 */
		if(0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE)) {
			/* The TDR is empty... return */
			break;
		}
	}
}
#endif

/****************************************************************************
 * Name: up_setbrr
 *
 * Description:
 *   Calculate the correct value for the BRR given the configured frequency
 *   and the desired BAUD settings.
 *
 ****************************************************************************/
static inline
void	up_setbrr(struct up_dev_s *priv, unsigned int baud)
{
	uint16_t	brr;
	uint16_t	brrdiv;
	uint8_t		semr;

	semr = up_serialin(priv, RX_SCI_SEMR_OFFSET);
	brrdiv = 32U;
	if(0 != (0x10 & semr)) {
		brrdiv /= 2;
	}
	if(0 != (0x40 & semr)) {
		brrdiv /= 2;
	}
	brr = (uint32_t)(RX_PCLKB / brrdiv / baud) - 1;
	up_serialout(priv, RX_SCI_BRR_OFFSET, brr);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/
static
int		up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
	struct up_dev_s		*priv;
	uint8_t		smr;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable the transmitter and receiver */
	priv->scr  = up_serialin(priv, RX_SCI_SCR_OFFSET);
	priv->scr &= ~(RX_SCISCR_TE | RX_SCISCR_RE);
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	/* Set communication to be asynchronous with the configured number of data
	 * bits, parity, and stop bits.  Use the internal clock (undivided)
	 */
	smr = 0;
	if(7 == priv->bits) {
		smr |= RX_SCISMR_CHR;
	}
	if(1 == priv->parity) {
		smr |= (RX_SCISMR_PE|RX_SCISMR_OE);
	}
	else if(2 == priv->parity) {
		smr |= RX_SCISMR_PE;
	}
	if(priv->stopbits2) {
		smr |= RX_SCISMR_STOP;
	}
	up_serialout(priv, RX_SCI_SMR_OFFSET, smr);
	/* Set the baud based on the configured console baud and configured
	 * system clock.
	 */
	up_setbrr(priv, priv->baud);
	
	/* Select the internal clock source as input */
	priv->scr &= ~RX_SCISCR_CKEMASK;
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	/* Wait a bit for the clocking to settle */
	up_udelay(100);
	/* Then enable the transmitter and reciever */
	priv->scr |= (RX_SCISCR_TE | RX_SCISCR_RE);
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
#endif
	return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial port is closed
 *
 ****************************************************************************/
static
void		up_shutdown(struct uart_dev_s *dev)
{
	struct up_dev_s		*priv;

	priv = (struct up_dev_s*)dev->priv;
	up_disablesciint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/
static
int		up_attach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv;
	int		ret;

	priv = (struct up_dev_s*)dev->priv;
	/* Attach the RDR full IRQ (RXI) that is enabled by the RIE SCR bit */
	ret = irq_attach(priv->irq + RX_RXI_IRQ_OFFSET, up_interrupt, dev);
	if(OK != ret) {
		return ret;
	}
	/* The RIE interrupt enable also enables the receive error interrupt (ERI) */
	ret = irq_attach(priv->irq + RX_ERI_IRQ_OFFSET, up_interrupt, dev);
	if(OK != ret) {
		/* Detach the RXI interrupt on failure */
		(void)irq_detach(priv->irq + RX_RXI_IRQ_OFFSET);
		return ret;
	}
	/* Attach the TDR empty IRQ (TXI) enabled by the TIE SCR bit */
	ret = irq_attach(priv->irq + RX_TXI_IRQ_OFFSET, up_interrupt, dev);
	if(OK == ret) {
#ifdef CONFIG_ARCH_IRQPRIO
		/* All SCI0 interrupts share the same prioritization */
		up_prioritize_irq(priv->irq, 7);  /* Set SCI priority midway */
#endif
		/* Return OK on success */
		return OK;
	}
	/* Detach the ERI interrupt on failure */
	(void)irq_detach(priv->irq + RX_ERI_IRQ_OFFSET);
	/* Detach the RXI interrupt on failure */
	(void)irq_detach(priv->irq + RX_RXI_IRQ_OFFSET);
	return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/
static
void		up_detach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable all SCI interrupts */
	up_disablesciint(priv, NULL);
	/* Detach the SCI interrupts */
	(void)irq_detach(priv->irq + RX_RXI_IRQ_OFFSET);
	(void)irq_detach(priv->irq + RX_ERI_IRQ_OFFSET);
	(void)irq_detach(priv->irq + RX_RXI_IRQ_OFFSET);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the SCI interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   up_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static
int		up_interrupt(int irq, void *context, FAR void *arg)
{
	struct uart_dev_s	*dev;
	struct up_dev_s		*priv;

	dev = (struct uart_dev_s *)arg;
	DEBUGASSERT((NULL != dev) && (NULL != priv));
	priv = (struct up_dev_s*)dev->priv;
	/* Get the current SCI status  */
	priv->ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
	/* Handle receive-related events with RIE is enabled.  RIE is enabled at
	 * times that driver is open EXCEPT when the driver is actively copying
	 * data from the circular buffer.  In that case, the read events must
	 * pend until RIE is set
	 */
	if(0 != (priv->scr & RX_SCISCR_RIE)) {
		/* Handle incoming, receive bytes (RDRF: Receive Data Register Full) */
		/* Setting RDRF bit when data is entered as input */
		if ((priv->ssr & RX_SCISSR_RDRF) != 0) {
			/* Rx data register not empty ... process incoming bytes */
			uart_recvchars(dev);
		}
		/* Clear all read related events (probably already done in up_receive)) */
		priv->ssr &= ~(RX_SCISSR_RDRF|RX_SCISSR_ORER|RX_SCISSR_FER|RX_SCISSR_PER);
	}
	/* Handle outgoing, transmit bytes (TDRE: Transmit Data Register Empty)
	 * when TIE is enabled.  TIE is only enabled when the driver is waiting with
	 * buffered data.  Since TDRE is usually true,
	 */
	
	if((0 != (priv->ssr & RX_SCISSR_TDRE)) && (0 != (priv->scr & RX_SCISCR_TIE))) {
		/* Tx data register empty ... process outgoing bytes */
		uart_xmitchars(dev);
		/* Clear the TDR empty flag (Possibly done in up_send, will have not
		 * effect if the TDR is still empty)
		 */
		priv->ssr &= ~RX_SCISSR_TDRE;
	}
	/* Clear all (clear-able) status flags.  Note that that RX65-1 requires
	 * that you read the bit in the "1" then write "0" to the bit in order
	 * to clear it.  Any bits in the SSR that transitioned from 0->1 after
	 * we read the SR will not be effected by the following:
	 */
	up_serialout(priv, RX_SCI_SSR_OFFSET, priv->ssr);
	return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/
static
int		up_receive(struct uart_dev_s *dev, unsigned int *status)
{
	struct up_dev_s *priv;
	uint8_t rdr;
	uint8_t ssr;

	priv = (struct up_dev_s*)dev->priv;
	/* Read the character from the RDR port */
	rdr  = up_serialin(priv, RX_SCI_RDR_OFFSET);
	/* Clear all read related status in  real ssr (so that when when rxavailable
	 * is called again, it will return false.
	 */
	ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
	ssr &= ~(RX_SCISSR_RDRF|RX_SCISSR_ORER|RX_SCISSR_FER|RX_SCISSR_PER);
	up_serialout(priv, RX_SCI_SSR_OFFSET, ssr);
	/* For status, return the SSR at the time that the interrupt was received */
	*status = (uint32_t)priv->ssr << 8 | rdr;
	/* Return the received character */
	return (int)rdr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/
static
void		up_rxint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s	*priv;
	irqstate_t flags;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable interrupts to prevent asynchronous accesses */
	flags = enter_critical_section();
	/* Are we enabling or disabling? */
	if(enable) {
		/* Enable the RDR full interrupt */
		priv->scr |= RX_SCISCR_RIE;
	}
	else {
		/* Disable the RDR full interrupt */
		priv->scr &= ~RX_SCISCR_RIE;
	}
	/* Write the modified SCR value to hardware */
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the RDR is not empty
 *
 ****************************************************************************/
static
bool		up_rxavailable(struct uart_dev_s *dev)
{
	struct up_dev_s *priv;

	/* Return true if the RDR full bit is set in the SSR */
	priv = (struct up_dev_s*)dev->priv;
	return (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_RDRF));
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/
static
void		up_send(struct uart_dev_s *dev, int ch)
{
	struct up_dev_s		*priv;
	uint8_t		ssr;

	priv = (struct up_dev_s*)dev->priv;
	/* Write the data to the TDR */
	up_serialout(priv, RX_SCI_TDR_OFFSET, (uint8_t)ch);
	/* Clear the TDRE bit in the SSR */
	ssr  = up_serialin(priv, RX_SCI_SSR_OFFSET);
	ssr &= ~RX_SCISSR_TDRE;
	up_serialout(priv, RX_SCI_SSR_OFFSET, ssr);

}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/
static
void		up_txint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s *priv;
	irqstate_t	flags;
	int		i;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable interrupts to prevent asynchronous accesses */
	flags = enter_critical_section();
	/* Are we enabling or disabling? */
	if(enable) {
		/* Enable the TDR empty interrupt */
		priv->scr |= RX_SCISCR_TIE;
		/* If the TDR is already empty, then don't wait for the interrupt */
#if 1
		if (up_txready(dev)) {
			/* Tx data register empty ... process outgoing bytes.  Note:
			 * this could call up_txint to be called recursively.  However,
			 * in this event, priv->scr should hold the correct value upon
			 * return from uuart_xmitchars().
			 */
			uart_xmitchars(dev);
		}
#endif
	}
	else {
		/* Disable the TDR empty interrupt */
		priv->scr &= ~RX_SCISCR_TIE;
	}
	/* Write the modified SCR value to hardware */
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the TDR is empty
 *
 ****************************************************************************/
static
bool	up_txready(struct uart_dev_s *dev)
{
	struct up_dev_s *priv;

	priv = (struct up_dev_s*)dev->priv;
	return (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level SCI initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_consoleinit.
 *
 ****************************************************************************/
void	up_earlyconsoleinit(void)
{
	/* NOTE:  All GPIO configuration for the SCIs was performed in
	 * up_lowsetup
	 */
	/* Disable all SCIs */
#ifdef TTYS0_DEV
	up_disablesciint(TTYS0_DEV.priv, NULL);
#endif
#ifdef TTYS1_DEV
	up_disablesciint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
	up_disablesciint(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
	up_disablesciint(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
	up_disablesciint(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
	up_disablesciint(TTYS5_DEV.priv, NULL);
#endif
#ifdef TTYS6_DEV
	up_disablesciint(TTYS6_DEV.priv, NULL);
#endif
#ifdef TTYS7_DEV
	up_disablesciint(TTYS7_DEV.priv, NULL);
#endif
#ifdef TTYS8_DEV
	up_disablesciint(TTYS8_DEV.priv, NULL);
#endif
	/* Configuration whichever one is the console */
#ifdef HAVE_CONSOLE
	CONSOLE_DEV.isconsole = true;
	up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyconsoleinit was called previously.
 *
 ****************************************************************************/
void	up_serialinit(void)
{
	/* Register all SCIs */
#ifdef TTYS0_DEV
	(void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
	(void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
	(void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
	(void)uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
	(void)uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
	(void)uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
#ifdef TTYS6_DEV
	(void)uart_register("/dev/ttyS6", &TTYS6_DEV);
#endif
#ifdef TTYS7_DEV
	(void)uart_register("/dev/ttyS7", &TTYS7_DEV);
#endif
#ifdef TTYS8_DEV
	(void)uart_register("/dev/ttyS8", &TTYS8_DEV);
#endif
	/* Register the console */
#ifdef HAVE_CONSOLE
	(void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/
int		up_putc(int ch)
{
#ifdef HAVE_CONSOLE
	struct up_dev_s *priv;
	uint8_t  scr;

	priv = (struct up_dev_s*)CONSOLE_DEV.priv;
	up_disablesciint(priv, &scr);
	/* Check for LF */
	if(ch == '\n') {
		/* Add CR */
		up_waittxready(priv);
		up_serialout(priv, RX_SCI_TDR_OFFSET, '\r');
	}
	up_waittxready(priv);
	up_serialout(priv, RX_SCI_TDR_OFFSET, (uint8_t)ch);
	up_waittxready(priv);
	up_restoresciint(priv, scr);
#endif
	return ch;
}
#else /* USE_SERIALDRIVER */
/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/
int		up_putc(int ch)
{
#ifdef HAVE_CONSOLE
	/* Check for LF */
	if(ch == '\n') {
		/* Add CR */
		up_lowputc('\r');
	}
	up_lowputc(ch);
#endif
	return ch;
}

#endif /* USE_SERIALDRIVER */
