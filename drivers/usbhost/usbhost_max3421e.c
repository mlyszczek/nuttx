/****************************************************************************
 * drivers/usbhost/max3421e_otgfshost.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/max3421e.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/irq.h>

#include "chip.h"             /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_USBHOST_MAX3421E

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ***************************************************************/
/* MAX3421E USB Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST - Enable general USB host support
 *  CONFIG_USBHOST_MAX3421E - Enable the MAX3421E USB host support
 *
 * Options:
 *
 *  CONFIG_MAX3421E_MAX3421E_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_MAX3421E_MAX3421E_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_MAX3421E_MAX3421E_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_MAX3421E_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *  CONFIG_MAX3421E_MAX3421E_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *  CONFIG_MAX3421E_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG_FEATURES.
 *  CONFIG_MAX3421E_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *    packets. Depends on CONFIG_DEBUG_FEATURES.
 */

/* Pre-requisites (partial) */

#ifndef CONFIG_MAX3421E_SYSCFG
#  error "CONFIG_MAX3421E_SYSCFG is required"
#endif

/* Default RxFIFO size */

#ifndef CONFIG_MAX3421E_MAX3421E_RXFIFO_SIZE
#  define CONFIG_MAX3421E_MAX3421E_RXFIFO_SIZE 128
#endif

/* Default host non-periodic Tx FIFO size */

#ifndef CONFIG_MAX3421E_MAX3421E_NPTXFIFO_SIZE
#  define CONFIG_MAX3421E_MAX3421E_NPTXFIFO_SIZE 96
#endif

/* Default host periodic Tx fifo size register */

#ifndef CONFIG_MAX3421E_MAX3421E_PTXFIFO_SIZE
#  define CONFIG_MAX3421E_MAX3421E_PTXFIFO_SIZE 96
#endif

/* Maximum size of a descriptor */

#ifndef CONFIG_MAX3421E_DESCSIZE
#  define CONFIG_MAX3421E_DESCSIZE 128
#endif

/* Low-priority work queue support is required.  The high priority work
 * queue is not used because this driver requires SPI access and may
 * block or wait for a variety of reasons.
 */

#ifndef CONFIG_SCHED_LPWORK
#  warning Low priority work thread support is necessary (CONFIG_SCHED_LPWORK)
#endif

/* Register/packet debug depends on CONFIG_DEBUG_FEATURES */

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_MAX3421E_USBHOST_REGDEBUG
#  undef CONFIG_MAX3421E_USBHOST_PKTDUMP
#endif

/* Delays **********************************************************************/

#define MAX3421E_READY_DELAY         200000 /* In loop counts */
#define MAX3421E_FLUSH_DELAY         200000 /* In loop counts */
#define MAX3421E_SETUP_DELAY         SEC2TICK(5) /* 5 seconds in system ticks */
#define MAX3421E_DATANAK_DELAY       SEC2TICK(5) /* 5 seconds in system ticks */

/* Ever-present MIN/MAX macros */

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define  MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum max3421e_smstate_e
{
  SMSTATE_DETACHED = 0,  /* Not attached to a device */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_ENUM,          /* Attached, enumerating */
  SMSTATE_CLASS_BOUND,   /* Enumeration complete, class bound */
};

/* This structure retains the state of one host channel.  NOTE: Since there
 * is only one channel operation active at a time, some of the fields in
 * in the structure could be moved in struct max3421e_ubhost_s to achieve
 * some memory savings.
 */

struct max3421e_chan_s
{
  sem_t             waitsem;   /* Channel wait semaphore */
  volatile uint8_t  result;    /* The result of the transfer */
  uint8_t           chidx;     /* Channel index (0-3) */
  uint8_t           epno;      /* Device endpoint number (0-127) */
  uint8_t           eptype;    /* See MAX3421E_EPTYPE_* definitions */
  uint8_t           funcaddr;  /* Device function address */
  uint8_t           speed;     /* Device speed */
  uint8_t           interval;  /* Interrupt/isochronous EP polling interval */
  uint8_t           pid;       /* Data PID */
  uint8_t           npackets;  /* Number of packets (for data toggle) */
  bool              inuse;     /* True: This channel is "in use" */
  volatile bool     indata1;   /* IN data toggle. True: DATA01 (Bulk and INTR only) */
  volatile bool     outdata1;  /* OUT data toggle.  True: DATA01 */
  bool              in;        /* True: IN endpoint */
  volatile bool     waiter;    /* True: Thread is waiting for a channel event */
  uint16_t          maxpacket; /* Max packet size */
  uint16_t          buflen;    /* Buffer length (at start of transfer) */
  volatile uint16_t xfrd;      /* Bytes transferred (at end of transfer) */
  volatile uint16_t inflight;  /* Number of Tx bytes "in-flight" */
  FAR uint8_t      *buffer;    /* Transfer buffer pointer */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t  callback;  /* Transfer complete callback */
  FAR void         *arg;       /* Argument that accompanies the callback */
#endif
};

/* This structure retains the state of the USB host controller */

struct max3421e_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct
   * usbhost_driver_s to struct max3421e_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the interface to the max3421e lower-half driver */

  FAR const struct max3421e_lowerhalf_s *lower;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  /* Overall driver status */

  volatile uint8_t  smstate;   /* The state of the USB host state machine */
  uint8_t           chidx;     /* ID of channel waiting for space in Tx FIFO */
  uint8_t           ackstat;   /* See MAX3421E_ACKSTAT_* definitions */
  uint8_t           irqset;    /* Set of enabled interrupts */
  volatile bool     connected; /* Connected to device */
  volatile bool     change;    /* Connection change */
  volatile bool     pscwait;   /* True: Thread is waiting for a port event */
  sem_t             exclsem;   /* Support mutually exclusive access */
  sem_t             pscsem;    /* Semaphore to wait for a port event */
  struct work_s     irqwork;   /* Used to process interrupts */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  /* The state of each host channel, one for each device endpoint. */

  struct max3421e_chan_s chan[MAX3421E_NHOST_CHANNELS];
};

/* This is the MAX3421E connection structure */

struct usbhost_connection_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct
   * usbhost_connection_s to struct usbhost_connection_s.
   */

  struct usbhost_connection_s conn;

  /* Pointer to the associated state structure */

  FAR struct max3421e_usbhost_s *priv;
};

/* Supports allocation of both structures simultaneously */

struct usbhost_alloc_s
{
  struct max3421e_usbhost_s   priv;
  struct usbhost_connection_s conn;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI/Register operations **************************************************/

static void max3421e_lock(FAR struct max3421e_usbhost_s *priv);
static void max3421e_unlock(FAR struct max3421e_usbhost_s *priv);

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
static void max3421e_printreg(uint8_t addr, uint8_t val, bool iswrite);
static void max3421e_checkreg(uint8_t addr, uint8_t val, bool iswrite)
#endif

static inline uint8_t max3421e_fmtcmd(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, uint8_t dir);
static uint32_t max3421e_getreg(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr);
static void max3421e_putreg(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, uint8_t value);

static inline void max3421e_modifyreg(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, uint8_t clrbits, uint8_t setbits);

static void max3421e_recvblock(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, FAR void *buffer, size_t buflen);
static void max3421e_sndblock(FAR struct max3421e_usbhost_s *priv,
              uint8_t addr, FAR const void *buffer, size_t buflen);

#ifdef CONFIG_MAX3421E_USBHOST_PKTDUMP
#  define max3421e_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#  define max3421e_pktdump(m,b,n)
#endif

/* Semaphores ******************************************************************/

static void max3421e_takesem(sem_t *sem);
#define max3421e_givesem(s) nxsem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t max3421e_getle16(const uint8_t *val);

/* Channel management **********************************************************/

static int max3421e_chan_alloc(FAR struct max3421e_usbhost_s *priv);
static inline void max3421e_chan_free(FAR struct max3421e_usbhost_s *priv,
              int chidx);
static int max3421e_chan_waitsetup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_chan_asynchsetup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan, usbhost_asynch_t callback,
              FAR void *arg);
#endif
static int max3421e_chan_wait(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static void max3421e_chan_wakeup(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);

/* Control/data transfer logic *************************************************/

static void max3421e_transfer_start(FAR struct max3421e_usbhost_s *priv,
              int chidx);
static int max3421e_ctrl_sendsetup(FAR struct max3421e_usbhost_s *priv,
              int chidx, FAR const struct usb_ctrlreq_s *req);
static int max3421e_ctrl_senddata(FAR struct max3421e_usbhost_s *priv,
              int chidx, FAR uint8_t *buffer, unsigned int buflen, in);
static int max3421e_ctrl_recvdata(FAR struct max3421e_usbhost_s *priv,
              int chidx, FAR uint8_t *buffer, unsigned int buflen);
static int max3421e_in_setup(FAR struct max3421e_usbhost_s *priv, int chidx);
static ssize_t max3421e_in_transfer(FAR struct max3421e_usbhost_s *priv, int chidx,
              FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_in_next(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int max3421e_in_asynch(FAR struct max3421e_usbhost_s *priv, int chidx,
              FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback,
              FAR void *arg);
#endif
static int max3421e_out_setup(FAR struct max3421e_usbhost_s *priv, int chidx);
static ssize_t max3421e_out_transfer(FAR struct max3421e_usbhost_s *priv, int chidx,
              FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_out_next(FAR struct max3421e_usbhost_s *priv,
              FAR struct max3421e_chan_s *chan);
static int max3421e_out_asynch(FAR struct max3421e_usbhost_s *priv, int chidx,
              FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback,
              FAR void *arg);
#endif

/* Interrupt handling **********************************************************/

static void max3421e_connect_event(FAR struct max3421e_usbhost_s *priv)
static void max3421e_disconnect_event(FAR struct max3421e_usbhost_s *priv)
static int  max3421e_connected(FAR struct max3421e_usbhost_s *priv);
static void max3421e_disconnected(FAR struct max3421e_usbhost_s *priv);
static int  max3421e_irqwork(FAR void *arg);
static int  max3421e_interrupt(int irq, FAR void *context, FAR void *arg);

/* Interrupt controls */

static inline void max3421e_int_enable(FAR struct max3421e_usbhost_s *priv,
              uint8_t irqset);
static inline void max3421e_int_disable(FAR struct max3421e_usbhost_s *priv,
              uint8_t irqset);
static inline uint8_t max3421e_int_status(FAR struct max3421e_usbhost_s *priv);

/* USB host controller operations **********************************************/

static int max3421e_wait(FAR struct usbhost_connection_s *conn,
              FAR struct usbhost_hubport_s **hport);
static int max3421e_getspeed(FAR struct max3421e_usbhost_s *priv,
              FAR struct usbhost_connection_s *conn,
              FAR struct usbhost_hubport_s *hport);
static int max3421e_enumerate(FAR struct usbhost_connection_s *conn,
              FAR struct usbhost_hubport_s *hport);

static int max3421e_ep0configure(FAR struct usbhost_driver_s *drvr,
              usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed,
              uint16_t maxpacketsize);
static int max3421e_epalloc(FAR struct usbhost_driver_s *drvr,
              FAR const FAR struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep);
static int max3421e_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int max3421e_alloc(FAR struct usbhost_driver_s *drvr,
              FAR uint8_t **buffer, FAR size_t *maxlen);
static int max3421e_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int max3421e_ioalloc(FAR struct usbhost_driver_s *drvr,
              FAR uint8_t **buffer, size_t buflen);
static int max3421e_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int max3421e_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
              FAR const struct usb_ctrlreq_s *req, FAR uint8_t *buffer);
static int max3421e_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
              FAR const struct usb_ctrlreq_s *req, FAR const uint8_t *buffer);
static ssize_t max3421e_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
              FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
              FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback,
              FAR void *arg);
#endif
static int max3421e_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int max3421e_connect(FAR struct max3421e_usbhost_s *priv,
              FAR struct usbhost_hubport_s *hport, bool connected);
#endif
static void max3421e_disconnect(FAR struct usbhost_driver_s *drvr,
              FAR struct usbhost_hubport_s *hport);

/* Initialization **************************************************************/

static void max3421e_busreset(FAR struct max3421e_usbhost_s *priv);
static int  max3421e_startsof(FAR struct max3421e_usbhost_s *priv);
static void max3421e_flush_txfifos(uint32_t txfnum);
static void max3421e_flush_rxfifo(void);
static void max3421e_vbusdrive(FAR struct max3421e_usbhost_s *priv, bool state);

static inline int max3421e_sw_initialize(FAR struct max3421e_usbhost_s *priv,
              FAR struct usbhost_connection_s conn *conn,
              FAR const struct max3421e_lowerhalf_s *lower);
static inline int max3421e_hw_initialize(FAR struct max3421e_usbhost_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max3421e_lock
 *
 * Description:
 *   Lock and configure the SPI bus.
 *
 ****************************************************************************/

static void max3421e_lock(FAR struct max3421e_usbhost_s *priv)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi = priv->spi;

  DEBUGASSERT(lower != NULL && lower->spi != NULL);
  spi = priv->spi;

  (void)SPI_LOCK(spi, true);
  SPI_SETMODE(spi, lower->mode);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, lower->frequency);
}

/****************************************************************************
 * Name: max3421e_unlock
 *
 * Description:
 *   Unlock the SPI bus.
 *
 ****************************************************************************/

static void max3421e_unlock(FAR struct max3421e_usbhost_s *priv)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;

  DEBUGASSERT(lower != NULL && lower->spi != NULL);
  (void)SPI_LOCK(lower->spi, false);
}
/****************************************************************************
 * Name: max3421e_printreg
 *
 * Description:
 *   Print the contents of an MAX3421Exx register operation
 *
 ****************************************************************************/

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
static void max3421e_printreg(uint8_t addr, uint8_t val, bool iswrite)
{
  uinfo("%02x%s%02x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: max3421e_checkreg
 *
 * Description:
 *   Get the contents of an MAX3421E register
 *
 ****************************************************************************/

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
static void max3421e_checkreg(uint8_t addr, uint8_t val, bool iswrite)
{
  static uint8_t prevaddr = 0;
  static uint8_t preval = 0;
  static unsigned int count = 0;
  static bool prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              max3421e_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      max3421e_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: max3421e_fmtcmd
 *
 * Description:
 *   Format a command
 *
 *   The command byte contains the register address, a direction bit, and an
 *   ACKSTAT bit:
 *
 *     Bits 3-7:  Command
 *     Bit 2:     Unused
 *     Bit 1:     Direction (read = 0, write = 1)
 *     Bit 0:     ACKSTAT
 *
 ****************************************************************************/

static inline uint8_t max3421e_fmtcmd(FAR struct max3421e_usbhost_s *priv,
                                      uint8_t addr, uint8_t dir)
{
  uint8_t cmd = addr | dir | priv->acstat;
  priv->ackstat = MAX3421E_ACKSTAT_FALSE;
  return cmd;
}

/****************************************************************************
 * Name: max3421e_getreg
 *
 * Description:
 *   Get the contents of an MAX3421E register
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static uint32_t max3421e_getreg(FAR struct max3421e_usbhost_s *priv,
                                uint8_t addr)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi = priv->spi;
  uint8_t cmd;
  uint8_t value;

  DEBUGASSERT(lower != NULL && lower->spi != NULL);
  spi = priv->spi;

  /* Select the MAX4321E */

  SPI_SELECT(spi, lower->devid, true);

  /* Send the read command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_READ);
  (void)SPI_SEND(spi, cmd);

  /* Read the value of the register */

  value = SPI_SEND(spi, 0xff);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, lower->devid, false);

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Check if we need to print this value */

  max3421e_checkreg(addr, value, false);
#endif

  return value;
}

/****************************************************************************
 * Name: max3421e_putreg
 *
 * Description:
 *   Set the contents of an MAX3421E register to a value
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static void max3421e_putreg(FAR struct max3421e_usbhost_s *priv,
                            uint8_t addr, uint8_t value)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi = priv->spi;
  uint8_t cmd;

  DEBUGASSERT(lower != NULL && lower->spi != NULL);
  spi = priv->spi;

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Check if we need to print this value */

  max3421e_checkreg(addr, val, true);
#endif

  /* Select the MAX4321E */

  SPI_SELECT(spi, lower->devid, true);

  /* Send the write command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_WRITE);
  (void)SPI_SEND(spi, cmd);

  /* Send the new value for the register */

  (void)SPI_SEND(spi, value);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, lower->devid, false);
}

/****************************************************************************
 * Name: max3421e_modifyreg
 *
 * Description:
 *   Modify selected bits of an MAX3421E register.
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static inline void max3421e_modifyreg(FAR struct max3421e_usbhost_s *priv,
                                      uint8_t addr, uint8_t clrbits,
                                      uint8_t setbits)
{
  uint8_t value;

  value = max3421e_getreg(priv, addr);
  value &= ~clrbits;
  value |= setbits;
  max3421e_putreg(priv, addr, value);
}

/****************************************************************************
 * Name: max3421e_recvblock
 *
 * Description:
 *   Receive a block of data from the MAX341E.
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static void max3421e_recvblock(FAR struct max3421e_usbhost_s *priv,
                               uint8_t addr, FAR void *buffer, size_t buflen)
{

  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi = priv->spi;
  uint8_t cmd;
  uint8_t value;

  DEBUGASSERT(lower != NULL && lower->spi != NULL);
  spi = priv->spi;

  /* Select the MAX4321E */

  SPI_SELECT(spi, lower->devid, true);

  /* Send the read command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_READ);
  (void)SPI_SEND(spi, cmd);

  /* Read the block of values from the register(s) */

  SPI_RECVBLOCK(spi, buffer, buflen);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, lower->devid, false);

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Dump the block of data received */

  lib_dumpbuffer("Received:", buffer, buflen);
#endif
}

/****************************************************************************
 * Name: max3421e_sndblock
 *
 * Description:
 *   Send a block of data to the MAX341E.
 *
 * Assumption:
 *   SPI bus is locked
 *
 ****************************************************************************/

static void max3421e_sndblock(FAR struct max3421e_usbhost_s *priv,
                              uint8_t addr, FAR const void *buffer,
                              size_t buflen)
{
  FAR const struct max3421e_lowerhalf_s *lower = priv->lower;
  FAR struct spi_dev_s *spi = priv->spi;
  uint8_t cmd;

  DEBUGASSERT(lower != NULL && lower->spi != NULL);
  spi = priv->spi;

#ifdef CONFIG_MAX3421E_USBHOST_REGDEBUG
  /* Dump the block of data to be sent */

  lib_dumpbuffer("Sending:", buffer, buflen);
#endif

  /* Select the MAX4321E */

  SPI_SELECT(spi, lower->devid, true);

  /* Send the wrte command byte */

  cmd = max3421e_fmtcmd(priv, addr, MAX3421E_DIR_WRITE);
  (void)SPI_SEND(spi, cmd);

  /* Send the new value for the register */

  SPI_SNDBLOCK(spi, buffer, buflen);

  /* De-select the MAX4321E */

  SPI_SELECT(spi, lower->devid, false);
}

/****************************************************************************
 * Name: max3421e_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void max3421e_takesem(sem_t *sem)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: max3421e_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t max3421e_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: max3421e_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 ****************************************************************************/

static int max3421e_chan_alloc(FAR struct max3421e_usbhost_s *priv)
{
  int chidx;

  /* Search the table of channels */

  for (chidx = 0; chidx < MAX3421E_NHOST_CHANNELS; chidx++)
    {
      /* Is this channel available? */

      if (!priv->chan[chidx].inuse)
        {
          /* Yes... make it "in use" and return the index */

          priv->chan[chidx].inuse = true;
          return chidx;
        }
    }

  /* All of the channels are "in-use" */

  return -EBUSY;
}

/****************************************************************************
 * Name: max3421e_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 ****************************************************************************/

static void max3421e_chan_free(FAR struct max3421e_usbhost_s *priv, int chidx)
{
  DEBUGASSERT((unsigned)chidx < MAX3421E_NHOST_CHANNELS);

  /* Halt the channel */

  max3421e_chan_halt(priv, chidx, CHREASON_FREED);

  /* Mark the channel available */

  priv->chan[chidx].inuse = false;
}

/****************************************************************************
 * Name: max3421e_chan_waitsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Called from a normal thread context BEFORE the transfer has been started.
 *
 ****************************************************************************/

static int max3421e_chan_waitsetup(FAR struct max3421e_usbhost_s *priv,
                                FAR struct max3421e_chan_s *chan)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
       */

      chan->waiter   = true;
#ifdef CONFIG_USBHOST_ASYNCH
      chan->callback = NULL;
      chan->arg      = NULL;
#endif
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: max3421e_chan_asynchsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Might be called from the level of an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_chan_asynchsetup(FAR struct max3421e_usbhost_s *priv,
                                  FAR struct max3421e_chan_s *chan,
                                  usbhost_asynch_t callback, FAR void *arg)
{
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set waiter to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
       */

      chan->waiter   = false;
      chan->callback = callback;
      chan->arg      = arg;
      ret            = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: max3421e_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int max3421e_chan_wait(FAR struct max3421e_usbhost_s *priv,
                           FAR struct max3421e_chan_s *chan)
{
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that the following operations will be atomic.  On
   * the host global interrupt needs to be disabled.  However, here we disable
   * all interrupts to exploit that fact that interrupts will be re-enabled
   * while we wait.
   */

  flags = enter_critical_section();

  /* Loop, testing for an end of transfer condition.  The channel 'result'
   * was set to EBUSY and 'waiter' was set to true before the transfer; 'waiter'
   * will be set to false and 'result' will be set appropriately when the
   * transfer is completed.
   */

  do
    {
      /* Wait for the transfer to complete.  NOTE the transfer may already
       * completed before we get here or the transfer may complete while we
       * wait here.
       */

      ret = nxsem_wait(&chan->waitsem);

      /* nxsem_wait should succeed.  But it is possible that we could be
       * awakened by a signal too.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (chan->waiter);

  /* The transfer is complete re-enable interrupts and return the result */

  ret = -(int)chan->result;
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: max3421e_chan_wakeup
 *
 * Description:
 *   A channel transfer has completed... wakeup any threads waiting for the
 *   transfer to complete.
 *
 * Assumptions:
 *   This function is called from the transfer complete interrupt handler for
 *   the channel.  Interrupts are disabled.
 *
 ****************************************************************************/

static void max3421e_chan_wakeup(FAR struct max3421e_usbhost_s *priv,
                              FAR struct max3421e_chan_s *chan)
{
  /* Is the transfer complete? */

  if (chan->result != EBUSY)
    {
      /* Is there a thread waiting for this transfer to complete? */

      if (chan->waiter)
        {
#ifdef CONFIG_USBHOST_ASYNCH
          /* Yes.. there should not also be a callback scheduled */

          DEBUGASSERT(chan->callback == NULL);
#endif
          /* Wake'em up! */

          usbhost_vtrace2(chan->in ? MAX3421E_VTRACE2_CHANWAKEUP_IN :
                                     MAX3421E_VTRACE2_CHANWAKEUP_OUT,
                          chan->epno, chan->result);

          max3421e_givesem(&chan->waitsem);
          chan->waiter = false;
        }

#ifdef CONFIG_USBHOST_ASYNCH
      /* No.. is an asynchronous callback expected when the transfer
       * completes?
       */

      else if (chan->callback)
        {
          /* Handle continuation of IN/OUT pipes */

          if (chan->in)
            {
              max3421e_in_next(priv, chan);
            }
          else
            {
              max3421e_out_next(priv, chan);
            }
        }
#endif
    }
}

/****************************************************************************
 * Name: max3421e_transfer_start
 *
 * Description:
 *   Start at transfer on the selected IN or OUT channel.
 *
 ****************************************************************************/

static void max3421e_transfer_start(FAR struct max3421e_usbhost_s *priv,
                                    int chidx)
{
  FAR struct max3421e_chan_s *chan;
  uin8_t regval;
  unsigned int npackets;
  unsigned int maxpacket;

  max3421e_pktdump("Sending", chan->buffer, chan->buflen);

  /* Set up the initial state of the transfer */

  chan           = &priv->chan[chidx];

  usbhost_vtrace2(MAX3421E_VTRACE2_STARTTRANSFER, chidx, chan->buflen);

  chan->result   = EBUSY;
  chan->inflight = 0;
  chan->xfrd     = 0;
  priv->chidx    = chidx;

  /* If the transfer size is greater than one packet, then calculate the
   * number of packets that will be received/sent, including any partial
   * final packet.
   */

  maxpacket = chan->maxpacket;

  if (chan->buflen > maxpacket)
    {
      npackets = (chan->buflen + maxpacket - 1) / maxpacket;
    }
  else
    {
      npackets = 1;
    }

  /* Save the number of packets in the transfer.  We will need this in
   * order to set the next data toggle correctly when the transfer
   * completes.
   */

  chan->npackets = (uint8_t)npackets;

  /* If this is an out transfer, then we need to copy the outgoing data into
   * SNDFIFO.
   */

  if (!chan->in && chan->buflen > 0)
    {
      FAR const uint8_t *src;
      unsigned int wrsize;
      unsigned int maxfifo;

      /* Yes.. Get the size of the biggest thing that we can put in the Tx FIFO now */

      wrsize  = chan->buflen;
      maxfifo = 2 * maxpacket; /* Double buffered */

      if (wrsize > maxfifo)
        {
          wrsize = maxfifo;
        }

      /* Write packet into the SNDFIFO. */

      max3421e_sndblock(priv, MAX3421E_USBHOST_SUDFIFO, chan->buffer, wrsize);

      /* Increment the count of bytes "in-flight" in the Tx FIFO */

      chan->inflight += wrsize;

      /* Did we put the entire buffer into the Tx FIFO? */

      if (wrsize < chan->buflen)
        {
          /* No, there was insufficient space to hold the entire transfer ...
           * Enable the Tx FIFO interrupt to handle the transfer when the
           * SNDFIFO becomes empty.
           */

          max3421e_int_enable(priv, USBHOST_HIRQ_SNDBAVIRQ);
        }
    }
}

/****************************************************************************
 * Name: max3421e_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 * Assumptions:
 *   Caller has the SPI locked
 *
 ****************************************************************************/

static int max3421e_ctrl_sendsetup(FAR struct max3421e_usbhost_s *priv,
                                   int chidx,
                                   FAR const struct usb_ctrlreq_s *req)
{
  FAR struct max3421e_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Loop while the device reports NAK (and a timeout is not exceeded */

  chan  = &priv->chan[chidx];
  start = clock_systimer();

  do
    {
      /* Send the  SETUP packet */

      chan->pid    = MAX3421E_PID_SETUP;
      chan->buffer = (FAR uint8_t *)req;
      chan->buflen = USB_SIZEOF_CTRLREQ;
      chan->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = max3421e_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN, 0);
          return ret;
        }

      /* Start the transfer */

      max3421e_transfer_start(priv, chidx);

      /* Wait for the transfer to complete */

      ret = max3421e_chan_wait(priv, chan);

      /* Return on success and for all failures other than EAGAIN.  EAGAIN
       * means that the device NAKed the SETUP command and that we should
       * try a few more times.
       */

      if (ret != -EAGAIN)
        {
          /* Output some debug information if the transfer failed */

          if (ret < 0)
            {
              usbhost_trace1(MAX3421E_TRACE1_TRNSFRFAILED, ret);
            }

          /* Return the result in any event */

          return ret;
        }

      /* Get the elapsed time (in frames) */

      elapsed = clock_systimer() - start;
    }
  while (elapsed < MAX3421E_SETUP_DELAY);

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: max3421e_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 * Assumptions:
 *   Caller has the SPI locked
 *
 ****************************************************************************/

static int max3421e_ctrl_senddata(FAR struct max3421e_usbhost_s *priv,
                                  int chidx, FAR uint8_t *buffer,
                                  unsigned int buflen)
{
  FAR struct max3421e_chan_s *chan = &priv->chan[chidx];
  int ret;

  /* Save buffer information */

  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  /* Set the DATA PID */

  if (buflen == 0)
    {
      /* For status OUT stage with buflen == 0, set PID DATA1 */

      chan->outdata1 = true;
    }

  /* Set the Data PID as per the outdata1 boolean */

  chan->pid = chan->outdata1 ? MAX3421E_PID_DATA1 : MAX3421E_PID_DATA0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = max3421e_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  max3421e_transfer_start(priv, chidx);

  /* Wait for the transfer to complete and return the result */

  return max3421e_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: max3421e_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive status
 *   in the status phase of an OUT control transfer
 *
 * Assumptions:
 *   Caller has the SPI locked
 *
 ****************************************************************************/

static int max3421e_ctrl_recvdata(FAR struct max3421e_usbhost_s *priv,
                                  FAR uint8_t *buffer, unsigned int buflen)
{
  FAR struct max3421e_chan_s *chan = &priv->chan[chidx];
  int ret;

  /* Save buffer information */

  chan->pid    = MAX3421E_PID_DATA1;
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  /* Set up for the wait BEFORE starting the transfer */

  ret = max3421e_chan_waitsetup(priv, chan);
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN, 0);
      return ret;
    }

  /* Start the transfer */

  max3421e_transfer_start(priv, chidx);

  /* Wait for the transfer to complete and return the result */

  return max3421e_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: max3421e_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int max3421e_in_setup(FAR struct max3421e_usbhost_s *priv, int chidx)
{
  FAR struct max3421e_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
    default:
    case USB_EP_ATTR_XFER_CONTROL: /* Control */
      {
        /* This kind of transfer on control endpoints other than EP0 are not
         * currently supported
         */

        return -ENOSYS;
      }

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous */
      {
        /* Set up the IN data PID */

        usbhost_vtrace2(MAX3421E_VTRACE2_ISOCIN, chidx, chan->buflen);
        chan->pid = MAX3421E_PID_DATA0;
      }
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk */
      {
        /* Setup the IN data PID */

        usbhost_vtrace2(MAX3421E_VTRACE2_BULKIN, chidx, chan->buflen);
        chan->pid = chan->indata1 ? MAX3421E_PID_DATA1 : MAX3421E_PID_DATA0;
      }
      break;

    case USB_EP_ATTR_XFER_INT: /* Interrupt */
      {
        /* Setup the IN data PID */

        usbhost_vtrace2(MAX3421E_VTRACE2_INTRIN, chidx, chan->buflen);
        chan->pid = chan->indata1 ? MAX3421E_PID_DATA1 : MAX3421E_PID_DATA0;
      }
      break;
    }

  /* Start the transfer */

  max3421e_transfer_start(priv, chidx);
  return OK;
}

/****************************************************************************
 * Name: max3421e_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 ****************************************************************************/

static ssize_t max3421e_in_transfer(FAR struct max3421e_usbhost_s *priv, int chidx,
                                 FAR uint8_t *buffer, size_t buflen)
{
  FAR struct max3421e_chan_s *chan;
  clock_t start;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs any error other than a simple NAK.  NAK would
   * simply indicate the end of the transfer (short-transfer).
   */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;
  xfrd         = 0;

  start = clock_systimer();
  while (chan->xfrd < chan->buflen)
    {
      /* Set up for the wait BEFORE starting the transfer */

      ret = max3421e_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      ret = max3421e_in_setup(priv, chidx);
      if (ret < 0)
        {
          uerr("ERROR: max3421e_in_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = max3421e_chan_wait(priv, chan);

      /* EAGAIN indicates that the device NAKed the transfer. */

      if (ret < 0)
        {
          /* The transfer failed.  If we received a NAK, return all data
           * buffered so far (if any).
           */

          if (ret == -EAGAIN)
            {
              /* Was data buffered prior to the NAK? */

              if (xfrd > 0)
                {
                  /* Yes, return the amount of data received.
                   *
                   * REVISIT: This behavior is clearly correct for CDC/ACM
                   * bulk transfers and HID interrupt transfers.  But I am
                   * not so certain for MSC bulk transfers which, I think,
                   * could have NAKed packets in the middle of a transfer.
                   */

                  return xfrd;
                }
              else
                {
                  useconds_t delay;

                  /* Get the elapsed time.  Has the timeout elapsed?
                   * if not then try again.
                   */

                  clock_t elapsed = clock_systimer() - start;
                  if (elapsed >= MAX3421E_DATANAK_DELAY)
                    {
                      /* Timeout out... break out returning the NAK as
                       * as a failure.
                       */

                      return (ssize_t)ret;
                    }

                  /* Wait a bit before retrying after a NAK. */

                  if (chan->eptype == MAX3421E_HCCHAR_EPTYP_INTR)
                    {
                      /* For interrupt (and isochronous) endpoints, the
                       * polling rate is determined by the bInterval field
                       * of the endpoint descriptor (in units of frames
                       * which we treat as milliseconds here).
                       */

                      if (chan->interval > 0)
                        {
                          /* Convert the delay to units of microseconds */

                          delay = (useconds_t)chan->interval * 1000;
                        }
                      else
                        {
                          /* Out of range! For interrupt endpoints, the valid
                           * range is 1-255 frames.  Assume one frame.
                           */

                          delay = 1000;
                        }
                    }
                  else
                    {
                      /* For Isochronous endpoints, bInterval must be 1.  Bulk
                       * endpoints do not have a polling interval.  Rather,
                       * the should wait until data is received.
                       *
                       * REVISIT:  For bulk endpoints this 1 msec delay is only
                       * intended to give the CPU a break from the bulk EP tight
                       * polling loop.  But are there performance issues?
                       */

                      delay = 1000;
                    }

                  /* Wait for the next polling interval.  For interrupt and
                   * isochronous endpoints, this is necessaryto assure the
                   * polling interval.  It is used in other cases only to
                   * prevent the polling from consuming too much CPU bandwith.
                   *
                   * Small delays could require more resolution than is provided
                   * by the system timer.  For example, if the system timer
                   * resolution is 10MS, then nxsig_usleep(1000) will actually request
                   * a delay 20MS (due to both quantization and rounding).
                   *
                   * REVISIT: So which is better?  To ignore tiny delays and
                   * hog the system bandwidth?  Or to wait for an excessive
                   * amount and destroy system throughput?
                   */

                  if (delay > CONFIG_USEC_PER_TICK)
                    {
                      nxsig_usleep(delay - CONFIG_USEC_PER_TICK);
                    }
                }
            }
          else
            {
              /* Some unexpected, fatal error occurred. */

              usbhost_trace1(MAX3421E_TRACE1_TRNSFRFAILED, ret);

              /* Break out and return the error */

              uerr("ERROR: max3421e_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }
        }
      else
        {
          /* Successfully received another chunk of data... add that to the
           * runing total.  Then continue reading until we read 'buflen'
           * bytes of data or until the devices NAKs (implying a short
           * packet).
           */

          xfrd += chan->xfrd;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: max3421e_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_in_next(FAR struct max3421e_usbhost_s *priv,
                          FAR struct max3421e_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer complete OK? */

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = max3421e_in_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      uerr("ERROR: max3421e_in_setup failed: %d\n", ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  uinfo("Transfer complete:  %d\n", result);

  /* Extract the callback information */

  callback       = chan->callback;
  arg            = chan->arg;
  nbytes         = chan->xfrd;

  chan->callback = NULL;
  chan->arg      = NULL;
  chan->xfrd     = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: max3421e_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_in_asynch(FAR struct max3421e_usbhost_s *priv, int chidx,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct max3421e_chan_s *chan;
  int ret;

  /* Set up for the transfer data and callback BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = max3421e_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      uerr("ERROR: max3421e_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = max3421e_in_setup(priv, chidx);
  if (ret < 0)
    {
      uerr("ERROR: max3421e_in_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: max3421e_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int max3421e_out_setup(FAR struct max3421e_usbhost_s *priv, int chidx)
{
  FAR struct max3421e_chan_s *chan;

  /* Set up for the transfer based on the direction and the endpoint type */

  chan = &priv->chan[chidx];
  switch (chan->eptype)
    {
    default:
    case USB_EP_ATTR_XFER_CONTROL: /* Control */
      {
        /* This kind of transfer on control endpoints other than EP0 are not
         * currently supported
         */

        return -ENOSYS;
      }

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous */
      {
        /* Set up the OUT data PID */

        usbhost_vtrace2(MAX3421E_VTRACE2_ISOCOUT, chidx, chan->buflen);
        chan->pid = MAX3421E_PID_DATA0;
      }
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk */
      {
        /* Setup the OUT data PID */

        usbhost_vtrace2(MAX3421E_VTRACE2_BULKOUT, chidx, chan->buflen);
        chan->pid = chan->outdata1 ? MAX3421E_PID_DATA1 : MAX3421E_PID_DATA0;
      }
      break;

    case USB_EP_ATTR_XFER_INT: /* Interrupt */
      {
        /* Setup the OUT data PID */

        usbhost_vtrace2(MAX3421E_VTRACE2_INTROUT, chidx, chan->buflen);
        chan->pid = chan->outdata1 ? MAX3421E_PID_DATA1 : MAX3421E_PID_DATA0;

        /* Toggle the OUT data PID for the next transfer */

        chan->outdata1 ^= true;
      }
      break;
    }

  /* Start the transfer */

  max3421e_transfer_start(priv, chidx);
  return OK;
}

/****************************************************************************
 * Name: max3421e_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 ****************************************************************************/

static ssize_t max3421e_out_transfer(FAR struct max3421e_usbhost_s *priv, int chidx,
                                  FAR uint8_t *buffer, size_t buflen)
{
  FAR struct max3421e_chan_s *chan;
  clock_t start;
  clock_t elapsed;
  size_t xfrlen;
  ssize_t xfrd;
  int ret;

  /* Loop until the transfer completes (i.e., buflen is decremented to zero)
   * or a fatal error occurs (any error other than a simple NAK)
   */

  chan  = &priv->chan[chidx];
  start = clock_systimer();
  xfrd  = 0;

  while (buflen > 0)
    {
      /* Transfer one packet at a time.  The hardware is capable of queueing
       * multiple OUT packets, but I just haven't figured out how to handle
       * the case where a single OUT packet in the group is NAKed.
       */

      xfrlen       = MIN(chan->maxpacket, buflen);
      chan->buffer = buffer;
      chan->buflen = xfrlen;
      chan->xfrd   = 0;

      /* Set up for the wait BEFORE starting the transfer */

      ret = max3421e_chan_waitsetup(priv, chan);
      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN, 0);
          return (ssize_t)ret;
        }

      /* Set up for the transfer based on the direction and the endpoint type */

      ret = max3421e_out_setup(priv, chidx);
      if (ret < 0)
        {
          uerr("ERROR: max3421e_out_setup failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Wait for the transfer to complete and get the result */

      ret = max3421e_chan_wait(priv, chan);

      /* Handle transfer failures */

      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_TRNSFRFAILED, ret);

          /* Check for a special case:  If (1) the transfer was NAKed and (2)
           * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
           * should be able to just flush the Rx and Tx FIFOs and try again.
           * We can detect this latter case because then the transfer buffer
           * pointer and buffer size will be unaltered.
           */

          elapsed = clock_systimer() - start;
          if (ret != -EAGAIN ||                  /* Not a NAK condition OR */
              elapsed >= MAX3421E_DATANAK_DELAY ||  /* Timeout has elapsed OR */
              chan->xfrd > 0)                    /* Data has been partially transferred */
            {
              /* Break out and return the error */

              uerr("ERROR: max3421e_chan_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Is this flush really necessary? What does the hardware do with the
           * data in the FIFO when the NAK occurs?  Does it discard it?
           */

          max3421e_flush_txfifos(MAX3421E_GRSTCTL_TXFNUM_HALL);

          /* Get the device a little time to catch up.  Then retry the transfer
           * using the same buffer pointer and length.
           */

          nxsig_usleep(20*1000);
        }
      else
        {
          /* Successfully transferred.  Update the buffer pointer and length */

          buffer += xfrlen;
          buflen -= xfrlen;
          xfrd   += chan->xfrd;
        }
    }

  return xfrd;
}

/****************************************************************************
 * Name: max3421e_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void max3421e_out_next(FAR struct max3421e_usbhost_s *priv,
                           FAR struct max3421e_chan_s *chan)
{
  usbhost_asynch_t callback;
  FAR void *arg;
  ssize_t nbytes;
  int result;
  int ret;

  /* Is the full transfer complete? Did the last chunk transfer complete OK? */

  result = -(int)chan->result;
  if (chan->xfrd < chan->buflen && result == OK)
    {
      /* Yes.. Set up for the next transfer based on the direction and the
       * endpoint type
       */

      ret = max3421e_out_setup(priv, chan->chidx);
      if (ret >= 0)
        {
          return;
        }

      uerr("ERROR: max3421e_out_setup failed: %d\n", ret);
      result = ret;
    }

  /* The transfer is complete, with or without an error */

  uinfo("Transfer complete:  %d\n", result);

  /* Extract the callback information */

  callback       = chan->callback;
  arg            = chan->arg;
  nbytes         = chan->xfrd;

  chan->callback = NULL;
  chan->arg      = NULL;
  chan->xfrd     = 0;

  /* Then perform the callback */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: max3421e_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_out_asynch(FAR struct max3421e_usbhost_s *priv, int chidx,
                            FAR uint8_t *buffer, size_t buflen,
                            usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct max3421e_chan_s *chan;
  int ret;

  /* Set up for the transfer data and callback BEFORE starting the first transfer */

  chan         = &priv->chan[chidx];
  chan->buffer = buffer;
  chan->buflen = buflen;
  chan->xfrd   = 0;

  ret = max3421e_chan_asynchsetup(priv, chan, callback, arg);
  if (ret < 0)
    {
      uerr("ERROR: max3421e_chan_asynchsetup failed: %d\n", ret);
      return ret;
    }

  /* Set up for the transfer based on the direction and the endpoint type */

  ret = max3421e_out_setup(priv, chidx);
  if (ret < 0)
    {
      uerr("ERROR: max3421e_out_setup failed: %d\n", ret);
    }

  /* And return with the transfer pending */

  return ret;
}
#endif

/****************************************************************************
 * Name: max3421e_connect_event
 *
 * Description:
 *   Send a token to the device.
 *
 ****************************************************************************/

static int max4321e_sendtoken(FAR struct max3421e_usbhost_s *priv,
                              uint8_t epno, uint8_t token)
{
  uint16_t timeout;
  uint_t regval;
  int ret;

  /* Send the specified token */

  max3421e_putreg(priv, MAX3421E_USBHOST_HXFR, token | epno);
  usleep(23);

  timeout = 0xffff;
  while (timeout > 0)
    {
      /* Get the result */

      regval  = max3421e_getreg(priv, MAX3421E_USBHOST_HRSL);
      regval &= USBHOST_HRSL_HRSLT_MASK;

      if (regval == USBHOST_HRSL_HRSLT_BUSY)
        {
          /* The result is not yet available */

          usleep(3);
        }
      else if (regval == USBHOST_HRSL_HRSLT_NAK)
        {
          /* The request was NAKed */

          timeout--;

          /* Send the token again */

          max3421e_putreg(priv, MAX3421E_USBHOST_HXFR, token | epno);
          usleep(5);
        }
      else if (regval != USBHOST_HRSL_HRSLT_SUCCESS)
        {
          return -EIO;
        }
      else
        {
          return OK;
        }
    }

    return -ETIMEDOUT;
}

/****************************************************************************
 * Name: max3421e_connect_event
 *
 * Description:
 *   Handle a connection event.
 *
 ****************************************************************************/

static void max3421e_connect_event(FAR struct max3421e_usbhost_s *priv)
{
  /* We we previously disconnected? */

  if (!priv->connected)
    {
      /* Yes.. then now we are connected */

      usbhost_vtrace1(MAX3421E_VTRACE1_CONNECTED, 0);
      priv->connected = true;
      priv->change    = true;
      DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

      /* Notify any waiters */

      priv->smstate = SMSTATE_ATTACHED;
      if (priv->pscwait)
        {
          max3421e_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: max3421e_disconnect_event
 *
 * Description:
 *   Handle a disconnection event.
 *
 ****************************************************************************/

static void max3421e_disconnect_event(FAR struct max3421e_usbhost_s *priv)
{
  /* Were we previously connected? */

  if (priv->connected)
    {
      /* Yes.. then we no longer connected */

      usbhost_vtrace1(MAX3421E_VTRACE1_DISCONNECTED, 0);

      /* Are we bound to a class driver? */

      if (priv->rhport.hport.devclass)
        {
          /* Yes.. Disconnect the class driver */

          CLASS_DISCONNECTED(priv->rhport.hport.devclass);
          priv->rhport.hport.devclass = NULL;
        }

      /* Re-Initialize Host for new Enumeration */

      priv->smstate   = SMSTATE_DETACHED;
      priv->connected = false;
      priv->change    = true;

      priv->rhport.hport.speed = USB_SPEED_FULL;
      priv->rhport.hport.funcaddr = 0;

      /* Notify any waiters that there is a change in the connection state */

      if (priv->pscwait)
        {
          max3421e_givesem(&priv->pscsem);
          priv->pscwait = false;
        }
    }
}

/****************************************************************************
 * Name: max3421e_connected
 *
 * Description:
 *   USB host port interrupt handler
 *
 ****************************************************************************/

static int max3421e_connected(FAR struct max3421e_usbhost_s *priv)
{
  int ret;

  /* Stop SOF generation and reset the bus */

  max3421e_busreset(priv);
  sleep(1);

  /* Check for low- or full-speed and restart SOF generation. */

  ret = max3421e_startsof(priv);
  if (ret < 0)
    {
      usbhost_vtrace1(MAX3421E_VTRACE1_INT_DISCONNECTED, 0);
      return ret;
    }

  usbhost_vtrace1(MAX3421E_VTRACE1_INT_CONNECTED, 0);

  /* Were we previously disconnected? */

  max3421e_connect_event(priv);
  return OK;
}

/****************************************************************************
 * Name: max3421e_disconnected
 *
 * Description:
 *   USB disconnect detected interrupt handler
 *
 ****************************************************************************/

static void max3421e_disconnected(FAR struct max3421e_usbhost_s *priv)
{
  usbhost_vtrace1(MAX3421E_VTRACE1_INT_DISCONNECTED, 0);

  /* Disable the SOF generator */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE, USBHOST_MODE_SOFKAENAB, 0);

  /* Handle the disconnection event */

  max3421e_disconnect_event(priv);
}

/****************************************************************************
 * Name: max3421e_irqwork
 *
 * Description:
 *   MAX3421E interrupt worker.  Perform MAX3421E interrupt processing on the
 *   high priority work queue thread.  Interrupts were disabled by the
 *   interrupt handler when the interrupt was received.  This worker must
 *   re-enable MAX3421E interrupts when interrupt processing is complete.
 *
 ****************************************************************************/

static int max3421e_irqwork(FAR void *arg)
{
  FAR struct max3421e_usbhost_s *priv;
  uint8_t pending;
  uint8_t regval;

  priv = (FAR struct max3421e_usbhost_s *)arg;
  DEBUGASSERT(priv != NULL && priv-lower != NULL);

  /* Loop while there are pending interrupts to process.  This loop may save a
   * little interrupt handling overhead.
   */

  for (; ; )
    {
      /* Get the unmasked bits in the GINT status */

      pending = max3421e_int_status(priv);
      lower->acknowledge(lower);

      /* Break out of the loop when there are no further pending interrupts. */

      if (pending == 0)
        {
          break;
        }

      /* Possibilities:
       *
       *   HXFRDNIRQ
       *   FRAMEIRQ
       *   CONNIRQ
       *   SUSDNIRQ
       *   SNDBAVIRQ
       *   RCVDAVIRQ
       *   RSMREQIRQ
       *   BUSEVENTIRQ
       *
       * Only CONNIRQ handled here.
       */

      /* CONNIRQ: Has a peripheral been connected or disconnected */

      if ((pending & USBHOST_HIRQ_CONNIRQ) != 0)
        {
          /* Check if a peripheral device has been connected */

          ret = max3421e_connected(priv);
          if (ret < 0)
            {
              /* No.. then a device must have been disconnected. */

              max3421e_disconnected(priv);
            }

          /* Clear the pending CONNIRQ interrupt */

          max3421e_putreg(priv, MAX3421E_USBHOST_HIRQ,
                          USBHOST_HIRQ_CONNIRQ);
        }

      /* SNDBAV: The SNDFIFO is available */

      else if ((pending & USBHOST_HIRQ_SNDBAVIRQ) != 0)
        {
          /* Disable furtherSNDBAV interrupts */

           max3421e_int_disable(priv, USBHOST_HIRQ_SNDBAVIRQ);

          /* Finish long transfer, possibly re-enabling the SNDBAV
           * interrupt (see max3421e_transfer_start)
           */
#warning Missing logic

          /* Clear the pending NDBAV interrupt */

          max3421e_putreg(priv, MAX3421E_USBHOST_HIRQ,
                          USBHOST_HIRQ_SNDBAVIRQ);
        }
    }

  /* Re-enable interrupts */

  lower->enable(lower, true);
}

/****************************************************************************
 * Name: max3421e_interrupt
 *
 * Description:
 *   MAX3421E interrupt handler.  This interrupt handler simply defers
 *   interrupt processing to the high priority work queue thread.  This is
 *   necessary because we cannot perform interrupt/DMA driven SPI accesses
 *   from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct max3421e_usbhost_s *priv;
  FAR const struct max3421e_lowerhalf_s *lower;

  priv = (FAR struct max3421e_usbhost_s *)arg;
  DEBUGASSERT(priv != NULL && priv-lower != NULL);
  lower = (FAR const struct max3421e_lowerhalf_s *)priv->lower;

  /* Disable further interrupts until work associated with this interrupt
   * has been processed.
   */

  lower->enable(lower, false);

  /* And defer interrupt processing to the high priority work queue thread */

  (void)work_queue(LPWORK, &priv->irqwork, max3421e_irqwork, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: max3421e_int_enable, max3421e_int_disable, and max3421e_int_status
 *
 * Description:
 *   Respectively enable, disable, or get status of the USB host interrupt
 *   (HIRQ) and a mask of enabled interrupts.
 *
 * Input Parameters:
 *   priv - Private state data
 *   irqset - IRQ bits to be set (max3421e_int_status only)
 *
 * Returned Value:
 *   The current unmasks interrupt status  (max3421e_int_status only)
 *
 ****************************************************************************/

static inline void max3421e_int_enable(FAR struct max3421e_usbhost_s *priv,
                                       uint8_t irqset)
{
  priv->irqset |= irqset;
  max3421e_puttreg(priv, MAX3421E_USBHOST_HIEN, priv->irqset);
}

static inline void max3421e_int_disable(FAR struct max3421e_usbhost_s *priv,
                                        uint8_t irqset)
{
  priv->irqset &= ~irqset;
  max3421e_puttreg(priv, MAX3421E_USBHOST_HIEN, priv->irqset);
}

static inline uint8_t max3421e_int_status(FAR struct max3421e_usbhost_s *priv)
{
  return max3421e_getreg(priv, MAX3421E_USBHOST_HIEN) & priv->irqset;
}

/****************************************************************************
 * Name: max3421e_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *      connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device in connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_wait(FAR struct usbhost_connection_s *conn,
                         FAR struct usbhost_hubport_s **hport)
{
  FAR struct max3421e_connection_s *maxconn;
  FAR struct max3421e_usbhost_s *priv;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;

  maxconn = (FAR struct max3421e_connection_s *)conn;
  DEBUGASSERT(maxconn != NULL && maxconn->priv != NULL);
  priv = maxconn->priv;

  /* Loop until a change in connection state is detected */

  flags = enter_critical_section();
  for (; ; )
    {
      /* Is there a change in the connection state of the single root hub
       * port?
       */

      if (priv->change)
        {
          connport = &priv->rhport.hport;

          /* Yes. Remember the new state */

          connport->connected = priv->connected;
          priv->change = false;

          /* And return the root hub port */

          *hport = connport;
          leave_critical_section(flags);

          uinfo("RHport Connected: %s\n", connport->connected ? "YES" : "NO");
          return OK;
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (priv->hport)
        {
          /* Yes.. return the external hub port */

          connport = (struct usbhost_hubport_s *)priv->hport;
          priv->hport = NULL;

          *hport = connport;
          leave_critical_section(flags);

          uinfo("Hub port Connected: %s\n", connport->connected ? "YES" : "NO");
          return OK;
        }
#endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      max3421e_takesem(&priv->pscsem);
    }
}

/****************************************************************************
 * Name: max3421e_getspeed
 *
 * Description:
 *   Get the speed of the connected device.
 *
 * Input Parameters:
 *   priv - Driver private state structure
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_getspeed(FAR struct max3421e_usbhost_s *priv,
                             FAR struct usbhost_connection_s *conn,
                             FAR struct usbhost_hubport_s *hport)
{
  uin8_t regval;
  int ret;

  DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      usbhost_trace1(MAX3421E_TRACE1_DEVDISCONN, 0);
      return -ENODEV;
    }

  DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

  /* USB 2.0 spec says at least 50ms delay before port reset.  We wait
   * 100ms.
   */

  nxsig_usleep(100*1000);

  /* Stop SOF generation and reset the host port */

  max3421e_busreset(priv);
  sleep(1);

  /* Get the current device speed */

  ret = max3421e_startsof(priv);
  if (ret < 0)
    {
      usbhost_vtrace1(MAX3421E_VTRACE1_INT_DISCONNECTED, 0);
    }

  return ret;
}

static int max3421e_enumerate(FAR struct usbhost_connection_s *conn,
                              FAR struct usbhost_hubport_s *hport)
{
  FAR struct max3421e_connection_s *maxconn;
  FAR struct max3421e_usbhost_s *priv;
  int ret;

  maxconn = (FAR struct max3421e_connection_s *)conn;
  DEBUGASSERT(maxconn != NULL && maxconn->priv != NULL);
  priv = maxconn->priv;

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */
#warning REVISIT:  Isn't this already done?

ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = max3421e_getspeed(priv, conn, hport);
      if (ret < 0)
        {
           return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uinfo("Enumerate the device\n");
  priv->smstate = SMSTATE_ENUM;
  ret = usbhost_enumerate(hport, &hport->devclass);

  /* The enumeration may fail either because of some HCD interfaces failure
   * or because the device class is not supported.  In either case, we just
   * need to perform the disconnection operation and make ready for a new
   * enumeration.
   */

  if (ret < 0)
    {
      /* Return to the disconnected state */

      uerr("ERROR: Enumeration failed: %d\n", ret);
      max3421e_disconnect_event(priv);
    }

  return ret;
}

/************************************************************************************
 * Name: max3421e_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support an
 *   external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   speed - The speed of the port USB_SPEED_LOW or _FULL
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int max3421e_ep0configure(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                                 uint8_t funcaddr, uint8_t speed,
                                 uint16_t maxpacketsize)
{
  FAR struct max3421e_usbhost_s *priv = (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;

  DEBUGASSERT(drvr != NULL && funcaddr < 128 && maxpacketsize <= 64);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Configure the EP0 channel */

  chan            = &priv->chan[EP0];
  chan->funcaddr  = funcaddr;
  chan->speed     = speed;
  chan->maxpacket = maxpacketsize;

  max3421e_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: max3421e_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int max3421e_epalloc(FAR struct usbhost_driver_s *drvr,
                            FAR const struct usbhost_epdesc_s *epdesc,
                            FAR usbhost_ep_t *ep)
{
  FAR struct max3421e_usbhost_s *priv = (FAR struct max3421e_usbhost_s *)drvr;
  struct usbhost_hubport_s *hport;
  FAR struct max3421e_chan_s *chan;
  uint16_t maxpacket;
  uint8_t epno;
  int ret;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);
  hport = epdesc->hport;
  DEBUGASSERT(hport != NULL);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Allocate a host channel for the endpoint */

  chidx = max3421e_chan_alloc(priv);
  if (chidx < 0)
    {
      uerr("ERROR: Failed to allocate a host channel\n");
      max3421e_givesem(&priv->exclsem);
      return chidx;
    }

  /* Decode the endpoint descriptor to initialize the channel data structures.
   * Note:  Here we depend on the fact that the endpoint point type is
   * encoded in the same way in the endpoint descriptor as it is in the OTG
   * HS hardware.
   */

  chan            = &priv->chan[epno];
  chan->epno      = epdesc->addr & USB_EPNO_MASK;
  chan->in        = epdesc->in;
  chan->eptype    = epdesc->xfrtype;
  chan->funcaddr  = hport->funcaddr;
  chan->speed     = hport->speed;
  chan->interval  = epdesc->interval;
  chan->maxpacket = epdesc->mxpacketsize;;
  chan->indata1   = false;
  chan->outdata1  = false;

  /* Return the endpoint number as the endpoint "handle" */

  *ep = (usbhost_ep_t)chidx;
  max3421e_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: max3421e_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int max3421e_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct max3421e_usbhost_s *priv = (FAR struct max3421e_usbhost_s *)drvr;

  DEBUGASSERT(priv);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Halt the channel and mark the channel available */

  max3421e_chan_free(priv, (int)ep);
  max3421e_givesem(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: max3421e_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_alloc(FAR struct usbhost_driver_s *drvr,
                          FAR uint8_t **buffer, FAR size_t *maxlen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special memory requirement for the MAX3421E. */

  alloc = (FAR uint8_t *)kmm_malloc(CONFIG_MAX3421E_DESCSIZE);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated address and size of the descriptor buffer */

  *buffer = alloc;
  *maxlen = CONFIG_MAX3421E_DESCSIZE;
  return OK;
}

/****************************************************************************
 * Name: max3421e_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/************************************************************************************
 * Name: max3421e_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int max3421e_ioalloc(FAR struct usbhost_driver_s *drvr,
                            FAR uint8_t **buffer, size_t buflen)
{
  FAR uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* There is no special memory requirement */

  alloc = (FAR uint8_t *)kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated buffer */

  *buffer = alloc;
  return OK;
}

/************************************************************************************
 * Name: max3421e_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int max3421e_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  /* There is no special memory requirement */

  DEBUGASSERT(drvr && buffer);
  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: max3421e_ctrlin and max3421e_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int max3421e_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                           FAR const struct usb_ctrlreq_s *req,
                           FAR uint8_t *buffer)
{
  FAR struct max3421e_usbhost_s *priv = (FAR struct max3421e_usbhost_s *)drvr;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int retries;
  int chidx;
  int ret;

  DEBUGASSERT(priv != NULL && req != NULL);
  chidx = (int)ep;
  DEBUGASSERT(chidx >= 0 && chidx < MAX3421E_NHOST_CHANNELS);

  usbhost_vtrace2(MAX3421E_VTRACE2_CTRLIN, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = max3421e_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < MAX3421E_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      max3421e_lock(priv);
      ret = max3421e_ctrl_sendsetup(priv, chidx, req);
      max3421e_unlock(priv);

      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systimer();
      do
        {
          /* Handle the IN data phase (if any) */

          if (buflen > 0)
            {
              max3421e_lock(priv);
              ret = max3421e_ctrl_recvdata(priv, chidx, buffer, buflen);
              max3421e_unlock(priv);

              if (ret < 0)
                {
                  usbhost_trace1(MAX3421E_TRACE1_RECVDATA, -ret);
                }
            }

          /* Handle the status OUT phase */

          if (ret == OK)
            {
              priv->chan[ep0info->outndx].outdata1 ^= true;

              max3421e_lock(priv);
              ret = max3421e_ctrl_senddata(priv, chidx, NULL, 0);
              max3421e_unlock(priv);

              if (ret == OK)
                {
                  /* All success transactions exit here */

                  max3421e_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(MAX3421E_TRACE1_SENDDATA, ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systimer() - start;
        }
      while (elapsed < MAX3421E_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts have been exhausted */

  max3421e_givesem(&priv->exclsem);
  return -ETIMEDOUT;
}

static int max3421e_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                            FAR const struct usb_ctrlreq_s *req,
                            FAR const uint8_t *buffer)
{
  FAR struct max3421e_usbhost_s *priv = (FAR struct max3421e_usbhost_s *)drvr;
  uint16_t buflen;
  clock_t start;
  clock_t elapsed;
  int chidx;
  int retries;
  int ret;

  DEBUGASSERT(priv != NULL && req != NULL);
  chidx = (int)ep;
  DEBUGASSERT(chidx >= 0 && chidx < MAX3421E_NHOST_CHANNELS);

  usbhost_vtrace2(MAX3421E_VTRACE2_CTRLOUT, req->type, req->req);
  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* Extract values from the request */

  buflen = max3421e_getle16(req->len);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Loop, retrying until the retry time expires */

  for (retries = 0; retries < MAX3421E_RETRY_COUNT; retries++)
    {
      /* Send the SETUP request */

      max3421e_lock(priv);
      ret = max3421e_ctrl_sendsetup(priv, chidx, req);
      max3421e_unlock(priv);

      if (ret < 0)
        {
          usbhost_trace1(MAX3421E_TRACE1_SENDSETUP, -ret);
          continue;
        }

      /* Get the start time.  Loop again until the timeout expires */

      start = clock_systimer();
      do
        {
          /* Handle the data OUT phase (if any) */

          if (buflen > 0)
            {
              /* Start DATA out transfer (only one DATA packet) */

              priv->chan[EP0].outdata1 = true;

              max3421e_lock(priv);
              ret = max3421e_ctrl_senddata(priv, chidx, NULL, 0);
              max3421e_unlock(priv);

              if (ret < 0)
                {
                  usbhost_trace1(MAX3421E_TRACE1_SENDDATA, -ret);
                }
            }

          /* Handle the status IN phase */

          if (ret == OK)
            {
              max3421e_lock(priv);
              ret = max3421e_ctrl_recvdata(priv, chidx, NULL, 0);
              max3421e_unlock(priv);

              if (ret == OK)
                {
                  /* All success transactions exit here */

                  max3421e_givesem(&priv->exclsem);
                  return OK;
                }

              usbhost_trace1(MAX3421E_TRACE1_RECVDATA, ret < 0 ? -ret : ret);
            }

          /* Get the elapsed time (in frames) */

          elapsed = clock_systimer() - start;
        }
      while (elapsed < MAX3421E_DATANAK_DELAY);
    }

  /* All failures exit here after all retries and timeouts have been exhausted */

  max3421e_givesem(&priv->exclsem);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: max3421e_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes. Only
 *   one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value is
 *   returned that indicates the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static ssize_t max3421e_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                                 FAR uint8_t *buffer, size_t buflen)
{
  FAR struct max3421e_usbhost_s *priv  = (FAR struct max3421e_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  ssize_t nbytes;

  uinfo("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < MAX3421E_NHOST_CHANNELS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      nbytes = max3421e_in_transfer(priv, chidx, buffer, buflen);
    }
  else
    {
      nbytes = max3421e_out_transfer(priv, chidx, buffer, buflen);
    }

  max3421e_givesem(&priv->exclsem);
  return nbytes;
}

/****************************************************************************
 * Name: max3421e_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the callback will be invoked with the provided transfer.
 *   This method is useful for receiving interrupt transfers which may come
 *   infrequently.
 *
 *   Only one transfer may be queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until the transfer completes.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback function
 *     when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int max3421e_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                           FAR uint8_t *buffer, size_t buflen,
                           usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct max3421e_usbhost_s *priv  = (FAR struct max3421e_usbhost_s *)drvr;
  unsigned int chidx = (unsigned int)ep;
  int ret;

  uinfo("chidx: %d buflen: %d\n",  (unsigned int)ep, buflen);

  DEBUGASSERT(priv && buffer && chidx < MAX3421E_NHOST_CHANNELS && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  max3421e_takesem(&priv->exclsem);

  /* Handle IN and OUT transfer slightly differently */

  if (priv->chan[chidx].in)
    {
      ret = max3421e_in_asynch(priv, chidx, buffer, buflen, callback, arg);
    }
  else
    {
      ret = max3421e_out_asynch(priv, chidx, buffer, buflen, callback, arg);
    }

  max3421e_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/************************************************************************************
 * Name: max3421e_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Canceled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which an
 *      asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

static int max3421e_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct max3421e_usbhost_s *priv  = (FAR struct max3421e_usbhost_s *)drvr;
  FAR struct max3421e_chan_s *chan;
  unsigned int chidx = (unsigned int)ep;
  irqstate_t flags;

  uinfo("chidx: %u: %d\n",  chidx);

  DEBUGASSERT(priv && chidx < MAX3421E_NHOST_CHANNELS);
  chan = &priv->chan[chidx];

  /* We need to disable interrupts to avoid race conditions with the asynchronous
   * completion of the transfer being cancelled.
   */

  flags = enter_critical_section();

  /* Halt the channel */
#warning Missing logic

  chan->result = -ESHUTDOWN;

  /* Is there a thread waiting for this transfer to complete? */

  if (chan->waiter)
    {
#ifdef CONFIG_USBHOST_ASYNCH
      /* Yes.. there should not also be a callback scheduled */

      DEBUGASSERT(chan->callback == NULL);
#endif

      /* Wake'em up! */

      max3421e_givesem(&chan->waitsem);
      chan->waiter = false;
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. is an asynchronous callback expected when the transfer
   * completes?
   */

  else if (chan->callback)
    {
      usbhost_asynch_t callback;
      FAR void *arg;

      /* Extract the callback information */

      callback       = chan->callback;
      arg            = chan->arg;

      chan->callback = NULL;
      chan->arg      = NULL;
      chan->xfrd     = 0;

      /* Then perform the callback */

      callback(arg, -ESHUTDOWN);
    }
#endif

  leave_critical_section(flags);
  return OK;
}

/************************************************************************************
 * Name: max3421e_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int max3421e_connect(FAR struct usbhost_driver_s *drvr,
                            FAR struct usbhost_hubport_s *hport,
                            bool connected)
{
  FAR struct max3421e_usbhost_s *priv = (FAR struct max3421e_usbhost_s *)drvr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && hport != NULL);

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  uinfo("Hub port %d connected: %s\n", hport->port, connected ? "YES" : "NO");

  /* Report the connection event */

  flags = enter_critical_section();
  priv->hport = hport;
  if (priv->pscwait)
    {
      priv->pscwait = false;
      max3421e_givesem(&priv->pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: max3421e_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be a port
 *      on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void max3421e_disconnect(FAR struct usbhost_driver_s *drvr,
                                FAR struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/
/****************************************************************************
 * Name: max3421e_busreset
 *
 * Description:
 *   Reset the USB host port.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void max3421e_busreset(FAR struct max3421e_usbhost_s *priv)
{
  uint8_t regval;

  /* Disable the SOF generator */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE, USBHOST_MODE_SOFKAENAB, 0);

  /* Perform the reset */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_HCTL, 0, USBHOST_HCTL_BUSRST);
  while ((max3421e_regreg(priv, MAX3421E_USBHOST_HCTL) & MAX3421E_USBHOST_HCTL) == 0)
    {
      usleep(250);
    }
}

/****************************************************************************
 * Name: max3421e_startsof
 *
 * Description:
 *   Called after bus reset.  Determine bus speed and restart SOFs.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   OK if successfully connect; -ENODEV if not connected.
 *
 ****************************************************************************/

static int max3421e_startsof(FAR struct max3421e_usbhost_s *priv)
{
  uint8_t clrbits;
  uint8_t setbits;
  uint8_t regval;
  bool lowspeed;

  /* Check if we are already in low- or full-speed mode */

  regval = max3421e_getreg(priv, MAX3421E_USBHOST_MODE);
  lowspeed = ((regval & USBHOST_MODE_SPEED) != 0);

  /* Enable SAMPLEBUS */

  max3421e_modifyreg(MAX3421E_USBHOST_HCTL, 0, USBHOST_HCTL_BUSSAMPLE);
  while (max3421e_getreg(priv, MAX3421E_USBHOST_HCTL) &
         USBHOST_HCTL_BUSSAMPLE) == 0)
    {
      usleep(5);
    }

  /* Check for low- or full-speed and start SOF (actually already started
   * by max3421e_busreset).
   */

  clrbits   = 0;
  startbits = USBHOST_MODE_HOST | USBHOST_MODE_DMPULLD | USBHOST_MODE_DPPULLDN;

  regval = max3421e_getreg(priv, MAX3421E_USBHOST_HSRL);
  switch (regval & (USBHOST_HRSL_KSTATUS | USBHOST_HRSL_JSTATUS)
    {
      default:
      case (USBHOST_HRSL_KSTATUS | USBHOST_HRSL_JSTATUS):
        /* Invalid state */

        usbhost_vtrace1(MAX3421E_BAD_JKSTATE, 0);

        /* Fall through */

      case 0:
        /* 0:  Not connected */

        return -ENODEV;

      case USBHOST_HRSL_KSTATUS:
        /* J=0, K=1: low-speed in full-speed (or vice versa) */

        if (lowspeed)
          {
            /* Full speed in low speed */

            clrbits = USBHOST_MODE_SPEED;
          }
        else
          {
            /* Low speed in full speed */

            setbits = USBHOST_MODE_SPEED;
          }
        break;

      case USBHOST_HRSL_JSTATUS:
        /* J=1,K=0: full-speed in full-speed (or vice versa) */

        if (lowspeed)
          {
            /* Low speed in low speed */

            setbits = USBHOST_MODE_SPEED;
          }
        else
          {
            /* Full speed in full speed */

            clrbits = USBHOST_MODE_SPEED;
          }
        break;
    }

  /* Restart the SOF generator */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE, 0, USBHOST_MODE_SOFKAENAB);

 /* Wait for the first SOF received and 20ms has passed */

 while ((max3421e_getreg(priv, MAX3421E_USBHOST_HIRQ) &
         USBHOST_HIRQ_FRAMEIRQ) == 0)
   {
   }

  usleep(20*1000);
  return OK;
}

/****************************************************************************
 * Name: max3421e_flush_txfifos
 *
 * Description:
 *   Flush the selected Tx FIFO.
 *
 * Input Parameters:
 *   txfnum -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void max3421e_flush_txfifos(uint32_t txfnum)
{
#warning Missing logic
}

/****************************************************************************
 * Name: max3421e_flush_rxfifo
 *
 * Description:
 *   Flush the Rx FIFO.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void max3421e_flush_rxfifo(void)
{
#warning Missing logic
}

/****************************************************************************
 * Name: max3421e_vbusdrive
 *
 * Description:
 *   Drive the Vbus +5V.
 *
 * Input Parameters:
 *   priv  - USB host driver private data structure.
 *   state - True: Drive, False: Don't drive
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void max3421e_vbusdrive(FAR struct max3421e_usbhost_s *priv, bool state)
{
#warning Missing logic
}

/****************************************************************************
 * Name: max3421e_sw_initialize
 *
 * Description:
 *   One-time setup of the host driver state structure.
 *
 * Input Parameters:
 *   priv  -- USB host driver private data structure.
 *   conn  -- Custom USB host connection structure.
 *   lower -- The lower half driver instance.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline int max3421e_sw_initialize(FAR struct max3421e_usbhost_s *priv,
              FAR struct usbhost_connection_s conn *conn,
              FAR const struct max3421e_lowerhalf_s *lower);
{
  FAR struct usbhost_driver_s *drvr;
  FAR struct usbhost_hubport_s *hport;
  int ret;
  int i;

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = max3421e_ep0configure;
  drvr->epalloc        = max3421e_epalloc;
  drvr->epfree         = max3421e_epfree;
  drvr->alloc          = max3421e_alloc;
  drvr->free           = max3421e_free;
  drvr->ioalloc        = max3421e_ioalloc;
  drvr->iofree         = max3421e_iofree;
  drvr->ctrlin         = max3421e_ctrlin;
  drvr->ctrlout        = max3421e_ctrlout;
  drvr->transfer       = max3421e_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = max3421e_asynch;
#endif
  drvr->cancel         = max3421e_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = max3421e_connect;
#endif
  drvr->disconnect     = max3421e_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = (usbhost_ep_t)&priv->ep0;
  hport->speed         = USB_SPEED_FULL;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->rhport);

  /* Initialize semaphores */

  nxsem_init(&priv->pscsem,  0, 0);
  nxsem_init(&priv->exclsem, 0, 1);

  /* The pscsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_setprotocol(&priv->pscsem, SEM_PRIO_NONE);

  /* Initialize the driver state data */

  priv->lower     = lower;
  priv->smstate   = SMSTATE_DETACHED;
  priv->connected = false;
  priv->ackstat   = MAX3421E_ACKSTAT_FALSE;
  priv->irqset    = 0;
  priv->change    = false;

  /* Put all of the channels back in their initial, allocated state */

  memset(priv->chan, 0,
         MAX3421E_NHOST_CHANNELS * sizeof(struct max3421e_chan_s));

  /* Initialize each channel */

  for (i = 0; i < MAX3421E_NHOST_CHANNELS; i++)
    {
      FAR struct max3421e_chan_s *chan = &priv->chan[i];

      chan->chidx = i;

      /* The waitsem semaphore is used for signaling and, hence, should not
       * have priority inheritance enabled.
       */

      nxsem_init(&chan->waitsem,  0, 0);
      nxsem_setprotocol(&chan->waitsem, SEM_PRIO_NONE);
    }

  /* Initialize the connection structure */

  conn->conn.wait = max3421e_wait;
  conn->conn.wait = max3421e_enumerate;
  conn->priv      = priv;

  /* Attach USB host controller interrupt handler */

  ret = lower->attach(lower, max3421e_interrupt, priv)'
  if (ret < 0)
    {
      usbhost_trace1(MAX3421E_TRACE1_IRQATTACH, 0);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: max3421e_hw_initialize
 *
 * Description:
 *   One-time setup of the host controller harware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int max3421e_hw_initialize(FAR struct max3421e_usbhost_s *priv)
{
  uint8_t revision;
  int ret;

  /* Get exclusive access to the SPI bus */

  max3421e_lock(priv);

  /* Reset the MAX3421E by toggling the CHIPRES bit in the USBCTRL register. */

  max3421e_putreg(priv, MAX3421E_USBHOST_USBCTL, USBHOST_USBCTL_CHIPRES);
  max3421e_putreg(priv, MAX3421E_USBHOST_USBCTL, 0);

  /* Wait for the oscillator to become stable */

  while ((max3421e_gutreg(priv, MAX3421E_USBHOST_USBIRQ) &
                          USBHOST_USBIRQ_OSCOKIRQ) == 0)
    {
    }

  /* Disable interrupts, clear pending interrupts, and reset the interrupt
   * state
   */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_CPUCTL, USBHOST_CPUCTL_IE, 0);
  max3421e_putreg(priv, MAX3421E_USBHOST_HIEN, 0);
  max3421e_putreg(priv, MAX3421E_USBHOST_HIRQ, 0xff);

  priv->irqset  = 0;
  priv->ackstat = MAX3421E_ACKSTAT_FALSE;

  /* Configure full duplex SPI, edge-active rising edge interrupt */

  max3421e_putreg(priv, MAX3421E_USBHOST_PINCTL,
                  USBHOST_PINCTL_FDUPSPI | USBHOST_PINCTL_POSINT);

  /* Configure as full-speed USB host */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_MODE,
                     USBHOST_MODE_SPEED | USBHOST_MODE_SOFKAENAB,
                     USBHOST_MODE_HOST | USBHOST_MODE_DMPULLD |
                     USBHOST_MODE_DPPULLDN);

  /* Enable and clear the connection detected (CONDIRQ) interrupt */

  max3421e_int_enable(priv, USBHOST_HIRQ_CONNIRQ);
  max3421e_putreg(priv, MAX3421E_USBHOST_HIRQ, USBHOST_HIRQ_CONNIRQ);

  /* Enable MAX3412E interrupts */

  max3421e_modifyreg(priv, MAX3421E_USBHOST_CPUCTL, 0, USBHOST_CPUCTL_IE);

  usbhost_trace1(MAX3421E_TRACE1_INITIALIZED, 0);

  revision = max3421e_getreg(priv, MAX3421E_USBHOST_REVISION);
  if (revision != USBHOST_REVISION)
    {
      usbhost_trace1(MAX3421E_TRACE1_BADREVISION, revision);
      max3421e_unlock(priv);
      return -ENODEV;
    }

  /* Perform a bus reset to reconnect after a power down */

  ret = max3421e_connected(priv);
  if (ret < 0)
    {
      /* Nothing connected. */

      max3421e_disconnected(priv);
    }

  max3421e_unlock(priv);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max3421e_usbhost_initialize
 *
 * Description:
 *   Initialize MAX3421E as USB host controller.
 *
 * Input Parameters:
 *   lower - The interface to the lower half driver
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

FAR struct usbhost_connection_s *
max3421e_usbhost_initialize(FAR const struct max3421e_lowerhalf_s *lower)
{
  FAR struct usbhost_alloc_s *alloc;
  FAR struct max3421e_usbhost_s *priv;
  FAR struct usbhost_connection_s conn *conn;
  int ret;

  DEBUGASSERT(lower != NULL && lower->spi != NULL && lower->attach != NULL &&
              lower->attach != NULL && lower->acknowledge != NULL);

  /* Allocate and instance of the MAX4321E state structure */

  alloc = (FAR struct usbhost_alloc_s *)
    kmm_malloc(sizeof(struct usbhost_alloc_s));

  if (alloc < 0)
    {
      usberr("ERROR: Failed to allocate the state structure\n");
      return NULL;
    }

  priv = &alloc->priv;
  conn = &alloc->conn;

  /* Initialize the state of the host driver */

  ret = max3421e_sw_initialize(priv, conn, lower);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  /* Initialize the MAX3421E, putting it into full operational state. */

  ret = max3421e_hw_initialize(priv);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(MAX3421E_IRQ_OTGFS);
  return &conn->conn;

errout_with_alloc;
  kmm_free(alloc);
  return NULL;
}

#endif /* CONFIG_USBHOST_MAX3421E */
