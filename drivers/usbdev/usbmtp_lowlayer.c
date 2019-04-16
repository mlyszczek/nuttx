/****************************************************************************
 * drivers/usbdev/usbmtp_lowlayer.c
 *
 *   Copyright (C) 2008-2010, 2012, 2016, 2019 Gregory Nutt. All rights
 *     reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/arch.h>
#include <nuttx/scsi.h>
#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/mtp/mtp.h>

#include "usbmtp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Race condition workaround found by David Hewson.  This race condition:
 *
 * "seems to relate to stalling the endpoint when a short response is
 *  generated which causes a residue to exist when the CSW would be returned.
 *  I think there's two issues here.  The first being if the transfer which
 *  had just been queued before the stall had not completed then it wouldn't
 *  then complete once the endpoint was stalled?  The second is that the
 *  subsequent transfer for the CSW would be dropped on the floor (by the
 *  epsubmit() function) if the end point was still stalled as the control
 *  transfer to resume it hadn't occurred."
 *
 * If queuing of stall requests is supported by DCD then this workaround is
 * not required.  In this case, (1) the stall is not sent until all write
 * requests preceding the stall request are sent, (2) the stall is sent,
 * and then after the stall is cleared, (3) all write requests queued after
 * the stall are sent.
 */

#ifndef CONFIG_ARCH_USBDEV_STALLQUEUE
#  define USBMTP_STALL_RACEWAR 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Debug ********************************************************************/

#if defined(CONFIG_DEBUG_INFO) && defined (CONFIG_DEBUG_USB)
static void     usbmtp_dumpdata(const char *msg, const uint8_t *buf,
                  int buflen);
#else
#  define usbmtp_dumpdata(msg, buf, len)
#endif

/* Utility Support Functions ************************************************/

static uint16_t usbmtp_getbe16(uint8_t *buf);
static uint32_t usbmtp_getbe32(uint8_t *buf);
static void     usbmtp_putbe16(uint8_t * buf, uint16_t val);
static void     usbmtp_putbe24(uint8_t *buf, uint32_t val);
static void     usbmtp_putbe32(uint8_t *buf, uint32_t val);
#if 0 /* not used */
static uint16_t usbmtp_getle16(uint8_t *buf);
#endif
static uint32_t usbmtp_getle32(uint8_t *buf);
#if 0 /* not used */
static void     usbmtp_putle16(uint8_t * buf, uint16_t val);
#endif
static void     usbmtp_putle32(uint8_t *buf, uint32_t val);

/* MTP Command Processing ***************************************************/

static inline int usbmtp_cmdtestunitready(FAR struct usbmtp_dev_s *priv);
static inline int usbmtp_setupcmd(FAR struct usbmtp_dev_s *priv,
                uint8_t cdblen, uint8_t flags);

/* MTP Worker Thread ********************************************************/

static int    usbmtp_idlestate(FAR struct usbmtp_dev_s *priv);
static int    usbmtp_cmdparsestate(FAR struct usbmtp_dev_s *priv);
static int    usbmtp_cmdstatusstate(FAR struct usbmtp_dev_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmtp_dumpdata
 ****************************************************************************/

#if defined(CONFIG_DEBUG_INFO) && defined (CONFIG_DEBUG_USB)
static void usbmtp_dumpdata(const char *msg, const uint8_t *buf, int buflen)
{
  int i;

  uinfo("Enter!\n");

  syslog(LOG_DEBUG, "%s:", msg);
  for (i = 0; i < buflen; i++)
    {
      syslog(LOG_DEBUG, " %02x", buf[i]);
    }

  syslog(LOG_DEBUG, "\n");
}
#endif

/****************************************************************************
 * Name: usbmtp_getbe16
 *
 * Description:
 *   Get a 16-bit big-endian value reference by the byte pointer
 *
 ****************************************************************************/

static uint16_t usbmtp_getbe16(uint8_t *buf)
{
  uinfo("Enter!\n");
  return ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
}

/****************************************************************************
 * Name: usbmtp_getbe32
 *
 * Description:
 *   Get a 32-bit big-endian value reference by the byte pointer
 *
 ****************************************************************************/

static uint32_t usbmtp_getbe32(uint8_t *buf)
{
  uinfo("Enter!\n");
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);
}

/****************************************************************************
 * Name: usbmtp_putbe16
 *
 * Description:
 *   Store a 16-bit value in big-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmtp_putbe16(uint8_t * buf, uint16_t val)
{
  uinfo("Enter!\n");
  buf[0] = val >> 8;
  buf[1] = val;
}

/****************************************************************************
 * Name: usbmtp_putbe24
 *
 * Description:
 *   Store a 32-bit value in big-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmtp_putbe24(uint8_t *buf, uint32_t val)
{
  uinfo("Enter!\n");
  buf[0] = val >> 16;
  buf[1] = val >> 8;
  buf[2] = val;
}

/****************************************************************************
 * Name: usbmtp_putbe32
 *
 * Description:
 *   Store a 32-bit value in big-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmtp_putbe32(uint8_t *buf, uint32_t val)
{
  uinfo("Enter!\n");
  buf[0] = val >> 24;
  buf[1] = val >> 16;
  buf[2] = val >> 8;
  buf[3] = val;
}

/****************************************************************************
 * Name: usbmtp_getle16
 *
 * Description:
 *   Get a 16-bit little-endian value reference by the byte pointer
 *
 ****************************************************************************/

#if 0 /* not used */
static uint16_t usbmtp_getle16(uint8_t *buf)
{
  return ((uint16_t)buf[1] << 8) | ((uint16_t)buf[0]);
}
#endif

/****************************************************************************
 * Name: usbmtp_getle32
 *
 * Description:
 *   Get a 32-bit little-endian value reference by the byte pointer
 *
 ****************************************************************************/

static uint32_t usbmtp_getle32(uint8_t *buf)
{
  uinfo("Enter!\n");
  return ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[1] << 8) | ((uint32_t)buf[0]);
}

/****************************************************************************
 * Name: usbmtp_putle16
 *
 * Description:
 *   Store a 16-bit value in little-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

#if 0 /* not used */
static void usbmtp_putle16(uint8_t * buf, uint16_t val)
{
  buf[0] = val;
  buf[1] = val >> 8;
}
#endif

/****************************************************************************
 * Name: usbmtp_putle32
 *
 * Description:
 *   Store a 32-bit value in little-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmtp_putle32(uint8_t *buf, uint32_t val)
{
  uinfo("Enter!\n");
  buf[0] = val;
  buf[1] = val >> 8;
  buf[2] = val >> 16;
  buf[3] = val >> 24;
}

/****************************************************************************
 * Name: usbmtp_lowlayer_wait
 *
 * Description:
 *   Wait for a MTP worker thread event.
 *
 ****************************************************************************/

static void usbmtp_lowlayer_wait(FAR struct usbmtp_dev_s *priv)
{
  irqstate_t flags;
  int ret;

  uinfo("Enter!\n");

  /* We must hold the MTP lock to call this function */

  DEBUGASSERT(priv->thlock.semcount < 1);

  /* A flag is used to prevent driving up the semaphore count.  This function
   * is called (primarily) from the MTP work thread so we must disable
   * interrupts momentarily to assure that test of the flag and the wait for
   * the semaphore count are atomic.  Interrupts will, of course, be re-
   * enabled while we wait for the event.
   */

  flags = enter_critical_section();
  priv->thwaiting = true;

  /* Relinquish our lock on the MTP state data */

  usbmtp_lowlayer_unlock(priv);

  /* Now wait for a MTP event to be signalled */

  do
    {
      ret = sem_wait(&priv->thwaitsem);
      DEBUGASSERT(ret == OK || errno == EINTR);
      UNUSED(ret); /* Eliminate warnings when debug is off */
    }
  while (priv->thwaiting);

  /* Re-acquire our lock on the MTP state data */

  usbmtp_lowlayer_lock(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbmtp_idlestate
 *
 * Description:
 *   Called from the worker thread in the USBMTP_STATE_IDLE state.  Checks
 *   for the receipt of a bulk CBW.
 *
 * Returned value:
 *   If no new, valid CBW is available, this function returns a negated errno.
 *   Otherwise, when a new CBW is successfully parsed, this function sets
 *   priv->thstate to USBMTP_STATE_CMDPARSE and returns OK.
 *
 ****************************************************************************/

static int usbmtp_idlestate(FAR struct usbmtp_dev_s *priv)
{
  FAR struct usbmtp_req_s *privreq;
  FAR struct usbdev_req_s *req;
  uint8_t *buffer;
  //uint16_t mtpcmd;
  irqstate_t flags;
  int ret = -EINVAL;

  uinfo("Enter!\n");

  /* Take a request from the rdreqlist */

  flags = enter_critical_section();
  privreq = (FAR struct usbmtp_req_s *)sq_remfirst(&priv->rdreqlist);
  leave_critical_section(flags);

  /* Has anything been received? If not, just return an error.
   * This will cause us to remain in the IDLE state.  When a USB request is
   * received, the worker thread will be awakened in the USBMTP_STATE_IDLE
   * and we will be called again.
   */

  if (!privreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_IDLERDREQLISTEMPTY), 0);
      return -ENOMEM;
    }

  req = privreq->req;
  buffer = (uint8_t *)req->buf;

  /* Handle the CBW */

  usbmtp_dumpdata("MTP-", (FAR uint8_t *)buffer,
                  USBMSC_CBW_SIZEOF - USBMSC_MAXCDBLEN + 1);

  usbmtp_lowlayer_lock(priv);

  /* Process received request */

  mtp_process_request((FAR uint8_t *)buffer);

#if 0
  /* Get the current command */

  mtpcmd = buffer[7] << 8 | buffer[6];

  usbmtp_dumpdata("MTP CMD", &mtpcmd, 2);

  switch (mtpcmd)
    {
    case MTP_OPEN_SESSION:                  /* MTP OpenSession */
      syslog(LOG_DEBUG, "Open Session Command!\n");
      ret = usbmtp_cmdtestunitready(priv);
      break;

    case MTP_GET_DEV_INFO:                  /* MTP GetDeviceInfo */
      syslog(LOG_DEBUG, "GetDeviceInfo Command!\n");
      ret = usbmtp_cmdtestunitready(priv);
      break;

    default:
      priv->u.alloclen = 0;
      if (ret == OK)
        {
          ret = -EINVAL;
        }

      break;
    }
#endif //if 0

  usbmtp_lowlayer_unlock(priv);

  priv->thstate = USBMTP_STATE_CMDPARSE;
  ret = OK;

  /* In any event, return the request to be refilled */

  if (EP_SUBMIT(priv->epbulkout, req) != OK)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_IDLERDSUBMIT), (uint16_t)-ret);
    }

  return ret;
}

/****************************************************************************
 * Name: usbmtp_cmdparsestate
 *
 * Description:
 *   Called from the worker thread in the USBMTP_STATE_CMDPARSE state.
 *   This state is entered when usbmtp_idlestate obtains a valid CBW
 *   containing MTP commands.  This function processes those MTP commands.
 *
 * Returned value:
 *   If no write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMTP_STATE_CMDPARSE
 *   state.  Otherwise, when the new CBW is successfully process, this
 *   function sets priv->thstate to one of USBMTP_STATE_CMDREAD,
 *   USBMTP_STATE_CMDWRITE, or USBMTP_STATE_CMDFINISH and returns OK.
 *
 ****************************************************************************/

static int usbmtp_cmdparsestate(FAR struct usbmtp_dev_s *priv)
{
  FAR struct usbmtp_req_s *privreq;
  int ret = -EINVAL;

  uinfo("Enter!\n");

  /* Is a response required? */

  if (priv->thstate == USBMTP_STATE_CMDPARSE)
    {
      if (priv->cbwdir == USBMTP_FLAGS_DIRDEVICE2HOST)
        {
          /* The number of bytes in the response cannot exceed the host
           * 'allocation length' in the command.
           */

          if (priv->nreqbytes > priv->u.alloclen)
            {
              priv->nreqbytes = priv->u.alloclen;
            }

          /* The residue is then the number of bytes that were not sent */

          priv->residue -= priv->nreqbytes;
        }

      /* On return, we need the following:
       *
       *   1. Setup for CMDFINISH state if appropriate
       *   2. priv->thstate set to either CMDPARSE if no buffer was
       *      available or CMDFINISH to send the response
       *   3. Return OK to continue; <0 to wait for the next event
       */

      usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDFINISH),
               priv->cdb[0]);

      priv->thstate = USBMTP_STATE_CMDSTATUS;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usbmtp_cmdstatusstate
 *
 * Description:
 *   Called from the worker thread in the USBMTP_STATE_CMDSTATUS state.
 *   That state is after a CBW has been fully processed.  This function sends
 *   the concluding CSW.
 *
 * Returned value:
 *   If no write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMTP_STATE_CMDSTATUS
 *   state.  Otherwise, when the MTP statis is successfully returned, this
 *   function sets priv->thstate to USBMTP_STATE_IDLE and returns OK.
 *
 ****************************************************************************/

static int usbmtp_cmdstatusstate(FAR struct usbmtp_dev_s *priv)
{
  FAR struct usbmtp_lun_s *lun = priv->lun;
  FAR struct usbmtp_req_s *privreq;
  FAR struct usbdev_req_s *req;
  FAR struct usbmsc_csw_s *csw;
  irqstate_t flags;
  uint32_t sd;
  uint8_t status = USBMSC_CSWSTATUS_PASS;
  uint8_t *buffer;
  int ret;

  uinfo("Enter!\n");

  /* Take a request from the wrreqlist */

  flags = enter_critical_section();
  privreq = (FAR struct usbmtp_req_s *)sq_remfirst(&priv->wrreqlist);
  leave_critical_section(flags);

  /* If there no request structures available, then just return an error.
   * This will cause us to remain in the CMDSTATUS status.  When a request
   * is returned, the worker thread will be awakened in the
   * USBMTP_STATE_CMDSTATUS and we will be called again.
   */

  if (!privreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDSTATUSWRREQLISTEMPTY), 0);
      return -ENOMEM;
    }

  req    = privreq->req;
  csw    = (FAR struct usbmsc_cbw_s *)req->buf;
  buffer = (FAR uint8_t *)req->buf;

  /* Format and submit the response */

  csw->signature[0] = 'U';
  csw->signature[1] = 'S';
  csw->signature[2] = 'B';
  csw->signature[3] = 'S';
  usbmtp_putle32(csw->tag, priv->cbwtag);
  usbmtp_putle32(csw->residue, priv->residue);
  csw->status       = status;

  buffer[0] = 0x0c;
  buffer[1] = 0x00;
  buffer[2] = 0x00;
  buffer[3] = 0x00;
  buffer[4] = 0x03;
  buffer[5] = 0x00;
  buffer[6] = 0x01;
  buffer[7] = 0x20;
  buffer[8] = 0x00;
  buffer[9] = 0x00;
  buffer[10] = 0x00;
  buffer[11] = 0x00;

  usbmtp_dumpdata("SCSCI CSW", (FAR uint8_t *)csw,
                  12 /* USBMSC_CSW_SIZEOF */);

  req->len       = 12; /* USBMSC_CSW_SIZEOF */
  req->callback  = usbmtp_wrcomplete;
  req->priv      = privreq;
  req->flags     = USBDEV_REQFLAGS_NULLPKT;

  ret            = EP_SUBMIT(priv->epbulkin, req);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SNDSTATUSSUBMIT),
               (uint16_t)-ret);

      flags = enter_critical_section();
      (void)sq_addlast((FAR sq_entry_t *)privreq, &priv->wrreqlist);
      leave_critical_section(flags);
    }

  /* Return to the IDLE state */

  usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDSTATUSIDLE), 0);
  priv->thstate = USBMTP_STATE_IDLE;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmtp_lowlayer_main
 *
 * Description:
 *   This is the main function of the USB storage worker thread.  It loops
 *   until USB-related events occur, then processes those events accordingly
 *
 ****************************************************************************/

int usbmtp_lowlayer_main(int argc, char *argv[])
{
  FAR struct usbmtp_dev_s *priv;
  irqstate_t flags;
  uint16_t eventset;
  int ret;

  uinfo("Started\n");

  /* Get the MTP state data handed off from the initialization logic */

  priv = g_usbmtp_handoff;
  DEBUGASSERT(priv);

  g_usbmtp_handoff = NULL;
  usbmtp_synch_signal(priv);

  /* This thread is started before the USB storage class is fully initialized.
   * wait here until we are told to begin.  Start in the NOTINITIALIZED state
   */

  uinfo("Waiting to be signalled\n");

  usbmtp_lowlayer_lock(priv);
  priv->thstate = USBMTP_STATE_STARTED;
  while ((priv->theventset & USBMTP_EVENT_READY) == 0 &&
         (priv->theventset & USBMTP_EVENT_TERMINATEREQUEST) == 0)
    {
      usbmtp_lowlayer_wait(priv);
    }

  uinfo("Running\n");

  /* Transition to the INITIALIZED/IDLE state */

  priv->thstate    = USBMTP_STATE_IDLE;
  eventset         = priv->theventset;
  priv->theventset = USBMTP_EVENT_NOEVENTS;
  usbmtp_lowlayer_unlock(priv);

  /* Then loop until we are asked to terminate */

  while ((eventset & USBMTP_EVENT_TERMINATEREQUEST) == 0)
    {
      /* Wait for some interesting event.  Note that we must both take the
       * lock (to eliminate race conditions with other threads) and disable
       * interrupts (to eliminate race conditions with USB interrupt
       * handling.
       */

      usbmtp_lowlayer_lock(priv);
      flags = enter_critical_section();
      if (priv->theventset == USBMTP_EVENT_NOEVENTS)
        {
          usbmtp_lowlayer_wait(priv);
        }

      /* Sample any events before re-enabling interrupts.  Any events that
       * occur after re-enabling interrupts will have to be handled in the
       * next time through the loop.
       */

      eventset         = priv->theventset;
      priv->theventset = USBMTP_EVENT_NOEVENTS;
      usbmtp_lowlayer_unlock(priv);

      /* Were we awakened by some event that requires immediate action?
       *
       * - The USBMTP_EVENT_DISCONNECT is signalled from the disconnect
       *   method after all transfers have been stopped, when the host is
       *   disconnected.
       *
       * - The CUSBMTP_EVENT_RESET is signalled when the bulk-storage-
       *   specific USBMTP_REQ_MSRESET EP0 setup received.  We must stop
       *   the current operation and reinialize state.
       *
       * - The USBMTP_EVENT_CFGCHANGE is signaled when the EP0 setup logic
       *   receives a valid USB_REQ_SETCONFIGURATION request
       *
       * - The USBMTP_EVENT_IFCHANGE is signaled when the EP0 setup logic
       *   receives a valid USB_REQ_SETINTERFACE request
       *
       * - The USBMTP_EVENT_ABORTBULKOUT event is signalled by the CMDFINISH
       *   logic when there is a residue after processing a host-to-device
       *   transfer.  We need to discard all incoming request.
       *
       * All other events are just wakeup calls and are intended only
       * drive the state machine.
       */

      if ((eventset & (USBMTP_EVENT_DISCONNECT | USBMTP_EVENT_RESET |
                       USBMTP_EVENT_CFGCHANGE | USBMTP_EVENT_IFCHANGE |
                       USBMTP_EVENT_ABORTBULKOUT)) != 0)
        {
          /* These events require that the current configuration be reset */

          if ((eventset & USBMTP_EVENT_IFCHANGE) != 0)
            {
              usbmtp_resetconfig(priv);
            }

          /* These events require that a new configuration be established */

          if ((eventset & (USBMTP_EVENT_CFGCHANGE)) != 0)
            {
              usbmtp_setconfig(priv, priv->thvalue);
            }

          /* These events required that we send a deferred EP0 setup response */

          if ((eventset & (USBMTP_EVENT_RESET | USBMTP_EVENT_CFGCHANGE |
                           USBMTP_EVENT_IFCHANGE)) != 0)
            {
              usbmtp_deferredresponse(priv, false);
            }

          /* For all of these events... terminate any transactions in progress */

          priv->thstate = USBMTP_STATE_IDLE;
        }

      leave_critical_section(flags);

      /* Loop processing each MTP command state.  Each state handling
       * function will do the following:
       *
       * - If it must block for an event, it will return a negated errno value
       * - If it completes the processing for that state, it will (1) set
       *   the next appropriate state value and (2) return OK.
       *
       * So the following will loop until we must block for an event in
       * a particular state.  When we are awakened by an event (above) we
       * will resume processing in the same state.
       */

      do
        {
          switch (priv->thstate)
            {
            case USBMTP_STATE_IDLE:             /* Started and waiting for commands */
               ret = usbmtp_idlestate(priv);
               break;

            case USBMTP_STATE_CMDPARSE:         /* Parsing the received a command */
               ret = usbmtp_cmdparsestate(priv);
               break;

            case USBMTP_STATE_CMDSTATUS:        /* Processing the status phase of a command */
              ret = usbmtp_cmdstatusstate(priv);
              break;

            default:
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INVALIDSTATE),
                       priv->thstate);

              priv->thstate = USBMTP_STATE_IDLE;
              ret           = OK;
              break;
            }
        }
      while (ret == OK);
    }

  /* Transition to the TERMINATED state and exit */

  priv->thstate = USBMTP_STATE_TERMINATED;
  usbmtp_synch_signal(priv);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: usbmtp_lowlayer_signal
 *
 * Description:
 *   Signal the MTP worker thread that MTP events need service.
 *
 ****************************************************************************/

void usbmtp_lowlayer_signal(FAR struct usbmtp_dev_s *priv)
{
  irqstate_t flags;

  uinfo("Enter!\n");

  /* A flag is used to prevent driving up the semaphore count.  This function
   * is called (primarily) from interrupt level logic so we must disable
   * interrupts momentarily to assure that test of the flag and the increment
   * of the semaphore count are atomic.
   */

  flags = enter_critical_section();
  if (priv->thwaiting)
    {
      priv->thwaiting = false;
      sem_post(&priv->thwaitsem);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbmtp_lowlayer_lock
 *
 * Description:
 *   Get exclusive access to MTP state data.
 *
 ****************************************************************************/

void usbmtp_lowlayer_lock(FAR struct usbmtp_dev_s *priv)
{
  int ret;

  uinfo("Enter!\n");

  do
    {
      ret = sem_wait(&priv->thlock);
      DEBUGASSERT(ret == OK || errno == EINTR);
    }
  while (ret < 0);
}
