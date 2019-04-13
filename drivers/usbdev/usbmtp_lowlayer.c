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
 *   Wait for a SCSI worker thread event.
 *
 ****************************************************************************/

static void usbmtp_lowlayer_wait(FAR struct usbmtp_dev_s *priv)
{
  irqstate_t flags;
  int ret;

  uinfo("Enter!\n");

  /* We must hold the SCSI lock to call this function */

  DEBUGASSERT(priv->thlock.semcount < 1);

  /* A flag is used to prevent driving up the semaphore count.  This function
   * is called (primarily) from the SCSI work thread so we must disable
   * interrupts momentarily to assure that test of the flag and the wait fo
   * the semaphore count are atomic.  Interrupts will, of course, be re-
   * enabled while we wait for the event.
   */

  flags = enter_critical_section();
  priv->thwaiting = true;

  /* Relinquish our lock on the SCSI state data */

  usbmtp_lowlayer_unlock(priv);

  /* Now wait for a SCSI event to be signalled */

  do
    {
      ret = sem_wait(&priv->thwaitsem);
      DEBUGASSERT(ret == OK || errno == EINTR);
      UNUSED(ret); /* Eliminate warnings when debug is off */
    }
  while (priv->thwaiting);

  /* Re-acquire our lock on the SCSI state data */

  usbmtp_lowlayer_lock(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbmtp_cmdtestunitready
 *
 * Description:
 *  Handle the SCSI_CMD_TESTUNITREADY command
 *
 ****************************************************************************/

static inline int usbmtp_cmdtestunitready(FAR struct usbmtp_dev_s *priv)
{
  int ret;

  uinfo("Enter!\n");

  priv->u.alloclen = 0;
  ret = usbmtp_setupcmd(priv, 6, USBMTP_FLAGS_DIRNONE);
  return ret;
}

/****************************************************************************
 * Name: usbmtp_setupcmd
 *
 * Description:
 *   Called after each SCSI command is identified in order to perform setup
 *   and verification operations that are common to all SCSI commands.  This
 *   function performs the following common setup operations:
 *
 *     1. Determine the direction of the response
 *     2. Verify lengths
 *     3. Setup and verify the LUN
 *
 *   Includes special logic for INQUIRY and REQUESTSENSE commands
 *
 ****************************************************************************/

static int inline usbmtp_setupcmd(FAR struct usbmtp_dev_s *priv,
                                  uint8_t cdblen, uint8_t flags)
{
  FAR struct usbmtp_lun_s *lun = NULL;
  uint32_t datlen;
  uint8_t dir;
  int ret = OK;

  uinfo("Enter!\n");

  /* Verify the LUN and set up the current LUN reference in the
   * device structure
   */

  if (priv->cbwlun < priv->nluns)
    {
      /* LUN number is valid in a valid range, but the LUN is not necessarily
       * bound to a block driver.  That will be checked as necessary in each
       * individual command.
       */

      lun       = &priv->luntab[priv->cbwlun];
      priv->lun = lun;
    }

  /* Only a few commands may specify unsupported LUNs */

  else if ((flags & USBMTP_FLAGS_LUNNOTNEEDED) == 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDBADLUN), priv->cbwlun);
      ret = -EINVAL;
    }

  /* Extract the direction and data transfer length */

  dir    = flags & USBMTP_FLAGS_DIRMASK;      /* Expected data direction */
  datlen = 0;
  if ((flags & USBMTP_FLAGS_BLOCKXFR) == 0)
    {
      /* Non-block transfer.  Data length: Host allocation to receive data
       * (only for device-to-host transfers.  At present, alloclen is set
       * to zero for all host-to-device, non-block transfers.
       */

      datlen = priv->u.alloclen;
    }
  else if (lun)
    {
      /* Block transfer: Calculate the total size of all sectors to be transferred */

      datlen = priv->u.alloclen * lun->sectorsize;
    }

  /* Check the data direction.  This was set up at the end of the
   * IDLE state when the CBW was parsed, but if there is no data,
   * then the direction is none.
   */

  if (datlen == 0)
    {
      /* No data.. then direction is none */

      dir = USBMTP_FLAGS_DIRNONE;
    }

  /* Compare the expected data length in the command to the data length
   * in the CBW.
   */

  else if (priv->cbwlen < datlen)
    {
      /* Clip to the length in the CBW and declare a phase error */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PHASEERROR1), priv->cdb[0]);
      if ((flags & USBMTP_FLAGS_BLOCKXFR) != 0)
        {
          priv->u.alloclen = priv->cbwlen;
        }
      else if (lun)
        {
          priv->u.xfrlen = priv->cbwlen / lun->sectorsize;
        }

      priv->phaseerror = 1;
    }

  /* Initialize the residue */

  priv->residue = priv->cbwlen;

  /* Conflicting data directions is a phase error */

  if (priv->cbwdir != dir && datlen > 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PHASEERROR2), priv->cdb[0]);
      priv->phaseerror = 1;
      ret             = -EINVAL;
    }

  /* Compare the length of data in the cdb[] with the expected length
   * of the command.  These sizes should match exactly.
   */

  if (cdblen != priv->cdblen)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PHASEERROR3), priv->cdb[0]);
      priv->phaseerror = 1;
      ret              = -EINVAL;
    }

  /* Was a valid LUN provided? */

  if (lun)
    {
      /* Retain the sense data for the REQUEST SENSE command */

      if ((flags & USBMTP_FLAGS_RETAINSENSEDATA) == 0)
        {
          /* Discard the sense data */

          lun->sd     = SCSI_KCQ_NOSENSE;
          lun->sdinfo = 0;
        }

      /* If a unit attention condition exists, then only a restricted set of
       * commands is permitted.
       */

      if (lun->uad != SCSI_KCQ_NOSENSE && (flags & USBMTP_FLAGS_UACOKAY) != 0)
        {
          /* Command not permitted */

          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDUNEVIOLATION),
                   priv->cbwlun);
          lun->sd  = lun->uad;
          lun->uad = SCSI_KCQ_NOSENSE;
          ret      = -EINVAL;
        }
    }

  /* The final, 1-byte field of every SCSI command is the Control field which
   * must be zero
   */

  if (priv->cdb[cdblen - 1] != 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SCSICMDCONTROL), 0);
      if (lun)
        {
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
        }

      ret = -EINVAL;
    }

  return ret;
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
  uint16_t mtpcmd;
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

  /* Get the current command */

  mtpcmd = buffer[7] << 8 | buffer[6];

  usbmtp_dumpdata("MTP CMD", &mtpcmd, 2);

  usbmtp_lowlayer_lock(priv);
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
          priv->lun->sd = SCSI_KCQIR_INVALIDCOMMAND;
          ret = -EINVAL;
        }

      break;
    }

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
 *   containing SCSI commands.  This function processes those SCSI commands.
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
      /* All commands come through this path (EXCEPT read6/10/12 and
       * write6/10/12).  For all other commands, the following setup is
       * expected for the response based on data direction:
       *
       * For direction NONE:
       *   1. priv->u.alloclen == 0
       *   2. priv->nreqbytes == 0
       *
       * For direction = device-to-host:
       *   1. priv->u.alloclen == allocation length; space set aside by the
       *      host to receive the device data.  The size of the response
       *      cannot exceed this value.
       *   2. response data is in the request currently at the head of
       *      the priv->wrreqlist queue.  priv->nreqbytes is set to the length
       *      of data in the response.
       *
       * For direction host-to-device
       *   At present, there are no supported commands that should have
       *   host-to-device transfers (except write6/10/12 and that command
       *   logic does not take this path.  The 'residue' is left at the full
       *   host-to-device data size.
       *
       * For all:
       *   ret set to <0 if an error occurred in parsing the commands.
       */

      /* For from device-to-hose operations, the residue is the expected size
       * (the initial value of 'residue') minus the amount actually returned
       * in the response:
       */

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
 *   state.  Otherwise, when the SCSI statis is successfully returned, this
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

  /* Extract the sense data from the LUN structure */

  if (lun)
    {
      sd = lun->sd;
    }
  else
    {
      sd = SCSI_KCQIR_INVALIDLUN;
    }

  /* Determine the CSW status: PASS, PHASEERROR, or FAIL */

  if (priv->phaseerror)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SNDPHERROR), 0);
      status = USBMSC_CSWSTATUS_PHASEERROR;
      sd     = SCSI_KCQIR_INVALIDCOMMAND;
    }
  else if (sd != SCSI_KCQ_NOSENSE)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SNDCSWFAIL), 0);
      status = USBMSC_CSWSTATUS_FAIL;
    }

  /* Format and submit the CSW */

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

  /* Get the SCSI state data handed off from the initialization logic */

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

      /* Loop processing each SCSI command state.  Each state handling
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
 *   Signal the SCSI worker thread that SCSI events need service.
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
 *   Get exclusive access to SCSI state data.
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
