/****************************************************************************
 * drivers/usbdev/usbmtp.h
 *
 *   Copyright (C) 2008-2013, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Mass storage class device.  Bulk-only with SCSI subclass.
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

#ifndef __DRIVERS_USBDEV_USBMTP_H
#define __DRIVERS_USBDEV_USBMTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <queue.h>
#include <semaphore.h>

#include <nuttx/fs/fs.h>
#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbmtp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* If the USB mass storage device is configured as part of a composite device
 * then both CONFIG_USBDEV_COMPOSITE and CONFIG_USBMTP_COMPOSITE must be
 * defined.
 */

#ifndef CONFIG_USBDEV_COMPOSITE
#  undef CONFIG_USBMTP_COMPOSITE
#endif

#if defined(CONFIG_USBMTP_COMPOSITE) && !defined(CONFIG_USBMTP_STRBASE)
#  define CONFIG_USBMTP_STRBASE (4)
#endif

/* Interface IDs.  If the mass storage driver is built as a component of a
 * composite device, then the interface IDs may need to be offset.
 */

#ifndef CONFIG_USBMTP_COMPOSITE
#  undef CONFIG_USBMTP_IFNOBASE
#  define CONFIG_USBMTP_IFNOBASE 0
#endif

#ifndef CONFIG_USBMTP_IFNOBASE
#  define CONFIG_USBMTP_IFNOBASE 0
#endif

/* Number of requests in the write queue */

#ifndef CONFIG_USBMTP_NWRREQS
#  define CONFIG_USBMTP_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_USBMTP_NRDREQS
#  define CONFIG_USBMTP_NRDREQS 4
#endif

/* Logical endpoint numbers / max packet sizes */

#ifndef CONFIG_USBMTP_COMPOSITE
#  ifndef CONFIG_USBMTP_EPINTIN
#    warning "EPINTIN not defined in the configuration"
#    define CONFIG_USBMTP_EPINTIN 1
#  endif
#endif

#ifndef CONFIG_USBMTP_COMPOSITE
#  ifndef CONFIG_USBMTP_EPBULKOUT
#    warning "EPBULKOUT not defined in the configuration"
#    define CONFIG_USBMTP_EPBULKOUT 2
#  endif
#endif

#ifndef CONFIG_USBMTP_COMPOSITE
#  ifndef CONFIG_USBMTP_EPBULKIN
#    warning "EPBULKIN not defined in the configuration"
#    define CONFIG_USBMTP_EPBULKIN 3
#  endif
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_USBMTP_COMPOSITE
#  ifndef CONFIG_USBMTP_EP0MAXPACKET
#    define CONFIG_USBMTP_EP0MAXPACKET 64
#  endif
#endif

#ifndef CONFIG_USBMTP_BULKINREQLEN
#  ifdef CONFIG_USBDEV_DUALSPEED
#    define CONFIG_USBMTP_BULKINREQLEN 512
#  else
#    define CONFIG_USBMTP_BULKINREQLEN 64
#  endif
#else
#  ifdef CONFIG_USBDEV_DUALSPEED
#    if CONFIG_USBMTP_BULKINREQLEN < 512
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMTP_BULKINREQLEN
#      define CONFIG_USBMTP_BULKINREQLEN 512
#    endif
#  else
#    if CONFIG_USBMTP_BULKINREQLEN < 64
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMTP_BULKINREQLEN
#      define CONFIG_USBMTP_BULKINREQLEN 64
#    endif
#  endif
#endif

#ifndef CONFIG_USBMTP_BULKOUTREQLEN
#  ifdef CONFIG_USBDEV_DUALSPEED
#    define CONFIG_USBMTP_BULKOUTREQLEN 512
#  else
#    define CONFIG_USBMTP_BULKOUTREQLEN 64
#  endif
#else
#  ifdef CONFIG_USBDEV_DUALSPEED
#    if CONFIG_USBMTP_BULKOUTREQLEN < 512
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMTP_BULKOUTREQLEN
#      define CONFIG_USBMTP_BULKOUTREQLEN 512
#    endif
#  else
#    if CONFIG_USBMTP_BULKOUTREQLEN < 64
#      warning "Bulk in buffer size smaller than max packet"
#      undef CONFIG_USBMTP_BULKOUTREQLEN
#      define CONFIG_USBMTP_BULKOUTREQLEN 64
#    endif
#  endif
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_USBMTP_COMPOSITE
#  ifndef CONFIG_USBMTP_VENDORID
#    warning "CONFIG_USBMTP_VENDORID not defined"
#    define CONFIG_USBMTP_VENDORID 0x584e
#  endif

#  ifndef CONFIG_USBMTP_PRODUCTID
#    warning "CONFIG_USBMTP_PRODUCTID not defined"
#    define CONFIG_USBMTP_PRODUCTID 0x5342
#  endif

#  ifndef CONFIG_USBMTP_VERSIONNO
#    define CONFIG_USBMTP_VERSIONNO (0x0399)
#  endif

#  ifndef CONFIG_USBMTP_VENDORSTR
#    warning "No Vendor string specified"
#    define CONFIG_USBMTP_VENDORSTR  "NuttX"
# endif

#  ifndef CONFIG_USBMTP_PRODUCTSTR
#    warning "No Product string specified"
#    define CONFIG_USBMTP_PRODUCTSTR "USBdev Storage"
#  endif

#  undef CONFIG_USBMTP_SERIALSTR
#  define CONFIG_USBMTP_SERIALSTR "0101"
#endif

#undef CONFIG_USBMTP_CONFIGSTR
#define CONFIG_USBMTP_CONFIGSTR "Bulk"

/* SCSI daemon */

#ifndef CONFIG_USBMTP_SCSI_PRIO
#  define CONFIG_USBMTP_SCSI_PRIO 128
#endif

#ifndef CONFIG_USBMTP_SCSI_STACKSIZE
#  define CONFIG_USBMTP_SCSI_STACKSIZE 2048
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_USBMTP_EP0MAXPACKET
#  define CONFIG_USBMTP_EP0MAXPACKET 64
#endif

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBMTP_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBMTP_SELFPOWERED (0)
#endif

#ifdef  CONFIG_USBDEV_REMOTEWAKEUP
#  define USBMTP_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define USBMTP_REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/* Current state of the worker thread */

#define USBMTP_STATE_NOTSTARTED       (0)  /* Thread has not yet been started */
#define USBMTP_STATE_STARTED          (1)  /* Started, but is not yet initialized */
#define USBMTP_STATE_IDLE             (2)  /* Started and waiting for commands */
#define USBMTP_STATE_CMDPARSE         (3)  /* Processing a received command */
#define USBMTP_STATE_CMDREAD          (4)  /* Processing a SCSI read command */
#define USBMTP_STATE_CMDWRITE         (5)  /* Processing a SCSI write command */
#define USBMTP_STATE_CMDFINISH        (6)  /* Finish command processing */
#define USBMTP_STATE_CMDSTATUS        (7)  /* Processing the final status of the command */
#define USBMTP_STATE_TERMINATED       (8)  /* Thread has exitted */

/* Event communicated to worker thread */

#define USBMTP_EVENT_NOEVENTS         (0)      /* There are no outstanding events */
#define USBMTP_EVENT_READY            (1 << 0) /* Initialization is complete */
#define USBMTP_EVENT_RDCOMPLETE       (1 << 1) /* A read has completed there is data to be processed */
#define USBMTP_EVENT_WRCOMPLETE       (1 << 2) /* A write has completed and a request is available */
#define USBMTP_EVENT_TERMINATEREQUEST (1 << 3) /* Shutdown requested */
#define USBMTP_EVENT_DISCONNECT       (1 << 4) /* USB disconnect received */
#define USBMTP_EVENT_RESET            (1 << 5) /* USB storage setup reset received */
#define USBMTP_EVENT_CFGCHANGE        (1 << 6) /* USB setup configuration change received */
#define USBMTP_EVENT_IFCHANGE         (1 << 7) /* USB setup interface change received */
#define USBMTP_EVENT_ABORTBULKOUT     (1 << 8) /* SCSI receive failure */

/* SCSI command flags (passed to usbmtp_setupcmd()) */

#define USBMTP_FLAGS_DIRMASK          (0x03) /* Bits 0-1: Data direction */
#define USBMTP_FLAGS_DIRNONE          (0x00) /*   No data to send */
#define USBMTP_FLAGS_DIRHOST2DEVICE   (0x01) /*   Host-to-device */
#define USBMTP_FLAGS_DIRDEVICE2HOST   (0x02) /*   Device-to-host */
#define USBMTP_FLAGS_BLOCKXFR         (0x04) /* Bit 2: Command is a block transfer request */
#define USBMTP_FLAGS_LUNNOTNEEDED     (0x08) /* Bit 3: Command does not require a valid LUN */
#define USBMTP_FLAGS_UACOKAY          (0x10) /* Bit 4: Command OK if unit attention condition */
#define USBMTP_FLAGS_RETAINSENSEDATA  (0x20) /* Bit 5: Do not clear sense data */

/* Descriptors **************************************************************/

/* Big enough to hold our biggest descriptor */

#define USBMTP_MXDESCLEN              (64)
#define USBMTP_MAXSTRLEN              (USBMTP_MXDESCLEN-2)

/* String language */

#define USBMTP_STR_LANGUAGE           (0x0409) /* en-us */

/* Descriptor strings */

#ifndef CONFIG_USBMTP_COMPOSITE
#  define USBMTP_MANUFACTURERSTRID    (1)
#  define USBMTP_PRODUCTSTRID         (2)
#  define USBMTP_SERIALSTRID          (3)
#  define USBMTP_CONFIGSTRID          (6)
#  define USBMTP_INTERFACESTRID       USBMTP_CONFIGSTRID

#  undef CONFIG_USBMTP_STRBASE
#  define CONFIG_USBMTP_STRBASE       (0)
#else
#  define USBMTP_INTERFACESTRID       (CONFIG_USBMTP_STRBASE+1)
#endif

#define USBMTP_LASTSTRID              USBMTP_INTERFACESTRID
#define USBMTP_NSTRIDS               (USBMTP_LASTSTRID - CONFIG_USBMTP_STRBASE)

/* Configuration Descriptor */

#define USBMTP_INTERFACEID            (CONFIG_USBMTP_IFNOBASE+0)
#define USBMTP_ALTINTERFACEID         (0)

#define USBMTP_CONFIGIDNONE           (0) /* Config ID means to return to address mode */

/* Endpoint configuration */

#define USBMTP_MKEPBULKOUT(devDesc)   ((devDesc)->epno[USBMTP_EP_BULKOUT_IDX])
#define USBMTP_EPOUTBULK_ATTR         (USB_EP_ATTR_XFER_BULK)

#define USBMTP_MKEPBULKIN(devDesc)    (USB_DIR_IN | (devDesc)->epno[USBMTP_EP_BULKIN_IDX]) 
#define USBMTP_EPINBULK_ATTR          (USB_EP_ATTR_XFER_BULK)

#define USBMTP_MKEPINTIN(desc)        (USB_DIR_IN | (desc)->epno[USBMTP_EP_INTIN_IDX])
#define USBMTP_EPINTIN_ATTR           (USB_EP_ATTR_XFER_INT)

#define USBMTP_HSBULKMAXPACKET        (512)
#define USBMTP_HSBULKMXPKTSHIFT       (9)
#define USBMTP_HSBULKMXPKTMASK        (0x000001ff)
#define USBMTP_FSBULKMAXPACKET        (64)
#define USBMTP_FSBULKMXPKTSHIFT       (6)
#define USBMTP_FSBULKMXPKTMASK        (0x0000003f)

/* Configuration descriptor size */

#ifndef CONFIG_USBMTP_COMPOSITE

/* The size of the config descriptor: (9 + 9 + 2*7) = 32 */

#  define SIZEOF_USBMTP_CFGDESC \
     (USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + USBMTP_NENDPOINTS * USB_SIZEOF_EPDESC)

#else

/* The size of the config descriptor: (9 + 2*7) = 23 */

#  define SIZEOF_USBMTP_CFGDESC \
     (USB_SIZEOF_IFDESC + USBMTP_NENDPOINTS * USB_SIZEOF_EPDESC)

#endif

/* Block driver helpers *****************************************************/

#define USBMTP_DRVR_READ(l,b,s,n) \
  ((l)->inode->u.i_bops->read((l)->inode,b,s,n))
#define USBMTP_DRVR_WRITE(l,b,s,n) \
  ((l)->inode->u.i_bops->write((l)->inode,b,s,n))
#define USBMTP_DRVR_GEOMETRY(l,g) \
  ((l)->inode->u.i_bops->geometry((l)->inode,g))

/* Everpresent MIN/MAX macros ***********************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* Endpoint descriptors */

enum usbmtp_epdesc_e
{
  USBMTP_EPBULKIN  = 0,               /* Bulk IN endpoint descriptor */
  USBMTP_EPBULKOUT,                   /* Bulk OUT endpoint descriptor */
  USBMTP_EPINTIN                      /* Int */
};

/* Container to support a list of requests */

struct usbmtp_req_s
{
  FAR struct usbmtp_req_s *flink;     /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;       /* The contained request */
};

/* This structure describes one LUN: */

struct usbmtp_lun_s
{
  FAR struct inode    *inode;         /* Inode structure of open'ed block driver */
  uint8_t          readonly:1;        /* Media is read-only */
  uint8_t          locked:1;          /* Media removal is prevented */
  uint16_t         sectorsize;        /* The size of one sector */
  uint32_t         sd;                /* Sense data */
  uint32_t         sdinfo;            /* Sense data information */
  uint32_t         uad;               /* Unit needs attention data */
  off_t            startsector;       /* Sector offset to start of partition */
  size_t           nsectors;          /* Number of sectors in the partition */
};

/* Describes the overall state of one instance of the driver */

struct usbmtp_dev_s
{
  FAR struct usbdev_s *usbdev;        /* usbdev driver pointer (Non-null if registered) */

  /* SCSI worker kernel thread interface */

  pid_t             thpid;            /* The worker thread task ID */
  sem_t             thsynch;          /* Used to synchronizer terminal events */
  sem_t             thlock;           /* Used to get exclusive access to the state data */
  sem_t             thwaitsem;        /* Used to signal worker thread */
  volatile bool     thwaiting;        /* True: worker thread is waiting for an event */
  volatile uint8_t  thstate;          /* State of the worker thread */
  volatile uint16_t theventset;       /* Set of pending events signaled to worker thread */
  volatile uint8_t  thvalue;          /* Value passed with the event (must persist) */

  /* Storage class configuration and state */

  uint8_t           nluns:4;          /* Number of LUNs */
  uint8_t           config;           /* Configuration number */

  /* Endpoints */

  FAR struct usbdev_ep_s  *epintin;   /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkin;  /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout; /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;   /* Control request (for ep0 setup responses) */

  /* SCSI command processing */

  struct usbmtp_lun_s *lun;           /* Currently selected LUN */
  struct usbmtp_lun_s *luntab;        /* Allocated table of all LUNs */
  uint8_t cdb[USBMSC_MAXCDBLEN];      /* Command data (cdb[]) from CBW */
  uint8_t           phaseerror:1;     /* Need to send phase sensing status */
  uint8_t           shortpacket:1;    /* Host transmission stopped unexpectedly */
  uint8_t           cbwdir:2;         /* Direction from CBW. See USBMTP_FLAGS_DIR* definitions */
  uint8_t           cdblen;           /* Length of cdb[] from CBW */
  uint8_t           cbwlun;           /* LUN from the CBW */
  uint16_t          nsectbytes;       /* Bytes buffered in iobuffer[] */
  uint16_t          nreqbytes;        /* Bytes buffered in head write requests */
  uint16_t          iosize;           /* Size of iobuffer[] */
  uint32_t          cbwlen;           /* Length of data from CBW */
  uint32_t          cbwtag;           /* Tag from the CBW */
  union
    {
      uint32_t      xfrlen;           /* Read/Write: Sectors remaining to be transferred */
      uint32_t      alloclen;         /* Other device-to-host: Host allocation length */
    } u;
  uint32_t          sector;           /* Current sector (relative to lun->startsector) */
  uint32_t          residue;          /* Untransferred amount reported in the CSW */
  uint8_t          *iobuffer;         /* Buffer for data transfers */

  /* Write request list */

  struct sq_queue_s wrreqlist;        /* List of empty write request containers */
  struct sq_queue_s rdreqlist;        /* List of filled read request containers */

  struct usbdev_devinfo_s devinfo;

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (wrreqlist), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct usbmtp_req_s    wrreqs[CONFIG_USBMTP_NWRREQS];
  struct usbmtp_req_s    rdreqs[CONFIG_USBMTP_NRDREQS];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* String *******************************************************************/

/* Mass storage class vendor/product/serial number strings */

#ifndef CONFIG_USBMTP_COMPOSITE
EXTERN const char g_mtpvendorstr[];
EXTERN const char g_mtpproductstr[];
EXTERN const char g_mtpserialstr[];

/* If we are using a composite device, then vendor/product/serial number strings
 * are provided by the composite device logic.
 */

#else
EXTERN const char g_compvendorstr[];
EXTERN const char g_compproductstr[];
EXTERN const char g_compserialstr[];

#define g_mtpvendorstr  g_compvendorstr
#define g_mtpproductstr g_compproductstr
#define g_mtpserialstr  g_compserialstr
#endif

/* Used to hand-off the state structure when the SCSI worker thread is started */

EXTERN FAR struct usbmtp_dev_s *g_usbmtp_handoff;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: usbmtp_lowlayer_lock
 *
 * Description:
 *   Get exclusive access to SCSI state data.
 *
 ****************************************************************************/

void usbmtp_lowlayer_lock(FAR struct usbmtp_dev_s *priv);

/****************************************************************************
 * Name: usbmtp_lowlayer_unlock
 *
 * Description:
 *   Relinquish exclusive access to SCSI state data.
 *
 ****************************************************************************/

#define usbmtp_lowlayer_unlock(priv) sem_post(&priv->thlock)

/*****************************************************************************
 * Name: usbmtp_lowlayer_signal
 *
 * Description:
 *   Signal the SCSI worker thread that SCSI events need service.
 *
 ****************************************************************************/

void usbmtp_lowlayer_signal(FAR struct usbmtp_dev_s *priv);

/****************************************************************************
 * Name: usbmtp_synch_signal
 *
 * Description:
 *   ACK controlling tasks request for synchronization.
 *
 ****************************************************************************/

#define usbmtp_synch_signal(priv) sem_post(&priv->thsynch)

/****************************************************************************
 * Name: usbmtp_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

struct usb_strdesc_s;
int usbmtp_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/****************************************************************************
 * Name: usbmtp_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBMTP_COMPOSITE
FAR const struct usb_devdesc_s *usbmtp_getdevdesc(void);
#endif

/****************************************************************************
 * Name: usbmtp_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in ( sizeof(struct usb_epdesc_s) ).
 *
 ****************************************************************************/

int usbmtp_copy_epdesc(enum usbmtp_epdesc_e epid,
                       FAR struct usb_epdesc_s *epdesc,
                       FAR struct usbdev_devinfo_s *devinfo,
                       bool hispeed);

/****************************************************************************
 * Name: usbmtp_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t usbmtp_mkcfgdesc(FAR uint8_t *buf, FAR struct usbdev_devinfo_s *devinfo,
                         uint8_t speed, uint8_t type);
#else
int16_t usbmtp_mkcfgdesc(FAR uint8_t *buf, FAR struct usbdev_devinfo_s *devinfo);
#endif

/****************************************************************************
 * Name: usbmtp_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBMTP_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbmtp_getqualdesc(void);
#endif

/****************************************************************************
 * Name: usbmtp_lowlayer_main
 *
 * Description:
 *   This is the main function of the USB storage worker thread.  It loops
 *   until USB-related events occur, then processes those events accordingly
 *
 ****************************************************************************/

int usbmtp_lowlayer_main(int argc, char *argv[]);

/****************************************************************************
 * Name: usbmtp_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

int usbmtp_setconfig(FAR struct usbmtp_dev_s *priv, uint8_t config);

/****************************************************************************
 * Name: usbmtp_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

void usbmtp_resetconfig(FAR struct usbmtp_dev_s *priv);

/****************************************************************************
 * Name: usbmtp_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

void usbmtp_wrcomplete(FAR struct usbdev_ep_s *ep,
                       FAR struct usbdev_req_s *req);

/****************************************************************************
 * Name: usbmtp_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

void usbmtp_rdcomplete(FAR struct usbdev_ep_s *ep,
                       FAR struct usbdev_req_s *req);

/****************************************************************************
 * Name: usbmtp_deferredresponse
 *
 * Description:
 *   Some EP0 setup request cannot be responded to immediately becuase they
 *   require some asynchronous action from the SCSI worker thread.  This
 *   function is provided for the SCSI thread to make that deferred response.
 *   The specific requests that require this deferred response are:
 *
 *   1. USB_REQ_SETCONFIGURATION,
 *   2. USB_REQ_SETINTERFACE, or
 *   3. USBMTP_REQ_MSRESET
 *
 *   In all cases, the success reponse is a zero-length packet; the failure
 *   response is an EP0 stall.
 *
 * Input parameters:
 *   priv  - Private state structure for this USB storage instance
 *   stall - true is the action failed and a stall is required
 *
 ****************************************************************************/

void usbmtp_deferredresponse(FAR struct usbmtp_dev_s *priv, bool failed);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __DRIVERS_USBDEV_USBMTP_H */
