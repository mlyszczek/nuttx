/****************************************************************************
 * drivers/usbdev/usbmtp_desc.c
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev_trace.h>

#include "usbmtp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Descriptors **************************************************************/
/* Device descriptor.  If the USB mass storage device is configured as part
 * of a composite device, then the device descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_USBMTP_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBMTP_EP0MAXPACKET,                   /* maxpacketsize */
  {                                             /* vendor */
    LSBYTE(CONFIG_USBMTP_VENDORID),
    MSBYTE(CONFIG_USBMTP_VENDORID)
  },
  {                                             /* product */
    LSBYTE(CONFIG_USBMTP_PRODUCTID),
    MSBYTE(CONFIG_USBMTP_PRODUCTID) },
  {                                             /* device */
    LSBYTE(CONFIG_USBMTP_VERSIONNO),
    MSBYTE(CONFIG_USBMTP_VERSIONNO)
  },
  USBMTP_MANUFACTURERSTRID,                     /* imfgr */
  USBMTP_PRODUCTSTRID,                          /* iproduct */
  USBMTP_SERIALSTRID,                           /* serno */
  USBMTP_NCONFIGS                               /* nconfigs */
};
#endif


#ifdef CONFIG_USBDEV_DUALSPEED
#ifndef CONFIG_USBMTP_COMPOSITE
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBMTP_EP0MAXPACKET,                   /* mxpacketsize */
  USBMTP_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* Strings ******************************************************************/

#ifndef CONFIG_USBMTP_COMPOSITE
const char g_mtpvendorstr[]  = CONFIG_USBMTP_VENDORSTR;
const char g_mtpproductstr[] = CONFIG_USBMTP_PRODUCTSTR;
const char g_mtpserialstr[]  = CONFIG_USBMTP_SERIALSTR;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmtp_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int usbmtp_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBMTP_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(USBMTP_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(USBMTP_STR_LANGUAGE);
        return 4;
      }

    case USBMTP_MANUFACTURERSTRID:
      str = g_mtpvendorstr;
      break;

    case USBMTP_PRODUCTSTRID:
      str = g_mtpproductstr;
      break;

    case USBMTP_SERIALSTRID:
      str = g_mtpserialstr;
      break;
#endif

 /* case USBMTP_CONFIGSTRID: */
    case USBMTP_INTERFACESTRID:
      str = CONFIG_USBMTP_CONFIGSTR;
      break;

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   if (len > (USBMTP_MAXSTRLEN / 2))
     {
       len = (USBMTP_MAXSTRLEN / 2);
     }

   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
}

/****************************************************************************
 * Name: usbmtp_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBMTP_COMPOSITE
FAR const struct usb_devdesc_s *usbmtp_getdevdesc(void)
{
  return &g_devdesc;
}
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
                       bool hispeed)
{
#ifndef CONFIG_USBDEV_DUALSPEED
    /* unused */

    (void)hispeed;
#endif

    switch (epid)
    {
    case USBMTP_EPBULKIN:  /* Bulk IN endpoint */
      {
        epdesc->len = USB_SIZEOF_EPDESC;            /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
        epdesc->addr = USBMTP_MKEPBULKIN(devinfo);  /* Endpoint address */
        epdesc->attr = USBMTP_EPINBULK_ATTR;        /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
        if (hispeed)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMTP_HSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMTP_HSBULKMAXPACKET);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMTP_FSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMTP_FSBULKMAXPACKET);
          }

        epdesc->interval = 0;                       /* Interval */
      }
      break;

    case USBMTP_EPBULKOUT:  /* Bulk OUT endpoint */
      {
        epdesc->len = USB_SIZEOF_EPDESC;            /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
        epdesc->addr = USBMTP_MKEPBULKOUT(devinfo); /* Endpoint address */
        epdesc->attr = USBMTP_EPOUTBULK_ATTR;       /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
        if (hispeed)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMTP_HSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMTP_HSBULKMAXPACKET);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMTP_FSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMTP_FSBULKMAXPACKET);
          }

        epdesc->interval = 0;                       /* Interval */
      }
      break;

    case USBMTP_EPINTIN:  /* Interrupt IN endpoint */
        {
          epdesc->len  = USB_SIZEOF_EPDESC;            /* Descriptor length */
          epdesc->type = USB_DESC_TYPE_ENDPOINT;       /* Descriptor type */
          epdesc->addr = USBMTP_MKEPINTIN(devinfo);    /* Endpoint address */
          epdesc->attr = USBMTP_EPINTIN_ATTR;          /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
          if (hispeed)
            {
              /* Maximum packet size (high speed) */

              epdesc->mxpacketsize[0] = LSBYTE(USBMTP_HSBULKMAXPACKET);
              epdesc->mxpacketsize[1] = MSBYTE(USBMTP_HSBULKMAXPACKET);
            }
          else
#endif
            {
              /* Maximum packet size (full speed) */

              epdesc->mxpacketsize[0] = LSBYTE(USBMTP_FSBULKMAXPACKET);
              epdesc->mxpacketsize[1] = MSBYTE(USBMTP_FSBULKMAXPACKET);
            }

          epdesc->interval = 8;                       /* Interval */
      }
      break;

    default:
        return 0;
    }

  return sizeof(struct usb_epdesc_s);
}

/****************************************************************************
 * Name: usbmtp_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t usbmtp_mkcfgdesc(uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo,
                         uint8_t speed, uint8_t type)
#else
int16_t usbmtp_mkcfgdesc(uint8_t *buf,
                        FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  int length = 0;
  bool hispeed = false;

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  /* Fill in all descriptors directly to the buf */

  /* Configuration descriptor.  If the USB mass storage device is
   * configured as part of a composite device, then the configuration
   * descriptor will be provided by the composite device logic.
   */

#ifndef CONFIG_USBMTP_COMPOSITE
  {
    /* Configuration descriptor  If the USB mass storage device is configured as part
     * of a composite device, then the configuration descriptor will be provided by the
     * composite device logic.
     */

    FAR struct usb_cfgdesc_s *dest = (FAR struct usb_cfgdesc_s *)buf;

    dest->len         = USB_SIZEOF_CFGDESC;               /* Descriptor length */
    dest->type        = USB_DESC_TYPE_CONFIG;             /* Descriptor type */
    dest->totallen[0] = LSBYTE(SIZEOF_USBMTP_CFGDESC);    /* LS Total length */
    dest->totallen[1] = MSBYTE(SIZEOF_USBMTP_CFGDESC);    /* MS Total length */
    dest->ninterfaces = USBMTP_NINTERFACES;               /* Number of interfaces */
    dest->cfgvalue    = USBMTP_CONFIGID;                  /* Configuration value */
    dest->icfg        = USBMTP_CONFIGSTRID;               /* Configuration */
    dest->attr        = USB_CONFIG_ATTR_ONE |             /* Attributes */
                        USBMTP_SELFPOWERED |
                        USBMTP_REMOTEWAKEUP;
    dest->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2; /* Max power (mA/2) */

    buf    += sizeof(struct usb_cfgdesc_s);
    length += sizeof(struct usb_cfgdesc_s);
  }
#endif

  /* Copy the canned interface descriptor */

  {
    /* Single interface descriptor */

    FAR struct usb_ifdesc_s * dest = (struct usb_ifdesc_s *)buf;

    dest->len      = USB_SIZEOF_IFDESC;                        /* Descriptor length */
    dest->type     = USB_DESC_TYPE_INTERFACE;                  /* Descriptor type */
    dest->ifno     = devinfo->ifnobase;                        /* Interface number */
    dest->alt      = USBMTP_ALTINTERFACEID;                    /* Alternate setting */
    dest->neps     = USBMTP_NENDPOINTS;                        /* Number of endpoints */
    dest->classid  = USB_CLASS_VENDOR_SPEC;                    /* Interface class */
    dest->subclass = USBMTP_SUBCLASS_VENDOR_SPEC;              /* Interface sub-class */
    dest->protocol = USBMTP_PROTO_CCI;                         /* Interface protocol */
    dest->iif      = devinfo->strbase + USBMTP_INTERFACESTRID; /* iInterface */

    buf    += sizeof(struct usb_ifdesc_s);
    length += sizeof(struct usb_ifdesc_s);
  }

  /* Make the two endpoint configurations */

  /* Bulk IN endpoint descriptor */

  {
    int len = usbmtp_copy_epdesc(USBMTP_EPBULKIN, (FAR struct usb_epdesc_s *)buf,
                                 devinfo, hispeed);

    buf += len;
    length += len;
  }

  /* Bulk OUT endpoint descriptor */

  {
    int len = usbmtp_copy_epdesc(USBMTP_EPBULKOUT,
                                 (FAR struct usb_epdesc_s *)buf, devinfo,
                                 hispeed);

    buf += len;
    length += len;
  }

  /* Interrupt IN endpoint descriptor */

  if (buf != NULL)
    {
      int len = usbmtp_copy_epdesc(USBMTP_EPINTIN,
                                   (FAR struct usb_epdesc_s *)buf, devinfo,
                                   hispeed);

      buf += len;
      length += len;
    }

  return SIZEOF_USBMTP_CFGDESC;
}

/****************************************************************************
 * Name: usbmtp_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBMTP_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbmtp_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif
