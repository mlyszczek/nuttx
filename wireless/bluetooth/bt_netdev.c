/****************************************************************************
 * wireless/bluetooth/bt_netdev.c
 * Network stack interface
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/net/sixlowpan.h>

#include "bt_ioctl.h"

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_BLUETOOTH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* Frame size */

#if defined(CONFIG_NET_BLUETOOTH_FRAMELEN)
#  define MACNET_FRAMELEN CONFIG_NET_BLUETOOTH_FRAMELEN
#else
#  define MACNET_FRAMELEN BLUETOOTH_MAX_PHY_PACKET_SIZE
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define TXPOLL_WDDELAY   (1*CLK_TCK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is our private version of the MAC callback stucture */

struct bt_callback_s
{
  /* This holds the information visible to the MAC layer */

  FAR struct bt_driver_s *mc_priv;     /* Our priv data */
};

/* The bt_driver_s encapsulates all state information for a single
 * Bluetooth device interface.
 */

struct bt_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct radio_driver_s md_dev;   /* Interface understood by the network */
                                  /* Cast compatible with struct bt_driver_s */

  /* For internal use by this driver */

  sem_t md_exclsem;               /* Exclusive access to struct */
  struct bt_callback_s md_cb;     /* Callback information */
  bool md_bifup;                  /* true:ifup false:ifdown */
  WDOG_ID md_txpoll;              /* TX poll timer */
  struct work_s md_pollwork;      /* Defer poll work to the work queue */

  /* Hold a list of events */

  bool md_enableevents : 1;       /* Are events enabled? */
  bool md_eventpending : 1;       /* Is there a get event using the semaphore? */
  sem_t md_eventsem;              /* Signaling semaphore for waiting get event */
  sq_queue_t primitive_queue;     /* For holding primitives to pass along */

#ifndef CONFIG_DISABLE_SIGNALS
  /* MAC Service notification information */

  bool    md_notify_registered;
  uint8_t md_notify_signo;
  pid_t   md_notify_pid;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions ********************************************************/

static int  bt_advertise(FAR struct net_driver_s *dev);
static inline void bt_netmask(FAR struct net_driver_s *dev);

/* IEE802.15.4 MAC callback functions ***************************************/

static int  bt_notify(FAR struct bt_maccb_s *maccb,
              FAR struct bluetooth_primitive_s *primitive);
static int  bt_rxframe(FAR struct bt_driver_s *maccb,
              FAR struct bluetooth_data_ind_s *ind);

/* Network interface support ************************************************/
/* Common TX logic */

static int  bt_txpoll_callback(FAR struct net_driver_s *dev);
static void bt_txpoll_work(FAR void *arg);
static void bt_txpoll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int  bt_ifup(FAR struct net_driver_s *dev);
static int  bt_ifdown(FAR struct net_driver_s *dev);

static void bt_txavail_work(FAR void *arg);
static int  bt_txavail(FAR struct net_driver_s *dev);

#ifdef CONFIG_NET_IGMP
static int  bt_addmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
static int  bt_rmmac(FAR struct net_driver_s *dev,
              FAR const uint8_t *mac);
#endif
static int  bt_get_mhrlen(FAR struct radio_driver_s *netdev,
              FAR const void *meta);
static int  bt_req_data(FAR struct radio_driver_s *netdev,
              FAR const void *meta, FAR struct iob_s *framelist);
static int  bt_properties(FAR struct radio_driver_s *netdev,
              FAR struct radiodev_properties_s *properties);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static struct sixlowpan_reassbuf_s g_iobuffer;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_advertise
 *
 * Description:
 *   Advertise the MAC and IPv6 address for this node.
 *
 *   Creates a MAC-based IP address from the 6-byte address address assigned
 *   to the device.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0200 xxxx xxxx xxxx
 *
 ****************************************************************************/

static int bt_advertise(FAR struct net_driver_s *dev)
{
  FAR struct bt_driver_s *priv;
  FAR uint8_t *addr;
  union bluetooth_macarg_u arg;
  int ret;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL);
  priv = (FAR struct bt_driver_s *)dev->d_private;

  /* Get the 6-byte address from the device */
#warning Missing logic

  /* Set the MAC address using that address */

  BLUETOOTH_ADDRCOPY(dev->d_mac.radio.nv_addr, addr);
  dev->d_mac.radio.nv_addrlen = BLUETOOTH_ADDRSIZE;

#ifdef CONFIG_NET_IPv6
  /* Set the IP address based on the 6-byte address */

  dev->d_ipv6addr[0]  = HTONS(0xfe80);
  dev->d_ipv6addr[1]  = 0;
  dev->d_ipv6addr[2]  = 0;
  dev->d_ipv6addr[3]  = 0x200;
  dev->d_ipv6addr[5]  = (uint16_t)addr[0] << 8 | (uint16_t)addr[1];
  dev->d_ipv6addr[6]  = (uint16_t)addr[2] << 8 | (uint16_t)addr[3];
  dev->d_ipv6addr[7]  = (uint16_t)addr[4] << 8 | (uint16_t)addr[5];
#endif
  return OK;
}

/****************************************************************************
 * Name: bt_netmask
 *
 * Description:
 *   Create a netmask of a MAC-based IP address which is based on the 6-byte
 *   Bluetooth address.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx
 *
 ****************************************************************************/

static inline void bt_netmask(FAR struct net_driver_s *dev)
{
#ifdef CONFIG_NET_IPv6
  dev->d_ipv6netmask[0]  = 0xffff;
  dev->d_ipv6netmask[1]  = 0xffff;
  dev->d_ipv6netmask[2]  = 0xffff;
  dev->d_ipv6netmask[3]  = 0xffff;
  dev->d_ipv6netmask[4]  = 0;
  dev->d_ipv6netmask[5]  = 0;
  dev->d_ipv6netmask[6]  = 0;
  dev->d_ipv6netmask[7]  = 0;
#endif
}

/****************************************************************************
 * Name: bt_notify
 *
 * Description:
 *
 ****************************************************************************/

static int bt_notify(FAR struct bt_maccb_s *maccb,
                     FAR struct bluetooth_primitive_s *primitive)
{
  FAR struct bt_callback_s *cb =
    (FAR struct bt_callback_s *)maccb;
  FAR struct bt_driver_s *priv;

  DEBUGASSERT(cb != NULL && cb->mc_priv != NULL);
  priv = cb->mc_priv;

  /* Handle the special case for data indications or "incoming frames" */

  if (primitive->type == BLUETOOTH_PRIMITIVE_IND_DATA)
    {
      return bt_rxframe(priv, &primitive->u.dataind);
    }

  /* If there is a registered notification receiver, queue the event and signal
   * the receiver. Events should be popped from the queue from the application
   * at a reasonable rate in order for the MAC layer to be able to allocate new
   * notifications.
   */

  if (priv->md_enableevents)
    {
      /* Get exclusive access to the driver structure.  We don't care about any
       * signals so if we see one, just go back to trying to get access again
       */

      while (nxsem_wait(&priv->md_exclsem) < 0);

      sq_addlast((FAR sq_entry_t *)primitive, &priv->primitive_queue);

      /* Check if there is a read waiting for data */

      if (priv->md_eventpending)
        {
          /* Wake the thread waiting for the data transmission */

          priv->md_eventpending = false;
          nxsem_post(&priv->md_eventsem);
        }

#ifndef CONFIG_DISABLE_SIGNALS
      if (priv->md_notify_registered)
        {
#ifdef CONFIG_CAN_PASS_STRUCTS
          union sigval value;
          value.sival_int = (int)primitive->type;
          (void)nxsig_queue(priv->md_notify_pid, priv->md_notify_signo,
                            value);
#else
          (void)nxsig_queue(priv->md_notify_pid, priv->md_notify_signo,
                            (FAR void *)primitive->type);
#endif
        }
#endif

      nxsem_post(&priv->md_exclsem);
      return OK;
    }

  /* By returning a negative value, we let the MAC know that we don't want the
   * primitive and it will free it for us
   */

  return -1;
}

/****************************************************************************
 * Name: bt_rxframe
 *
 * Description:
 *   Handle received frames forward by the Bluetooth stack.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  On success, the ind and its contained iob will be freed.
 *   The ind will be intact if this function returns a failure.
 *
 ****************************************************************************/

static int bt_rxframe(FAR struct bt_driver_s *priv,
                          FAR struct bluetooth_data_ind_s *ind)
{
  FAR struct iob_s *iob;
  int ret;

  /* Ignore the frame if the network is not up */

  if (!priv->md_bifup)
    {
      wlwarn("WARNING: Dropped... Network is down\n");
      return -ENETDOWN;
    }

  /* Peek the IOB contained the frame in the struct bluetooth_data_ind_s */

  DEBUGASSERT(priv != NULL && ind != NULL && ind->frame != NULL);
  iob = ind->frame;

  /* Remove the IOB containing the frame. */

  ind->frame = NULL;

  net_lock();

  /* Transfer the frame to the network logic */

#ifdef CONFIG_NET_BLUETOOTH
  /* Invoke the PF_BLUETOOTH tap first.  If the frame matches
   * with a connected PF_BLUETOOTH socket, it will take the
   * frame and return success.
   */

  ret = bluetooth_input(&priv->md_dev, iob, (FAR void *)ind);
  if (ret < 0)
#endif
#ifdef CONFIG_NET_6LOWPAN
    {
      /* If the frame is not a 6LoWPAN frame, then return an error.  The
       * first byte following the MAC head at the io_offset should be a
       * valid IPHC header.
       */

      if ((iob->io_data[iob->io_offset] & SIXLOWPAN_DISPATCH_NALP_MASK) ==
          SIXLOWPAN_DISPATCH_NALP)
        {
          wlwarn("WARNING: Dropped... Not a 6LoWPAN frame: %02x\n",
                 iob->io_data[iob->io_offset]);
          ret = -EINVAL;
        }
      else
        {
          /* Make sure the our single packet buffer is attached */

          priv->md_dev.r_dev.d_buf = g_iobuffer.rb_buf;

          /* And give the packet to 6LoWPAN */

          ret = sixlowpan_input(&priv->md_dev, iob, (FAR void *)ind);
        }
    }

  if (ret < 0)
#endif
    {
      net_unlock();
      ind->frame = iob;
      return ret;
    }

  /* Increment statistics */

  NETDEV_RXPACKETS(&priv->md_dev.r_dev);
  NETDEV_RXIPV6(&priv->md_dev.r_dev);

  net_unlock();

  /* sixlowpan_input() will free the IOB, but we must free the struct
   * bluetooth_primitive_s container here.
   */

  bluetooth_primitive_free((FAR struct bluetooth_primitive_s *)ind);
  return OK;
}

/****************************************************************************
 * Name: bt_txpoll_callback
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int bt_txpoll_callback(FAR struct net_driver_s *dev)
{
  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: bt_txpoll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void bt_txpoll_work(FAR void *arg)
{
  FAR struct bt_driver_s *priv = (FAR struct bt_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

#ifdef CONFIG_NET_6LOWPAN
  /* Make sure the our single packet buffer is attached */

  priv->md_dev.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

  /* Then perform the poll */

  (void)devif_timer(&priv->md_dev.r_dev, bt_txpoll_callback);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->md_txpoll, TXPOLL_WDDELAY, bt_txpoll_expiry, 1,
                 (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: bt_txpoll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void bt_txpoll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct bt_driver_s *priv = (FAR struct bt_driver_s *)arg;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(LPWORK, &priv->md_pollwork, bt_txpoll_work, priv, 0);
}

/****************************************************************************
 * Name: bt_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Bluetooth interface when an IP address
 *   is provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int bt_ifup(FAR struct net_driver_s *dev)
{
  FAR struct bt_driver_s *priv =
    (FAR struct bt_driver_s *)dev->d_private;
  int ret;

  /* Set the IP address based on the addressing assigned to the node */

  ret = bt_advertise(dev);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_IPv6
      wlinfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
             dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
             dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
             dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
      wlinfo("             Node: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
             dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1],
             dev->d_mac.radio.nv_addr[2], dev->d_mac.radio.nv_addr[3],
             dev->d_mac.radio.nv_addr[4], dev->d_mac.radio.nv_addr[5],
             dev->d_mac.radio.nv_addr[6], dev->d_mac.radio.nv_addr[7]);
#else
      wlinfo("             Node: %02x:%02x\n",
             dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1]);
#endif
#else
      if (dev->d_mac.radio.nv_addrlen == 8)
        {
          ninfo("Bringing up: Node: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x PANID=%02x:%02x\n",
                 dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1],
                 dev->d_mac.radio.nv_addr[2], dev->d_mac.radio.nv_addr[3],
                 dev->d_mac.radio.nv_addr[4], dev->d_mac.radio.nv_addr[5],
                 dev->d_mac.radio.nv_addr[6], dev->d_mac.radio.nv_addr[7],
                 priv->lo_panid[0], priv->lo_panid[1]);
        }
      else if (dev->d_mac.radio.nv_addrlen == 2)
        {
          ninfo("Bringing up: Node: %02x:%02x PANID=%02x:%02x\n",
                 dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1],
                 priv->lo_panid[0], priv->lo_panid[1]);
        }
      else
        {
          nerr("ERROR: No address assigned\n");
        }
#endif

      /* Set and activate a timer process */

      (void)wd_start(priv->md_txpoll, TXPOLL_WDDELAY, bt_txpoll_expiry,
                     1, (wdparm_t)priv);

      /* The interface is now up */

      priv->md_bifup = true;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: bt_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int bt_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct bt_driver_s *priv = (FAR struct bt_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interruption */

  flags = enter_critical_section();

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->md_txpoll);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the bt_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->md_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: bt_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void bt_txavail_work(FAR void *arg)
{
  FAR struct bt_driver_s *priv = (FAR struct bt_driver_s *)arg;

  wlinfo("ifup=%u\n", priv->md_bifup);

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->md_bifup)
    {
#ifdef CONFIG_NET_6LOWPAN
      /* Make sure the our single packet buffer is attached */

      priv->md_dev.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

      /* Then poll the network for new XMIT data */

      (void)devif_poll(&priv->md_dev.r_dev, bt_txpoll_callback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: bt_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int bt_txavail(FAR struct net_driver_s *dev)
{
  FAR struct bt_driver_s *priv = (FAR struct bt_driver_s *)dev->d_private;

  wlinfo("Available=%u\n", work_available(&priv->md_pollwork));

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->md_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(LPWORK, &priv->md_pollwork, bt_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: bt_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int bt_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* Add the MAC address to the hardware multicast routing table.  Not used
   * with Bluetooth.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: bt_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int bt_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* Remove the MAC address from the hardware multicast routing table  Not used
   * with Bluetooth.
   */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: bt_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *
 * Returned Value:
 *   A non-negative MAC header length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int bt_get_mhrlen(FAR struct radio_driver_s *netdev,
                         FAR const void *meta)
{
  return BLUETOOTH_HDRLEN;
}

/****************************************************************************
 * Name: bt_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int bt_req_data(FAR struct radio_driver_s *netdev,
                           FAR const void *meta, FAR struct iob_s *framelist)
{
  FAR struct bt_driver_s *priv =
    (FAR struct bt_driver_s *)netdev;
  FAR const struct bluetooth_frame_meta_s *pktmeta =
    (FAR const struct bluetooth_frame_meta_s *)meta;
  FAR struct iob_s *iob;
  int ret;

  wlinfo("Received framelist\n");

  DEBUGASSERT(priv != NULL && pktmeta != NULL && framelist != NULL);

  /* Add the incoming list of frames to the MAC's outgoing queue */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->md_dev.r_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      /* Transfer the frame to the MAC. */

      do
        {
          ret = bt_req_data(pktmeta, iob);
        }
      while (ret == -EINTR);

      if (ret < 0)
        {
          wlerr("ERROR: bt_req_data failed: %d\n", ret);

          iob_free(iob);
          for (iob = framelist; iob != NULL; iob = framelist)
            {
              /* Remove the IOB from the queue and free */

              framelist = iob->io_flink;
              iob_free(iob);
            }

          NETDEV_TXERRORS(&priv->md_dev.r_dev);
          return ret;
        }

      NETDEV_TXDONE(&priv->md_dev.r_dev);
    }

  return OK;
}

/****************************************************************************
 * Name: bt_properties
 *
 * Description:
 *   Different packet radios may have different properties.  If there are
 *   multiple packet radios, then those properties have to be queried at
 *   run time.  This information is provided to the 6LoWPAN network via the
 *   following structure.
 *
 * Input Parameters:
 *   netdev     - The network device to be queried
 *   properties - Location where radio properties will be returned.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int bt_properties(FAR struct radio_driver_s *netdev,
                             FAR struct radiodev_properties_s *properties)
{
  DEBUGASSERT(netdev != NULL && properties != NULL);
  memset(properties, 0, sizeof(struct radiodev_properties_s));

  /* General */

  properties->sp_addrlen  = BLUETOOTH_ADDRSIZE;  /* Length of an address */
  properties->sp_framelen = MACNET_FRAMELEN;  /* Fixed frame length */

  /* Multicast address -- not supported */

  properties->sp_mcast.nv_addrlen = BLUETOOTH_ADDRSIZE;
  memset(properties->sp_mcast.nv_addr, 0xff, RADIO_MAX_ADDRLEN);

  /* Broadcast address -- not supported */

  properties->sp_bcast.nv_addrlen = BLUETOOTH_ADDRSIZE;
  memset(properties->sp_mcast.nv_addr, 0xff, RADIO_MAX_ADDRLEN);

#ifdef CONFIG_NET_STARPOINT
  /* Star hub node address -- not supported. */

  properties->sp_hubnode.nv_addrlen = BLUETOOTH_ADDRSIZE;
  memset(properties->sp_hubnode.nv_addr, RADIO_MAX_ADDRLEN);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_netdev_register
 *
 * Description:
 *   Register a network driver to access the Bluetooth MAC layer using a
 *   6LoWPAN IPv6 or AF_BLUETOOTH socket.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int bt_netdev_register(void)
{
  FAR struct bt_driver_s *priv;
  FAR struct radio_driver_s *radio;
  FAR struct net_driver_s  *dev;
  FAR struct bt_maccb_s *maccb;
  int ret;

  DEBUGASSERT(mac != NULL);

  /* Get the interface structure associated with this interface number. */

  priv = (FAR struct bt_driver_s *)
    kmm_zalloc(sizeof(struct bt_driver_s));

  if (priv == NULL)
    {
      nerr("ERROR: Failed to allocate the device structure\n");
      return -ENOMEM;
    }

  /* Initialize the driver structure */

  radio               = &priv->md_dev;
  dev                 = &radio->r_dev;
  dev->d_ifup         = bt_ifup;           /* I/F up (new IP address) callback */
  dev->d_ifdown       = bt_ifdown;         /* I/F down callback */
  dev->d_txavail      = bt_txavail;        /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  dev->d_addmac       = bt_addmac;         /* Add multicast MAC address */
  dev->d_rmmac        = bt_rmmac;          /* Remove multicast MAC address */
#endif
 #ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl        = bt_ioctl;          /* Handle network IOCTL commands */
#endif
  dev->d_private      = (FAR void *)priv;  /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->md_txpoll     = wd_create();       /* Create periodic poll timer */

  /* Setup a locking semaphore for exclusive device driver access */

  nxsem_init(&priv->md_exclsem, 0, 1);

  DEBUGASSERT(priv->md_txpoll != NULL);

  /* Set the network mask. */

  bt_netmask(dev);

  /* Initialize the Network frame-related callbacks */

  radio->r_get_mhrlen = bt_get_mhrlen;  /* Get MAC header length */
  radio->r_req_data   = bt_req_data;    /* Enqueue frame for transmission */
  radio->r_properties = bt_properties;  /* Return radio properies */

  /* Initialize fields related to MAC event handling */

  priv->md_eventpending = false;
  nxsem_init(&priv->md_eventsem, 0, 0);
  nxsem_setprotocol(&priv->md_eventsem, SEM_PRIO_NONE);

  sq_init(&priv->primitive_queue);

  priv->md_enableevents = false;
  priv->md_notify_registered = false;

  /* Initialize the MAC callbacks */

  priv->md_cb.mc_priv = priv;

  maccb           = &priv->md_cb.mc_cb;
  maccb->flink    = NULL;
  maccb->prio     = CONFIG_BLUETOOTH_NETDEV_RECVRPRIO;
  maccb->notify   = bt_notify;
  maccb->rxframe  = bt_rxframe;

  /* Initialize the Bluetooth stack */

  ret = bt_init();
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize Bluetooth: %d\n", ret);
      goto errout;
    }

  /* Put the interface in the down state. */

  bt_ifdown(dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->md_dev.r_dev, NET_LL_BLUETOOTH);
  if (ret >= 0)
    {
      return OK;
    }

errout:
  /* Release wdog timers */

  wd_delete(priv->md_txpoll);

  /* Free memory and return the error */

  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_skeleton */
