/****************************************************************************
 * net/bluetooth/bluetooth_finddev.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>

#include "netdev/netdev.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bluetooth_finddev_s
{
    FAR const struct bluetooth_saddr_s *addr;
    FAR struct radio_driver_s *radio;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth5_dev_callback
 *
 * Description:
 *   Check if this device matches the connections local address.
 *
 * Input Parameters:
 *   dev - The next network device in the enumeration
 *
 * Returned Value:
 *   0 if there is no match (meaning to continue looking); 1 if there is a
 *   match (meaning to stop the search).
 *
 ****************************************************************************/

static int bluetooth5_dev_callback(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR struct bluetooth_finddev_s *match =
    (FAR struct bluetooth_finddev_s *)arg;

  DEBUGASSERT(dev != NULL && match != NULL && match->addr != NULL);

  /* First, check if this network device is an Bluetooth radio and that
   * it has an assigned Bluetooth address.
   */

  if (dev->d_lltype == NET_LL_BLUETOOTH && dev->d_mac.radio.nv_addrlen > 0)
    {
      DEBUGASSERT(dev->d_mac.radio.nv_addrlen == 2 ||
                  dev->d_mac.radio.nv_addrlen == 8);

      /* Does the address mode match? */

      if (match->addr->s_mode == BLUETOOTH_ADDRMODE_SHORT &&
          dev->d_mac.radio.nv_addrlen == 2)
        {
          /* Does the device address match */

          if (BLUETOOTH_SADDRCMP(dev->d_mac.radio.nv_addr,
                                  match->addr->s_saddr))
            {
              /* Yes.. save the match and return 1 to stop the search */

              match->radio = (FAR struct radio_driver_s *)dev;
              return 1;
            }
        }
      else if (match->addr->s_mode == BLUETOOTH_ADDRMODE_EXTENDED &&
               dev->d_mac.radio.nv_addrlen == 8)
        {
          /* Does the device address match */

          if (BLUETOOTH_EADDRCMP(dev->d_mac.radio.nv_addr,
                                  match->addr->s_eaddr))
            {
              /* Yes.. save the match and return 1 to stop the search */

              match->radio = (FAR struct radio_driver_s *)dev;
              return 1;
            }
        }
    }

  /* Keep looking */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_find_device
 *
 * Description:
 *   Select the network driver to use with the Bluetooth transaction.
 *
 * Input Parameters:
 *   conn - Bluetooth connection structure (not currently used).
 *   addr - The address to match the devices assigned address
 *
 * Returned Value:
 *   A pointer to the network driver to use.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

FAR struct radio_driver_s *
  bluetooth_find_device(FAR struct bluetooth_conn_s *conn,
                         FAR const struct bluetooth_saddr_s *addr)
{
  struct bluetooth_finddev_s match;
  int ret;

  DEBUGASSERT(conn != NULL);
  match.addr  = addr;
  match.radio = NULL;

  /* If the socket is not bound to a local address, then return a failure */

  if (addr->s_mode == BLUETOOTH_ADDRMODE_NONE)
    {
      return NULL;
    }

  DEBUGASSERT(addr->s_mode == BLUETOOTH_ADDRMODE_SHORT ||
              addr->s_mode == BLUETOOTH_ADDRMODE_EXTENDED);

  /* Other, search for the Bluetooth network device whose MAC is equal to
   * the sockets bount local address.
   */

  ret = netdev_foreach(bluetooth5_dev_callback, (FAR void *)&match);
  if (ret == 1)
    {
      DEBUGASSERT(match.radio != NULL);
      return match.radio;
    }

  return NULL;
}

#endif /* CONFIG_NET_BLUETOOTH */
