/****************************************************************************
 * net/bluetooth/bluetooth_conn.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/bluetooth/bluetooth_mac.h>

#include "devif/devif.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections.  Protected via the
 * network lock.
 */

static struct bluetooth_conn_s
  g_bluetooth_connections[CONFIG_NET_BLUETOOTH_NCONNS];

/* A list of all free packet socket connections */

static dq_queue_t g_free_bluetooth_connections;

/* A list of all allocated packet socket connections */

static dq_queue_t g_active_bluetooth_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_conn_initialize
 *
 * Description:
 *   Initialize the Bluetooth connection structure allocator.  Called
 *   once and only from bluetooth_initialize().
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void bluetooth_conn_initialize(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_bluetooth_connections);
  dq_init(&g_active_bluetooth_connections);

  for (i = 0; i < CONFIG_NET_BLUETOOTH_NCONNS; i++)
    {
      /* Link each pre-allocated connection structure into the free list. */

      dq_addlast(&g_bluetooth_connections[i].node,
                 &g_free_bluetooth_connections);
    }
}

/****************************************************************************
 * Name: bluetooth_conn_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized packet socket connection structure. This
 *   is normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *bluetooth_conn_alloc(void)
{
  FAR struct bluetooth_conn_s *conn;

  /* The free list is only accessed from user, non-interrupt level and
   * is protected by a semaphore (that behaves like a mutex).
   */

  net_lock();
  conn = (FAR struct bluetooth_conn_s *)
    dq_remfirst(&g_free_bluetooth_connections);

  if (conn)
    {
      /* Enqueue the connection into the active list */

      memset(conn, 0, sizeof(struct bluetooth_conn_s));
      dq_addlast(&conn->node, &g_active_bluetooth_connections);
    }

  net_unlock();
  return conn;
}

/****************************************************************************
 * Name: bluetooth_conn_free()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void bluetooth_conn_free(FAR struct bluetooth_conn_s *conn)
{
  FAR struct bluetooth_container_s *container;
  FAR struct bluetooth_container_s *next;

  /* The free list is only accessed from user, non-interrupt level and
   * is protected by a semaphore (that behaves like a mutex).
   */

  DEBUGASSERT(conn->crefs == 0);

  /* Remove the connection from the active list */

  net_lock();
  dq_rem(&conn->node, &g_active_bluetooth_connections);

  /* Check if there any any frames attached to the container */

  for (container = conn->rxhead; container != NULL; container = next)
    {
      /* Remove the frame from the list */

      next                = container->ic_flink;
      container->ic_flink = NULL;

      /* Free the contained frame data (should be only one in chain) */

      if (container->ic_iob)
        {
          iob_free(container->ic_iob);
        }

      /* And free the container itself */

      bluetooth_container_free(container);
    }

  /* Free the connection */

  dq_addlast(&conn->node, &g_free_bluetooth_connections);
  net_unlock();
}

/****************************************************************************
 * Name: bluetooth_conn_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided Bluetooth header
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *
  bluetooth_conn_active(FAR const struct bluetooth_data_ind_s *meta)
{
  FAR struct bluetooth_conn_s *conn;

  DEBUGASSERT(meta != NULL);

  for (conn  = (FAR struct bluetooth_conn_s *)g_active_bluetooth_connections.head;
       conn != NULL;
       conn = (FAR struct bluetooth_conn_s *)conn->node.flink)
    {
      /* Does the destination address match the bound address of the socket. */
      /* REVISIT: Currently and explict address must be assigned.  Should we
       * support some moral equivalent to INADDR_ANY?
       */

      if (meta->dest.mode != conn->laddr.s_mode)
        {
          continue;
        }

      if (meta->dest.mode == BLUETOOTH_ADDRMODE_SHORT &&
          !BLUETOOTH_SADDRCMP(meta->dest.saddr, conn->laddr.s_saddr))
        {
          continue;
        }

      if (meta->dest.mode == BLUETOOTH_ADDRMODE_EXTENDED &&
          !BLUETOOTH_EADDRCMP(meta->dest.saddr, conn->laddr.s_eaddr))
        {
          continue;
        }

      /* Is the socket "connected?" to a remote peer?  If so, check if the
       * source address matches the connected remote adress.
       */

      switch (conn->raddr.s_mode)
        {
          case BLUETOOTH_ADDRMODE_NONE:
            return conn;  /* No.. accept the connection */

          case BLUETOOTH_ADDRMODE_SHORT:
            if (BLUETOOTH_SADDRCMP(meta->dest.saddr, conn->raddr.s_saddr))
              {
                return conn;
              }
            break;

          case BLUETOOTH_ADDRMODE_EXTENDED:
            if (BLUETOOTH_EADDRCMP(meta->dest.eaddr, conn->raddr.s_eaddr))
              {
                return conn;
              }
            break;

           default:
             nerr("ERROR: Invalid address mode: %u\n", conn->raddr.s_mode);
             return NULL;
        }
    }

  return conn;
}

/****************************************************************************
 * Name: bluetooth_conn_next()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *
  bluetooth_conn_next(FAR struct bluetooth_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct bluetooth_conn_s *)
        g_active_bluetooth_connections.head;
    }
  else
    {
      return (FAR struct bluetooth_conn_s *)conn->node.flink;
    }
}

#endif /* CONFIG_NET_BLUETOOTH */
