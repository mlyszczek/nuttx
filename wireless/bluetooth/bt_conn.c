/****************************************************************************
 * wireless/bluetooth/bt_conn.c
 * Bluetooth connection handling.
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wireless/bt_log.h>
#include <nuttx/wireless/bt_hci.h>
#include <nuttx/wireless/bt_bluetooth.h>

#include "bt_atomic.h"
#include "bt_hcicore.h"
#include "bt_conn.h"
#include "bt_l2cap.h"
#include "bt_keys.h"
#include "bt_smp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_conn_s conns[CONFIG_BLUETOOTH_MAX_CONN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_WIRELESS_INFO
static const char *state2str(enum bt_conn_state_e state)
{
  switch (state)
    {
    case BT_CONN_DISCONNECTED:
      return "disconnected";

    case BT_CONN_CONNECT_SCAN:
      return "connect-scan";

    case BT_CONN_CONNECT:
      return "connect";

    case BT_CONN_CONNECTED:
      return "connected";

    case BT_CONN_DISCONNECT:
      return "disconnect";

    default:
      return "(unknown)";
    }
}
#endif

static void bt_conn_reset_rx_state(FAR struct bt_conn_s *conn)
{
  if (!conn->rx_len)
    {
      return;
    }

  bt_buf_put(conn->rx);
  conn->rx     = NULL;
  conn->rx_len = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_conn_recv(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf, uint8_t flags)
{
  struct bt_l2cap_hdr *hdr;
  uint16_t len;

  winfo("handle %u len %u flags %02x\n", conn->handle, buf->len, flags);

  /* Check packet boundary flags */
  switch (flags)
    {
      case 0x02:
        /* First packet */

        hdr = (void *)buf->data;
        len = sys_le16_to_cpu(hdr->len);

        winfo("First, len %u final %u\n", buf->len, len);

        if (conn->rx_len)
          {
            wlerr("ERROR: Unexpected first L2CAP frame\n");
            bt_conn_reset_rx_state(conn);
          }

        conn->rx_len = (sizeof(*hdr) + len) - buf->len;
        winfo("rx_len %u\n", conn->rx_len);
        if (conn->rx_len)
          {
            conn->rx = buf;
            return;
          }

        break;

      case 0x01:
        /* Continuation */

        if (!conn->rx_len)
          {
            wlerr("ERROR: Unexpected L2CAP continuation\n");
            bt_conn_reset_rx_state(conn);
            bt_buf_put(buf);
            return;
          }

        if (buf->len > conn->rx_len)
          {
            wlerr("ERROR: L2CAP data overflow\n");
            bt_conn_reset_rx_state(conn);
            bt_buf_put(buf);
            return;
          }

        winfo("Cont, len %u rx_len %u\n", buf->len, conn->rx_len);

        if (buf->len > bt_buf_tailroom(conn->rx))
          {
            wlerr("ERROR: Not enough buffer space for L2CAP data\n");
            bt_conn_reset_rx_state(conn);
            bt_buf_put(buf);
            return;
          }

        memcpy(bt_buf_add(conn->rx, buf->len), buf->data, buf->len);
        conn->rx_len -= buf->len;
        bt_buf_put(buf);

        if (conn->rx_len)
          {
            return;
          }

        buf          = conn->rx;
        conn->rx     = NULL;
        conn->rx_len = 0;

        break;

      default:
        wlerr("ERROR: Unexpected ACL flags (0x%02x)\n", flags);
        bt_conn_reset_rx_state(conn);
        bt_buf_put(buf);
        return;
    }

  hdr = (void *)buf->data;
  len = sys_le16_to_cpu(hdr->len);

  if (sizeof(*hdr) + len != buf->len)
    {
      wlerr("ERROR: ACL len mismatch (%u != %u)\n", len, buf->len);
      bt_buf_put(buf);
      return;
    }

  winfo("Successfully parsed %u byte L2CAP packet\n", buf->len);

  bt_l2cap_recv(conn, buf);
}

void bt_conn_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_acl_hdr_s *hdr;
  struct nano_fifo frags;
  uint16_t len;
  uint16_t remaining = buf->len;
  FAR uint8_t *ptr;

  winfo("conn handle %u buf len %u\n", conn->handle, buf->len);

  if (conn->state != BT_CONN_CONNECTED)
    {
      wlerr("ERROR: not connected!\n");
      return;
    }

  nano_fifo_init(&frags);

  len         = min(remaining, bt_dev.le_mtu);

  hdr         = bt_buf_push(buf, sizeof(*hdr));
  hdr->handle = sys_cpu_to_le16(conn->handle);
  hdr->len    = sys_cpu_to_le16(len);

  buf->len   -= remaining - len;
  ptr         = bt_buf_tail(buf);

  nano_fifo_put(&frags, buf);
  remaining  -= len;

  while (remaining)
    {
      buf = bt_l2cap_create_pdu(conn);

      len = min(remaining, bt_dev.le_mtu);

      /* Copy from original buffer */

      memcpy(bt_buf_add(buf, len), ptr, len);
      ptr        += len;

      hdr         = bt_buf_push(buf, sizeof(*hdr));
      hdr->handle = sys_cpu_to_le16(conn->handle | (1 << 12));
      hdr->len    = sys_cpu_to_le16(len);

      nano_fifo_put(&frags, buf);
      remaining  -= len;
    }

  while ((buf = nano_fifo_get(&frags)))
    {
      nano_fifo_put(&conn->tx_queue, buf);
    }
}

static void conn_tx_fiber(int arg1, int arg2)
{
  FAR struct bt_conn_s *conn = (FAR struct bt_conn_s *)arg1;
  FAR struct bt_buf_s *buf;

  winfo("Started for handle %u\n", conn->handle);

  while (conn->state == BT_CONN_CONNECTED)
    {
      /* Wait until the controller can accept ACL packets */

      winfo("calling sem_take_wait\n");
      nano_fiber_sem_take_wait(&bt_dev.le_pkts_sem);

      /* Check for disconnection */

      if (conn->state != BT_CONN_CONNECTED)
        {
          nano_fiber_sem_give(&bt_dev.le_pkts_sem);
          break;
        }

      /* Get next ACL packet for connection */

      buf = nano_fifo_get_wait(&conn->tx_queue);
      if (conn->state != BT_CONN_CONNECTED)
        {
          nano_fiber_sem_give(&bt_dev.le_pkts_sem);
          bt_buf_put(buf);
          break;
        }

      winfo("passing buf %p len %u to driver\n", buf, buf->len);
      bt_dev.drv->send(buf);
      bt_buf_put(buf);
    }

  winfo("handle %u disconnected - cleaning up\n", conn->handle);

  /* Give back any allocated buffers */

  while ((buf = nano_fifo_get(&conn->tx_queue)))
    {
      bt_buf_put(buf);
    }

  bt_conn_reset_rx_state(conn);

  winfo("handle %u exiting\n", conn->handle);
  bt_conn_put(conn);
}

FAR struct bt_conn_s *bt_conn_add(FAR const bt_addr_le_t *peer,
                                  uint8_t role)
{
  FAR struct bt_conn_s *conn = NULL;
  int i;

  for (i = 0; i < ARRAY_SIZE(conns); i++)
    {
      if (!bt_addr_le_cmp(&conns[i].dst, BT_ADDR_LE_ANY))
        {
          conn = &conns[i];
          break;
        }
    }

  if (!conn)
    {
      return NULL;
    }

  memset(conn, 0, sizeof(*conn));

  bt_atomic_set(&conn->ref, 1);
  conn->role = role;
  bt_addr_le_copy(&conn->dst, peer);

  return conn;
}

void bt_conn_set_state(FAR struct bt_conn_s *conn,
                       enum bt_conn_state_e state)
{
  enum bt_conn_state_e old_state;

  winfo("%s -> %s\n", state2str(conn->state), state2str(state));

  if (conn->state == state)
    {
      wlwarn("no transition\n");
      return;
    }

  old_state   = conn->state;
  conn->state = state;

  /* Take a reference for the first state transition after bt_conn_add() and
   * keep it until reaching DISCONNECTED again.
   */

  if (old_state == BT_CONN_DISCONNECTED)
    {
      bt_conn_get(conn);
    }

  switch (conn->state)
    {
      case BT_CONN_CONNECTED:
        nano_fifo_init(&conn->tx_queue);
        fiber_start(conn->tx_stack, sizeof(conn->tx_stack),
                    conn_tx_fiber, (int)bt_conn_get(conn), 0, 7, 0);
        break;

      case BT_CONN_DISCONNECTED:
        /* Send dummy buffer to wake up and stop the tx fiber for states where it
         * was running.
         */

        if (old_state == BT_CONN_CONNECTED || old_state == BT_CONN_DISCONNECT)
          {
            nano_fifo_put(&conn->tx_queue, bt_buf_get(BT_DUMMY, 0));
          }

        /* Release the reference we took for the very first state transition. */

        bt_conn_put(conn);
        break;

      case BT_CONN_CONNECT_SCAN:
      case BT_CONN_CONNECT:
      case BT_CONN_DISCONNECT:
        break;

      default:
        wlwarn("no valid (%u) state was set\n", state);
        break;
    }
}

FAR struct bt_conn_s *bt_conn_lookup_handle(uint16_t handle)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(conns); i++)
    {
      /* We only care about connections with a valid handle */

      if (conns[i].state != BT_CONN_CONNECTED &&
          conns[i].state != BT_CONN_DISCONNECT)
        {
          continue;
        }

      if (conns[i].handle == handle)
        {
          return bt_conn_get(&conns[i]);
        }
    }

  return NULL;
}

FAR struct bt_conn_s *bt_conn_lookup_addr_le(FAR const bt_addr_le_t * peer)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(conns); i++)
    {
      if (!bt_addr_le_cmp(peer, &conns[i].dst))
        {
          return bt_conn_get(&conns[i]);
        }
    }

  return NULL;
}

FAR struct bt_conn_s *bt_conn_lookup_state(FAR const bt_addr_le_t * peer,
                                           enum bt_conn_state_e state)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(conns); i++)
    {
      if (!bt_addr_le_cmp(&conns[i].dst, BT_ADDR_LE_ANY))
        {
          continue;
        }

      if (bt_addr_le_cmp(peer, BT_ADDR_LE_ANY) &&
          bt_addr_le_cmp(peer, &conns[i].dst))
        {
          continue;
        }

      if (conns[i].state == state)
        {
          return bt_conn_get(&conns[i]);
        }
    }

  return NULL;
}

FAR struct bt_conn_s *bt_conn_get(FAR struct bt_conn_s *conn)
{
  bt_atomic_incr(&conn->ref);

  winfo("handle %u ref %u\n", conn->handle, bt_atomic_get(&conn->ref));

  return conn;
}

void bt_conn_put(FAR struct bt_conn_s *conn)
{
  bt_atomic_t old_ref;

  old_ref = bt_atomic_decr(&conn->ref);

  winfo("handle %u ref %u\n", conn->handle, bt_atomic_get(&conn->ref));

  if (old_ref > 1)
    {
      return;
    }

  bt_addr_le_copy(&conn->dst, BT_ADDR_LE_ANY);
}

const bt_addr_le_t *bt_conn_get_dst(FAR const struct bt_conn_s *conn)
{
  return &conn->dst;
}

int bt_conn_security(FAR struct bt_conn_s *conn, bt_security_t sec)
{
  FAR struct bt_keys_s *keys;

  if (conn->state != BT_CONN_CONNECTED)
    {
      return -ENOTCONN;
    }

  /* Nothing to do */

  if (sec == BT_SECURITY_LOW)
    {
      return 0;
    }

  /* For now we only support JustWorks */

  if (sec > BT_SECURITY_MEDIUM)
    {
      return -EINVAL;
    }

  if (conn->encrypt)
    {
      return 0;
    }

  if (conn->role == BT_HCI_ROLE_SLAVE)
    {
      return bt_smp_send_security_req(conn);
    }

  keys = bt_keys_find(BT_KEYS_LTK, &conn->dst);
  if (keys)
    {
      return bt_conn_le_start_encryption(conn, keys->ltk.rand,
                                         keys->ltk.ediv, keys->ltk.val);
    }

  return bt_smp_send_pairing_req(conn);
}

void bt_conn_set_auto_conn(FAR struct bt_conn_s *conn, bool auto_conn)
{
  if (auto_conn)
    {
      bt_atomic_set_bit(conn->flags, BT_CONN_AUTO_CONNECT);
    }
  else
    {
      bt_atomic_clrbit(conn->flags, BT_CONN_AUTO_CONNECT);
    }
}

static int bt_hci_disconnect(FAR struct bt_conn_s *conn, uint8_t reason)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_hci_cp_disconnect_s *disconn;
  int err;

  buf = bt_hci_cmd_create(BT_HCI_OP_DISCONNECT, sizeof(*disconn));
  if (!buf)
    {
      return -ENOBUFS;
    }

  disconn         = bt_buf_add(buf, sizeof(*disconn));
  disconn->handle = sys_cpu_to_le16(conn->handle);
  disconn->reason = reason;

  err = bt_hci_cmd_send(BT_HCI_OP_DISCONNECT, buf);
  if (err)
    {
      return err;
    }

  bt_conn_set_state(conn, BT_CONN_DISCONNECT);
  return 0;
}

static int bt_hci_connect_le_cancel(FAR struct bt_conn_s *conn)
{
  int err;

  err = bt_hci_cmd_send(BT_HCI_OP_LE_CREATE_CONN_CANCEL, NULL);
  if (err)
    {
      return err;
    }

  return 0;
}

int bt_conn_disconnect(FAR struct bt_conn_s *conn, uint8_t reason)
{
  /* Disconnection is initiated by us, so auto connection shall be disabled.
   * Otherwise the passive scan would be enabled and we could send LE Create
   * Connection as soon as the remote starts advertising.
   */

  bt_conn_set_auto_conn(conn, false);

  switch (conn->state)
    {
      case BT_CONN_CONNECT_SCAN:
        bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
        bt_le_scan_update();
        return 0;

      case BT_CONN_CONNECT:
        return bt_hci_connect_le_cancel(conn);

      case BT_CONN_CONNECTED:
        return bt_hci_disconnect(conn, reason);

      case BT_CONN_DISCONNECT:
        return 0;

      case BT_CONN_DISCONNECTED:
      default:
        return -ENOTCONN;
    }
}

FAR struct bt_conn_s *bt_conn_create_le(FAR const bt_addr_le_t *peer)
{
  FAR struct bt_conn_s *conn;

  conn = bt_conn_lookup_addr_le(peer);
  if (conn)
    {
      switch (conn->state)
        {
          case BT_CONN_CONNECT_SCAN:
          case BT_CONN_CONNECT:
          case BT_CONN_CONNECTED:
            return conn;

          default:
            bt_conn_put(conn);
            return NULL;
        }
    }

  conn = bt_conn_add(peer, BT_HCI_ROLE_MASTER);
  if (!conn)
    {
      return NULL;
    }

  bt_conn_set_state(conn, BT_CONN_CONNECT_SCAN);
  bt_le_scan_update();
  return conn;
}

int bt_conn_le_start_encryption(FAR struct bt_conn_s *conn, uint64_t rand,
                                uint16_t ediv, FAR const uint8_t *ltk)
{
  FAR struct bt_hci_cp_le_start_encryption_s *cp;
  FAR struct bt_buf_s *buf;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_START_ENCRYPTION, sizeof(*cp));
  if (!buf)
    {
      return -ENOBUFS;
    }

  cp         = bt_buf_add(buf, sizeof(*cp));
  cp->handle = sys_cpu_to_le16(conn->handle);
  cp->rand   = rand;
  cp->ediv   = ediv;
  memcpy(cp->ltk, ltk, sizeof(cp->ltk));

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_START_ENCRYPTION, buf, NULL);
}

int bt_conn_le_conn_update(FAR struct bt_conn_s *conn, uint16_t min,
                           uint16_t max, uint16_t latency, uint16_t timeout)
{
  FAR struct hci_cp_le_conn_update_s *conn_update;
  FAR struct bt_buf_s *buf;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_CONN_UPDATE, sizeof(*conn_update));
  if (!buf)
    {
      return -ENOBUFS;
    }

  conn_update                      = bt_buf_add(buf, sizeof(*conn_update));
  memset(conn_update, 0, sizeof(*conn_update));
  conn_update->handle              = sys_cpu_to_le16(conn->handle);
  conn_update->conn_interval_min   = sys_cpu_to_le16(min);
  conn_update->conn_interval_max   = sys_cpu_to_le16(max);
  conn_update->conn_latency        = sys_cpu_to_le16(latency);
  conn_update->supervision_timeout = sys_cpu_to_le16(timeout);

  return bt_hci_cmd_send(BT_HCI_OP_LE_CONN_UPDATE, buf);
}
