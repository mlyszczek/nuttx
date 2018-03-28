/****************************************************************************
 * wireless/bluetooth/bt_att.c
 * HCI core Bluetooth handling.
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
 *    and/or other materials provided with the distribution.
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

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wireless/bt_core.h>
#include <nuttx/wireless/bt_hci.h>

#include "bt_hcicore.h"
#include "bt_keys.h"
#include "bt_conn.h"
#include "bt_l2cap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many buffers to use for incoming ACL data */

#define ACL_IN_MAX   7
#define ACL_OUT_MAX  7

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum bt_stackdir_e
{
  STACK_DIRECTION_UP,
  STACK_DIRECTION_DOWN,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Stacks for the fibers */

static BT_STACK_NOINIT(g_rx_fiber_stack, 1024);
static BT_STACK_NOINIT(g_rx_prio_fiber_stack, 256);
static BT_STACK_NOINIT(g_cmd_tx_fiber_stack, 256);

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
static nano_context_id_t g_rx_prio_fiber_id;
#endif

struct bt_dev_s g_btdev;

static FAR struct bt_conn_cb_s *g_callback_list;
static bt_le_scan_cb_t *g_scan_dev_found_cb;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
static FAR const char *bt_addr_str(FAR const bt_addr_t *addr)
{
  static char bufs[2][18];
  static uint8_t cur;
  FAR char *str;

  str  = bufs[cur++];
  cur %= ARRAY_SIZE(bufs);
  bt_addr_to_str(addr, str, sizeof(bufs[cur]));

  return str;
}

static FAR const char *bt_addr_le_str(FAR const bt_addr_le_t *addr)
{
  static char bufs[2][27];
  static uint8_t cur;
  FAR char *str;

  str  = bufs[cur++];
  cur %= ARRAY_SIZE(bufs);
  bt_addr_le_to_str(addr, str, sizeof(bufs[cur]));

  return str;
}
#endif /* CONFIG_DEBUG_WIRELESS_INFO */

static void bt_connected(FAR struct bt_conn_s *conn)
{
  FAR struct bt_conn_s_cb_s *cb;

  for (cb = g_callback_list; cb; cb = cb->next)
    {
      if (cb->connected)
        {
          cb->connected(conn);
        }
    }
}

static void bt_disconnected(FAR struct bt_conn_s *conn)
{
  FAR struct bt_conn_s_cb_s *cb;

  for (cb = g_callback_list; cb; cb = cb->next)
    {
      if (cb->disconnected)
        {
          cb->disconnected(conn);
        }
    }
}

void bt_conn_cb_register(FAR struct bt_conn_s_cb_s *cb)
{
  cb->next        = g_callback_list;
  g_callback_list = cb;
}

FAR struct bt_buf_s *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len)
{
  FAR struct bt_hci_cmd_hdr_s *hdr;
  FAR struct bt_buf_s *buf;

  winfo("opcode %x param_len %u\n", opcode, param_len);

  buf = bt_buf_get(BT_CMD, g_btdev.drv->head_reserve);
  if (!buf)
    {
      wlerr("ERROR: Cannot get free buffer\n");
      return NULL;
    }

  winfo("buf %p\n", buf);

  buf->hci.opcode = opcode;
  buf->hci.sync   = NULL;

  hdr             = bt_buf_add(buf, sizeof(*hdr));
  hdr->opcode     = sys_cpu_to_le16(opcode);
  hdr->param_len  = param_len;

  return buf;
}

int bt_hci_cmd_send(uint16_t opcode, FAR struct bt_buf_s *buf)
{
  if (!buf)
    {
      buf = bt_hci_cmd_create(opcode, 0);
      if (!buf)
        {
          return -ENOBUFS;
        }
    }

  winfo("opcode %x len %u\n", opcode, buf->len);

  /* Host Number of Completed Packets can ignore the ncmd value and does not
   * generate any cmd complete/status events.
   */

  if (opcode == BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS)
    {
      g_btdev.drv->send(buf);
      bt_buf_put(buf);
      return 0;
    }

  nano_fifo_put(&g_btdev.cmd_tx_queue, buf);
  return 0;
}

int bt_hci_cmd_send_sync(uint16_t opcode, FAR struct bt_buf_s *buf,
                         FAR struct bt_buf_s **rsp)
{
  struct nano_sem sync_sem;
  int err;

  /* This function cannot be called from the rx fiber since it relies on the
   * very same fiber in processing the cmd_complete event and giving back the
   * blocking semaphore.
   */

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
  if (context_self_get() == g_rx_prio_fiber_id)
    {
      wlerr("ERROR: called from invalid context!\n");
      return -EDEADLK;
    }
#endif

  if (!buf)
    {
      buf = bt_hci_cmd_create(opcode, 0);
      if (!buf)
        {
          return -ENOBUFS;
        }
    }

  winfo("opcode %x len %u\n", opcode, buf->len);

  nano_sem_init(&sync_sem);
  buf->hci.sync = &sync_sem;

  nano_fifo_put(&g_btdev.cmd_tx_queue, buf);

  nano_sem_take_wait(&sync_sem);

  /* Indicate failure if we failed to get the return parameters */

  if (!buf->hci.sync)
    {
      err = -EIO;
    }
  else
    {
      err = 0;
    }

  if (rsp)
    {
      *rsp = buf->hci.sync;
    }
  else if (buf->hci.sync)
    {
      bt_buf_put(buf->hci.sync);
    }

  bt_buf_put(buf);
  return err;
}

static void hci_acl(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_acl_hdr *hdr = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle;
  uint16_t len = sys_le16_to_cpu(hdr->len);
  uint8_t flags;

  winfo("buf %p\n", buf);

  handle          = sys_le16_to_cpu(hdr->handle);
  flags           = (handle >> 12);
  buf->acl.handle = bt_acl_handle(handle);

  bt_buf_pull(buf, sizeof(*hdr));

  winfo("handle %u len %u flags %u\n", buf->acl.handle, len, flags);

  if (buf->len != len)
    {
      wlerr("ERROR: ACL data length mismatch (%u != %u)\n", buf->len, len);
      bt_buf_put(buf);
      return;
    }

  conn = bt_conn_lookup_handle(buf->acl.handle);
  if (!conn)
    {
      wlerr("ERROR: Unable to find conn for handle %u\n", buf->acl.handle);
      bt_buf_put(buf);
      return;
    }

  bt_conn_recv(conn, buf, flags);
  bt_conn_put(conn);
}

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
static void analyze_stack(FAR const char *name, FAR const char *stack,
                          unsigned size, int stack_growth)
{
  unsigned stack_offset, pcnt, unused = 0;
  unsigned pcnt;
  unsigned unused = 0;
  unsigned i;

  /* The CCS is always placed on a 4-byte aligned boundary - if the stack
   * beginning doesn't match that there will be some unused bytes in the
   * beginning.
   */

  stack_offset = __tCCS_SIZEOF + ((4 - ((unsigned)stack % 4)) % 4);

  if (stack_growth == STACK_DIRECTION_DOWN)
    {
      for (i = stack_offset; i < size; i++)
        {
          if ((unsigned char)stack[i] == 0xaa)
            {
              unused++;
            }
          else
            {
              break;
            }
        }
    }
  else
    {
      for (i = size - 1; i >= stack_offset; i--)
        {
          if ((unsigned char)stack[i] == 0xaa)
            {
              unused++;
            }
          else
            {
              break;
            }
        }
    }

  /* Calculate the real size reserved for the stack */

  size -= stack_offset;
  pcnt  = ((size - unused) * 100) / size;

  wlinfo("%s (real size %u):\tunused %u\tusage %u / %u (%u %%)\n",
         name, size + stack_offset, unused, size - unused, size, pcnt);
}

static void analyze_stacks(FAR struct bt_conn_s *conn,
                           FAR struct bt_conn_s **ref)
{
  int stack_growth;

  wlinfo("sizeof(tCCS) = %u\n", __tCCS_SIZEOF);

  if (conn > *ref)
    {
      wlinfo("stack grows up\n");
      stack_growth = STACK_DIRECTION_UP;
    }
  else
    {
      wlinfo("stack grows down\n");
      stack_growth = STACK_DIRECTION_DOWN;
    }

  analyze_stack("rx stack", g_rx_fiber_stack, sizeof(g_rx_fiber_stack),
                stack_growth);
  analyze_stack("cmd rx stack", g_rx_prio_fiber_stack,
                sizeof(g_rx_prio_fiber_stack), stack_growth);
  analyze_stack("cmd tx stack", g_cmd_tx_fiber_stack,
                sizeof(g_cmd_tx_fiber_stack), stack_growth);
  analyze_stack("conn tx stack", conn->tx_stack, sizeof(conn->tx_stack),
                stack_growth);
}
#else
#  define analyze_stacks(...)
#endif

/* HCI event processing */

static void hci_encrypt_change(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_encrypt_change_s *evt = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle = sys_le16_to_cpu(evt->handle);

  winfo("status %u handle %u encrypt 0x%02x\n",
        evt->status, handle, evt->encrypt);

  if (evt->status)
    {
      return;
    }

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR: Unable to look up conn with handle %u\n", handle);
      return;
    }

  conn->encrypt = evt->encrypt;

  bt_l2cap_encrypt_change(conn);
  bt_conn_put(conn);
}

static void hci_reset_complete(FAR struct bt_buf_s *buf)
{
  uint8_t status = buf->data[0];

  winfo("status %u\n", status);

  if (status)
    {
      return;
    }

  g_scan_dev_found_cb = NULL;
  g_btdev.scan_enable = BT_LE_SCAN_DISABLE;
  g_btdev.scan_filter = BT_LE_SCAN_FILTER_DUP_ENABLE;
}

static void hci_cmd_done(uint16_t opcode, uint8_t status,
                         FAR struct bt_buf_s *buf)
{
  FAR struct bt_buf_s *sent = g_btdev.sent_cmd;

  if (!sent)
    {
      return;
    }

  if (g_btdev.sent_cmd->hci.opcode != opcode)
    {
      wlerr("ERROR: Unexpected completion of opcode 0x%04x\n", opcode);
      return;
    }

  g_btdev.sent_cmd = NULL;

  /* If the command was synchronous wake up bt_hci_cmd_send_sync() */

  if (sent->hci.sync)
    {
      FAR struct nano_sem_s *sem = sent->hci.sync;

      if (status)
        {
          sent->hci.sync = NULL;
        }
      else
        {
          sent->hci.sync = bt_buf_hold(buf);
        }

      nano_fiber_sem_give(sem);
    }
  else
    {
      bt_buf_put(sent);
    }
}

static void hci_cmd_complete(FAR struct bt_buf_s *buf)
{
  FAR struct hci_evt_cmd_complete *evt = (FAR void *)buf->data;
  uint16_t opcode = sys_le16_to_cpu(evt->opcode);
  FAR uint8_t *status;

  winfo("opcode %x\n", opcode);

  bt_buf_pull(buf, sizeof(*evt));

  /* All command return parameters have a 1-byte status in the beginning, so we
   * can safely make this generalization.
   */

  status = buf->data;

  switch (opcode)
    {
      case BT_HCI_OP_RESET:
        hci_reset_complete(buf);
        break;

      default:
        winfo("Unhandled opcode %x\n", opcode);
        break;
    }

  hci_cmd_done(opcode, *status, buf);

  if (evt->ncmd && !g_btdev.ncmd)
    {
      /* Allow next command to be sent */

      g_btdev.ncmd = 1;
      nano_fiber_sem_give(&g_btdev.ncmd_sem);
    }
}

static void hci_cmd_status(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_cmd_status_s *evt = (FAR void *)buf->data;
  uint16_t opcode = sys_le16_to_cpu(evt->opcode);

  winfo("opcode %x\n", opcode);

  bt_buf_pull(buf, sizeof(*evt));

  switch (opcode)
    {
      default:
        winfo("Unhandled opcode %x\n", opcode);
        break;
    }

  hci_cmd_done(opcode, evt->status, buf);

  if (evt->ncmd && !g_btdev.ncmd)
    {
      /* Allow next command to be sent */

      g_btdev.ncmd = 1;
      nano_fiber_sem_give(&g_btdev.ncmd_sem);
    }
}

static void hci_num_completed_packets(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_num_completed_packets_s *evt = (FAR void *)buf->data;
  uint16_t i, num_handles = sys_le16_to_cpu(evt->num_handles);

  winfo("num_handles %u\n", num_handles);

  for (i = 0; i < num_handles; i++)
    {
      uint16_t handle, count;

      handle = sys_le16_to_cpu(evt->h[i].handle);
      count  = sys_le16_to_cpu(evt->h[i].count);

      winfo("handle %u count %u\n", handle, count);

      while (count--)
        {
          nano_fiber_sem_give(&g_btdev.le_pkts_sem);
        }
    }
}

static void hci_encrypt_key_refresh_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_encrypt_key_refresh_complete_s *evt = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle;

  handle = sys_le16_to_cpu(evt->handle);

  winfo("status %u handle %u\n", evt->status, handle);

  if (evt->status)
    {
      return;
    }

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR: Unable to look up conn with handle %u\n", handle);
      return;
    }

  bt_l2cap_encrypt_change(conn);
  bt_conn_put(conn);
}

static void copy_id_addr(FAR struct bt_conn_s *conn,
                         FAR const bt_addr_le_t *addr)
{
  FAR struct bt_keys_s *keys;

  /* If we have a keys struct we already know the identity */

  if (conn->keys)
    {
      return;
    }

  keys = bt_keys_find_irk(addr);
  if (keys)
    {
      bt_addr_le_copy(&conn->dst, &keys->addr);
      conn->keys = keys;
    }
  else
    {
      bt_addr_le_copy(&conn->dst, addr);
    }
}

static int bt_hci_start_scanning(uint8_t scan_type, uint8_t scan_filter)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_buf_s *rsp;
  FAR struct bt_hci_cp_le_set_scan_params_s *set_param;
  FAR struct bt_hci_cp_le_set_scan_enable_s *scan_enable;
  int err;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_PARAMS, sizeof(*set_param));
  if (!buf)
    {
      return -ENOBUFS;
    }

  set_param = bt_buf_add(buf, sizeof(*set_param));
  memset(set_param, 0, sizeof(*set_param));
  set_param->scan_type = scan_type;

  /* for the rest parameters apply default values according to spec 4.2, vol2,
   * part E, 7.8.10
   */

  set_param->interval      = sys_cpu_to_le16(0x0010);
  set_param->window        = sys_cpu_to_le16(0x0010);
  set_param->filter_policy = 0x00;
  set_param->addr_type     = 0x00;

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_SCAN_PARAMS, buf);
  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_ENABLE, sizeof(*scan_enable));
  if (!buf)
    {
      return -ENOBUFS;
    }

  scan_enable             = bt_buf_add(buf, sizeof(*scan_enable));
  memset(scan_enable, 0, sizeof(*scan_enable));
  scan_enable->filter_dup = scan_filter;
  scan_enable->enable     = BT_LE_SCAN_ENABLE;

  err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_SCAN_ENABLE, buf, &rsp);
  if (err)
    {
      return err;
    }

  /* Update scan state in case of success (0) status */

  err = rsp->data[0];
  if (!err)
    {
      g_btdev.scan_enable = BT_LE_SCAN_ENABLE;
    }

  bt_buf_put(rsp);
  return err;
}

static int bt_hci_stop_scanning(void)
{
  FAR struct bt_buf_s *buf, *rsp;
  FAR struct bt_hci_cp_le_set_scan_enable_s *scan_enable;
  int err;

  if (g_btdev.scan_enable == BT_LE_SCAN_DISABLE)
    {
      return -EALREADY;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_ENABLE,
                          sizeof(*scan_enable));
  if (!buf)
    {
      return -ENOBUFS;
    }

  scan_enable             = bt_buf_add(buf, sizeof(*scan_enable));
  memset(scan_enable, 0x0, sizeof(*scan_enable));
  scan_enable->filter_dup = 0x00;
  scan_enable->enable     = BT_LE_SCAN_DISABLE;

  err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_SCAN_ENABLE, buf, &rsp);
  if (err)
    {
      return err;
    }

  /* Update scan state in case of success (0) status */

  err = rsp->data[0];
  if (!err)
    {
      g_btdev.scan_enable = BT_LE_SCAN_DISABLE;
    }

  bt_buf_put(rsp);
  return err;
}

static int hci_le_create_conn(FAR const bt_addr_le_t *addr)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_hci_cp_le_create_conn_s *cp;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_CREATE_CONN, sizeof(*cp));
  if (!buf)
    {
      return -ENOBUFS;
    }

  cp                      = bt_buf_add(buf, sizeof(*cp));
  memset(cp, 0x0, sizeof(*cp));
  bt_addr_le_copy(&cp->peer_addr, addr);
  cp->conn_interval_max   = sys_cpu_to_le16(0x0028);
  cp->conn_interval_min   = sys_cpu_to_le16(0x0018);
  cp->scan_interval       = sys_cpu_to_le16(0x0060);
  cp->scan_window         = sys_cpu_to_le16(0x0030);
  cp->supervision_timeout = sys_cpu_to_le16(0x07D0);

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_CREATE_CONN, buf, NULL);
}

/* Used to determine whether to start scan and which scan type should be used */

int bt_le_scan_update(void)
{
  FAR struct bt_conn_s *conn;

  if (g_btdev.scan_enable)
    {
      int err;

      if (g_scan_dev_found_cb)
        {
          return 0;
        }

      err = bt_hci_stop_scanning();
      if (err)
        {
          return err;
        }
    }

  if (g_scan_dev_found_cb)
    {
      return bt_hci_start_scanning(BT_LE_SCAN_ACTIVE, g_btdev.scan_filter);
    }

  conn = bt_conn_lookup_state(BT_ADDR_LE_ANY, BT_CONN_CONNECT_SCAN);
  if (!conn)
    {
      return 0;
    }

  bt_conn_put(conn);

  return bt_hci_start_scanning(BT_LE_SCAN_PASSIVE, g_btdev.scan_filter);
}

static void hci_disconn_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_disconn_complete_s *evt = (FAR void *)buf->data;
  uint16_t handle = sys_le16_to_cpu(evt->handle);
  FAR struct bt_conn_s *conn;

  winfo("status %u handle %u reason %u\n", evt->status, handle, evt->reason);

  if (evt->status)
    {
      return;
    }

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR: Unable to look up conn with handle %u\n", handle);
      return;
    }

  bt_l2cap_disconnected(conn);
  bt_disconnected(conn);

  /* Check stack usage (no-op if not enabled) */

  analyze_stacks(conn, &conn);

  bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
  conn->handle = 0;

  if (bt_atomic_testbit(conn->flags, BT_CONN_AUTO_CONNECT))
    {
      bt_conn_set_state(conn, BT_CONN_CONNECT_SCAN);
      bt_le_scan_update();
    }

  bt_conn_put(conn);

  if (g_btdev.adv_enable)
    {
      FAR struct bt_buf_s *buf;

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
      if (buf)
        {
          memcpy(bt_buf_add(buf, 1), &g_btdev.adv_enable, 1);
          bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_ENABLE, buf);
        }
    }
}

static void le_conn_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_le_conn_complete_s *evt = (FAR void *)buf->data;
  uint16_t handle = sys_le16_to_cpu(evt->handle);
  FAR struct bt_conn_s *conn;
  FAR struct bt_keys_s *keys;

  winfo("status %u handle %u role %u %s\n", evt->status, handle,
         evt->role, bt_addr_le_str(&evt->peer_addr));

  /* Make lookup to check if there's a connection object in CONNECT state
   * associated with passed peer LE address.
   */

  keys = bt_keys_find_irk(&evt->peer_addr);
  if (keys)
    {
      conn = bt_conn_lookup_state(&keys->addr, BT_CONN_CONNECT);
    }
  else
    {
      conn = bt_conn_lookup_state(&evt->peer_addr, BT_CONN_CONNECT);
    }

  if (evt->status)
    {
      if (!conn)
        {
          return;
        }

      bt_conn_set_state(conn, BT_CONN_DISCONNECTED);

      /* Drop the reference got by lookup call in CONNECT state. We are now in
       * DISCONNECTED state since no successful LE link been made.
       */

      bt_conn_put(conn);
      return;
    }

  if (!conn)
    {
      conn = bt_conn_add(&evt->peer_addr, evt->role);
    }

  if (!conn)
    {
      wlerr("ERROR: Unable to add new conn for handle %u\n", handle);
      return;
    }

  conn->handle           = handle;
  conn->src.type         = BT_ADDR_LE_PUBLIC;
  memcpy(conn->src.val, g_btdev.bdaddr.val, sizeof(g_btdev.bdaddr.val));
  copy_id_addr(conn, &evt->peer_addr);
  conn->le_conn_interval = sys_le16_to_cpu(evt->interval);

  bt_conn_set_state(conn, BT_CONN_CONNECTED);

  bt_l2cap_connected(conn);

  if (evt->role == BT_HCI_ROLE_SLAVE)
    {
      bt_l2cap_update_conn_param(conn);
    }

  bt_connected(conn);
  bt_conn_put(conn);
  bt_le_scan_update();
}

static void check_pending_conn(FAR const bt_addr_le_t *addr, uint8_t evtype,
                               FAR struct bt_keys_s *keys)
{
  FAR struct bt_conn_s *conn;

  /* Return if event is not connectible */

  if (evtype != BT_LE_ADV_IND && evtype != BT_LE_ADV_DIRECT_IND)
    {
      return;
    }

  if (keys)
    {
      conn = bt_conn_lookup_state(&keys->addr, BT_CONN_CONNECT_SCAN);
    }
  else
    {
      conn = bt_conn_lookup_state(addr, BT_CONN_CONNECT_SCAN);
    }

  if (!conn)
    {
      return;
    }

  if (bt_hci_stop_scanning())
    {
      goto done;
    }

  if (hci_le_create_conn(addr))
    {
      goto done;
    }

  bt_conn_set_state(conn, BT_CONN_CONNECT);

done:
  bt_conn_put(conn);
}

static void le_adv_report(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_ev_le_advertising_info_s *info;
  uint8_t num_reports = buf->data[0];

  winfo("Adv number of reports %u\n", num_reports);

  info = bt_buf_pull(buf, sizeof(num_reports));

  while (num_reports--)
    {
      int8_t rssi = info->data[info->length];
      FAR struct bt_keys_s *keys;
      bt_addr_le_t addr;

      winfo("%s event %u, len %u, rssi %d dBm\n",
             bt_addr_le_str(&info->addr), info->evt_type, info->length,
             rssi);

      keys = bt_keys_find_irk(&info->addr);
      if (keys)
        {
          bt_addr_le_copy(&addr, &keys->addr);
          winfo("Identity %s matched RPA %s\n",
                 bt_addr_le_str(&keys->addr), bt_addr_le_str(&info->addr));
        }
      else
        {
          bt_addr_le_copy(&addr, &info->addr);
        }

      if (g_scan_dev_found_cb)
        {
          g_scan_dev_found_cb(&addr, rssi, info->evt_type,
                            info->data, info->length);
        }

      check_pending_conn(&info->addr, info->evt_type, keys);

      /* Get next report iteration by moving pointer to right offset in buf
       * according to spec 4.2, Vol 2, Part E, 7.7.65.2.
       */

      info = bt_buf_pull(buf, sizeof(*info) + info->length + sizeof(rssi));
    }
}

static void le_ltk_request(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_le_ltk_request_s *evt = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle;

  handle = sys_le16_to_cpu(evt->handle);

  winfo("handle %u\n", handle);

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR: Unable to lookup conn for handle %u\n", handle);
      return;
    }

  if (!conn->keys)
    {
      conn->keys = bt_keys_find(BT_KEYS_SLAVE_LTK, &conn->dst);
    }

  if (conn->keys && (conn->keys->keys & BT_KEYS_SLAVE_LTK) &&
      conn->keys->slave_ltk.rand == evt->rand &&
      conn->keys->slave_ltk.ediv == evt->ediv)
    {
      FAR struct bt_hci_cp_le_ltk_req_reply_s *cp;

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_LTK_REQ_REPLY, sizeof(*cp));
      if (!buf)
        {
          wlerr("ERROR: Out of command buffers\n");
          goto done;
        }

      cp         = bt_buf_add(buf, sizeof(*cp));
      cp->handle = evt->handle;
      memcpy(cp->ltk, conn->keys->slave_ltk.val, 16);

      bt_hci_cmd_send(BT_HCI_OP_LE_LTK_REQ_REPLY, buf);
    }
  else
    {
      FAR struct bt_hci_cp_le_ltk_req_neg_reply_s *cp;

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_LTK_REQ_NEG_REPLY, sizeof(*cp));
      if (!buf)
        {
          wlerr("ERROR: Out of command buffers\n");
          goto done;
        }

      cp         = bt_buf_add(buf, sizeof(*cp));
      cp->handle = evt->handle;

      bt_hci_cmd_send(BT_HCI_OP_LE_LTK_REQ_NEG_REPLY, buf);
    }

done:
  bt_conn_put(conn);
}

static void hci_le_meta_event(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_le_meta_event_s *evt = (FAR void *)buf->data;

  bt_buf_pull(buf, sizeof(*evt));

  switch (evt->subevent)
    {
      case BT_HCI_EVT_LE_CONN_COMPLETE:
        le_conn_complete(buf);
        break;

      case BT_HCI_EVT_LE_ADVERTISING_REPORT:
        le_adv_report(buf);
        break;

      case BT_HCI_EVT_LE_LTK_REQUEST:
        le_ltk_request(buf);
        break;

      default:
        winfo("Unhandled LE event %x\n", evt->subevent);
        break;
    }
}

static void hci_event(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_hdr_s *hdr = (FAR void *)buf->data;

  winfo("event %u\n", hdr->evt);

  bt_buf_pull(buf, sizeof(*hdr));

  switch (hdr->evt)
    {
      case BT_HCI_EVT_DISCONN_COMPLETE:
        hci_disconn_complete(buf);
        break;

      case BT_HCI_EVT_ENCRYPT_CHANGE:
        hci_encrypt_change(buf);
        break;

      case BT_HCI_EVT_ENCRYPT_KEY_REFRESH_COMPLETE:
        hci_encrypt_key_refresh_complete(buf);
        break;

      case BT_HCI_EVT_LE_META_EVENT:
        hci_le_meta_event(buf);
        break;

      default:
        wlwarn("Unhandled event 0x%02x\n", hdr->evt);
        break;
    }

  bt_buf_put(buf);
}

static void hci_cmd_tx_fiber(void)
{
  FAR struct bt_driver_s *drv = g_btdev.drv;

  winfo("started\n");

  while (1)
    {
      FAR struct bt_buf_s *buf;

      /* Wait until ncmd > 0 */

      winfo("calling sem_take_wait\n");
      nano_fiber_sem_take_wait(&g_btdev.ncmd_sem);

      /* Get next command - wait if necessary */

      winfo("calling fifo_get_wait\n");
      buf = nano_fifo_get_wait(&g_btdev.cmd_tx_queue);
      g_btdev.ncmd = 0;

      winfo("Sending command %x (buf %p) to driver\n", buf->hci.opcode, buf);

      drv->send(buf);

      /* Clear out any existing sent command */

      if (g_btdev.sent_cmd)
        {
          wlerr("ERROR: Uncleared pending sent_cmd\n");
          bt_buf_put(g_btdev.sent_cmd);
          g_btdev.sent_cmd = NULL;
        }

      g_btdev.sent_cmd = buf;
    }
}

static void hci_rx_fiber(void)
{
  FAR struct bt_buf_s *buf;

  winfo("started\n");

  while (1)
    {
      winfo("calling fifo_get_wait\n");
      buf = nano_fifo_get_wait(&g_btdev.rx_queue);

      winfo("buf %p type %u len %u\n", buf, buf->type, buf->len);

      switch (buf->type)
        {
          case BT_ACL_IN:
            hci_acl(buf);
            break;

          case BT_EVT:
            hci_event(buf);
            break;

          default:
            wlerr("ERROR: Unknown buf type %u\n", buf->type);
            bt_buf_put(buf);
            break;
        }

    }
}

static void rx_prio_fiber(void)
{
  FAR struct bt_buf_s *buf;

  winfo("started\n");

  /* So we can avoid bt_hci_cmd_send_sync deadlocks */

#if defined(CONFIG_DEBUG_WIRELESS_INFO)
  g_rx_prio_fiber_id = context_self_get();
#endif

  while (1)
    {
      struct bt_hci_evt_hdr_s *hdr;

      winfo("calling fifo_get_wait\n");
      buf = nano_fifo_get_wait(&g_btdev.rx_prio_queue);

      winfo("buf %p type %u len %u\n", buf, buf->type, buf->len);

      if (buf->type != BT_EVT)
        {
          wlerr("ERROR: Unknown buf type %u\n", buf->type);
          bt_buf_put(buf);
          continue;
        }

      hdr = (FAR void *)buf->data;
      bt_buf_pull(buf, sizeof(*hdr));

      switch (hdr->evt)
        {
          case BT_HCI_EVT_CMD_COMPLETE:
            hci_cmd_complete(buf);
            break;

          case BT_HCI_EVT_CMD_STATUS:
            hci_cmd_status(buf);
           break;

          case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
            hci_num_completed_packets(buf);
            break;

          default:
            wlerr("ERROR: Unknown event 0x%02x\n", hdr->evt);
            break;
        }

      bt_buf_put(buf);
    }
}

static void read_local_features_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_local_features_s *rp = (FAR void *)buf->data;

  winfo("status %u\n", rp->status);

  memcpy(g_btdev.features, rp->features, sizeof(g_btdev.features));
}

static void read_local_ver_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_local_version_info_s *rp = (FAR void *)buf->data;

  winfo("status %u\n", rp->status);

  g_btdev.hci_version  = rp->hci_version;
  g_btdev.hci_revision = sys_le16_to_cpu(rp->hci_revision);
  g_btdev.manufacturer = sys_le16_to_cpu(rp->manufacturer);
}

static void read_bdaddr_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_bd_addr_s *rp = (FAR void *)buf->data;

  winfo("status %u\n", rp->status);

  bt_addr_copy(&g_btdev.bdaddr, &rp->bdaddr);
}

static void read_le_features_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_le_read_local_features_s *rp = (FAR void *)buf->data;

  winfo("status %u\n", rp->status);

  memcpy(g_btdev.le_features, rp->features, sizeof(g_btdev.le_features));
}

static void read_buffer_size_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_buffer_size_s *rp = (FAR void *)buf->data;

  winfo("status %u\n", rp->status);

  /* If LE-side has buffers we can ignore the BR/EDR values */

  if (g_btdev.le_mtu)
    {
      return;
    }

  g_btdev.le_mtu  = sys_le16_to_cpu(rp->acl_max_len);
  g_btdev.le_pkts = sys_le16_to_cpu(rp->acl_max_num);
}

static void le_read_buffer_size_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_le_read_buffer_size_s *rp = (FAR void *)buf->data;

  winfo("status %u\n", rp->status);

  g_btdev.le_mtu  = sys_le16_to_cpu(rp->le_max_len);
  g_btdev.le_pkts = rp->le_max_num;
}

static int hci_init(void)
{
  FAR struct bt_hci_cp_host_buffer_size_s *hbs;
  FAR struct bt_hci_cp_set_event_mask_s *ev;
  FAR struct bt_buf_s *buf;
  FAR struct bt_buf_s *rsp;
  FAR uint8_t *enable;
  int i, err;

  /* Send HCI_RESET */

  bt_hci_cmd_send(BT_HCI_OP_RESET, NULL);

  /* Read Local Supported Features */

  err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_LOCAL_FEATURES, NULL, &rsp);
  if (err)
    {
      return err;
    }

  read_local_features_complete(rsp);
  bt_buf_put(rsp);

  /* Read Local Version Information */

  err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_LOCAL_VERSION_INFO, NULL, &rsp);
  if (err)
    {
      return err;
    }

  read_local_ver_complete(rsp);
  bt_buf_put(rsp);

  /* Read Bluetooth Address */

  err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_BD_ADDR, NULL, &rsp);
  if (err)
    {
      return err;
    }

  read_bdaddr_complete(rsp);
  bt_buf_put(rsp);

  /* For now we only support LE capable controllers */

  if (!lmp_le_capable(g_btdev))
    {
      wlerr("ERROR: Non-LE capable controller detected!\n");
      return -ENODEV;
    }

  /* Read Low Energy Supported Features */

  err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_LOCAL_FEATURES, NULL, &rsp);
  if (err)
    {
      return err;
    }

  read_le_features_complete(rsp);
  bt_buf_put(rsp);

  /* Read LE Buffer Size */

  err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_BUFFER_SIZE, NULL, &rsp);
  if (err)
    {
      return err;
    }

  le_read_buffer_size_complete(rsp);
  bt_buf_put(rsp);

  buf = bt_hci_cmd_create(BT_HCI_OP_SET_EVENT_MASK, sizeof(*ev));
  if (!buf)
    {
      return -ENOBUFS;
    }

  ev = bt_buf_add(buf, sizeof(*ev));
  memset(ev, 0, sizeof(*ev));
  ev->events[0] |= 0x10;        /* Disconnection Complete */
  ev->events[1] |= 0x08;        /* Read Remote Version Information Complete */
  ev->events[1] |= 0x20;        /* Command Complete */
  ev->events[1] |= 0x40;        /* Command Status */
  ev->events[1] |= 0x80;        /* Hardware Error */
  ev->events[2] |= 0x04;        /* Number of Completed Packets */
  ev->events[3] |= 0x02;        /* Data Buffer Overflow */
  ev->events[7] |= 0x20;        /* LE Meta-Event */

  if (g_btdev.le_features[0] & BT_HCI_LE_ENCRYPTION)
    {
      ev->events[0] |= 0x80;    /* Encryption Change */
      ev->events[5] |= 0x80;    /* Encryption Key Refresh Complete */
    }

  bt_hci_cmd_send_sync(BT_HCI_OP_SET_EVENT_MASK, buf, NULL);

  buf = bt_hci_cmd_create(BT_HCI_OP_HOST_BUFFER_SIZE, sizeof(*hbs));
  if (!buf)
    {
      return -ENOBUFS;
    }

  hbs = bt_buf_add(buf, sizeof(*hbs));
  memset(hbs, 0, sizeof(*hbs));
  hbs->acl_mtu = sys_cpu_to_le16(BT_BUF_MAX_DATA -
                                 sizeof(struct bt_hci_acl_hdr) -
                                 g_btdev.drv->head_reserve);
  hbs->acl_pkts = sys_cpu_to_le16(ACL_IN_MAX);

  err = bt_hci_cmd_send(BT_HCI_OP_HOST_BUFFER_SIZE, buf);
  if (err)
    {
      return err;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_SET_CTL_TO_HOST_FLOW, 1);
  if (!buf)
    {
      return -ENOBUFS;
    }

  enable  = bt_buf_add(buf, sizeof(*enable));
  *enable = 0x01;

  err = bt_hci_cmd_send_sync(BT_HCI_OP_SET_CTL_TO_HOST_FLOW, buf, NULL);
  if (err)
    {
      return err;
    }

  if (lmp_bredr_capable(g_btdev))
    {
      struct bt_hci_cp_write_le_host_supp *cp;

      /* Use BR/EDR buffer size if LE reports zero buffers */

      if (!g_btdev.le_mtu)
        {
          err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_BUFFER_SIZE, NULL, &rsp);
          if (err)
            {
              return err;
            }

          read_buffer_size_complete(rsp);
          bt_buf_put(rsp);
        }

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_WRITE_LE_HOST_SUPP, sizeof(*cp));
      if (!buf)
        {
          return -ENOBUFS;
        }

      /* Explicitly enable LE for dual-mode controllers */

      cp        = bt_buf_add(buf, sizeof *cp);
      cp->le    = 0x01;
      cp->simul = 0x00;

      bt_hci_cmd_send_sync(BT_HCI_OP_LE_WRITE_LE_HOST_SUPP, buf, NULL);
    }

  winfo("HCI ver %u rev %u, manufacturer %u\n", g_btdev.hci_version,
        g_btdev.hci_revision, g_btdev.manufacturer);
  winfo("ACL buffers: pkts %u mtu %u\n", g_btdev.le_pkts, g_btdev.le_mtu);

  /* Initialize & prime the semaphore for counting controller-side available
   * ACL packet buffers.
   */

  nano_sem_init(&g_btdev.le_pkts_sem);
  for (i = 0; i < g_btdev.le_pkts; i++)
    {
      nano_sem_give(&g_btdev.le_pkts_sem);
    }

  return 0;
}

/* fibers, fifos and semaphores initialization */

static void cmd_queue_init(void)
{
  nano_fifo_init(&g_btdev.cmd_tx_queue);
  nano_sem_init(&g_btdev.ncmd_sem);

  /* Give cmd_sem allowing to send first HCI_Reset cmd */

  g_btdev.ncmd = 1;
  nano_task_sem_give(&g_btdev.ncmd_sem);

  fiber_start(g_cmd_tx_fiber_stack, sizeof(g_cmd_tx_fiber_stack),
              (nano_fiber_entry_t) hci_cmd_tx_fiber, 0, 0, 7, 0);
}

static void rx_queue_init(void)
{
  nano_fifo_init(&g_btdev.rx_queue);
  fiber_start(g_rx_fiber_stack, sizeof(g_rx_fiber_stack),
              (nano_fiber_entry_t) hci_rx_fiber, 0, 0, 7, 0);

  nano_fifo_init(&g_btdev.rx_prio_queue);
  fiber_start(g_rx_prio_fiber_stack, sizeof(g_rx_prio_fiber_stack),
              (nano_fiber_entry_t) rx_prio_fiber, 0, 0, 7, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Interface to HCI driver layer */

void bt_recv(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_hdr_s *hdr;

  winfo("buf %p len %u\n", buf, buf->len);

  if (buf->type == BT_ACL_IN)
    {
      nano_fifo_put(&g_btdev.rx_queue, buf);
      return;
    }

  if (buf->type != BT_EVT)
    {
      wlerr("ERROR: Invalid buf type %u\n", buf->type);
      bt_buf_put(buf);
      return;
    }

  /* Command Complete/Status events have their own cmd_rx queue, all other
   * events go through rx queue.
   */

  hdr = (FAR void *)buf->data;
  if (hdr->evt == BT_HCI_EVT_CMD_COMPLETE ||
      hdr->evt == BT_HCI_EVT_CMD_STATUS ||
      hdr->evt == BT_HCI_EVT_NUM_COMPLETED_PACKETS)
    {
      nano_fifo_put(&g_btdev.rx_prio_queue, buf);
      return;
    }

  nano_fifo_put(&g_btdev.rx_queue, buf);
}

int bt_driver_register(FAR struct bt_driver_s *drv)
{
  if (g_btdev.drv)
    {
      return -EALREADY;
    }

  if (!drv->open || !drv->send)
    {
      return -EINVAL;
    }

  g_btdev.drv = drv;
  return 0;
}

void bt_driver_unregister(FAR struct bt_driver_s *drv)
{
  g_btdev.drv = NULL;
}

int bt_init(void)
{
  FAR struct bt_driver_s *drv = g_btdev.drv;
  int err;

  if (!drv)
    {
      wlerr("ERROR: No HCI driver registered\n");
      return -ENODEV;
    }

  bt_buf_init(ACL_IN_MAX, ACL_OUT_MAX);

  cmd_queue_init();
  rx_queue_init();

  err = drv->open();
  if (err)
    {
      wlerr("ERROR: HCI driver open failed (%d)\n", err);
      return err;
    }

  err = hci_init();
  if (err)
    {
      return err;
    }

  return bt_l2cap_init();
}

int bt_start_advertising(uint8_t type, FAR const struct bt_eir_s *ad,
                         FAR const struct bt_eir_s *sd)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_hci_cp_le_set_adv_data_s *set_data;
  FAR struct bt_hci_cp_le_set_adv_data_s *scan_rsp;
  FAR struct bt_hci_cp_le_set_adv_parameters_s *set_param;
  int i;

  if (!ad)
    {
      goto send_scan_rsp;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_DATA, sizeof(*set_data));
  if (!buf)
    {
      return -ENOBUFS;
    }

  set_data = bt_buf_add(buf, sizeof(*set_data));

  memset(set_data, 0, sizeof(*set_data));

  for (i = 0; ad[i].len; i++)
    {
      /* Check if ad fit in the remaining buffer */

      if (set_data->len + ad[i].len + 1 > 29)
        {
          break;
        }

      memcpy(&set_data->data[set_data->len], &ad[i], ad[i].len + 1);
      set_data->len += ad[i].len + 1;
    }

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_DATA, buf);

send_scan_rsp:
  if (!sd)
    {
      goto send_set_param;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_RSP_DATA,
                          sizeof(*scan_rsp));
  if (!buf)
    {
      return -ENOBUFS;
    }

  scan_rsp = bt_buf_add(buf, sizeof(*scan_rsp));

  memset(scan_rsp, 0, sizeof(*scan_rsp));

  for (i = 0; sd[i].len; i++)
    {
      /* Check if ad fit in the remaining buffer */

      if (scan_rsp->len + sd[i].len + 1 > 29)
        {
          break;
        }

      memcpy(&scan_rsp->data[scan_rsp->len], &sd[i], sd[i].len + 1);
      scan_rsp->len += sd[i].len + 1;
    }

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_SCAN_RSP_DATA, buf);

send_set_param:
  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_PARAMETERS,
                          sizeof(*set_param));
  if (!buf)
    {
      return -ENOBUFS;
    }

  set_param = bt_buf_add(buf, sizeof(*set_param));

  memset(set_param, 0, sizeof(*set_param));
  set_param->min_interval = sys_cpu_to_le16(0x0800);
  set_param->max_interval = sys_cpu_to_le16(0x0800);
  set_param->type         = type;
  set_param->channel_map  = 0x07;

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_PARAMETERS, buf);

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
  if (!buf)
    {
      return -ENOBUFS;
    }

  g_btdev.adv_enable = 0x01;
  memcpy(bt_buf_add(buf, 1), &g_btdev.adv_enable, 1);

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_ADV_ENABLE, buf, NULL);
}

int bt_stop_advertising(void)
{
  FAR struct bt_buf_s *buf;

  if (!g_btdev.adv_enable)
    {
      return -EALREADY;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
  if (!buf)
    {
      return -ENOBUFS;
    }

  g_btdev.adv_enable = 0x00;
  memcpy(bt_buf_add(buf, 1), &g_btdev.adv_enable, 1);

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_ADV_ENABLE, buf, NULL);
}

int bt_start_scanning(uint8_t scan_filter, bt_le_scan_cb_t cb)
{
  /* Return if active scan is already enabled */

  if (g_scan_dev_found_cb)
    {
      return -EALREADY;
    }

  g_scan_dev_found_cb = cb;
  g_btdev.scan_filter = scan_filter;

  return bt_le_scan_update();
}

int bt_stop_scanning(void)
{
  /* Return if active scanning is already disabled */

  if (!g_scan_dev_found_cb)
    {
      return -EALREADY;
    }

  g_scan_dev_found_cb = NULL;
  g_btdev.scan_filter = BT_LE_SCAN_FILTER_DUP_ENABLE;

  return bt_le_scan_update();
}
