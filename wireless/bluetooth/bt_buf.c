/****************************************************************************
 * wireless/bluetooth/bt_buf_s.c
 * Bluetooth buffer management
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/wireless/bt_hci.h>
#include <nuttx/wireless/bt_core.h>
#include <nuttx/wireless/bt_buf.h>

#include "bt_hcicore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct bt_buf_s *bt_buf_get(enum bt_buf_type_e type, size_t reserve_head)
{
  FAR struct sq_queue_s *avail = get_avail(type);
  FAR struct bt_buf_s *buf;
  FAR struct iob_s *iob;

  winfo("type %d reserve %u\n", type, reserve_head);

  iob = iob_alloc(false);
  if (iob == NULL)
    {
      if (context_type_get() == NANO_CTX_ISR)
        {
          wlerr("ERROR: Failed to get free buffer\n");
          return NULL;
        }

      wlwarn("WARNING: Low on buffers. Waiting (type %d)\n", type);
      iob = iob_alloc(avail);
    }

  iob->io_len    = sizeof(struct bt_buf_s);
  iob->io_offset = 0;
  iob->io_pktlen = sizeof(struct bt_buf_s);

  buf            = iob->io_data;
  memset(buf, 0, sizeof(struct bt_buf_s));

  buf->iob       = iob;
  buf->ref       = 1;
  buf->type      = type;
  buf->data      = buf->buf + reserve_head;

  winfo("buf %p type %d reserve %u\n", buf, buf->type, reserve_head);
  return buf;
}

void bt_buf_put(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_cp_host_num_completed_packets_s *cp;
  FAR struct bt_hci_handle_count_s *hc;
  enum bt_buf_type_e type;
  uint16_t handle;

  winfo("buf %p ref %u type %d\n", buf, buf->ref, buf->type);

  if (--buf->ref > 0)
    {
      return;
    }

  handle = buf->acl.handle;
  type   = buf->type;

  DEBUGASSERT(buf->iob != NULL);
  iob_free(buf->iob);

  if (type != BT_ACL_IN)
    {
      return;
    }

  winfo("Reporting completed packet for handle %u\n", handle);

  buf = bt_hci_cmd_create(BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS,
                          sizeof(*cp) + sizeof(*hc));
  if (!buf)
    {
      wlerr("ERROR: Unable to allocate new HCI command\n");
      return;
    }

  cp              = bt_buf_add(buf, sizeof(*cp));
  cp->num_handles = sys_cpu_to_le16(1);

  hc              = bt_buf_add(buf, sizeof(*hc));
  hc->handle      = sys_cpu_to_le16(handle);
  hc->count       = sys_cpu_to_le16(1);

  bt_hci_cmd_send(BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS, buf);
}

FAR struct bt_buf_s *bt_buf_hold(FAR struct bt_buf_s *buf)
{
  winfo("buf %p (old) ref %u type %d\n", buf, buf->ref, buf->type);
  buf->ref++;
  return buf;
}

FAR void *bt_buf_add(FAR struct bt_buf_s *buf, size_t len)
{
  FAR uint8_t *tail = bt_buf_tail(buf);

  winfo("buf %p len %u\n", buf, len);

  DEBUGASSERT(bt_buf_tailroom(buf) >= len);

  buf->len += len;
  return tail;
}

void bt_buf_add_le16(FAR struct bt_buf_s *buf, uint16_t value)
{
  winfo("buf %p value %u\n", buf, value);

  value = sys_cpu_to_le16(value);
  memcpy(bt_buf_add(buf, sizeof(value)), &value, sizeof(value));
}

FAR void *bt_buf_push(FAR struct bt_buf_s *buf, size_t len)
{
  winfo("buf %p len %u\n", buf, len);

  DEBUGASSERT(bt_buf_headroom(buf) >= len);

  buf->data -= len;
  buf->len  += len;
  return buf->data;
}

FAR void *bt_buf_pull(FAR struct bt_buf_s *buf, size_t len)
{
  winfo("buf %p len %u\n", buf, len);

  DEBUGASSERT(buf->len >= len);

  buf->len -= len;
  return buf->data += len;
}

uint16_t bt_buf_pull_le16(FAR struct bt_buf_s * buf)
{
  uint16_t value;

  value = UNALIGNED_GET((FAR uint16_t *)buf->data);
  bt_buf_pull(buf, sizeof(value));

  return sys_le16_to_cpu(value);
}

size_t bt_buf_headroom(FAR struct bt_buf_s * buf)
{
  return buf->data - buf->buf;
}

size_t bt_buf_tailroom(FAR struct bt_buf_s * buf)
{
  return BT_BUF_MAX_DATA - bt_buf_headroom(buf) - buf->len;
}

int bt_buf_init(void)
{
  winfo("Configured buffers: ACL in: %d ACL out: %d size: %u\n",
         acl_in, acl_out, sizeof(struct bt_buf_s));
  winfo("Configured IOBs: IOBs: %u size: %u\n",
         CONFIG_IOB_NBUFFERS, CONFIG_IOB_BUFSIZE);

  DEBUGASSERT(CONFIG_IOB_NBUFFERS > (acl_out + acl_in));
  DEBUGASSERT(sizeof(struct bt_fifo_s) <= CONFIG_IOB_BUFSIZE);
  return 0;
}
