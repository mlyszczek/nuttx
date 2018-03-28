/****************************************************************************
 * wireless/bluetooth/bt_keys.c
 * Bluetooth key handling
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
#include <debug.h>

#include <nuttx/wireless/bt_log.h>
#include <nuttx/wireless/bt_bluetooth.h>
#include <nuttx/wireless/bt_hci.h>

#include "bt_hcicore.h"
#include "bt_smp.h"
#include "bt_keys.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define bt_keys_foreach(list, cur, member) \
  for (cur = list; *cur; cur = &(*cur)->member)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_keys_s key_pool[CONFIG_BLUETOOTH_MAX_PAIRED];

static FAR struct bt_keys_s *ltks;
static FAR struct bt_keys_s *slave_ltks;
static FAR struct bt_keys_s *irks;
static FAR struct bt_keys_s *local_csrks;
static FAR struct bt_keys_s *remote_csrks;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct bt_keys_s *bt_keys_get_addr(FAR const bt_addr_le_t *addr)
{
  FAR struct bt_keys_s *keys;
  int i;

  winfo("%s\n", bt_addr_le_str(addr));

  for (i = 0; i < ARRAY_SIZE(key_pool); i++)
    {
      keys = &key_pool[i];

      if (!bt_addr_le_cmp(&keys->addr, addr))
        {
          return keys;
        }

      if (!bt_addr_le_cmp(&keys->addr, BT_ADDR_LE_ANY))
        {
          bt_addr_le_copy(&keys->addr, addr);
          winfo("created %p for %s\n", keys, bt_addr_le_str(addr));
          return keys;
        }
    }

  winfo("unable to create keys for %s\n", bt_addr_le_str(addr));

  return NULL;
}

void bt_keys_clear(FAR struct bt_keys_s *keys, int type)
{
  FAR struct bt_keys_s **cur;

  winfo("keys for %s type %d\n", bt_addr_le_str(&keys->addr), type);

  if (((type & keys->keys) & BT_KEYS_SLAVE_LTK))
    {
      bt_keys_foreach(&slave_ltks, cur, slave_ltk.next)
        {
          if (*cur == keys)
            {
              *cur = (*cur)->slave_ltk.next;
              break;
            }
        }

      keys->keys &= ~BT_KEYS_SLAVE_LTK;
    }

  if (((type & keys->keys) & BT_KEYS_LTK))
    {
      bt_keys_foreach(&ltks, cur, ltk.next)
        {
          if (*cur == keys)
            {
              *cur = (*cur)->ltk.next;
              break;
            }
        }

      keys->keys &= ~BT_KEYS_LTK;
    }

  if (((type & keys->keys) & BT_KEYS_IRK))
    {
      bt_keys_foreach(&irks, cur, irk.next)
        {
          if (*cur == keys)
            {
              *cur = (*cur)->irk.next;
              break;
            }
        }

      keys->keys &= ~BT_KEYS_IRK;
    }

  if (((type & keys->keys) & BT_KEYS_LOCAL_CSRK))
    {
      bt_keys_foreach(&local_csrks, cur, local_csrk.next)
        {
          if (*cur == keys)
            {
              *cur = (*cur)->local_csrk.next;
              break;
            }
        }

      keys->keys &= ~BT_KEYS_LOCAL_CSRK;
    }

  if (((type & keys->keys) & BT_KEYS_REMOTE_CSRK))
    {
      bt_keys_foreach(&remote_csrks, cur, remote_csrk.next)
        {
          if (*cur == keys)
            {
              *cur = (*cur)->remote_csrk.next;
              break;
            }
        }

      keys->keys &= ~BT_KEYS_REMOTE_CSRK;
    }

  if (!keys->keys)
    {
      memset(keys, 0, sizeof(*keys));
    }
}

FAR struct bt_keys_s *bt_keys_find(int type, FAR const bt_addr_le_t *addr)
{
  FAR struct bt_keys_s **cur;

  winfo("type %d %s\n", type, bt_addr_le_str(addr));

  switch (type)
    {
    case BT_KEYS_SLAVE_LTK:
      bt_keys_foreach(&slave_ltks, cur, slave_ltk.next)
        {
          if (!bt_addr_le_cmp(&(*cur)->addr, addr))
            {
              break;
            }
        }

      return *cur;

    case BT_KEYS_LTK:
      bt_keys_foreach(&ltks, cur, ltk.next)
        {
          if (!bt_addr_le_cmp(&(*cur)->addr, addr))
            {
              break;
            }
        }

      return *cur;

    case BT_KEYS_IRK:
      bt_keys_foreach(&irks, cur, irk.next)
        {
          if (!bt_addr_le_cmp(&(*cur)->addr, addr))
            {
              break;
            }
        }

      return *cur;

    case BT_KEYS_LOCAL_CSRK:
      bt_keys_foreach(&local_csrks, cur, local_csrk.next)
        {
          if (!bt_addr_le_cmp(&(*cur)->addr, addr))
            {
              break;
            }
        }

      return *cur;

    case BT_KEYS_REMOTE_CSRK:
      bt_keys_foreach(&remote_csrks, cur, remote_csrk.next)
        {
          if (!bt_addr_le_cmp(&(*cur)->addr, addr))
            {
              break;
            }
        }

      return *cur;

    default:
      return NULL;
    }
}

void bt_keys_add_type(FAR struct bt_keys_s *keys, int type)
{
  if ((keys->keys & type) != 0)
    {
      return;
    }

  switch (type)
    {
    case BT_KEYS_SLAVE_LTK:
      keys->slave_ltk.next   = slave_ltks;
      slave_ltks             = keys;
      break;

    case BT_KEYS_LTK:
      keys->ltk.next         = ltks;
      ltks                   = keys;
      break;

    case BT_KEYS_IRK:
      keys->irk.next         = irks;
      irks                   = keys;
      break;

    case BT_KEYS_LOCAL_CSRK:
      keys->local_csrk.next  = local_csrks;
      local_csrks            = keys;
      break;

    case BT_KEYS_REMOTE_CSRK:
      keys->remote_csrk.next = remote_csrks;
      remote_csrks           = keys;
      break;

    default:
      wlerr("ERROR: Unknown key type %d\n", type);
      return;
    }

  keys->keys |= type;
}

FAR struct bt_keys_s *bt_keys_get_type(int type,
                                       FAR const bt_addr_le_t *addr)
{
  FAR struct bt_keys_s *keys;

  winfo("type %d %s\n", type, bt_addr_le_str(addr));

  keys = bt_keys_find(type, addr);
  if (keys)
    {
      return keys;
    }

  keys = bt_keys_get_addr(addr);
  if (!keys)
    {
      return NULL;
    }

  bt_keys_add_type(keys, type);
  return keys;
}

FAR struct bt_keys_s *bt_keys_find_irk(FAR const bt_addr_le_t * addr)
{
  FAR struct bt_keys_s **cur;

  winfo("%s\n", bt_addr_le_str(addr));

  if (!bt_addr_le_is_rpa(addr))
    {
      return NULL;
    }

  bt_keys_foreach(&irks, cur, irk.next)
  {
    FAR struct bt_irk *irk = &(*cur)->irk;

    if (!bt_addr_cmp((bt_addr_t *) addr->val, &irk->rpa))
      {
        winfo("cached RPA %s for %s\n", bt_addr_str(&irk->rpa),
               bt_addr_le_str(&(*cur)->addr));
        return *cur;
      }

    if (bt_smp_irk_matches(irk->val, (bt_addr_t *) addr->val))
      {
        FAR struct bt_keys_s *match = *cur;

        winfo("RPA %s matches %s\n", bt_addr_str(&irk->rpa),
               bt_addr_le_str(&(*cur)->addr));

        bt_addr_copy(&irk->rpa, (bt_addr_t *) addr->val);

        /* Move to the beginning of the list for faster future lookups. */

        if (match != irks)
          {
            /* Remove match from list */

            *cur = irk->next;

            /* Add match to the beginning */

            irk->next = irks;
            irks = match;
          }

        return match;
      }
  }

  winfo("No IRK for %s\n", bt_addr_le_str(addr));
  return NULL;
}
