/****************************************************************************
 * drivers/clk/clk-rpmsg.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin<zhuyanlin@pinecone.net>
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

#include <string.h>

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLK_RPMSG_EPT_NAME          "rpmsg-clk"

#define CLK_RPMSG_ENABLE            0
#define CLK_RPMSG_DISABLE           1
#define CLK_RPMSG_SETRATE           2
#define CLK_RPMSG_SETPHASE          3
#define CLK_RPMSG_GETPHASE          4
#define CLK_RPMSG_GETRATE           5
#define CLK_RPMSG_ROUNDRATE         6
#define CLK_RPMSG_ISENABLED         7

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)             (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct clk_rpmsg_priv_s
{
  struct rpmsg_endpoint     ept;
  struct list_node          clk_list;  /* head of clk_rpmsg_s struct */
  struct list_node          node;      /* list node for struct priv */
  const char               *cpuname;
};

struct clk_rpmsg_s
{
  struct clk               *clk;
  uint32_t                  count;
  struct list_node          node;
};

struct clk_rpmsg_cookie_s
{
  sem_t                     sem;
  int64_t                   result;
};

begin_packed_struct struct clk_rpmsg_header_s
{
  uint32_t                  command;
  uint32_t                  response;
  int64_t                   result;
  uint64_t                  cookie;
} end_packed_struct;

begin_packed_struct struct clk_rpmsg_enable_s
{
  struct clk_rpmsg_header_s header;
  char                      name[0];
} end_packed_struct;

#define clk_rpmsg_disable_s clk_rpmsg_enable_s
#define clk_rpmsg_isenabled_s clk_rpmsg_enable_s

begin_packed_struct struct clk_rpmsg_setrate_s
{
  struct clk_rpmsg_header_s header;
  uint64_t                  rate;
  char                      name[0];
} end_packed_struct;

#define clk_rpmsg_getrate_s clk_rpmsg_enable_s
#define clk_rpmsg_roundrate_s clk_rpmsg_setrate_s

begin_packed_struct struct clk_rpmsg_setphase_s
{
  struct clk_rpmsg_header_s header;
  int32_t                   degrees;
  char                      name[0];
} end_packed_struct;

#define clk_rpmsg_getphase_s clk_rpmsg_enable_s

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct clk_rpmsg_priv_s *clk_rpmsg_get_priv(const char *name);
static struct rpmsg_endpoint *clk_rpmsg_get_ept(const char **name);
static struct clk_rpmsg_s *clk_rpmsg_get_clk(struct rpmsg_endpoint *ept,
                                             const char *name);

static int clk_rpmsg_enable_handler(struct rpmsg_endpoint *ept, void *data,
                                    size_t len, uint32_t src, void *priv);
static int clk_rpmsg_disable_handler(struct rpmsg_endpoint *ept, void *data,
                                     size_t len, uint32_t src, void *priv);
static int clk_rpmsg_getrate_handler(struct rpmsg_endpoint *ept, void *data,
                                     size_t len, uint32_t src, void *priv);
static int clk_rpmsg_roundrate_handler(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv);
static int clk_rpmsg_setrate_handler(struct rpmsg_endpoint *ept, void *data,
                                     size_t len, uint32_t src, void *priv);
static int clk_rpmsg_setphase_handler(struct rpmsg_endpoint *ept, void *data,
                                      size_t len, uint32_t src, void *priv);
static int clk_rpmsg_getphase_handler(struct rpmsg_endpoint *ept, void *data,
                                      size_t len, uint32_t src, void *priv);
static int clk_rpmsg_isenabled_handler(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv);

static void clk_rpmsg_device_created(struct rpmsg_device *rdev, void *priv_);
static void clk_rpmsg_device_destroy(struct rpmsg_device *rdev, void *priv_);
static int clk_rpmsg_ept_cb(struct rpmsg_endpoint *ept, void *data,
                            size_t len, uint32_t src, void *priv);

static int64_t clk_rpmsg_sendrecv(struct rpmsg_endpoint *ept, uint32_t command,
                                  struct clk_rpmsg_header_s *msg, int32_t len);
static int clk_rpmsg_enable(struct clk *clk);
static void clk_rpmsg_disable(struct clk *clk);
static int clk_rpmsg_is_enabled(struct clk *clk);
static uint32_t clk_rpmsg_round_rate(struct clk *clk, uint32_t rate,
            uint32_t *parent_rate);
static int clk_rpmsg_set_rate(struct clk *clk, uint32_t rate, uint32_t parent_rate);
static uint32_t clk_rpmsg_recalc_rate(struct clk *clk, uint32_t parent_rate);
static int clk_rpmsg_get_phase(struct clk *clk);
static int clk_rpmsg_set_phase(struct clk *clk, int degrees);

/****************************************************************************
 * Private Datas
 ****************************************************************************/

static mutex_t g_clk_rpmsg_lock          = MUTEX_INITIALIZER;
static struct list_node g_clk_rpmsg_priv = LIST_INITIAL_VALUE(g_clk_rpmsg_priv);

static const rpmsg_ept_cb g_clk_rpmsg_handler[] =
{
  [CLK_RPMSG_ENABLE]    = clk_rpmsg_enable_handler,
  [CLK_RPMSG_DISABLE]   = clk_rpmsg_disable_handler,
  [CLK_RPMSG_SETRATE]   = clk_rpmsg_setrate_handler,
  [CLK_RPMSG_SETPHASE]  = clk_rpmsg_setphase_handler,
  [CLK_RPMSG_GETPHASE]  = clk_rpmsg_getphase_handler,
  [CLK_RPMSG_GETRATE]   = clk_rpmsg_getrate_handler,
  [CLK_RPMSG_ROUNDRATE] = clk_rpmsg_roundrate_handler,
  [CLK_RPMSG_ISENABLED] = clk_rpmsg_isenabled_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct clk_rpmsg_priv_s *clk_rpmsg_get_priv(const char *name)
{
  struct clk_rpmsg_priv_s *priv;

  nxmutex_lock(&g_clk_rpmsg_lock);

  list_for_every_entry(&g_clk_rpmsg_priv, priv, struct clk_rpmsg_priv_s, node)
    {
      size_t len = strlen(priv->cpuname);

      if (!strncmp(priv->cpuname, name, len) &&
            (name[len] == '/' || name[len] == 0))
        {
          goto out; /* Find the target, exit */
        }
    }

  priv = NULL;

out:
  nxmutex_unlock(&g_clk_rpmsg_lock);
  return priv;
}

static struct rpmsg_endpoint *clk_rpmsg_get_ept(const char **name)
{
  struct clk_rpmsg_priv_s *priv;

  priv = clk_rpmsg_get_priv(*name);
  if (priv == NULL)
    {
      return NULL;
    }

  /* transfer to local clk name */
  *name += strlen(priv->cpuname) + 1;

  return &priv->ept;
}

static struct clk_rpmsg_s *clk_rpmsg_get_clk(struct rpmsg_endpoint *ept,
                                             const char *name)
{
  struct clk_rpmsg_priv_s *priv = ept->priv;
  struct list_node *clk_list = &priv->clk_list;
  struct clk_rpmsg_s *clkrp;

  list_for_every_entry(clk_list, clkrp, struct clk_rpmsg_s, node)
    {
      if (!strcmp(clk_get_name(clkrp->clk), name))
        {
          return clkrp;
        }
    }

  clkrp = kmm_zalloc(sizeof(*clkrp));
  if (!clkrp)
    {
      return NULL;
    }

  clkrp->clk = clk_get(name);
  if (!clkrp->clk)
    {
      kmm_free(clkrp);
      return NULL;
    }

  list_add_head(clk_list, &clkrp->node);

  return clkrp;
}

static int clk_rpmsg_enable_handler(struct rpmsg_endpoint *ept, void *data,
                                    size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_enable_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_enable(clkrp->clk);
      if (!msg->header.result)
        {
          clkrp->count++;
        }
    }
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_disable_handler(struct rpmsg_endpoint *ept, void *data,
                                     size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_disable_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      clk_disable(clkrp->clk);
      clkrp->count--;
      msg->header.result = 0;
    }
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_getrate_handler(struct rpmsg_endpoint *ept, void *data,
                                     size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_getrate_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    msg->header.result = clk_get_rate(clkrp->clk);
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_roundrate_handler(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_roundrate_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    msg->header.result = clk_round_rate(clkrp->clk, msg->rate);
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_setrate_handler(struct rpmsg_endpoint *ept, void *data,
                                     size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_setrate_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    msg->header.result = clk_set_rate(clkrp->clk, msg->rate);
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_setphase_handler(struct rpmsg_endpoint *ept, void *data,
                                      size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_setphase_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    msg->header.result = clk_set_phase(clkrp->clk, msg->degrees);
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_getphase_handler(struct rpmsg_endpoint *ept, void *data,
                                      size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_getphase_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    msg->header.result = clk_get_phase(clkrp->clk);
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_isenabled_handler(struct rpmsg_endpoint *ept, void *data,
                                       size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_isenabled_s *msg = data;
  struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    msg->header.result = clk_is_enabled(clkrp->clk);
  else
    msg->header.result = -ENOENT;

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int64_t clk_rpmsg_sendrecv(struct rpmsg_endpoint *ept, uint32_t command,
                                  struct clk_rpmsg_header_s *msg, int32_t len)
{
  struct clk_rpmsg_cookie_s cookie;
  int ret;

  msg->command  = command;
  msg->response = 0;
  msg->cookie   = (uintptr_t)&cookie;

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_setprotocol(&cookie.sem, SEM_PRIO_NONE);
  cookie.result  = -EIO;

  ret = rpmsg_send_nocopy(ept, msg, len);
  if (ret < 0)
    return ret;

  ret = nxsem_wait_uninterruptible(&cookie.sem);
  if (ret < 0)
    return ret;

  return cookie.result;
}

static void clk_rpmsg_device_created(struct rpmsg_device *rdev, void *priv_)
{
  struct clk_rpmsg_priv_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct clk_rpmsg_priv_s));
  if (!priv)
    {
      return;
    }

  priv->ept.priv = priv;
  priv->cpuname  = rpmsg_get_cpuname(rdev);

  list_initialize(&priv->clk_list);

  nxmutex_lock(&g_clk_rpmsg_lock);
  list_add_head(&g_clk_rpmsg_priv, &priv->node);
  nxmutex_unlock(&g_clk_rpmsg_lock);

  ret = rpmsg_create_ept(&priv->ept, rdev, CLK_RPMSG_EPT_NAME,
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         clk_rpmsg_ept_cb, NULL);
  if (ret)
    {
      free(priv);
    }
}

static void clk_rpmsg_device_destroy(struct rpmsg_device *rdev, void *priv_)
{
  struct clk_rpmsg_s *clkrp, *clkrp_tmp;
  struct clk_rpmsg_priv_s *priv;

  priv = clk_rpmsg_get_priv(rpmsg_get_cpuname(rdev));
  if (!priv)
    {
      return;
    }

  list_for_every_entry_safe(&priv->clk_list, clkrp, clkrp_tmp,
                              struct clk_rpmsg_s, node)
    {
      while (clkrp->count--)
        {
          clk_disable(clkrp->clk);
        }

      list_delete(&clkrp->node);
      kmm_free(clkrp);
    }

  nxmutex_lock(&g_clk_rpmsg_lock);
  list_delete(&priv->node);
  nxmutex_unlock(&g_clk_rpmsg_lock);

  rpmsg_destroy_ept(&priv->ept);
  kmm_free(priv);
}

static int clk_rpmsg_ept_cb(struct rpmsg_endpoint *ept, void *data,
                            size_t len, uint32_t src, void *priv)
{
  struct clk_rpmsg_header_s *hdr = data;
  uint32_t cmd = hdr->command;
  int ret = -EINVAL;

  if (hdr->response)
    {
      struct clk_rpmsg_cookie_s *cookie =
      (struct clk_rpmsg_cookie_s *)(uintptr_t)hdr->cookie;
      if (cookie)
        {
          cookie->result = hdr->result;
          nxsem_post(&cookie->sem);
          ret = 0;
        }
    }
  else if (cmd < ARRAY_SIZE(g_clk_rpmsg_handler) && g_clk_rpmsg_handler[cmd])
    {
      hdr->response = 1;
      ret = g_clk_rpmsg_handler[cmd](ept, data, len, src, priv);
    }

  return ret;
}

static int clk_rpmsg_enable(struct clk *clk)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_enable_s *msg;
  const char *name = clk->name;
  uint32_t size, len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return -ENODEV;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return -ENOMEM;

  DEBUGASSERT(len <= size);

  cstr2bstr(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_ENABLE,
          (struct clk_rpmsg_header_s *)msg, len);
}

static void clk_rpmsg_disable(struct clk *clk)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_disable_s *msg;
  const char *name = clk->name;
  uint32_t size, len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return;

  DEBUGASSERT(len <= size);

  cstr2bstr(msg->name, name);

  clk_rpmsg_sendrecv(ept, CLK_RPMSG_DISABLE,
      (struct clk_rpmsg_header_s *)msg, len);
}

static int clk_rpmsg_is_enabled(struct clk *clk)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_enable_s *msg;
  const char *name = clk->name;
  uint32_t size, len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return -ENODEV;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return -ENOMEM;

  DEBUGASSERT(len <= size);

  cstr2bstr(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_ISENABLED,
          (struct clk_rpmsg_header_s *)msg, len);
}

static uint32_t clk_rpmsg_round_rate(struct clk *clk, uint32_t rate, uint32_t *parent_rate)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_roundrate_s *msg;
  const char *name = clk->name;
  uint32_t size, len;
  int64_t ret;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return 0;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return 0;

  DEBUGASSERT(len <= size);

  msg->rate = rate;
  cstr2bstr(msg->name, name);

  ret = clk_rpmsg_sendrecv(ept, CLK_RPMSG_ROUNDRATE,
            (struct clk_rpmsg_header_s *)msg, len);
  if (ret < 0)
    return 0;

  return ret;
}

static int clk_rpmsg_set_rate(struct clk *clk, uint32_t rate, uint32_t parent_rate)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_setrate_s *msg;
  const char *name = clk->name;
  uint32_t size, len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return -ENODEV;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return -ENOMEM;

  DEBUGASSERT(len <= size);

  msg->rate = rate;
  cstr2bstr(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_SETRATE,
            (struct clk_rpmsg_header_s *)msg, len);
}

static uint32_t clk_rpmsg_recalc_rate(struct clk *clk, uint32_t parent_rate)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_getrate_s *msg;
  const char *name = clk->name;
  uint32_t size, len;
  int64_t ret;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return 0;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return 0;

  DEBUGASSERT(len <= size);

  cstr2bstr(msg->name, name);

  ret = clk_rpmsg_sendrecv(ept, CLK_RPMSG_GETRATE,
            (struct clk_rpmsg_header_s *)msg, len);
  if (ret < 0)
    return 0;

  return ret;
}

static int clk_rpmsg_get_phase(struct clk *clk)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_getphase_s *msg;
  const char *name = clk->name;
  uint32_t size, len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return -ENODEV;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return -ENOMEM;

  DEBUGASSERT(len <= size);

  cstr2bstr(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_GETPHASE,
            (struct clk_rpmsg_header_s *)msg, len);
}

static int clk_rpmsg_set_phase(struct clk *clk, int degrees)
{
  struct rpmsg_endpoint *ept;
  struct clk_rpmsg_setphase_s *msg;
  const char *name = clk->name;
  uint32_t size, len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    return -ENODEV;

  len = sizeof(*msg) + B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    return -ENOMEM;

  DEBUGASSERT(len <= size);

  msg->degrees = degrees;
  cstr2bstr(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_SETPHASE,
            (struct clk_rpmsg_header_s *)msg, len);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops clk_rpmsg_ops =
{
  .enable = clk_rpmsg_enable,
  .disable = clk_rpmsg_disable,
  .is_enabled = clk_rpmsg_is_enabled,
  .recalc_rate = clk_rpmsg_recalc_rate,
  .round_rate = clk_rpmsg_round_rate,
  .set_rate = clk_rpmsg_set_rate,
  .set_phase = clk_rpmsg_set_phase,
  .get_phase = clk_rpmsg_get_phase,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct clk *clk_register_rpmsg(const char *name, uint8_t flags)
{
  if (strchr(name, '/') == NULL)
    return NULL;

  return clk_register(name, NULL, 0, flags | CLK_IGNORE_UNUSED, &clk_rpmsg_ops, NULL, 0);
}

int clk_rpmsg_initialize(void)
{
  return rpmsg_register_callback(NULL,
                                 clk_rpmsg_device_created,
                                 clk_rpmsg_device_destroy,
                                 NULL);
}
