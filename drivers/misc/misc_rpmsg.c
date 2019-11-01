/****************************************************************************
 * drivers/misc/misc_rpmsg.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@fishsemi.net>
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
#include <nuttx/environ.h>

#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/mtd/song_onchip_flash.h>
#include <nuttx/power/pm.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/rptun/rptun.h>
#include <metal/utilities.h>

#include <fcntl.h>
#include <crc32.h>
#include <string.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define MISC_RPMSG_MAX_PRIV             2

#define MISC_RPMSG_EPT_NAME             "rpmsg-misc"

#define MISC_RPMSG_RETENT_ADD           0
#define MISC_RPMSG_RETENT_SET           1
#define MISC_RPMSG_REMOTE_BOOT          2
#define MISC_RPMSG_REMOTE_CLOCKSYNC     3
#define MISC_RPMSG_REMOTE_ENVSYNC       4
#define MISC_RPMSG_REMOTE_INFOWRITE     5

#define MISC_RETENT_MAGIC               (0xdeadbeef)

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)                 (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct misc_rpmsg_head_s
{
  uint32_t command;
} end_packed_struct;

begin_packed_struct struct misc_rpmsg_retent_add_s
{
  uint32_t command;
  uint32_t blkid;
  uintptr_t base;
  uint32_t size;
  uint32_t dma;
} end_packed_struct;

begin_packed_struct struct misc_rpmsg_retent_set_s
{
  uint32_t command;
  uint32_t blkid;
  uint32_t flush;
} end_packed_struct;

begin_packed_struct struct misc_rpmsg_remote_boot_s
{
  uint32_t command;
  char     name[16];
} end_packed_struct;

#define misc_rpmsg_remote_clocksync_s misc_rpmsg_head_s

begin_packed_struct struct misc_rpmsg_remote_envsync_s
{
  uint32_t command;
  uint32_t response;
  uint32_t result;
  char     name[16];
  char     value[32];
} end_packed_struct;

begin_packed_struct struct misc_rpmsg_remote_infowrite_s
{
  uint32_t command;
  char     name[16];
  uint8_t  value[32];
  uint32_t len;
} end_packed_struct;

struct misc_rpmsg_s
{
  struct misc_dev_s     dev;
  struct rpmsg_endpoint ept;
  struct metal_list     blks;
  const char            *cpuname;
};

struct misc_retent_blk_s
{
  uint32_t magic;
  uintptr_t base;
  uint32_t size;
  uint32_t dma;
  uint32_t crc;
  uint32_t flush;
  uint32_t blkid;
  struct metal_list node;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void misc_rpmsg_device_created(struct rpmsg_device *rdev, void *priv_);
static void misc_rpmsg_device_destroy(struct rpmsg_device *rdev, void *priv_);
static int misc_rpmsg_ept_cb(struct rpmsg_endpoint *ept, void *data,
                             size_t len, uint32_t src, void *priv_);

static int misc_retent_add_handler(struct rpmsg_endpoint *ept,
                                   void *data, size_t len,
                                   uint32_t src, void *priv_);
static int misc_retent_set_handler(struct rpmsg_endpoint *ept,
                                   void *data, size_t len,
                                   uint32_t src, void *priv_);
static int misc_remote_boot_handler(struct rpmsg_endpoint *ept,
                                   void *data, size_t len,
                                   uint32_t src, void *priv_);
static int misc_remote_clocksync_handler(struct rpmsg_endpoint *ept,
                                         void *data, size_t len,
                                         uint32_t src, void *priv_);
static int misc_remote_envsync_handler(struct rpmsg_endpoint *ept,
                                       void *data, size_t len,
                                       uint32_t src, void *priv_);
static int misc_remote_infowrite_handler(struct rpmsg_endpoint *ept,
                                         void *data, size_t len,
                                         uint32_t src, void *priv_);

static int misc_retent_save_blk(struct misc_retent_blk_s *blk, int fd);
static int misc_retent_save(struct misc_dev_s *dev, char *file);
static int misc_retent_restore_blk(struct misc_retent_blk_s *blk, int fd);
static int misc_retent_restore(struct misc_dev_s *dev, char *file);

static int misc_retent_add(struct misc_rpmsg_s *priv, unsigned long arg);
static int misc_retent_set(struct misc_rpmsg_s *priv, unsigned long arg);
static int misc_remote_boot(struct misc_rpmsg_s *priv, unsigned long arg);
static int misc_remote_clocksync(struct misc_rpmsg_s *priv, unsigned long arg);
static int misc_remote_envsync(struct misc_rpmsg_s *priv, unsigned long arg);
static int misc_remote_infowrite(struct misc_rpmsg_s *priv, unsigned long arg);
static int misc_dev_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_misc_rpmsg_handler[] =
{
  [MISC_RPMSG_RETENT_ADD]       = misc_retent_add_handler,
  [MISC_RPMSG_RETENT_SET]       = misc_retent_set_handler,
  [MISC_RPMSG_REMOTE_BOOT]      = misc_remote_boot_handler,
  [MISC_RPMSG_REMOTE_CLOCKSYNC] = misc_remote_clocksync_handler,
  [MISC_RPMSG_REMOTE_ENVSYNC]   = misc_remote_envsync_handler,
  [MISC_RPMSG_REMOTE_INFOWRITE] = misc_remote_infowrite_handler,
};

static const struct misc_ops_s g_misc_ops =
{
  .retent_save    = misc_retent_save,
  .retent_restore = misc_retent_restore,
};

static const struct file_operations g_misc_devops =
{
  .ioctl = misc_dev_ioctl,
};

static struct misc_rpmsg_s *g_misc_priv[MISC_RPMSG_MAX_PRIV];
static int g_misc_idx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void misc_rpmsg_device_created(struct rpmsg_device *rdev, void *priv_)
{
  struct misc_rpmsg_s *priv = priv_;

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;

      rpmsg_create_ept(&priv->ept, rdev, MISC_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       misc_rpmsg_ept_cb, NULL);
    }
}

static void misc_rpmsg_device_destroy(struct rpmsg_device *rdev, void *priv_)
{
  struct misc_rpmsg_s *priv = priv_;

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

static int misc_rpmsg_ept_cb(struct rpmsg_endpoint *ept, void *data,
                             size_t len, uint32_t src, void *priv)
{
  struct misc_rpmsg_head_s *head = data;
  uint32_t command = head->command;

  if (command < ARRAY_SIZE(g_misc_rpmsg_handler))
    {
      return g_misc_rpmsg_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

static int misc_retent_add_handler(struct rpmsg_endpoint *ept,
                                   void *data, size_t len,
                                   uint32_t src, void *priv_)
{
  struct misc_rpmsg_retent_add_s *msg = data;
  struct misc_rpmsg_s *priv = priv_;
  struct misc_retent_blk_s *blk;

  blk = kmm_zalloc(sizeof(struct misc_retent_blk_s));
  if (!blk)
    {
      return -ENOMEM;
    }

  blk->blkid = msg->blkid;
  blk->base  = (uintptr_t)up_addrenv_pa_to_va(msg->base);
  blk->size  = msg->size;
  blk->dma   = msg->dma;
  blk->flush = true;

  metal_list_add_tail(&priv->blks, &blk->node);
  return 0;
}

static int misc_retent_set_handler(struct rpmsg_endpoint *ept,
                                   void *data, size_t len,
                                   uint32_t src, void *priv_)
{
  struct misc_rpmsg_retent_set_s *msg = data;
  struct misc_rpmsg_s *priv = priv_;
  struct metal_list *node;

  metal_list_for_each(&priv->blks, node)
    {
      struct misc_retent_blk_s *blk;

      blk = metal_container_of(node, struct misc_retent_blk_s, node);
      if (blk->blkid == msg->blkid)
        {
          blk->flush = msg->flush;
        }
    }

  return 0;
}

static int misc_remote_boot_handler(struct rpmsg_endpoint *ept,
                                   void *data, size_t len,
                                   uint32_t src, void *priv_)
{
  struct misc_rpmsg_remote_boot_s *msg = data;

  if (rptun_boot(msg->name))
    {
      _err("Boot core err, name %s\n", msg->name);
    }

  return 0;
}

static int misc_remote_clocksync_(const char *cpuname)
{
  int i;
  struct misc_rpmsg_remote_clocksync_s msg =
    {
      .command = MISC_RPMSG_REMOTE_CLOCKSYNC,
    };

  for (i = 0; i < g_misc_idx; i++)
    {
      if (!cpuname || strcmp(cpuname, g_misc_priv[i]->cpuname))
        {
          rpmsg_send(&g_misc_priv[i]->ept, &msg, sizeof(msg));
        }
    }

  return 0;
}

static int misc_remote_clocksync_handler(struct rpmsg_endpoint *ept,
                                         void *data, size_t len,
                                         uint32_t src, void *priv_)
{
#ifdef CONFIG_RTC
  struct misc_rpmsg_s *priv = priv_;
  clock_synchronize();
  misc_remote_clocksync_(priv->cpuname);
#endif
  return 0;
}

static int misc_remote_envsync_handler(struct rpmsg_endpoint *ept,
                                       void *data, size_t len,
                                       uint32_t src, void *priv_)
{
  struct misc_rpmsg_remote_envsync_s *msg = data;

  if (msg->response)
    {
      if (!msg->result)
        {
          return setenv_global(msg->name, msg->value, 1);
        }
        return 0;
    }
  else
    {
      msg->response = 1;

      char *value = getenv_global(msg->name);
      if (!value)
        {
          msg->result = -EINVAL;
        }
      else
        {
          nbstr2cstr(msg->value, value, 32);
        }
      return rpmsg_send(ept, msg, sizeof(*msg));
    }
}

static int misc_remote_infowrite_handler(struct rpmsg_endpoint *ept,
                                         void *data, size_t len,
                                         uint32_t src, void *priv_)
{
  struct misc_rpmsg_remote_infowrite_s *msg = data;
  int fd;

  fd = open("/dev/onchip-info", 0);
  if (fd < 0)
    {
      syslog(LOG_ERR, "open onchip-info err\n");
    }
  else
    {
      struct song_onchip_env_info_s env =
        {
          .name = msg->name,
          .buf  = msg->value,
          .len  = msg->len,
        };

      if (ioctl(fd, MTDIOC_ENVWRITE, (unsigned long)&env))
        {
          syslog(LOG_ERR, "ioctl MTDIOC_ENVWRITE err\n");
        }
      close(fd);
    }

  return 0;
}

static int misc_retent_save_blk(struct misc_retent_blk_s *blk, int fd)
{
  off_t begin;
  char *temp;
  int ret;

  /* Update block magic and crc */

  blk->magic = MISC_RETENT_MAGIC;
  blk->crc   = crc32((const uint8_t *)blk->base, blk->size);

  /* Check need flush ? If don't, just update header, then jump */

  if (!blk->flush)
    {
      write(fd, blk, sizeof(struct misc_retent_blk_s));
      lseek(fd, blk->size, SEEK_CUR);
      return 0;
    }

  /* Read flash header to check later */

  temp = kmm_malloc(sizeof(struct misc_retent_blk_s));
  if (!temp)
    {
      return -ENOMEM;
    }

  begin = lseek(fd, 0, SEEK_CUR);
  ret = read(fd, temp, sizeof(struct misc_retent_blk_s));
  if (ret < 0)
    {
      goto fail;
    }

  /* Check flash header is equal with block header ?
   * If so, skip it, or flush ram data to flash.
   */

  if (ret != sizeof(struct misc_retent_blk_s) ||
          memcmp(temp, blk, sizeof(struct misc_retent_blk_s)))
    {
      /* Flush block header first */

      lseek(fd, begin, SEEK_SET);
      ret = write(fd, blk, sizeof(struct misc_retent_blk_s));
      if (ret != sizeof(struct misc_retent_blk_s))
        {
          goto fail;
        }

      if (blk->dma)
        {
          write(fd, (void *)blk->base, blk->size);
        }
      else
        {
          char *wabuf;

          /* FLASH/DMAS can't access some memory */

          wabuf = kmm_malloc(blk->size);
          if (!wabuf)
            {
              ret = -ENOMEM;
              goto fail;
            }

          memcpy(wabuf, (void *)blk->base, blk->size);
          write(fd, wabuf, blk->size);

          kmm_free(wabuf);
        }
    }
  else
    {
      /* Skip the flash, don't overwrite */

      lseek(fd, blk->size, SEEK_CUR);
    }

  ret = 0;
fail:
  kmm_free(temp);
  return ret;
}

static int misc_retent_save(struct misc_dev_s *dev, char *file)
{
  struct misc_rpmsg_s *priv = (struct misc_rpmsg_s *)dev;
  struct metal_list *node;
  int ret;
  int fd;

  fd = open(file, O_RDWR | O_CREAT);
  if (fd < 0)
    {
      return -EINVAL;
    }

  metal_list_for_each(&priv->blks, node)
    {
      struct misc_retent_blk_s *blk;

      blk = metal_container_of(node, struct misc_retent_blk_s, node);
      ret = misc_retent_save_blk((struct misc_retent_blk_s *)blk, fd);
      if (ret)
        {
          unlink(file);
          goto fail;
        }
    }

  /* Remove the reset */

  ftruncate(fd, lseek(fd, 0, SEEK_CUR));

  ret = 0;
fail:
  close(fd);
  return ret;
}

static int misc_retent_restore_blk(struct misc_retent_blk_s *blk, int fd)
{
  int ret = 0;

  if (!blk->flush)
    {
      lseek(fd, blk->size, SEEK_CUR);
      goto fail;
    }

  if (blk->dma)
    {
      ret = read(fd, (void *)blk->base, blk->size);
      if (ret != blk->size)
        {
          goto fail;
        }

      if (blk->crc != crc32((const uint8_t *)blk->base, blk->size))
        {
          ret = -EINVAL;
          goto fail;
        }
    }
  else
    {
      char *temp;

      /* FLASH/DMAS can't access some memory */

      temp = kmm_malloc(blk->size);
      if (!temp)
        {
          ret = -ENOMEM;
          goto fail;
        }

      ret = read(fd, temp, blk->size);
      if (ret != blk->size)
        {
          kmm_free(temp);
          goto fail;
        }

      if (blk->crc != crc32((const uint8_t *)temp, blk->size))
        {
          ret = -EINVAL;
          kmm_free(temp);
          goto fail;
        }

      memcpy((void *)blk->base, temp, blk->size);
      kmm_free(temp);
    }

  return 0;
fail:
  memset((void *)blk->base, 0, blk->size);
  return ret;
}

static int misc_retent_restore(struct misc_dev_s *dev, char *file)
{
  struct misc_retent_blk_s *blk;
  int ret;
  int fd;

  fd = open(file, O_RDONLY);
  if (fd < 0)
    {
      return -EINVAL;
    }

  blk = kmm_malloc(sizeof(struct misc_retent_blk_s));
  if (!blk)
    {
      close(fd);
      return -ENOMEM;
    }

  /* Workaround here, don't enter sleep when read flash */

  pm_stay(0, PM_NORMAL);

  while (1)
    {
      ret = read(fd, blk, sizeof(struct misc_retent_blk_s));
      if (ret != sizeof(struct misc_retent_blk_s))
        {
          goto fail;
        }

      if (blk->magic != MISC_RETENT_MAGIC)
        {
          ret = -EINVAL;
          goto fail;
        }

      ret = misc_retent_restore_blk(blk, fd);
      if (ret)
        {
          goto fail;
        }
    }

  ret = 0;
fail:
  pm_relax(0, PM_NORMAL);
  kmm_free(blk);
  close(fd);
  return ret;
}

static int misc_retent_add(struct misc_rpmsg_s *priv, unsigned long arg)
{
  struct misc_retent_add_s *add = (struct misc_retent_add_s *)arg;
  struct misc_rpmsg_retent_add_s msg;

  msg.command = MISC_RPMSG_RETENT_ADD;
  msg.blkid   = add->blkid;
  msg.base    = (uintptr_t)up_addrenv_va_to_pa(add->base);
  msg.size    = add->size;
  msg.dma     = add->dma;

  return rpmsg_send(&priv->ept, &msg, sizeof(struct misc_rpmsg_retent_add_s));
}

static int misc_retent_set(struct misc_rpmsg_s *priv, unsigned long arg)
{
  struct misc_retent_set_s *set = (struct misc_retent_set_s *)arg;
  struct misc_rpmsg_retent_set_s msg;

  msg.command = MISC_RPMSG_RETENT_SET;
  msg.blkid   = set->blkid;
  msg.flush   = set->flush;

  return rpmsg_send(&priv->ept, &msg, sizeof(struct misc_rpmsg_retent_set_s));
}

static int misc_remote_boot(struct misc_rpmsg_s *priv, unsigned long arg)
{
  struct misc_remote_boot_s *remote = (struct misc_remote_boot_s *)arg;
  struct misc_rpmsg_remote_boot_s msg;

  msg.command = MISC_RPMSG_REMOTE_BOOT;
  ncstr2bstr(msg.name, remote->name, 16);

  return rpmsg_send(&priv->ept, &msg, sizeof(struct misc_rpmsg_remote_boot_s));
}

static int misc_remote_clocksync(struct misc_rpmsg_s *priv, unsigned long arg)
{
  struct misc_rpmsg_remote_clocksync_s msg =
    {
      .command = MISC_RPMSG_REMOTE_CLOCKSYNC,
    };

  return rpmsg_send(&priv->ept, &msg, sizeof(msg));
}

static int misc_remote_envsync(struct misc_rpmsg_s *priv, unsigned long arg)
{
  struct misc_remote_envsync_s *env = (struct misc_remote_envsync_s *)arg;
  struct misc_rpmsg_remote_envsync_s msg;

  msg.command  = MISC_RPMSG_REMOTE_ENVSYNC;
  msg.response = 0;
  msg.result   = 0;
  ncstr2bstr(msg.name, env->name, 16);

  return rpmsg_send(&priv->ept, &msg, sizeof(msg));
}

static int misc_remote_infowrite(struct misc_rpmsg_s *priv, unsigned long arg)
{
  struct misc_remote_infowrite_s *env = (struct misc_remote_infowrite_s *)arg;
  struct misc_rpmsg_remote_infowrite_s msg;
  uint32_t len;

  msg.command = MISC_RPMSG_REMOTE_INFOWRITE;
  ncstr2bstr(msg.name, env->name, 16);

  len = env->len > sizeof(msg.value) ? sizeof(msg.value) : env->len;
  memcpy(msg.value, env->value, len);
  msg.len = len;

  return rpmsg_send(&priv->ept, &msg, sizeof(msg));
}

static int misc_dev_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct misc_rpmsg_s *priv = inode->i_private;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case MISC_RETENT_ADD:
        ret = misc_retent_add(priv, arg);
        break;
      case MISC_RETENT_SET:
        ret = misc_retent_set(priv, arg);
        break;
      case MISC_REMOTE_CLOCKSYNC:
        ret = misc_remote_clocksync(priv, arg);
        break;
      case MISC_REMOTE_BOOT:
        ret = misc_remote_boot(priv, arg);
        break;
      case MISC_REMOTE_ENVSYNC:
        ret = misc_remote_envsync(priv, arg);
        break;
      case MISC_REMOTE_INFOWRITE:
        ret = misc_remote_infowrite(priv, arg);
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct misc_dev_s *misc_rpmsg_initialize(const char *cpuname,
                                         bool devctl)
{
  struct misc_rpmsg_s *priv;
  int ret;

  priv = kmm_malloc(sizeof(struct misc_rpmsg_s));
  if (!priv)
    {
      return NULL;
    }

  metal_list_init(&priv->blks);

  priv->cpuname = cpuname;
  priv->dev.ops = &g_misc_ops;

  ret = rpmsg_register_callback(priv,
                                misc_rpmsg_device_created,
                                misc_rpmsg_device_destroy,
                                NULL);
  if (ret)
    {
      kmm_free(priv);
      return NULL;
    }

  if (devctl)
    {
      /* Client */

      register_driver("/dev/misc", &g_misc_devops, 0666, priv);
    }
  else
    {
      /* Server */

      if (g_misc_idx >= MISC_RPMSG_MAX_PRIV)
        {
          return NULL;
        }

      g_misc_priv[g_misc_idx++] = priv;
    }

  return &priv->dev;
}

int misc_rpmsg_clocksync(void)
{
  struct file filep;
  int ret;

  ret = file_open(&filep, "/dev/misc", 0, 0);
  if (ret)
    {
      /* Server */

      return misc_remote_clocksync_(NULL);
    }

  /* Client */

  ret = file_ioctl(&filep, MISC_REMOTE_CLOCKSYNC, 0);
  file_close(&filep);

  return ret;
}
