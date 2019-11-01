/********************************************************************************************
 * include/nuttx/misc.h
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_MISC_H
#define __INCLUDE_NUTTX_MISC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/fs/ioctl.h>

#include <string.h>

#ifdef CONFIG_MISC_RPMSG

#include <openamp/open_amp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MISC_RETENT_ADD         _MISCIOC(1)
#define MISC_RETENT_SET         _MISCIOC(2)
#define MISC_REMOTE_BOOT        _MISCIOC(3)
#define MISC_REMOTE_CLOCKSYNC   _MISCIOC(4)
#define MISC_REMOTE_ENVSYNC     _MISCIOC(5)
#define MISC_REMOTE_INFOWRITE   _MISCIOC(6)

/* Access macros ************************************************************/

/****************************************************************************
 * Name: MISC_RETENT_SAVE
 *
 * Description:
 *   Save ram data to flash
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   file - File to be saved to
 *
 * Returned Value:
 *   Zero on success, NULL on failure.
 *
 ****************************************************************************/

#define MISC_RETENT_SAVE(d,f) ((d)?(d)->ops->retent_save((d),(f)):0)

/****************************************************************************
 * Name: MISC_RETENT_RESTORE
 *
 * Description:
 *   Restore from flash
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   file - File to be restored from
 *
 * Returned Value:
 *   Zero on success, NULL on failure.
 *
 ****************************************************************************/

#define MISC_RETENT_RESTORE(d,f) ((d)?(d)->ops->retent_restore((d),(f)):0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct misc_dev_s;
struct misc_ops_s
{
  int (*retent_save)(struct misc_dev_s *dev, char *file);
  int (*retent_restore)(struct misc_dev_s *dev, char *file);
};

struct misc_dev_s
{
  const struct misc_ops_s *ops;
};

struct misc_retent_add_s
{
  uint32_t blkid;
  void     *base;
  uint32_t size;
  bool     dma;
};

struct misc_retent_set_s
{
  uint32_t blkid;
  uint32_t flush;
};

struct misc_remote_boot_s
{
  const char *name;
};

struct misc_remote_envsync_s
{
  const char *name;
  const char *value;
};

struct misc_remote_infowrite_s
{
  const char *name;
  uint8_t    *value;
  uint32_t    len;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct misc_dev_s *misc_rpmsg_initialize(const char *cpuname,
                                         bool devctl);
int misc_rpmsg_clocksync(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MISC_RPMSG */
#endif /* __INCLUDE_NUTTX_MISC_H */
