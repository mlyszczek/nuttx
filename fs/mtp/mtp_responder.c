/****************************************************************************
 * fs/tmpfs/mtp_responder.c
 *
 *   Copyright (C) 2019 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <sys/stat.h>
#include <sys/statfs.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/mtp.h>
#include <nuttx/mtp/mtp.h>

#include "mtp_responder.h"

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTP helpers */

/* File system operations */

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct mtp_resp_s g_response;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mtp_open_session(void)
{
  g_response.length   = 12;
  g_response.type     = 3;
  g_response.opcode   = MTP_OK_RESP;
  g_response.trans_id = 0;
}

static void mtp_get_dev_info(void)
{
  g_response.length   = 32;
  g_response.type     = 2;
  g_response.opcode   = 0x1001;
  g_response.trans_id = 1;
  g_response.param[0] = 0x00060064;
  g_response.param[1] = 0x00640000;
  g_response.param[2] = 0x69006d14;
  g_response.param[3] = 0x72006300;
  g_response.param[4] = 0x00006f00;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mtp_process_request(FAR struct mtp_proto_s *proto)
{
  uint16_t mtpcmd;
  int ret = OK;

  /* Get the current command */

  mtpcmd = proto->opcode;

  switch (mtpcmd)
    {
      case MTP_OPEN_SESSION:    /* MTP OpenSession */
        syslog(LOG_DEBUG, "Open Session Command!\n");
        mtp_open_session();
        break;

      case MTP_GET_DEV_INFO:    /* MTP GetDeviceInfo */
        syslog(LOG_DEBUG, "GetDeviceInfo Command!\n");
        mtp_get_dev_info();
        break;

      default:
        syslog(LOG_DEBUG, "Unknown Command!\n");
        if (ret == OK)
          {
            ret = -EINVAL;
          }

        break;
    }

  return ret;
}

FAR uint8_t *mtp_get_response(void)
{
  return (FAR uint8_t *)&g_response;
}

#endif /* CONFIG_DISABLE_MOUNTPOINT */
