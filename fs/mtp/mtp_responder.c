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

void mtp_open_session(FAR struct mtp_proto_s *proto)
{
  g_response.length      = 12;
  g_response.type        = MTP_TYPE_RESPONSE;
  g_response.opcode      = MTP_OK_RESP;
  g_response.trans_id    = proto->trans_id;
}

void mtp_get_dev_info(FAR struct mtp_proto_s *proto)
{
  g_response.length      = 73;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_DEV_INFO;
  g_response.trans_id    = proto->trans_id;

  /* uint16 = 100 */

  g_response.payload[0]  = 0x64;
  g_response.payload[1]  = 0x00;

  /* uint32 = 6 , extension */

  g_response.payload[2]  = 0x06;
  g_response.payload[3]  = 0;
  g_response.payload[4]  = 0;
  g_response.payload[5]  = 0;

  /* uint16 = 100 */

  g_response.payload[6]  = 0x64;
  g_response.payload[7]  = 0x00;

  /* size = 1, empty string in unicode */

  g_response.payload[8]  = 1;
  g_response.payload[9]  = 0x00;
  g_response.payload[10] = 0x00;

  /* 00 00 */

  g_response.payload[11] = 0x00;
  g_response.payload[12] = 0x00;

  /* Supported Operations */

  /* First we need to report number of functions */

  g_response.payload[13] = 0x0a;
  g_response.payload[14] = 0x00;
  g_response.payload[15] = 0x00;
  g_response.payload[16] = 0x00;

  /* GetDeviceInfo       = 0x1001 */

  g_response.payload[17] = 0x01;
  g_response.payload[18] = 0x10;

  /* OpenSession         = 0x1002 */

  g_response.payload[19] = 0x02;
  g_response.payload[20] = 0x10;

  /* CloseSession        = 0x1003 */

  g_response.payload[21] = 0x03;
  g_response.payload[22] = 0x10;

  /* GetStorageIDs       = 0x1004 */

  g_response.payload[23] = 0x04;
  g_response.payload[24] = 0x10;

  /* GetStorageInfo      = 0x1005 */

  g_response.payload[25] = 0x05;
  g_response.payload[26] = 0x10;

  /* GetNumObjects       = 0x1006 */

  g_response.payload[27] = 0x06;
  g_response.payload[28] = 0x10;

  /* GetObjectHandles    = 0x1007 */

  g_response.payload[29] = 0x07;
  g_response.payload[30] = 0x10;

  /* GetOjectInfo        = 0x1008 */

  g_response.payload[31] = 0x08;
  g_response.payload[32] = 0x10;

  /* GetObject           = 0x1009 */

  g_response.payload[33] = 0x09;
  g_response.payload[34] = 0x10;

  /* GetPartialObj       = 0x101b */

  g_response.payload[35] = 0x1b;
  g_response.payload[36] = 0x10;

  /* size = 1, empty string in unicode */

  g_response.payload[37] = 1;
  g_response.payload[38] = 0x00;
  g_response.payload[39] = 0x00;

  /* size = 1, empty string in unicode */

  g_response.payload[40] = 1;
  g_response.payload[41] = 0x00;
  g_response.payload[42] = 0x00;

  /* size = 1, empty string in unicode */

  g_response.payload[43] = 1;
  g_response.payload[44] = 0x00;
  g_response.payload[45] = 0x00;

  /* Format Codes */

  g_response.payload[46] = 2;

  /* Code for Undefined Object */

  g_response.payload[47] = 0x00;
  g_response.payload[48] = 0x30;

  /* Code for Association */

  g_response.payload[49] = 0x01;
  g_response.payload[50] = 0x30;

  /* Manufacturer */

  g_response.payload[51] = 1;
  g_response.payload[52] = 0x00;
  g_response.payload[53] = 0x00;

  /* Product */

  g_response.payload[54] = 1;
  g_response.payload[55] = 0x00;
  g_response.payload[56] = 0x00;

  /* version "", size = 1, empty string in unicode */

  g_response.payload[57] = 1;
  g_response.payload[58] = 0x00;
  g_response.payload[59] = 0x00;

  /* end */

  g_response.payload[60] = 0x00;
}

void mtp_get_storage_ids(FAR struct mtp_proto_s *proto)
{
  g_response.length      = 47;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_STORAGE_IDS;
  g_response.trans_id    = proto->trans_id;

  /* StorageID */

  g_response.payload[0]  = 0x01;
  g_response.payload[1]  = 0x00;
  g_response.payload[2]  = 0x00;
  g_response.payload[3]  = 0x00;

  g_response.payload[4]  = 0x01;
  g_response.payload[5]  = 0x00;

  g_response.payload[6]  = 0x01;
  g_response.payload[7]  = 0x00;
}

void mtp_get_storage_info(FAR struct mtp_proto_s *proto)
{
  g_response.length      = 47;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_STORAGE_INFO;
  g_response.trans_id    = proto->trans_id;

  /* Return the StorageID: FixRAM 0x0001 | Hierar: 0x0002 */

  g_response.payload[0]  = 0x03;
  g_response.payload[1]  = 0x00;
  g_response.payload[2]  = 0x02;
  g_response.payload[3]  = 0x00;

  /* Access: Read/Write */

  g_response.payload[4]  = 0x00;
  g_response.payload[5]  = 0x00;

  /* Max size in bytes : 128KB */

  g_response.payload[6]  = 0x00;
  g_response.payload[7]  = 0x00;
  g_response.payload[8]  = 0x02;
  g_response.payload[9]  = 0x00;
  g_response.payload[10] = 0x00;
  g_response.payload[11] = 0x00;
  g_response.payload[12] = 0x00;
  g_response.payload[13] = 0x00;

  /* Free space in bytes : 64KB */

  g_response.payload[14] = 0x00;
  g_response.payload[15] = 0x00;
  g_response.payload[16] = 0x01;
  g_response.payload[17] = 0x00;
  g_response.payload[18] = 0x00;
  g_response.payload[19] = 0x00;
  g_response.payload[20] = 0x00;
  g_response.payload[21] = 0x00;

  /* Free objects :  ~16 Millions of objects */

  g_response.payload[22] = 0x00;
  g_response.payload[23] = 0x00;
  g_response.payload[24] = 0x00;
  g_response.payload[25] = 0x01;

  /* Storage Description: "A" */

  g_response.payload[26] = 0x02;
  g_response.payload[27] = 0x41;
  g_response.payload[28] = 0x00;
  g_response.payload[29] = 0x00;
  g_response.payload[30] = 0x00;

  /* Volume Identifier: "B" */

  g_response.payload[31] = 0x02;
  g_response.payload[32] = 0x42;
  g_response.payload[33] = 0x00;
  g_response.payload[34] = 0x00;
  g_response.payload[35] = 0x00;
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
      case MTP_OPEN_SESSION:     /* MTP OpenSession */
        syslog(LOG_DEBUG, "Open Session Command!\n");
        mtp_open_session(proto);
        break;

      case MTP_GET_DEV_INFO:     /* MTP GetDeviceInfo */
        syslog(LOG_DEBUG, "GetDeviceInfo Command!\n");
        mtp_get_dev_info(proto);
        break;

      case MTP_CLOSE_SESSION:    /* MTP OpenSession */
        syslog(LOG_DEBUG, "Close Session Command!\n");
#if 0 /* REVISIT */
        mtp_close_session(proto);
#endif
        break;

      case MTP_GET_STORAGE_IDS:  /* MTP GetStorageIDs */
        syslog(LOG_DEBUG, "GetStorageIDs Command!\n");
        mtp_get_storage_ids(proto);
        break;

      case MTP_GET_STORAGE_INFO: /* MTP GetStorageInfo */
        syslog(LOG_DEBUG, "GetStorageInfo Command!\n");
        mtp_get_storage_info(proto);
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
