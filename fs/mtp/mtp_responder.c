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

static uint32_t            g_curid    = 1;
struct mtp_resp_s          g_response;
FAR struct mtp_obj_list_s *g_obj_list = NULL;

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

  /* Standard Version: 1.00 */

  g_response.payload[0]  = 0x64;
  g_response.payload[1]  = 0x00;

  /* Vendor Extension ID: 0x06 (Microsoft) */

  g_response.payload[2]  = 0x06;
  g_response.payload[3]  = 0;
  g_response.payload[4]  = 0;
  g_response.payload[5]  = 0;

  /* MTP Version: 1.00 */

  g_response.payload[6]  = 0x64;
  g_response.payload[7]  = 0x00;

  /* MTP Extensions */

  g_response.payload[8]  = 1;
  g_response.payload[9]  = 0x00;
  g_response.payload[10] = 0x00;

  /* Functional Mode: 00 00 */

  g_response.payload[11] = 0x00;
  g_response.payload[12] = 0x00;

  /* Supported Operations */

  /* First we need to report number of functions */

  g_response.payload[13] = 0x0a;
  g_response.payload[14] = 0x00;
  g_response.payload[15] = 0x00;
  g_response.payload[16] = 0x00;

  /* GetDeviceInfo */

  g_response.payload[17] = (MTP_GET_DEV_INFO & 0xff);
  g_response.payload[18] = (MTP_GET_DEV_INFO & 0xff00) >> 8;

  /* OpenSession */

  g_response.payload[19] = (MTP_OPEN_SESSION & 0xff);
  g_response.payload[20] = (MTP_OPEN_SESSION & 0xff00) >> 8;

  /* CloseSession */

  g_response.payload[21] = (MTP_CLOSE_SESSION & 0xff);
  g_response.payload[22] = (MTP_CLOSE_SESSION & 0xff00) >> 8;

  /* GetStorageIDs */

  g_response.payload[23] = (MTP_GET_STORAGE_IDS & 0xff);
  g_response.payload[24] = (MTP_GET_STORAGE_IDS & 0xff00) >> 8;

  /* GetStorageInfo */

  g_response.payload[25] = (MTP_GET_STORAGE_INFO & 0xff);
  g_response.payload[26] = (MTP_GET_STORAGE_INFO & 0xff00) >> 8;

  /* GetNumObjects */

  g_response.payload[27] = (MTP_GET_NUM_OBJS & 0xff);
  g_response.payload[28] = (MTP_GET_NUM_OBJS & 0xff00) >> 8;

  /* GetObjectHandles */

  g_response.payload[29] = (MTP_GET_OBJ_HANDLES & 0xff);
  g_response.payload[30] = (MTP_GET_OBJ_HANDLES & 0xff00) >> 8;

  /* GetOjectInfo */

  g_response.payload[31] = (MTP_GET_OBJ_INFO & 0xff);
  g_response.payload[32] = (MTP_GET_OBJ_INFO & 0xff00) >> 8;

  /* GetObject */

  g_response.payload[33] = (MTP_GET_OBJ & 0xff);
  g_response.payload[34] = (MTP_GET_OBJ & 0xff00) >> 8;

  /* GetPartialObj */

  g_response.payload[35] = (MTP_GET_PARTIAL_OBJ & 0xff);
  g_response.payload[36] = (MTP_GET_PARTIAL_OBJ & 0xff00) >> 8;

  /* Events Supported */

  g_response.payload[37] = 1;
  g_response.payload[38] = 0x00;
  g_response.payload[39] = 0x00;

  /* Device Properties Supported */

  g_response.payload[40] = 1;
  g_response.payload[41] = 0x00;
  g_response.payload[42] = 0x00;

  /* Capture Formats */

  g_response.payload[43] = 1;
  g_response.payload[44] = 0x00;
  g_response.payload[45] = 0x00;

  /* Playback Formats */

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

  /* Model */

  g_response.payload[54] = 1;
  g_response.payload[55] = 0x00;
  g_response.payload[56] = 0x00;

  /* Device Version */

  g_response.payload[57] = 1;
  g_response.payload[58] = 0x00;
  g_response.payload[59] = 0x00;

  /* Serial Number */

  g_response.payload[60] = 0x00;
}

void mtp_get_storage_ids(FAR struct mtp_proto_s *proto)
{
  g_response.length      = 20;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_STORAGE_IDS;
  g_response.trans_id    = proto->trans_id;

  /* Number of Storages: 1 */

  g_response.payload[0]  = 0x01;
  g_response.payload[1]  = 0x00;
  g_response.payload[2]  = 0x00;
  g_response.payload[3]  = 0x00;

  /* StorageID */

  g_response.payload[4]  = (MTP_DEFAULT_STORAGE_ID & 0xff);
  g_response.payload[5]  = (MTP_DEFAULT_STORAGE_ID & 0xff00)     >> 8;
  g_response.payload[6]  = (MTP_DEFAULT_STORAGE_ID & 0xff0000)   >> 16;
  g_response.payload[7]  = (MTP_DEFAULT_STORAGE_ID & 0xff000000) >> 24;
}

void mtp_get_storage_info(FAR struct mtp_proto_s *proto)
{
  g_response.length      = 58;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_STORAGE_INFO;
  g_response.trans_id    = proto->trans_id;

  /* Storage Type */

  g_response.payload[0]  = (MTP_STORAGE_FIXEDRAM & 0xff);
  g_response.payload[1]  = (MTP_STORAGE_FIXEDRAM & 0xff00) >> 8;

  /* File System Type */

  g_response.payload[2]  = (MTP_FS_GENHIERAR & 0xff);
  g_response.payload[3]  = (MTP_FS_GENHIERAR & 0xff00) >> 8;

  /* Access: Read/Write */

  g_response.payload[4]  = (MTP_FS_READWRITE & 0xff);
  g_response.payload[5]  = (MTP_FS_READWRITE & 0xff00) >> 8;

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

  /* Storage Description: "Memory" */

  g_response.payload[26] = 0x07;
  g_response.payload[27] = 'M';
  g_response.payload[28] = 0x00;
  g_response.payload[29] = 'e';
  g_response.payload[30] = 0x00;
  g_response.payload[31] = 'm';
  g_response.payload[32] = 0x00;
  g_response.payload[33] = 'o';
  g_response.payload[34] = 0x00;
  g_response.payload[35] = 'r';
  g_response.payload[36] = 0x00;
  g_response.payload[37] = 'y';
  g_response.payload[38] = 0x00;
  g_response.payload[39] = 0x00;
  g_response.payload[40] = 0x00;

  /* Volume Identifier: "B" */

  g_response.payload[41] = 0x02;
  g_response.payload[42] = 'B';
  g_response.payload[43] = 0x00;
  g_response.payload[44] = 0x00;
  g_response.payload[45] = 0x00;
}

void mtp_get_obj_handles(FAR struct mtp_proto_s *proto)
{
  int ret;

  /* Create the response header, length will be updated later */

  g_response.length      = 0;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_OBJ_HANDLES;
  g_response.trans_id    = proto->trans_id;

  /* Verify is the File System if was not mapped yet */

  if (g_obj_list == NULL)
    {
      ret = map_file_object();
      if (ret < 0)
        {
          syslog(LOG_DEBUG, "Failed to map file system!\n");
        }
    }

  /* Check if we received the right StorageID */

  if (proto->param[0] != MTP_DEFAULT_STORAGE_ID)
    {
      syslog(LOG_DEBUG, "Wrong StorageID = 0x%08X!\n", proto->param[0]);
    }

  /* Only default Object Format is support */

  if (proto->param[1] != MTP_DEFAULT_OBJ_FORMAT)
    {
      syslog(LOG_DEBUG, "Wrong Object Format = 0x%08X!\n", proto->param[1]);

      /* Specification_By_Format_Unsupported */
    }

  /* Is this the Root Directory request? */

  if (proto->param[2] == MTP_ROOT_OBJECT)
    {
      syslog(LOG_DEBUG, "Returning Objects at the Root!\n");
      create_root_resp(&g_response);
    }
  else
    {
      syslog(LOG_DEBUG, "Returning Objects from sub-directories!\n");
    }
}

void mtp_get_obj_info(FAR struct mtp_proto_s *proto)
{
  FAR struct mtp_obj_list_s *obj;
  uint32_t obj_id;
  int pos = 53;

  /* Get the id of the object we need to return */

  obj_id = proto->param[0];

  /* Create the response header */

  g_response.length      = 0;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_OBJ_INFO;
  g_response.trans_id    = proto->trans_id;

  g_response.payload[0]  = 0x01;
  g_response.payload[1]  = 0x00;
  g_response.payload[2]  = 0x01;
  g_response.payload[3]  = 0x00;
  g_response.payload[4]  = 0x01;
  g_response.payload[5]  = 0x30;
  g_response.payload[6]  = 0x00;
  g_response.payload[7]  = 0x00;
  g_response.payload[8]  = 0x00;
  g_response.payload[9]  = 0x00;
  g_response.payload[10] = 0x00;
  g_response.payload[11] = 0x00;
  g_response.payload[12] = 0x00;
  g_response.payload[13] = 0x00;
  g_response.payload[14] = 0x00;
  g_response.payload[15] = 0x00;
  g_response.payload[16] = 0x00;
  g_response.payload[17] = 0x00;
  g_response.payload[18] = 0x00;
  g_response.payload[19] = 0x00;
  g_response.payload[20] = 0x00;
  g_response.payload[21] = 0x00;
  g_response.payload[22] = 0x00;
  g_response.payload[23] = 0x00;
  g_response.payload[24] = 0x00;
  g_response.payload[25] = 0x00;
  g_response.payload[26] = 0x00;
  g_response.payload[27] = 0x00;
  g_response.payload[28] = 0x00;
  g_response.payload[29] = 0x00;
  g_response.payload[30] = 0x00;
  g_response.payload[31] = 0x00;
  g_response.payload[32] = 0x00;
  g_response.payload[33] = 0x00;
  g_response.payload[34] = 0x00;
  g_response.payload[35] = 0x00;
  g_response.payload[36] = 0x00;
  g_response.payload[37] = 0x00;
  g_response.payload[38] = 0x00;
  g_response.payload[39] = 0x00;
  g_response.payload[40] = 0x00;
  g_response.payload[41] = 0x00;

  /* Association Type */

  g_response.payload[42] = 0x01;
  g_response.payload[43] = 0x00;

  /* Association Description */

  g_response.payload[44] = 0x00;
  g_response.payload[45] = 0x00;

  g_response.payload[46] = 0x00;
  g_response.payload[47] = 0x00;
  g_response.payload[48] = 0x00;
  g_response.payload[49] = 0x00;
  g_response.payload[50] = 0x00;
  g_response.payload[51] = 0x00;

  /* Sweep the list of objects */

  obj = g_obj_list;

  while (obj != NULL)
    {
      /* Is this the object we are looking for? */

      if (obj->obj_id == obj_id)
        {
          int i;

          /* Show ID of object found */

          syslog(LOG_DEBUG, "Object 0x%X was found!\n", obj_id);

          /* Is this a file or a directory ? */

          if (obj->is_dir)
            {
              g_response.payload[4]  = 1;
              g_response.payload[42] = 1;
            }
          else
            {
              g_response.payload[4]  = 0;
              g_response.payload[42] = 0;
            }

          /* Filename size */

          g_response.payload[52] = strlen(obj->obj_name);

          for (i = 0; obj->obj_name[i] != '\0' ; i++)
            {
              g_response.payload[pos] = obj->obj_name[i];
              pos++;
              g_response.payload[pos] = 0x00;
              pos++;
            }

          /* End of Unicode: 0x00 0x00 */

          g_response.payload[pos] = 0x00;
          pos++;
          g_response.payload[pos] = 0x00;
          pos++;

          /* No more fields: 0x00 0x00 0x00 */

          g_response.payload[pos] = 0x00;
          pos++;
          g_response.payload[pos] = 0x00;
          pos++;
          g_response.payload[pos] = 0x00;
          pos++;

          /* Update container size */

          g_response.length      = 12 + pos;

          break;
        }

      obj = obj->next;
    }
}

void mtp_get_obj(FAR struct mtp_proto_s *proto)
{
  /* Create the response header */

  g_response.length      = 28;
  g_response.type        = MTP_TYPE_DATA;
  g_response.opcode      = MTP_GET_OBJ;
  g_response.trans_id    = proto->trans_id;

  /* Send the file binary data */

  g_response.payload[0]  = 'T';
  g_response.payload[1]  = 'h';
  g_response.payload[2]  = 'i';
  g_response.payload[3]  = 's';
  g_response.payload[4]  = ' ';
  g_response.payload[5]  = 'i';
  g_response.payload[6]  = 's';
  g_response.payload[7]  = ' ';
  g_response.payload[8]  = 'a';
  g_response.payload[9]  = ' ';
  g_response.payload[10] = 'T';
  g_response.payload[11] = 'e';
  g_response.payload[12] = 's';
  g_response.payload[13] = 't';
  g_response.payload[14] = '!';
  g_response.payload[15] = 0xa;
}

int create_root_resp(FAR struct mtp_resp_s *resp)
{
  FAR struct mtp_obj_list_s *obj;
  uint32_t obj_count = 0;
  int pos            = 4;

  /* Let to search for the objects */

  obj = g_obj_list;

  while (obj != NULL)
    {
      if (obj->in_root)
        {
          obj_count++;
          syslog(LOG_DEBUG, "%d = 0x%08x => %s\n", obj_count,
                                                   obj->obj_id,
                                                   obj->obj_name);
          resp->payload[pos++] = obj->obj_id  & 0xff;
          resp->payload[pos++] = (obj->obj_id & 0xff00)     >> 8;
          resp->payload[pos++] = (obj->obj_id & 0xff0000)   >> 16;
          resp->payload[pos++] = (obj->obj_id & 0xff000000) >> 24;
        }

      obj = obj->next;
    }

  /* Update length with total packet size */

  resp->length = 12 + pos;

  /* Update the amount of objects */

  resp->payload[0] = obj_count & 0xff;
  resp->payload[1] = (obj_count & 0xff00)     >> 8;
  resp->payload[2] = (obj_count & 0xff0000)   >> 16;
  resp->payload[3] = (obj_count & 0xff000000) >> 24;
}

int map_file_object(void)
{
  FAR struct mtp_obj_list_s *first;
  FAR struct mtp_obj_list_s *next;
  FAR char *files[3] = {"Alan", "test.txt", "read.me"};
  int i;

  /* Check if the Object List was created */

  if (g_obj_list == NULL)
    {
      /* Initialyze the Object List */

      g_obj_list = kmm_malloc(sizeof(struct mtp_obj_list_s));
      if (!g_obj_list)
        {
          return -ENOMEM;
        }
    }

  /* Keep an eye on the first object */

  first = g_obj_list;

  /* The first object is a directory */

  g_obj_list->obj_id   = g_curid++;
  g_obj_list->obj_name = strdup(files[0]);
  g_obj_list->is_dir   = true;
  g_obj_list->in_root  = true;
  g_obj_list->next     = NULL;

  for (i = 1; i < 3; i++)
    {
      next = g_obj_list;

      g_obj_list = kmm_malloc(sizeof(struct mtp_obj_list_s));
      if (!g_obj_list)
        {
          return -ENOMEM;
        }

      next->next = g_obj_list;

      g_obj_list->obj_id   = g_curid++;
      g_obj_list->obj_name = strdup(files[i]);
      g_obj_list->is_dir   = false;
      g_obj_list->in_root  = true;
      g_obj_list->next     = NULL;
    }

  /* Return g_obj_list to the first obj */

  g_obj_list = first;

  return OK;
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

      case MTP_GET_OBJ_HANDLES:  /* MTP GetObjectHandles */
        syslog(LOG_DEBUG, "GetObjectHandles Command!\n");
        mtp_get_obj_handles(proto);
        break;

      case MTP_GET_OBJ_INFO:     /* MTP GetObjectInfo */
        syslog(LOG_DEBUG, "GetObjectInfo Command!\n");
        mtp_get_obj_info(proto);
        break;

      case MTP_GET_OBJ:     /* MTP GetObject */
        syslog(LOG_DEBUG, "GetObject Command!\n");
        mtp_get_obj(proto);
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
