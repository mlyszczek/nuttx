/****************************************************************************
 * fs/spiffs.h/spiffs_hydrogen.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of version 0.3.7 of SPIFFS by Peter Andersion.  That
 * version was originally released under the MIT license but is here re-
 * released under the NuttX BSD license.
 *
 *   Copyright (c) 2013-2017 Peter Andersson (pelleplutt1976@gmail.com)
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

#include <sys/statfs.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>

#include "spiffs.h"
#include "spiffs_nucleus.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t spiffs_fflush_cache(FAR struct spiffs_file_s *fobj);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

ssize_t spiffs_hydro_write(FAR struct spiffs_s *fs,
                           FAR struct spiffs_file_s *fobj, FAR void *buffer,
                           off_t offset, size_t len)
{
  ssize_t remaining = len;
  int ret = OK;

  if (fobj->size != SPIFFS_UNDEFINED_LEN && offset < fobj->size)
    {
      int32_t m_len = MIN((int32_t) (fobj->size - offset), len);
      ret = spiffs_object_modify(fs, fobj, offset, (uint8_t *) buffer, m_len);
      SPIFFS_CHECK_RES(ret);
      remaining -= m_len;
      uint8_t *buf_8 = (uint8_t *) buffer;
      buf_8 += m_len;
      buffer = buf_8;
      offset += m_len;
    }

  if (remaining > 0)
    {
      ret = spiffs_object_append(fs, fobj, offset, (uint8_t *) buffer,
                                 remaining);
      SPIFFS_CHECK_RES(ret);
    }

  return len;
}

ssize_t spiffs_hydro_read(FAR struct spiffs_s *fs,
                          FAR struct spiffs_file_s *fobj, FAR void *buffer,
                          size_t buflen)
{
  ssize_t nread;

  if ((fobj->flags & O_RDONLY) == 0)
    {
      return -EACCES;
    }

  if (fobj->size == SPIFFS_UNDEFINED_LEN && buflen > 0)
    {
      /* special case for zero sized files */

      return SPIFFS_ERR_END_OF_OBJECT;
    }

  spiffs_fflush_cache(fobj);

  if (fs->f_pos + buflen >= fobj->size)
    {
      /* reading beyond file size */

      int32_t avail = fobj->size - fs->f_pos;
      if (avail <= 0)
        {
          return SPIFFS_ERR_END_OF_OBJECT;
        }

      nread = spiffs_object_read(fs, fobj, fs->f_pos, avail,
                                 (FAR uint8_t *)buffer);
      if (nread == SPIFFS_ERR_END_OF_OBJECT)
        {
          fs->f_pos += avail;
          return avail;
        }
      else if (nread < 0)
        {
          return (int)nread;
        }
      else
        {
          buflen = avail;
        }
    }
  else
    {
      /* reading within file size */

      nread = spiffs_object_read(fs, fobj, fs->f_pos, buflen,
                                 (FAR uint8_t *)buffer);
      if (nread < 0)
        {
          return (int)nread;
        }
    }

  fs->f_pos += nread;
  return nread;
}

int spiffs_stat_pgndx(FAR struct spiffs_s *fs, int16_t pgndx, int16_t objid,
                    FAR struct stat *buf)
{
  struct spiffs_pgobj_ixheader_s objhdr;
  int16_t ix;
  uint32_t obj_id_addr;
  mode_t mode;
  int ret;

  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ, objid,
                   SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                   sizeof(struct spiffs_pgobj_ixheader_s),
                   (uint8_t *) & objhdr);
  if (ret < 0)
    {
      return ret;
    }

  obj_id_addr = SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, pgndx)) +
    SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pgndx) * sizeof(int16_t);
  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, objid, obj_id_addr,
                   sizeof(int16_t), (FAR uint8_t *)&ix);
  if (ret < 0)
    {
      return ret;
    }

  /* Build the struct stat */

  mode  = S_IRWXO | S_IRWXG  | S_IRWXU;  /* Assume all permisions */
  mode |= S_IFREG;                       /* Assume regular file */

  /* REVISIT:  Should the file object type derive from objhdr.type? */

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = mode;
  buf->st_size = objhdr.size == SPIFFS_UNDEFINED_LEN ? 0 : objhdr.size;

#warning REVISIT: Need st_blksize and st_blocks

  return ret;
}

/* Checks if there are any cached writes for the object ID associated with
 * given file object. If so, these writes are flushed.
 */

static int spiffs_fflush_cache(FAR struct spiffs_file_s *fobj)
{
  FAR struct spiffs_s *fs = fobj->fs;
  int ret = OK;

  if ((fobj->flags & O_DIRECT) == 0)
    {
      if (fobj->cache_page == 0)
        {
          /* see if object ID is associated with cache already */

          fobj->cache_page = spiffs_cache_page_get_byfd(fs, fobj);
        }

      if (fobj->cache_page)
        {
          spiffs_cacheinfo("CACHE_WR_DUMP: dumping cache page " _SPIPRIi
                           " for fobj " _SPIPRIfd ", flush, offs:"
                           _SPIPRIi " size:" _SPIPRIi "\n", fobj->cache_page->ix,
                           fobj->objid, fobj->cache_page->offset, fobj->cache_page->size);
          ret = spiffs_hydro_write(fs, fobj,
                                   spiffs_get_cache_page(fs, spiffs_get_cache(fs), fobj->cache_page->ix),
                                   fobj->cache_page->offset, fobj->cache_page->size);
          if (ret < OK)
            {
              ferr("ERROR: spiffs_hydro_write failed %d\n", ret);
            }

          spiffs_cache_fd_release(fs, fobj->cache_page);
        }
    }

  return ret;
}
