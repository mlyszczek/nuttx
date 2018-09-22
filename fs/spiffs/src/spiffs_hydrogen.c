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

int32_t SPIFFS_format(FAR struct spiffs_s *fs)
{
  int32_t ret;

  int16_t blkndx = 0;
  while (blkndx < fs->block_count)
    {
      fs->max_erase_count = 0;
      ret = spiffs_erase_block(fs, blkndx);
      if (ret < 0)
        {
          return ret;
        }

      blkndx++;
    }

  return 0;
}

#if SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH
int32_t SPIFFS_probe_fs(FAR struct spiffs_config_s *config)
{
  finfo("%s\n", __func__);
  int32_t ret = spiffs_probe(config);
  return ret;
}
#endif

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

  ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ, objid,
                   SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                   sizeof(struct spiffs_pgobj_ixheader_s),
                   (uint8_t *) & objhdr);
  if (ret < 0)
    {
      return ret;
    }

  obj_id_addr = SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, pgndx)) +
    SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pgndx) * sizeof(int16_t);
  ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, objid, obj_id_addr,
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

int SPIFFS_rename(FAR struct spiffs_s *fs, FAR const char *oldpath,
                  FAR const char *newpath)
{
  FAR struct spiffs_file_s *fobj;
  int16_t oldpgndx;
  int16_t newpgndx;
  int ret;

  finfo("%s %s %s\n", __func__, oldpath, newpath);

  if (strlen(newpath) > SPIFFS_NAME_MAX - 1 ||
      strlen(oldpath) > SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Get the page index of the object header for the oldpath */

  ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)oldpath, &oldpgndx);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if there is any file object corresponding to the newpath */

  ret = spiffs_find_objhdr_pgndx(fs, (FAR const uint8_t *)newpath, &newpgndx);
  if (ret == -ENOENT)
    {
      ret = OK;
    }
  else if (ret == OK)
    {
      ret = -EEXIST;
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Allocate  new file object */

  fobj = (FAR struct spiffs_file_s *)kmm_zalloc(sizeof(struct spiffs_file_s));
  if (fobj == NULL)
    {
      return -ENOMEM;
    }

  ret = spiffs_object_open_bypage(fs, oldpgndx, fobj, 0, 0);
  if (ret < 0)
    {
      kmm_free(fobj);
      return ret;
    }

  ret = spiffs_object_update_index_hdr(fs, fobj, fobj->objid, fobj->objhdr_pgndx, 0,
                                       (const uint8_t *)newpath, 0, &newpgndx);

  kmm_free(fobj);
  return ret;
}

int32_t SPIFFS_check(FAR struct spiffs_s *fs)
{
  int32_t ret;

  finfo("Entry\n");

  ret = spiffs_lookup_consistency_check(fs, 0);
  ret = spiffs_object_index_consistency_check(fs);
  ret = spiffs_page_consistency_check(fs);
  ret = spiffs_obj_lu_scan(fs);

  return ret;
}

int32_t SPIFFS_gc_quick(FAR struct spiffs_s *fs, uint16_t max_free_pages)
{
  int32_t ret;

  finfo(_SPIPRIi "\n", max_free_pages);

  ret = spiffs_gc_quick(fs, max_free_pages);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

int32_t SPIFFS_gc(FAR struct spiffs_s *fs, uint32_t size)
{
  int32_t ret;

  finfo(_SPIPRIi "\n", size);

  ret = spiffs_gc_check(fs, size);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

int32_t SPIFFS_eof(FAR struct spiffs_s *fs, int16_t objid)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd "\n", objid);

  ret = spiffs_find_fileobject(fs, objid, &fobj);
  if (ret < 0)
    {
      return ret;
    }

  ret = spiffs_fflush_cache(fobj);
  if (ret < 0)
    {
      return ret;
    }

  ret = (fs->f_pos >= (fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size));

  return ret;
}

int32_t SPIFFS_set_file_callback_func(FAR struct spiffs_s *fs, spiffs_file_callback cb_func)
{
  finfo("Entry\n");
  fs->file_cb = cb_func;
  return 0;
}

#if SPIFFS_IX_MAP
int32_t SPIFFS_ix_map(FAR struct spiffs_s *fs, int16_t objid,
                      FAR struct spiffs_ix_map_s * map, off_t offset, uint32_t len,
                      FAR int16_t *map_buf)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd " " _SPIPRIi " " _SPIPRIi "\n", objid, offset, len);

  ret = spiffs_find_fileobject(fs, objid, &fobj);
  if (ret < 0)
    {
      return ret;
    }

  if (fobj->ix_map != NULL)
    {
      return SPIFFS_ERR_IX_MAP_MAPPED;
    }

  map->map_buf = map_buf;
  map->offset = offset;

  /* nb: spndx range includes last */

  map->start_spndx = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  map->end_spndx = (offset + len) / SPIFFS_DATA_PAGE_SIZE(fs);
  memset(map_buf, 0,
         sizeof(int16_t) * (map->end_spndx - map->start_spndx + 1));
  fobj->ix_map = map;

  /* scan for pgndxes */

  ret = spiffs_populate_ix_map(fs, fobj, 0, map->end_spndx - map->start_spndx + 1);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}

int32_t SPIFFS_ix_unmap(FAR struct spiffs_s *fs, int16_t objid)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd "\n", objid);

  ret = spiffs_find_fileobject(fs, objid, &fobj);
  if (ret < 0)
    {
      return ret;
    }

  if (fobj->ix_map == 0)
    {
      return SPIFFS_ERR_IX_MAP_UNMAPPED;
    }

  fobj->ix_map = 0;

  return ret;
}

int32_t SPIFFS_ix_remap(FAR struct spiffs_s *fs, int16_t objid, uint32_t offset)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret = OK;

  finfo(_SPIPRIfd " " _SPIPRIi "\n", objid, offset);

  ret = spiffs_find_fileobject(fs, objid, &fobj);
  if (ret < 0)
    {
      return ret;
    }

  if (fobj->ix_map == 0)
    {
      return SPIFFS_ERR_IX_MAP_UNMAPPED;
    }

  FAR struct spiffs_ix_map_s *map = fobj->ix_map;

  int32_t spndx_diff = offset / SPIFFS_DATA_PAGE_SIZE(fs) - map->start_spndx;
  map->offset = offset;

  /* move existing pgndxes if within map offs */

  if (spndx_diff != 0)
    {
      int i;

      /* move vector. spndx range includes last */

      const int32_t vec_len = map->end_spndx - map->start_spndx + 1;
      map->start_spndx += spndx_diff;
      map->end_spndx += spndx_diff;

      if (spndx_diff >= vec_len)
        {
          /* moving beyond range */

          memset(&map->map_buf, 0, vec_len * sizeof(int16_t));

          /* populate_ix_map is inclusive */

          ret = spiffs_populate_ix_map(fs, fobj, 0, vec_len - 1);
          if (ret < 0)
            {
              return ret;
            }
        }
      else if (spndx_diff > 0)
        {
          /* diff positive */

          for (i = 0; i < vec_len - spndx_diff; i++)
            {
              map->map_buf[i] = map->map_buf[i + spndx_diff];
            }

          /* memset is non-inclusive */

          memset(&map->map_buf[vec_len - spndx_diff], 0,
                 spndx_diff * sizeof(int16_t));

          /* populate_ix_map is inclusive */

          ret = spiffs_populate_ix_map(fs, fobj, vec_len - spndx_diff, vec_len - 1);
        }
      else
        {
          /* diff negative */

          for (i = vec_len - 1; i >= -spndx_diff; i--)
            {
              map->map_buf[i] = map->map_buf[i + spndx_diff];
            }

          /* memset is non-inclusive */

          memset(&map->map_buf[0], 0, -spndx_diff * sizeof(int16_t));

          /* populate_ix_map is inclusive */

          ret = spiffs_populate_ix_map(fs, fobj, 0, -spndx_diff - 1);
        }
    }

  return ret;
}

int32_t SPIFFS_bytes_to_ix_map_entries(FAR struct spiffs_s *fs, uint32_t bytes)
{

  /* always add one extra page, the offset might change to the middle of a page */

  return (bytes + SPIFFS_DATA_PAGE_SIZE(fs)) / SPIFFS_DATA_PAGE_SIZE(fs);
}

int32_t SPIFFS_ix_map_entries_to_bytes(FAR struct spiffs_s *fs, uint32_t map_page_ix_entries)
{
  return map_page_ix_entries * SPIFFS_DATA_PAGE_SIZE(fs);
}

#endif  /* SPIFFS_IX_MAP */
