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
  SPIFFS_LOCK(fs);

  int16_t bix = 0;
  while (bix < fs->block_count)
    {
      fs->max_erase_count = 0;
      ret = spiffs_erase_block(fs, bix);
      if (ret < 0)
        {
          SPIFFS_UNLOCK(fs);
          return ret;
        }

      bix++;
    }

  SPIFFS_UNLOCK(fs);
  return 0;
}

#if SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH
int32_t SPIFFS_probe_fs(spiffs_config * config)
{
  finfo("%s\n", __func__);
  int32_t ret = spiffs_probe(config);
  return ret;
}
#endif

int32_t SPIFFS_mount(FAR struct spiffs_s *fs, spiffs_config * config, uint8_t * work,
                   void *cache, uint32_t cache_size,
                   spiffs_check_callback check_cb)
{
  FAR void *user_data;
  uint8_t ptr_size = sizeof(FAR void *);
  uint8_t addr_lsb;

  finfo("sz:" _SPIPRIi " logpgsz:" _SPIPRIi " logblksz:" _SPIPRIi
        " perasz:" _SPIPRIi " addr:" _SPIPRIad " fdsz:" _SPIPRIi
        " cachesz:" _SPIPRIi "\n",
        SPIFFS_CFG_PHYS_SZ(fs), SPIFFS_CFG_LOG_PAGE_SZ(fs),
        SPIFFS_CFG_LOG_BLOCK_SZ(fs), SPIFFS_CFG_PHYS_ERASE_SZ(fs),
        SPIFFS_CFG_PHYS_ADDR(fs), cache_size);

#warning REVISIT:  We need to allocate the volume structure

  SPIFFS_LOCK(fs);
  user_data = fs->user_data;
  memset(fs, 0, sizeof(struct spiffs_s));
  memcpy(&fs->cfg, config, sizeof(spiffs_config));
  fs->user_data = user_data;
  fs->block_count = SPIFFS_CFG_PHYS_SZ(fs) / SPIFFS_CFG_LOG_BLOCK_SZ(fs);
  fs->work = &work[0];
  fs->lu_work = &work[SPIFFS_CFG_LOG_PAGE_SZ(fs)];

  /* Align cache pointer to 4 byte boundary */

  addr_lsb = ((uint8_t) (intptr_t) cache) & (ptr_size - 1);
  if (addr_lsb)
    {
      FAR uint8_t *cache_8 = (FAR uint8_t *)cache;

      cache_8    += (ptr_size - addr_lsb);
      cache       = cache_8;
      cache_size -= (ptr_size - addr_lsb);
    }

  if (cache_size & (ptr_size - 1))
    {
      cache_size -= (cache_size & (ptr_size - 1));
    }

  fs->cache = cache;
  fs->cache_size =
    (cache_size >
     (SPIFFS_CFG_LOG_PAGE_SZ(fs) * 32)) ? SPIFFS_CFG_LOG_PAGE_SZ(fs) *
    32 : cache_size;
  spiffs_cache_init(fs);

  int32_t ret;

#if SPIFFS_USE_MAGIC
  ret = SPIFFS_CHECK_MAGIC_POSSIBLE(fs) ? OK : SPIFFS_ERR_MAGIC_NOT_POSSIBLE;
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }
#endif

  fs->config_magic = SPIFFS_SUPER_MAGIC;

  ret = spiffs_obj_lu_scan(fs);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  finfo("page index byte len:         " _SPIPRIi "\n",
        (uint32_t) SPIFFS_CFG_LOG_PAGE_SZ(fs));
  finfo("object lookup pages:         " _SPIPRIi "\n",
        (uint32_t) SPIFFS_OBJ_LOOKUP_PAGES(fs));
  finfo("page pages per block:        " _SPIPRIi "\n",
        (uint32_t) SPIFFS_PAGES_PER_BLOCK(fs));
  finfo("page header length:          " _SPIPRIi "\n",
        (uint32_t) sizeof(struct spiffs_page_header_s));
  finfo("object header index entries: " _SPIPRIi "\n",
        (uint32_t) SPIFFS_OBJ_HDR_IX_LEN(fs));
  finfo("object index entries:        " _SPIPRIi "\n",
        (uint32_t) SPIFFS_OBJ_IX_LEN(fs));
  finfo("free blocks:                 " _SPIPRIi "\n",
        (uint32_t) fs->free_blocks);

  fs->check_cb = check_cb;
  fs->mounted = 1;
  SPIFFS_UNLOCK(fs);
  return 0;
}

void SPIFFS_unmount(FAR struct spiffs_s *fs)
{
  FAR struct spiffs_file_s *fobj;

  finfo("Entry\n");

  SPIFFS_LOCK(fs);
  while ((fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq)) != NULL)
    {
      /* Free the file object */

      spiffs_file_free(fs, fobj);
    }

  /* Free the volume memory (note that the semaphore is now stale!) */

  kmm_free(fobj);
}

ssize_t spiffs_hydro_write(FAR struct spiffs_s *fs,
                           FAR struct spiffs_file_s *fobj, FAR void *buffer,
                           off_t offset, size_t len)
{
  ssize_t remaining = len;
  int ret = OK;

  if (fobj->size != SPIFFS_UNDEFINED_LEN && offset < fobj->size)
    {
      int32_t m_len = MIN((int32_t) (fobj->size - offset), len);
      ret = spiffs_object_modify(fobj, offset, (uint8_t *) buffer, m_len);
      SPIFFS_CHECK_RES(ret);
      remaining -= m_len;
      uint8_t *buf_8 = (uint8_t *) buffer;
      buf_8 += m_len;
      buffer = buf_8;
      offset += m_len;
    }

  if (remaining > 0)
    {
      ret = spiffs_object_append(fobj, offset, (uint8_t *) buffer, remaining);
      SPIFFS_CHECK_RES(ret);
    }

  return len;
}

ssize_t spiffs_hydro_read(FAR struct spiffs_s *fs,
                          FAR struct spiffs_file_s *fobj, FAR void *buffer,
                          size_t buflen)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      return ret;
    }

  if ((fobj->flags & O_RDONLY) == 0)
    {
      return SPIFFS_ERR_NOT_READABLE;
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

      ret = spiffs_object_read(fobj, fs->f_pos, avail, (FAR uint8_t *)buffer);
      if (ret == SPIFFS_ERR_END_OF_OBJECT)
        {
          fs->f_pos += avail;
          return avail;
        }
      else if (ret < 0)
        {
          return ret;
        }
      else
        {
          buflen = avail;
        }
    }
  else
    {
      /* reading within file size */

      ret = spiffs_object_read(fobj, fs->f_pos, buflen, (FAR uint8_t *)buffer);
      if (ret < 0)
        {
          return ret;
        }
    }

  fs->f_pos += buflen;
  return buflen;
}

int32_t SPIFFS_remove(FAR struct spiffs_s *fs, const char *path)
{
  FAR struct spiffs_file_s *fobj;
  int16_t pix;
  int32_t ret;


  finfo("'%s'\n", path);

  if (strlen(path) > SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Allocate  new file object */

  fobj = (FAR struct spiffs_file_s *)kmm_zalloc(sizeof(struct spiffs_file_s));
  if (fobj == NULL)
    {
      SPIFFS_UNLOCK(fs);
      return -ENOMEM;
    }

  ret = spiffs_object_find_object_index_header_by_name(fs, (const uint8_t *)path,
                                                       &pix);
  if (ret < OK)
    {
      SPIFFS_UNLOCK(fs);
      kmm_free(fobj);
      return ret;
    }

  ret = spiffs_object_open_by_page(fs, pix, fobj, 0, 0);
  if (ret != OK)
    {
      SPIFFS_UNLOCK(fs);
      kmm_free(fobj);
      return ret;
    }

  ret = spiffs_object_truncate(fobj, 0, 1);
  if (ret != OK)
    {
      SPIFFS_UNLOCK(fs);
      kmm_free(fobj);
      return ret;
    }

  SPIFFS_UNLOCK(fs);
  return 0;
}

int32_t SPIFFS_fremove(FAR struct spiffs_s *fs, int16_t id)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd "\n", id);
  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }


  if ((fobj->flags & O_WRONLY) == 0)
    {
      SPIFFS_UNLOCK(fs);
      return SPIFFS_ERR_NOT_WRITABLE;
    }

  spiffs_cache_fd_release(fs, fobj->cache_page);
  ret = spiffs_object_truncate(fobj, 0, 1);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  SPIFFS_UNLOCK(fs);
  return 0;
}

int spiffs_stat_pix(FAR struct spiffs_s *fs, int16_t pix, int16_t id,
                    FAR struct stat *buf)
{
  struct spiffs_pgobj_ixheader_s objix_hdr;
  int16_t ix;
  uint32_t obj_id_addr;
  int32_t ret;

  ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ, id,
                   SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(struct spiffs_pgobj_ixheader_s),
                   (uint8_t *) & objix_hdr);
  if (ret < 0)
    {
      return ret;
    }

  obj_id_addr = SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, pix)) +
    SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pix) * sizeof(int16_t);
  ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, id, obj_id_addr,
                   sizeof(int16_t), (FAR uint8_t *)&ix);
  if (ret < 0)
    {
      return ret;
    }

  buf->id   = ix & ~SPIFFS_OBJ_ID_IX_FLAG;
  buf->type = objix_hdr.type;
  buf->size = objix_hdr.size == SPIFFS_UNDEFINED_LEN ? 0 : objix_hdr.size;
  buf->pix  = pix;

  strncpy((char *)buf->name, (char *)objix_hdr.name, SPIFFS_NAME_MAX);
#if SPIFFS_OBJ_META_LEN
  memcpy(buf->meta, objix_hdr.meta, SPIFFS_OBJ_META_LEN);
#endif

  return ret;
}

int32_t SPIFFS_stat(FAR struct spiffs_s *fs, const char *path, FAR struct stat *buf)
{
  int32_t ret;
  int16_t pix;

  finfo("'%s'\n", path);

  if (strlen(path) > SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  SPIFFS_LOCK(fs);

  ret = spiffs_object_find_object_index_header_by_name(fs, (const uint8_t *)path,
                                                       &pix);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  ret = spiffs_stat_pix(fs, pix, 0, buf);

  SPIFFS_UNLOCK(fs);
  return ret;
}

/* Checks if there are any cached writes for the object id associated with
 * given file object. If so, these writes are flushed.
 */

static int spiffs_fflush_cache(FAR struct spiffs_file_s *fobj)
{
  FAR struct spiffs_s *fs = fobj->fs;
  int ret;

  if ((fobj->flags & O_DIRECT) == 0)
    {
      if (fobj->cache_page == 0)
        {
          /* see if object id is associated with cache already */

          fobj->cache_page = spiffs_cache_page_get_by_fd(fs, fobj);
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
              fs->err_code = ret;
            }

          spiffs_cache_fd_release(fs, fobj->cache_page);
        }
    }

  return ret;
}

int32_t SPIFFS_rename(FAR struct spiffs_s *fs, const char *old_path, const char *new_path)
{
  int16_t pix_old;
  int16_t pix_dummy;
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo("%s %s %s\n", __func__, old_path, new_path);

  if (strlen(new_path) > SPIFFS_NAME_MAX - 1 ||
      strlen(old_path) > SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  SPIFFS_LOCK(fs);

  ret = spiffs_object_find_object_index_header_by_name(fs, (const uint8_t *)old_path,
                                                       &pix_old);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  ret = spiffs_object_find_object_index_header_by_name(fs, (const uint8_t *)new_path,
                                                       &pix_dummy);
  if (ret == SPIFFS_ERR_NOT_FOUND)
    {
      ret = OK;
    }
  else if (ret == OK)
    {
      ret = SPIFFS_ERR_CONFLICTING_NAME;
    }

  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  /* Allocate  new file object */

  fobj = (FAR struct spiffs_file_s *)kmm_zalloc(sizeof(struct spiffs_file_s));
  if (fobj == NULL)
    {
      return -ENOMEM;
    }

  ret = spiffs_object_open_by_page(fs, pix_old, fobj, 0, 0);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      kmm_free(fobj);
      return ret;
    }


  ret = spiffs_object_update_index_hdr(fs, fobj, fobj->objid, fobj->objix_hdr_pix, 0,
                                       (const uint8_t *)new_path, 0, 0, &pix_dummy);

  SPIFFS_UNLOCK(fs);
  kmm_free(fobj);
  return ret;
}

#if SPIFFS_OBJ_META_LEN
int32_t SPIFFS_update_meta(FAR struct spiffs_s *fs, const char *name, const void *meta)
{
  int16_t pix, pix_dummy;
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  SPIFFS_LOCK(fs);

  ret = spiffs_object_find_object_index_header_by_name(fs, (const uint8_t *)name,
                                                       &pix);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  /* Allocate  new file object */

  fobj = (FAR struct spiffs_file_s *)kmm_zalloc((struct spiffs_file_s));
  if (fobj == NULL)
    {
      return -ENOMEM;
    }

  ret = spiffs_object_open_by_page(fs, pix, fobj, 0, 0);
  if (ret != OK)
    {
      SPIFFS_UNLOCK(fs);
      kmm_free(fobj);
      return ret;
    }

  ret = spiffs_object_update_index_hdr(fs, fobj, fobj->objid, fobj->objix_hdr_pix, 0, 0,
                                       meta, 0, &pix_dummy);

  kmm_free(fobj);
  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_fupdate_meta(FAR struct spiffs_s *fs, int16_t id, const void *meta)
{
  int32_t ret;
  FAR struct spiffs_file_s *fobj;
  int16_t pix_dummy;

  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  if ((fobj->flags & O_WRONLY) == 0)
    {
      SPIFFS_UNLOCK(fs);
      return SPIFFS_ERR_NOT_WRITABLE;
    }

  ret = spiffs_object_update_index_hdr(fs, fobj, fobj->objid, fobj->objix_hdr_pix, 0, 0,
                                       meta, 0, &pix_dummy);
  SPIFFS_UNLOCK(fs);
  return ret;
}
#endif  /* SPIFFS_OBJ_META_LEN */

int32_t SPIFFS_check(FAR struct spiffs_s *fs)
{
  int32_t ret;

  finfo("Entry\n");
  SPIFFS_LOCK(fs);

  ret = spiffs_lookup_consistency_check(fs, 0);
  ret = spiffs_object_index_consistency_check(fs);
  ret = spiffs_page_consistency_check(fs);
  ret = spiffs_obj_lu_scan(fs);

  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_info(FAR struct spiffs_s *fs, uint32_t * total, uint32_t * used)
{
  int32_t ret = OK;
  uint32_t pages_per_block;
  uint32_t blocks;
  uint32_t obj_lu_pages;
  uint32_t data_page_size;
  uint32_t total_data_pages;

  finfo("Entry\n");
  SPIFFS_LOCK(fs);

  pages_per_block = SPIFFS_PAGES_PER_BLOCK(fs);
  blocks = fs->block_count;
  obj_lu_pages = SPIFFS_OBJ_LOOKUP_PAGES(fs);
  data_page_size = SPIFFS_DATA_PAGE_SIZE(fs);

   /* -2 for  spare blocks, +1 for emergency page */

  total_data_pages = (blocks - 2) * (pages_per_block - obj_lu_pages) + 1;

  if (total)
    {
      *total = total_data_pages * data_page_size;
    }

  if (used)
    {
      *used = fs->stats_p_allocated * data_page_size;
    }

  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_gc_quick(FAR struct spiffs_s *fs, uint16_t max_free_pages)
{
  int32_t ret;

  finfo(_SPIPRIi "\n", max_free_pages);
  SPIFFS_LOCK(fs);

  ret = spiffs_gc_quick(fs, max_free_pages);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  SPIFFS_UNLOCK(fs);
  return 0;
}

int32_t SPIFFS_gc(FAR struct spiffs_s *fs, uint32_t size)
{
  int32_t ret;

  finfo(_SPIPRIi "\n", size);
  SPIFFS_LOCK(fs);

  ret = spiffs_gc_check(fs, size);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  SPIFFS_UNLOCK(fs);
  return 0;
}

int32_t SPIFFS_eof(FAR struct spiffs_s *fs, int16_t id)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd "\n", id);
  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  ret = spiffs_fflush_cache(fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  ret = (fs->f_pos >= (fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size));

  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_tell(FAR struct spiffs_s *fs, int16_t id)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd "\n", id);
  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  ret = spiffs_fflush_cache(fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  ret = fs->f_pos;

  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_set_file_callback_func(FAR struct spiffs_s *fs, spiffs_file_callback cb_func)
{
  finfo("Entry\n");
  SPIFFS_LOCK(fs);
  fs->file_cb = cb_func;
  SPIFFS_UNLOCK(fs);
  return 0;
}

#if SPIFFS_IX_MAP
int32_t SPIFFS_ix_map(FAR struct spiffs_s *fs, int16_t id,
                      FAR struct spiffs_ix_map_s * map, off_t offset, uint32_t len,
                      FAR int16_t *map_buf)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd " " _SPIPRIi " " _SPIPRIi "\n", id, offset, len);
  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  if (fobj->ix_map != NULL)
    {
      SPIFFS_UNLOCK(fs);
      return SPIFFS_ERR_IX_MAP_MAPPED;
    }

  map->map_buf = map_buf;
  map->offset = offset;

  /* nb: spix range includes last */

  map->start_spix = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  map->end_spix = (offset + len) / SPIFFS_DATA_PAGE_SIZE(fs);
  memset(map_buf, 0,
         sizeof(int16_t) * (map->end_spix - map->start_spix + 1));
  fobj->ix_map = map;

  /* scan for pixes */

  ret = spiffs_populate_ix_map(fs, fobj, 0, map->end_spix - map->start_spix + 1);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_ix_unmap(FAR struct spiffs_s *fs, int16_t id)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret;

  finfo(_SPIPRIfd "\n", id);
  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  if (fobj->ix_map == 0)
    {
      SPIFFS_UNLOCK(fs);
      return SPIFFS_ERR_IX_MAP_UNMAPPED;
    }

  fobj->ix_map = 0;

  SPIFFS_UNLOCK(fs);
  return ret;
}

int32_t SPIFFS_ix_remap(FAR struct spiffs_s *fs, int16_t id, uint32_t offset)
{
  FAR struct spiffs_file_s *fobj;
  int32_t ret = OK;

  finfo(_SPIPRIfd " " _SPIPRIi "\n", id, offset);
  SPIFFS_LOCK(fs);

  ret = spiffs_find_fileobject(fs, id, &fobj);
  if (ret < 0)
    {
      SPIFFS_UNLOCK(fs);
      return ret;
    }

  if (fobj->ix_map == 0)
    {
      SPIFFS_UNLOCK(fs);
      return SPIFFS_ERR_IX_MAP_UNMAPPED;
    }

  FAR struct spiffs_ix_map_s *map = fobj->ix_map;

  int32_t spix_diff = offset / SPIFFS_DATA_PAGE_SIZE(fs) - map->start_spix;
  map->offset = offset;

  /* move existing pixes if within map offs */

  if (spix_diff != 0)
    {
      int i;

      /* move vector. spix range includes last */

      const int32_t vec_len = map->end_spix - map->start_spix + 1;
      map->start_spix += spix_diff;
      map->end_spix += spix_diff;

      if (spix_diff >= vec_len)
        {
          /* moving beyond range */

          memset(&map->map_buf, 0, vec_len * sizeof(int16_t));

          /* populate_ix_map is inclusive */

          ret = spiffs_populate_ix_map(fs, fobj, 0, vec_len - 1);
          if (ret < 0)
            {
              SPIFFS_UNLOCK(fs);
              return ret;
            }
        }
      else if (spix_diff > 0)
        {
          /* diff positive */

          for (i = 0; i < vec_len - spix_diff; i++)
            {
              map->map_buf[i] = map->map_buf[i + spix_diff];
            }

          /* memset is non-inclusive */

          memset(&map->map_buf[vec_len - spix_diff], 0,
                 spix_diff * sizeof(int16_t));

          /* populate_ix_map is inclusive */

          ret = spiffs_populate_ix_map(fs, fobj, vec_len - spix_diff, vec_len - 1);
        }
      else
        {
          /* diff negative */

          for (i = vec_len - 1; i >= -spix_diff; i--)
            {
              map->map_buf[i] = map->map_buf[i + spix_diff];
            }

          /* memset is non-inclusive */

          memset(&map->map_buf[0], 0, -spix_diff * sizeof(int16_t));

          /* populate_ix_map is inclusive */

          ret = spiffs_populate_ix_map(fs, fobj, 0, -spix_diff - 1);
        }
    }

  SPIFFS_UNLOCK(fs);
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
