/****************************************************************************
 * fs/spiffs.h/spiffs_cache.c
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

#include <nuttx/mtd/mtd.h>

#include "spiffs.h"
#include "spiffs_mtd.h"
#include "spiffs_nucleus.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* returns cached page for give page index, or null if no such cached page */

static FAR struct spiffs_cache_page_s *spiffs_cache_page_get(FAR struct spiffs_s *fs,
                                                int16_t pgndx)
{
  int i;

  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  if ((cache->cpage_use_map & cache->cpage_use_mask) == 0)
    {
      return 0;
    }

  for (i = 0; i < cache->cpage_count; i++)
    {
      FAR struct spiffs_cache_page_s *cp = spiffs_get_cache_page_hdr(fs, cache, i);
      if ((cache->cpage_use_map & (1 << i)) &&
          (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR) == 0 && cp->pgndx == pgndx)
        {
          cp->last_access = cache->last_access;
          return cp;
        }
    }

  return 0;
}

/* frees cached page */

static int32_t spiffs_cache_page_free(FAR struct spiffs_s *fs, int ix, uint8_t write_back)
{
  int32_t res = OK;
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  FAR struct spiffs_cache_page_s *cp = spiffs_get_cache_page_hdr(fs, cache, ix);

  if (cache->cpage_use_map & (1 << ix))
    {
      if (write_back &&
          (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR) == 0 &&
          (cp->flags & SPIFFS_CACHE_FLAG_DIRTY))
        {
          uint8_t *mem = spiffs_get_cache_page(fs, cache, ix);
          spiffs_cacheinfo("CACHE_FREE: write cache page " _SPIPRIi " pgndx "
                           _SPIPRIpg "\n", ix, cp->pgndx);
          res =
            spiffs_mtd_write(fs, SPIFFS_PAGE_TO_PADDR(fs, cp->pgndx),
                             SPIFFS_CFG_LOG_PAGE_SZ(fs), mem);
        }

      if (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR)
        {
          spiffs_cacheinfo("CACHE_FREE: free cache page " _SPIPRIi " objid "
                           _SPIPRIid "\n", ix, cp->objid);
        }
      else
        {
          spiffs_cacheinfo("CACHE_FREE: free cache page " _SPIPRIi " pgndx "
                           _SPIPRIpg "\n", ix, cp->pgndx);
        }

      cache->cpage_use_map &= ~(1 << ix);
      cp->flags = 0;
    }

  return res;
}

/* removes the oldest accessed cached page */

static int32_t spiffs_cache_page_remove_oldest(FAR struct spiffs_s *fs, uint8_t flag_mask,
                                               uint8_t flags)
{
  int32_t res = OK;
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  uint32_t oldest_val = 0;
  int cand_ix = -1;
  int i;

  if ((cache->cpage_use_map & cache->cpage_use_mask) != cache->cpage_use_mask)
    {
      /* at least one free cpage */

      return OK;
    }

  /* all busy, scan thru all to find the cpage which has oldest access */

  for (i = 0; i < cache->cpage_count; i++)
    {
      FAR struct spiffs_cache_page_s *cp = spiffs_get_cache_page_hdr(fs, cache, i);
      if ((cache->last_access - cp->last_access) > oldest_val &&
          (cp->flags & flag_mask) == flags)
        {
          oldest_val = cache->last_access - cp->last_access;
          cand_ix = i;
        }
    }

  if (cand_ix >= 0)
    {
      res = spiffs_cache_page_free(fs, cand_ix, 1);
    }

  return res;
}

/* allocates a new cached page and returns it, or null if all cache pages
 * are busy.
 */

static FAR struct spiffs_cache_page_s *spiffs_cache_page_allocate(FAR struct spiffs_s *fs)
{
  int i;

  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  if (cache->cpage_use_map == 0xffffffff)
    {
      /* out of cache memory */

      return 0;
    }

  for (i = 0; i < cache->cpage_count; i++)
    {
      if ((cache->cpage_use_map & (1 << i)) == 0)
        {
          FAR struct spiffs_cache_page_s *cp = spiffs_get_cache_page_hdr(fs, cache, i);
          cache->cpage_use_map |= (1 << i);
          cp->last_access = cache->last_access;
          return cp;
        }
    }

  /* out of cache entries */

  return 0;
}

/* drops the cache page for give page index */

void spiffs_cache_drop_page(FAR struct spiffs_s *fs, int16_t pgndx)
{
  FAR struct spiffs_cache_page_s *cp = spiffs_cache_page_get(fs, pgndx);
  if (cp)
    {
      spiffs_cache_page_free(fs, cp->ix, 0);
    }
}

/* reads from spi flash or the cache */

int32_t spiffs_phys_rd(FAR struct spiffs_s *fs, uint8_t op, int16_t objid,
                       uint32_t addr, uint32_t len, uint8_t *dst)
{
  int32_t res = OK;
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  FAR struct spiffs_cache_page_s *cp =
    spiffs_cache_page_get(fs, SPIFFS_PADDR_TO_PAGE(fs, addr));

  cache->last_access++;
  if (cp)
    {
      /* we've already got one, you see */

#if CONFIG_SPIFFS_CACHEDBG
      fs->cache_hits++;
#endif
      cp->last_access = cache->last_access;
      uint8_t *mem = spiffs_get_cache_page(fs, cache, cp->ix);
      memcpy(dst, &mem[SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)], len);
    }
  else
    {
      if ((op & SPIFFS_OP_TYPE_MASK) == SPIFFS_OP_T_OBJ_LU2)
        {
          /* For second layer lookup functions, we do not cache in order to
           * prevent shredding
           */

          return spiffs_mtd_read(fs, addr, len, dst);
        }
#  if CONFIG_SPIFFS_CACHEDBG
      fs->cache_misses++;
#  endif
      /* this operation will always free one cache page (unless all already
       * free), the result code stems from the write operation of the
       * possibly freed cache page
       */

      res = spiffs_cache_page_remove_oldest(fs, SPIFFS_CACHE_FLAG_TYPE_WR, 0);

      cp = spiffs_cache_page_allocate(fs);
      if (cp)
        {
          FAR uint8_t *mem;
          int32_t res2;

          cp->flags = SPIFFS_CACHE_FLAG_WRTHRU;
          cp->pgndx = SPIFFS_PADDR_TO_PAGE(fs, addr);
          spiffs_cacheinfo("CACHE_ALLO: allocated cache page " _SPIPRIi
                           " for pgndx " _SPIPRIpg "\n", cp->ix, cp->pgndx);

          res2 = spiffs_mtd_read(fs, addr -
                                 SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr),
                                 SPIFFS_CFG_LOG_PAGE_SZ(fs),
                                 spiffs_get_cache_page(fs, cache, cp->ix));
          if (res2 != OK)
            {
              /* honor read failure before possible write failure (bad idea?) */

              res = res2;
            }

          mem = spiffs_get_cache_page(fs, cache, cp->ix);
          memcpy(dst, &mem[SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)], len);
        }
      else
        {
          /* this will never happen, last resort for sake of symmetry */

          int32_t res2 = spiffs_mtd_read(fs, addr, len, dst);
          if (res2 != OK)
            {
              /* honor read failure before possible write failure (bad idea?) */

              res = res2;
            }
        }
    }

  return res;
}

/* writes to spi flash and/or the cache */

int32_t spiffs_phys_wr(FAR struct spiffs_s *fs, uint8_t op, int16_t objid,
                       uint32_t addr, uint32_t len, uint8_t *src)
{
  int16_t pgndx = SPIFFS_PADDR_TO_PAGE(fs, addr);
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  FAR struct spiffs_cache_page_s *cp = spiffs_cache_page_get(fs, pgndx);

  if (cp && (op & SPIFFS_OP_COM_MASK) != SPIFFS_OP_C_WRTHRU)
    {
      /* Have a cache page.  Copy in data to cache page */

      if ((op & SPIFFS_OP_COM_MASK) == SPIFFS_OP_C_DELE &&
          (op & SPIFFS_OP_TYPE_MASK) != SPIFFS_OP_T_OBJ_LU)
        {
          /* Page is being deleted, wipe from cache - unless it is a lookup
           * page
           */

          spiffs_cache_page_free(fs, cp->ix, 0);
          return spiffs_mtd_write(fs, addr, len, src);
        }

      uint8_t *mem = spiffs_get_cache_page(fs, cache, cp->ix);
      memcpy(&mem[SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)], src, len);

      cache->last_access++;
      cp->last_access = cache->last_access;

      if (cp->flags & SPIFFS_CACHE_FLAG_WRTHRU)
        {
          /* page is being updated, no write-cache, just pass thru */

          return spiffs_mtd_write(fs, addr, len, src);
        }
      else
        {
          return OK;
        }
    }
  else
    {
      /* no cache page, no write cache - just write thru */

      return spiffs_mtd_write(fs, addr, len, src);
    }
}

/* Returns the cache page that this fobj refers, or null if no cache page */

FAR struct spiffs_cache_page_s *spiffs_cache_page_get_byfd(FAR struct spiffs_s *fs,
                                                            FAR struct spiffs_file_s *fobj)
{
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  int i;

  if ((cache->cpage_use_map & cache->cpage_use_mask) == 0)
    {
      /* All cpages free, no cpage cannot be assigned to the ID */

      return 0;
    }

  for (i = 0; i < cache->cpage_count; i++)
    {
      FAR struct spiffs_cache_page_s *cp = spiffs_get_cache_page_hdr(fs, cache, i);
      if ((cache->cpage_use_map & (1 << i)) &&
          (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR) && cp->objid == fobj->objid)
        {
          return cp;
        }
    }

  return 0;
}

/* Allocates a new cache page and refers this to given fobj - flushes an old cache
 * page if all cache is busy
 */

FAR struct spiffs_cache_page_s *
spiffs_cache_page_allocate_byfd(FAR struct spiffs_s *fs,
                                 FAR struct spiffs_file_s *fobj)
{
  /* before this function is called, it is ensured that there is no already
   * existing cache page with same object ID
   */

  spiffs_cache_page_remove_oldest(fs, SPIFFS_CACHE_FLAG_TYPE_WR, 0);
  FAR struct spiffs_cache_page_s *cp = spiffs_cache_page_allocate(fs);
  if (cp == 0)
    {
      /* could not get cache page */

      return 0;
    }

  cp->flags = SPIFFS_CACHE_FLAG_TYPE_WR;
  cp->objid = fobj->objid;
  fobj->cache_page = cp;
  spiffs_cacheinfo("CACHE_ALLO: allocated cache page " _SPIPRIi " for fobj "
                   _SPIPRIfd "\n", cp->ix, fobj->objid);
  return cp;
}

/* Unrefers all file objects that this cache page refers to and releases the cache
 * page
 */

void spiffs_cache_fd_release(FAR struct spiffs_s *fs, FAR struct spiffs_cache_page_s *cp)
{
  FAR struct spiffs_file_s *fobj;

  if (cp == 0)
    {
      return;
    }

  for (fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq);
       fobj != NULL;
       fobj  = (FAR struct spiffs_file_s *)dq_next((FAR dq_entry_t *)fobj))
    {
      fobj->cache_page = 0;
    }

  spiffs_cache_page_free(fs, cp->ix, 0);
  cp->objid = 0;
}

/* Initializes the cache */

void spiffs_cache_init(FAR struct spiffs_s *fs)
{
  uint32_t sz = fs->cache_size;
  uint32_t cache_mask = 0;
  int i;
  int cache_entries =
    (sz - sizeof(struct spiffs_cache_s )) / (SPIFFS_CACHE_PAGE_SIZE(fs));

  if (fs->cache == 0)
    {
      return;
    }

  if (cache_entries <= 0)
    {
      return;
    }

  for (i = 0; i < cache_entries; i++)
    {
      cache_mask <<= 1;
      cache_mask |= 1;
    }

  struct spiffs_cache_s cache;
  memset(&cache, 0, sizeof(struct spiffs_cache_s ));
  cache.cpage_count = cache_entries;
  cache.cpages = (uint8_t *) ((uint8_t *) fs->cache + sizeof(struct spiffs_cache_s ));

  cache.cpage_use_map = 0xffffffff;
  cache.cpage_use_mask = cache_mask;
  memcpy(fs->cache, &cache, sizeof(struct spiffs_cache_s ));

  FAR struct spiffs_cache_s *c = spiffs_get_cache(fs);

  memset(c->cpages, 0, c->cpage_count * SPIFFS_CACHE_PAGE_SIZE(fs));

  c->cpage_use_map &= ~(c->cpage_use_mask);
  for (i = 0; i < cache.cpage_count; i++)
    {
      spiffs_get_cache_page_hdr(fs, c, i)->ix = i;
    }
}
