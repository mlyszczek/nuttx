/****************************************************************************
 * fs/spiffs.h/spiffs_nucleus.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of version 9.3.7 of SPIFFS by Peter Andersion.  That
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

#include "spiffs.h"
#include "spiffs_nucleus.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  spiffs_fd *fd;
  uint32_t remaining_objix_pages_to_visit;
  spiffs_span_ix map_objix_start_spix;
  spiffs_span_ix map_objix_end_spix;
} spiffs_ix_map_populate_state;

typedef struct
{
  spiffs_obj_id min_obj_id;
  spiffs_obj_id max_obj_id;
  uint32_t compaction;
  const uint8_t *conflicting_name;
} spiffs_free_obj_id_state;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t spiffs_page_data_check(spiffs * fs, spiffs_fd * fd,
                                      spiffs_page_ix pix, spiffs_span_ix spix)
{
  int32_t res = OK;

  if (pix == (spiffs_page_ix) - 1)
    {
      /* referring to page 0xffff...., bad object index */

      return SPIFFS_ERR_INDEX_REF_FREE;
    }

  if (pix % SPIFFS_PAGES_PER_BLOCK(fs) < SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
      /* referring to an object lookup page, bad object index */

      return SPIFFS_ERR_INDEX_REF_LU;
    }

  if (pix > SPIFFS_MAX_PAGES(fs))
    {
      /* referring to a bad page */

      return SPIFFS_ERR_INDEX_REF_INVALID;
    }

#if SPIFFS_PAGE_CHECK
  spiffs_page_header ph;
  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                   fd->file_nbr,
                   SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(spiffs_page_header), (uint8_t *) & ph);
  SPIFFS_CHECK_RES(res);
  SPIFFS_VALIDATE_DATA(ph, fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG, spix);
#endif
  return res;
}

static int32_t spiffs_page_index_check(spiffs * fs, spiffs_fd * fd,
                                       spiffs_page_ix pix, spiffs_span_ix spix)
{
  int32_t res = OK;
  if (pix == (spiffs_page_ix) - 1)
    {
      /* referring to page 0xffff...., bad object index */

      return SPIFFS_ERR_INDEX_FREE;
    }

  if (pix % SPIFFS_PAGES_PER_BLOCK(fs) < SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
      /* referring to an object lookup page, bad object index */

      return SPIFFS_ERR_INDEX_LU;
    }

  if (pix > SPIFFS_MAX_PAGES(fs))
    {
      /* referring to a bad page */

      return SPIFFS_ERR_INDEX_INVALID;
    }

#if SPIFFS_PAGE_CHECK
  spiffs_page_header ph;
  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                   fd->file_nbr,
                   SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(spiffs_page_header), (uint8_t *) & ph);
  SPIFFS_CHECK_RES(res);
  SPIFFS_VALIDATE_OBJIX(ph, fd->obj_id, spix);
#endif

  return res;
}

int32_t spiffs_phys_cpy(spiffs * fs,
                      spiffs_file fh, uint32_t dst, uint32_t src, uint32_t len)
{
  int32_t res;
  uint8_t b[SPIFFS_COPY_BUFFER_STACK];

  while (len > 0)
    {
      uint32_t chunk_size = MIN(SPIFFS_COPY_BUFFER_STACK, len);
      res =
        _spiffs_rd(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_MOVS, fh, src,
                   chunk_size, b);
      SPIFFS_CHECK_RES(res);
      res =
        _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_MOVD, fh, dst,
                   chunk_size, b);
      SPIFFS_CHECK_RES(res);
      len -= chunk_size;
      src += chunk_size;
      dst += chunk_size;
    }

  return OK;
}

/* Find object lookup entry containing given id with visitor.
 * Iterate over object lookup pages in each block until a given object id entry is found.
 * When found, the visitor function is called with block index, entry index and user data.
 * If visitor returns SPIFFS_VIS_CONTINUE, the search goes on. Otherwise, the search will be
 * ended and visitor's return code is returned to caller.
 * If no visitor is given (0) the search returns on first entry with matching object id.
 * If no match is found in all look up, SPIFFS_VIS_END is returned.
 * @param fs                    the file system
 * @param starting_block        the starting block to start search in
 * @param starting_lu_entry     the look up index entry to start search in
 * @param flags                 ored combination of SPIFFS_VIS_CHECK_ID, SPIFFS_VIS_CHECK_PH,
 *                              SPIFFS_VIS_NO_WRAP
 * @param obj_id                argument object id
 * @param v                     visitor callback function
 * @param user_const_p          any const pointer, passed to the callback visitor function
 * @param user_var_p            any pointer, passed to the callback visitor function
 * @param block_ix              reported block index where match was found
 * @param lu_entry              reported look up index where match was found
 */

int32_t spiffs_obj_lu_find_entry_visitor(spiffs * fs,
                                         spiffs_block_ix starting_block,
                                         int starting_lu_entry,
                                         uint8_t flags,
                                         spiffs_obj_id obj_id,
                                         spiffs_visitor_f v,
                                         const void *user_const_p,
                                         void *user_var_p,
                                         spiffs_block_ix * block_ix,
                                         int *lu_entry)
{
  int32_t res = OK;
  int32_t entry_count = fs->block_count * SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs);
  spiffs_block_ix cur_block = starting_block;
  uint32_t cur_block_addr = starting_block * SPIFFS_CFG_LOG_BLOCK_SZ(fs);
  spiffs_obj_id *obj_lu_buf = (spiffs_obj_id *) fs->lu_work;
  int cur_entry = starting_lu_entry;
  int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(spiffs_obj_id));

  /* wrap initial */

  if (cur_entry > (int)SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) - 1)
    {
      cur_entry = 0;
      cur_block++;
      cur_block_addr = cur_block * SPIFFS_CFG_LOG_BLOCK_SZ(fs);
      if (cur_block >= fs->block_count)
        {
          if (flags & SPIFFS_VIS_NO_WRAP)
            {
              return SPIFFS_VIS_END;
            }
          else
            {
              /* block wrap */

              cur_block = 0;
              cur_block_addr = 0;
            }
        }
    }

  /* check each block */

  while (res == OK && entry_count > 0)
    {
      int obj_lookup_page = cur_entry / entries_per_page;

      /* check each object lookup page */

      while (res == OK &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;
          res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                           0, cur_block_addr + SPIFFS_PAGE_TO_PADDR(fs,
                                                                    obj_lookup_page),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);

          /* check each entry */

          while (res == OK && cur_entry - entry_offset < entries_per_page &&     /* for 
                                                                                  * non-last 
                                                                                  * obj 
                                                                                  * lookup 
                                                                                  * pages */
                 cur_entry < (int)SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs))    /* for
                                                                         * last 
                                                                         * obj
                                                                         * lookup 
                                                                         * page */
            {
              if ((flags & SPIFFS_VIS_CHECK_ID) == 0 ||
                  obj_lu_buf[cur_entry - entry_offset] == obj_id)
                {
                  if (block_ix)
                    {
                      *block_ix = cur_block;
                    }

                  if (lu_entry)
                    {
                      *lu_entry = cur_entry;
                    }

                  if (v)
                    {
                      res = v(fs,
                              (flags & SPIFFS_VIS_CHECK_PH) ? obj_id :
                              obj_lu_buf[cur_entry - entry_offset], cur_block,
                              cur_entry, user_const_p, user_var_p);
                      if (res == SPIFFS_VIS_COUNTINUE ||
                          res == SPIFFS_VIS_COUNTINUE_RELOAD)
                        {
                          if (res == SPIFFS_VIS_COUNTINUE_RELOAD)
                            {
                              res =
                                _spiffs_rd(fs,
                                           SPIFFS_OP_T_OBJ_LU |
                                           SPIFFS_OP_C_READ, 0,
                                           cur_block_addr +
                                           SPIFFS_PAGE_TO_PADDR(fs,
                                                                obj_lookup_page),
                                           SPIFFS_CFG_LOG_PAGE_SZ(fs),
                                           fs->lu_work);
                              SPIFFS_CHECK_RES(res);
                            }

                          res = OK;
                          cur_entry++;
                          entry_count--;
                          continue;
                        }
                      else
                        {
                          return res;
                        }
                    }
                  else
                    {
                      return OK;
                    }
                }

              entry_count--;
              cur_entry++;
            }

          obj_lookup_page++;
        }

      cur_entry = 0;
      cur_block++;
      cur_block_addr += SPIFFS_CFG_LOG_BLOCK_SZ(fs);

      if (cur_block >= fs->block_count)
        {
          if (flags & SPIFFS_VIS_NO_WRAP)
            {
              return SPIFFS_VIS_END;
            }
          else
            {
              /* block wrap */

              cur_block = 0;
              cur_block_addr = 0;
            }
        }
    }

  SPIFFS_CHECK_RES(res);
  return SPIFFS_VIS_END;
}

int32_t spiffs_erase_block(spiffs * fs, spiffs_block_ix bix)
{
  int32_t res;
  uint32_t addr = SPIFFS_BLOCK_TO_PADDR(fs, bix);
  int32_t size = SPIFFS_CFG_LOG_BLOCK_SZ(fs);

  /* here we ignore res, just try erasing the block */

  while (size > 0)
    {
      finfo("erase " _SPIPRIad ":" _SPIPRIi "\n", addr,
                 SPIFFS_CFG_PHYS_ERASE_SZ(fs));
      SPIFFS_HAL_ERASE(fs, addr, SPIFFS_CFG_PHYS_ERASE_SZ(fs));

      addr += SPIFFS_CFG_PHYS_ERASE_SZ(fs);
      size -= SPIFFS_CFG_PHYS_ERASE_SZ(fs);
    }

  fs->free_blocks++;

  /* register erase count for this block */

  res = _spiffs_wr(fs, SPIFFS_OP_C_WRTHRU | SPIFFS_OP_T_OBJ_LU2, 0,
                   SPIFFS_ERASE_COUNT_PADDR(fs, bix),
                   sizeof(spiffs_obj_id), (uint8_t *) & fs->max_erase_count);
  SPIFFS_CHECK_RES(res);

#if SPIFFS_USE_MAGIC
  /* finally, write magic */

  spiffs_obj_id magic = SPIFFS_MAGIC(fs, bix);
  res = _spiffs_wr(fs, SPIFFS_OP_C_WRTHRU | SPIFFS_OP_T_OBJ_LU2, 0,
                   SPIFFS_MAGIC_PADDR(fs, bix),
                   sizeof(spiffs_obj_id), (uint8_t *) & magic);
  SPIFFS_CHECK_RES(res);
#endif

  fs->max_erase_count++;
  if (fs->max_erase_count == SPIFFS_OBJ_ID_IX_FLAG)
    {
      fs->max_erase_count = 0;
    }

  return res;
}

#if SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH
int32_t spiffs_probe(spiffs_config * cfg)
{
  int32_t res;
  uint32_t paddr;
  spiffs dummy_fs;              /* create a dummy fs struct just to be able to
                                 * use macros */
  memcpy(&dummy_fs.cfg, cfg, sizeof(spiffs_config));
  dummy_fs.block_count = 0;

  /* Read three magics, as one block may be in an aborted erase state.
   * At least two of these must contain magic and be in decreasing order.
   */

  spiffs_obj_id magic[3];
  spiffs_obj_id bix_count[3];

  spiffs_block_ix bix;
  for (bix = 0; bix < 3; bix++)
    {
      paddr = SPIFFS_MAGIC_PADDR(&dummy_fs, bix);

#if SPIFFS_HAL_CALLBACK_EXTRA
      /* not any proper fs to report here, so callback with null
       * (cross fingers that no-one gets angry)
       */

      res =
        cfg->hal_read_f((void *)0, paddr, sizeof(spiffs_obj_id),
                        (uint8_t *) & magic[bix]);
#else
      res =
        cfg->hal_read_f(paddr, sizeof(spiffs_obj_id), (uint8_t *) & magic[bix]);
#endif
      bix_count[bix] = magic[bix] ^ SPIFFS_MAGIC(&dummy_fs, 0);
      SPIFFS_CHECK_RES(res);
    }

  /* check that we have sane number of blocks */

  if (bix_count[0] < 3)
    {
      return SPIFFS_ERR_PROBE_TOO_FEW_BLOCKS;
    }

  /* check that the order is correct, take aborted erases in calculation
   * first block aborted erase
   */

  if (magic[0] == (spiffs_obj_id) (-1) && bix_count[1] - bix_count[2] == 1)
    {
      return (bix_count[1] + 1) * cfg->log_block_size;
    }

  /* second block aborted erase */

  if (magic[1] == (spiffs_obj_id) (-1) && bix_count[0] - bix_count[2] == 2)
    {
      return bix_count[0] * cfg->log_block_size;
    }

  /* third block aborted erase */

  if (magic[2] == (spiffs_obj_id) (-1) && bix_count[0] - bix_count[1] == 1)
    {
      return bix_count[0] * cfg->log_block_size;
    }

  /* no block has aborted erase */

  if (bix_count[0] - bix_count[1] == 1 && bix_count[1] - bix_count[2] == 1)
    {
      return bix_count[0] * cfg->log_block_size;
    }

  return SPIFFS_ERR_PROBE_NOT_A_FS;
}
#endif /* SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH */

static int32_t spiffs_obj_lu_scan_v(spiffs * fs,
                                    spiffs_obj_id obj_id,
                                    spiffs_block_ix bix,
                                    int ix_entry,
                                    const void *user_const_p, void *user_var_p)
{
  if (obj_id == SPIFFS_OBJ_ID_FREE)
    {
      if (ix_entry == 0)
        {
          /* todo optimize further, return SPIFFS_NEXT_BLOCK */

          fs->free_blocks++;
        }
    }
  else if (obj_id == SPIFFS_OBJ_ID_DELETED)
    {
      fs->stats_p_deleted++;
    }
  else
    {
      fs->stats_p_allocated++;
    }

  return SPIFFS_VIS_COUNTINUE;
}

/* Scans thru all obj lu and counts free, deleted and used pages
 * Find the maximum block erase count
 * Checks magic if enabled
 */

int32_t spiffs_obj_lu_scan(spiffs * fs)
{
  int32_t res;
  spiffs_block_ix bix;
  int entry;
#if SPIFFS_USE_MAGIC
  spiffs_block_ix unerased_bix = (spiffs_block_ix) - 1;
#endif

  /* find out erase count.  if enabled, check magic */

  bix = 0;
  spiffs_obj_id erase_count_final;
  spiffs_obj_id erase_count_min = SPIFFS_OBJ_ID_FREE;
  spiffs_obj_id erase_count_max = 0;

  while (bix < fs->block_count)
    {
#if SPIFFS_USE_MAGIC
      spiffs_obj_id magic;
      res = _spiffs_rd(fs,
                       SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                       0, SPIFFS_MAGIC_PADDR(fs, bix),
                       sizeof(spiffs_obj_id), (uint8_t *) & magic);

      SPIFFS_CHECK_RES(res);
      if (magic != SPIFFS_MAGIC(fs, bix))
        {
          if (unerased_bix == (spiffs_block_ix) - 1)
            {
              /* allow one unerased block as it might be powered down during an 
               * erase
               */

              unerased_bix = bix;
            }
          else
            {
              /* more than one unerased block, bail out */

              SPIFFS_CHECK_RES(SPIFFS_ERR_NOT_A_FS);
            }
        }
#endif
      spiffs_obj_id erase_count;
      res = _spiffs_rd(fs,
                       SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                       0, SPIFFS_ERASE_COUNT_PADDR(fs, bix),
                       sizeof(spiffs_obj_id), (uint8_t *) & erase_count);
      SPIFFS_CHECK_RES(res);
      if (erase_count != SPIFFS_OBJ_ID_FREE)
        {
          erase_count_min = MIN(erase_count_min, erase_count);
          erase_count_max = MAX(erase_count_max, erase_count);
        }

      bix++;
    }

  if (erase_count_min == 0 && erase_count_max == SPIFFS_OBJ_ID_FREE)
    {
      /* clean system, set counter to zero */

      erase_count_final = 0;
    }
  else if (erase_count_max - erase_count_min > (SPIFFS_OBJ_ID_FREE) / 2)
    {
      /* wrap, take min */

      erase_count_final = erase_count_min + 1;
    }
  else
    {
      erase_count_final = erase_count_max + 1;
    }

  fs->max_erase_count = erase_count_final;

#if SPIFFS_USE_MAGIC
  if (unerased_bix != (spiffs_block_ix) - 1)
    {
      /* found one unerased block, remedy */

      finfo("mount: erase block " _SPIPRIbl "\n", bix);
      res = spiffs_erase_block(fs, unerased_bix);
      SPIFFS_CHECK_RES(res);
    }
#endif

  /* count blocks */

  fs->free_blocks = 0;
  fs->stats_p_allocated = 0;
  fs->stats_p_deleted = 0;

  res = spiffs_obj_lu_find_entry_visitor(fs,
                                         0,
                                         0,
                                         0,
                                         0,
                                         spiffs_obj_lu_scan_v,
                                         0, 0, &bix, &entry);

  if (res == SPIFFS_VIS_END)
    {
      res = OK;
    }

  SPIFFS_CHECK_RES(res);
  return res;
}

/* Find free object lookup entry.  Iterate over object lookup pages in each
 * block until a free object id entry is found
 */

int32_t spiffs_obj_lu_find_free(spiffs * fs,
                                spiffs_block_ix starting_block,
                                int starting_lu_entry,
                                spiffs_block_ix * block_ix, int *lu_entry)
{
  int32_t res;

  if (!fs->cleaning && fs->free_blocks < 2)
    {
      res = spiffs_gc_quick(fs, 0);
      if (res == SPIFFS_ERR_NO_DELETED_BLOCKS)
        {
          res = OK;
        }

      SPIFFS_CHECK_RES(res);

      if (fs->free_blocks < 2)
        {
          return SPIFFS_ERR_FULL;
        }
    }

  res = spiffs_obj_lu_find_id(fs, starting_block, starting_lu_entry,
                              SPIFFS_OBJ_ID_FREE, block_ix, lu_entry);
  if (res == OK)
    {
      fs->free_cursor_block_ix = *block_ix;
      fs->free_cursor_obj_lu_entry = (*lu_entry) + 1;
      if (*lu_entry == 0)
        {
          fs->free_blocks--;
        }
    }

  if (res == SPIFFS_ERR_FULL)
    {
      finfo("fs full\n");
    }

  return res;
}

/* Find object lookup entry containing given id.  Iterate over object lookup
 * pages in each block until a given object id entry is found
 */

int32_t spiffs_obj_lu_find_id(spiffs * fs,
                              spiffs_block_ix starting_block,
                              int starting_lu_entry,
                              spiffs_obj_id obj_id,
                              spiffs_block_ix * block_ix, int *lu_entry)
{
  int32_t res =
    spiffs_obj_lu_find_entry_visitor(fs, starting_block, starting_lu_entry,
                                     SPIFFS_VIS_CHECK_ID, obj_id, 0, 0, 0,
                                     block_ix, lu_entry);
  if (res == SPIFFS_VIS_END)
    {
      res = SPIFFS_ERR_NOT_FOUND;
    }

  return res;
}

static int32_t spiffs_obj_lu_find_id_and_span_v(spiffs * fs,
                                                spiffs_obj_id obj_id,
                                                spiffs_block_ix bix,
                                                int ix_entry,
                                                const void *user_const_p,
                                                void *user_var_p)
{
  int32_t res;
  spiffs_page_header ph;
  spiffs_page_ix pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, ix_entry);
  res = _spiffs_rd(fs, 0, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   SPIFFS_PAGE_TO_PADDR(fs, pix), sizeof(spiffs_page_header),
                   (uint8_t *) & ph);

  SPIFFS_CHECK_RES(res);

  if (ph.obj_id == obj_id &&
      ph.span_ix == *((spiffs_span_ix *) user_var_p) &&
      (ph.
       flags & (SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_DELET |
                SPIFFS_PH_FLAG_USED)) == SPIFFS_PH_FLAG_DELET &&
      !((obj_id & SPIFFS_OBJ_ID_IX_FLAG) &&
        (ph.flags & SPIFFS_PH_FLAG_IXDELE) == 0 && ph.span_ix == 0) &&
      (user_const_p == 0 || *((const spiffs_page_ix *)user_const_p) != pix))
    {
      return OK;
    }
  else
    {
      return SPIFFS_VIS_COUNTINUE;
    }
}

/* Find object lookup entry containing given id and span index.  Iterate over
 * object lookup pages in each block until a given object id entry is found
 */

int32_t spiffs_obj_lu_find_id_and_span(spiffs * fs,
                                       spiffs_obj_id obj_id,
                                       spiffs_span_ix spix,
                                       spiffs_page_ix exclusion_pix,
                                       spiffs_page_ix * pix)
{
  int32_t res;
  spiffs_block_ix bix;
  int entry;

  res = spiffs_obj_lu_find_entry_visitor(fs,
                                         fs->cursor_block_ix,
                                         fs->cursor_obj_lu_entry,
                                         SPIFFS_VIS_CHECK_ID,
                                         obj_id,
                                         spiffs_obj_lu_find_id_and_span_v,
                                         exclusion_pix ? &exclusion_pix : 0,
                                         &spix, &bix, &entry);

  if (res == SPIFFS_VIS_END)
    {
      res = SPIFFS_ERR_NOT_FOUND;
    }

  SPIFFS_CHECK_RES(res);

  if (pix)
    {
      *pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry);
    }

  fs->cursor_block_ix = bix;
  fs->cursor_obj_lu_entry = entry;

  return res;
}

/* Find object lookup entry containing given id and span index in page headers only
 * Iterate over object lookup pages in each block until a given object id entry is found
 */

int32_t spiffs_obj_lu_find_id_and_span_by_phdr(spiffs * fs,
                                             spiffs_obj_id obj_id,
                                             spiffs_span_ix spix,
                                             spiffs_page_ix exclusion_pix,
                                             spiffs_page_ix * pix)
{
  int32_t res;
  spiffs_block_ix bix;
  int entry;

  res = spiffs_obj_lu_find_entry_visitor(fs,
                                         fs->cursor_block_ix,
                                         fs->cursor_obj_lu_entry,
                                         SPIFFS_VIS_CHECK_PH,
                                         obj_id,
                                         spiffs_obj_lu_find_id_and_span_v,
                                         exclusion_pix ? &exclusion_pix : 0,
                                         &spix, &bix, &entry);

  if (res == SPIFFS_VIS_END)
    {
      res = SPIFFS_ERR_NOT_FOUND;
    }

  SPIFFS_CHECK_RES(res);

  if (pix)
    {
      *pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry);
    }

  fs->cursor_block_ix = bix;
  fs->cursor_obj_lu_entry = entry;
  return res;
}

#if SPIFFS_IX_MAP
/* update index map of given fd with given object index data */

static void spiffs_update_ix_map(spiffs * fs,
                                 spiffs_fd * fd, spiffs_span_ix objix_spix,
                                 spiffs_page_object_ix * objix)
{
  spiffs_ix_map *map = fd->ix_map;
  spiffs_span_ix map_objix_start_spix =
    SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, map->start_spix);
  spiffs_span_ix map_objix_end_spix =
    SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, map->end_spix);

  /* check if updated ix is within map range */

  if (objix_spix < map_objix_start_spix || objix_spix > map_objix_end_spix)
    {
      return;
    }

  /* update memory mapped page index buffer to new pages
   * get range of updated object index map data span indices
   */

  spiffs_span_ix objix_data_spix_start =
    SPIFFS_DATA_SPAN_IX_FOR_OBJ_IX_SPAN_IX(fs, objix_spix);
  spiffs_span_ix objix_data_spix_end = objix_data_spix_start +
    (objix_spix == 0 ? SPIFFS_OBJ_HDR_IX_LEN(fs) : SPIFFS_OBJ_IX_LEN(fs));

  /* calc union of object index range and index map range array */

  spiffs_span_ix map_spix = MAX(map->start_spix, objix_data_spix_start);
  spiffs_span_ix map_spix_end = MIN(map->end_spix + 1, objix_data_spix_end);

  while (map_spix < map_spix_end)
    {
      spiffs_page_ix objix_data_pix;
      if (objix_spix == 0)
        {
          /* get data page from object index header page */

          objix_data_pix =
            ((spiffs_page_ix *) ((uint8_t *) objix +
                                 sizeof(spiffs_page_object_ix_header)))
            [map_spix];
        }
      else
        {
          /* get data page from object index page */

          objix_data_pix =
            ((spiffs_page_ix *) ((uint8_t *) objix +
                                 sizeof(spiffs_page_object_ix)))
            [SPIFFS_OBJ_IX_ENTRY(fs, map_spix)];
        }

      if (objix_data_pix == (spiffs_page_ix) - 1)
        {
          /* reached end of object, abort */

          break;
        }

      map->map_buf[map_spix - map->start_spix] = objix_data_pix;
      finfo("map " _SPIPRIid ":" _SPIPRIsp " (" _SPIPRIsp "--" _SPIPRIsp
                 ") objix.spix:" _SPIPRIsp " to pix " _SPIPRIpg "\n",
                 fd->obj_id, map_spix - map->start_spix, map->start_spix,
                 map->end_spix, objix->p_hdr.span_ix, objix_data_pix);

      map_spix++;
    }
}

static int32_t spiffs_populate_ix_map_v(spiffs * fs,
                                        spiffs_obj_id obj_id,
                                        spiffs_block_ix bix,
                                        int ix_entry,
                                        const void *user_const_p,
                                        void *user_var_p)
{
  int32_t res;
  spiffs_ix_map_populate_state *state =
    (spiffs_ix_map_populate_state *) user_var_p;
  spiffs_page_ix pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, ix_entry);

  /* load header to check it */

  spiffs_page_object_ix *objix = (spiffs_page_object_ix *) fs->work;
  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   0, SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(spiffs_page_object_ix), (uint8_t *) objix);
  SPIFFS_CHECK_RES(res);
  SPIFFS_VALIDATE_OBJIX(objix->p_hdr, obj_id, objix->p_hdr.span_ix);

  /* check if hdr is ok, and if objix range overlap with ix map range */

  if ((objix->p_hdr.
       flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL |
                SPIFFS_PH_FLAG_IXDELE)) ==
      (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE) &&
      objix->p_hdr.span_ix >= state->map_objix_start_spix &&
      objix->p_hdr.span_ix <= state->map_objix_end_spix)
    {
      /* ok, load rest of object index */

      res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                       0, SPIFFS_PAGE_TO_PADDR(fs,
                                               pix) +
                       sizeof(spiffs_page_object_ix),
                       SPIFFS_CFG_LOG_PAGE_SZ(fs) -
                       sizeof(spiffs_page_object_ix),
                       (uint8_t *) objix + sizeof(spiffs_page_object_ix));
      SPIFFS_CHECK_RES(res);

      spiffs_update_ix_map(fs, state->fd, objix->p_hdr.span_ix, objix);

      state->remaining_objix_pages_to_visit--;
      finfo("map " _SPIPRIid " (" _SPIPRIsp "--" _SPIPRIsp
                 ") remaining objix pages " _SPIPRIi "\n", state->fd->obj_id,
                 state->fd->ix_map->start_spix, state->fd->ix_map->end_spix,
                 state->remaining_objix_pages_to_visit);
    }

  if (res == OK)
    {
      res =
        state->
        remaining_objix_pages_to_visit ? SPIFFS_VIS_COUNTINUE : SPIFFS_VIS_END;
    }

  return res;
}

/* populates index map, from vector entry start to vector entry end, inclusive */

int32_t spiffs_populate_ix_map(spiffs * fs, spiffs_fd * fd, uint32_t vec_entry_start,
                             uint32_t vec_entry_end)
{
  int32_t res;
  spiffs_ix_map *map = fd->ix_map;
  spiffs_ix_map_populate_state state;
  vec_entry_start =
    MIN((uint32_t) (map->end_spix - map->start_spix), vec_entry_start);
  vec_entry_end = MAX((uint32_t) (map->end_spix - map->start_spix), vec_entry_end);

  if (vec_entry_start > vec_entry_end)
    {
      return SPIFFS_ERR_IX_MAP_BAD_RANGE;
    }

  state.map_objix_start_spix =
    SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, map->start_spix + vec_entry_start);
  state.map_objix_end_spix =
    SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, map->start_spix + vec_entry_end);
  state.remaining_objix_pages_to_visit =
    state.map_objix_end_spix - state.map_objix_start_spix + 1;
  state.fd = fd;

  res = spiffs_obj_lu_find_entry_visitor(fs,
                                         SPIFFS_BLOCK_FOR_PAGE(fs, fd->objix_hdr_pix),
                                         SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, fd->objix_hdr_pix),
                                         SPIFFS_VIS_CHECK_ID,
                                         fd->obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                         spiffs_populate_ix_map_v, 0, &state, 0,
                                         0);

  if (res == SPIFFS_VIS_END)
    {
      res = OK;
    }

  return res;
}

#endif

/* Allocates a free defined page with given obj_id
 * Occupies object lookup entry and page
 * data may be NULL; where only page header is stored, len and page_offs is ignored
 */

int32_t spiffs_page_allocate_data(spiffs * fs,
                                spiffs_obj_id obj_id,
                                spiffs_page_header * ph,
                                uint8_t * data,
                                uint32_t len,
                                uint32_t page_offs,
                                uint8_t finalize, spiffs_page_ix * pix)
{
  int32_t res = OK;
  spiffs_block_ix bix;
  int entry;

  /* find free entry */

  res =
    spiffs_obj_lu_find_free(fs, fs->free_cursor_block_ix,
                            fs->free_cursor_obj_lu_entry, &bix, &entry);
  SPIFFS_CHECK_RES(res);

  /* occupy page in object lookup */

  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT,
                   0, SPIFFS_BLOCK_TO_PADDR(fs, bix) +
                   entry * sizeof(spiffs_obj_id), sizeof(spiffs_obj_id),
                   (uint8_t *) & obj_id);
  SPIFFS_CHECK_RES(res);

  fs->stats_p_allocated++;

  /* write page header */

  ph->flags &= ~SPIFFS_PH_FLAG_USED;
  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                   0, SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, bix, entry),
                   sizeof(spiffs_page_header), (uint8_t *) ph);
  SPIFFS_CHECK_RES(res);

  /* write page data */

  if (data)
    {
      res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                       0, SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, bix,
                                                           entry) +
                       sizeof(spiffs_page_header) + page_offs, len, data);
      SPIFFS_CHECK_RES(res);
    }

  /* finalize header if necessary */

  if (finalize && (ph->flags & SPIFFS_PH_FLAG_FINAL))
    {
      ph->flags &= ~SPIFFS_PH_FLAG_FINAL;
      res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                       0, SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, bix,
                                                           entry) +
                       offsetof(spiffs_page_header, flags), sizeof(uint8_t),
                       (uint8_t *) & ph->flags);
      SPIFFS_CHECK_RES(res);
    }

  /* return written page */

  if (pix)
    {
      *pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry);
    }

  return res;
}

/* Moves a page from src to a free page and finalizes it. Updates page index.
 * Page data is given in param page. If page data is null, provided header
 * is used for meta-info and page data is physically copied.
 */

int32_t spiffs_page_move(spiffs * fs,
                         spiffs_file fh,
                         uint8_t * page_data,
                         spiffs_obj_id obj_id,
                         spiffs_page_header * page_hdr,
                         spiffs_page_ix src_pix, spiffs_page_ix * dst_pix)
{
  int32_t res;
  uint8_t was_final = 0;
  spiffs_page_header *p_hdr;
  spiffs_block_ix bix;
  int entry;
  spiffs_page_ix free_pix;

  /* find free entry */

  res =
    spiffs_obj_lu_find_free(fs, fs->free_cursor_block_ix,
                            fs->free_cursor_obj_lu_entry, &bix, &entry);
  SPIFFS_CHECK_RES(res);
  free_pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry);

  if (dst_pix)
    *dst_pix = free_pix;

  p_hdr = page_data ? (spiffs_page_header *) page_data : page_hdr;
  if (page_data)
    {
      /* got page data */

      was_final = (p_hdr->flags & SPIFFS_PH_FLAG_FINAL) == 0;

      /* write unfinalized page */

      p_hdr->flags |= SPIFFS_PH_FLAG_FINAL;
      p_hdr->flags &= ~SPIFFS_PH_FLAG_USED;
      res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                       0, SPIFFS_PAGE_TO_PADDR(fs, free_pix),
                       SPIFFS_CFG_LOG_PAGE_SZ(fs), page_data);
    }
  else
    {
      /* copy page data */

      res =
        spiffs_phys_cpy(fs, fh, SPIFFS_PAGE_TO_PADDR(fs, free_pix),
                        SPIFFS_PAGE_TO_PADDR(fs, src_pix),
                        SPIFFS_CFG_LOG_PAGE_SZ(fs));
    }

  SPIFFS_CHECK_RES(res);

  /* mark entry in destination object lookup */

  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT, 0,
                   SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, free_pix)) +
                   SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, free_pix) *
                   sizeof(spiffs_page_ix), sizeof(spiffs_obj_id),
                   (uint8_t *) & obj_id);
  SPIFFS_CHECK_RES(res);

  fs->stats_p_allocated++;

  if (was_final)
    {
      /* mark finalized in destination page */

      p_hdr->flags &= ~(SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_USED);
      res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, fh,
                       SPIFFS_PAGE_TO_PADDR(fs, free_pix) +
                       offsetof(spiffs_page_header, flags), sizeof(uint8_t),
                       (uint8_t *) & p_hdr->flags);
      SPIFFS_CHECK_RES(res);
    }

  /* mark source deleted */

  res = spiffs_page_delete(fs, src_pix);
  return res;
}

/* Deletes a page and removes it from object lookup. */

int32_t spiffs_page_delete(spiffs * fs, spiffs_page_ix pix)
{
  int32_t res;

  /* mark deleted entry in source object lookup */

  spiffs_obj_id d_obj_id = SPIFFS_OBJ_ID_DELETED;
  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_DELE, 0,
                   SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, pix)) +
                   SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pix) *
                   sizeof(spiffs_page_ix), sizeof(spiffs_obj_id),
                   (uint8_t *) & d_obj_id);
  SPIFFS_CHECK_RES(res);

  fs->stats_p_deleted++;
  fs->stats_p_allocated--;

  /* mark deleted in source page */

  uint8_t flags = 0xff;
#if SPIFFS_NO_BLIND_WRITES
  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                   0, SPIFFS_PAGE_TO_PADDR(fs,
                                           pix) + offsetof(spiffs_page_header,
                                                           flags),
                   sizeof(flags), &flags);
  SPIFFS_CHECK_RES(res);
#endif
  flags &= ~(SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_USED);
  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_DELE, 0,
                   SPIFFS_PAGE_TO_PADDR(fs, pix) + offsetof(spiffs_page_header, flags),
                   sizeof(flags), &flags);

  return res;
}

/* Create an object index header page with empty index and undefined length */

int32_t spiffs_object_create(spiffs * fs,
                             spiffs_obj_id obj_id,
                             const uint8_t name[],
                             const uint8_t meta[],
                             spiffs_obj_type type, spiffs_page_ix * objix_hdr_pix)
{
  int32_t res = OK;
  spiffs_block_ix bix;
  spiffs_page_object_ix_header oix_hdr;
  int entry;

  res = spiffs_gc_check(fs, SPIFFS_DATA_PAGE_SIZE(fs));
  SPIFFS_CHECK_RES(res);

  obj_id |= SPIFFS_OBJ_ID_IX_FLAG;

  /* find free entry */

  res =
    spiffs_obj_lu_find_free(fs, fs->free_cursor_block_ix,
                            fs->free_cursor_obj_lu_entry, &bix, &entry);
  SPIFFS_CHECK_RES(res);
  finfo("create: found free page @ " _SPIPRIpg " bix:" _SPIPRIbl " entry:"
             _SPIPRIsp "\n", (spiffs_page_ix) SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs,
                                                                             bix,
                                                                             entry),
             bix, entry);

  /* occupy page in object lookup */

  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT,
                   0, SPIFFS_BLOCK_TO_PADDR(fs,
                                            bix) +
                   entry * sizeof(spiffs_obj_id), sizeof(spiffs_obj_id),
                   (uint8_t *) & obj_id);
  SPIFFS_CHECK_RES(res);

  fs->stats_p_allocated++;

  /* write empty object index page */

  oix_hdr.p_hdr.obj_id = obj_id;
  oix_hdr.p_hdr.span_ix = 0;
  oix_hdr.p_hdr.flags =
    0xff & ~(SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_USED);
  oix_hdr.type = type;
  oix_hdr.size = SPIFFS_UNDEFINED_LEN;  /* keep ones so we can update later
                                         * without wasting this page */
  strncpy((char *)oix_hdr.name, (const char *)name, SPIFFS_NAME_MAX);
#if SPIFFS_OBJ_META_LEN
  if (meta)
    {
      memcpy(oix_hdr.meta, meta, SPIFFS_OBJ_META_LEN);
    }
  else
    {
      memset(oix_hdr.meta, 0xff, SPIFFS_OBJ_META_LEN);
    }
#else
  UNUSEd(meta);
#endif

  /* update page */

  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                   0, SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, bix, entry),
                   sizeof(spiffs_page_object_ix_header), (uint8_t *) & oix_hdr);

  SPIFFS_CHECK_RES(res);
  spiffs_cb_object_event(fs, (spiffs_page_object_ix *) & oix_hdr,
                         SPIFFS_EV_IX_NEW, obj_id, 0,
                         SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry),
                         SPIFFS_UNDEFINED_LEN);

  if (objix_hdr_pix)
    {
      *objix_hdr_pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry);
    }

  return res;
}

/* update object index header with any combination of name/size/index
 * new_objix_hdr_data may be null, if so the object index header page is loaded
 * name may be null, if so name is not changed
 * size may be null, if so size is not changed
 */

int32_t spiffs_object_update_index_hdr(spiffs * fs,
                                       spiffs_fd * fd,
                                       spiffs_obj_id obj_id,
                                       spiffs_page_ix objix_hdr_pix,
                                       uint8_t * new_objix_hdr_data,
                                       const uint8_t name[],
                                       const uint8_t meta[],
                                       uint32_t size, spiffs_page_ix * new_pix)
{
  int32_t res = OK;
  spiffs_page_object_ix_header *objix_hdr;
  spiffs_page_ix new_objix_hdr_pix;

  obj_id |= SPIFFS_OBJ_ID_IX_FLAG;

  if (new_objix_hdr_data)
    {
      /* object index header page already given to us, no need to load it */

      objix_hdr = (spiffs_page_object_ix_header *) new_objix_hdr_data;
    }
  else
    {
      /* read object index header page */

      res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                       fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs, objix_hdr_pix),
                       SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
      SPIFFS_CHECK_RES(res);
      objix_hdr = (spiffs_page_object_ix_header *) fs->work;
    }

  SPIFFS_VALIDATE_OBJIX(objix_hdr->p_hdr, obj_id, 0);

  /* change name */

  if (name)
    {
      strncpy((char *)objix_hdr->name, (const char *)name, SPIFFS_NAME_MAX);
    }

#if SPIFFS_OBJ_META_LEN
  if (meta)
    {
      memcpy(objix_hdr->meta, meta, SPIFFS_OBJ_META_LEN);
    }
#else
  UNJUSED(meta);
#endif

  if (size)
    {
      objix_hdr->size = size;
    }

  /* move and update page */

  res =
    spiffs_page_move(fs, fd == 0 ? 0 : fd->file_nbr, (uint8_t *) objix_hdr, obj_id,
                     0, objix_hdr_pix, &new_objix_hdr_pix);

  if (res == OK)
    {
      if (new_pix)
        {
          *new_pix = new_objix_hdr_pix;
        }

      /* callback on object index update */

      spiffs_cb_object_event(fs, (spiffs_page_object_ix *) objix_hdr,
                             new_objix_hdr_data ? SPIFFS_EV_IX_UPD :
                             SPIFFS_EV_IX_UPD_HDR, obj_id,
                             objix_hdr->p_hdr.span_ix, new_objix_hdr_pix,
                             objix_hdr->size);
      if (fd)
        {
          fd->objix_hdr_pix = new_objix_hdr_pix;  /* if this is not in the
                                                   * registered cluster */
        }
    }

  return res;
}

void spiffs_cb_object_event(spiffs * fs,
                            spiffs_page_object_ix * objix,
                            int ev,
                            spiffs_obj_id obj_id_raw,
                            spiffs_span_ix spix,
                            spiffs_page_ix new_pix, uint32_t new_size)
{
  spiffs_obj_id obj_id = obj_id_raw & ~SPIFFS_OBJ_ID_IX_FLAG;
  uint32_t i;
  spiffs_fd *fds = (spiffs_fd *) fs->fd_space;

  finfo("       CALLBACK  %s obj_id:" _SPIPRIid " spix:" _SPIPRIsp " npix:"
             _SPIPRIpg " nsz:" _SPIPRIi "\n", (const char *[])
             {
             "UPD", "NEW", "DEL", "MOV", "HUP", "???"}[MIN(ev, 5)],
             obj_id_raw, spix, new_pix, new_size);

  /* update index caches in all file descriptors */

  for (i = 0; i < fs->fd_count; i++)
    {
      spiffs_fd *cur_fd = &fds[i];
      if ((cur_fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG) != obj_id)
        {
          continue;             /* fd not related to updated file */
        }

#if !SPIFFS_TEMPORAL_FD_CACHE
      if (cur_fd->file_nbr == 0)
        {
          continue;             /* fd closed */
        }
#endif

      if (spix == 0)
        {                       /* object index header update */
          if (ev != SPIFFS_EV_IX_DEL)
            {
#if SPIFFS_TEMPORAL_FD_CACHE
              if (cur_fd->score == 0)
                {
                  continue;     /* never used fd */
                }
#endif

              finfo("       callback: setting fd " _SPIPRIfd ":" _SPIPRIid
                         "(fdoffs:" _SPIPRIi " offs:" _SPIPRIi
                         ") objix_hdr_pix to " _SPIPRIpg ", size:" _SPIPRIi
                         "\n", cur_fd->file_nbr,
                         cur_fd->obj_id, cur_fd->fdoffset, cur_fd->offset,
                         new_pix, new_size);

              cur_fd->objix_hdr_pix = new_pix;
              if (new_size != 0)
                {
                  /* update size and offsets for fds to this file */

                  cur_fd->size = new_size;
                  uint32_t act_new_size =
                    new_size == SPIFFS_UNDEFINED_LEN ? 0 : new_size;
                  if (act_new_size > 0 && cur_fd->cache_page)
                    {
                      act_new_size =
                        MAX(act_new_size,
                            cur_fd->cache_page->offset +
                            cur_fd->cache_page->size);
                    }

                  if (cur_fd->offset > act_new_size)
                    {
                      cur_fd->offset = act_new_size;
                    }

                  if (cur_fd->fdoffset > act_new_size)
                    {
                      cur_fd->fdoffset = act_new_size;
                    }

                  if (cur_fd->cache_page &&
                      cur_fd->cache_page->offset > act_new_size + 1)
                    {
                      spiffs_cacheinfo
                        ("CACHE_DROP: file trunced, dropping cache page "
                         _SPIPRIi ", no writeback\n", cur_fd->cache_page->ix);
                      spiffs_cache_fd_release(fs, cur_fd->cache_page);
                    }
                }
            }
          else
            {
              /* removing file */

              if (cur_fd->file_nbr && cur_fd->cache_page)
                {
                  spiffs_cacheinfo
                    ("CACHE_DROP: file deleted, dropping cache page " _SPIPRIi
                     ", no writeback\n", cur_fd->cache_page->ix);
                  spiffs_cache_fd_release(fs, cur_fd->cache_page);
                }

              finfo("       callback: release fd " _SPIPRIfd ":" _SPIPRIid
                         " span:" _SPIPRIsp " objix_pix to " _SPIPRIpg "\n",
                         cur_fd->file_nbr, cur_fd->obj_id, spix, new_pix);

              cur_fd->file_nbr = 0;
              cur_fd->obj_id = SPIFFS_OBJ_ID_DELETED;
            }
        }

      if (cur_fd->cursor_objix_spix == spix)
        {
          if (ev != SPIFFS_EV_IX_DEL)
            {
              finfo("       callback: setting fd " _SPIPRIfd ":" _SPIPRIid
                         " span:" _SPIPRIsp " objix_pix to " _SPIPRIpg "\n",
                         cur_fd->file_nbr, cur_fd->obj_id, spix, new_pix);
              cur_fd->cursor_objix_pix = new_pix;
            }
          else
            {
              cur_fd->cursor_objix_pix = 0;
            }
        }
    }

#if SPIFFS_IX_MAP
  /* update index maps */

  if (ev == SPIFFS_EV_IX_UPD || ev == SPIFFS_EV_IX_NEW)
    {
      for (i = 0; i < fs->fd_count; i++)
        {
          spiffs_fd *cur_fd = &fds[i];

          /* check fd opened, having ix map, match obj id */
 
          if (cur_fd->file_nbr == 0 ||
              cur_fd->ix_map == 0 ||
              (cur_fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG) != obj_id)
            {
              continue;
            }

          finfo("       callback: map ix update fd " _SPIPRIfd ":"
                     _SPIPRIid " span:" _SPIPRIsp "\n", cur_fd->file_nbr,
                     cur_fd->obj_id, spix);
          spiffs_update_ix_map(fs, cur_fd, spix, objix);
        }
    }
#endif

  /* callback to user if object index header */

  if (fs->file_cb_f && spix == 0 && (obj_id_raw & SPIFFS_OBJ_ID_IX_FLAG))
    {
      spiffs_fileop_type op;
      if (ev == SPIFFS_EV_IX_NEW)
        {
          op = SPIFFS_CB_CREATED;
        }
      else if (ev == SPIFFS_EV_IX_UPD ||
               ev == SPIFFS_EV_IX_MOV || ev == SPIFFS_EV_IX_UPD_HDR)
        {
          op = SPIFFS_CB_UPDATED;
        }
      else if (ev == SPIFFS_EV_IX_DEL)
        {
          op = SPIFFS_CB_DELETED;
        }
      else
        {
          finfo("       callback: WARNING unknown callback event " _SPIPRIi
                     "\n", ev);
          return;               /* bail out */
        }

      fs->file_cb_f(fs, op, obj_id, new_pix);
    }
}

/* Open object by id */

int32_t spiffs_object_open_by_id(spiffs * fs,
                                 spiffs_obj_id obj_id,
                                 spiffs_fd * fd,
                                 spiffs_flags flags, spiffs_mode mode)
{
  int32_t res = OK;
  spiffs_page_ix pix;

  res =
    spiffs_obj_lu_find_id_and_span(fs, obj_id | SPIFFS_OBJ_ID_IX_FLAG, 0, 0,
                                   &pix);
  SPIFFS_CHECK_RES(res);

  res = spiffs_object_open_by_page(fs, pix, fd, flags, mode);

  return res;
}

/* Open object by page index */

int32_t spiffs_object_open_by_page(spiffs * fs,
                                   spiffs_page_ix pix,
                                   spiffs_fd * fd,
                                   spiffs_flags flags, spiffs_mode mode)
{
  int32_t res = OK;
  spiffs_page_object_ix_header oix_hdr;
  spiffs_obj_id obj_id;
  int entry;

  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                   fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(spiffs_page_object_ix_header), (uint8_t *) & oix_hdr);
  SPIFFS_CHECK_RES(res);

  spiffs_block_ix bix = SPIFFS_BLOCK_FOR_PAGE(fs, pix);
  entry = SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pix);

  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                   0, SPIFFS_BLOCK_TO_PADDR(fs,
                                            bix) +
                   entry * sizeof(spiffs_obj_id), sizeof(spiffs_obj_id),
                   (uint8_t *) & obj_id);

  fd->fs = fs;
  fd->objix_hdr_pix = pix;
  fd->size = oix_hdr.size;
  fd->offset = 0;
  fd->cursor_objix_pix = pix;
  fd->cursor_objix_spix = 0;
  fd->obj_id = obj_id;
  fd->flags = flags;

  SPIFFS_VALIDATE_OBJIX(oix_hdr.p_hdr, fd->obj_id, 0);

  finfo("open: fd " _SPIPRIfd " is obj id " _SPIPRIid "\n", fd->file_nbr, fd->obj_id);
  return res;
}

/* Append to object.  Deep current object index (header) page in fs->work buffer */

int32_t spiffs_object_append(spiffs_fd * fd, uint32_t offset, uint8_t * data, uint32_t len)
{
  spiffs *fs = fd->fs;
  int32_t res = OK;
  uint32_t written = 0;

  finfo("append: " _SPIPRIi " bytes @ offs " _SPIPRIi " of size " _SPIPRIi
             "\n", len, offset, fd->size);

  if (offset > fd->size)
    {
      finfo("append: offset reversed to size\n");
      offset = fd->size;
    }

  res = spiffs_gc_check(fs, len + SPIFFS_DATA_PAGE_SIZE(fs));   /* add an extra 
                                                                 * page of data 
                                                                 * worth for meta */
  if (res != OK)
    {
      finfo("append: gc check fail " _SPIPRIi "\n", res);
    }

  SPIFFS_CHECK_RES(res);

  spiffs_page_object_ix_header *objix_hdr =
    (spiffs_page_object_ix_header *) fs->work;
  spiffs_page_object_ix *objix = (spiffs_page_object_ix *) fs->work;
  spiffs_page_header p_hdr;

  spiffs_span_ix cur_objix_spix = 0;
  spiffs_span_ix prev_objix_spix = (spiffs_span_ix) - 1;
  spiffs_page_ix cur_objix_pix = fd->objix_hdr_pix;
  spiffs_page_ix new_objix_hdr_page;

  spiffs_span_ix data_spix = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  spiffs_page_ix data_page;
  uint32_t page_offs = offset % SPIFFS_DATA_PAGE_SIZE(fs);

  /* write all data */

  while (res == OK && written < len)
    {
      /* calculate object index page span index */

      cur_objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);

      /* handle storing and loading of object indices */

      if (cur_objix_spix != prev_objix_spix)
        {
          /* new object index page.  Within this clause we return directly
           * if something fails, object index mess-up
           */

          if (written > 0)
            {
              /* store previous object index page, unless first pass */

              finfo("append: " _SPIPRIid " store objix " _SPIPRIpg ":"
                         _SPIPRIsp ", written " _SPIPRIi "\n", fd->obj_id,
                         cur_objix_pix, prev_objix_spix, written);

              if (prev_objix_spix == 0)
                {
                  /* this is an update to object index header page */

                  objix_hdr->size = offset + written;
                  if (offset == 0)
                    {
                      /* was an empty object, update same page (size was
                       * 0xffffffff)
                       */

                      res = spiffs_page_index_check(fs, fd, cur_objix_pix, 0);
                      SPIFFS_CHECK_RES(res);
                      res =
                        _spiffs_wr(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_UPDT,
                                   fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs,
                                                                      cur_objix_pix),
                                   SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
                      SPIFFS_CHECK_RES(res);
                    }
                  else
                    {
                      /* was a nonempty object, update to new page */

                      res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                                           fd->objix_hdr_pix,
                                                           fs->work, 0, 0,
                                                           offset + written,
                                                           &new_objix_hdr_page);
                      SPIFFS_CHECK_RES(res);
                      finfo("append: " _SPIPRIid " store new objix_hdr, "
                                 _SPIPRIpg ":" _SPIPRIsp ", written " _SPIPRIi
                                 "\n", fd->obj_id, new_objix_hdr_page, 0,
                                 written);
                    }
                }
              else
                {
                  /* this is an update to an object index page */

                  res =
                    spiffs_page_index_check(fs, fd, cur_objix_pix,
                                            prev_objix_spix);
                  SPIFFS_CHECK_RES(res);

                  res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_UPDT,
                                   fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs,
                                                                      cur_objix_pix),
                                   SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
                  SPIFFS_CHECK_RES(res);
                  spiffs_cb_object_event(fs, (spiffs_page_object_ix *) fs->work,
                                         SPIFFS_EV_IX_UPD, fd->obj_id,
                                         objix->p_hdr.span_ix, cur_objix_pix,
                                         0);

                  /* update length in object index header page */

                  res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                                       fd->objix_hdr_pix, 0, 0,
                                                       0, offset + written,
                                                       &new_objix_hdr_page);
                  SPIFFS_CHECK_RES(res);
                  finfo("append: " _SPIPRIid " store new size I " _SPIPRIi
                             " in objix_hdr, " _SPIPRIpg ":" _SPIPRIsp
                             ", written " _SPIPRIi "\n", fd->obj_id,
                             offset + written, new_objix_hdr_page, 0, written);
                }

              fd->size = offset + written;
              fd->offset = offset + written;
            }

          /* create or load new object index page */

          if (cur_objix_spix == 0)
            {
              /* load object index header page, must always exist */

              finfo("append: " _SPIPRIid " load objixhdr page " _SPIPRIpg
                         ":" _SPIPRIsp "\n", fd->obj_id, cur_objix_pix,
                         cur_objix_spix);
              res =
                _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                           fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs,
                                                              cur_objix_pix),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
              SPIFFS_CHECK_RES(res);
              SPIFFS_VALIDATE_OBJIX(objix_hdr->p_hdr, fd->obj_id,
                                    cur_objix_spix);
            }
          else
            {
              spiffs_span_ix len_objix_spix =
                SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs,
                                            (fd->size -
                                             1) / SPIFFS_DATA_PAGE_SIZE(fs));

              /* on subsequent passes, create a new object index page */

              if (written > 0 || cur_objix_spix > len_objix_spix)
                {
                  p_hdr.obj_id = fd->obj_id | SPIFFS_OBJ_ID_IX_FLAG;
                  p_hdr.span_ix = cur_objix_spix;
                  p_hdr.flags =
                    0xff & ~(SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_INDEX);
                  res =
                    spiffs_page_allocate_data(fs,
                                              fd->
                                              obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                              &p_hdr, 0, 0, 0, 1,
                                              &cur_objix_pix);
                  SPIFFS_CHECK_RES(res);

                  /* quick "load" of new object index page */

                  memset(fs->work, 0xff, SPIFFS_CFG_LOG_PAGE_SZ(fs));
                  memcpy(fs->work, &p_hdr, sizeof(spiffs_page_header));
                  spiffs_cb_object_event(fs, (spiffs_page_object_ix *) fs->work,
                                         SPIFFS_EV_IX_NEW, fd->obj_id,
                                         cur_objix_spix, cur_objix_pix, 0);
                  finfo("append: " _SPIPRIid " create objix page, "
                             _SPIPRIpg ":" _SPIPRIsp ", written " _SPIPRIi "\n",
                             fd->obj_id, cur_objix_pix, cur_objix_spix,
                             written);
                }
              else
                {
                  /* on first pass, we load existing object index page */

                  spiffs_page_ix pix;
                  finfo("append: " _SPIPRIid " find objix span_ix:"
                             _SPIPRIsp "\n", fd->obj_id, cur_objix_spix);
                  if (fd->cursor_objix_spix == cur_objix_spix)
                    {
                      pix = fd->cursor_objix_pix;
                    }
                  else
                    {
                      res =
                        spiffs_obj_lu_find_id_and_span(fs,
                                                       fd->
                                                       obj_id |
                                                       SPIFFS_OBJ_ID_IX_FLAG,
                                                       cur_objix_spix, 0, &pix);
                      SPIFFS_CHECK_RES(res);
                    }
                  finfo("append: " _SPIPRIid " found object index at page "
                             _SPIPRIpg " [fd size " _SPIPRIi "]\n", fd->obj_id,
                             pix, fd->size);
                  res =
                    _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                               fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs, pix),
                               SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
                  SPIFFS_CHECK_RES(res);
                  SPIFFS_VALIDATE_OBJIX(objix_hdr->p_hdr, fd->obj_id,
                                        cur_objix_spix);
                  cur_objix_pix = pix;
                }

              fd->cursor_objix_pix = cur_objix_pix;
              fd->cursor_objix_spix = cur_objix_spix;
              fd->offset = offset + written;
              fd->size = offset + written;
            }

          prev_objix_spix = cur_objix_spix;
        }

      /* write data */

      uint32_t to_write =
        MIN(len - written, SPIFFS_DATA_PAGE_SIZE(fs) - page_offs);
      if (page_offs == 0)
        {
          /* at beginning of a page, allocate and write a new page of data */

          p_hdr.obj_id = fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG;
          p_hdr.span_ix = data_spix;
          p_hdr.flags = 0xff & ~(SPIFFS_PH_FLAG_FINAL); /* finalize immediately */
          res =
            spiffs_page_allocate_data(fs, fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG,
                                      &p_hdr, &data[written], to_write,
                                      page_offs, 1, &data_page);
          finfo("append: " _SPIPRIid " store new data page, " _SPIPRIpg ":"
                     _SPIPRIsp " offset:" _SPIPRIi ", len " _SPIPRIi
                     ", written " _SPIPRIi "\n", fd->obj_id, data_page,
                     data_spix, page_offs, to_write, written);
        }
      else
        {
          /* append to existing page, fill out free data in existing page */

          if (cur_objix_spix == 0)
            {
              /* get data page from object index header page */

              data_page =
                ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                                     sizeof(spiffs_page_object_ix_header)))
                [data_spix];
            }
          else
            {
              /* get data page from object index page */

              data_page =
                ((spiffs_page_ix *) ((uint8_t *) objix +
                                     sizeof(spiffs_page_object_ix)))
                [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)];
            }

          res = spiffs_page_data_check(fs, fd, data_page, data_spix);
          SPIFFS_CHECK_RES(res);

          res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                           fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs,
                                                              data_page) +
                           sizeof(spiffs_page_header) + page_offs, to_write,
                           &data[written]);
          finfo("append: " _SPIPRIid " store to existing data page, "
                     _SPIPRIpg ":" _SPIPRIsp " offset:" _SPIPRIi ", len "
                     _SPIPRIi ", written " _SPIPRIi "\n", fd->obj_id, data_page,
                     data_spix, page_offs, to_write, written);
        }

      if (res != OK)
        {
          break;
        }

      /* update memory representation of object index page with new data page */

      if (cur_objix_spix == 0)
        {
          /* update object index header page */

          ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                               sizeof(spiffs_page_object_ix_header)))[data_spix]
            = data_page;

          finfo("append: " _SPIPRIid " wrote page " _SPIPRIpg
                     " to objix_hdr entry " _SPIPRIsp " in mem\n", fd->obj_id,
                     data_page, data_spix);

          objix_hdr->size = offset + written;
        }
      else
        {
          /* update object index page */

          ((spiffs_page_ix *) ((uint8_t *) objix +
                               sizeof(spiffs_page_object_ix)))
            [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)] = data_page;

          finfo("append: " _SPIPRIid " wrote page " _SPIPRIpg
                     " to objix entry " _SPIPRIsp " in mem\n", fd->obj_id,
                     data_page, (spiffs_span_ix) SPIFFS_OBJ_IX_ENTRY(fs,
                                                                     data_spix));
        }

      /* update internals */

      page_offs = 0;
      data_spix++;
      written += to_write;
    }

  fd->size = offset + written;
  fd->offset = offset + written;
  fd->cursor_objix_pix = cur_objix_pix;
  fd->cursor_objix_spix = cur_objix_spix;

  /* finalize updated object indices */

  int32_t res2 = OK;
  if (cur_objix_spix != 0)
    {
      /* wrote beyond object index header page.  Write last modified object
       * index page, unless object header index page
       */

      finfo("append: " _SPIPRIid " store objix page, " _SPIPRIpg ":"
                 _SPIPRIsp ", written " _SPIPRIi "\n", fd->obj_id,
                 cur_objix_pix, cur_objix_spix, written);

      res2 = spiffs_page_index_check(fs, fd, cur_objix_pix, cur_objix_spix);
      SPIFFS_CHECK_RES(res2);

      res2 = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_UPDT,
                        fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs, cur_objix_pix),
                        SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
      SPIFFS_CHECK_RES(res2);
      spiffs_cb_object_event(fs, (spiffs_page_object_ix *) fs->work,
                             SPIFFS_EV_IX_UPD, fd->obj_id, objix->p_hdr.span_ix,
                             cur_objix_pix, 0);

      /* update size in object header index page */

      res2 = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                            fd->objix_hdr_pix, 0, 0, 0,
                                            offset + written,
                                            &new_objix_hdr_page);
      finfo("append: " _SPIPRIid " store new size II " _SPIPRIi
                 " in objix_hdr, " _SPIPRIpg ":" _SPIPRIsp ", written " _SPIPRIi
                 ", res " _SPIPRIi "\n", fd->obj_id, offset + written,
                 new_objix_hdr_page, 0, written, res2);
      SPIFFS_CHECK_RES(res2);
    }
  else
    {
      /* wrote within object index header page */

      if (offset == 0)
        {
          /* wrote to empty object - simply update size and write whole page */

          objix_hdr->size = offset + written;
          finfo("append: " _SPIPRIid " store fresh objix_hdr page, "
                     _SPIPRIpg ":" _SPIPRIsp ", written " _SPIPRIi "\n",
                     fd->obj_id, cur_objix_pix, cur_objix_spix, written);

          res2 = spiffs_page_index_check(fs, fd, cur_objix_pix, cur_objix_spix);
          SPIFFS_CHECK_RES(res2);

          res2 = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_UPDT,
                            fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs,
                                                               cur_objix_pix),
                            SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
          SPIFFS_CHECK_RES(res2);

          /* callback on object index update */

          spiffs_cb_object_event(fs, (spiffs_page_object_ix *) fs->work,
                                 SPIFFS_EV_IX_UPD_HDR, fd->obj_id,
                                 objix_hdr->p_hdr.span_ix, cur_objix_pix,
                                 objix_hdr->size);
        }
      else
        {
          /* modifying object index header page, update size and make new copy */

          res2 = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                                fd->objix_hdr_pix, fs->work, 0,
                                                0, offset + written,
                                                &new_objix_hdr_page);
          finfo("append: " _SPIPRIid " store modified objix_hdr page, "
                     _SPIPRIpg ":" _SPIPRIsp ", written " _SPIPRIi "\n",
                     fd->obj_id, new_objix_hdr_page, 0, written);
          SPIFFS_CHECK_RES(res2);
        }
    }

  return res;
}

/* Modify object.  Keep current object index (header) page in fs->work buffer */

int32_t spiffs_object_modify(spiffs_fd * fd, uint32_t offset,
                             uint8_t * data, uint32_t len)
{
  spiffs *fs = fd->fs;
  int32_t res = OK;
  uint32_t written = 0;

  res = spiffs_gc_check(fs, len + SPIFFS_DATA_PAGE_SIZE(fs));
  SPIFFS_CHECK_RES(res);

  spiffs_page_object_ix_header *objix_hdr =
    (spiffs_page_object_ix_header *) fs->work;
  spiffs_page_object_ix *objix = (spiffs_page_object_ix *) fs->work;
  spiffs_page_header p_hdr;

  spiffs_span_ix cur_objix_spix = 0;
  spiffs_span_ix prev_objix_spix = (spiffs_span_ix) - 1;
  spiffs_page_ix cur_objix_pix = fd->objix_hdr_pix;
  spiffs_page_ix new_objix_hdr_pix;

  spiffs_span_ix data_spix = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  spiffs_page_ix data_pix;
  uint32_t page_offs = offset % SPIFFS_DATA_PAGE_SIZE(fs);

  /* write all data */

  while (res == OK && written < len)
    {
      /* calculate object index page span index */

      cur_objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);

      /* handle storing and loading of object indices */

      if (cur_objix_spix != prev_objix_spix)
        {
          /* new object index page.  Within this clause we return directly
           * if something fails, object index mess-up
           */

          if (written > 0)
            {
              /* store previous object index (header) page, unless first pass */

              if (prev_objix_spix == 0)
                {
                  /* store previous object index header page */

                  res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                                       fd->objix_hdr_pix,
                                                       fs->work, 0, 0, 0,
                                                       &new_objix_hdr_pix);
                  finfo("modify: store modified objix_hdr page, " _SPIPRIpg
                             ":" _SPIPRIsp ", written " _SPIPRIi "\n",
                             new_objix_hdr_pix, 0, written);
                  SPIFFS_CHECK_RES(res);
                }
              else
                {
                  /* store new version of previous object index page */

                  spiffs_page_ix new_objix_pix;

                  res =
                    spiffs_page_index_check(fs, fd, cur_objix_pix,
                                            prev_objix_spix);
                  SPIFFS_CHECK_RES(res);

                  res =
                    spiffs_page_move(fs, fd->file_nbr, (uint8_t *) objix,
                                     fd->obj_id, 0, cur_objix_pix,
                                     &new_objix_pix);
                  finfo("modify: store previous modified objix page, "
                             _SPIPRIid ":" _SPIPRIsp ", written " _SPIPRIi "\n",
                             new_objix_pix, objix->p_hdr.span_ix, written);
                  SPIFFS_CHECK_RES(res);
                  spiffs_cb_object_event(fs, (spiffs_page_object_ix *) objix,
                                         SPIFFS_EV_IX_UPD, fd->obj_id,
                                         objix->p_hdr.span_ix, new_objix_pix,
                                         0);
                }
            }

          /* load next object index page */

          if (cur_objix_spix == 0)
            {
              /* load object index header page, must exist */

              finfo("modify: load objixhdr page " _SPIPRIpg ":" _SPIPRIsp
                         "\n", cur_objix_pix, cur_objix_spix);
              res =
                _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                           fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs,
                                                              cur_objix_pix),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
              SPIFFS_CHECK_RES(res);
              SPIFFS_VALIDATE_OBJIX(objix_hdr->p_hdr, fd->obj_id,
                                    cur_objix_spix);
            }
          else
            {
              /* load existing object index page on first pass */

              spiffs_page_ix pix;
              finfo("modify: find objix span_ix:" _SPIPRIsp "\n",
                         cur_objix_spix);
              if (fd->cursor_objix_spix == cur_objix_spix)
                {
                  pix = fd->cursor_objix_pix;
                }
              else
                {
                  res =
                    spiffs_obj_lu_find_id_and_span(fs,
                                                   fd->
                                                   obj_id |
                                                   SPIFFS_OBJ_ID_IX_FLAG,
                                                   cur_objix_spix, 0, &pix);
                  SPIFFS_CHECK_RES(res);
                }
              finfo("modify: found object index at page " _SPIPRIpg "\n",
                         pix);
              res =
                _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                           fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs, pix),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
              SPIFFS_CHECK_RES(res);
              SPIFFS_VALIDATE_OBJIX(objix_hdr->p_hdr, fd->obj_id,
                                    cur_objix_spix);
              cur_objix_pix = pix;
            }

          fd->cursor_objix_pix = cur_objix_pix;
          fd->cursor_objix_spix = cur_objix_spix;
          fd->offset = offset + written;
          prev_objix_spix = cur_objix_spix;
        }

      /* write partial data */

      uint32_t to_write =
        MIN(len - written, SPIFFS_DATA_PAGE_SIZE(fs) - page_offs);
      spiffs_page_ix orig_data_pix;

      if (cur_objix_spix == 0)
        {
          /* get data page from object index header page */

          orig_data_pix =
            ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                                 sizeof(spiffs_page_object_ix_header)))
            [data_spix];
        }
      else
        {
          /* get data page from object index page */

          orig_data_pix =
            ((spiffs_page_ix *) ((uint8_t *) objix +
                                 sizeof(spiffs_page_object_ix)))
            [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)];
        }

      p_hdr.obj_id = fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG;
      p_hdr.span_ix = data_spix;
      p_hdr.flags = 0xff;

      if (page_offs == 0 && to_write == SPIFFS_DATA_PAGE_SIZE(fs))
        {
          /* a full page, allocate and write a new page of data */

          res =
            spiffs_page_allocate_data(fs, fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG,
                                      &p_hdr, &data[written], to_write,
                                      page_offs, 1, &data_pix);
          finfo("modify: store new data page, " _SPIPRIpg ":" _SPIPRIsp
                     " offset:" _SPIPRIi ", len " _SPIPRIi ", written " _SPIPRIi
                     "\n", data_pix, data_spix, page_offs, to_write, written);
        }
      else
        {
          /* write to existing page, allocate new and copy unmodified data */

          res = spiffs_page_data_check(fs, fd, orig_data_pix, data_spix);
          SPIFFS_CHECK_RES(res);

          res =
            spiffs_page_allocate_data(fs, fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG,
                                      &p_hdr, 0, 0, 0, 0, &data_pix);
          if (res != OK)
            break;

          /* copy unmodified data */

          if (page_offs > 0)
            {
              /* before modification */

              res = spiffs_phys_cpy(fs, fd->file_nbr,
                                    SPIFFS_PAGE_TO_PADDR(fs,
                                                         data_pix) +
                                    sizeof(spiffs_page_header),
                                    SPIFFS_PAGE_TO_PADDR(fs,
                                                         orig_data_pix) +
                                    sizeof(spiffs_page_header), page_offs);
              if (res != OK)
                break;
            }
          if (page_offs + to_write < SPIFFS_DATA_PAGE_SIZE(fs))
            {
              /* after modification */

              res = spiffs_phys_cpy(fs, fd->file_nbr,
                                    SPIFFS_PAGE_TO_PADDR(fs,
                                                         data_pix) +
                                    sizeof(spiffs_page_header) + page_offs +
                                    to_write, SPIFFS_PAGE_TO_PADDR(fs,
                                                                   orig_data_pix)
                                    + sizeof(spiffs_page_header) + page_offs +
                                    to_write,
                                    SPIFFS_DATA_PAGE_SIZE(fs) - (page_offs +
                                                                 to_write));
              if (res != OK)
                {
                  break;
                }
            }

          res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                           fd->file_nbr,
                           SPIFFS_PAGE_TO_PADDR(fs,
                                                data_pix) +
                           sizeof(spiffs_page_header) + page_offs, to_write,
                           &data[written]);
          if (res != OK)
            {
              break;
            }

          p_hdr.flags &= ~SPIFFS_PH_FLAG_FINAL;
          res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                           fd->file_nbr,
                           SPIFFS_PAGE_TO_PADDR(fs,
                                                data_pix) +
                           offsetof(spiffs_page_header, flags), sizeof(uint8_t),
                           (uint8_t *) & p_hdr.flags);
          if (res != OK)
            {
              break;
            }

          finfo("modify: store to existing data page, src:" _SPIPRIpg
                     ", dst:" _SPIPRIpg ":" _SPIPRIsp " offset:" _SPIPRIi
                     ", len " _SPIPRIi ", written " _SPIPRIi "\n",
                     orig_data_pix, data_pix, data_spix, page_offs, to_write,
                     written);
        }

      /* delete original data page */

      res = spiffs_page_delete(fs, orig_data_pix);
      if (res != OK)
        {
          break;
        }

      /* update memory representation of object index page with new data page */

      if (cur_objix_spix == 0)
        {
          /* update object index header page */

          ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                               sizeof(spiffs_page_object_ix_header)))[data_spix]
            = data_pix;
          finfo("modify: wrote page " _SPIPRIpg " to objix_hdr entry "
                     _SPIPRIsp " in mem\n", data_pix, data_spix);
        }
      else
        {
          /* update object index page */

          ((spiffs_page_ix *) ((uint8_t *) objix +
                               sizeof(spiffs_page_object_ix)))
            [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)] = data_pix;
          finfo("modify: wrote page " _SPIPRIpg " to objix entry "
                     _SPIPRIsp " in mem\n", data_pix,
                     (spiffs_span_ix) SPIFFS_OBJ_IX_ENTRY(fs, data_spix));
        }

      /* update internals */

      page_offs = 0;
      data_spix++;
      written += to_write;
    }

  fd->offset = offset + written;
  fd->cursor_objix_pix = cur_objix_pix;
  fd->cursor_objix_spix = cur_objix_spix;

  /* finalize updated object indices */

  int32_t res2 = OK;
  if (cur_objix_spix != 0)
    {
      /* wrote beyond object index header page.  Write last modified object
       * index page.  Move and update page
       */

      spiffs_page_ix new_objix_pix;

      res2 = spiffs_page_index_check(fs, fd, cur_objix_pix, cur_objix_spix);
      SPIFFS_CHECK_RES(res2);

      res2 =
        spiffs_page_move(fs, fd->file_nbr, (uint8_t *) objix, fd->obj_id, 0,
                         cur_objix_pix, &new_objix_pix);
      finfo("modify: store modified objix page, " _SPIPRIpg ":" _SPIPRIsp
                 ", written " _SPIPRIi "\n", new_objix_pix, cur_objix_spix,
                 written);
      fd->cursor_objix_pix = new_objix_pix;
      fd->cursor_objix_spix = cur_objix_spix;
      SPIFFS_CHECK_RES(res2);
      spiffs_cb_object_event(fs, (spiffs_page_object_ix *) objix,
                             SPIFFS_EV_IX_UPD, fd->obj_id, objix->p_hdr.span_ix,
                             new_objix_pix, 0);

    }
  else
    {
      /* wrote within object index header page */

      res2 = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                            fd->objix_hdr_pix, fs->work, 0, 0,
                                            0, &new_objix_hdr_pix);
      finfo("modify: store modified objix_hdr page, " _SPIPRIpg ":"
                 _SPIPRIsp ", written " _SPIPRIi "\n", new_objix_hdr_pix, 0,
                 written);
      SPIFFS_CHECK_RES(res2);
    }

  return res;
}

static int32_t spiffs_object_find_object_index_header_by_name_v(spiffs * fs,
                                                                spiffs_obj_id
                                                                obj_id,
                                                                spiffs_block_ix
                                                                bix, int ix_entry,
                                                                const void
                                                                *user_const_p,
                                                                void *user_var_p)
{
  int32_t res;
  spiffs_page_object_ix_header objix_hdr;
  spiffs_page_ix pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, ix_entry);

  if (obj_id == SPIFFS_OBJ_ID_FREE || obj_id == SPIFFS_OBJ_ID_DELETED ||
      (obj_id & SPIFFS_OBJ_ID_IX_FLAG) == 0)
    {
      return SPIFFS_VIS_COUNTINUE;
    }

  res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   0, SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(spiffs_page_object_ix_header), (uint8_t *) & objix_hdr);
  SPIFFS_CHECK_RES(res);
  if (objix_hdr.p_hdr.span_ix == 0 &&
      (objix_hdr.p_hdr.
       flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL |
                SPIFFS_PH_FLAG_IXDELE)) ==
      (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE))
    {
      if (strcmp((const char *)user_const_p, (char *)objix_hdr.name) == 0)
        {
          return OK;
        }
    }

  return SPIFFS_VIS_COUNTINUE;
}

/* Finds object index header page by name */

int32_t spiffs_object_find_object_index_header_by_name(spiffs * fs,
                                                       const uint8_t
                                                       name[SPIFFS_NAME_MAX],
                                                       spiffs_page_ix * pix)
{
  int32_t res;
  spiffs_block_ix bix;
  int entry;

  res = spiffs_obj_lu_find_entry_visitor(fs,
                                         fs->cursor_block_ix,
                                         fs->cursor_obj_lu_entry,
                                         0,
                                         0,
                                         spiffs_object_find_object_index_header_by_name_v,
                                         name, 0, &bix, &entry);

  if (res == SPIFFS_VIS_END)
    {
      res = SPIFFS_ERR_NOT_FOUND;
    }

  SPIFFS_CHECK_RES(res);

  if (pix)
    {
      *pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, entry);
    }

  fs->cursor_block_ix = bix;
  fs->cursor_obj_lu_entry = entry;

  return res;
}

/* Truncates object to new size. If new size is null, object may be removed totally */

int32_t spiffs_object_truncate(spiffs_fd * fd, uint32_t new_size, uint8_t remove_full)
{
  int32_t res = OK;
  spiffs *fs = fd->fs;

  if ((fd->size == SPIFFS_UNDEFINED_LEN || fd->size == 0) && !remove_full)
    {
      /* no op */

      return res;
    }

  /* need 2 pages if not removing: object index page + possibly chopped data
   * page
   */

  if (remove_full == 0)
    {
      res = spiffs_gc_check(fs, SPIFFS_DATA_PAGE_SIZE(fs) * 2);
      SPIFFS_CHECK_RES(res);
    }

  spiffs_page_ix objix_pix = fd->objix_hdr_pix;
  spiffs_span_ix data_spix =
    (fd->size > 0 ? fd->size - 1 : 0) / SPIFFS_DATA_PAGE_SIZE(fs);
  uint32_t cur_size = fd->size == (uint32_t) SPIFFS_UNDEFINED_LEN ? 0 : fd->size;
  spiffs_span_ix cur_objix_spix = 0;
  spiffs_span_ix prev_objix_spix = (spiffs_span_ix) - 1;
  spiffs_page_object_ix_header *objix_hdr =
    (spiffs_page_object_ix_header *) fs->work;
  spiffs_page_object_ix *objix = (spiffs_page_object_ix *) fs->work;
  spiffs_page_ix data_pix;
  spiffs_page_ix new_objix_hdr_pix;

  /* before truncating, check if object is to be fully removed and mark this */

  if (remove_full && new_size == 0)
    {
      uint8_t flags =
        ~(SPIFFS_PH_FLAG_USED | SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
          SPIFFS_PH_FLAG_IXDELE);
      res =
        _spiffs_wr(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_UPDT, fd->file_nbr,
                   SPIFFS_PAGE_TO_PADDR(fs,
                                        fd->objix_hdr_pix) +
                   offsetof(spiffs_page_header, flags), sizeof(uint8_t),
                   (uint8_t *) & flags);
      SPIFFS_CHECK_RES(res);
    }

  /* delete from end of object until desired len is reached */

  while (cur_size > new_size)
    {
      cur_objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);

      /* put object index for current data span index in work buffer */

      if (prev_objix_spix != cur_objix_spix)
        {
          if (prev_objix_spix != (spiffs_span_ix) - 1)
            {
              /* remove previous object index page */

              finfo("truncate: delete objix page " _SPIPRIpg ":" _SPIPRIsp
                         "\n", objix_pix, prev_objix_spix);

              res = spiffs_page_index_check(fs, fd, objix_pix, prev_objix_spix);
              SPIFFS_CHECK_RES(res);

              res = spiffs_page_delete(fs, objix_pix);
              SPIFFS_CHECK_RES(res);
              spiffs_cb_object_event(fs, (spiffs_page_object_ix *) 0,
                                     SPIFFS_EV_IX_DEL, fd->obj_id,
                                     objix->p_hdr.span_ix, objix_pix, 0);
              if (prev_objix_spix > 0)
                {
                  /* Update object index header page, unless we totally want to 
                   * remove the file.
                   * If fully removing, we're not keeping consistency as good
                   * as when storing the header between chunks,
                   * would we be aborted. But when removing full files, a
                   * crammed system may otherwise
                   * report ERR_FULL a la windows. We cannot have that.
                   * Hence, take the risk - if aborted, a file check would free 
                   * the lost pages and mend things
                   * as the file is marked as fully deleted in the beginning.
                   */

                  if (remove_full == 0)
                    {
                      finfo("truncate: update objix hdr page " _SPIPRIpg
                                 ":" _SPIPRIsp " to size " _SPIPRIi "\n",
                                 fd->objix_hdr_pix, prev_objix_spix, cur_size);
                      res =
                        spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                                       fd->objix_hdr_pix, 0, 0,
                                                       0, cur_size,
                                                       &new_objix_hdr_pix);
                      SPIFFS_CHECK_RES(res);
                    }
                  fd->size = cur_size;
                }
            }

          /* load current object index (header) page */

          if (cur_objix_spix == 0)
            {
              objix_pix = fd->objix_hdr_pix;
            }
          else
            {
              res =
                spiffs_obj_lu_find_id_and_span(fs,
                                               fd->
                                               obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                               cur_objix_spix, 0, &objix_pix);
              SPIFFS_CHECK_RES(res);
            }

          finfo("truncate: load objix page " _SPIPRIpg ":" _SPIPRIsp
                     " for data spix:" _SPIPRIsp "\n", objix_pix,
                     cur_objix_spix, data_spix);
          res =
            _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ, fd->file_nbr,
                       SPIFFS_PAGE_TO_PADDR(fs, objix_pix),
                       SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
          SPIFFS_CHECK_RES(res);
          SPIFFS_VALIDATE_OBJIX(objix_hdr->p_hdr, fd->obj_id, cur_objix_spix);
          fd->cursor_objix_pix = objix_pix;
          fd->cursor_objix_spix = cur_objix_spix;
          fd->offset = cur_size;

          prev_objix_spix = cur_objix_spix;
        }

      if (cur_objix_spix == 0)
        {
          /* get data page from object index header page */

          data_pix =
            ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                                 sizeof(spiffs_page_object_ix_header)))
            [data_spix];
          ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                               sizeof(spiffs_page_object_ix_header)))[data_spix]
            = SPIFFS_OBJ_ID_FREE;
        }
      else
        {
          /* get data page from object index page */

          data_pix =
            ((spiffs_page_ix *) ((uint8_t *) objix +
                                 sizeof(spiffs_page_object_ix)))
            [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)];
          ((spiffs_page_ix *) ((uint8_t *) objix +
                               sizeof(spiffs_page_object_ix)))
            [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)] = SPIFFS_OBJ_ID_FREE;
        }

      finfo("truncate: got data pix " _SPIPRIpg "\n", data_pix);

      if (new_size == 0 || remove_full ||
          cur_size - new_size >= SPIFFS_DATA_PAGE_SIZE(fs))
        {
          /* delete full data page */

          res = spiffs_page_data_check(fs, fd, data_pix, data_spix);
          if (res != SPIFFS_ERR_DELETED && res != OK &&
              res != SPIFFS_ERR_INDEX_REF_FREE)
            {
              finfo("truncate: err validating data pix " _SPIPRIi "\n",
                         res);
              break;
            }

          if (res == OK)
            {
              res = spiffs_page_delete(fs, data_pix);
              if (res != OK)
                {
                  finfo("truncate: err deleting data pix " _SPIPRIi "\n",
                             res);
                  break;
                }
            }
          else if (res == SPIFFS_ERR_DELETED ||
                   res == SPIFFS_ERR_INDEX_REF_FREE)
            {
              res = OK;
            }

          /* update current size */

          if (cur_size % SPIFFS_DATA_PAGE_SIZE(fs) == 0)
            {
              cur_size -= SPIFFS_DATA_PAGE_SIZE(fs);
            }
          else
            {
              cur_size -= cur_size % SPIFFS_DATA_PAGE_SIZE(fs);
            }

          fd->size = cur_size;
          fd->offset = cur_size;
          finfo("truncate: delete data page " _SPIPRIpg " for data spix:"
                _SPIPRIsp ", cur_size:" _SPIPRIi "\n", data_pix, data_spix,
                cur_size);
        }
      else
        {
          /* delete last page, partially */

          spiffs_page_header p_hdr;
          spiffs_page_ix new_data_pix;
          uint32_t bytes_to_remove =
            SPIFFS_DATA_PAGE_SIZE(fs) - (new_size % SPIFFS_DATA_PAGE_SIZE(fs));
          finfo("truncate: delete " _SPIPRIi " bytes from data page "
                     _SPIPRIpg " for data spix:" _SPIPRIsp ", cur_size:"
                     _SPIPRIi "\n", bytes_to_remove, data_pix, data_spix,
                     cur_size);

          res = spiffs_page_data_check(fs, fd, data_pix, data_spix);
          if (res != OK)
            {
              break;
            }

          p_hdr.obj_id = fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG;
          p_hdr.span_ix = data_spix;
          p_hdr.flags = 0xff;
 
          /* allocate new page and copy unmodified data */

          res =
            spiffs_page_allocate_data(fs, fd->obj_id & ~SPIFFS_OBJ_ID_IX_FLAG,
                                      &p_hdr, 0, 0, 0, 0, &new_data_pix);
          if (res != OK)
            {
              break;
            }

          res = spiffs_phys_cpy(fs, 0,
                                SPIFFS_PAGE_TO_PADDR(fs,
                                                     new_data_pix) +
                                sizeof(spiffs_page_header),
                                SPIFFS_PAGE_TO_PADDR(fs,
                                                     data_pix) +
                                sizeof(spiffs_page_header),
                                SPIFFS_DATA_PAGE_SIZE(fs) - bytes_to_remove);
          if (res != OK)
            {
              break;
            }

          /* delete original data page */

          res = spiffs_page_delete(fs, data_pix);
          if (res != OK)
            {
              break;
            }

          p_hdr.flags &= ~SPIFFS_PH_FLAG_FINAL;
          res = _spiffs_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                           fd->file_nbr,
                           SPIFFS_PAGE_TO_PADDR(fs,
                                                new_data_pix) +
                           offsetof(spiffs_page_header, flags), sizeof(uint8_t),
                           (uint8_t *) & p_hdr.flags);
          if (res != OK)
            {
              break;
            }

          /* update memory representation of object index page with new data
           * page
           */

          if (cur_objix_spix == 0)
            {
              /* update object index header page */

              ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                                   sizeof(spiffs_page_object_ix_header)))
                [data_spix] = new_data_pix;
              finfo("truncate: wrote page " _SPIPRIpg
                         " to objix_hdr entry " _SPIPRIsp " in mem\n",
                         new_data_pix, (spiffs_span_ix) SPIFFS_OBJ_IX_ENTRY(fs,
                                                                            data_spix));
            }
          else
            {
              /* update object index page */

              ((spiffs_page_ix *) ((uint8_t *) objix +
                                   sizeof(spiffs_page_object_ix)))
                [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)] = new_data_pix;
              finfo("truncate: wrote page " _SPIPRIpg " to objix entry "
                         _SPIPRIsp " in mem\n", new_data_pix,
                         (spiffs_span_ix) SPIFFS_OBJ_IX_ENTRY(fs, data_spix));
            }

          cur_size = new_size;
          fd->size = new_size;
          fd->offset = cur_size;
          break;
        }

      data_spix--;
    }

  /* update object indices */

  if (cur_objix_spix == 0)
    {
      /* update object index header page */

      if (cur_size == 0)
        {
          if (remove_full)
            {
              /* remove object altogether */

              finfo("truncate: remove object index header page " _SPIPRIpg
                         "\n", objix_pix);

              res = spiffs_page_index_check(fs, fd, objix_pix, 0);
              SPIFFS_CHECK_RES(res);

              res = spiffs_page_delete(fs, objix_pix);
              SPIFFS_CHECK_RES(res);
              spiffs_cb_object_event(fs, (spiffs_page_object_ix *) 0,
                                     SPIFFS_EV_IX_DEL, fd->obj_id, 0, objix_pix,
                                     0);
            }
          else
            {
              /* make uninitialized object */

              finfo("truncate: reset objix_hdr page " _SPIPRIpg "\n",
                         objix_pix);
              memset(fs->work + sizeof(spiffs_page_object_ix_header), 0xff,
                     SPIFFS_CFG_LOG_PAGE_SZ(fs) -
                     sizeof(spiffs_page_object_ix_header));
              res =
                spiffs_object_update_index_hdr(fs, fd, fd->obj_id, objix_pix,
                                               fs->work, 0, 0,
                                               SPIFFS_UNDEFINED_LEN,
                                               &new_objix_hdr_pix);
              SPIFFS_CHECK_RES(res);
            }
        }
      else
        {
          /* update object index header page */

          finfo("truncate: update object index header page with indices and size\n");
          res =
            spiffs_object_update_index_hdr(fs, fd, fd->obj_id, objix_pix,
                                           fs->work, 0, 0, cur_size,
                                           &new_objix_hdr_pix);
          SPIFFS_CHECK_RES(res);
        }
    }
  else
    {
      /* update both current object index page and object index header page */

      spiffs_page_ix new_objix_pix;

      res = spiffs_page_index_check(fs, fd, objix_pix, cur_objix_spix);
      SPIFFS_CHECK_RES(res);

      /* move and update object index page */

      res =
        spiffs_page_move(fs, fd->file_nbr, (uint8_t *) objix_hdr, fd->obj_id, 0,
                         objix_pix, &new_objix_pix);
      SPIFFS_CHECK_RES(res);
      spiffs_cb_object_event(fs, (spiffs_page_object_ix *) objix_hdr,
                             SPIFFS_EV_IX_UPD, fd->obj_id, objix->p_hdr.span_ix,
                             new_objix_pix, 0);
      finfo("truncate: store modified objix page, " _SPIPRIpg ":" _SPIPRIsp
                 "\n", new_objix_pix, cur_objix_spix);
      fd->cursor_objix_pix = new_objix_pix;
      fd->cursor_objix_spix = cur_objix_spix;
      fd->offset = cur_size;

      /* update object index header page with new size */

      res = spiffs_object_update_index_hdr(fs, fd, fd->obj_id,
                                           fd->objix_hdr_pix, 0, 0, 0, cur_size,
                                           &new_objix_hdr_pix);
      SPIFFS_CHECK_RES(res);
    }

  fd->size = cur_size;
  return res;
}

int32_t spiffs_object_read(spiffs_fd * fd, uint32_t offset, uint32_t len, uint8_t *dst)
{
  int32_t res = OK;
  spiffs *fs = fd->fs;
  spiffs_page_ix objix_pix;
  spiffs_page_ix data_pix;
  spiffs_span_ix data_spix = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  uint32_t cur_offset = offset;
  spiffs_span_ix cur_objix_spix;
  spiffs_span_ix prev_objix_spix = (spiffs_span_ix) - 1;
  spiffs_page_object_ix_header *objix_hdr =
    (spiffs_page_object_ix_header *) fs->work;
  spiffs_page_object_ix *objix = (spiffs_page_object_ix *) fs->work;

  while (cur_offset < offset + len)
    {
#if SPIFFS_IX_MAP
      /* check if we have a memory, index map and if so, if we're within index
       * map's range and if so, if the entry is populated
       */

      if (fd->ix_map && data_spix >= fd->ix_map->start_spix &&
          data_spix <= fd->ix_map->end_spix &&
          fd->ix_map->map_buf[data_spix - fd->ix_map->start_spix])
        {
          data_pix = fd->ix_map->map_buf[data_spix - fd->ix_map->start_spix];
        }
      else
        {
#endif
          cur_objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);
          if (prev_objix_spix != cur_objix_spix)
            {
              /* load current object index (header) page */

              if (cur_objix_spix == 0)
                {
                  objix_pix = fd->objix_hdr_pix;
                }
              else
                {
                  finfo("read: find objix " _SPIPRIid ":" _SPIPRIsp "\n",
                             fd->obj_id, cur_objix_spix);
                  if (fd->cursor_objix_spix == cur_objix_spix)
                    {
                      objix_pix = fd->cursor_objix_pix;
                    }
                  else
                    {
                      res =
                        spiffs_obj_lu_find_id_and_span(fs,
                                                       fd->
                                                       obj_id |
                                                       SPIFFS_OBJ_ID_IX_FLAG,
                                                       cur_objix_spix, 0,
                                                       &objix_pix);
                      SPIFFS_CHECK_RES(res);
                    }
                }

              finfo("read: load objix page " _SPIPRIpg ":" _SPIPRIsp
                         " for data spix:" _SPIPRIsp "\n", objix_pix,
                         cur_objix_spix, data_spix);
              res =
                _spiffs_rd(fs, SPIFFS_OP_T_OBJ_IX | SPIFFS_OP_C_READ,
                           fd->file_nbr, SPIFFS_PAGE_TO_PADDR(fs, objix_pix),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
              SPIFFS_CHECK_RES(res);
              SPIFFS_VALIDATE_OBJIX(objix->p_hdr, fd->obj_id, cur_objix_spix);

              fd->offset = cur_offset;
              fd->cursor_objix_pix = objix_pix;
              fd->cursor_objix_spix = cur_objix_spix;

              prev_objix_spix = cur_objix_spix;
            }

          if (cur_objix_spix == 0)
            {
              /* get data page from object index header page */

              data_pix =
                ((spiffs_page_ix *) ((uint8_t *) objix_hdr +
                                     sizeof(spiffs_page_object_ix_header)))
                [data_spix];
            }
          else
            {
              /* get data page from object index page */

              data_pix =
                ((spiffs_page_ix *) ((uint8_t *) objix +
                                     sizeof(spiffs_page_object_ix)))
                [SPIFFS_OBJ_IX_ENTRY(fs, data_spix)];
            }
#if SPIFFS_IX_MAP
        }
#endif

      /* all remaining data */

      uint32_t len_to_read = offset + len - cur_offset;

      /* remaining data in page */

      len_to_read =
        MIN(len_to_read,
            SPIFFS_DATA_PAGE_SIZE(fs) -
            (cur_offset % SPIFFS_DATA_PAGE_SIZE(fs)));

      /* remaining data in file */

      len_to_read = MIN(len_to_read, fd->size);
      finfo("read: offset:" _SPIPRIi " rd:" _SPIPRIi " data spix:"
                 _SPIPRIsp " is data_pix:" _SPIPRIpg " addr:" _SPIPRIad "\n",
                 cur_offset, len_to_read, data_spix, data_pix,
                 (uint32_t) (SPIFFS_PAGE_TO_PADDR(fs, data_pix) +
                          sizeof(spiffs_page_header) +
                          (cur_offset % SPIFFS_DATA_PAGE_SIZE(fs))));
      if (len_to_read <= 0)
        {
          res = SPIFFS_ERR_END_OF_OBJECT;
          break;
        }

      res = spiffs_page_data_check(fs, fd, data_pix, data_spix);
      SPIFFS_CHECK_RES(res);
      res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                       fd->file_nbr,
                       SPIFFS_PAGE_TO_PADDR(fs,
                                            data_pix) +
                       sizeof(spiffs_page_header) +
                       (cur_offset % SPIFFS_DATA_PAGE_SIZE(fs)), len_to_read,
                       dst);
      SPIFFS_CHECK_RES(res);
      dst += len_to_read;
      cur_offset += len_to_read;
      fd->offset = cur_offset;
      data_spix++;
    }

  return res;
}

static int32_t spiffs_obj_lu_find_free_obj_id_bitmap_v(spiffs * fs,
                                                       spiffs_obj_id id,
                                                       spiffs_block_ix bix,
                                                       int ix_entry,
                                                       const void *user_const_p,
                                                       void *user_var_p)
{
  if (id != SPIFFS_OBJ_ID_FREE && id != SPIFFS_OBJ_ID_DELETED)
    {
      spiffs_obj_id min_obj_id = *((spiffs_obj_id *) user_var_p);
      const uint8_t *conflicting_name = (const uint8_t *)user_const_p;

      /* if conflicting name parameter is given, also check if this name is
       * found in object index hdrs
       */

      if (conflicting_name && (id & SPIFFS_OBJ_ID_IX_FLAG))
        {
          spiffs_page_ix pix =
            SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, ix_entry);
          int res;
          spiffs_page_object_ix_header objix_hdr;
          res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                           0, SPIFFS_PAGE_TO_PADDR(fs, pix),
                           sizeof(spiffs_page_object_ix_header),
                           (uint8_t *) & objix_hdr);
          SPIFFS_CHECK_RES(res);
          if (objix_hdr.p_hdr.span_ix == 0 &&
              (objix_hdr.p_hdr.
               flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL |
                        SPIFFS_PH_FLAG_IXDELE)) ==
              (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE))
            {
              if (strcmp((const char *)user_const_p, (char *)objix_hdr.name) ==
                  0)
                {
                  return SPIFFS_ERR_CONFLICTING_NAME;
                }
            }
        }

      id &= ~SPIFFS_OBJ_ID_IX_FLAG;
      uint32_t bit_ix = (id - min_obj_id) & 7;
      int byte_ix = (id - min_obj_id) >> 3;
      if (byte_ix >= 0 && (uint32_t) byte_ix < SPIFFS_CFG_LOG_PAGE_SZ(fs))
        {
          fs->work[byte_ix] |= (1 << bit_ix);
        }
    }

  return SPIFFS_VIS_COUNTINUE;
}

static int32_t spiffs_obj_lu_find_free_obj_id_compact_v(spiffs * fs,
                                                        spiffs_obj_id id,
                                                        spiffs_block_ix bix,
                                                        int ix_entry,
                                                        const void *user_const_p,
                                                        void *user_var_p)
{
  if (id != SPIFFS_OBJ_ID_FREE && id != SPIFFS_OBJ_ID_DELETED &&
      (id & SPIFFS_OBJ_ID_IX_FLAG))
    {
      int32_t res;
      const spiffs_free_obj_id_state *state =
        (const spiffs_free_obj_id_state *)user_const_p;
      spiffs_page_object_ix_header objix_hdr;

      res = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                       0, SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, bix, ix_entry),
                       sizeof(spiffs_page_object_ix_header),
                       (uint8_t *) & objix_hdr);
      if (res == OK && objix_hdr.p_hdr.span_ix == 0 &&
          ((objix_hdr.p_hdr.
            flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                     SPIFFS_PH_FLAG_DELET)) == (SPIFFS_PH_FLAG_DELET)))
        {
          /* ok object look up entry */

          if (state->conflicting_name &&
              strcmp((const char *)state->conflicting_name,
                     (char *)objix_hdr.name) == 0)
            {
              return SPIFFS_ERR_CONFLICTING_NAME;
            }

          id &= ~SPIFFS_OBJ_ID_IX_FLAG;
          if (id >= state->min_obj_id && id <= state->max_obj_id)
            {
              uint8_t *map = (uint8_t *) fs->work;
              int ix = (id - state->min_obj_id) / state->compaction;
              map[ix]++;
            }
        }
    }

  return SPIFFS_VIS_COUNTINUE;
}

/* Scans thru all object lookup for object index header pages. If total possible number of
 * object ids cannot fit into a work buffer, these are grouped. When a group containing free
 * object ids is found, the object lu is again scanned for object ids within group and bitmasked.
 * Finally, the bitmask is searched for a free id
 */

int32_t spiffs_obj_lu_find_free_obj_id(spiffs * fs, spiffs_obj_id * obj_id,
                                       const uint8_t * conflicting_name)
{
  int32_t res = OK;
  uint32_t max_objects = (fs->block_count * SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs)) / 2;
  spiffs_free_obj_id_state state;
  spiffs_obj_id free_obj_id = SPIFFS_OBJ_ID_FREE;

  state.min_obj_id = 1;
  state.max_obj_id = max_objects + 1;
  if (state.max_obj_id & SPIFFS_OBJ_ID_IX_FLAG)
    {
      state.max_obj_id = ((spiffs_obj_id) - 1) & ~SPIFFS_OBJ_ID_IX_FLAG;
    }

  state.compaction = 0;
  state.conflicting_name = conflicting_name;
  while (res == OK && free_obj_id == SPIFFS_OBJ_ID_FREE)
    {
      if (state.max_obj_id - state.min_obj_id <=
          (spiffs_obj_id) SPIFFS_CFG_LOG_PAGE_SZ(fs) * 8)
        {
          /* possible to represent in bitmap */

          uint32_t i, j;
          finfo("free_obj_id: BITM min:" _SPIPRIid " max:" _SPIPRIid "\n",
                     state.min_obj_id, state.max_obj_id);

          memset(fs->work, 0, SPIFFS_CFG_LOG_PAGE_SZ(fs));
          res =
            spiffs_obj_lu_find_entry_visitor(fs, 0, 0, 0, 0,
                                             spiffs_obj_lu_find_free_obj_id_bitmap_v,
                                             conflicting_name,
                                             &state.min_obj_id, 0, 0);
          if (res == SPIFFS_VIS_END)
            {
              res = OK;
            }

          SPIFFS_CHECK_RES(res);

          /* traverse bitmask until found free obj_id */

          for (i = 0; i < SPIFFS_CFG_LOG_PAGE_SZ(fs); i++)
            {
              uint8_t mask = fs->work[i];
              if (mask == 0xff)
                {
                  continue;
                }

              for (j = 0; j < 8; j++)
                {
                  if ((mask & (1 << j)) == 0)
                    {
                      *obj_id = (i << 3) + j + state.min_obj_id;
                      return OK;
                    }
                }
            }

          return SPIFFS_ERR_FULL;
        }
      else
        {
          /* not possible to represent all ids in range in a bitmap, compact
           * and count
           */

          if (state.compaction != 0)
            {
              /* select element in compacted table, decrease range and
               * recompact
               */

              uint32_t i;
              uint32_t min_i = 0;
              uint8_t *map = (uint8_t *) fs->work;
              uint8_t min_count = 0xff;

              for (i = 0; i < SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(uint8_t); i++)
                {
                  if (map[i] < min_count)
                    {
                      min_count = map[i];
                      min_i = i;
                      if (min_count == 0)
                        {
                          break;
                        }
                    }
                }

              if (min_count == state.compaction)
                {
                  /* there are no free objids! */

                  finfo("free_obj_id: compacted table is full\n");
                  return SPIFFS_ERR_FULL;
                }

              finfo("free_obj_id: COMP select index:" _SPIPRIi
                         " min_count:" _SPIPRIi " min:" _SPIPRIid " max:"
                         _SPIPRIid " compact:" _SPIPRIi "\n", min_i, min_count,
                         state.min_obj_id, state.max_obj_id, state.compaction);

              if (min_count == 0)
                {
                  /* no id in this range, skip compacting and use directly */

                  *obj_id = min_i * state.compaction + state.min_obj_id;
                  return OK;
                }
              else
                {
                  finfo("free_obj_id: COMP SEL chunk:" _SPIPRIi " min:"
                             _SPIPRIid " -> " _SPIPRIid "\n", state.compaction,
                             state.min_obj_id,
                             state.min_obj_id + min_i * state.compaction);
                  state.min_obj_id += min_i * state.compaction;
                  state.max_obj_id = state.min_obj_id + state.compaction;

                  /* decrease compaction */
                }

              if ((state.max_obj_id - state.min_obj_id <=
                   (spiffs_obj_id) SPIFFS_CFG_LOG_PAGE_SZ(fs) * 8))
                {
                  /* no need for compacting, use bitmap */

                  continue;
                }
            }

          /* in a work memory of log_page_size bytes, we may fit in
           * log_page_size ids
           * todo what if compaction is > 255 - then we cannot fit it in a byte
           */

          state.compaction =
            (state.max_obj_id -
             state.min_obj_id) / ((SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(uint8_t)));

          finfo("free_obj_id: COMP min:" _SPIPRIid " max:" _SPIPRIid
                     " compact:" _SPIPRIi "\n", state.min_obj_id,
                     state.max_obj_id, state.compaction);

          memset(fs->work, 0, SPIFFS_CFG_LOG_PAGE_SZ(fs));
          res =
            spiffs_obj_lu_find_entry_visitor(fs, 0, 0, 0, 0,
                                             spiffs_obj_lu_find_free_obj_id_compact_v,
                                             &state, 0, 0, 0);
          if (res == SPIFFS_VIS_END)
            {
              res = OK;
            }

          SPIFFS_CHECK_RES(res);
          state.conflicting_name = 0;   /* searched for conflicting name once,
                                         * no need to do it again */
        }
    }

  return res;
}

#if SPIFFS_TEMPORAL_FD_CACHE
/* djb2 hash */

static uint32_t spiffs_hash(spiffs * fs, const uint8_t * name)
{
  (void)fs;
  uint32_t hash = 5381;
  uint8_t c;

  int i = 0;
  while ((c = name[i++]) && i < SPIFFS_NAME_MAX)
    {
      hash = (hash * 33) ^ c;
    }

  return hash;
}
#endif

int32_t spiffs_fd_find_new(spiffs * fs, spiffs_fd ** fd, const char *name)
{
#if SPIFFS_TEMPORAL_FD_CACHE
  uint32_t i;
  uint16_t min_score = 0xffff;
  uint32_t cand_ix = (uint32_t) - 1;
  uint32_t name_hash = name ? spiffs_hash(fs, (const uint8_t *)name) : 0;
  spiffs_fd *fds = (spiffs_fd *) fs->fd_space;

  if (name)
    {
      /* first, decrease score of all closed descriptors */

      for (i = 0; i < fs->fd_count; i++)
        {
          spiffs_fd *cur_fd = &fds[i];
          if (cur_fd->file_nbr == 0)
            {
              if (cur_fd->score > 1)
                {
                  /* score == 0 indicates never used fd */

                  cur_fd->score--;
                }
            }
        }
    }

  /* find the free fd with least score or name match */

  for (i = 0; i < fs->fd_count; i++)
    {
      spiffs_fd *cur_fd = &fds[i];
      if (cur_fd->file_nbr == 0)
        {
          if (name && cur_fd->name_hash == name_hash)
            {
              cand_ix = i;
              break;
            }

          if (cur_fd->score < min_score)
            {
              min_score = cur_fd->score;
              cand_ix = i;
            }
        }
    }

  if (cand_ix != (uint32_t) - 1)
    {
      spiffs_fd *cur_fd = &fds[cand_ix];
      if (name)
        {
          if (cur_fd->name_hash == name_hash && cur_fd->score > 0)
            {
              /* opened an fd with same name hash, assume same file
               * set search point to saved obj index page and hope we have a
               * correct match directly when start searching - if not, we
               * will just keep searching until it is found
               */

              fs->cursor_block_ix =
                SPIFFS_BLOCK_FOR_PAGE(fs, cur_fd->objix_hdr_pix);
              fs->cursor_obj_lu_entry =
                SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, cur_fd->objix_hdr_pix);

              /* update score */

              if (cur_fd->score < 0xffff - SPIFFS_TEMPORAL_CACHE_HIT_SCORE)
                {
                  cur_fd->score += SPIFFS_TEMPORAL_CACHE_HIT_SCORE;
                }
              else
                {
                  cur_fd->score = 0xffff;
                }
            }
          else
            {
              /* no hash hit, restore this fd to initial state */

              cur_fd->score = SPIFFS_TEMPORAL_CACHE_HIT_SCORE;
              cur_fd->name_hash = name_hash;
            }
        }

      cur_fd->file_nbr = cand_ix + 1;
      *fd = cur_fd;
      return OK;
    }
  else
    {
      return SPIFFS_ERR_OUT_OF_FILE_DESCS;
    }

#else
  uint32_t i;
  spiffs_fd *fds = (spiffs_fd *) fs->fd_space;

  for (i = 0; i < fs->fd_count; i++)
    {
      spiffs_fd *cur_fd = &fds[i];
      if (cur_fd->file_nbr == 0)
        {
          cur_fd->file_nbr = i + 1;
          *fd = cur_fd;
          return OK;
        }
    }

  return SPIFFS_ERR_OUT_OF_FILE_DESCS;
#endif
}

int32_t spiffs_fd_return(spiffs * fs, spiffs_file f)
{
  if (f <= 0 || f > (int16_t) fs->fd_count)
    {
      return SPIFFS_ERR_BAD_DESCRIPTOR;
    }

  spiffs_fd *fds = (spiffs_fd *) fs->fd_space;
  spiffs_fd *fd = &fds[f - 1];
  if (fd->file_nbr == 0)
    {
      return SPIFFS_ERR_FILE_CLOSED;
    }

  fd->file_nbr = 0;
#if SPIFFS_IX_MAP
  fd->ix_map = 0;
#endif
  return OK;
}

int32_t spiffs_fd_get(spiffs * fs, spiffs_file f, spiffs_fd ** fd)
{
  if (f <= 0 || f > (int16_t) fs->fd_count)
    {
      return SPIFFS_ERR_BAD_DESCRIPTOR;
    }

  spiffs_fd *fds = (spiffs_fd *) fs->fd_space;
  *fd = &fds[f - 1];
  if ((*fd)->file_nbr == 0)
    {
      return SPIFFS_ERR_FILE_CLOSED;
    }

  return OK;
}

#if SPIFFS_TEMPORAL_FD_CACHE
void spiffs_fd_temporal_cache_rehash(spiffs * fs,
                                     const char *old_path, const char *new_path)
{
  uint32_t i;
  uint32_t old_hash = spiffs_hash(fs, (const uint8_t *)old_path);
  uint32_t new_hash = spiffs_hash(fs, (const uint8_t *)new_path);
  spiffs_fd *fds = (spiffs_fd *) fs->fd_space;

  for (i = 0; i < fs->fd_count; i++)
    {
      spiffs_fd *cur_fd = &fds[i];
      if (cur_fd->score > 0 && cur_fd->name_hash == old_hash)
        {
          cur_fd->name_hash = new_hash;
        }
    }
}
#endif
