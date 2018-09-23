/****************************************************************************
 * fs/spiffs.h/spiffs_gc.c
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

#include "spiffs.h"
#include "spiffs_nucleus.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
  {
    FIND_OBJ_DATA,
    MOVE_OBJ_DATA,
    MOVE_OBJ_IX,
    FINISHED
  } spiffs_gc_clean_state;

typedef struct
  {
    spiffs_gc_clean_state state;
    int16_t cur_obj_id;
    int16_t cur_objndx_spndx;
    int16_t cur_objndx_pgndx;
    int16_t cur_data_pgndx;
    int stored_scan_entry_index;
    uint8_t obj_id_found;
  } spiffs_gc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Erases a logical block and updates the erase counter.
 * If cache is enabled, all pages that might be cached in this block
 * is dropped.
 */

static int32_t spiffs_gc_erase_block(FAR struct spiffs_s *fs, int16_t blkndx)
{
  int32_t ret;
  uint32_t i;

  spiffs_gcinfo("Erase block " _SPIPRIbl "\n", blkndx);
  ret = spiffs_erase_block(fs, blkndx);
  SPIFFS_CHECK_RES(ret);

  for (i = 0; i < SPIFFS_PAGES_PER_BLOCK(fs); i++)
    {
      spiffs_cache_drop_page(fs, SPIFFS_PAGE_FOR_BLOCK(fs, blkndx) + i);
    }

  return ret;
}

/* Searches for blocks where all entries are deleted - if one is found,
 * the block is erased. Compared to the non-quick gc, the quick one ensures
 * that no updates are needed on existing objects on pages that are erased.
 */

int32_t spiffs_gc_quick(FAR struct spiffs_s *fs, uint16_t max_free_pages)
{
  int32_t ret = OK;
  uint32_t blocks = fs->geo.neraseblocks;
  int16_t cur_block = 0;
  uint32_t cur_block_addr = 0;
  int cur_entry = 0;
  int16_t *obj_lu_buf = (int16_t *) fs->lu_work;

  spiffs_gcinfo("Running\n");
#if CONFIG_SPIFFS_GCDBG
  fs->stats_gc_runs++;
#endif

  int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t));

  /* find fully deleted blocks */

  while (ret == OK && blocks--)
    {
      uint16_t deleted_pages_in_block = 0;
      uint16_t free_pages_in_block = 0;
      int obj_lookup_page = 0;

      /* check each object lookup page */

      while (ret == OK &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;
          ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                           0, cur_block_addr + SPIFFS_PAGE_TO_PADDR(fs,
                                                                    obj_lookup_page),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);

          /* check each entry */

          while (ret == OK &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry <
                 (int)(SPIFFS_PAGES_PER_BLOCK(fs) -
                       SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t id = obj_lu_buf[cur_entry - entry_offset];
              if (id == SPIFFS_OBJ_ID_DELETED)
                {
                  deleted_pages_in_block++;
                }
              else if (id == SPIFFS_OBJ_ID_FREE)
                {
                  /* kill scan, go for next block */

                  free_pages_in_block++;
                  if (free_pages_in_block > max_free_pages)
                    {
                      obj_lookup_page = SPIFFS_OBJ_LOOKUP_PAGES(fs);
                      ret = 1;  /* kill object lu loop */
                      break;
                    }
                }
              else
                {
                  /* kill scan, go for next block */

                  obj_lookup_page = SPIFFS_OBJ_LOOKUP_PAGES(fs);
                  ret = 1;      /* kill object lu loop */
                  break;
                }

              cur_entry++;
            }

          obj_lookup_page++;
        }

      if (ret == 1)
        {
          ret = OK;
        }

      if (ret == OK &&
          deleted_pages_in_block + free_pages_in_block ==
          SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs) &&
          free_pages_in_block <= max_free_pages)
        {
          /* found a fully deleted block */

          fs->stats_p_deleted -= deleted_pages_in_block;
          ret = spiffs_gc_erase_block(fs, cur_block);
          return ret;
        }

      cur_entry = 0;
      cur_block++;
      cur_block_addr += SPIFFS_CFG_LOG_BLOCK_SZ(fs);
    }

  if (ret == OK)
    {
      ret = SPIFFS_ERR_NO_DELETED_BLOCKS;
    }

  return ret;
}

/* Checks if garbage collecting is necessary. If so a candidate block is found,
 * cleansed and erased
 * This function will  try to make room for given amount of bytes in the
 * filesystem by moving pages and erasing blocks.
 *
 * If it is physically impossible, err_no will be set to -ENOSPC. If
 * there already is this amount (or more) of free space, SPIFFS_gc will
 * silently return. It is recommended to call statfs() before invoking
 * this method in order to determine what amount of bytes to give.
 *
 * NB: the garbage collector is automatically called when spiffs needs free
 * pages. The reason for this function is to give possibility to do background
 * tidying when user knows the system is idle.
 *
 * Use with care.
 *
 * Input Parameters:
 *   fs            the file system struct
 *   size          amount of bytes that should be freed
 */

int spiffs_gc_check(FAR struct spiffs_s *fs, off_t len)
{
  int32_t free_pages =
    (SPIFFS_PAGES_PER_BLOCK(fs) -
     SPIFFS_OBJ_LOOKUP_PAGES(fs)) * (fs->geo.neraseblocks - 2) -
    fs->stats_p_allocated - fs->stats_p_deleted;
  int tries = 0;
  int ret;

  if (fs->free_blocks > 3 &&
      (int32_t) len < free_pages * (int32_t) SPIFFS_DATA_PAGE_SIZE(fs))
    {
      return OK;
    }

  uint32_t needed_pages =
    (len + SPIFFS_DATA_PAGE_SIZE(fs) - 1) / SPIFFS_DATA_PAGE_SIZE(fs);

#if 0
  if (fs->free_blocks <= 2 && (int32_t)needed_pages > free_pages)
    {
      spiffs_gcinfo("Full freeblk:" _SPIPRIi " needed:" _SPIPRIi " free:"
                    _SPIPRIi " dele:" _SPIPRIi "\n",
                    fs->free_blocks, needed_pages, free_pages, fs->stats_p_deleted);
      return -ENOSPC;
    }
#endif

  if ((int32_t) needed_pages > (int32_t) (free_pages + fs->stats_p_deleted))
    {
      spiffs_gcinfo("Full freeblk:" _SPIPRIi " needed:" _SPIPRIi
                    " free:" _SPIPRIi " dele:" _SPIPRIi "\n", fs->free_blocks,
                    needed_pages, free_pages, fs->stats_p_deleted);
      return -ENOSPC;
    }

  do
    {
      spiffs_gcinfo("#" _SPIPRIi ": run gc free_blocks:" _SPIPRIi
                    " pfree:" _SPIPRIi " pallo:" _SPIPRIi " pdele:" _SPIPRIi
                    " [" _SPIPRIi "] len:" _SPIPRIi " of " _SPIPRIi "\n", tries,
                    fs->free_blocks, free_pages, fs->stats_p_allocated,
                    fs->stats_p_deleted,
                    (free_pages + fs->stats_p_allocated + fs->stats_p_deleted),
                    len, (uint32_t) (free_pages * SPIFFS_DATA_PAGE_SIZE(fs)));

      int16_t *cands;
      int count;
      int16_t cand;
      int32_t prev_free_pages = free_pages;

      /* if the fs is crammed, ignore block age when selecting candidate - kind 
       * of a bad state
       */

      ret = spiffs_gc_find_candidate(fs, &cands, &count, free_pages <= 0);
      SPIFFS_CHECK_RES(ret);
      if (count == 0)
        {
          spiffs_gcinfo("No candidates, return\n");
          return (int32_t) needed_pages <
            free_pages ? OK : -ENOSPC;
        }

#if CONFIG_SPIFFS_GCDBG
      fs->stats_gc_runs++;
#endif
      cand = cands[0];
      fs->cleaning = 1;

      ret = spiffs_gc_clean(fs, cand);
      fs->cleaning = 0;
      if (ret < 0)
        {
          spiffs_gcinfo("Cleaning block " _SPIPRIi ", result " _SPIPRIi "\n",
                        cand, ret);
        }
      else
        {
          spiffs_gcinfo("Cleaning block " _SPIPRIi ", result " _SPIPRIi "\n",
                        cand, ret);
        }

      SPIFFS_CHECK_RES(ret);

      ret = spiffs_gc_erase_page_stats(fs, cand);
      SPIFFS_CHECK_RES(ret);

      ret = spiffs_gc_erase_block(fs, cand);
      SPIFFS_CHECK_RES(ret);

      free_pages = (SPIFFS_PAGES_PER_BLOCK(fs) -
                    SPIFFS_OBJ_LOOKUP_PAGES(fs)) * (fs->geo.neraseblocks - 2) -
                    fs->stats_p_allocated - fs->stats_p_deleted;

      if (prev_free_pages <= 0 && prev_free_pages == free_pages)
        {
          /* abort early to reduce wear, at least tried once */

          spiffs_gcinfo("Early abort, no result on gc when fs crammed\n");
          break;
        }

    }
  while (++tries < SPIFFS_GC_MAX_RUNS && (fs->free_blocks <= 2 ||
                                          (int32_t) len >
                                          free_pages *
                                          (int32_t) SPIFFS_DATA_PAGE_SIZE(fs)));

  free_pages =
    (SPIFFS_PAGES_PER_BLOCK(fs) -
     SPIFFS_OBJ_LOOKUP_PAGES(fs)) * (fs->geo.neraseblocks - 2) -
    fs->stats_p_allocated - fs->stats_p_deleted;

  if ((int32_t) len > free_pages * (int32_t) SPIFFS_DATA_PAGE_SIZE(fs))
    {
      ret = -ENOSPC;
    }

  spiffs_gcinfo("Finished, " _SPIPRIi " dirty, blocks " _SPIPRIi " free, "
                _SPIPRIi " pages free, " _SPIPRIi " tries, ret " _SPIPRIi "\n",
                fs->stats_p_allocated + fs->stats_p_deleted, fs->free_blocks,
                free_pages, tries, ret);

  return ret;
}

/* Updates page statistics for a block that is about to be erased */

int32_t spiffs_gc_erase_page_stats(FAR struct spiffs_s *fs, int16_t blkndx)
{
  int32_t ret = OK;
  int obj_lookup_page = 0;
  int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t));
  int16_t *obj_lu_buf = (int16_t *) fs->lu_work;
  int cur_entry = 0;
  uint32_t dele = 0;
  uint32_t allo = 0;

  /* check each object lookup page */

  while (ret == OK && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
      int entry_offset = obj_lookup_page * entries_per_page;
      ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                       0,
                       blkndx * SPIFFS_CFG_LOG_BLOCK_SZ(fs) +
                       SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                       SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);

      /* check each entry */

      while (ret == OK &&
             cur_entry - entry_offset < entries_per_page &&
             cur_entry <
             (int)(SPIFFS_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs)))
        {
          int16_t id = obj_lu_buf[cur_entry - entry_offset];
          if (id == SPIFFS_OBJ_ID_FREE)
            {
            }
          else if (id == SPIFFS_OBJ_ID_DELETED)
            {
              dele++;
            }
          else
            {
              allo++;
            }

          cur_entry++;
        }

      obj_lookup_page++;
    }

  spiffs_gcinfo("Wipe pallo:" _SPIPRIi " pdele:" _SPIPRIi "\n",
                allo,  dele);

  fs->stats_p_allocated -= allo;
  fs->stats_p_deleted -= dele;
  return ret;
}

/* Finds block candidates to erase */

int32_t spiffs_gc_find_candidate(FAR struct spiffs_s *fs,
                                 int16_t ** block_candidates,
                                 int *candidate_count, char fs_crammed)
{
  int32_t ret = OK;
  uint32_t blocks = fs->geo.neraseblocks;
  int16_t cur_block = 0;
  uint32_t cur_block_addr = 0;
  int16_t *obj_lu_buf = (int16_t *) fs->lu_work;
  int cur_entry = 0;

  /* using fs->work area as sorted candidate memory,
   * (int16_t)cand_blkndx/(int32_t)score
   */

  int max_candidates =
    MIN(fs->geo.neraseblocks,
        (SPIFFS_CFG_LOG_PAGE_SZ(fs) - 8) / (sizeof(int16_t) +
                                            sizeof(int32_t)));
  *candidate_count = 0;
  memset(fs->work, 0xff, SPIFFS_CFG_LOG_PAGE_SZ(fs));

  /* divide up work area into block indices and scores */

  int16_t *cand_blocks = (int16_t *) fs->work;
  int32_t *cand_scores =
    (int32_t *) (fs->work + max_candidates * sizeof(int16_t));

  /* align cand_scores on int32_t boundary */

  cand_scores =
    (int32_t *) (((intptr_t) cand_scores + sizeof(intptr_t) - 1) &
               ~(sizeof(intptr_t) - 1));

  *block_candidates = cand_blocks;

  int entries_per_page = (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t));

  /* check each block */

  while (ret == OK && blocks--)
    {
      uint16_t deleted_pages_in_block = 0;
      uint16_t used_pages_in_block = 0;

      int obj_lookup_page = 0;

      /* check each object lookup page */

      while (ret == OK &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;
          ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                           0, cur_block_addr + SPIFFS_PAGE_TO_PADDR(fs,
                                                                    obj_lookup_page),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);

          /* check each entry */

          while (ret == OK &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry <
                 (int)(SPIFFS_PAGES_PER_BLOCK(fs) -
                       SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t id = obj_lu_buf[cur_entry - entry_offset];
              if (id == SPIFFS_OBJ_ID_FREE)
                {
                  /* when a free entry is encountered, scan logic ensures that
                   * all following entries are free also
                   */

                  ret = 1;      /* kill object lu loop */
                  break;
                }
              else if (id == SPIFFS_OBJ_ID_DELETED)
                {
                  deleted_pages_in_block++;
                }
              else
                {
                  used_pages_in_block++;
                }

              cur_entry++;
            }

          obj_lookup_page++;
        }

      if (ret == 1)
        {
          ret = OK;
        }

      /* calculate score and insert into candidate table
       * stoneage sort, but probably not so many blocks
       */

      if (ret == OK /* && deleted_pages_in_block > 0 */ )
        {
          /* read erase count */

          int16_t erase_count;
          ret = _spiffs_rd(fs, SPIFFS_OP_C_READ | SPIFFS_OP_T_OBJ_LU2, 0,
                           SPIFFS_ERASE_COUNT_PADDR(fs, cur_block),
                           sizeof(int16_t), (uint8_t *) & erase_count);
          SPIFFS_CHECK_RES(ret);

          int16_t erase_age;
          if (fs->max_erase_count > erase_count)
            {
              erase_age = fs->max_erase_count - erase_count;
            }
          else
            {
              erase_age =
                SPIFFS_OBJ_ID_FREE - (erase_count - fs->max_erase_count);
            }

          int32_t score =
            deleted_pages_in_block * SPIFFS_GC_HEUR_W_DELET +
            used_pages_in_block * SPIFFS_GC_HEUR_W_USED +
            erase_age * (fs_crammed ? 0 : SPIFFS_GC_HEUR_W_ERASE_AGE);
          int cand_ix = 0;
          spiffs_gcinfo("blkndx:" _SPIPRIbl " del:" _SPIPRIi " use:"
                        _SPIPRIi " score:" _SPIPRIi "\n", cur_block,
                        deleted_pages_in_block, used_pages_in_block, score);
          while (cand_ix < max_candidates)
            {
              if (cand_blocks[cand_ix] == (int16_t) - 1)
                {
                  cand_blocks[cand_ix] = cur_block;
                  cand_scores[cand_ix] = score;
                  break;
                }
              else if (cand_scores[cand_ix] < score)
                {
                  int reorder_cand_ix = max_candidates - 2;
                  while (reorder_cand_ix >= cand_ix)
                    {
                      cand_blocks[reorder_cand_ix + 1] =
                        cand_blocks[reorder_cand_ix];
                      cand_scores[reorder_cand_ix + 1] =
                        cand_scores[reorder_cand_ix];
                      reorder_cand_ix--;
                    }

                  cand_blocks[cand_ix] = cur_block;
                  cand_scores[cand_ix] = score;
                  break;
                }

              cand_ix++;
            }

          (*candidate_count)++;
        }

      cur_entry = 0;
      cur_block++;
      cur_block_addr += SPIFFS_CFG_LOG_BLOCK_SZ(fs);
    }

  return ret;
}

/* Empties given block by moving all data into free pages of another block
 * Strategy:
 *   loop:
 *   scan object lookup for object data pages
 *   for first found id, check spndx and load corresponding object index page to memory
 *   push object scan lookup entry index
 *     rescan object lookup, find data pages with same id and referenced by same object index
 *     move data page, update object index in memory
 *     when reached end of lookup, store updated object index
 *   pop object scan lookup entry index
 *   repeat loop until end of object lookup
 *   scan object lookup again for remaining object index pages, move to new page in other block
 */

int32_t spiffs_gc_clean(FAR struct spiffs_s *fs, int16_t blkndx)
{
  int32_t ret = OK;
  const int entries_per_page =
    (SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t));

  /* this is the global localizer being pushed and popped */

  int cur_entry = 0;
  int16_t *obj_lu_buf = (int16_t *) fs->lu_work;
  spiffs_gc gc;                 /* our stack frame/state */
  int16_t cur_pgndx = 0;
  FAR struct spiffs_pgobj_ixheader_s *objhdr =
    (FAR struct spiffs_pgobj_ixheader_s *) fs->work;
  spiffs_page_object_ix *objndx = (spiffs_page_object_ix *) fs->work;

  spiffs_gcinfo("Cleaning block " _SPIPRIbl "\n", blkndx);

  memset(&gc, 0, sizeof(spiffs_gc));
  gc.state = FIND_OBJ_DATA;

  if (fs->free_blkndx == blkndx)
    {
      /* move free cursor to next block, cannot use free pages from the block
       * we want to clean
       */

      fs->free_blkndx = (blkndx + 1) % fs->geo.neraseblocks;
      fs->free_entry = 0;
      spiffs_gcinfo("Move free cursor to block " _SPIPRIbl "\n",
                    fs->free_blkndx);
    }

  while (ret == OK && gc.state != FINISHED)
    {
      spiffs_gcinfo("gc_clean: state = " _SPIPRIi " entry:" _SPIPRIi "\n",
                    gc.state, cur_entry);
      gc.obj_id_found = 0;  /* reset (to no found data page) */

      /* scan through lookup pages */

      int obj_lookup_page = cur_entry / entries_per_page;
      uint8_t scan = 1;

      /* check each object lookup page */

      while (scan && ret == OK &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;
          ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                           0,
                           blkndx * SPIFFS_CFG_LOG_BLOCK_SZ(fs) +
                           SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);

          /* check each object lookup entry */

          while (scan && ret == OK &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry <
                 (int)(SPIFFS_PAGES_PER_BLOCK(fs) -
                       SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t id = obj_lu_buf[cur_entry - entry_offset];
              cur_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, blkndx, cur_entry);

              /* act upon object id depending on gc state */

              switch (gc.state)
                {
                case FIND_OBJ_DATA:
                  /* find a data page. */

                  if (id != SPIFFS_OBJ_ID_DELETED &&
                      id != SPIFFS_OBJ_ID_FREE &&
                      ((id & SPIFFS_OBJ_ID_IX_FLAG) == 0))
                    {
                      /* found a data page, stop scanning and handle in switch
                       * case below
                       */

                      spiffs_gcinfo("FIND_DATA state:" _SPIPRIi
                                    " - found obj id " _SPIPRIid "\n", gc.state,
                                    id);
                      gc.obj_id_found = 1;
                      gc.cur_obj_id = id;
                      gc.cur_data_pgndx = cur_pgndx;
                      scan = 0;
                    }
                  break;

                case MOVE_OBJ_DATA:
                  /* evacuate found data pages for corresponding object index
                   * we have in memory, update memory representation
                   */

                  if (id == gc.cur_obj_id)
                    {
                      struct spiffs_page_header_s p_hdr;
                      ret =
                        _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                   sizeof(struct spiffs_page_header_s),
                                   (uint8_t *) & p_hdr);
                      SPIFFS_CHECK_RES(ret);
                      spiffs_gcinfo("MOVE_DATA found data page "
                                    _SPIPRIid ":" _SPIPRIsp " @ " _SPIPRIpg
                                    "\n", gc.cur_obj_id, p_hdr.span_ix,
                                    cur_pgndx);
                      if (SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, p_hdr.span_ix) !=
                          gc.cur_objndx_spndx)
                        {
                          spiffs_gcinfo("MOVE_DATA no objndx spndx match, take in another run\n");
                        }
                      else
                        {
                          int16_t new_data_pgndx;
                          if (p_hdr.flags & SPIFFS_PH_FLAG_DELET)
                            {
                              /* move page */

                              ret =
                                spiffs_page_move(fs, 0, 0, id, &p_hdr,
                                                 cur_pgndx, &new_data_pgndx);
                              spiffs_gcinfo("MOVE_DATA move objndx "
                                            _SPIPRIid ":" _SPIPRIsp " page "
                                            _SPIPRIpg " to " _SPIPRIpg "\n",
                                            gc.cur_obj_id, p_hdr.span_ix,
                                            cur_pgndx, new_data_pgndx);
                              SPIFFS_CHECK_RES(ret);

                              /* move wipes obj_lu, reload it */

                              ret =
                                _spiffs_rd(fs,
                                           SPIFFS_OP_T_OBJ_LU |
                                           SPIFFS_OP_C_READ, 0,
                                           blkndx * SPIFFS_CFG_LOG_BLOCK_SZ(fs) +
                                           SPIFFS_PAGE_TO_PADDR(fs,
                                                                obj_lookup_page),
                                           SPIFFS_CFG_LOG_PAGE_SZ(fs),
                                           fs->lu_work);
                              SPIFFS_CHECK_RES(ret);
                            }
                          else
                            {
                              /* page is deleted but not deleted in lookup,
                               * scrap it - might seem unnecessary as we will
                               * erase this block, but we might get aborted
                               */

                              spiffs_gcinfo("MOVE_DATA wipe objndx "
                                            _SPIPRIid ":" _SPIPRIsp " page "
                                            _SPIPRIpg "\n", id,
                                            p_hdr.span_ix, cur_pgndx);
                              ret = spiffs_page_delete(fs, cur_pgndx);
                              SPIFFS_CHECK_RES(ret);
                              new_data_pgndx = SPIFFS_OBJ_ID_FREE;
                            }

                          /* update memory representation of object index page
                           * with new data page
                           */

                          if (gc.cur_objndx_spndx == 0)
                            {
                              /* update object index header page */

                              ((int16_t *) ((uint8_t *) objhdr +
                                                   sizeof(struct spiffs_pgobj_ixheader_s)))
                                [p_hdr.span_ix] = new_data_pgndx;
                              spiffs_gcinfo("MOVE_DATA wrote page "
                                            _SPIPRIpg " to objhdr entry "
                                            _SPIPRIsp " in mem\n", new_data_pgndx,
                                            (int16_t)
                                            SPIFFS_OBJ_IX_ENTRY(fs, p_hdr.span_ix));
                            }
                          else
                            {
                              /* update object index page */

                              ((int16_t *) ((uint8_t *) objndx +
                                                   sizeof(spiffs_page_object_ix)))
                                [SPIFFS_OBJ_IX_ENTRY(fs, p_hdr.span_ix)] =
                                new_data_pgndx;
                              spiffs_gcinfo("MOVE_DATA wrote page "
                                            _SPIPRIpg " to objndx entry "
                                            _SPIPRIsp " in mem\n", new_data_pgndx,
                                            (int16_t)
                                            SPIFFS_OBJ_IX_ENTRY(fs,
                                                                p_hdr.span_ix));
                            }
                        }
                    }
                  break;

                case MOVE_OBJ_IX:
                  /* find and evacuate object index pages */

                  if (id != SPIFFS_OBJ_ID_DELETED &&
                      id != SPIFFS_OBJ_ID_FREE &&
                      (id & SPIFFS_OBJ_ID_IX_FLAG))
                    {
                      /* found an index object id */

                      struct spiffs_page_header_s p_hdr;
                      int16_t new_pgndx;

                      /* load header */

                      ret =
                        _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                   sizeof(struct spiffs_page_header_s),
                                   (uint8_t *) & p_hdr);
                      SPIFFS_CHECK_RES(ret);
                      if (p_hdr.flags & SPIFFS_PH_FLAG_DELET)
                        {
                          /* move page */

                          ret =
                            spiffs_page_move(fs, 0, 0, id, &p_hdr, cur_pgndx,
                                             &new_pgndx);
                          spiffs_gcinfo("MOVE_OBJIX move objndx "
                                        _SPIPRIid ":" _SPIPRIsp " page "
                                        _SPIPRIpg " to " _SPIPRIpg "\n", id,
                                        p_hdr.span_ix, cur_pgndx, new_pgndx);
                          SPIFFS_CHECK_RES(ret);
                          spiffs_cb_object_event(fs,
                                                 (spiffs_page_object_ix *) &
                                                 p_hdr, SPIFFS_EV_IX_MOV,
                                                 id, p_hdr.span_ix, new_pgndx,
                                                 0);

                          /* move wipes obj_lu, reload it */

                          ret =
                            _spiffs_rd(fs,
                                       SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, 0,
                                       blkndx * SPIFFS_CFG_LOG_BLOCK_SZ(fs) +
                                       SPIFFS_PAGE_TO_PADDR(fs,
                                                            obj_lookup_page),
                                       SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
                          SPIFFS_CHECK_RES(ret);
                        }
                      else
                        {
                          /* page is deleted but not deleted in lookup, scrap
                           * it - might seem unnecessary as we will erase this
                           * block, but we might get aborted
                           */

                          spiffs_gcinfo("MOVE_OBJIX wipe objndx "
                                        _SPIPRIid ":" _SPIPRIsp " page "
                                        _SPIPRIpg "\n", id, p_hdr.span_ix,
                                        cur_pgndx);
                          ret = spiffs_page_delete(fs, cur_pgndx);
                          if (ret == OK)
                            {
                              spiffs_cb_object_event(fs,
                                                     (spiffs_page_object_ix *)
                                                     0, SPIFFS_EV_IX_DEL,
                                                     id, p_hdr.span_ix,
                                                     cur_pgndx, 0);
                            }
                        }

                      SPIFFS_CHECK_RES(ret);
                    }
                  break;

                default:
                  scan = 0;
                  break;
                }

              cur_entry++;
            }

          obj_lookup_page++;    /* no need to check scan variable here,
                                 * obj_lookup_page is set in start of loop
                                 */
        }
        

      if (ret != OK)
        {
          break;
        }

      /* state finalization and switch */

      switch (gc.state)
        {
        case FIND_OBJ_DATA:
          if (gc.obj_id_found)
            {
              /* handle found data page -
               * find out corresponding obj ix page and load it to memory
               */

              struct spiffs_page_header_s p_hdr;
              int16_t objndx_pgndx;
              gc.stored_scan_entry_index = cur_entry;   /* push cursor */
              cur_entry = 0;    /* restart scan from start */
              gc.state = MOVE_OBJ_DATA;
              ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                               0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                               sizeof(struct spiffs_page_header_s), (uint8_t *) & p_hdr);
              SPIFFS_CHECK_RES(ret);
              gc.cur_objndx_spndx =
                SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, p_hdr.span_ix);
              spiffs_gcinfo("FIND_DATA find objndx span_ix:" _SPIPRIsp
                            "\n", gc.cur_objndx_spndx);
              ret =
                spiffs_obj_lu_find_id_and_span(fs,
                                               gc.
                                               cur_obj_id |
                                               SPIFFS_OBJ_ID_IX_FLAG,
                                               gc.cur_objndx_spndx, 0,
                                               &objndx_pgndx);
              if (ret == -ENOENT)
                {
                  /* on borked systems we might get an ERR_NOT_FOUND here -
                   * this is handled by simply deleting the page as it is not
                   * referenced from anywhere
                   */

                  spiffs_gcinfo("FIND_OBJ_DATA objndx not found! Wipe page "
                     _SPIPRIpg "\n", gc.cur_data_pgndx);
                  ret = spiffs_page_delete(fs, gc.cur_data_pgndx);
                  SPIFFS_CHECK_RES(ret);

                  /* then we restore states and continue scanning for data
                   * pages
                   */

                  cur_entry = gc.stored_scan_entry_index;       /* pop cursor */
                  gc.state = FIND_OBJ_DATA;
                  break;        /* done */
                }
              SPIFFS_CHECK_RES(ret);
              spiffs_gcinfo("FIND_DATA found object index at page "
                            _SPIPRIpg "\n", objndx_pgndx);
              ret =
                _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ, 0,
                           SPIFFS_PAGE_TO_PADDR(fs, objndx_pgndx),
                           SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->work);
              SPIFFS_CHECK_RES(ret);

              /* cannot allow a gc if the presumed index in fact is no index, a
               * check must run or lot of data may be lost
               */

              SPIFFS_VALIDATE_OBJIX(objndx->p_hdr,
                                    gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                    gc.cur_objndx_spndx);
              gc.cur_objndx_pgndx = objndx_pgndx;
            }
          else
            {
              /* No more data pages found, passed thru all block, start
               * evacuating object indices
               */

              gc.state = MOVE_OBJ_IX;
              cur_entry = 0;    /* Restart entry scan index */
            }
          break;

        case MOVE_OBJ_DATA:
          {
            /* Store modified objndx (hdr) page residing in memory now that all
             * data pages belonging to this object index and residing in the
             * block we want to evacuate
             */

            int16_t new_objndx_pgndx;
            gc.state = FIND_OBJ_DATA;
            cur_entry = gc.stored_scan_entry_index;     /* pop cursor */
            if (gc.cur_objndx_spndx == 0)
              {
                /* store object index header page */

                ret =
                  spiffs_object_update_index_hdr(fs, 0,
                                                 gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                                 gc.cur_objndx_pgndx, fs->work, 0,
                                                 0, &new_objndx_pgndx);
                spiffs_gcinfo("MOVE_DATA store modified objhdr page, "
                   _SPIPRIpg ":" _SPIPRIsp "\n", new_objndx_pgndx, 0);
                SPIFFS_CHECK_RES(ret);
              }
            else
              {
                /* store object index page */

                ret =
                  spiffs_page_move(fs, 0, fs->work,
                                   gc.cur_obj_id | SPIFFS_OBJ_ID_IX_FLAG, 0,
                                   gc.cur_objndx_pgndx, &new_objndx_pgndx);
                spiffs_gcinfo("MOVE_DATA store modified objndx page, "
                              _SPIPRIpg ":" _SPIPRIsp "\n", new_objndx_pgndx,
                              objndx->p_hdr.span_ix);
                SPIFFS_CHECK_RES(ret);
                spiffs_cb_object_event(fs, (spiffs_page_object_ix *) fs->work,
                                       SPIFFS_EV_IX_UPD, gc.cur_obj_id,
                                       objndx->p_hdr.span_ix, new_objndx_pgndx, 0);
              }
          }
          break;

        case MOVE_OBJ_IX:
          /* scanned thru all block, no more object indices found - our work
           * here is done
           */

          gc.state = FINISHED;
          break;

        default:
          cur_entry = 0;
          break;
        }

      spiffs_gcinfo("state-> " _SPIPRIi "\n", gc.state);
    }

  return ret;
}
