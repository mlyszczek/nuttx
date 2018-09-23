/****************************************************************************
 * fs/spiffs.h/spiffs_check.c
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

/* Contains functionality for checking file system consistency
 * and mending problems.
 * Three levels of consistency checks are implemented:
 *
 * Look up consistency
 *   Checks if indices in lookup pages are coherent with page headers
 * Object index consistency
 *   Checks if there are any orphaned object indices (missing object index
 *     headers).
 *   If an object index is found but not its header, the object index is
 *     deleted.
 *   This is critical for the following page consistency check.
 * Page consistency
 *   Checks for pages that ought to be indexed, ought not to be indexed, are
 *     multiple indexed
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "spiffs.h"
#include "spiffs_nucleus.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Look up consistency */

/* searches in the object indices and returns the referenced page index given
 * the object ID and the data span index
 * destroys fs->lu_work
 */

static int32_t spiffs_object_get_data_page_index_reference(FAR struct spiffs_s *fs,
                                                           int16_t objid,
                                                           int16_t
                                                           data_spndx,
                                                           int16_t * pgndx,
                                                           int16_t *
                                                           objndx_pgndx)
{
  uint32_t addr;
  int32_t ret;

  /* calculate object index span index for given data page span index */

  int16_t objndx_spndx = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spndx);

  /* find obj index for obj ID and span index */

  ret = spiffs_obj_lu_find_id_and_span(fs, objid | SPIFFS_OBJ_ID_IX_FLAG,
                                       objndx_spndx, 0, objndx_pgndx);
  SPIFFS_CHECK_RES(ret);

  /* load obj index entry */

  addr = SPIFFS_PAGE_TO_PADDR(fs, *objndx_pgndx);
  if (objndx_spndx == 0)
    {
      /* get referenced page from object index header */

      addr += sizeof(struct spiffs_pgobj_ixheader_s) +
              data_spndx * sizeof(int16_t);
    }
  else
    {
      /* get referenced page from object index */

      addr += sizeof(spiffs_page_object_ix) +
              SPIFFS_OBJ_IX_ENTRY(fs, data_spndx) *
              sizeof(int16_t);
    }

  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ, 0, addr,
                   sizeof(int16_t), (uint8_t *) pgndx);

  return ret;
}

/* copies page contents to a new page */

static int32_t spiffs_rewrite_page(FAR struct spiffs_s *fs, int16_t cur_pgndx,
                                   struct spiffs_page_header_s * p_hdr,
                                   int16_t * new_pgndx)
{
  int32_t ret;

  ret = spiffs_page_allocate_data(fs, p_hdr->objid, p_hdr, 0, 0, 0, 0,
                                  new_pgndx);
  SPIFFS_CHECK_RES(ret);
  ret = spiffs_phys_cpy(fs, 0,
                        SPIFFS_PAGE_TO_PADDR(fs, *new_pgndx) +
                        sizeof(struct spiffs_page_header_s),
                        SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                        sizeof(struct spiffs_page_header_s),
                        SPIFFS_DATA_PAGE_SIZE(fs));
  SPIFFS_CHECK_RES(ret);
  return ret;
}

/* rewrites the object index for given object ID and replaces the
 * data page index to a new page index
 */

static int32_t spiffs_rewrite_index(FAR struct spiffs_s *fs, int16_t objid,
                                   int16_t data_spndx,
                                   int16_t new_data_pgndx,
                                   int16_t objndx_pgndx)
{
  int32_t ret;
  int16_t blkndx;
  int entry;
  int16_t free_pgndx;

  objid |= SPIFFS_OBJ_ID_IX_FLAG;

  /* find free entry */

  ret = spiffs_obj_lu_find_free(fs, fs->free_blkndx,
                                fs->free_entry, &blkndx, &entry);
  SPIFFS_CHECK_RES(ret);
  free_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, blkndx, entry);

  /* calculate object index span index for given data page span index */

  int16_t objndx_spndx = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spndx);
  if (objndx_spndx == 0)
    {
      /* calc index in index header */

      entry = data_spndx;
    }
  else
    {
      /* calc entry in index */

      entry = SPIFFS_OBJ_IX_ENTRY(fs, data_spndx);
    }

  /* load index */

  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   0, SPIFFS_PAGE_TO_PADDR(fs, objndx_pgndx),
                   SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
  SPIFFS_CHECK_RES(ret);
  struct spiffs_page_header_s *objndx_p_hdr = (struct spiffs_page_header_s *) fs->lu_work;

  /* be ultra safe, double check header against provided data */

  if (objndx_p_hdr->objid != objid)
    {
      spiffs_page_delete(fs, free_pgndx);
      return SPIFFS_ERR_CHECK_OBJ_ID_MISM;
    }

  if (objndx_p_hdr->span_ix != objndx_spndx)
    {
      spiffs_page_delete(fs, free_pgndx);
      return SPIFFS_ERR_CHECK_SPIX_MISM;
    }

  if ((objndx_p_hdr->flags & (SPIFFS_PH_FLAG_USED | SPIFFS_PH_FLAG_IXDELE |
                             SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                             SPIFFS_PH_FLAG_DELET)) !=
      (SPIFFS_PH_FLAG_IXDELE | SPIFFS_PH_FLAG_DELET))
    {
      spiffs_page_delete(fs, free_pgndx);
      return SPIFFS_ERR_CHECK_FLAGS_BAD;
    }

  /* rewrite in mem */

  if (objndx_spndx == 0)
    {
      ((int16_t *) ((uint8_t *) fs->lu_work +
                           sizeof(struct spiffs_pgobj_ixheader_s)))[data_spndx] =
        new_data_pgndx;
    }
  else
    {
      ((int16_t *) ((uint8_t *) fs->lu_work +
                           sizeof(spiffs_page_object_ix)))[SPIFFS_OBJ_IX_ENTRY
                                                           (fs, data_spndx)] =
        new_data_pgndx;
    }

  ret = spiffs_phys_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                   0, SPIFFS_PAGE_TO_PADDR(fs, free_pgndx),
                   SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
  SPIFFS_CHECK_RES(ret);
  ret = spiffs_phys_wr(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT,
                   0, SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs,free_pgndx)) +
                   SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, free_pgndx) *
                   sizeof(int16_t), sizeof(int16_t),
                   (FAR uint8_t *)&objid);
  SPIFFS_CHECK_RES(ret);
  ret = spiffs_page_delete(fs, objndx_pgndx);

  return ret;
}

/* deletes an object just by marking object index header as deleted */

static int32_t spiffs_delete_obj_lazy(FAR struct spiffs_s *fs, int16_t objid)
{
  int16_t objhdr_pgndx;
  uint8_t flags = 0xff;
  int32_t ret;

  ret = spiffs_obj_lu_find_id_and_span(fs, objid, 0, 0, &objhdr_pgndx);
  if (ret == -ENOENT)
    {
      return OK;
    }

  SPIFFS_CHECK_RES(ret);
#if SPIFFS_NO_BLIND_WRITES
  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, 0,
                   SPIFFS_PAGE_TO_PADDR(fs, objhdr_pgndx) +
                   offsetof(struct spiffs_page_header_s, flags), sizeof(flags),
                   &flags);
  SPIFFS_CHECK_RES(ret);
#endif
  flags &= ~SPIFFS_PH_FLAG_IXDELE;
  ret = spiffs_phys_wr(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT, 0,
                   SPIFFS_PAGE_TO_PADDR(fs, objhdr_pgndx) +
                   offsetof(struct spiffs_page_header_s, flags), sizeof(flags),
                   &flags);
  return ret;
}

/* validates the given look up entry */

static int32_t spiffs_lookup_check_validate(FAR struct spiffs_s *fs, int16_t lu_obj_id,
                                            struct spiffs_page_header_s * p_hdr,
                                            int16_t cur_pgndx,
                                            int16_t cur_block,
                                            int cur_entry, int *reload_lu)
{
  uint8_t delete_page = 0;
  int32_t ret = OK;
  int16_t objndx_pgndx;
  int16_t ref_pgndx;

  (void)cur_block;
  (void)cur_entry;

  /* check validity, take actions */

  if (((lu_obj_id == SPIFFS_OBJ_ID_DELETED) &&
       (p_hdr->flags & SPIFFS_PH_FLAG_DELET)) ||
      ((lu_obj_id == SPIFFS_OBJ_ID_FREE) &&
       (p_hdr->flags & SPIFFS_PH_FLAG_USED) == 0))
    {
      /* look up entry deleted / free but used in page header */

      spiffs_checkinfo("LU: pgndx %04x"
                       " deleted/free in lu but not on page\n", cur_pgndx);
      *reload_lu = 1;
      delete_page = 1;
      if (p_hdr->flags & SPIFFS_PH_FLAG_INDEX)
        {
          /* header says data page
           * data page can be removed if not referenced by some object index
           */

          ret = spiffs_object_get_data_page_index_reference(fs, p_hdr->objid,
                                                            p_hdr->span_ix,
                                                            &ref_pgndx, &objndx_pgndx);
          if (ret == -ENOENT)
            {
              /* No object with this objid, so remove page safely */

              ret = OK;
            }
          else
            {
              SPIFFS_CHECK_RES(ret);
              if (ref_pgndx == cur_pgndx)
                {
                  /* data page referenced by object index but deleted in lu
                   * copy page to new place and re-write the object index to
                   * new place
                   */

                  int16_t new_pgndx;
                  ret = spiffs_rewrite_page(fs, cur_pgndx, p_hdr, &new_pgndx);

                  spiffs_checkinfo
                    ("LU: FIXUP: data page not found elsewhere, rewriting "
                     "%04x to new page %04x\n", cur_pgndx,
                     new_pgndx);
                  SPIFFS_CHECK_RES(ret);

                  *reload_lu = 1;

                  spiffs_checkinfo("LU: FIXUP: %04x rewritten to "
                                   "%04x, affected objndx_pgndx %04x"
                                   "\n", cur_pgndx, new_pgndx, objndx_pgndx);
                  ret = spiffs_rewrite_index(fs, p_hdr->objid, p_hdr->span_ix,
                                             new_pgndx, objndx_pgndx);
                  if (ret <= _SPIFFS_ERR_CHECK_FIRST &&
                      ret > _SPIFFS_ERR_CHECK_LAST)
                    {
                      /* index bad also, cannot mend this file */

                      spiffs_checkinfo("LU: FIXUP: index bad %d"
                                       ", cannot mend!\n", ret);
                      ret = spiffs_page_delete(fs, new_pgndx);
                      SPIFFS_CHECK_RES(ret);
                      ret = spiffs_delete_obj_lazy(fs, p_hdr->objid);
                    }

                  SPIFFS_CHECK_RES(ret);
                }
            }
        }
      else
        {
          /* header says index page
           * index page can be removed if other index with same objid and
           * span index is found
           */

          ret = spiffs_obj_lu_find_id_and_span(fs,
                                               p_hdr->objid | SPIFFS_OBJ_ID_IX_FLAG,
                                               p_hdr->span_ix, cur_pgndx, 0);
          if (ret == -ENOENT)
            {
              /* no such index page found, check for a data page amongst page
               * headers.  lu cannot be trusted
               */

              ret = spiffs_obj_lu_find_id_and_span_byphdr(fs,
                                                          p_hdr->objid | SPIFFS_OBJ_ID_IX_FLAG,
                                                          0, 0, 0);
              if (ret == OK)
                {
                  /* ignore other errors
                   * got a data page also, assume lu corruption only, rewrite
                   * to new page
                   */

                  int16_t new_pgndx;
                  ret = spiffs_rewrite_page(fs, cur_pgndx, p_hdr, &new_pgndx);
                  spiffs_checkinfo
                    ("LU: FIXUP: ix page with data not found elsewhere, rewriting "
                     "%04x to new page %04x\n", cur_pgndx,
                     new_pgndx);
                  SPIFFS_CHECK_RES(ret);
                  *reload_lu = 1;
                }
            }
          else
            {
              SPIFFS_CHECK_RES(ret);
            }
        }
    }

  if (lu_obj_id != SPIFFS_OBJ_ID_FREE && lu_obj_id != SPIFFS_OBJ_ID_DELETED)
    {
      /* look up entry used */

      if ((p_hdr->objid | SPIFFS_OBJ_ID_IX_FLAG) !=
          (lu_obj_id | SPIFFS_OBJ_ID_IX_FLAG))
        {
          spiffs_checkinfo("LU: pgndx %04x differ in objid lu="
                           "%04x ph:%04x\n", cur_pgndx, lu_obj_id,
                           p_hdr->objid);
          delete_page = 1;
          if ((p_hdr->flags & SPIFFS_PH_FLAG_DELET) == 0 ||
              (p_hdr->flags & SPIFFS_PH_FLAG_FINAL) ||
              (p_hdr->flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_IXDELE)) ==
              0)
            {
              /* page deleted or not finalized, just remove it */
            }
          else
            {
              if (p_hdr->flags & SPIFFS_PH_FLAG_INDEX)
                {
                  /* if data page, check for reference to this page */

                  ret = spiffs_object_get_data_page_index_reference(fs,
                                                                    p_hdr->objid,
                                                                    p_hdr->span_ix,
                                                                    &ref_pgndx,
                                                                    &objndx_pgndx);
                  if (ret == -ENOENT)
                    {
                      /* no object with this objid, so remove page safely */

                      ret = OK;
                    }
                  else
                    {
                      SPIFFS_CHECK_RES(ret);

                      /* if found, rewrite page with object ID, update index,
                       * and delete current
                       */

                      if (ref_pgndx == cur_pgndx)
                        {
                          int16_t new_pgndx;
                          ret = spiffs_rewrite_page(fs, cur_pgndx, p_hdr, &new_pgndx);
                          SPIFFS_CHECK_RES(ret);
                          ret = spiffs_rewrite_index(fs, p_hdr->objid,
                                                     p_hdr->span_ix, new_pgndx,
                                                     objndx_pgndx);
                          if (ret <= _SPIFFS_ERR_CHECK_FIRST &&
                              ret > _SPIFFS_ERR_CHECK_LAST)
                            {
                              /* index bad also, cannot mend this file */

                              spiffs_checkinfo("LU: FIXUP: index bad %d"
                                               ", cannot mend!\n", ret);
                              ret = spiffs_page_delete(fs, new_pgndx);
                              SPIFFS_CHECK_RES(ret);
                              ret = spiffs_delete_obj_lazy(fs, p_hdr->objid);
                              *reload_lu = 1;
                            }

                          SPIFFS_CHECK_RES(ret);
                        }
                    }
                }
              else
                {
                  /* else if index, check for other pages with both ID's
                   * and span index
                   */

                  int16_t objndx_pgndx_lu, objndx_pgndx_ph;

                  /* see if other object index page exists for lookup objid
                   * and span index
                   */

                  ret = spiffs_obj_lu_find_id_and_span(fs,
                                                       lu_obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                                       p_hdr->span_ix, 0,
                                                       &objndx_pgndx_lu);
                  if (ret == -ENOENT)
                    {
                      ret = OK;
                      objndx_pgndx_lu = 0;
                    }

                  SPIFFS_CHECK_RES(ret);

                  /* see if other object index exists for page header objid
                   * and span index
                   */

                  ret = spiffs_obj_lu_find_id_and_span(fs,
                                                       p_hdr->objid | SPIFFS_OBJ_ID_IX_FLAG,
                                                       p_hdr->span_ix, 0,
                                                       &objndx_pgndx_ph);
                  if (ret == -ENOENT)
                    {
                      ret = OK;
                      objndx_pgndx_ph = 0;
                    }

                  SPIFFS_CHECK_RES(ret);

                  /* if both ID's found, just delete current */

                  if (objndx_pgndx_ph == 0 || objndx_pgndx_lu == 0)
                    {
                      /* otherwise try finding first corresponding data pages */

                      int16_t data_pgndx_lu, data_pgndx_ph;

                      /* see if other data page exists for look up objid and
                       * span index
                       */

                      ret = spiffs_obj_lu_find_id_and_span(fs,
                                                           lu_obj_id & ~SPIFFS_OBJ_ID_IX_FLAG,
                                                           0, 0, &data_pgndx_lu);
                      if (ret == -ENOENT)
                        {
                          ret = OK;
                          objndx_pgndx_lu = 0;
                        }

                      SPIFFS_CHECK_RES(ret);

                      /* see if other data page exists for page header objid
                       * and span index
                       */

                      ret = spiffs_obj_lu_find_id_and_span(fs,
                                                           p_hdr->objid & ~SPIFFS_OBJ_ID_IX_FLAG,
                                                           0, 0, &data_pgndx_ph);
                      if (ret == -ENOENT)
                        {
                          ret = OK;
                          objndx_pgndx_ph = 0;
                        }

                      SPIFFS_CHECK_RES(ret);

                      struct spiffs_page_header_s new_ph;
                      new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                              SPIFFS_PH_FLAG_INDEX |
                                              SPIFFS_PH_FLAG_FINAL);
                      new_ph.span_ix = p_hdr->span_ix;
                      int16_t new_pgndx;

                      if ((objndx_pgndx_lu && data_pgndx_lu && data_pgndx_ph && objndx_pgndx_ph == 0) ||
                          (objndx_pgndx_lu == 0 && data_pgndx_ph && objndx_pgndx_ph == 0))
                        {
                          /* got a data page for page header objid
                           * rewrite as obj_id_ph
                           */

                          new_ph.objid = p_hdr->objid | SPIFFS_OBJ_ID_IX_FLAG;
                          ret = spiffs_rewrite_page(fs, cur_pgndx, &new_ph, &new_pgndx);
                          spiffs_checkinfo("LU: FIXUP: rewrite page %04x"
                                           " as %04x to pgndx %04x"
                                           "\n", cur_pgndx, new_ph.objid,
                                           new_pgndx);

                          SPIFFS_CHECK_RES(ret);
                          *reload_lu = 1;
                        }
                      else
                        if ((objndx_pgndx_ph && data_pgndx_ph && data_pgndx_lu && objndx_pgndx_lu == 0) ||
                            (objndx_pgndx_ph == 0 && data_pgndx_lu && objndx_pgndx_lu == 0))
                        {
                          /* got a data page for look up objid
                           * rewrite as obj_id_lu
                           */

                          new_ph.objid = lu_obj_id | SPIFFS_OBJ_ID_IX_FLAG;
                          spiffs_checkinfo("LU: FIXUP: rewrite page %04x"
                                           " as %04x\n", cur_pgndx,
                                           new_ph.objid);

                          ret = spiffs_rewrite_page(fs, cur_pgndx, &new_ph, &new_pgndx);
                          SPIFFS_CHECK_RES(ret);
                          *reload_lu = 1;
                        }
                      else
                        {
                          /* cannot safely do anything */

                          spiffs_checkinfo
                            ("LU: FIXUP: nothing to do, just delete\n");
                        }
                    }
                }
            }
        }
      else
        if (((lu_obj_id & SPIFFS_OBJ_ID_IX_FLAG) &&
             (p_hdr->flags & SPIFFS_PH_FLAG_INDEX)) ||
            ((lu_obj_id & SPIFFS_OBJ_ID_IX_FLAG) == 0 &&
             (p_hdr->flags & SPIFFS_PH_FLAG_INDEX) == 0))
        {
          spiffs_checkinfo("LU: %04x lu/page index marking differ\n",
                           cur_pgndx);
          int16_t data_pgndx, objndx_pgndx_d;

          /* see if other data page exists for given objid and span index */

          ret = spiffs_obj_lu_find_id_and_span(fs,
                                               lu_obj_id & ~SPIFFS_OBJ_ID_IX_FLAG,
                                               p_hdr->span_ix, cur_pgndx, &data_pgndx);
          if (ret == -ENOENT)
            {
              ret = OK;
              data_pgndx = 0;
            }

          SPIFFS_CHECK_RES(ret);

          /* see if other object index exists for given objid and span index */

          ret = spiffs_obj_lu_find_id_and_span(fs,
                                               lu_obj_id | SPIFFS_OBJ_ID_IX_FLAG,
                                               p_hdr->span_ix, cur_pgndx,
                                               &objndx_pgndx_d);
          if (ret == -ENOENT)
            {
              ret = OK;
              objndx_pgndx_d = 0;
            }

          SPIFFS_CHECK_RES(ret);

          delete_page = 1;

          /* if other data page exists and object index exists, just delete
           * page
           */

          if (data_pgndx && objndx_pgndx_d)
            {
              spiffs_checkinfo
                ("LU: FIXUP: other index and data page exists, simply remove\n");
            }

          /* if only data page exists, make this page index */

          else if (data_pgndx && objndx_pgndx_d == 0)
            {
              spiffs_checkinfo
                ("LU: FIXUP: other data page exists, make this index\n");

              struct spiffs_page_header_s new_ph;
              int16_t new_pgndx;
              new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                      SPIFFS_PH_FLAG_FINAL |
                                      SPIFFS_PH_FLAG_INDEX);
              new_ph.objid = lu_obj_id | SPIFFS_OBJ_ID_IX_FLAG;
              new_ph.span_ix = p_hdr->span_ix;
              ret = spiffs_page_allocate_data(fs, new_ph.objid, &new_ph,
                                              0, 0, 0, 1, &new_pgndx);
              SPIFFS_CHECK_RES(ret);
              ret = spiffs_phys_cpy(fs, 0, SPIFFS_PAGE_TO_PADDR(fs, new_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_CFG_LOG_PAGE_SZ(fs) -
                                    sizeof(struct spiffs_page_header_s));
              SPIFFS_CHECK_RES(ret);
            }

          /* if only index exists, make data page */

          else if (data_pgndx == 0 && objndx_pgndx_d)
            {
              spiffs_checkinfo
                ("LU: FIXUP: other index page exists, make this data\n");

              struct spiffs_page_header_s new_ph;
              int16_t new_pgndx;
              new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED | SPIFFS_PH_FLAG_FINAL);
              new_ph.objid = lu_obj_id & ~SPIFFS_OBJ_ID_IX_FLAG;
              new_ph.span_ix = p_hdr->span_ix;
              ret = spiffs_page_allocate_data(fs, new_ph.objid, &new_ph,
                                              0, 0, 0, 1, &new_pgndx);
              SPIFFS_CHECK_RES(ret);
              ret = spiffs_phys_cpy(fs, 0, SPIFFS_PAGE_TO_PADDR(fs, new_pgndx) +
                                sizeof(struct spiffs_page_header_s),
                                SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                sizeof(struct spiffs_page_header_s),
                                SPIFFS_CFG_LOG_PAGE_SZ(fs) -
                                sizeof(struct spiffs_page_header_s));
              SPIFFS_CHECK_RES(ret);
            }
          else
            {
              /* if nothing exists, we cannot safely make a decision - delete */
            }
        }
      else if ((p_hdr->flags & SPIFFS_PH_FLAG_DELET) == 0)
        {
          spiffs_checkinfo("LU: pgndx %04x"
                           " busy in lu but deleted on page\n", cur_pgndx);
          delete_page = 1;
        }
      else if ((p_hdr->flags & SPIFFS_PH_FLAG_FINAL))
        {
          spiffs_checkinfo("LU: pgndx %04x busy but not final\n",
                           cur_pgndx);

          /* page can be removed if not referenced by object index */

          *reload_lu = 1;
          ret = spiffs_object_get_data_page_index_reference(fs, lu_obj_id,
                                                            p_hdr->span_ix,
                                                            &ref_pgndx, &objndx_pgndx);
          if (ret == -ENOENT)
            {
              /* no object with this ID, so remove page safely */

              ret = OK;
              delete_page = 1;
            }
          else
            {
              SPIFFS_CHECK_RES(ret);
              if (ref_pgndx != cur_pgndx)
                {
                  spiffs_checkinfo
                    ("LU: FIXUP: other finalized page is referred, just delete\n");
                  delete_page = 1;
                }
              else
                {
                  uint8_t flags = 0xff;

                  /* page referenced by object index but not final
                   * just finalize
                   */

                  spiffs_checkinfo
                    ("LU: FIXUP: unfinalized page is referred, finalizing\n");

#if SPIFFS_NO_BLIND_WRITES
                  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                   offsetof(struct spiffs_page_header_s, flags),
                                   sizeof(flags), &flags);
                  SPIFFS_CHECK_RES(ret);
#endif
                  flags &= ~SPIFFS_PH_FLAG_FINAL;
                  ret = spiffs_phys_wr(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx) +
                                   offsetof(struct spiffs_page_header_s, flags),
                                   sizeof(flags), &flags);
                }
            }
        }
    }

  if (delete_page)
    {
      spiffs_checkinfo("LU: FIXUP: deleting page %04x\n", cur_pgndx);
      ret = spiffs_page_delete(fs, cur_pgndx);
      SPIFFS_CHECK_RES(ret);
    }

  return ret;
}

static int32_t spiffs_lookup_check_v(FAR struct spiffs_s *fs, int16_t objid,
                                     int16_t cur_block, int cur_entry,
                                     const void *user_const_p, void *user_var_p)
{
  (void)user_const_p;
  struct spiffs_page_header_s p_hdr;
  int16_t cur_pgndx =
    SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, cur_block, cur_entry);

  (void)user_var_p;
  int32_t ret = OK;

  /* load header */

  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                   sizeof(struct spiffs_page_header_s), (uint8_t *) & p_hdr);
  SPIFFS_CHECK_RES(ret);

  int reload_lu = 0;

  ret = spiffs_lookup_check_validate(fs, objid, &p_hdr, cur_pgndx, cur_block,
                                     cur_entry, &reload_lu);
  SPIFFS_CHECK_RES(ret);

  if (ret == OK)
    {
      return reload_lu ? SPIFFS_VIS_COUNTINUE_RELOAD : SPIFFS_VIS_COUNTINUE;
    }

  return ret;
}

/* Scans all object look up. For each entry, corresponding page header is checked for validity.
 * If an object index header page is found, this is also checked
 */

int32_t spiffs_lookup_consistency_check(FAR struct spiffs_s *fs, uint8_t check_all_objects)
{
  int32_t ret = OK;

  (void)check_all_objects;

  ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0, spiffs_lookup_check_v,
                                         0, 0, 0, 0);

  if (ret == SPIFFS_VIS_END)
    {
      ret = OK;
    }

  return ret;
}

/* Page consistency
 *
 * Scans all pages (except lu pages), reserves 4 bits in working memory for each page
 * bit 0: 0 == FREE|DELETED, 1 == USED
 * bit 1: 0 == UNREFERENCED, 1 == REFERENCED
 * bit 2: 0 == NOT_INDEX,    1 == INDEX
 * bit 3: unused
 * A consistent file system will have only pages being
 *  - x000 free, unreferenced, not index
 *  - x011 used, referenced only once, not index
 *  - x101 used, unreferenced, index
 * The working memory might not fit all pages so several scans might be needed
 */

static int32_t spiffs_page_consistency_check_i(FAR struct spiffs_s *fs)
{
  const uint32_t bits = 4;
  const int16_t pages_per_scan = SPIFFS_CFG_LOG_PAGE_SZ(fs) * 8 / bits;

  int32_t ret = OK;
  int16_t pgndx_offset = 0;

  /* for each range of pages fitting into work memory */

  while (pgndx_offset < SPIFFS_PAGES_PER_BLOCK(fs) * fs->geo.neraseblocks)
    {
      /* set this flag to abort all checks and rescan the page range */

      uint8_t restart = 0;
      memset(fs->work, 0, SPIFFS_CFG_LOG_PAGE_SZ(fs));

      int16_t cur_block = 0;

      /* build consistency bitmap for ID range traversing all blocks */

      while (!restart && cur_block < fs->geo.neraseblocks)
        {
          /* Traverse each page except for lookup pages */

          int16_t cur_pgndx =
            SPIFFS_OBJ_LOOKUP_PAGES(fs) +
            SPIFFS_PAGES_PER_BLOCK(fs) * cur_block;
          while (!restart &&
                 cur_pgndx < SPIFFS_PAGES_PER_BLOCK(fs) * (cur_block + 1))
            {
              uint8_t within_range;

              /* read header */

              struct spiffs_page_header_s p_hdr;
              ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                               0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                               sizeof(struct spiffs_page_header_s), (uint8_t *) & p_hdr);
              SPIFFS_CHECK_RES(ret);

              within_range = (cur_pgndx >= pgndx_offset &&
                              cur_pgndx < pgndx_offset + pages_per_scan);
              const uint32_t pgndx_byte_ix = (cur_pgndx - pgndx_offset) / (8 / bits);
              const uint8_t pgndx_bit_ix = (cur_pgndx & ((8 / bits) - 1)) * bits;

              if (within_range &&
                  (p_hdr.flags & SPIFFS_PH_FLAG_DELET) &&
                  (p_hdr.flags & SPIFFS_PH_FLAG_USED) == 0)
                {
                  /* used */

                  fs->work[pgndx_byte_ix] |= (1 << (pgndx_bit_ix + 0));
                }

              if ((p_hdr.flags & SPIFFS_PH_FLAG_DELET) &&
                  (p_hdr.flags & SPIFFS_PH_FLAG_IXDELE) &&
                  (p_hdr.flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_USED)) == 0)
                {
                  int entries;
                  int i;

                  /* found non-deleted index */

                  if (within_range)
                    {
                      fs->work[pgndx_byte_ix] |= (1 << (pgndx_bit_ix + 2));
                    }

                  /* load non-deleted index */

                  ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                   SPIFFS_CFG_LOG_PAGE_SZ(fs), fs->lu_work);
                  SPIFFS_CHECK_RES(ret);

                  /* traverse index for referenced pages */

                  int16_t *object_page_index;
                  struct spiffs_page_header_s *objndx_p_hdr =
                    (struct spiffs_page_header_s *) fs->lu_work;

                  int16_t data_spndx_offset;
                  if (p_hdr.span_ix == 0)
                    {
                      /* object header page index */

                      entries = SPIFFS_OBJ_HDR_IX_LEN(fs);
                      data_spndx_offset = 0;
                      object_page_index =
                        (int16_t *) ((uint8_t *) fs->lu_work +
                                            sizeof(struct spiffs_pgobj_ixheader_s));
                    }
                  else
                    {
                      /* object page index */

                      entries = SPIFFS_OBJ_IX_LEN(fs);
                      data_spndx_offset =
                        SPIFFS_OBJ_HDR_IX_LEN(fs) +
                        SPIFFS_OBJ_IX_LEN(fs) * (p_hdr.span_ix - 1);
                      object_page_index =
                        (int16_t *) ((uint8_t *) fs->lu_work +
                                            sizeof(spiffs_page_object_ix));
                    }

                  /* for all entries in index */

                  for (i = 0; !restart && i < entries; i++)
                    {
                      int16_t rpgndx = object_page_index[i];
                      uint8_t rpgndx_within_range = rpgndx >= pgndx_offset &&
                        rpgndx < pgndx_offset + pages_per_scan;

                      if ((rpgndx != (int16_t) - 1 &&
                           rpgndx > SPIFFS_MAX_PAGES(fs)) || (rpgndx_within_range &&
                                                            SPIFFS_IS_LOOKUP_PAGE
                                                            (fs, rpgndx)))
                        {

                          /* bad reference */

                          spiffs_checkinfo("PA: pgndx %04x"
                                           "x bad pgndx / LU referenced from page "
                                           "%04x\n", rpgndx, cur_pgndx);

                          /* check for data page elsewhere */

                          int16_t data_pgndx;
                          ret = spiffs_obj_lu_find_id_and_span(fs,
                                                               objndx_p_hdr->objid & ~SPIFFS_OBJ_ID_IX_FLAG,
                                                               data_spndx_offset + i,
                                                               0, &data_pgndx);
                          if (ret == -ENOENT)
                            {
                              ret = OK;
                              data_pgndx = 0;
                            }

                          SPIFFS_CHECK_RES(ret);

                          if (data_pgndx == 0)
                            {
                              /* if not, allocate free page */

                              struct spiffs_page_header_s new_ph;
                              new_ph.flags = 0xff & ~(SPIFFS_PH_FLAG_USED |
                                                      SPIFFS_PH_FLAG_FINAL);
                              new_ph.objid = objndx_p_hdr->objid & ~SPIFFS_OBJ_ID_IX_FLAG;
                              new_ph.span_ix = data_spndx_offset + i;
                              ret = spiffs_page_allocate_data(fs, new_ph.objid,
                                                              &new_ph, 0, 0, 0, 1,
                                                              &data_pgndx);
                              SPIFFS_CHECK_RES(ret);
                              spiffs_checkinfo
                                ("PA: FIXUP: found no existing data page, created new @ "
                                 "%04x\n", data_pgndx);
                            }

                          /* remap index */

                          spiffs_checkinfo("PA: FIXUP: rewriting index pgndx "
                                           "%04x\n", cur_pgndx);
                          ret = spiffs_rewrite_index(fs,
                                                     objndx_p_hdr->objid | SPIFFS_OBJ_ID_IX_FLAG,
                                                     data_spndx_offset + i, data_pgndx,
                                                     cur_pgndx);
                          if (ret <= _SPIFFS_ERR_CHECK_FIRST &&
                              ret > _SPIFFS_ERR_CHECK_LAST)
                            {
                              /* index bad also, cannot mend this file */

                              spiffs_checkinfo("PA: FIXUP: index bad %d"
                                               ", cannot mend - delete object\n",
                                               ret);

                              /* Delete file */

                              ret = spiffs_page_delete(fs, cur_pgndx);
                            }

                          SPIFFS_CHECK_RES(ret);
                          restart = 1;
                        }
                      else if (rpgndx_within_range)
                        {

                          /* valid reference. read referenced page header */

                          struct spiffs_page_header_s rp_hdr;
                          ret = spiffs_phys_rd(fs,
                                           SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                           0, SPIFFS_PAGE_TO_PADDR(fs, rpgndx),
                                           sizeof(struct spiffs_page_header_s),
                                           (uint8_t *) & rp_hdr);
                          SPIFFS_CHECK_RES(ret);

                          /* cross reference page header check */

                          if (rp_hdr.objid != (p_hdr.objid & ~SPIFFS_OBJ_ID_IX_FLAG) ||
                              rp_hdr.span_ix != data_spndx_offset + i ||
                              (rp_hdr.flags & (SPIFFS_PH_FLAG_DELET |
                                               SPIFFS_PH_FLAG_INDEX |
                                               SPIFFS_PH_FLAG_USED)) !=
                              (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_INDEX))
                            {
                              spiffs_checkinfo("PA: pgndx %04x"
                                               " has inconsistent page header ix objid/span:"
                                               "%04x/%04x, ref objid/span:%04x/%04x flags=%02x\n",
                                               rpgndx,
                                               p_hdr.objid & ~SPIFFS_OBJ_ID_IX_FLAG,
                                               data_spndx_offset + i,
                                               rp_hdr.objid, rp_hdr.span_ix,
                                               rp_hdr.flags);

                              /* try finding correct page */

                              int16_t data_pgndx;
                              ret = spiffs_obj_lu_find_id_and_span(fs,
                                                                   p_hdr.objid & ~SPIFFS_OBJ_ID_IX_FLAG,
                                                                   data_spndx_offset + i, rpgndx,
                                                                   &data_pgndx);
                              if (ret == -ENOENT)
                                {
                                  ret = OK;
                                  data_pgndx = 0;
                                }

                              SPIFFS_CHECK_RES(ret);

                              if (data_pgndx == 0)
                                {
                                  /* not found, this index is badly borked */

                                  spiffs_checkinfo
                                    ("PA: FIXUP: index bad, delete object objid "
                                     "%04x\n", p_hdr.objid);

                                  ret = spiffs_delete_obj_lazy(fs, p_hdr.objid);
                                  SPIFFS_CHECK_RES(ret);
                                  break;
                                }
                              else
                                {
                                  /* found it, so rewrite index */

                                  spiffs_checkinfo
                                    ("PA: FIXUP: found correct data pgndx "
                                     "%04x, rewrite ix pgndx %04x"
                                     " objid %04x\n", data_pgndx, cur_pgndx,
                                     p_hdr.objid);
                                  ret = spiffs_rewrite_index(fs, p_hdr.objid,
                                                             data_spndx_offset + i,
                                                             data_pgndx, cur_pgndx);
                                  if (ret <= _SPIFFS_ERR_CHECK_FIRST &&
                                      ret > _SPIFFS_ERR_CHECK_LAST)
                                    {
                                      /* index bad also, cannot mend this file */

                                      spiffs_checkinfo("PA: FIXUP: index bad %d"
                                                       ", cannot mend!\n", ret);

                                      ret = spiffs_delete_obj_lazy(fs, p_hdr.objid);
                                    }

                                  SPIFFS_CHECK_RES(ret);
                                  restart = 1;
                                }
                            }
                          else
                            {
                              /* mark rpgndx as referenced */

                              const uint32_t rpgndx_byte_ix =
                                (rpgndx - pgndx_offset) / (8 / bits);
                              const uint8_t rpgndx_bit_ix =
                                (rpgndx & ((8 / bits) - 1)) * bits;

                              if (fs->
                                  work[rpgndx_byte_ix] & (1 << (rpgndx_bit_ix + 1)))
                                {
                                  spiffs_checkinfo("PA: pgndx %04x"
                                                   " multiple referenced from page "
                                                   "%04x\n", rpgndx,
                                                   cur_pgndx);
                                  /* Here, we should have fixed all broken
                                   * references - getting this means there
                                   * must be multiple files with same object
                                   * ID. Only solution is to delete
                                   * the object which is referring to this page
                                   */

                                  spiffs_checkinfo("PA: FIXUP: removing object "
                                                   "%04x and page "
                                                   "%04x\n", p_hdr.objid,
                                                   cur_pgndx);

                                  ret = spiffs_delete_obj_lazy(fs, p_hdr.objid);
                                  SPIFFS_CHECK_RES(ret);

                                  /* extra precaution, delete this page also */

                                  ret = spiffs_page_delete(fs, cur_pgndx);
                                  SPIFFS_CHECK_RES(ret);
                                  restart = 1;
                                }
                              fs->work[rpgndx_byte_ix] |=
                                (1 << (rpgndx_bit_ix + 1));
                            }
                        }
                    }
                }

              /* next page */

              cur_pgndx++;
            }

          /* next block */

          cur_block++;
        }

      /* check consistency bitmap */

      if (!restart)
        {
          int16_t objndx_pgndx;
          int16_t rpgndx;

          uint32_t byte_ix;
          uint8_t bit_ix;
          for (byte_ix = 0; !restart && byte_ix < SPIFFS_CFG_LOG_PAGE_SZ(fs);
               byte_ix++)
            {
              for (bit_ix = 0; !restart && bit_ix < 8 / bits; bit_ix++)
                {
                  uint8_t bitmask = (fs->work[byte_ix] >> (bit_ix * bits)) & 0x7;
                  int16_t cur_pgndx =
                    pgndx_offset + byte_ix * (8 / bits) + bit_ix;

                  /* 000 ok - free, unreferenced, not index */

                  if (bitmask == 0x1)
                    {

                      /* 001 */

                      spiffs_checkinfo("PA: pgndx %04x"
                                       " USED, UNREFERENCED, not index\n",
                                       cur_pgndx);

                      uint8_t rewrite_ix_to_this = 0;
                      uint8_t delete_page = 0;

                      /* check corresponding object index entry */

                      struct spiffs_page_header_s p_hdr;
                      ret =
                        spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                   0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                   sizeof(struct spiffs_page_header_s),
                                   (uint8_t *) & p_hdr);
                      SPIFFS_CHECK_RES(ret);

                      ret = spiffs_object_get_data_page_index_reference(fs,
                                                                        p_hdr.objid,
                                                                        p_hdr.span_ix,
                                                                        &rpgndx,
                                                                        &objndx_pgndx);
                      if (ret == OK)
                        {
                          if (((rpgndx == (int16_t) - 1 ||
                                rpgndx > SPIFFS_MAX_PAGES(fs)) ||
                               (SPIFFS_IS_LOOKUP_PAGE(fs, rpgndx))))
                            {
                              /* pointing to a bad page altogether, rewrite
                               * index to this
                               */

                              rewrite_ix_to_this = 1;
                              spiffs_checkinfo("PA: corresponding ref is bad: "
                                               "%04x, rewrite to this "
                                               "%04x\n", rpgndx, cur_pgndx);
                            }
                          else
                            {
                              /* pointing to something else, check what */

                              struct spiffs_page_header_s rp_hdr;
                              ret = spiffs_phys_rd(fs,
                                               SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ, 0,
                                               SPIFFS_PAGE_TO_PADDR(fs, rpgndx),
                                               sizeof(struct spiffs_page_header_s),
                                               (uint8_t *) & rp_hdr);
                              SPIFFS_CHECK_RES(ret);
                              if (((p_hdr.objid & ~SPIFFS_OBJ_ID_IX_FLAG) ==
                                   rp_hdr.objid) &&
                                  ((rp_hdr.flags & (SPIFFS_PH_FLAG_INDEX |
                                                    SPIFFS_PH_FLAG_DELET |
                                                    SPIFFS_PH_FLAG_USED |
                                                    SPIFFS_PH_FLAG_FINAL)) ==
                                   (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_DELET)))
                                {
                                  /* pointing to something else valid, just
                                   * delete this page then
                                   */

                                  spiffs_checkinfo
                                    ("PA: corresponding ref is good but different: "
                                     "%04x, delete this %04x\n",
                                     rpgndx, cur_pgndx);
                                  delete_page = 1;
                                }
                              else
                                {
                                  /* pointing to something weird, update index
                                   * to point to this page instead
                                   */

                                  if (rpgndx != cur_pgndx)
                                    {
                                      spiffs_checkinfo
                                        ("PA: corresponding ref is weird: "
                                         "%04x %s%s%s%s, rewrite this "
                                         "%04x\n", rpgndx,
                                         (rp_hdr.flags & SPIFFS_PH_FLAG_INDEX) ? "" :
                                         "INDEX ",
                                         (rp_hdr.flags & SPIFFS_PH_FLAG_DELET) ? "" :
                                         "DELETED ",
                                         (rp_hdr.flags & SPIFFS_PH_FLAG_USED) ?
                                         "NOTUSED " : "",
                                         (rp_hdr.flags & SPIFFS_PH_FLAG_FINAL) ?
                                         "NOTFINAL " : "", cur_pgndx);
                                      rewrite_ix_to_this = 1;
                                    }
                                  else
                                    {
                                      /* should not happen, destined for fubar */
                                    }
                                }
                            }
                        }
                      else if (ret == -ENOENT)
                        {
                          spiffs_checkinfo
                            ("PA: corresponding ref not found, delete "
                             "%04x\n", cur_pgndx);
                          delete_page = 1;
                          ret = OK;
                        }

                      if (rewrite_ix_to_this)
                        {
                          /* if pointing to invalid page, redirect index to
                           * this page
                           */

                          spiffs_checkinfo("PA: FIXUP: rewrite index objid "
                                           "%04x data spndx %04x"
                                           " to point to this pgndx: %04x"
                                           "\n", p_hdr.objid, p_hdr.span_ix,
                                           cur_pgndx);
                          ret =  spiffs_rewrite_index(fs, p_hdr.objid,
                                                      p_hdr.span_ix, cur_pgndx,
                                                     objndx_pgndx);
                          if (ret <= _SPIFFS_ERR_CHECK_FIRST &&
                              ret > _SPIFFS_ERR_CHECK_LAST)
                            {
                              /* index bad also, cannot mend this file */

                              spiffs_checkinfo("PA: FIXUP: index bad %d"
                                               ", cannot mend!\n", ret);

                              ret = spiffs_page_delete(fs, cur_pgndx);
                              SPIFFS_CHECK_RES(ret);
                              ret = spiffs_delete_obj_lazy(fs, p_hdr.objid);
                            }

                          SPIFFS_CHECK_RES(ret);
                          restart = 1;
                          continue;
                        }
                      else if (delete_page)
                        {
                          spiffs_checkinfo("PA: FIXUP: deleting page %04x"
                                           "\n", cur_pgndx);

                          ret = spiffs_page_delete(fs, cur_pgndx);
                        }

                      SPIFFS_CHECK_RES(ret);
                    }

                  if (bitmask == 0x2)
                    {
                      /* 010 */

                      spiffs_checkinfo("PA: pgndx %04x"
                                       " FREE, REFERENCED, not index\n",
                                       cur_pgndx);

                      /* no op, this should be taken care of when checking
                       * valid references
                       */
                    }

                  /* 011 ok - busy, referenced, not index */

                  if (bitmask == 0x4)
                    {
                      /* 100 */

                      spiffs_checkinfo("PA: pgndx %04x"
                                       " FREE, unreferenced, INDEX\n", cur_pgndx);

                      /* this should never happen, major fubar */
                    }

                  /* 101 ok - busy, unreferenced, index */

                  if (bitmask == 0x6)
                    {
                      /* 110 */

                      spiffs_checkinfo("PA: pgndx %04x"
                                       " FREE, REFERENCED, INDEX\n", cur_pgndx);

                      /* no op, this should be taken care of when checking
                       * valid references
                       */
                    }

                  if (bitmask == 0x7)
                    {
                      /* 111 */

                      spiffs_checkinfo("PA: pgndx %04x"
                                       " USED, REFERENCED, INDEX\n", cur_pgndx);

                      /* no op, this should be taken care of when checking
                       * valid references
                       */
                    }
                }
            }
        }

      spiffs_checkinfo("PA: processed %04x, restart %d\n",
                       pgndx_offset, restart);

      /* next page range */

      if (!restart)
        {
          pgndx_offset += pages_per_scan;
        }
    }

  return ret;
}

/* Checks consistency amongst all pages and fixes irregularities */

int32_t spiffs_page_consistency_check(FAR struct spiffs_s *fs)
{
  return spiffs_page_consistency_check_i(fs);
}

/* Object index consistency */

/* searches for given object ID in temporary object ID index,
 * returns the index or -1
 */

static int spiffs_object_index_search(FAR struct spiffs_s *fs, int16_t objid)
{
  uint32_t i;
  int16_t *obj_table = (int16_t *) fs->work;
  objid &= ~SPIFFS_OBJ_ID_IX_FLAG;
  for (i = 0; i < SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t); i++)
    {
      if ((obj_table[i] & ~SPIFFS_OBJ_ID_IX_FLAG) == objid)
        {
          return i;
        }
    }
  return -1;
}

static int32_t spiffs_object_index_consistency_check_v(FAR struct spiffs_s *fs,
                                                     int16_t objid,
                                                     int16_t cur_block,
                                                     int cur_entry,
                                                     const void *user_const_p,
                                                     void *user_var_p)
{
  (void)user_const_p;
  int32_t res_c = SPIFFS_VIS_COUNTINUE;
  int32_t ret = OK;
  uint32_t *log_ix = (uint32_t *) user_var_p;
  int16_t *obj_table = (int16_t *) fs->work;

  if (objid != SPIFFS_OBJ_ID_FREE && objid != SPIFFS_OBJ_ID_DELETED &&
      (objid & SPIFFS_OBJ_ID_IX_FLAG))
    {
      struct spiffs_page_header_s p_hdr;
      int16_t cur_pgndx =
        SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, cur_block, cur_entry);

      /* load header */

      ret = spiffs_phys_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                       0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                       sizeof(struct spiffs_page_header_s), (uint8_t *) & p_hdr);
      SPIFFS_CHECK_RES(ret);

      if (p_hdr.span_ix == 0 &&
          (p_hdr.
           flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                    SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE)) ==
          (SPIFFS_PH_FLAG_DELET))
        {
          spiffs_checkinfo("IX: pgndx %04x, objid:%04x spndx:"
                           "%04x header not fully deleted - deleting\n",
                           cur_pgndx, objid, p_hdr.span_ix);

          ret = spiffs_page_delete(fs, cur_pgndx);
          SPIFFS_CHECK_RES(ret);
          return res_c;
        }

      if ((p_hdr.
           flags & (SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_FINAL |
                    SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE)) ==
          (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE))
        {
          return res_c;
        }

      if (p_hdr.span_ix == 0)
        {
          /* objndx header page, register objid as reachable */

          int r = spiffs_object_index_search(fs, objid);
          if (r == -1)
            {
              /* not registered, do it */

              obj_table[*log_ix] = objid & ~SPIFFS_OBJ_ID_IX_FLAG;
              (*log_ix)++;
              if (*log_ix >= SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t))
                {
                  *log_ix = 0;
                }
            }
        }
      else
        {
          /* span index
           * objndx page, see if header can be found
           */

          int r = spiffs_object_index_search(fs, objid);
          uint8_t delete = 0;
          if (r == -1)
            {
              /* not in temporary index, try finding it */

              int16_t objhdr_pgndx;
              ret =
                spiffs_obj_lu_find_id_and_span(fs, objid | SPIFFS_OBJ_ID_IX_FLAG,
                                               0, 0, &objhdr_pgndx);
              res_c = SPIFFS_VIS_COUNTINUE_RELOAD;
              if (ret == OK)
                {
                  /* found, register as reachable */

                  obj_table[*log_ix] = objid & ~SPIFFS_OBJ_ID_IX_FLAG;
                }
              else if (ret == -ENOENT)
                {
                  /* not found, register as unreachable */

                  delete = 1;
                  obj_table[*log_ix] = objid | SPIFFS_OBJ_ID_IX_FLAG;
                }
              else
                {
                  SPIFFS_CHECK_RES(ret);
                }
              (*log_ix)++;
              if (*log_ix >= SPIFFS_CFG_LOG_PAGE_SZ(fs) / sizeof(int16_t))
                {
                  *log_ix = 0;
                }
            }
          else
            {
              /* in temporary index, check reachable flag */

              if ((obj_table[r] & SPIFFS_OBJ_ID_IX_FLAG))
                {
                  /* registered as unreachable */

                  delete = 1;
                }
            }

          if (delete)
            {
              spiffs_checkinfo("IX: FIXUP: pgndx %04x, obj objid=%04x"
                               " spndx:%04x"
                               " is orphan index - deleting\n", cur_pgndx, objid,
                               p_hdr.span_ix);

              ret = spiffs_page_delete(fs, cur_pgndx);
              SPIFFS_CHECK_RES(ret);
            }
        }
    }

  return res_c;
}

/* Removes orphaned and partially deleted index pages.
 * Scans for index pages. When an index page is found, corresponding index header is searched for.
 * If no such page exists, the index page cannot be reached as no index header exists and must be
 * deleted.
 */

int32_t spiffs_object_index_consistency_check(FAR struct spiffs_s *fs)
{
  int32_t ret = OK;

  /* impl note:
   * fs->work is used for a temporary object index memory, listing found object 
   * ids and indicating whether they can be reached or not. Acting as a fifo if
   * object ids cannot fit.  In the temporary object index memory,
   * SPIFFS_OBJ_ID_IX_FLAG bit is used to indicate a reachable/unreachable
   * object ID.
   */

  memset(fs->work, 0, SPIFFS_CFG_LOG_PAGE_SZ(fs));
  uint32_t obj_id_log_ix = 0;

  ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0,
                             spiffs_object_index_consistency_check_v, 0,
                             &obj_id_log_ix, 0, 0);
  if (ret == SPIFFS_VIS_END)
    {
      ret = OK;
    }

  return ret;
}
