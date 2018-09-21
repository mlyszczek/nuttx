/****************************************************************************
 * fs/spiffs/spiffs.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes logic taken from 0.3.7 of SPIFFS by Peter Andersion.  That
 * version was originally released under the MIT license.
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

#include <sys/stat.h>
#include <sys/statfs.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>

#include "spiffs.h"
#include "spiffs_nucleus.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define spiffs_lock_volume(fs)       (spiffs_lock_reentrant(&fs->exclsem))
#define spiffs_lock_file(fobj)       (spiffs_lock_reentrant(&fobj->exclsem))

#define spiffs_unlock_volume(fs)     (spiffs_unlock_reentrant(&fs->exclsem))
#define spiffs_unlock_file(fobj)     (spiffs_unlock_reentrant(&fobj->exclsem))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* TMPFS helpers */

static void spiffs_lock_reentrant(FAR struct spiffs_sem_s *sem);
static void spiffs_unlock_reentrant(FAR struct spiffs_sem_s *sem);

/* File system operations */

static int  spiffs_open(FAR struct file *filep, FAR const char *relpath,
              int oflags, mode_t mode);
static int  spiffs_close(FAR struct file *filep);
static ssize_t spiffs_read(FAR struct file *filep, FAR char *buffer,
              size_t buflen);
static ssize_t spiffs_write(FAR struct file *filep, FAR const char *buffer,
              size_t buflen);
static off_t spiffs_seek(FAR struct file *filep, off_t offset, int whence);
static int  spiffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int  spiffs_sync(FAR struct file *filep);
static int  spiffs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int  spiffs_fstat(FAR const struct file *filep, FAR struct stat *buf);
static int  spiffs_truncate(FAR struct file *filep, off_t length);

static int  spiffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
              FAR struct fs_dirent_s *dir);
static int  spiffs_closedir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  spiffs_readdir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  spiffs_rewinddir(FAR struct inode *mountpt,
              FAR struct fs_dirent_s *dir);
static int  spiffs_bind(FAR struct inode *blkdriver, FAR const void *data,
              FAR void **handle);
static int  spiffs_unbind(FAR void *handle, FAR struct inode **blkdriver,
              unsigned int flags);
static int  spiffs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf);
static int  spiffs_unlink(FAR struct inode *mountpt, FAR const char *relpath);
static int  spiffs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
              mode_t mode);
static int  spiffs_rmdir(FAR struct inode *mountpt, FAR const char *relpath);
static int  spiffs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
              FAR const char *newrelpath);
static int  spiffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
              FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations spiffs_operations =
{
  spiffs_open,       /* open */
  spiffs_close,      /* close */
  spiffs_read,       /* read */
  spiffs_write,      /* write */
  spiffs_seek,       /* seek */
  spiffs_ioctl,      /* ioctl */

  spiffs_sync,       /* sync */
  spiffs_dup,        /* dup */
  spiffs_fstat,      /* fstat */
  spiffs_truncate,   /* truncate */

  spiffs_opendir,    /* opendir */
  spiffs_closedir,   /* closedir */
  spiffs_readdir,    /* readdir */
  spiffs_rewinddir,  /* rewinddir */

  spiffs_bind,       /* bind */
  spiffs_unbind,     /* unbind */
  spiffs_statfs,     /* statfs */

  spiffs_unlink,     /* unlink */
  spiffs_mkdir,      /* mkdir */
  spiffs_rmdir,      /* rmdir */
  spiffs_rename,     /* rename */
  spiffs_stat,       /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_lock_reentrant
 ****************************************************************************/

static void spiffs_lock_reentrant(FAR struct spiffs_sem_s *rsem)
{
  pid_t me;

  /* Do we already hold the semaphore? */

  me = getpid();
  if (me == rsem->holder)
    {
      /* Yes... just increment the count */

      rsem->count++;
      DEBUGASSERT(rsem->count > 0);
    }

  /* Take the semaphore (perhaps waiting) */

  else
    {
      int ret;

      do
        {
          ret = nxsem_wait(&rsem->sem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);

      /* No we hold the semaphore */

      rsem->holder = me;
      rsem->count  = 1;
    }
}

/****************************************************************************
 * Name: spiffs_unlock_reentrant
 ****************************************************************************/

static void spiffs_unlock_reentrant(FAR struct spiffs_sem_s *rsem)
{
  DEBUGASSERT(rsem->holder == getpid());

  /* Is this our last count on the semaphore? */

  if (rsem->count > 1)
    {
      /* No.. just decrement the count */

      rsem->count--;
    }

  /* Yes.. then we can really release the semaphore */

  else
    {
      rsem->holder = SPIFFS_NO_HOLDER;
      rsem->count  = 0;
      nxsem_post(&rsem->sem);
    }
}

/****************************************************************************
 * Name: spiffs_readdir_callback
 ****************************************************************************/

static int spiffs_readdir_callback(FAR struct spiffs_s *fs,
                                   int16_t objid, int16_t bix, int ix_entry,
                                   FAR const void *user_const_p,
                                   FAR void *user_var_p)
{
  struct spiffs_pgobj_ixheader_s objix_hdr;
  int16_t pix;
  int ret;

  if (objid == SPIFFS_OBJ_ID_FREE || objid == SPIFFS_OBJ_ID_DELETED ||
      (objid & SPIFFS_OBJ_ID_IX_FLAG) == 0)
    {
      return SPIFFS_VIS_COUNTINUE;
    }

  pix = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PIX(fs, bix, ix_entry);
  ret = _spiffs_rd(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   0, SPIFFS_PAGE_TO_PADDR(fs, pix),
                   sizeof(struct spiffs_pgobj_ixheader_s),
                   (FAR uint8_t *) & objix_hdr);
  if (ret < 0)
    {
      ferr("ERROR: _spiffs_rd failed: %d\n", ret);
      return ret;
    }

  if ((objid & SPIFFS_OBJ_ID_IX_FLAG) &&
      objix_hdr.p_hdr.span_ix == 0 &&
      (objix_hdr.p_hdr.flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL |
                                SPIFFS_PH_FLAG_IXDELE)) ==
      (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_IXDELE))
    {
      FAR struct fs_dirent_s *dir = (FAR struct fs_dirent_s *)user_var_p;
      FAR struct dirent *entryp;

      DEBUASSERT(dir != NULL);
      entryp = &dir->fd_dir;

      strncpy(entryp->d_name, (FAR char *)objix_hdr.name, NAME_MAX + 1);
      entryp->d_type = objix_hdr.type;
      return OK;
    }

  return SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Name: spiffs_open
 ****************************************************************************/

static int spiffs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  off_t offset;
  int16_t pix;
  int ret;

  finfo("relpath: %s oflags; %04x\n", relpath, oflags);
  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Skip over any leading directory separators (shouldn't be any) */

  for (; *relpath == '/'; relpath++)
    {
    }

  /* Check the length of the relative path */

  if (strlen(relpath) > SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Allocate a new file object */

  fobj = (FAR struct spiffs_file_s *)kmm_zalloc(sizeof(struct spiffs_file_s));
  if (fobj == NULL)
    {
      return -ENOMEM;
    }

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Check of the file object already exists */

  ret = spiffs_object_find_object_index_header_by_name(fs,
                                                       (FAR const uint8_t *)relpath,
                                                       &pix);
  if (ret < 0 && (oflags & O_CREAT) == 0)
    {
      /* It does not exist and we were not asked to create it */

      goto errout_with_fileobject;
    }
  else if (ret == OK && (oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
    {
      /* O_CREAT and O_EXCL and file exists - fail */

      ret = -EEXIST;
      goto errout_with_fileobject;
    }
  else if ((oflags & O_CREAT) != 0 && ret == -ENOENT)
    {
      int16_t objid;

      /* The file does not exist.  We need to create the it. */

      ret = spiffs_obj_lu_find_free_obj_id(fs, &objid, 0);
      if (ret < 0)
        {
          goto errout_with_fileobject;
        }

      ret = spiffs_object_create(fs, objid, (FAR const uint8_t *)relpath, 0,
                                 DTYPE_FILE, &pix);
      if (ret < 0)
        {
          goto errout_with_fileobject;
        }

      /* Since we created the file, we don't need to truncate it */

      oflags &= ~O_TRUNC;
    }
  else if (ret < 0)
    {
      goto errout_with_fileobject;
    }

  /* Open the file */

  ret = spiffs_object_open_by_page(fs, pix, fobj, oflags, mode);
  if (ret < 0)
    {
      goto errout_with_fileobject;
    }

  /* Truncate the file to zero length */

  if ((oflags & O_TRUNC) != 0)
    {
      ret = spiffs_object_truncate(fobj, 0, false);
      if (ret < 0)
        {
          goto errout_with_fileobject;
        }
    }

  /* Save the struct spiffs_file_s instance as the file private data */

  filep->f_priv = fobj;

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  offset = 0;
  if ((oflags & (O_APPEND | O_WROK)) == (O_APPEND | O_WROK))
    {
      offset = fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size;
    }

  /* Save the file position */

  filep->f_pos = offset;

  /* Finish initialization of the file object */

  sem_init(&fobj->exclsem.sem, 0, 1);
  fobj->exclsem.holder = SPIFFS_NO_HOLDER;

  /* Add the new file object to the tail of the open file list */

  dq_addlast((FAR sq_entry_t *)fobj, &fs->objq);

  spiffs_unlock_volume(fs);
  return OK;

errout_with_fileobject:
  kmm_free(fobj);

errout_with_lock:
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_close
 ****************************************************************************/

static int spiffs_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("filep: %p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover our private data from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* Decrement the reference count on the file */

  DEBUGASSERT(fobj->crefs > 0);
  if (fobj->crefs > 0)
    {
      fobj->crefs--;
    }

  filep->f_priv = NULL;

  /* If the reference count decremented to zero and the file has been
   * unlinked, then free resources related to the open file.
   */

  if (fobj->crefs == 0 && (fobj->flags & SFO_FLAG_UNLINKED) != 0)
    {
      /* Free the file object while we hold the lock?  Weird but this
       * should be safe because the object is unlinked and could not
       * have any other references.
       */

      spiffs_file_free(fs, fobj);
      return OK;
    }

  /* Release the lock on the file */

  spiffs_unlock_file(fobj);
  return OK;
}

/****************************************************************************
 * Name: spiffs_read
 ****************************************************************************/

static ssize_t spiffs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nread;
  off_t startpos;
  off_t endpos;

  finfo("filep: %p buffer: %p buflen: %lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* Read from FLASH */

  nread = spiffs_hydro_read(fs, fobj, buffer, buflen);
  if (nread == SPIFFS_ERR_END_OF_OBJECT)
    {
      nread = 0;
    }

  /* Release the lock on the file */

  spiffs_unlock_file(fobj);
  return nread;
}

/****************************************************************************
 * Name: spiffs_write
 ****************************************************************************/

static ssize_t spiffs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  ssize_t nwritten;
  off_t offset;
  int ret;

  finfo("filep: %p buffer: %p buflen: %lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* Verify that the file was opened with write access */

  if ((fobj->oflags & O_WROK) == 0)
    {
      ret = -EACCES;
      goto errout_with_lock;
    }

  /* Write to FLASH (or cache) */

  offset = filep->f_pos;

  if (fobj->cache_page == 0)
    {
      /* See if object id is associated with cache already */

      fobj->cache_page = spiffs_cache_page_get_by_fd(fs, fobj);
    }

  if ((fobj->flags & O_DIRECT) == 0)
    {
      if (buflen < (size_t)SPIFFS_CFG_LOG_PAGE_SZ(fs))
        {
          /* Small write, try to cache it */

          bool alloc_cpage = true;
          if (fobj->cache_page != NULL)
            {
              /* We have a cached page for this object already, check cache
               * page boundaries
               */

              if (offset < fobj->cache_page->offset ||
                  offset > fobj->cache_page->offset + fobj->cache_page->size ||
                  offset + buflen > fobj->cache_page->offset + SPIFFS_CFG_LOG_PAGE_SZ(fs))
                {
                  /* Boundary violation, write back cache first and allocate
                   * new
                   */

                  spiffs_cacheinfo("Cache page=%d for fobj ID=%d "
                                   "Boundary violation, offset=%d size=%d\n",
                                   fobj->cache_page->ix, fobj->objid,
                                   fobj->cache_page->offset, fobj->cache_page->size);
                  nwritten = spiffs_hydro_write(fs, fobj,
                                                spiffs_get_cache_page(fs, spiffs_get_cache(fs),
                                                                      fobj->cache_page->ix),
                                                fobj->cache_page->offset,
                                                fobj->cache_page->size);
                  spiffs_cache_fd_release(fs, fobj->cache_page);
                  if (nwritten < 0)
                    {
                      ret = (int)nwritten;
                      goto errout_with_lock;
                    }
                }
              else
                {
                  /* Writing within cache */

                  alloc_cpage = false;
                }
            }

          if (alloc_cpage)
            {
              fobj->cache_page = spiffs_cache_page_allocate_by_fd(fs, fobj);
              if (fobj->cache_page)
                {
                  fobj->cache_page->offset = offset;
                  fobj->cache_page->size   = 0;

                  spiffs_cacheinfo("Allocated cache page %d for fobj %d\n",
                                   fobj->cache_page->ix, fobj->objid);
                }
            }

          if (fobj->cache_page)
            {
              FAR struct spiffs_cache_s *cache;
              FAR uint8_t *cpage_data;
              off_t offset_in_cpage;

              offset_in_cpage = offset - fobj->cache_page->offset;
              spiffs_cacheinfo("Storing to cache page %d for fobj %d offset=%d:%d buflen=%d\n",
                               fobj->cache_page->ix, fobj->objid, offset,
                               offset_in_cpage, buflen);
              cache = spiffs_get_cache(fs);
              cpage_data = spiffs_get_cache_page(fs, cache, fobj->cache_page->ix);

              memcpy(&cpage_data[offset_in_cpage], buffer, buflen);
              fobj->cache_page->size = MAX(fobj->cache_page->size, offset_in_cpage + buflen);

              nwritten = buflen;
              goto success_with_lock;
            }
          else
            {
              nwritten = spiffs_hydro_write(fs, fobj, buffer, offset, buflen);
              if (nwritten < 0)
                {
                  ret = (int)nwritten;
                  goto errout_with_lock;
                }

               goto success_with_lock;
            }
        }
      else
        {
          /* Big write, no need to cache it - but first check if there is a
           * cached write already
           */

          if (fobj->cache_page)
            {
              /* Write back cache first */

              spiffs_cacheinfo("Cache page=%d for fobj ID=%d "
                               "Boundary violation, offset=%d size=%d\n",
                               fobj->cache_page->ix, fobj->objid,
                               fobj->cache_page->offset, fobj->cache_page->size);

              nwritten = spiffs_hydro_write(fs, fobj,
                                            spiffs_get_cache_page(fs,
                                                                  spiffs_get_cache(fs),
                                                                  fobj->cache_page->ix),
                                            fobj->cache_page->offset,
                                            fobj->cache_page->size);
              spiffs_cache_fd_release(fs, fobj->cache_page);

              if (nwritten < 0)
                {
                  ret = (int)nwritten;
                  goto errout_with_lock;
                }

              /* Data written below */
            }
        }
    }

  nwritten = spiffs_hydro_write(fs, fobj, buffer, offset, buflen);
  if (nwritten < 0)
    {
      ret = (int)nwritten;
      goto errout_with_lock;
    }

success_with_lock:
  /* Update the file position */

  filep->f_pos += nwritten;

  /* Release our access to the file object */

  spiffs_unlock_file(fobj);
  return nwritten;

errout_with_lock:
  spiffs_unlock_file(fobj);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: spiffs_seek
 ****************************************************************************/

static off_t spiffs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int16_t data_spix;
  int16_t objix_spix;
  off_t fsize;
  off_t pos;
  int ret;

  finfo("filep=%p offset=%ld whence=%d\n", filep, (long)offset, whence);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* Get the new file offset */

  spiffs_fflush_cache(fobj);

  fsize = fobj->size == SPIFFS_UNDEFINED_LEN ? 0 : fobj->size;

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET: /* The offset is set to offset bytes. */
          pos = offset;
          break;

      case SEEK_CUR: /* The offset is set to its current location plus
                      * offset bytes. */
          pos = offset + filep->f_pos;
          break;

      case SEEK_END: /* The offset is set to the size of the file plus
                      * offset bytes. */
          pos = fsize + offset;
          break;

      default:
          return -EINVAL;
    }

  /* Verify the resulting file position */

  if (pos < 0)
    {
      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* Attempts to set the position beyond the end of file should
   * work if the file is open for write access.
   *
   * REVISIT: This simple implementation has no per-open storage that
   * would be needed to retain the open flags.
   */

  if (pos > fsize)
    {
      filep->f_pos = fsize;
      ret = -ENOSYS;
      goto errout_with_lock;
    }

  /* Set up for the new file position */


  data_spix  = (pos > 0 ? (pos - 1) : 0) / SPIFFS_DATA_PAGE_SIZE(fs);
  objix_spix = SPIFFS_OBJ_IX_ENTRY_SPAN_IX(fs, data_spix);

  if (fobj->cursor_objix_spix != objix_spix)
    {
      int16_t pix;

      ret = spiffs_obj_lu_find_id_and_span(fs, fobj->objid | SPIFFS_OBJ_ID_IX_FLAG,
                                           objix_spix, 0, &pix);
      if (ret < 0)
        {
          SPIFFS_UNLOCK(fs);
          return ret;
        }

      fobj->cursor_objix_spix = objix_spix;
      fobj->cursor_objix_pix  = pix;
    }

  filep->f_pos = pos;
  spiffs_unlock_file(fobj);
  return pos;

errout_with_lock:
  spiffs_unlock_file(fobj);
  return (off_t)ret;
}

/****************************************************************************
 * Name: spiffs_ioctl
 ****************************************************************************/

static int spiffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  finfo("filep=%p cmd=%d arg=%ld\n", filep, cmd, (long)arg);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* There are no supported IOCTL commands */

  return -ENOTTY;
}

/****************************************************************************
 * Name: spiffs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int spiffs_sync(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("filep=%p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* Flush all cached write data */

  ret = spiffs_fflush_cache(fobj);

  spiffs_unlock_file(fobj);
  return ret;
}

/****************************************************************************
 * Name: spiffs_dup
 ****************************************************************************/

static int spiffs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct spiffs_file_s *fobj;

  finfo("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL &&
              newp->f_priv == NULL && newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  fobj = oldp->f_priv;
  DEBUGASSERT(fobj != NULL);

  /* Increment the reference count (atomically)*/

  spiffs_lock_file(fobj);
  fobj->crefs++;
  spiffs_unlock_file(fobj);

  /* Save a copy of the file object as the dup'ed file. */

  newp->f_priv = fobj;
  return OK;
}

/****************************************************************************
 * Name: spiffs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fobj', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int spiffs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("filep=%p buf=%p\n", filep, buf);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL && buf != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume state data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* Flush the cache and perform the common stat() operation */

  spiffs_fflush_cache(fobj);

  ret = spiffs_stat_pix(fs, fobj->objix_hdr_pix, fobj->objid, buf);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_stat_pix failed: %d\n", ret);
    }

  spiffs_unlock_file(fobj);
  return ret;
}

/****************************************************************************
 * Name: spiffs_truncate
 ****************************************************************************/

static int spiffs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct spiffs_file_s *fobj;
  int ret;

  finfo("filep: %p length: %ld\n", filep, (long)length);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL &&  length >= 0);

  /* Recover the file object state from the struct file instance */

  fobj = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(fobj);

  /* REVISIT:  I believe that this can only truncate to smaller sizes. */

  ret = spiffs_object_truncate(fobj, length, false);

  spiffs_unlock_file(fobj);
  return ret;
}

/****************************************************************************
 * Name: spiffs_opendir
 ****************************************************************************/

static int spiffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s *dir)
{
  finfo("mountpt: %p relpath: %s dir: %p\n",
        mountpt, relpath, dir);

  DEBUGASSERT(mountpt != NULL && relpath != NULL && dir != NULL);

  /* Initialize for traversal of the 'directory' */

  dir->u.spiffs.block   = 0;
  dir->u.spiffs.entry   = 0;
  return OK;
}

/****************************************************************************
 * Name: spiffs_closedir
 ****************************************************************************/

static int spiffs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* There is nothing to be done */

  return OK;
}

/****************************************************************************
 * Name: spiffs_readdir
 ****************************************************************************/

static int spiffs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct spiffs_s *fs;
  int16_t bix;
  int entry;
  int status;
  int ret;

  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Lock the SPIFFS volume */

  spiffs_lock_volume(fs);

  /* And visit the next file object */

  ret = spiffs_foreach_objlu(fs, dir->u.spiffs.block, dir->u.spiffs.entry,
                             SPIFFS_VIS_NO_WRAP, 0, spiffs_readdir_callback,
                             NULL, dir, &bix, &entry);
  if (ret == OK)
    {
      dir->u.spiffs.block = bix;
      dir->u.spiffs.entry = entry + 1;
    }

  /* Release the lock on the file system */

  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_rewinddir
 ****************************************************************************/

static int spiffs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Reset as when opendir() was called. */

  dir->u.spiffs.block   = 0;
  dir->u.spiffs.entry   = 0;

  return OK;
}

/****************************************************************************
 * Name: spiffs_bind
 ****************************************************************************/

static int spiffs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct spiffs_s *fs;

  finfo("blkdriver: %p data: %p handle: %p\n", blkdriver, data, handle);
  DEBUGASSERT(blkdriver == NULL && handle != NULL);

  /* Create an instance of the SPIFFS file system */

  fs = (FAR struct spiffs_s *)kmm_zalloc(sizeof(struct spiffs_s));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

#warning Missing logic

  /* Return the new file system handle */

  *handle = (FAR void *)fs;
  return OK;
}

/****************************************************************************
 * Name: spiffs_unbind
 ****************************************************************************/

static int spiffs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                         unsigned int flags)
{
  FAR struct spiffs_s *fs = (FAR struct spiffs_s *)handle;
  int ret;

  finfo("handle: %p blkdriver: %p flags: %02x\n",
        handle, blkdriver, flags);
  DEBUGASSERT(fs != NULL);

  /* Lock the file system */

  spiffs_lock_volume(fs);

  /* Traverse all directory entries (recursively), freeing all resources. */
#warning Missing logic

  /* Now we can destroy the root file system and the file system itself. */
#warning Missing logic

  nxsem_destroy(&fs->exclsem.sem);
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_statfs
 ****************************************************************************/

static int spiffs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj;
  uint32_t pages_per_block;
  uint32_t blocks;
  uint32_t obj_lupages;
  uint32_t data_pgsize;
  uint32_t ndata_pages;
  uint32_t nfile_objs;
  uint32_t total;
  uint32_t used;
  int ret = OK;

  finfo("mountpt: %p buf: %p\n", mountpt, buf);
  DEBUGASSERT(mountpt != NULL && buf != NULL);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Lock the SPIFFS volume */

  spiffs_lock_volume(fs);

  /* Collect some statistics */

  pages_per_block  = SPIFFS_PAGES_PER_BLOCK(fs);
  blocks           = fs->block_count;
  obj_lupages      = SPIFFS_OBJ_LOOKUP_PAGES(fs);
  data_pgsize      = SPIFFS_DATA_PAGE_SIZE(fs);

   /* -2 for  spare blocks, +1 for emergency page */

  ndata_pages      = (blocks - 2) * (pages_per_block - obj_lupages) + 1;
  total            = ndata_pages * data_pgsize;
  used             = fs->stats_p_allocated * data_pgsize;

  /* Count the number of file objects */

  nfile_objs       = 0;
  for (fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq);
       fobj != NULL;
       fobj  = (FAR struct spiffs_file_s *)dq_next((FAR dq_entry_t *)fobj))
    {
      nfile_objs++;
    }

  /* Fill in the statfs structure */

  buf->f_type      = SPIFFS_SUPER_MAGIC;
  buf->f_namelen   = SPIFFS_NAME_MAX - 1;
  buf->f_bsize     = data_pgsize;
  buf->f_blocks    = ndata_pages;
  buf->f_bfree     = ndata_pages - used;
  buf->f_bavail    = buf->f_bfree;
  buf->f_files     = nfile_objs;
  buf->f_ffree     = buf->f_bfree;  /* SWAG */

  /* Release the lock on the file system */

  spiffs_unlock_volume(fs);
  return OK;
}

/****************************************************************************
 * Name: spiffs_unlink
 ****************************************************************************/

static int spiffs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *fobj = NULL;
  FAR const char *name;
  int ret;

  finfo("mountpt: %p relpath: %s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Find the file object and parent directory associated with this relative
   * path.  If successful, spiffs_find_file will lock both the file object
   * and the parent directory and take one reference count on each.
   */

  ret = spiffs_find_file(fs, relpath, &fobj, 0);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  DEBUGASSERT(fobj != NULL);

  /* Get the file name from the relative path */

  name = strrchr(relpath, '/');
  if (name != NULL)
    {
      /* Skip over the file '/' character */

      name++;
    }
  else
    {
      /* The name must lie in the root directory */

      name = relpath;
    }

#warning Missing logic

  /* If the reference count is not one, then just mark the file as
   * unlinked
   */

  if (fobj->crefs > 1)
    {
      /* Make the file object as unlinked */

      fobj->flags |= SFO_FLAG_UNLINKED;

      /* Release the reference count on the file object */

      fobj->crefs--;
      spiffs_unlock_file(fobj);
    }

  /* Otherwise we can free resources held by the open file now */

  else
    {
      nxsem_destroy(&fobj->exclsem.sem);
      spiffs_file_free(fs, fobj);
    }

  /* Release the lock on the volume */

  spiffs_unlock_volume(fs);

  return OK;

errout_with_objects:
#warning Missing logic

errout_with_lock:
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_mkdir
 ****************************************************************************/

static int spiffs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                       mode_t mode)
{
  finfo("mountpt: %p relpath: %s mode: %04x\n", mountpt, relpath, mode);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Directories are not supported */

  return -ENOSYS;
}

/****************************************************************************
 * Name: spiffs_rmdir
 ****************************************************************************/

static int spiffs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  finfo("mountpt: %p relpath: %s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Directories are not supported */

  return -ENOSYS;
}

/****************************************************************************
 * Name: spiffs_rename
 ****************************************************************************/

static int spiffs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                        FAR const char *newrelpath)
{
  FAR struct spiffs_s *fs;
  int ret;

  finfo("mountpt: %p oldrelpath: %s newrelpath: %s\n",
        mountpt, oldrelpath, newrelpath);
  DEBUGASSERT(mountpt != NULL && oldrelpath != NULL && newrelpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

#warning Missing logic

  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_stat
 ****************************************************************************/

static int spiffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct spiffs_s *fs;
  int16_t pix;
  int ret;

  finfo("mountpt=%p relpath=%s buf=%p\n", mountpt, relpath, buf);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && buf != NULL);

  if (strlen(relpath) > SPIFFS_NAME_MAX - 1)
    {
      return -ENAMETOOLONG;
    }

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Find the object associated with this relative path */

  ret = spiffs_object_find_object_index_header_by_name(fs,
                                                       (FAR const uint8_t *)relpath,
                                                       &pix);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  /* And get information about the object */

  ret = spiffs_stat_pix(fs, pix, 0, buf);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_stat_pix failed: %d\n", ret);
    }

errout_with_lock:
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_file_free
 *
 * Description:
 *   Free all resources used a file object
 *
 * Input Parameters:
 *   fs   - A reference to the volume structure
 *   fobj - A reference to the file object to be removed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spiffs_file_free(FAR struct spiffs_s *fs, FAR struct spiffs_file_s *fobj)
{
  FAR struct spiffs_file_s *curr;
  int ret;

  /* Flush any buffered write data */

  ret = spiffs_fflush_cache(fobj);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_fflush_cache failed: %d\n", ret);
    }

  /* Remove the file object from the list of file objects in the volume
   * structure.
   */

  for (curr  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq);
       curr != NULL;
       curr  = (FAR struct spiffs_file_s *)dq_next((FAR dq_entry_t *)curr))
    {
      /* Is this the entry we are searching for? */

      if (curr == fobj)
        {
          /* Yes, remove it from the list of of file objects */

          dq_rem((FAR dq_entry_t *)curr, &fs->objq);
        }
    }

  DEBUGASSERT(curr != NULL);

  /* Then free the file object itself (which contains the lock we hold) */

  kmm_free(fobj);
}

