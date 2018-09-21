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
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>

#include "spiffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define spiffs_lock_volume(fs)       (spiffs_lock_reentrant(&fs->exclsem))
#define spiffs_lock_file(sfo)        (spiffs_lock_reentrant(&sfo->exclsem))
#define spiffs_lock_directory(sdo)   (spiffs_lock_reentrant(&sdo->exclsem))
#define spiffs_unlock_volume(fs)     (spiffs_unlock_reentrant(&fs->exclsem))
#define spiffs_unlock_file(sfo)      (spiffs_unlock_reentrant(&sfo->exclsem))
#define spiffs_unlock_directory(sdo) (spiffs_unlock_reentrant(&sdo->exclsem))

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
 * Name: spiffs_open
 ****************************************************************************/

static int spiffs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct spiffs_s *fs;
  FAR struct spiffs_file_s *sfo;
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

  sfo = (FAR struct spiffs_file_s *)kmm_zalloc(sizeof(struct spiffs_file_s));
  if (sfo == NULL)
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
      int16_t id;

      /* The file does not exist.  We need to create the it. */

      ret = spiffs_obj_lu_find_free_obj_id(fs, &id, 0);
      if (ret < 0)
        {
          goto errout_with_fileobject;
        }

      ret = spiffs_object_create(fs, id, (FAR const uint8_t *)relpath, 0,
                                 SPIFFS_TYPE_FILE, &pix);
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

  ret = spiffs_object_open_by_page(fs, pix, sfo, oflags, mode);
  if (ret < 0)
    {
      goto errout_with_fileobject;
    }

  /* Truncate the file to zero length */

  if ((oflags & O_TRUNC) != 0)
    {
      ret = spiffs_object_truncate(sfo, 0, 0);
      if (ret < 0)
        {
          goto errout_with_fileobject;
        }
    }

  /* Save the struct spiffs_file_s instance as the file private data */

  filep->f_priv = sfo;

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  offset = 0;
  if ((oflags & (O_APPEND | O_WRONLY)) == (O_APPEND | O_WRONLY))
    {
#warning Missing logic
    }

  /* Save the file position */

  filep->f_pos = offset;

  /* Finish initialization of the file object */

  sfo->fdoffset = offset;
  sem_init(&sfo->exclsem.sem, 0, 1);
  sfo->exclsem.holder = SPIFFS_NO_HOLDER;

  /* Add the new file object to the had of the open file list */

  sfo->flink = fs->ohead;
  fs->ohead  = sfo;

  spiffs_unlock_volume(fs);
  return OK;

errout_with_fileobject:
  kmm_free(sfo);

errout_with_lock:
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_close
 ****************************************************************************/

static int spiffs_close(FAR struct file *filep)
{
  FAR struct spiffs_file_s *sfo;

  finfo("filep: %p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sfo = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(sfo);

  /* Decrement the reference count on the file */

  DEBUGASSERT(sfo->crefs > 0);
  if (sfo->crefs > 0)
    {
      sfo->crefs--;
    }

  filep->f_priv = NULL;

  /* If the reference count decremented to zero and the file has been
   * unlinked, then free resources related to the open file.
   */

  if (sfo->crefs == 0 && (sfo->flags & SFO_FLAG_UNLINKED) != 0)
    {
      /* Free the file object while we hold the lock?  Weird but this
       * should be safe because the object is unlinked and could not
       * have any other references.
       */

      spiffs_file_free(sfo);
      return OK;
    }

  /* Release the lock on the file */

  spiffs_unlock_file(sfo);
  return OK;
}

/****************************************************************************
 * Name: spiffs_read
 ****************************************************************************/

static ssize_t spiffs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct spiffs_file_s *sfo;
  ssize_t nread;
  off_t startpos;
  off_t endpos;

  finfo("filep: %p buffer: %p buflen: %lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sfo = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(sfo);

#warning Missing logic

  /* Release the lock on the file */

  spiffs_unlock_file(sfo);
  return nread;
}

/****************************************************************************
 * Name: spiffs_write
 ****************************************************************************/

static ssize_t spiffs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct spiffs_file_s *sfo;
  ssize_t nwritten;
  off_t startpos;
  off_t endpos;
  int ret;

  finfo("filep: %p buffer: %p buflen: %lu\n",
        filep, buffer, (unsigned long)buflen);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sfo = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(sfo);

#warning Missing logic

  /* Release the lock on the file */

  spiffs_unlock_file(sfo);
  return nwritten;

errout_with_lock:
  spiffs_unlock_file(sfo);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: spiffs_seek
 ****************************************************************************/

static off_t spiffs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct spiffs_file_s *sfo;
  off_t position;

  finfo("filep: %p\n", filep);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sfo = filep->f_priv;

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET: /* The offset is set to offset bytes. */
          position = offset;
          break;

      case SEEK_CUR: /* The offset is set to its current location plus
                      * offset bytes. */
          position = offset + filep->f_pos;
          break;

      case SEEK_END: /* The offset is set to the size of the file plus
                      * offset bytes. */
#warning Missing logic
//          position = offset + sfo->tfo_size;
          break;

      default:
          return -EINVAL;
    }

  /* Attempts to set the position beyond the end of file will
   * work if the file is open for write access.
   *
   * REVISIT: This simple implementation has no per-open storage that
   * would be needed to retain the open flags.
   */

#warning Missing logic

  /* Save the new file position */

  filep->f_pos = position;
  return position;
}

/****************************************************************************
 * Name: spiffs_ioctl
 ****************************************************************************/

static int spiffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct spiffs_file_s *sfo;
  FAR void **ppv = (FAR void**)arg;

  finfo("filep: %p cmd: %d arg: %08lx\n", filep, cmd, arg);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sfo = filep->f_priv;
  DEBUGASSERT(sfo != NULL);

#warning Missing logic

  ferr("ERROR: Invalid cmd: %d\n", cmd);
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
  struct inode            *inode;
  struct spiffs_s_mountpt_s *fs;
  struct spiffs_s_ofile_s   *sf;
  int                      ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  spiffs_semtake(fs);
#warning Missing logic
  spiffs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_dup
 ****************************************************************************/

static int spiffs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct spiffs_file_s *sfo;

  finfo("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL &&
              newp->f_priv == NULL && newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sfo = oldp->f_priv;
  DEBUGASSERT(sfo != NULL);

  /* Increment the reference count (atomically)*/

  spiffs_lock_file(sfo);
  sfo->crefs++;
#warning Missing logic
  spiffs_unlock_file(sfo);

  /* Save a copy of the file object as the dup'ed file.  This
   * simple implementation does not many any per-open data
   * structures so there is not really much to the dup operation.
   */

  newp->f_priv = sfo;
  return OK;
}

/****************************************************************************
 * Name: spiffs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'sfo', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int spiffs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct spiffs_file_s *sfo;

  finfo("Fstat %p\n", buf);
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  sfo = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(sfo);

  /* Return information about the file in the stat buffer.*/
#warning Missing logic

  /* Release the lock on the file and return success. */

  spiffs_unlock_file(sfo);
  return OK;
}

/****************************************************************************
 * Name: spiffs_truncate
 ****************************************************************************/

static int spiffs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct spiffs_file_s *sfo;
  size_t oldsize;
  int ret = OK;

  finfo("filep: %p length: %ld\n", filep, (long)length);
  DEBUGASSERT(filep != NULL && length >= 0);

  /* Recover our private data from the struct file instance */

  sfo = filep->f_priv;

  /* Get exclusive access to the file */

  spiffs_lock_file(sfo);

#warning Missing logic

  /* Release the lock on the file */

errout_with_lock:
  spiffs_unlock_file(sfo);
  return ret;
}

/****************************************************************************
 * Name: spiffs_opendir
 ****************************************************************************/

static int spiffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct spiffs_s *fs;
  FAR struct spiff_directory_s *sdo;
  int ret;

  finfo("mountpt: %p relpath: %s dir: %p\n",
        mountpt, relpath, dir);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && dir != NULL);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Skip over any leading directory separators (shouldn't be any) */

  for (; *relpath == '/'; relpath++)
    {
    }

  /* Find the directory object associated with this relative path.
   * If successful, this action will lock both the parent directory and
   * the file object, adding one to the reference count of both.
   * In the event that -ENOENT, there will still be a reference and
   * lock on the returned directory.
   */

  ret = spiffs_find_directory(fs, relpath, &sdo, NULL);
  if (ret >= 0)
    {
#warning Missing logic
//      dir->u.tmpfs.sfo   = sdo;
//      dir->u.spiffs.tf_index = 0;

      spiffs_unlock_directory(sdo);
    }

  /* Release the lock on the file system and return the result */

  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_closedir
 ****************************************************************************/

static int spiffs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct spiff_directory_s *sdo;

  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Get the directory structure from the dir argument */

#warning Missing logic
//  sdo = dir->u.spiffs.sfo;
  DEBUGASSERT(sdo != NULL);

  /* Decrement the reference count on the directory object */

  spiffs_lock_directory(sdo);
  sdo->crefs--;
  spiffs_unlock_directory(sdo);
  return OK;
}

/****************************************************************************
 * Name: spiffs_readdir
 ****************************************************************************/

static int spiffs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct spiff_directory_s *sdo;
  unsigned int index;
  int ret;

  finfo("mountpt: %p dir: %p\n",  mountpt, dir);
  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Get the directory structure from the dir argument and lock it */

#warning Missing logic
//  sdo = dir->u.spiffs.sfo;
  DEBUGASSERT(sdo != NULL);

  spiffs_lock_directory(sdo);
#warning Missing logic

  spiffs_unlock_directory(sdo);
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

  /* Set the readdir index to zero */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * Name: spiffs_bind
 ****************************************************************************/

static int spiffs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct spiff_directory_s *sdo;
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
  FAR struct spiff_directory_s *sdo;
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

  nxsem_destroy(&sdo->exclsem.sem);
  kmm_free(sdo);

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
  FAR struct spiff_directory_s *sdo;
  size_t inuse;
  size_t avail;
  off_t blkalloc;
  off_t blkused;
  int ret;

  finfo("mountpt: %p buf: %p\n", mountpt, buf);
  DEBUGASSERT(mountpt != NULL && buf != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Set up the memory use for the file system and root directory object */
#warning Missing logid

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
  FAR struct spiff_directory_s *sdo;
  FAR struct spiffs_file_s *sfo = NULL;
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

  ret = spiffs_find_file(fs, relpath, &sfo, &sdo);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  DEBUGASSERT(sfo != NULL);

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

  if (sfo->crefs > 1)
    {
      /* Make the file object as unlinked */

      sfo->flags |= SFO_FLAG_UNLINKED;

      /* Release the reference count on the file object */

      sfo->crefs--;
      spiffs_unlock_file(sfo);
    }

  /* Otherwise we can free resources held by the open file now */

  else
    {
      nxsem_destroy(&sfo->exclsem.sem);
      spiffs_file_free(sfo);
    }

  /* Release the reference and lock on the parent directory */

  sdo->crefs--;
  spiffs_unlock_directory(sdo);
  spiffs_unlock_volume(fs);

  return OK;

errout_with_objects:
#warning Missing logic

  sdo->crefs--;
  spiffs_unlock_directory(sdo);

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
  FAR struct spiffs_s *fs;
  int ret;

  finfo("mountpt: %p relpath: %s mode: %04x\n", mountpt, relpath, mode);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Create the directory. */

  ret = spiffs_create_directory(fs, relpath, NULL);
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_rmdir
 ****************************************************************************/

static int spiffs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct spiffs_s *fs;
  FAR struct spiff_directory_s *parent;
  FAR struct spiff_directory_s *sdo;
  FAR const char *name;
  int ret;

  finfo("mountpt: %p relpath: %s\n", mountpt, relpath);
  DEBUGASSERT(mountpt != NULL && relpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Find the directory object and parent directory associated with this
   * relative path.  If successful, spiffs_find_file will lock both the
   * directory object and the parent directory and take one reference count
   * on each.
   */

  ret = spiffs_find_directory(fs, relpath, &sdo, &parent);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  /* Is the directory empty?  We cannot remove directories that still
   * contain references to file system objects.  No can we remove the
   * directory if there are outstanding references on it (other than
   * our reference).
   */
#warning Missing logic
  if (/* sdo->tdo_nentries > 0 || */ sdo->crefs > 1)
    {
      ret = -EBUSY;
      goto errout_with_objects;
    }

  /* Get the directory name from the relative path */

  name = strrchr(relpath, '/');
  if (name != NULL)
    {
      /* Skip over the fidirectoryle '/' character */

      name++;
    }
  else
    {
      /* The name must lie in the root directory */

      name = relpath;
    }

  /* Remove the directory from parent directory */

  ret = spiffs_remove_dirent(parent, name);
  if (ret < 0)
    {
      goto errout_with_objects;
    }

  /* Free the directory object */

  nxsem_destroy(&sdo->exclsem.sem);
  kmm_free(sdo);

  /* Release the reference and lock on the parent directory */

  parent->crefs--;
  spiffs_unlock_directory(parent);
  spiffs_unlock_volume(fs);

  return OK;

errout_with_objects:
  sdo->crefs--;
  spiffs_unlock_directory(sdo);

  parent->crefs--;
  spiffs_unlock_directory(parent);

errout_with_lock:
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Name: spiffs_rename
 ****************************************************************************/

static int spiffs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                        FAR const char *newrelpath)
{
  FAR struct spiff_directory_s *oldparent;
  FAR struct spiff_directory_s *newparent;
  FAR struct spiffs_s *fs;
  FAR const char *oldname;
  FAR char *newname;
  FAR char *copy;
  int ret;

  finfo("mountpt: %p oldrelpath: %s newrelpath: %s\n",
        mountpt, oldrelpath, newrelpath);
  DEBUGASSERT(mountpt != NULL && oldrelpath != NULL && newrelpath != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Duplicate the newpath variable so that we can modify it */

  copy = strdup(newrelpath);
  if (copy == NULL)
    {
      return -ENOMEM;
    }

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Separate the new path into the new file name and the path to the new
   * parent directory.
   */

  newname = strrchr(copy, '/');
  if (newname == NULL)
    {
      /* No subdirectories... use the root directory */
#warning Missing logic
      newname   = copy;
//      newparent =

      spiffs_lock_directory(newparent);
      newparent->crefs++;
    }
  else
    {
      /* Terminate the parent directory path */

      *newname++ = '\0';

      /* Locate the parent directory that should contain this name.
       * On success, spiffs_find_directory() will lockthe parent
       * directory and increment the reference count.
       */

      ret = spiffs_find_directory(fs, copy, &newparent, NULL);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }

  /* Verify that no object of this name already exists in the destination
   * directory.
   */

  ret = spiffs_find_dirent(newparent, newname);
  if (ret != -ENOENT)
    {
      /* Something with this name already exists in the directory.
       * OR perhaps some fatal error occurred.
       */

      if (ret >= 0)
        {
          ret = -EEXIST;
        }

      goto errout_with_newparent;
    }

  /* Find the old object at oldpath.  If successful, spiffs_find_object()
   * will lock both the object and the parent directory and will increment
   * the reference count on both.
   */
#warning Missing logic

  /* Get the old file name from the relative path */

  oldname = strrchr(oldrelpath, '/');
  if (oldname != NULL)
    {
      /* Skip over the file '/' character */

      oldname++;
    }
  else
    {
      /* The name must lie in the root directory */

      oldname = oldrelpath;
    }

  /* Remove the entry from the parent directory */

  ret = spiffs_remove_dirent(oldparent, oldname);
  if (ret < 0)
    {
      goto errout_with_oldparent;
    }

  /* Add an entry to the new parent directory. */

  ret = spiffs_add_dirent(&newparent, 0, newname);

errout_with_oldparent:
  oldparent->crefs--;
  spiffs_unlock_directory(oldparent);

errout_with_newparent:
  newparent->crefs--;
  spiffs_unlock_directory(newparent);

errout_with_lock:
  spiffs_unlock_volume(fs);
  kmm_free(copy);
  return ret;
}

/****************************************************************************
 * Name: spiffs_stat
 ****************************************************************************/

static int spiffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct spiffs_s *fs;
  int ret;

  finfo("mountpt=%p relpath=%s buf=%p\n", mountpt, relpath, buf);
  DEBUGASSERT(mountpt != NULL && relpath != NULL && buf != NULL);

  /* Get the file system structure from the inode reference. */

  fs = mountpt->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get exclusive access to the file system */

  spiffs_lock_volume(fs);

  /* Find the SPIFFS object at the relpath.  If successful,
   * spiffs_find_object() will lock the object and increment the
   * reference count on the object.
   */
#warning Missing logic

  /* We found it... Return information about the file object in the stat
   * buffer.
   */
#warning Missing logic

  /* Unlock the object and return success */
#warning Missing logic

  ret = OK;

errout_with_fslock:
  spiffs_unlock_volume(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
