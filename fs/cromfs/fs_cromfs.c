/****************************************************************************
 * fs/cromfs/fs_cromfs.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <lzf.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/binfmt/builtin.h>

#include "cromfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_CROMFS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents an open, regular file */

struct cromfs_file_s
{
  FAR const struct cromfs_node_s *ff_node;  /* The open file node */
  FAR uint8_t *ff_buffer;                   /* Decompression buffer */
};

#if 0 /* Not used */
/* This is the form of the callback from cromfs_foreach_node(): */

typedefCODE  int (*cromfs_foreach_t)(FAR struct cromfs_volume_s *fs,
                                     FAR const struct cromfs_node_s *node,
                                     FAR void *arg);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static FAR void *cromfs_offset2addr(FAR struct cromfs_volume_s *fs,
                                    size_t offset);
static size_t  cromfs_addr2offset(FAR struct cromfs_volume_s *fs,
                                  FAR void *addr);
#if 0 /* Not used */
static int     cromfs_foreach_node(FAR struct cromfs_volume_s *fs,
                                   cromfs_foreach_t callback, FAR void *arg);
#endif

/* Common file system methods */

static int     cromfs_open(FAR struct file *filep, const char *relpath,
                           int oflags, mode_t mode);
static int     cromfs_close(FAR struct file *filep);
static ssize_t cromfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static int     cromfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int     cromfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int     cromfs_fstat(FAR const struct file *filep, FAR struct stat *buf);

static int     cromfs_opendir(struct inode *mountpt, const char *relpath,
                              struct fs_dirent_s *dir);
static int     cromfs_readdir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int     cromfs_rewinddir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);

static int     cromfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                           FAR void **handle);
static int     cromfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                             unsigned int flags);
static int     cromfs_statfs(FAR struct inode *mountpt,
                             FAR struct statfs *buf);

static int     cromfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                           FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations cromfs_operations =
{
  cromfs_open,       /* open */
  cromfs_close,      /* close */
  cromfs_read,       /* read */
  NULL,              /* write */
  NULL,              /* seek */
  cromfs_ioctl,      /* ioctl */

  NULL,              /* sync */
  cromfs_dup,        /* dup */
  cromfs_fstat,      /* fstat */
  NULL,              /* truncate */

  cromfs_opendir,    /* opendir */
  NULL,              /* closedir */
  cromfs_readdir,    /* readdir */
  cromfs_rewinddir,  /* rewinddir */

  cromfs_bind,       /* bind */
  cromfs_unbind,     /* unbind */
  cromfs_statfs,     /* statfs */

  NULL,              /* unlink */
  NULL,              /* mkdir */
  NULL,              /* rmdir */
  NULL,              /* rename */
  cromfs_stat        /* stat */
};

/* The CROMFS uses a global, in-memory instance of the file system image
 * rather than a ROMDISK as does, same the ROMFS file system.  This is
 * primarily because the compression logic needs contiguous, in-memory
 * data.  One consequence of this is that there can only only be a single
 * CROMFS file system in the build.
 *
 * This is the address of the single CROMFS file system image:
 */

extern const struct cromfs_volume_s g_cromfs_image;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cromfs_offset2addr
 ****************************************************************************/

static FAR void *cromfs_offset2addr(FAR struct cromfs_volume_s *fs,
                                    size_t offset)
{
  /* Zero offset is a specials case:  It corresponds to a NULL address */

  if (offset == 0)
    {
      return NULL;
    }

  /* The root node lies at the beginning of the CROMFS image so we can
   * convert the offset into the image by simply adding the the address
   * of the root node.
   */

  DEBUGASSERT(offset < fs->cv_fsize);
  return (FAR void *)((FAR uint8_t *)fs + offset);
}

/****************************************************************************
 * Name: cromfs_offset2addr
 ****************************************************************************/

static size_t cromfs_addr2offset(FAR struct cromfs_volume_s *fs,
                                 FAR void *addr)
{
  uintptr_t start;
  uintptr_t target;
  size_t offset;

  /* NULL is a specials case:  It corresponds to offset zero */

  if (addr == NULL)
    {
      return 0;
    }

  /* Make sure that the address is "after" the start of file system image. */

  start  = (uintptr_t)fs;
  target = (uintptr_t)addr;
  DEBUGASSERT(target >= start);

  /* Get the non-zero offset and make sure that the offset is within the file
   * system image.
   */

  offset = target - start;
  DEBUGASSERT(offset < fs->cv_fsize);
  return offset;
}

/****************************************************************************
 * Name: cromfs_foreach_node
 ****************************************************************************/

#if 0 /* Not used */
static int cromfs_foreach_node(FAR struct cromfs_volume_s *fs,
                              cromfs_foreach_t callback, FAR void *arg)
{
  FAR const struct cromfs_node_s *node;
  int ret = OK;

  node = (FAR const struct cromfs_node_s *)cromfs_offset2addr(fs, fs->cv_root);
  while (node != NULL)
    {
      ret = callback(fs, node, arg);
      if (ret != OK)
        {
          return ret;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cromfs_open
 ****************************************************************************/

static int cromfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_node_s *node;
  FAR struct cromfs_file_s *ff;
  int ret;

  finfo("Open '%s'\n", relpath);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* CROMFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

 /* Locate the node for this relative path */

  node = NULL;
  ret  = cromfs_findnode(fs, &node, relpath);
  if (ret < 0)
    {
      /* Nothing exists at that relative path (or a really bad error occurred) */

      return ret;
    }

  DEBUGASSERT(node != NULL);

  /* Verify that the node is a regular file */

  if (!S_ISREG(node->cn_mode))
    {
      return -EISDIR;
    }

  /* Create an instance of the file private date to describe the opened
   * file.
   */

  ff = (FAR struct cromfs_file_s *)kmm_zalloc(sizeof(struct cromfs_file_s));
  if (ff == NULL)
    {
      return -ENOMEM;
    }

  /* Create a file buffer to support partial sector accesses */

  ff->ff_buffer = (FAR uint8_t *)kmm_malloc(fs->cv_bsize);
  if (!ff->ff_buffer)
    {
      kmm_free(ff);
      return -ENOMEM;
    }

  /* Save the node in the open file instance */

  ff->ff_node = node;

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)ff;
  return OK;
}

/****************************************************************************
 * Name: cromfs_close
 ****************************************************************************/

static int cromfs_close(FAR struct file *filep)
{
  FAR struct cromfs_file_s *ff;

  finfo("Closing\n");
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the open file instance from the file structure */

  ff = filep->f_priv;
  DEBUGASSERT(ff->ff_node != NULL && ff->ff_buffer != NULL);

  /* Free all resources consumed by the opened file */

  kmm_free(ff->ff_buffer);
  kmm_free(ff);

  return OK;
}

/****************************************************************************
 * Name: cromfs_read
 ****************************************************************************/

static ssize_t cromfs_read(FAR struct file *filep, char *buffer,
                           size_t buflen)
{
  FAR struct cromfs_file_s *ff;

  finfo("Read %d bytes from offset %d\n", buflen, filep->f_pos);
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Get the open file instance from the file structure */

  ff = filep->f_priv;
  DEBUGASSERT(ff->ff_node != NULL && ff->ff_buffer != NULL);

  /* Reading is not yet supported.  Just return end-of-file for now */
#warning Missing logic
  return 0;
}

/****************************************************************************
 * Name: cromfs_ioctl
 ****************************************************************************/

static int cromfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  /* No IOCTL commands yet supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: cromfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int cromfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_file_s *oldff;
  FAR struct cromfs_file_s *newff;

  finfo("Dup %p->%p\n", oldp, newp);
  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL &&
              newp->f_priv == NULL && newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  fs = (FAR struct cromfs_volume_s *)oldp->f_inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Get the open file instance from the file structure */

  oldff = oldp->f_priv;
  DEBUGASSERT(oldff->ff_node != NULL && oldff->ff_buffer != NULL);

  /* Allocate and initialize an new open file instance referring to the
   * same node.
   */

  newff = (FAR struct cromfs_file_s *)kmm_zalloc(sizeof(struct cromfs_file_s));
  if (newff == NULL)
    {
      return -ENOMEM;
    }

  /* Create a file buffer to support partial sector accesses */

  newff->ff_buffer = (FAR uint8_t *)kmm_malloc(fs->cv_bsize);
  if (newff->ff_buffer == NULL)
    {
      kmm_free(newff);
      return -ENOMEM;
    }

  /* Save the node in the open file instance */

  newff->ff_node = oldff->ff_node;

  /* Copy the index from the old to the new file structure */

  newp->f_priv = newff;
  return OK;
}

/****************************************************************************
 * Name: cromfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int cromfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_file_s *ff;
  size_t fsize;
  size_t bsize;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * volume private data from the inode structure
   */

  ff              = filep->f_priv;
  DEBUGASSERT(ff->ff_node != NULL && ff->ff_buffer != NULL);

  inode           = filep->f_inode;
  fs              = inode->i_private;

  /* Return the stat info */

  fsize           = ff->ff_node->cn_size;
  bsize           = fs->cv_bsize;

  buf->st_mode    = ff->ff_node->cn_mode;
  buf->st_size    = fsize;
  buf->st_blksize = bsize;
  buf->st_blocks  = (fsize + (bsize - 1)) / bsize;
  buf->st_atime   = 0;
  buf->st_mtime   = 0;
  buf->st_ctime   = 0;

  return OK;
}

/****************************************************************************
 * Name: cromfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int cromfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_node_s *node;
  int ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

 /* Locate the node for this relative path */

  node = NULL;
  ret  = cromfs_findnode(fs, &node, relpath);
  if (ret < 0)
    {
      /* Nothing exists at that relative path (or a really bad error occurred) */

      return ret;
    }

  DEBUGASSERT(node != NULL);

  /* Verify that the node is a directory */

  if (!S_ISDIR(node->cn_mode))
    {
      return -ENOTDIR;
    }

  /* Set the start node and next node to the first entry in the directory */

  dir->u.cromfs.cr_firstoffset = (size_t)cromfs_addr2offset(fs, node);
  dir->u.cromfs.cr_curroffset  = dir->u.cromfs.cr_firstoffset;
  return OK;
}

/****************************************************************************
 * Name: cromfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int cromfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_node_s *node;
  FAR char *name;
  size_t offset;

  finfo("mountpt: %p dir: %p\n", mountpt, dir);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Have we reached the end of the directory */

  offset = dir->u.cromfs.cr_curroffset;
  if (offset == 0)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      finfo("Entry %d: End of directory\n", offset);
      return -ENOENT;
    }

  /* Convert the offset to a node address (assuming that everything is in-memory) */

  node = (FAR struct cromfs_node_s *)cromfs_offset2addr(fs, offset);

  /* Save the filename and file type */

  name = (FAR char *)cromfs_offset2addr(fs, node->cn_name);
  finfo("Entry %lu: \"%s\"\n", (unsigned long)offset, name);
  strncpy(dir->fd_dir.d_name, name, NAME_MAX + 1);

  switch (node->cn_mode & s_IFTGT)
    {
      case S_IFDIR:  /* Directory */
        dir->fd_dir.d_type = DTYPE_DIRECTORY;
        break;

      case S_IFREG:  /* Regular file */
        dir->fd_dir.d_type = DTYPE_FILE;
        break;

      case S_IFIFO:  /* FIFO */
      case S_IFCHR:  /* Character driver */
      case S_IFBLK:  /* Block driver */
   /* case S_IFSOCK:    Socket */
      case S_IFMQ:   /* Message queue */
      case S_IFSEM:  /* Semaphore */
      case S_IFSHM:  /* Shared memory */
      default:
        DEBUGPANIC();
        dir->fd_dir.d_type = DTYPE_UNKNOWN;
        break;
    }

  /* Set up the next directory entry offset.  NOTE that we could use the
   * standard f_pos instead of our own private fb_index.
   */

  dir->u.cromfs.cr_curroffset = node->cn_peer;
  return OK;
}

/****************************************************************************
 * Name: cromfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int cromfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  finfo("Entry\n");

  dir->u.cromfs.cr_curroffset  = dir->u.cromfs.cr_firstoffset;
  return OK;
}

/****************************************************************************
 * Name: cromfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int cromfs_bind(FAR struct inode *blkdriver, const void *data,
                      void **handle)
{
  finfo("blkdriver: %p data: %p handle: %p\n", blkdriver, data, handle);
  DEBUGASSERT(blkdriver == NULL && handle != NULL);

  /* Return the new file system handle */

  *handle = (FAR void *)&g_cromfs_image;
  return OK;
}

/****************************************************************************
 * Name: cromfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int cromfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags)
{
   finfo("handle: %p blkdriver: %p flags: %02x\n",
          handle, blkdriver, flags);
  return OK;
}

/****************************************************************************
 * Name: cromfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int cromfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  FAR struct cromfs_volume_s *fs;

  finfo("mountpt: %p buf: %p\n", mountpt, buf);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs             = mountpt->i_private;

  /* Fill in the statfs info. */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = CROMFS_MAGIC;
  buf->f_namelen = NAME_MAX;
  buf->f_bsize   = fs->cv_bsize;
  buf->f_blocks  = fs->cv_nblocks;
  buf->f_files   = fs->cv_nnodes;
  return OK;
}

/****************************************************************************
 * Name: cromfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int cromfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  FAR struct cromfs_volume_s *fs;
  FAR struct cromfs_node_s *node;
  int ret;

  finfo("mountptr: %p relpath: %s buf: %p\n", mountpt, relpath, buf);

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && buf != NULL );
  memset(buf, 0, sizeof(struct stat));

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Locate the node for this relative path */

  node = NULL;
  ret  = cromfs_findnode(fs, &node, relpath);
  if (ret >= 0)
    {
      DEBUGASSERT(node != NULL);

      /* Return the struct stat info associate with this node */

      buf->st_mode    = node->cn_mode;
      buf->st_size    = node->cn_size;
      buf->st_blksize = fs->cv_bsize;
      buf->st_blocks  = (node->cn_size + (fs->cv_bsize - 1)) / fs->cv_bsize;
      ret             = OK;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_CROMFS */

