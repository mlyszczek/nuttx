/****************************************************************************
 * tools/gencromfs.c
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

#define _GNU_SOURCE 1
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <lzf.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef USE_MKSTEMP

#define TMP_NAMLEN        32         /* Actually only 22 */
#ifdef USE_MKSTEMP
#  define TMP_NAME        "/tmp/gencromfs-XXXXXX"
#else
#  define TMP_NAME        "/tmp/gencromfs-%06u"
#endif

#define NUTTX_IXOTH       (1 << 0)   /* Bits 0-2: Permissions for others: RWX */
#define NUTTX_IWOTH       (1 << 1)
#define NUTTX_IROTH       (1 << 2)

#define NUTTX_IRXOTH      (NUTTX_IROTH | NUTTX_IXOTH)

#define NUTTX_IXGRP       (1 << 3)   /* Bits 3-5: Group permissions: RWX */
#define NUTTX_IWGRP       (1 << 4)
#define NUTTX_IRGRP       (1 << 5)

#define NUTTX_IRXGRP      (NUTTX_IRGRP | NUTTX_IXGRP)

#define NUTTX_IXUSR       (1 << 6)   /* Bits 6-8: Owner permissions: RWX */
#define NUTTX_IWUSR       (1 << 7)
#define NUTTX_IRUSR       (1 << 8)

#define NUTTX_IRWXUSR     (NUTTX_IRUSR | NUTTX_IWUSR | NUTTX_IXUSR)
#define NUTTX_IRWUSR      (NUTTX_IRUSR | NUTTX_IWUSR)

#define NUTTX_IFDIR       (2 << 12)
#define NUTTX_IFREG       (4 << 12)

#define NUTTX_IFLNK       (1 << 15)  /* Bit 15: Symbolic link */

#define DIR_MODEFLAGS     (NUTTX_IFDIR | NUTTX_IRWXUSR | NUTTX_IRXGRP | NUTTX_IRXOTH)
#define DIRLINK_MODEFLAGS (NUTTX_IFLNK | DIR_MODEFLAGS)
#define FILE_MODEFLAGS    (NUTTX_IFREG | NUTTX_IRWUSR | NUTTX_IRGRP | NUTTX_IROTH)

#define CROMFS_MAGIC      0x4d4f5243
#define CROMFS_BLOCKSIZE  512

#define HEX_PER_BREAK     8
#define HEX_PER_LINE      16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cromfs_volume_s
{
  uint32_t cv_magic;     /* Must be first.  Must be CROMFS_MAGIC */
  uint16_t cv_nnodes;    /* Total number of nodes in-use */
  uint16_t cv_nblocks;   /* Total number of data blocks in-use */
  size_t cv_root;        /* Offset to the first node in the root file system */
  size_t cv_fsize;       /* Size of the compressed file system image */
  size_t cv_bsize;       /* Optimal block size for transfers */
};

struct cromfs_node_s
{
  mode_t cn_mode;        /* File type, attributes, and access mode bits */
  size_t cn_name;        /* Offset from the beginning of the volume header to the
                          * node name string.  NUL-terminated. */
  size_t cn_size;        /* Size of the uncompressed data (in bytes) */
  size_t cn_peer;        /* Offset to next node in this directory (for readdir()) */
  union
  {
    size_t cn_child;     /* Offset to first node in sub-directory (directories only) */
    size_t cn_link;      /* Offset to an arbitrary node (for hard link) */
    size_t cn_blocks;    /* Offset to first block of compressed data (for read) */
  } u;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_progname;       /* Name of this program */
static char *g_dirname;        /* Source directory path */
static char *g_outname;        /* Output file path */

static FILE *g_outstream;      /* Main output stream */
static FILE *g_tmpstream;      /* Temporary file output stream */

static const char g_delim[] =
  "*************************************************************************"
  "***********************";

static size_t g_offset;        /* Current image offset */
static size_t g_diroffset;     /* Offset for '.' */
static size_t g_parent_offset; /* Offset for '..' */

static unsigned int g_nnodes;  /* Number of nodes generated */
static unsigned int g_nblocks; /* Number of blocks of data generated */
static unsigned int g_nhex;    /* Number of hex characters on output line */
#ifndef USE_MKSTEMP
static unsigned int g_ntmps;   /* Number temporary files */
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void show_usage(void);
static void verify_directory(void);
static void verify_outfile(void);
static void init_outfile(void);
static FILE *open_tmpfile(void);
#ifndef USE_MKSTEMP
static void unlink_tmpfiles(void);
#endif
static void append_tmpfile(FILE *dest, FILE *src);
static void dump_hexbuffer(FILE *stream, const void *buffer, unsigned int nbytes);
static void dump_nextline(FILE *stream);
static void gen_dirlink(const char *name, size_t tgtoffs);
static void gen_directory(const char *path, const char *name);
static void gen_file(const char *path, const char *name, struct stat *buf);
static void process_direntry(const char *dirpath, struct dirent *direntry);
static void traverse_directory(const char *dirpath, const char *subdir);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "USAGE: %s <dir-path> <out-file>\n", g_progname);
  exit(1);
}

static void verify_directory(void)
{
  struct stat buf;
  int len;
  int ret;

  /* Trim any trailing '/' characters from the directory path. */

  len = strlen(g_dirname);
  while (len > 1 && g_dirname[len-1] == '/')
    {
      g_dirname[len-1] = '\0';
      len--;
    }

  if (len < 1)
    {
      fprintf(stderr, "ERROR: Source <dir-path> %s is invalid\n",
              g_dirname);
      show_usage();
    }

  /* stat the source directory containing the file system image */

  ret = stat(g_dirname, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      if (errcode == ENOENT)
        {
          fprintf(stderr, "ERROR: Source <dir-path> %s does not exist\n",
                  g_dirname);
        }
      else
        {
          fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
                 g_dirname, strerror(errcode));
        }

      show_usage();
    }

  /* Verify that the source is, indeed, a directory */

  else if (!S_ISDIR(buf.st_mode))
    {
      fprintf(stderr, "ERROR: Source <dir-path> %s is not a directory\n",
              g_dirname);
    }
}

static void verify_outfile(void)
{
  struct stat buf;
  int ret;

  /* stat the destination file */

  ret = stat(g_outname, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      if (errcode != ENOENT)
        {
          fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
                 g_outname, strerror(errcode));
          show_usage();
        }
    }

  /* Something exists at this path.  Verify that the destination is a regular file */

  else if (!S_ISREG(buf.st_mode))
    {
      fprintf(stderr, "ERROR: Destination <out-file> %s exists\n",
              g_outname);
      show_usage();
    }
  else
    {
      printf("Existing file %s will be replaced\n", g_outname);
    }
}

static void init_outfile(void)
{
  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * %s\n", g_outname);
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " *   Copyright (C) 2018 Gregory Nutt. All rights reserved.\n");
  fprintf(g_outstream, " *   Author: Gregory Nutt <gnutt@nuttx.org>\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " * Redistribution and use in source and binary forms, with or without\n");
  fprintf(g_outstream, " * modification, are permitted provided that the following conditions\n");
  fprintf(g_outstream, " * are met:\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " * 1. Redistributions of source code must retain the above copyright\n");
  fprintf(g_outstream, " *    notice, this list of conditions and the following disclaimer.\n");
  fprintf(g_outstream, " * 2. Redistributions in binary form must reproduce the above copyright\n");
  fprintf(g_outstream, " *    notice, this list of conditions and the following disclaimer in\n");
  fprintf(g_outstream, " *    the documentation and/or other materials provided with the\n");
  fprintf(g_outstream, " *    distribution.\n");
  fprintf(g_outstream, " * 3. Neither the name NuttX nor the names of its contributors may be\n");
  fprintf(g_outstream, " *    used to endorse or promote products derived from this software\n");
  fprintf(g_outstream, " *    without specific prior written permission.\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n");
  fprintf(g_outstream, " * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n");
  fprintf(g_outstream, " * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS\n");
  fprintf(g_outstream, " * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n");
  fprintf(g_outstream, " * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,\n");
  fprintf(g_outstream, " * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,\n");
  fprintf(g_outstream, " * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS\n");
  fprintf(g_outstream, " * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED\n");
  fprintf(g_outstream, " * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT\n");
  fprintf(g_outstream, " * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN\n");
  fprintf(g_outstream, " * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE\n");
  fprintf(g_outstream, " * POSSIBILITY OF SUCH DAMAGE.\n");
  fprintf(g_outstream, " *\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);

  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Included Files\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);

  fprintf(g_outstream, "#include <nuttx/config.h>\n\n");
  fprintf(g_outstream, "#include <stdint.h>\n\n");

  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Private Data\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);
}

static FILE *open_tmpfile(void)
{
  FILE *tmpstream;
#ifdef USE_MKSTEMP
  int fd;

  fd = mkstemp(TMP_NAME);
  if (fd < 0)
    {
      fprintf(stderr, "Failed to create temporary file: %s\n", strerror(errno));
      exit(1);
    }

  tmpstream = fdopen(fd, "w+");
  if (!tmpstream)
    {
      fprintf(stderr, "fdopen for tmp file failed: %s\n", strerror(errno));
      exit(1);
    }

#else
  char tmpname[TMP_NAMLEN];

  snprintf(tmpname, TMP_NAMLEN, TMP_NAME, g_ntmps);
  g_ntmps++;

  tmpstream = fopen(tmpname, "w+");
  if (!tmpstream)
    {
      fprintf(stderr, "fopen for tmp file %s failed: %s\n",
              tmpname, strerror(errno));
      exit(1);
    }
#endif

  return tmpstream;
}

#ifndef USE_MKSTEMP
static void unlink_tmpfiles(void)
{
  char tmpname[TMP_NAMLEN];
  unsigned int i;

  for (i = 0; i < g_ntmps; i++)
    {
      snprintf(tmpname, TMP_NAMLEN, TMP_NAME, i);
      (void)unlink(tmpname);
    }
}
#endif

static void append_tmpfile(FILE *dest, FILE *src)
{
  uint8_t iobuffer[1024];
  size_t nread;

  /* Rewind the source directory to be beginning.  We assume that the dest
   * is already at the end.
   */

  rewind(src);

  /* Then append the source to the destination */

  do
    {
      nread = fread(iobuffer, 1, 1024, src);
      if (nread > 0)
        {
          (void)fwrite(iobuffer, 1, nread, dest);
        }
    }
  while (nread > 0);

  /* We can now close the src temporary file */

  fclose(src);
}

static void dump_hexbuffer(FILE *stream, const void *buffer,
                           unsigned int nbytes)
{
  uint8_t *ptr = (uint8_t *)buffer;

  while (nbytes > 0)
    {
      if (g_nhex == 0 || g_nhex == HEX_PER_BREAK)
        {
          fprintf(stream, " ");
        }

      fprintf(stream, " 0x%02x,", *ptr++);

      if (++g_nhex >= HEX_PER_LINE)
        {
          fprintf(stream, "\n");
          g_nhex = 0;
        }

      nbytes--;
    }
}

static void dump_nextline(FILE *stream)
{
  if (g_nhex > 0)
    {
      fprintf(stream, "\n");
      g_nhex = 0;
    }
}

static void gen_dirlink(const char *name, size_t tgtoffs)
{
  struct cromfs_node_s node;
  int namlen;

  namlen          = strlen(name) + 1;

  /* Generate the hardlink node */

  node.cn_mode    = DIRLINK_MODEFLAGS;

  g_offset       += sizeof(struct cromfs_node_s);
  node.cn_name    = g_offset;
  node.cn_size    = 0;

  g_offset       += namlen;
  node.cn_peer    = g_offset;
  node.u.cn_link  = tgtoffs;

  fprintf(g_tmpstream, "\n  /* %s: Hard link */\n\n", name);
  dump_nextline(g_tmpstream);
  dump_hexbuffer(g_tmpstream, &node, sizeof(struct cromfs_node_s));
  dump_hexbuffer(g_tmpstream, name, namlen);

  g_nnodes++;
}

static void gen_directory(const char *path, const char *name)
{
  struct cromfs_node_s node;
  size_t save_offset        = g_offset;
  size_t save_diroffset     = g_diroffset;
  size_t save_parent_offset = g_parent_offset;
  FILE *save_tmpstream      = g_tmpstream;
  FILE *subtree_stream;
  int namlen;

  namlen          = strlen(name) + 1;

  /* Open a new temporary file */

  subtree_stream  = open_tmpfile();
  g_tmpstream     = subtree_stream;

  /* Update offsets for the subdirectory */

  g_parent_offset = g_diroffset;  /* New offset for '..' */
  g_diroffset     = g_offset;     /* New offset for '.' */

  /* Update the offset to account for the file node which we have not yet
   * written (we can't, we don't have enough information yet)
   */

  g_offset       += sizeof(struct cromfs_node_s) + namlen;

  /* Generate the '.' and '..' links for the directory (in the new temporary file) */

  gen_dirlink(".", g_diroffset);
  gen_dirlink("..", g_parent_offset);

  /* Then recurse to generate all of the nodes for the subtree */

  traverse_directory(path, name);

  /* When traverse_directory() returns, all of the nodes in the sub-tree under
   * 'name' will have been written to the new tmpfile.  g_offset is correct,
   * but other settings are not.
   *
   * Restore the state.
   */

  g_tmpstream     = save_tmpstream;
  g_diroffset     = save_diroffset;
  g_parent_offset = save_parent_offset;

  /* Generate the directory node */

  node.cn_mode    = DIR_MODEFLAGS;

  save_offset    += sizeof(struct cromfs_node_s);
  node.cn_name    = save_offset;
  node.cn_size    = 0;

  save_offset    += namlen;
  node.cn_peer    = g_offset;
  node.u.cn_child = save_offset;

  fprintf(g_tmpstream, "\n  /* Directory %s */\n\n", path);
  dump_nextline(g_tmpstream);
  dump_hexbuffer(g_tmpstream, &node, sizeof(struct cromfs_node_s));
  dump_hexbuffer(g_tmpstream, name, namlen);

  g_nnodes++;

  /* Now append the sub-tree nodes in the new tmpfile to the previous tmpfile */

  append_tmpfile(g_tmpstream, subtree_stream);
}

static void gen_file(const char *path, const char *name, struct stat *buf)
{
}

static void process_direntry(const char *dirpath, struct dirent *direntry)
{
  struct stat buf;
  char *path;
  int ret;

  asprintf(&path, "%s/%s", dirpath, direntry->d_name);

  ret = stat(path, &buf);
  if (ret < 0)
    {
      int errcode = errno;
      if (errcode == ENOENT)
        {
          fprintf(stderr, "ERROR: Directory entry %s does not exist\n", path);
        }
      else
        {
          fprintf(stderr, "ERROR: stat(%s) failed: %s\n",
                 path, strerror(errcode));
        }

      show_usage();
    }

  /* Verify that the source is, indeed, a directory */

  else if (S_ISDIR(buf.st_mode))
    {
      gen_directory(path, direntry->d_name);
    }
  else if (S_ISREG(buf.st_mode))
    {
      gen_file(path, direntry->d_name, &buf);
    }
  else
    {
      fprintf(stderr, "Omitting entry %s\n", path);
    }

  free(path);
}

static void traverse_directory(const char *dirpath, const char *subdir)
{
  DIR *dirp;
  struct dirent *direntry;
  char *diralloc = NULL;
  const char *dirptr;

  /* The first time this function is called, subdir will be NULL */

  if (subdir)
    {
      asprintf(&diralloc, "%s/%s", dirpath, subdir);
      dirptr = diralloc;
    }
  else
    {
      dirptr = dirpath;
    }

  /* Open the directory */

  dirp = opendir(dirptr);
  if (dirp == NULL)
    {
      fprintf(stderr, "ERROR: opendir(%s) failed: %s\n",
              dirptr, strerror(errno));
      show_usage();
    }

  /* Visit each entry in the directory */

  do
    {
      direntry = readdir(dirp);
      if (direntry != NULL)
        {
          /* Skip the '.' and '..' hard links */

          if (strcmp(direntry->d_name, ".") != 0 &&
              strcmp(direntry->d_name, "..") != 0)
            {
              /* Process the directory entry */

              process_direntry(dirptr, direntry);
            }
        }
    }
  while (direntry != NULL);

  /* Free any allocation that we made above */

  if (diralloc)
    {
      free(diralloc);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct cromfs_volume_s vol;
  char *ptr;

  /* Verify arguments */

  ptr = strrchr(argv[0], '/');
  g_progname = ptr == NULL ? argv[0] : ptr + 1;

  if (argc != 3)
    {
      fprintf(stderr, "Unexpected number of arguments\n");
      show_usage();
    }

  g_dirname  = argv[1];
  g_outname  = argv[2];

  verify_directory();
  verify_outfile();;

  g_outstream = fopen(g_outname, "w");
  if (!g_outstream)
    {
      fprintf(stderr, "open %s failed: %s\n", g_outname, strerror(errno));
      exit(1);
    }

  g_tmpstream = open_tmpfile();

  /* Set up the initial boilerplate at the beginning of each file */

  init_outfile();

  /* Set up some initial offsets */

  g_offset        = sizeof(struct cromfs_volume_s);  /* Current image offset */
  g_diroffset     = sizeof(struct cromfs_volume_s);  /* Offset for '.' */
  g_parent_offset = sizeof(struct cromfs_volume_s);  /* Offset for '..' */

  /* Generate the '.' link for the root directory (it can't have a '..') */

  gen_dirlink(".", g_diroffset);

  /* Then traverse each entry in the directory, generating node data for each
   * directory entry encountered.
   */

  traverse_directory(g_dirname, NULL);

  /* Now append the volume header to output file */

  fprintf(g_outstream, "/* CROMFS image */\n\n");
  fprintf(g_outstream, "uint8_t g_cromfs_image[] =\n");
  fprintf(g_outstream, "{\n");
  fprintf(g_outstream, "  /* Volume header */\n\n");

  vol.cv_magic    = CROMFS_MAGIC;
  vol.cv_nnodes   = g_nnodes;
  vol.cv_nblocks  = g_nblocks;
  vol.cv_root     = sizeof(struct cromfs_volume_s);
  vol.cv_fsize    = g_offset;
  vol.cv_bsize    = CROMFS_BLOCKSIZE;

  g_nhex          = 0;
  dump_hexbuffer(g_outstream, &vol, sizeof(struct cromfs_volume_s));

  dump_nextline(g_outstream);
  fprintf(g_outstream, "\n  /* Root directory */\n");

  /* Finally append the nodes to the output file */

  append_tmpfile(g_outstream,g_tmpstream);
  fprintf(g_outstream, "\n};\n");

  fclose(g_outstream);
#ifndef USE_MKSTEMP
  unlink_tmpfiles();
#endif
  return 0;
}
