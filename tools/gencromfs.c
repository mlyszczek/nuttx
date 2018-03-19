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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_progname;
static const char *g_dirname;
static const char *g_outname;

static FILE *g_outstream;
static FILE *g_nodes;

static const char g_delim[] =
"****************************************************************************";

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void show_usage(void);
static void verify_directory(void);
static void verify_outfile(void);
static void init_outfile(void);
static void init_nodefile(void);
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
  int ret;

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
  fprintf(g_outstream, " %s/\n", g_delim);

  fprintf(g_outstream, "#include <nuttx/config.h>\n\n");
  fprintf(g_outstream, "#include <sys/types.h>\n");
  fprintf(g_outstream, "#include <stdint.h>\n");
  fprintf(g_outstream, "#include <lzf.h>\n");

  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Pre-processor Definitions Types\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);

  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Private Types\n");
  fprintf(g_outstream, " %s/\n\n", g_delim);

  fprintf(g_outstream, "struct cromfs_volume_s\n");
  fprintf(g_outstream, "{\n");
  fprintf(g_outstream, "  uint32_t cv_magic;     /* Must be first.  Must be CROMFS_MAGIC */\n");
  fprintf(g_outstream, "  uint16_t cv_nnodes;    /* Total number of nodes in-use */\n");
  fprintf(g_outstream, "  uint16_t cv_nblocks;   /* Total number of data blocks in-use */\n");
  fprintf(g_outstream, "  size_t cv_root;        /* Offset to the first node in the root file system */\n");
  fprintf(g_outstream, "  size_t cv_fsize;       /* Size of the compressed file system image */\n");
  fprintf(g_outstream, "  size_t cv_bsize;       /* Optimal block size for transfers */\n");
  fprintf(g_outstream, "};\n\n");

  fprintf(g_outstream, "struct cromfs_node_s\n");
  fprintf(g_outstream, "{\n");
  fprintf(g_outstream, "  mode_t cn_mode;        /* File type, attributes, and access mode bits */\n");
  fprintf(g_outstream, "  size_t cn_name;        /* Offset from the beginning of the volume header to the\n");
  fprintf(g_outstream, "                          * node name string.  NUL-terminated. */\n");
  fprintf(g_outstream, "  size_t cn_size;        /* Size of the uncompressed data (in bytes) */\n");
  fprintf(g_outstream, "  size_t cn_peer;        /* Offset to next node in this directory (for readdir()) */\n");
  fprintf(g_outstream, "  union                  /* Must be last */\n");
  fprintf(g_outstream, "  {\n");
  fprintf(g_outstream, "    size_t cn_child;     /* Offset to first node in sub-directory (directories only) */\n");
  fprintf(g_outstream, "    size_t cn_link;      /* Offset to an arbitrary node (for hard link) */\n");
  fprintf(g_outstream, "    size_t cn_blocks;    /* Offset to first block of compressed data (for read) */\n");
  fprintf(g_outstream, "  } u;\n");
  fprintf(g_outstream, "};\n\n");
}

static void init_nodefile(void)
{
  fprintf(g_outstream, "/%s\n", g_delim);
  fprintf(g_outstream, " * Private Data\n");
  fprintf(g_outstream, " %s/\n", g_delim);
}

static void process_direntry(const char *dirpath, struct dirent *direntry)
{
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
          process_direntry(dirpath, direntry);
        }
    }
  while (direntry != NULL);

  /* Free any allocatin that we made above */

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
  char *ptr;
  int fd;

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

  fd = mkstemp("/tmp/gencromfs-XXXXXX");
  if (fd < 0)
    {
      fprintf(stderr, "Failed to create temporary file: %s\n", strerror(errno));
      exit(1);
    }

  g_nodes = fdopen(fd, "w");
  if (!g_nodes)
    {
      fprintf(stderr, "fdopen for tmp file failed: %s\n", strerror(errno));
      exit(1);
    }

  /* Set up the initial boilerplate at the beginning of each file */

  init_outfile();
  init_nodefile();

  /* Then traverse each entry in the directory, generating node data for each
   * directory entry encountered.
   */

  traverse_directory(g_dirname, NULL);

  /* Now append the volume header to output file */

  /* Finally append the nodes to the output file */

  fclose(g_outstream);
  fclose(g_nodes);
  return 0;
}
