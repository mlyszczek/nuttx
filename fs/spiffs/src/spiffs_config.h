/****************************************************************************
 * fs/spiffs.h/spiffs_config.h
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

#ifndef __FS_SPIFFS_SRC_SPIFFS_CONFIG_H
#define __FS_SPIFFS_SRC_SPIFFS_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <unistd.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */

#ifdef CONFIG_SPIFFS_GCDBG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define spiffs_gcinfo(format, ...)      _info(format, ##__VA_ARGS__)
#  else
#    define spiffs_gcinfo                   _info
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define spiffs_gcinfo(format, ...)
#  else
#    define spiffs_gcinfo                   (void)
#  endif
#endif

#ifdef CONFIG_SPIFFS_CACHEDBG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define spiffs_cacheinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  else
#    define spiffs_cacheinfo                _info
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define spiffs_cacheinfo(format, ...)
#  else
#    define spiffs_cacheinfo                (void)
#  endif
#endif

#ifdef CONFIG_SPIFFS_CHECKDBG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define spiffs_checkinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  else
#    define spiffs_checkinfo                _info
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define spiffs_checkinfo(format, ...)
#  else
#    define spiffs_checkinfo                (void)
#  endif
#endif

/* Defines spiffs debug print formatters */

/* some general signed number */

#ifndef _SPIPRIi
#  define _SPIPRIi   "%d"
#endif

/* address */

#ifndef _SPIPRIad
#  define _SPIPRIad  "%08x"
#endif

/* block */

#ifndef _SPIPRIbl
#  define _SPIPRIbl  "%04x"
#endif

/* page */

#ifndef _SPIPRIpg
#  define _SPIPRIpg  "%04x"
#endif

/* span index */

#ifndef _SPIPRIsp
#  define _SPIPRIsp  "%04x"
#endif

/* file descriptor */

#ifndef _SPIPRIfd
#  define _SPIPRIfd  "%d"
#endif

/* file object id */

#ifndef _SPIPRIid
#  define _SPIPRIid  "%04x"
#endif

/* file flags */

#ifndef _SPIPRIfl
#  define _SPIPRIfl  "%02x"
#endif

/* Always check header of each accessed page to ensure consistent state.
 * If enabled it will increase number of reads, will increase flash.
 */

#ifndef SPIFFS_PAGE_CHECK
#  define SPIFFS_PAGE_CHECK             1
#endif

/* Define maximum number of gc runs to perform to reach desired free pages. */

#ifndef SPIFFS_GC_MAX_RUNS
#  define SPIFFS_GC_MAX_RUNS            5
#endif

/* Garbage collecting examines all pages in a block which and sums up
 * to a block score. Deleted pages normally gives positive score and
 * used pages normally gives a negative score (as these must be moved).
 * To have a fair wear-leveling, the erase age is also included in score,
 * whose factor normally is the most positive.
 * The larger the score, the more likely it is that the block will
 * picked for garbage collection.
 */

/* Garbage collecting heuristics - weight used for deleted pages. */

#ifndef SPIFFS_GC_HEUR_W_DELET
#  define SPIFFS_GC_HEUR_W_DELET        (5)
#endif

/* Garbage collecting heuristics - weight used for used pages. */

#ifndef SPIFFS_GC_HEUR_W_USED
#  define SPIFFS_GC_HEUR_W_USED         (-1)
#endif

/* Garbage collecting heuristics - weight used for time between
 * last erased and erase of this block.
 */

#ifndef SPIFFS_GC_HEUR_W_ERASE_AGE
#  define SPIFFS_GC_HEUR_W_ERASE_AGE    (50)
#endif

/* Object name maximum length. Note that this length include the
 * zero-termination character, meaning maximum string of characters
 * can at most be SPIFFS_NAME_MAX - 1.
 */

#ifndef SPIFFS_NAME_MAX
#  define SPIFFS_NAME_MAX              (32)
#endif

#if SPIFFS_NAME_MAX > NAME_MAX
#  undef SPIFFS_NAME_MAX
#  define SPIFFS_NAME_MAX               NAME_MAX
#endif

/* Maximum length of the metadata associated with an object.
 * Setting to non-zero value enables metadata-related API but also
 * changes the on-disk format, so the change is not backward-compatible.
 *
 * Do note: the meta length must never exceed
 * logical_page_size - (SPIFFS_NAME_MAX + 64)
 *
 * This is derived from following:
 * logical_page_size - (SPIFFS_NAME_MAX + sizeof(struct spiffs_page_header_s) +
 * spiffs_object_ix_header fields + at least some LUT entries)
 */

#ifndef SPIFFS_OBJ_META_LEN
#  define SPIFFS_OBJ_META_LEN           (0)
#endif

/* Size of buffer allocated on stack used when copying data.
 * Lower value generates more read/writes. No meaning having it bigger
 * than logical page size.
 */

#ifndef SPIFFS_COPY_BUFFER_STACK
#  define SPIFFS_COPY_BUFFER_STACK      (64)
#endif

/* Enable this to have an identifiable spiffs filesystem. This will look for
 * a magic in all sectors to determine if this is a valid spiffs system or
 * not on mount point. If not, SPIFFS_format must be called prior to mounting
 * again.
 */

#ifndef SPIFFS_USE_MAGIC
#  define SPIFFS_USE_MAGIC              (0)
#endif

#if SPIFFS_USE_MAGIC
/* Only valid when SPIFFS_USE_MAGIC is enabled. If SPIFFS_USE_MAGIC_LENGTH is
 * enabled, the magic will also be dependent on the length of the filesystem.
 * For example, a filesystem configured and formatted for 4 megabytes will not
 * be accepted for mounting with a configuration defining the filesystem as 2
 * megabytes.
 */

#ifndef SPIFFS_USE_MAGIC_LENGTH
#  define SPIFFS_USE_MAGIC_LENGTH         (0)
#endif
#endif

/* SPIFFS_LOCK and SPIFFS_UNLOCK protects spiffs from re-entrancy on api level
 * These should be defined on a multi-threaded system
 */

/* define this to enter a mutex if you're running on a multi-threaded system */

#ifndef SPIFFS_LOCK
#  define SPIFFS_LOCK(fs)
#endif

/* define this to exit a mutex if you're running on a multi-threaded system */

#ifndef SPIFFS_UNLOCK
#  define SPIFFS_UNLOCK(fs)
#endif

/* Enable this if your target needs aligned data for index tables */

#ifndef SPIFFS_ALIGNED_OBJECT_INDEX_TABLES
#define SPIFFS_ALIGNED_OBJECT_INDEX_TABLES       0
#endif

/* Temporal file cache hit score. Each time a file is opened, all cached files
 * will lose one point. If the opened file is found in cache, that entry will
 * gain SPIFFS_TEMPORAL_CACHE_HIT_SCORE points. One can experiment with this
 * value for the specific access patterns of the application. However, it must
 * be between 1 (no gain for hitting a cached entry often) and 255.
 */

#ifndef SPIFFS_TEMPORAL_CACHE_HIT_SCORE
#  define SPIFFS_TEMPORAL_CACHE_HIT_SCORE     4
#endif

/* Enable to be able to map object indices to memory.
 * This allows for faster and more deterministic reading if cases of reading
 * large files and when changing file offset by seeking around a lot.
 * When mapping a file's index, the file system will be scanned for index pages
 * and the info will be put in memory provided by user. When reading, the
 * memory map can be looked up instead of searching for index pages on the
 * medium. This way, user can trade memory against performance.
 * Whole, parts of, or future parts not being written yet can be mapped. The
 * memory array will be owned by spiffs and updated accordingly during garbage
 * collecting or when modifying the indices. The latter is invoked by when the
 * file is modified in some way. The index buffer is tied to the file
 * descriptor.
 */

#ifndef SPIFFS_IX_MAP
#  define SPIFFS_IX_MAP                       1
#endif

/* By default SPIFFS in some cases relies on the property of NOR flash that bits
 * cannot be set from 0 to 1 by writing and that controllers will ignore such
 * bit changes. This results in fewer reads as SPIFFS can in some cases perform
 * blind writes, with all bits set to 1 and only those it needs reset set to 0.
 * Most of the chips and controllers allow this behavior, so the default is to
 * use this technique. If your controller is one of the rare ones that don't,
 * turn this option on and SPIFFS will perform a read-modify-write instead.
 */

#ifndef SPIFFS_NO_BLIND_WRITES
  #define SPIFFS_NO_BLIND_WRITES              0
#endif

#endif /* __FS_SPIFFS_SRC_SPIFFS_CONFIG_H */
