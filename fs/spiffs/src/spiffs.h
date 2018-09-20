/****************************************************************************
 * fs/spiffs.h/spiffs.h
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

#ifndef __FS_SPIFFS_SRC_SPIFFS_H
#define __FS_SPIFFS_SRC_SPIFFS_H

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "spiffs_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPIFFS_ERR_NOT_MOUNTED          -10000
#define SPIFFS_ERR_FULL                 -10001
#define SPIFFS_ERR_NOT_FOUND            -10002
#define SPIFFS_ERR_END_OF_OBJECT        -10003
#define SPIFFS_ERR_DELETED              -10004
#define SPIFFS_ERR_NOT_FINALIZED        -10005
#define SPIFFS_ERR_NOT_INDEX            -10006
#define SPIFFS_ERR_OUT_OF_FILE_DESCS    -10007
#define SPIFFS_ERR_FILE_CLOSED          -10008
#define SPIFFS_ERR_FILE_DELETED         -10009
#define SPIFFS_ERR_BAD_DESCRIPTOR       -10010
#define SPIFFS_ERR_IS_INDEX             -10011
#define SPIFFS_ERR_IS_FREE              -10012
#define SPIFFS_ERR_INDEX_SPAN_MISMATCH  -10013
#define SPIFFS_ERR_DATA_SPAN_MISMATCH   -10014
#define SPIFFS_ERR_INDEX_REF_FREE       -10015
#define SPIFFS_ERR_INDEX_REF_LU         -10016
#define SPIFFS_ERR_INDEX_REF_INVALID    -10017
#define SPIFFS_ERR_INDEX_FREE           -10018
#define SPIFFS_ERR_INDEX_LU             -10019
#define SPIFFS_ERR_INDEX_INVALID        -10020
#define SPIFFS_ERR_NOT_WRITABLE         -10021
#define SPIFFS_ERR_NOT_READABLE         -10022
#define SPIFFS_ERR_CONFLICTING_NAME     -10023
#define SPIFFS_ERR_NOT_CONFIGURED       -10024

#define SPIFFS_ERR_NOT_A_FS             -10025
#define SPIFFS_ERR_MOUNTED              -10026
#define SPIFFS_ERR_ERASE_FAIL           -10027
#define SPIFFS_ERR_MAGIC_NOT_POSSIBLE   -10028

#define SPIFFS_ERR_NO_DELETED_BLOCKS    -10029

#define SPIFFS_ERR_FILE_EXISTS          -10030

#define SPIFFS_ERR_NOT_A_FILE           -10031
#define SPIFFS_ERR_RO_NOT_IMPL          -10032
#define SPIFFS_ERR_RO_ABORTED_OPERATION -10033
#define SPIFFS_ERR_PROBE_TOO_FEW_BLOCKS -10034
#define SPIFFS_ERR_PROBE_NOT_A_FS       -10035
#define SPIFFS_ERR_NAME_TOO_LONG        -10036

#define SPIFFS_ERR_IX_MAP_UNMAPPED      -10037
#define SPIFFS_ERR_IX_MAP_MAPPED        -10038
#define SPIFFS_ERR_IX_MAP_BAD_RANGE     -10039

#define SPIFFS_ERR_SEEK_BOUNDS          -10040

#define SPIFFS_ERR_INTERNAL             -10050

#define SPIFFS_ERR_TEST                 -10100

#define SPIFFS_TYPE_FILE                (1)
#define SPIFFS_TYPE_DIR                 (2)
#define SPIFFS_TYPE_HARD_LINK           (3)
#define SPIFFS_TYPE_SOFT_LINK           (4)

#ifndef SPIFFS_LOCK
#  define SPIFFS_LOCK(fs)
#endif

#ifndef SPIFFS_UNLOCK
#  define SPIFFS_UNLOCK(fs)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* spiffs file descriptor index type. must be signed */

typedef int16_t spiffs_file;

/* spiffs file descriptor flags */

typedef uint16_t spiffs_flags;

/* spiffs file mode */

typedef uint16_t spiffs_mode;

/* object type */

typedef uint8_t spiffs_obj_type;

struct spiffs_t;

#if SPIFFS_HAL_CALLBACK_EXTRA

/* spi read call function type */

typedef int32_t(*spiffs_read)(struct spiffs_t * fs, uint32_t addr, uint32_t size,
                            uint8_t * dst);

/* spi write call function type */

typedef int32_t(*spiffs_write)(struct spiffs_t * fs, uint32_t addr, uint32_t size,
                             uint8_t * src);

/* spi erase call function type */

typedef int32_t(*spiffs_erase)(struct spiffs_t * fs, uint32_t addr, uint32_t size);

#else  /* SPIFFS_HAL_CALLBACK_EXTRA */

/* spi read call function type */

typedef int32_t(*spiffs_read)(uint32_t addr, uint32_t size, uint8_t * dst);

/* spi write call function type */

typedef int32_t(*spiffs_write)(uint32_t addr, uint32_t size, uint8_t * src);

/* spi erase call function type */

typedef int32_t(*spiffs_erase)(uint32_t addr, uint32_t size);
#endif  /* SPIFFS_HAL_CALLBACK_EXTRA */

/* file system check callback report operation */

typedef enum
{
  SPIFFS_CHECK_LOOKUP = 0,
  SPIFFS_CHECK_INDEX,
  SPIFFS_CHECK_PAGE
} spiffs_check_type;

/* file system check callback report type */

typedef enum
{
  SPIFFS_CHECK_PROGRESS = 0,
  SPIFFS_CHECK_ERROR,
  SPIFFS_CHECK_FIX_INDEX,
  SPIFFS_CHECK_FIX_LOOKUP,
  SPIFFS_CHECK_DELETE_ORPHANED_INDEX,
  SPIFFS_CHECK_DELETE_PAGE,
  SPIFFS_CHECK_DELETE_BAD_FILE
} spiffs_check_report;

/* file system check callback function */

#if SPIFFS_HAL_CALLBACK_EXTRA
typedef void (*spiffs_check_callback)(struct spiffs_t * fs,
                                      spiffs_check_type type,
                                      spiffs_check_report report, uint32_t arg1,
                                      uint32_t arg2);
#else  /* SPIFFS_HAL_CALLBACK_EXTRA */
typedef void (*spiffs_check_callback)(spiffs_check_type type,
                                     spiffs_check_report report, uint32_t arg1,
                                     uint32_t arg2);
#endif  /* SPIFFS_HAL_CALLBACK_EXTRA */

/* file system listener callback operation */

typedef enum
{
  /* the file has been created */

  SPIFFS_CB_CREATED = 0,

  /* the file has been updated or moved to another page */

  SPIFFS_CB_UPDATED,

  /* the file has been deleted */

  SPIFFS_CB_DELETED
} spiffs_fileop_type;

/* file system listener callback function */

typedef void (*spiffs_file_callback)(struct spiffs_t * fs,
                                     spiffs_fileop_type op,
                                     spiffs_obj_id obj_id,
                                     spiffs_page_ix pix);

/* spiffs spi configuration struct */

typedef struct
{
  /* physical read function */

  spiffs_read hal_read_f;

  /* physical write function */

  spiffs_write hal_write_f;

  /* physical erase function */

  spiffs_erase hal_erase_f;

  /* physical size of the spi flash */

  uint32_t phys_size;

  /* physical offset in spi flash used for spiffs, must be on block boundary
   */

  uint32_t phys_addr;

  /* physical size when erasing a block */

  uint32_t phys_erase_block;

  /* logical size of a block, must be on physical block size boundary and
   * must never be less than a physical block
   */

  uint32_t log_block_size;

  /* logical size of a page, must be at least log_block_size / 8 */

  uint32_t log_page_size;
} spiffs_config;

typedef struct spiffs_t
{
  /* file system configuration */

  spiffs_config cfg;

  /* number of logical blocks */

  uint32_t block_count;

  /* cursor for free blocks, block index */

  spiffs_block_ix free_cursor_block_ix;

  /* cursor for free blocks, entry index */

  int free_cursor_obj_lu_entry;

  /* cursor when searching, block index */

  spiffs_block_ix cursor_block_ix;

  /* cursor when searching, entry index */

  int cursor_obj_lu_entry;

  /* primary work buffer, size of a logical page */

  uint8_t *lu_work;

  /* secondary work buffer, size of a logical page */

  uint8_t *work;

  /* file descriptor memory area */

  uint8_t *fd_space;

  /* available file descriptors */

  uint32_t fd_count;

  /* last error */

  int32_t err_code;

  /* current number of free blocks */

  uint32_t free_blocks;

  /* current number of busy pages */

  uint32_t stats_p_allocated;

  /* current number of deleted pages */

  uint32_t stats_p_deleted;

  /* flag indicating that garbage collector is cleaning */

  uint8_t cleaning;

  /* max erase count amongst all blocks */

  spiffs_obj_id max_erase_count;

#if CONFIG_SPIFFS_GCDBG
  uint32_t stats_gc_runs;
#endif

  /* cache memory */

  FAR void *cache;

  /* cache size */

  uint32_t cache_size;
#if CONFIG_SPIFFS_CACHEDBG
  uint32_t cache_hits;
  uint32_t cache_misses;
#endif

  /* check callback function */

  spiffs_check_callback check_cb_f;

  /* file callback function */

  spiffs_file_callback file_cb_f;

  /* mounted flag */

  uint8_t mounted;

  /* user data */

  void *user_data;

  /* config magic */

  uint32_t config_magic;
} spiffs;

/* spiffs file status struct */

typedef struct
{
  spiffs_obj_id obj_id;
  uint32_t size;
  spiffs_obj_type type;
  spiffs_page_ix pix;
  uint8_t name[SPIFFS_NAME_MAX];
#if SPIFFS_OBJ_META_LEN
  uint8_t meta[SPIFFS_OBJ_META_LEN];
#endif
} spiffs_stat;

struct spiffs_dirent
{
  spiffs_obj_id obj_id;
  uint8_t name[SPIFFS_NAME_MAX];
  spiffs_obj_type type;
  uint32_t size;
  spiffs_page_ix pix;
#if SPIFFS_OBJ_META_LEN
  uint8_t meta[SPIFFS_OBJ_META_LEN];
#endif
};

typedef struct
{
  spiffs *fs;
  spiffs_block_ix block;
  int entry;
} spiffs_DIR;

#if SPIFFS_IX_MAP

typedef struct
{
  /* buffer with looked up data pixes */

  spiffs_page_ix *map_buf;

  /* precise file byte offset */

  uint32_t offset;

  /* start data span index of lookup buffer */

  spiffs_span_ix start_spix;

  /* end data span index of lookup buffer */

  spiffs_span_ix end_spix;
} spiffs_ix_map;

#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH
/* Special function. This takes a spiffs config struct and returns the number
 * of blocks this file system was formatted with. This function relies on
 * that following info is set correctly in given config struct:
 *
 * phys_addr, log_page_size, and log_block_size.
 *
 * Also, hal_read_f must be set in the config struct.
 *
 * One must be sure of the correct page size and that the physical address is
 * correct in the probed file system when calling this function. It is not
 * checked if the phys_addr actually points to the start of the file system,
 * so one might get a false positive if entering a phys_addr somewhere in the
 * middle of the file system at block boundary. In addition, it is not checked
 * if the page size is actually correct. If it is not, weird file system sizes
 * will be returned.
 *
 * If this function detects a file system it returns the assumed file system
 * size, which can be used to set the phys_size.
 *
 * Otherwise, it returns an error indicating why it is not regarded as a file
 * system.
 *
 * Note: this function is not protected with SPIFFS_LOCK and SPIFFS_UNLOCK
 * macros. It returns the error code directly, instead of as read by errno.
 *
 * @param config        essential parts of the physical and logical
 *                      configuration of the file system.
 */

int32_t SPIFFS_probe_fs(spiffs_config * config);
#endif  /* SPIFFS_USE_MAGIC && SPIFFS_USE_MAGIC_LENGTH */

/* Initializes the file system dynamic parameters and mounts the filesystem.
 * If SPIFFS_USE_MAGIC is enabled the mounting may fail with SPIFFS_ERR_NOT_A_FS
 * if the flash does not contain a recognizable file system.
 * In this case, SPIFFS_format must be called prior to remounting.
 * @param fs            the file system struct
 * @param config        the physical and logical configuration of the file system
 * @param work          a memory work buffer comprising 2*config->log_page_size
 *                      bytes used throughout all file system operations
 * @param fd_space      memory for file descriptors
 * @param fd_space_size memory size of file descriptors
 * @param cache         memory for cache, may be null
 * @param cache_size    memory size of cache
 * @param check_cb_f    callback function for reporting during consistency checks
 */

int32_t SPIFFS_mount(spiffs * fs, spiffs_config * config, uint8_t * work,
                     uint8_t * fd_space, uint32_t fd_space_size,
                     void *cache, uint32_t cache_size,
                     spiffs_check_callback check_cb_f);

/* Unmounts the file system. All file handles will be flushed of any
 * cached writes and closed.
 * @param fs            the file system struct
 */

void SPIFFS_unmount(spiffs * fs);

/* Creates a new file.
 * @param fs            the file system struct
 * @param path          the path of the new file
 * @param mode          ignored, for posix compliance
 */

int32_t SPIFFS_creat(spiffs * fs, const char *path, spiffs_mode mode);

/* Opens/creates a file.
 * @param fs            the file system struct
 * @param path          the path of the new file
 * @param flags         the flags for the open command, can be combinations of
 *                      O_APPEND, O_TRUNC, O_CREAT, O_RDONLY,
 *                      O_WRONLY, O_RDWR, O_DIRECT, O_EXCL
 * @param mode          ignored, for posix compliance
 */

spiffs_file SPIFFS_open(spiffs * fs, const char *path, spiffs_flags flags,
                        spiffs_mode mode);

/* Opens a file by given dir entry.
 * Optimization purposes, when traversing a file system with SPIFFS_readdir
 * a normal SPIFFS_open would need to traverse the filesystem again to find
 * the file, whilst SPIFFS_open_by_dirent already knows where the file resides.
 * @param fs            the file system struct
 * @param e             the dir entry to the file
 * @param flags         the flags for the open command, can be combinations of
 *                      O_APPEND, O_TRUNC, O_CREAT, SPIFFS_RD_ONLY,
 *                      SPIFFS_WR_ONLY, O_RDWR, O_DIRECT.
 *                      O_CREAT will have no effect in this case.
 * @param mode          ignored, for posix compliance
 */

  spiffs_file SPIFFS_open_by_dirent(spiffs * fs, struct spiffs_dirent *e,
                                  spiffs_flags flags, spiffs_mode mode);

/* Opens a file by given page index.
 * Optimization purposes, opens a file by directly pointing to the page
 * index in the spi flash.
 * If the page index does not point to a file header SPIFFS_ERR_NOT_A_FILE
 * is returned.
 * @param fs            the file system struct
 * @param page_ix       the page index
 * @param flags         the flags for the open command, can be combinations of
 *                      O_APPEND, O_TRUNC, O_CREAT, SPIFFS_RD_ONLY,
 *                      SPIFFS_WR_ONLY, O_RDWR, O_DIRECT.
 *                      O_CREAT will have no effect in this case.
 * @param mode          ignored, for posix compliance
 */

spiffs_file SPIFFS_open_by_page(spiffs * fs, spiffs_page_ix page_ix,
                                spiffs_flags flags, spiffs_mode mode);

/* Reads from given filehandle.
 * @param fs            the file system struct
 * @param fh            the filehandle
 * @param buf           where to put read data
 * @param len           how much to read
 * @returns number of bytes read, or -1 if error
 */

int32_t SPIFFS_read(spiffs * fs, spiffs_file fh, void *buf, int32_t len);

/* Writes to given filehandle.
 * @param fs            the file system struct
 * @param fh            the filehandle
 * @param buf           the data to write
 * @param len           how much to write
 * @returns number of bytes written, or -1 if error
 */

int32_t SPIFFS_write(spiffs * fs, spiffs_file fh, void *buf, int32_t len);

/* Moves the read/write file offset. Resulting offset is returned or negative if error.
 * lseek(fs, fd, 0, SEEK_CUR) will thus return current offset.
 * @param fs            the file system struct
 * @param fh            the filehandle
 * @param offs          how much/where to move the offset
 * @param whence        if SEEK_SET, the file offset shall be set to offset bytes
 *                      if SEEK_CUR, the file offset shall be set to its current location plus offset
 *                      if SEEK_END, the file offset shall be set to the size of the file plus offset, which should be negative
 */

int32_t SPIFFS_lseek(spiffs * fs, spiffs_file fh, int32_t offs, int whence);

/* Removes a file by path
 * @param fs            the file system struct
 * @param path          the path of the file to remove
 */
  int32_t SPIFFS_remove(spiffs * fs, const char *path);

/* Removes a file by filehandle
 * @param fs            the file system struct
 * @param fh            the filehandle of the file to remove
 */

int32_t SPIFFS_fremove(spiffs * fs, spiffs_file fh);

/* Gets file status by path
 * @param fs            the file system struct
 * @param path          the path of the file to stat
 * @param s             the stat struct to populate
 */

int32_t SPIFFS_stat(spiffs * fs, const char *path, spiffs_stat * s);

/* Gets file status by filehandle
 * @param fs            the file system struct
 * @param fh            the filehandle of the file to stat
 * @param s             the stat struct to populate
 */

int32_t SPIFFS_fstat(spiffs * fs, spiffs_file fh, spiffs_stat * s);

/*
 * Flushes all pending write operations from cache for given file
 * @param fs            the file system struct
 * @param fh            the filehandle of the file to flush
 */

int32_t SPIFFS_fflush(spiffs * fs, spiffs_file fh);

/* Closes a filehandle. If there are pending write operations, these are finalized before closing.
 * @param fs            the file system struct
 * @param fh            the filehandle of the file to close
 */

int32_t SPIFFS_close(spiffs * fs, spiffs_file fh);

/* Renames a file
 * @param fs            the file system struct
 * @param old           path of file to rename
 * @param newPath       new path of file
 */

int32_t SPIFFS_rename(spiffs * fs, const char *old, const char *newPath);

#if SPIFFS_OBJ_META_LEN

/* Updates file's metadata
 * @param fs            the file system struct
 * @param path          path to the file
 * @param meta          new metadata. must be SPIFFS_OBJ_META_LEN bytes long.
 */

int32_t SPIFFS_update_meta(spiffs * fs, const char *name, const void *meta);

/* Updates file's metadata
 * @param fs            the file system struct
 * @param fh            file handle of the file
 * @param meta          new metadata. must be SPIFFS_OBJ_META_LEN bytes long.
 */

int32_t SPIFFS_fupdate_meta(spiffs * fs, spiffs_file fh, const void *meta);
#endif

/* Returns last error of last file operation.
 * @param fs            the file system struct
 */

int SPIFFS_errno(spiffs *fs);

/* Clears last error.
 * @param fs            the file system struct
 */

void SPIFFS_clearerr(spiffs * fs);

/* Opens a directory stream corresponding to the given name.
 * The stream is positioned at the first entry in the directory.
 * On hydrogen builds the name argument is ignored as hydrogen builds always correspond
 * to a flat file structure - no directories.
 * @param fs            the file system struct
 * @param name          the name of the directory
 * @param d             pointer the directory stream to be populated
 */

spiffs_DIR *SPIFFS_opendir(spiffs * fs, const char *name, spiffs_DIR * d);

/* Closes a directory stream
 * @param d             the directory stream to close
 */

int32_t SPIFFS_closedir(spiffs_DIR * d);

/* Reads a directory into given spifs_dirent struct.
 * @param d             pointer to the directory stream
 * @param e             the dirent struct to be populated
 * @returns null if error or end of stream, else given dirent is returned
 */

struct spiffs_dirent *SPIFFS_readdir(spiffs_DIR * d, struct spiffs_dirent *e);

/* Runs a consistency check on given filesystem.
 * @param fs            the file system struct
 */

int32_t SPIFFS_check(spiffs * fs);

/* Returns number of total bytes available and number of used bytes.
 * This is an estimation, and depends on if there a many files with little
 * data or few files with much data.
 * NB: If used number of bytes exceeds total bytes, a SPIFFS_check should
 * run. This indicates a power loss in midst of things. In worst case
 * (repeated powerlosses in mending or gc) you might have to delete some files.
 *
 * @param fs            the file system struct
 * @param total         total number of bytes in filesystem
 * @param used          used number of bytes in filesystem
 */

int32_t SPIFFS_info(spiffs * fs, uint32_t * total, uint32_t * used);

/* Formats the entire file system. All data will be lost.
 * The filesystem must not be mounted when calling this.
 *
 * NB: formatting is awkward. Due to backwards compatibility, SPIFFS_mount
 * MUST be called prior to formatting in order to configure the filesystem.
 * If SPIFFS_mount succeeds, SPIFFS_unmount must be called before calling
 * SPIFFS_format.
 * If SPIFFS_mount fails, SPIFFS_format can be called directly without calling
 * SPIFFS_unmount first.
 *
 * @param fs            the file system struct
 */

int32_t SPIFFS_format(spiffs * fs);

/* Returns nonzero if spiffs is mounted, or zero if unmounted.
 * @param fs            the file system struct
 */

uint8_t SPIFFS_mounted(spiffs * fs);

/* Tries to find a block where most or all pages are deleted, and erase that
 * block if found. Does not care for wear levelling. Will not move pages
 * around.
 * If parameter max_free_pages are set to 0, only blocks with only deleted
 * pages will be selected.
 *
 * NB: the garbage collector is automatically called when spiffs needs free
 * pages. The reason for this function is to give possibility to do background
 * tidying when user knows the system is idle.
 *
 * Use with care.
 *
 * Setting max_free_pages to anything larger than zero will eventually wear
 * flash more as a block containing free pages can be erased.
 *
 * Will set err_no to OK if a block was found and erased,
 * SPIFFS_ERR_NO_DELETED_BLOCK if no matching block was found,
 * or other error.
 *
 * @param fs             the file system struct
 * @param max_free_pages maximum number allowed free pages in block
 */

int32_t SPIFFS_gc_quick(spiffs * fs, uint16_t max_free_pages);

/* Will try to make room for given amount of bytes in the filesystem by moving
 * pages and erasing blocks.
 * If it is physically impossible, err_no will be set to SPIFFS_ERR_FULL. If
 * there already is this amount (or more) of free space, SPIFFS_gc will
 * silently return. It is recommended to call SPIFFS_info before invoking
 * this method in order to determine what amount of bytes to give.
 *
 * NB: the garbage collector is automatically called when spiffs needs free
 * pages. The reason for this function is to give possibility to do background
 * tidying when user knows the system is idle.
 *
 * Use with care.
 *
 * @param fs            the file system struct
 * @param size          amount of bytes that should be freed
 */

int32_t SPIFFS_gc(spiffs * fs, uint32_t size);

/* Check if EOF reached.
 * @param fs            the file system struct
 * @param fh            the filehandle of the file to check
 */

int32_t SPIFFS_eof(spiffs * fs, spiffs_file fh);

/* Get position in file.
 * @param fs            the file system struct
 * @param fh            the filehandle of the file to check
 */

int32_t SPIFFS_tell(spiffs * fs, spiffs_file fh);

/* Registers a callback function that keeps track on operations on file
 * headers. Do note, that this callback is called from within internal spiffs
 * mechanisms. Any operations on the actual file system being callbacked from
 * in this callback will mess things up for sure - do not do this.
 * This can be used to track where files are and move around during garbage
 * collection, which in turn can be used to build location tables in ram.
 * Used in conjuction with SPIFFS_open_by_page this may improve performance
 * when opening a lot of files.
 * Must be invoked after mount.
 *
 * @param fs            the file system struct
 * @param cb_func       the callback on file operations
 */

int32_t SPIFFS_set_file_callback_func(spiffs * fs,
                                      spiffs_file_callback cb_func);

#if SPIFFS_IX_MAP
/* Maps the first level index lookup to a given memory map.
 * This will make reading big files faster, as the memory map will be used for
 * looking up data pages instead of searching for the indices on the physical
 * medium. When mapping, all affected indicies are found and the information is
 * copied to the array.
 * Whole file or only parts of it may be mapped. The index map will cover file
 * contents from argument offset until and including arguments (offset+len).
 * It is valid to map a longer range than the current file size. The map will
 * then be populated when the file grows.
 * On garbage collections and file data page movements, the map array will be
 * automatically updated. Do not tamper with the map array, as this contains
 * the references to the data pages. Modifying it from outside will corrupt any
 * future readings using this file descriptor.
 * The map will no longer be used when the file descriptor closed or the file
 * is unmapped.
 * This can be useful to get faster and more deterministic timing when reading
 * large files, or when seeking and reading a lot within a file.
 * @param fs      the file system struct
 * @param fh      the file handle of the file to map
 * @param map     a spiffs_ix_map struct, describing the index map
 * @param offset  absolute file offset where to start the index map
 * @param len     length of the mapping in actual file bytes
 * @param map_buf the array buffer for the look up data - number of required
 *                elements in the array can be derived from function
 *                SPIFFS_bytes_to_ix_map_entries given the length
 */

int32_t SPIFFS_ix_map(spiffs * fs, spiffs_file fh, spiffs_ix_map * map,
                      uint32_t offset, uint32_t len, spiffs_page_ix * map_buf);

/* Unmaps the index lookup from this filehandle. All future readings will
 * proceed as normal, requiring reading of the first level indices from
 * physical media.
 * The map and map buffer given in function SPIFFS_ix_map will no longer be
 * referenced by spiffs.
 * It is not strictly necessary to unmap a file before closing it, as closing
 * a file will automatically unmap it.
 * @param fs      the file system struct
 * @param fh      the file handle of the file to unmap
 */

int32_t SPIFFS_ix_unmap(spiffs * fs, spiffs_file fh);

/* Moves the offset for the index map given in function SPIFFS_ix_map. Parts or
 * all of the map buffer will repopulated.
 * @param fs      the file system struct
 * @param fh      the mapped file handle of the file to remap
 * @param offset  new absolute file offset where to start the index map
 */

int32_t SPIFFS_ix_remap(spiffs * fs, spiffs_file fh, uint32_t offs);

/* Utility function to get number of spiffs_page_ix entries a map buffer must
 * contain on order to map given amount of file data in bytes.
 * See function SPIFFS_ix_map and SPIFFS_ix_map_entries_to_bytes.
 * @param fs      the file system struct
 * @param bytes   number of file data bytes to map
 * @return        needed number of elements in a spiffs_page_ix array needed to
 *                map given amount of bytes in a file
 */

int32_t SPIFFS_bytes_to_ix_map_entries(spiffs * fs, uint32_t bytes);

/*
 * Utility function to amount of file data bytes that can be mapped when
 * mapping a file with buffer having given number of spiffs_page_ix entries.
 * See function SPIFFS_ix_map and SPIFFS_bytes_to_ix_map_entries.
 * @param fs      the file system struct
 * @param map_page_ix_entries   number of entries in a spiffs_page_ix array
 * @return        amount of file data in bytes that can be mapped given a map
 *                buffer having given amount of spiffs_page_ix entries
 */

int32_t SPIFFS_ix_map_entries_to_bytes(spiffs * fs, uint32_t map_page_ix_entries);

#endif  /* SPIFFS_IX_MAP */

#if defined(__cplusplus)
}
#endif

#endif  /* __FS_SPIFFS_SRC_SPIFFS_H */
