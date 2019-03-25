/****************************************************************************
 * include/nuttx/mtp/mtp.h
 * Media Transfer Protocol definitions
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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



/* Simple Type Summary */

#define MTP_DATA_UNDEF       0x0000 /* Undefined */
#define MTP_DATA_INT8        0x0001 /* Signed 8-bit integer */
#define MTP_DATA_UINT8       0x0002 /* Unsigned 8-bit integer */
#define MTP_DATA_INT16       0x0003 /* Signed 16-bit integer */
#define MTP_DATA_UINT16      0x0004 /* Unsigned 16-bit integer */
#define MTP_DATA_INT32       0x0005 /* Signed 32-bit integer */
#define MTP_DATA_UINT32      0x0006 /* Unsigned 32-bit integer */
#define MTP_DATA_INT64       0x0007 /* Signed 64-bit integer */
#define MTP_DATA_UINT64      0x0008 /* Unsigned 64-bit integer */
#define MTP_DATA_INT128      0x0009 /* Signed 128-bit integer */
#define MTP_DATA_UINT128     0x000a /* Unsigned 128-bit integer */
#define MTP_DATA_AINT8       0x4001 /* Array of signed 8-bit integers */
#define MTP_DATA_AUINT8      0x4002 /* Array of unsigned 8-bit integers */
#define MTP_DATA_AINT16      0x4003 /* Array of signed 16-bit integers */
#define MTP_DATA_AUINT16     0x4004 /* Array of unsigned 16-bit integers */
#define MTP_DATA_AINT32      0x4005 /* Array of signed 32-bit integers */
#define MTP_DATA_AUINT32     0x4006 /* Array of unsigned 32-bit integers */
#define MTP_DATA_AINT64      0x4007 /* Array of signed 64-bit integers */
#define MTP_DATA_AUINT64     0x4008 /* Array of unsigned 64-bit integers */
#define MTP_DATA_AINT128     0x4009 /* Array of signed 128-bit integers */
#define MTP_DATA_AUINT128    0x400a /* Array of unsigned 128-bit integers */
#define MTP_DATA_STR         0xffff /* Variable-length Unicode string */

/* Association Type */

#define MTP_ASSOC_UNDEF      0x0000 /* Undefined */
#define MTP_ASSOC_GENFOLDER  0x0001 /* Unused by PTP, used by MTP to indicate type if folder */
#define MTP_ASSOC_ALBUM      0x0002 /* Album - Reserved */
#define MTP_ASSOC_TIME_SEQ   0x0003 /* Time Sequence - Default Playback Delta */
#define MTP_ASSOC_HORIZ_PAN  0x0004 /* Horizontal Panoramic - Unused */
#define MTP_ASSOC_VERT_PAN   0x0005 /* Vertical Panoramic - Unused */
#define MTP_ASSOC_2D_PAN     0x0006 /* 2D Panoramic - Images per row */
#define MTP_ASSOC_ANC_DATA   0x0007 /* Ancillary Data - Undefined */

