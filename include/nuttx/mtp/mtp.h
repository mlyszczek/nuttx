/****************************************************************************
 * include/nuttx/mtp/mtp.h
 * Media Transfer Protocol definitions
 *
 *   Copyright (C) 2019 Alan Carvalho de Assis. All rights reserved.
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

#define MTP_DATA_UNDEF              0x0000 /* Undefined */
#define MTP_DATA_INT8               0x0001 /* Signed 8-bit integer */
#define MTP_DATA_UINT8              0x0002 /* Unsigned 8-bit integer */
#define MTP_DATA_INT16              0x0003 /* Signed 16-bit integer */
#define MTP_DATA_UINT16             0x0004 /* Unsigned 16-bit integer */
#define MTP_DATA_INT32              0x0005 /* Signed 32-bit integer */
#define MTP_DATA_UINT32             0x0006 /* Unsigned 32-bit integer */
#define MTP_DATA_INT64              0x0007 /* Signed 64-bit integer */
#define MTP_DATA_UINT64             0x0008 /* Unsigned 64-bit integer */
#define MTP_DATA_INT128             0x0009 /* Signed 128-bit integer */
#define MTP_DATA_UINT128            0x000a /* Unsigned 128-bit integer */
#define MTP_DATA_AINT8              0x4001 /* Array of signed 8-bit integers */
#define MTP_DATA_AUINT8             0x4002 /* Array of unsigned 8-bit integers */
#define MTP_DATA_AINT16             0x4003 /* Array of signed 16-bit integers */
#define MTP_DATA_AUINT16            0x4004 /* Array of unsigned 16-bit integers */
#define MTP_DATA_AINT32             0x4005 /* Array of signed 32-bit integers */
#define MTP_DATA_AUINT32            0x4006 /* Array of unsigned 32-bit integers */
#define MTP_DATA_AINT64             0x4007 /* Array of signed 64-bit integers */
#define MTP_DATA_AUINT64            0x4008 /* Array of unsigned 64-bit integers */
#define MTP_DATA_AINT128            0x4009 /* Array of signed 128-bit integers */
#define MTP_DATA_AUINT128           0x400a /* Array of unsigned 128-bit integers */
#define MTP_DATA_STR                0xffff /* Variable-length Unicode string */

/* Association Type */

#define MTP_ASSOC_UNDEF             0x0000 /* Undefined */
#define MTP_ASSOC_GENFOLDER         0x0001 /* Unused by PTP, used by MTP to indicate type if folder */
#define MTP_ASSOC_ALBUM             0x0002 /* Album - Reserved */
#define MTP_ASSOC_TIME_SEQ          0x0003 /* Time Sequence - Default Playback Delta */
#define MTP_ASSOC_HORIZ_PAN         0x0004 /* Horizontal Panoramic - Unused */
#define MTP_ASSOC_VERT_PAN          0x0005 /* Vertical Panoramic - Unused */
#define MTP_ASSOC_2D_PAN            0x0006 /* 2D Panoramic - Images per row */
#define MTP_ASSOC_ANC_DATA          0x0007 /* Ancillary Data - Undefined */

/* Note: This below header definitions are based on mtp-tools dissector
 * implemented on Lua. BSD License: https://github.com/tengelmeier/mtp-tools
 */

/* MTP Requests Operations */

#define MTP_REQ_UNDEF               0x1000 /* Undefined Request */
#define MTP_GET_DEV_INFO            0x1001
#define MTP_OPEN_SESSION            0x1002
#define MTP_CLOSE_SESSION           0x1003
#define MTP_GET_STORAGE_IDS         0x1004
#define MTP_GET_STORAGE_INFO        0x1005
#define MTP_GET_NUM_OBJS            0x1006
#define MTP_GET_OBJ_HANDLES         0x1007
#define MTP_GET_OBJ_INFO            0x1008
#define MTP_GET_OBJ                 0x1009
#define MTP_GET_THUMB               0x100a
#define MTP_DEL_OBJ                 0x100b
#define MTP_SEND_OBJ_INFO           0x100c
#define MTP_SEND_OBJ                0x100d
#define MTP_INIT_CAPTURE            0x100e
#define MTP_FORMAT_STORE            0x100f
#define MTP_RESET_DEV               0x1010
#define MTP_SELF_TEST               0x1011
#define MTP_SET_OBJ_PROTECTION      0x1012
#define MTP_POWER_DOWN              0x1013
#define MTP_GET_DEV_PROP_DESC       0x1014
#define MTP_GET_DEV_PROP_VALUE      0x1015
#define MTP_SET_DEV_PROP_VALUE      0x1016
#define MTP_RESET_DEV_PROP_VALUE    0x1017
#define MTP_TERMINATE_CAPTURE       0x1018
#define MTP_MOVE_OBJ                0x1019
#define MTP_COPY_OBJ                0x101a
#define MTP_GET_PARTIAL_OBJ         0x101b
#define MTP_INIT_OPEN_CAPTURE       0x101c
#define MTP_RESERVED_FIRST          0x1026
#define MTP_RESERVED_LAST           0x1fff
#define MTP_VENDOR_EXT_FIRST        0x9000
#define MTP_GET_SERVICE_IDS         0x9301
#define MTP_GET_SERVICE_INFO        0x9302
#define MTP_GET_SERVICE_CAPS        0x9303
#define MTP_GET_SERVICE_PROPS       0x9304
#define MTP_GET_SERVICE_PROP_LIST   0x9305
#define MTP_SET_SERVICE_PROP_LIST   0x9306
#define MTP_UPDATE_OBJ_PROP_LIST    0x9307
#define MTP_DEL_OBJ_PROP_LIST       0x9308
#define MTP_DEL_SERVICE_PROP_LIST   0x9309
#define MTP_GET_FORMAT_CAPS         0x930a
#define MTP_VENDOR_EXT_LAST         0x97ff
#define MTP_GET_OBJ_PROPS_SUPPORTED 0x9801
#define MTP_GET_OBJ_PROP_DESC       0x9802
#define MTP_GET_OBJ_PROP_VALUE      0x9803
#define MTP_SET_OBJ_PROP_VALUE      0x9804
#define MTP_GET_OBJ_PROP_LIST       0x9805
#define MTP_SET_OBJ_PROP_LIST       0x9806
#define MTP_GET_INTERDEP_PROP_DESC  0x9807
#define MTP_SEND_OBJ_PROP_LIST      0x9808
#define MTP_GET_FORMAT_CAPS         0x9809
#define MTP_UPDATE_OBJ_PROP_LIST    0x980a
#define MTP_DEL_OBJ_PROP_LIST       0x980b
#define MTP_GET_OBJ_REFS            0x9810
#define MTP_SET_OBJ_REFS            0x9811
#define MTP_UPDATE_DEV_FIRMWARE     0x9812
#define MTP_RESET_OBJ_PROP_VALUE    0x9813
#define MTP_GET_SERVICE_IDS         0x9900
#define MTP_GET_SERVICE_INFO        0x9901
#define MTP_GET_SERVICE_CAPS        0x9902
#define MTP_GET_SERVICE_PROPS       0x9903
#define MTP_GET_SERVICE_PROP_LIST   0x9904
#define MTP_SET_SERVICE_PROP_LIST   0x9905
#define MTP_DEL_SERVICE_PROP_LIST   0x9906
#define MTP_OPEN_OBJ_STREAM         0x9910
#define MTP_READ_OBJ_STREAM         0x9911
#define MTP_WRITE_OBJ_STREAM        0x9912
#define MTP_SEEK_OBJ_STREAM         0x9913
#define MTP_CLOSE_OBJ_STREAM        0x9914

/* MTP Response codes */

#define MTP_UNDEF_RESP              0x2000
#define MTP_OK_RESP                 0x2001
#define MTP_GENERAL_ERROR           0x2002
#define MTP_SESSION_NOT_OPEN        0x2003
#define MTP_INVAL_TRANSACTION_ID    0x2004
#define MTP_OPERATION_NOT_SUPPORTED 0x2005
#define MTP_PARAMETER_NOT_SUPPORTED 0x2006
#define MTP_INCOMPLETE_TRANSFER     0x2007
#define MTP_INVAL_STORAGE_ID        0x2008
#define MTP_INVAL_OBJ_HANDLE        0x2009
#define MTP_DEV_PROP_NOT_SUPPORTED  0x200a
#define MTP_INVAL_OBJ_FORMAT_CODE   0x200b
#define MTP_STORE_FULL              0x200c
#define MTP_OBJ_WRITE_PROTECTED     0x200d
#define MTP_STORE_WRITE_PROTECTED   0x200e
#define MTP_ACCESS_DENIED           0x200f
#define MTP_NO_THUMBNAIL_PRESENT    0x2010
#define MTP_SELF_TEST_FAILED        0x2011
#define MTP_PARTIAL_DELETION        0x2012
#define MTP_STORE_NOT_AVAILABLE     0x2013
#define MTP_NO_SPEC_BY_FORMAT       0x2014
#define MTP_NO_VALID_OBJ_INFO       0x2015
#define MTP_INVAL_CODE_FORMAT       0x2016
#define MTP_UNKNOWN_VENDOR_CODE     0x2017
#define MTP_CAPTURE_ALRDY_TERMINE   0x2018
#define MTP_DEV_BUSY                0x2019
#define MTP_INVAL_PARENT            0x201a
#define MTP_INVAL_PROP_FORMAT       0x201b
#define MTP_INVAL_PROP_VALUE        0x201c
#define MTP_INVAL_PARAMETER         0x201d
#define MTP_SESSION_ALREADY_OPENED  0x201e
#define MTP_TRANSACTION_CANCELLED   0x201f
#define MTP_SPEC_OF_DEST_UNSUPPORT  0x2020
#define MTP_RESERVED_FIRST          0x2024
#define MTP_RESERVED_LAST           0x2fff
#define MTP_VENDOR_EXTENSION_FIRST  0xa000
#define MTP_INVAL_SERVICE_ID        0xa301
#define MTP_INVAL_SERVICE_PROP_CODE 0xa302
#define MTP_VENDOR_EXTENSION_LAST   0xa7ff
#define MTP_INVAL_OBJ_PROP_CODE     0xa801
#define MTP_INVAL_OBJ_PROP_FORMAT   0xa802
#define MTP_INVAL_OBJ_PROP_VALUE    0xa803
#define MTP_INVAL_OBJ_REF           0xa804
#define MTP_INVAL_OBJ_GROUP_CODE    0xa805
#define MTP_INVAL_DATA_SET          0xa806
#define MTP_SPEC_BY_GROUP_UNSUPPORT 0xa807
#define MTP_OBJ_TOO_LARGE           0xa809
#define MTP_OBJ_PROP_NOT_SUPPORTED  0xa80a
#define MTP_INVAL_SERVICE_ID        0xa80b
#define MTP_INVAL_SERVICE_PROP_CODE 0xa80c
#define MTP_MAX_STRM_REACHED        0xa80c
#define MTP_MAX_STRM_PER_OBJ_REACH  0xa80e
#define MTP_SESSION_LIMIT_REACHED   0xa80f

/* Events */

#define MTP_UNDEF_EVENT             0x4000
#define MTP_CANCEL_TRANSACTION      0x4001
#define MTP_OBJECT_ADDED            0x4002
#define MTP_OBJECT_REMOVED          0x4003
#define MTP_STORE_ADDED             0x4004
#define MTP_STORE_REMOVED           0x4005
#define MTP_DEVICE_PROP_CHANGED     0x4006
#define MTP_OBJECT_INFO_CHANGED     0x4007
#define MTP_DEVICE_INFO_CHANGED     0x4008
#define MTP_REQ_OBJECT_TRANSFER     0x4009
#define MTP_STORE_FULL              0x400a
#define MTP_DEVICE_RESET            0x400b
#define MTP_STORAGE_INFO_CHANGED    0x400c
#define MTP_CAPTURE_COMPLETE        0x400d
#define MTP_UNREPORTED_STATUS       0x400e
#define MTP_RESERVED_FIRST          0x400f
#define MTP_RESERVED_LAST           0x4fff
#define MTP_VENDOR_EXTENSION_FIRST  0xc000
#define MTP_SERVICE_ADDED           0xc301
#define MTP_SERVICE_REMOVED         0xc302
#define MTP_SERVICE_PROP_CHANGED    0xc303
#define MTP_METHOD_COMPLETE         0xc304
#define MTP_VENDOR_EXTENSION_LAST   0xc7ff
#define MTP_OBJ_PROP_CHANGED        0xc801
#define MTP_OBJ_PROP_DESC_CHANGED   0xc802
#define MTP_SERVICE_ADDED           0xc804
#define MTP_SERVICE_REMOVED         0xc805
#define MTP_SERVICE_PROP_CHANGED    0xc806
#define MTP_METHOD_COMPLETE         0xc807

/* Device Properties */

#define MTP_NOT_USED                0x0000
#define MTP_UNDEFINED               0x5000
#define MTP_BATTERY_LEVEL           0x5001
#define MTP_FUNCTION_MODE           0x5002
#define MTP_IMAGE_SIZE              0x5003
#define MTP_COMPRESSION_SETTING     0x5004
#define MTP_WHITE_BALANCE           0x5005
#define MTP_RGB_GAIN                0x5006
#define MTP_FNUMBER                 0x5007
#define MTP_FOCAL_LENGTH            0x5008
#define MTP_FOCUS_DISTANCE          0x5009
#define MTP_FOCUS_MODE              0x500a
#define MTP_EXPOSURE_METERING_MODE  0x500b
#define MTP_FLASH_MODE              0x500c
#define MTP_EXPOSURE_TIME           0x500d
#define MTP_EXPOSURE_PROGRAM_MODE   0x500e
#define MTP_EXPOSURE_INDEX          0x500f
#define MTP_EXPOSURE_COMPENSATION   0x5010
#define MTP_DATE_TIME               0x5011
#define MTP_CAPTURE_DELAY           0x5012
#define MTP_STILL_CAPTURE_MODE      0x5013
#define MTP_CONTRAST                0x5014
#define MTP_SHARPNESS               0x5015
#define MTP_DIGITAL_ZOOM            0x5016
#define MTP_EFFECT_MODE             0x5017
#define MTP_BURST_NUMBER            0x5018
#define MTP_BURST_INTERVAL          0x5019
#define MTP_TIME_LAPSE_NUMBER       0x501a
#define MTP_TIME_LAPSE_INTERVAL     0x501b
#define MTP_FOCUS_METERING_MODE     0x501c
#define MTP_UPLOAD_URL              0x501d
#define MTP_ARTIST                  0x501e
#define MTP_COPYRIGHT_INFO          0x501f
#define MTP_RESERVED_FIRST          0x5020
#define MTP_RESERVED_LAST           0x5fff
#define MTP_VENDOR_EXTENSION_FIRST  0xd000
#define MTP_FUNCTIONAL_ID           0xd301
#define MTP_MODEL_UNIQUE_ID         0xd302
#define MTP_USE_DEVICE_STAGE        0xd303
#define MTP_VENDOR_EXTENSION_LAST   0xd3ff
#define MTP_SYNCHRONIZATION_PARTNER 0xd401
#define MTP_DEVICE_FRIENDLY_NAME    0xd402
#define MTP_VOLUME                  0xd403
#define MTP_CONSUMPT_FORMAT_PREF    0xd404
#define MTP_DEVICE_ICON             0xd405
#define MTP_SESSION_INIT_VER_INFO   0xd406
#define MTP_PERCEIVED_DEV_TYPE      0xd407
#define MTP_FUNCTIONAL_ID           0xd408
#define MTP_PLAYBACK_RATE           0xd410
#define MTP_PLAYBACK_OBJECT         0xd411
#define MTP_PLAYBACK_CONTAINER      0xd412
#define MTP_PLAYBACK_POSITION       0xd413
#define MTP_ALL                     0xffffffff

/* Object Properties */

#define MTP_NOT_USED                0x0000
#define MTP_UNDEFINED               0xd000
#define MTP_VENDOR_EXTENSION_FIRST  0xd800
#define MTP_VENDOR_EXTENSION_LAST   0xdbff
#define MTP_STORAGE_ID              0xdc01
#define MTP_OBJECT_FORMAT           0xdc02
#define MTP_PROTECTION_STATUS       0xdc03
#define MTP_OBJECT_SIZE             0xdc04
#define MTP_ASSOCIATION_TYPE        0xdc05
#define MTP_ASSOCIATION_DESC        0xdc06
#define MTP_OBJECT_FILE_NAME        0xdc07
#define MTP_DATE_CREATED            0xdc08
#define MTP_DATE_MODIFIED           0xdc09
#define MTP_KEYWORDS                0xdc0a
#define MTP_PARENT                  0xdc0b
#define MTP_ALLOWED_FOLDER_CONTENTS 0xdc0c
#define MTP_HIDDEN                  0xdc0d
#define MTP_SYSTEM_OBJECT           0xdc0e
#define MTP_PERS_UNIQUE_OBJ_IDENT   0xdc41
#define MTP_SYNC_ID                 0xdc42
#define MTP_PROPERTY_BAG            0xdc43
#define MTP_NAME                    0xdc44
#define MTP_CREATED_BY              0xdc45
#define MTP_ARTIST                  0xdc46
#define MTP_DATE_AUTHORED           0xdc47
#define MTP_DESCRIPTION             0xdc48
#define MTP_URL_REFERENCE           0xdc49
#define MTP_LANGUAGE_LOCALE         0xdc4a
#define MTP_COPYRIGHT_INFO          0xdc4b
#define MTP_SOURCE                  0xdc4c
#define MTP_ORIGIN_LOCATION         0xdc4d
#define MTP_DATE_ADDED              0xdc4e
#define MTP_NON_CONSUMABLE          0xdc4f
#define MTP_CORRUPT_UNPLAYABLE      0xdc50
#define MTP_PRODUCER_SERIAL_NUMBER  0xdc51
#define MTP_REPRESENT_SAMPLE_FORMAT 0xdc81
#define MTP_REPRESENT_SAMPLE_SIZE   0xdc82
#define MTP_REPRESENT_SAMPLE_HEIGHT 0xdc83
#define MTP_REPRESENT_SAMPLE_WIDTH  0xdc84
#define MTP_REPRESENT_SAMPLE_DUR    0xdc85
#define MTP_REPRESENT_SAMPLE_DATA   0xdc86
#define MTP_WIDTH                   0xdc87
#define MTP_HEIGHT                  0xdc88
#define MTP_DURATION                0xdc89
#define MTP_USER_RATING             0xdc8a
#define MTP_TRACK                   0xdc8b
#define MTP_GENRE                   0xdc8c
#define MTP_CREDITS                 0xdc8d
#define MTP_LYRICS                  0xdc8e
#define MTP_SUBSCRIPTION_CONTENT_ID 0xdc8f
#define MTP_PRODUCED_BY             0xdc90
#define MTP_USE_COUNT               0xdc91
#define MTP_SKIP_COUNT              0xdc92
#define MTP_LAST_ACCESSED           0xdc93
#define MTP_PARENTAL_RATING         0xdc94
#define MTP_META_GENRE              0xdc95
#define MTP_COMPOSER                0xdc96
#define MTP_EFFECTIVE_RATING        0xdc97
#define MTP_SUBTITLE                0xdc98
#define MTP_ORIGINAL_RELEASE_DATE   0xdc99
#define MTP_ALBUM_NAME              0xdc9a
#define MTP_ALBUM_ARTIST            0xdc9b
#define MTP_MOOD                    0xdc9c
#define MTP_DRM_PROTECTION          0xdc9d
#define MTP_SUBDESCRIPTION          0xdc9e
#define MTP_IS_CROPPED              0xdcd1
#define MTP_IS_COLOUR_CORRECTED     0xdcd2
#define MTP_IMAGE_BIT_DEPTH         0xdcd3
#define MTP_FNUMBER                 0xdcd4
#define MTP_EXPOSURE_TIME           0xdcd5
#define MTP_EXPOSURE_INDEX          0xdcd6
#define MTP_DISPLAY_NAME            0xdce0
#define MTP_BODY_TEXT               0xdce1
#define MTP_SUBJECT                 0xdce2
#define MTP_PRIORITY                0xdce3
#define MTP_GIVEN_NAME              0xdd00
#define MTP_MIDDLE_NAMES            0xdd01
#define MTP_FAMILY_NAME             0xdd02
#define MTP_PREFIX                  0xdd03
#define MTP_SUFFIX                  0xdd04
#define MTP_PHONETIC_GIVEN_NAME     0xdd05
#define MTP_PHONETIC_FAMILY_NAME    0xdd06
#define MTP_EMAIL_PRIMARY           0xdd07
#define MTP_EMAIL_PERS1             0xdd08
#define MTP_EMAIL_PERS2             0xdd09
#define MTP_EMAIL_BUSINESS1         0xdd0a
#define MTP_EMAIL_BUSINESS2         0xdd0b
#define MTP_EMAIL_OTHERS            0xdd0c
#define MTP_PHONE_NUMBER_PRIMARY    0xdd0d
#define MTP_PHONE_NUMBER_PERS       0xdd0e
#define MTP_PHONE_NUMBER_PERS2      0xdd0f
#define MTP_PHONE_NUMBER_BUSINESS   0xdd10
#define MTP_PHONE_NUMBER_BUSINESS2  0xdd11
#define MTP_PHONE_NUMBER_MOBIL      0xdd12
#define MTP_PHONE_NUMBER_MOBIL2     0xdd13
#define MTP_FAX_NUMBER_PRIMARY      0xdd14
#define MTP_FAX_NUMBER_PERS         0xdd15
#define MTP_FAX_NUMBER_BUSINESS     0xdd16
#define MTP_PAGER_NUMBER            0xdd17
#define MTP_PHONE_NUMBER_OTHERS     0xdd18
#define MTP_PRIMARY_WEB_ADDR        0xdd19
#define MTP_PERS_WEB_ADDR           0xdd1a
#define MTP_BUSINESS_WEB_ADDR       0xdd1a
#define MTP_INSTANCE_MSGER_ADDR     0xdd1c
#define MTP_INSTANCE_MSGER_ADDR2    0xdd1d
#define MTP_INSTANCE_MSGER_ADDR3    0xdd1e
#define MTP_POSTAL_ADDR_PERS_FULL   0xdd1f
#define MTP_POSTAL_ADDR_PERS_LINE1  0xdd20
#define MTP_POSTAL_ADDR_PERS_LINE2  0xdd21
#define MTP_POSTAL_ADDR_PERS_CITY   0xdd22
#define MTP_POSTAL_ADDR_PERS_REGION 0xdd23
#define MTP_POSTAL_ADDR_PERS_PCODE  0xdd24
#define MTP_POSTAL_ADDR_PERS_CTRY   0xdd25
#define MTP_POSTAL_ADDR_BIZ_FULL    0xdd26
#define MTP_POSTAL_ADDR_BIZ_LINE1   0xdd27
#define MTP_POSTAL_ADDR_BIZ_LINE2   0xdd28
#define MTP_POSTAL_ADDR_BIZ_CITY    0xdd29
#define MTP_POSTAL_ADDR_BIZ_REGION  0xdd2a
#define MTP_POSTAL_ADDR_BIZ_PCODE   0xdd2b
#define MTP_POSTAL_ADDR_BIZ_CTRY    0xdd2c
#define MTP_POSTAL_ADDR_OTHER_FULL  0xdd2d
#define MTP_POSTAL_ADDR_OTHER_LINE1 0xdd2e
#define MTP_POSTAL_ADDR_OTHER_LINE2 0xdd2f
#define MTP_POSTAL_ADDR_OTHER_CITY  0xdd30
#define MTP_POSTAL_ADDR_OTHER_REG   0xdd31
#define MTP_POSTAL_ADDR_OTHER_PCODE 0xdd32
#define MTP_POSTAL_ADDR_OTHER_CTRY  0xdd33
#define MTP_ORGANIZATION_NAME       0xdd34
#define MTP_PHONETIC_ORG_NAME       0xdd35
#define MTP_ROLE                    0xdd36
#define MTP_BIRTHDAY                0xdd37
#define MTP_MESSAGE_TO              0xdd40
#define MTP_MESSAGE_CC              0xdd41
#define MTP_MESSAGE_BCC             0xdd42
#define MTP_MESSAGE_READ            0xdd43
#define MTP_MESSAGE_RECEIVE_TIME    0xdd44
#define MTP_MESSAGE_SENDER          0xdd45
#define MTP_ACTIVITY_BEGIN_TIME     0xdd50
#define MTP_ACTIVITY_END_TIME       0xdd51
#define MTP_ACTIVITY_LOCATION       0xdd52
#define MTP_ACTIVITY_REQ_ATTENDEES  0xdd54
#define MTP_ACTIVITY_OPT_ATTENDEES  0xdd55
#define MTP_ACTIVITY_RESOURCES      0xdd56
#define MTP_ACTIVITY_ACCEPTED       0xdd57
#define MTP_ACTIVITY_TENTATIVE      0xdd58
#define MTP_ACTIVITY_DECLINED       0xdd59
#define MTP_ACTIVITY_REMINDER_TIME  0xdd5a
#define MTP_ACTIVITY_OWNER          0xdd5b
#define MTP_ACTIVITY_STATUS         0xdd5c
#define MTP_OWNER                   0xdd5d
#define MTP_EDITOR                  0xdd5e
#define MTP_WEBMASTER               0xdd5f
#define MTP_URL_SOURCE              0xdd60
#define MTP_URL_DESTINATION         0xdd61
#define MTP_TIME_BOOKMARK           0xdd62
#define MTP_OBJECT_BOOKMARK         0xdd63
#define MTP_BYTE_BOOKMARK           0xdd64
#define MTP_DATA_OFFSET             0xdd65
#define MTP_DATA_LENGTH             0xdd66
#define MTP_DATA_UNITS              0xdd67
#define MTP_DATA_REF_OBJ_RESOURCE   0xdd68
#define MTP_BACK_REFERENCES         0xdd69
#define MTP_LAST_BUILD_DATE         0xdd70
#define MTP_TIME_TO_LIVE            0xdd71
#define MTP_MEDIA_GUID              0xdd72
#define MTP_TOTAL_BITRATE           0xde91
#define MTP_BITRATE_TYPE            0xde92
#define MTP_SAMPLE_RATE             0xde93
#define MTP_NUMBER_OF_CHANNELS      0xde94
#define MTP_AUDIO_BIT_DEPTH         0xde95
#define MTP_BLOCK_ALIGNMENT         0xde96
#define MTP_SCAN_TYPE               0xde97
#define MTP_COLOUR_RANGE            0xde98
#define MTP_AUDIO_WAVE_CODEC        0xde99
#define MTP_AUDIO_BITRATE           0xde9a
#define MTP_VIDEO_FOUR_CC_CODEC     0xde9b
#define MTP_VIDEO_BITRATE           0xde9c
#define MTP_FRAMES_PER_MILLISECOND  0xde9d
#define MTP_KEY_FRAME_DISTANCE      0xde9e
#define MTP_BUFFER_SIZE             0xde9f
#define MTP_ENCODING_QUALITY        0xdea0
#define MTP_ENCODING_PROFILE        0xdea1
#define MTP_AUDIO_ENCODING_PROFILE  0xdea2
#define MTP_ALL                     0xffffffff

/* MTP File format */

#define MTP_FILE_NOT_USED           0x0000
#define MTP_FILE_UNDEFINED          0x3000
#define MTP_FILE_ASSOCIATION        0x3001
#define MTP_FILE_SCRIPT             0x3002
#define MTP_FILE_EXECUTABLE         0x3003
#define MTP_FILE_TEXT               0x3004
#define MTP_FILE_HTML               0x3005
#define MTP_FILE_DPOF               0x3006
#define MTP_FILE_AIFF               0x3007
#define MTP_FILE_WAVE               0x3008
#define MTP_FILE_MP3                0x3009
#define MTP_FILE_AVI                0x300a
#define MTP_FILE_MPEG               0x300b
#define MTP_FILE_ASF                0x300c
#define MTP_FILE_RESERVED_FIRST     0x300d
#define MTP_FILE_RESERVED_LAST      0x37ff
#define MTP_FILE_UNDEFINED_IMAGE    0x3800
#define MTP_FILE_EXIF_JPEG          0x3801
#define MTP_FILE_TIFF_EP            0x3802
#define MTP_FILE_FLASHPIX           0x3803
#define MTP_FILE_BMP                0x3804
#define MTP_FILE_CIFF               0x3805
#define MTP_FILE_GIF                0x3807
#define MTP_FILE_JFIF               0x3808
#define MTP_FILE_PCD                0x3809
#define MTP_FILE_PICT               0x380a
#define MTP_FILE_PNG                0x380b
#define MTP_FILE_TIFF               0x380d
#define MTP_FILE_TIFF_IT            0x380e
#define MTP_FILE_JP2                0x380f
#define MTP_FILE_JPX                0x3810
#define MTP_FILE_IMAGE_RESERV_FIRST 0x3811
#define MTP_FILE_IMAGE_RESERV_LAST  0x3fff
#define MTP_FILE_VENDOR_EXT_FIRST   0xb000
#define MTP_FILE_VENDOR_EXT_LAST    0xb7ff
#define MTP_FILE_UNDEFINED_FIRMWARE 0xb802
#define MTP_FILE_WBMP               0xb803
#define MTP_FILE_JPEG_XR            0xb804
#define MTP_FILE_WINDOWS_IMAGE_FMT  0xb881
#define MTP_FILE_UNDEF_AUDIO        0xb900
#define MTP_FILE_WMA                0xb901
#define MTP_FILE_OGG                0xb902
#define MTP_FILE_AAC                0xb903
#define MTP_FILE_AUDIBLE            0xb904
#define MTP_FILE_FLAC               0xb906
#define MTP_FILE_QCELP              0xb907
#define MTP_FILE_AMR                0xb908
#define MTP_FILE_UNDEF_VIDEO        0xb980
#define MTP_FILE_WMV                0xb981
#define MTP_FILE_MP4                0xb982
#define MTP_FILE_MP2                0xb983
#define MTP_FILE_3GP                0xb984
#define MTP_FILE_3G2                0xb985
#define MTP_FILE_AVC_HD             0xb986
#define MTP_FILE_ATSC_TS            0xb987
#define MTP_FILE_DVB_TS             0xb988
#define MTP_FILE_UNDEF_COLLECTION   0xba00
#define MTP_FILE_ABST_MMEDIA_ALBUM  0xba01
#define MTP_FILE_ABST_IMAGE_ALBUM   0xba02
#define MTP_FILE_ABST_AUDIO_ALBUM   0xba03
#define MTP_FILE_ABST_VIDEO_ALBUM   0xba04
#define MTP_FILE_ABST_AV_PLAYLIST   0xba05
#define MTP_FILE_ABST_CONTACT_GROUP 0xba06
#define MTP_FILE_ABST_MSG_FOLDER    0xba07
#define MTP_FILE_ABST_CHAP_PROD     0xba08
#define MTP_FILE_ABST_MEDIA_CAST    0xba0b
#define MTP_FILE_WPL_PLAYLIST       0xba10
#define MTP_FILE_M3U_PLAYLIST       0xba11
#define MTP_FILE_MPL_PLAYLIST       0xba12
#define MTP_FILE_ASX_PLAYLIST       0xba13
#define MTP_FILE_PLS_PLAYLIST       0xba14
#define MTP_FILE_UNDEF_DOC          0xba80
#define MTP_FILE_ABSTRACT_DOC       0xba81
#define MTP_FILE_XML_DOCUMENT       0xba82
#define MTP_FILE_MSFT_WORD_DOC      0xba83
#define MTP_FILE_MHT_COMP_HTML_DOC  0xba84
#define MTP_FILE_MSFT_EXCEL_SSHEET  0xba85
#define MTP_FILE_MSFT_PWRPOINT_DOC  0xba86
#define MTP_FILE_UNDEF_MSG          0xbb00
#define MTP_FILE_ABST_MSG           0xbb01
#define MTP_FILE_UNDEF_CONTACT      0xbb80
#define MTP_FILE_ABST_CONTACT       0xbb81
#define MTP_FILE_VCARD2             0xbb82
#define MTP_FILE_VCARD3             0xbb83
#define MTP_FILE_UNDEF_CAL_ITEM     0xbe00
#define MTP_FILE_ABST_CAL_ITEM      0xbe01
#define MTP_FILE_VCALENDAR1         0xbe02
#define MTP_FILE_VCALENDAR2         0xbe03
#define MTP_FILE_UNDEF_WIN_EXE      0xbe80
#define MTP_FILE_ALL_IMAGES         0xffffffff

struct mtp_proto_s
{
  uint32_t length;
  uint16_t type;
  uint16_t opcode;
  uint32_t trans_id;
  uint32_t param[5];
};

struct mtp_resp_s
{
  uint32_t length;
  uint16_t type;
  uint16_t opcode;
  uint32_t trans_id;
  uint32_t param[5];
};
