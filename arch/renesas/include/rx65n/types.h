/****************************************************************************
 * arch/renesas/include/rx65n/types.h
 *
 *   
 *
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly\
 * through sys/types.h
 */

#ifndef __ARCH_RENESAS_INCLUDE_RX65N_TYPES_H
#define __ARCH_RENESAS_INCLUDE_RX65N_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* These are the sizes of the standard integer types.  NOTE that these type
 * names have a leading underscore character.  This file will be included
 * (indirectly) by include/stdint.h and typedef'ed to the final name without
 * the underscore character.  This roundabout way of doings things allows
 * the stdint.h to be removed from the include/ directory in the event that
 * the user prefers to use the definitions provided by their toolchain header
 * files
 */

typedef signed char        _int8_t;
typedef unsigned char      _uint8_t;

typedef signed short       _int16_t;
typedef unsigned short     _uint16_t;

typedef signed int         _int32_t;
typedef unsigned int       _uint32_t;

typedef signed long long   _int64_t;
typedef unsigned long long _uint64_t;
#define __INT64_DEFINED

/* A pointer is 4 bytes */

typedef signed int         _intptr_t;
typedef unsigned int       _uintptr_t;

/* This is the size of the interrupt state save returned by
 * up_irq_save()
 */

typedef unsigned long      irqstate_t;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_RENESAS_INCLUDE_SH1_TYPES_H */
