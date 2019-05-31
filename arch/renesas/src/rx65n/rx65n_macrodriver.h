#ifndef STATUS_H
#define STATUS_H
/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "iodefine.h"

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/

/***********************************************************************************************************************
User definitions
***********************************************************************************************************************/
#ifndef	TRUE
#define TRUE (1)
#else
#if (1 != TRUE)
#error "TRUE is not defined by 1."
#endif
#endif

#ifndef	FALSE
#define FALSE (0)
#else
#if (0 != FALSE)
#error "FALSE is not defined by 0."
#endif
#endif

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#ifndef __TYPEDEF__

/* Status list definition */
#define MD_STATUSBASE        (0x00U)
#define MD_OK                (MD_STATUSBASE + 0x00U) /* register setting OK */
#define MD_SPT               (MD_STATUSBASE + 0x01U) /* IIC stop */
#define MD_NACK              (MD_STATUSBASE + 0x02U) /* IIC no ACK */
#define MD_BUSY1             (MD_STATUSBASE + 0x03U) /* busy 1 */
#define MD_BUSY2             (MD_STATUSBASE + 0x04U) /* busy 2 */

/* Error list definition */
#define MD_ERRORBASE         (0x80U)
#define MD_ERROR             (MD_ERRORBASE + 0x00U)  /* error */
#define MD_ARGERROR          (MD_ERRORBASE + 0x01U)  /* error argument input error */
#define MD_ERROR1            (MD_ERRORBASE + 0x02U)  /* error 1 */
#define MD_ERROR2            (MD_ERRORBASE + 0x03U)  /* error 2 */
#define MD_ERROR3            (MD_ERRORBASE + 0x04U)  /* error 3 */
#define MD_ERROR4            (MD_ERRORBASE + 0x05U)  /* error 4 */
#define MD_ERROR5            (MD_ERRORBASE + 0x06U)  /* error 5 */

#define nop()                asm("nop;")
#define brk()                asm("brk;")
#define wait()               asm("wait;")

#endif

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
#ifndef __TYPEDEF__
    #ifndef _STDINT_H
    	//typedef signed char         int8_t;
        typedef unsigned char       uint8_t;
        //typedef signed short        int16_t;
        typedef unsigned short      uint16_t;
        //typedef signed long         int32_t;
        //typedef unsigned long       uint32_t;
        
        //typedef signed char         int_least8_t;
        //typedef signed short        int_least16_t;
        //typedef signed long         int_least32_t;
        //typedef unsigned char       uint_least8_t;
        //typedef unsigned short      uint_least16_t;
        //typedef unsigned long       uint_least32_t;
    #endif 

    typedef unsigned short      MD_STATUS;
    #define __TYPEDEF__
#endif

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/

#endif
