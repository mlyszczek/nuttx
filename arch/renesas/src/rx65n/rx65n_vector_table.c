/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "rx65n_macrodriver.h"
#include "chip.h"
#include "stdint.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

//#define 	__USE_DEBUG_NOP_FOR_BREAKPOINTS

#define OFS_REG		__attribute__ ((section (".ofs1"))) /* 0xFE7F5D00 */ /* MDE, OFS0, OFS1 */
#define OFS_TMINF	__attribute__ ((section (".ofs2"))) /* 0xFE7F5D10 */
#define OFS_SPCC	__attribute__ ((section (".ofs3"))) /* 0xFE7F5D40 */
#define OFS_TMEF	__attribute__ ((section (".ofs4"))) /* 0xFE7F5D48 */
#define OFS_OSIS	__attribute__ ((section (".ofs5"))) /* 0xFE7F5D50 */
#define OFS_FAW		__attribute__ ((section (".ofs6"))) /* 0xFE7F5D64 */
#define OFS_ROMCODE	__attribute__ ((section (".ofs7"))) /* 0xFE7F5D70 */

const unsigned long	__SPCCreg		OFS_SPCC	= 0xFFFFFFFF;	/* SPCC register */
const unsigned long	__TMEFreg		OFS_TMEF	= 0xFFFFFFFF;	/* TMEF register */
const unsigned long	__OSISreg[4]	OFS_OSIS	= {				/* OSIS register (ID codes) */
						0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
const unsigned long	__TMINFreg		OFS_TMINF	= 0xFFFFFFFF;	/* TMINF register */
const unsigned long	__FAWreg		OFS_FAW		= 0xFFFFFFFF;	/* FAW register */
const unsigned long	__ROMCODEreg	OFS_ROMCODE	= 0xFFFFFFFF;	/* ROMCODE register */

/* MDE register (Single Chip Mode) */
#ifdef __RX_BIG_ENDIAN__
const unsigned long	__MDEreg		OFS_REG		= 0xFFFFFFF8;	/* big */
#else
const unsigned long	__MDEreg		OFS_REG		= 0xFFFFFFFF;	/* little */
#endif

const unsigned long	__OFS0reg		OFS_REG		= 0xFFFFFFFF;	/* OFS0 register */
const unsigned long	__OFS1reg		OFS_REG		= 0xFFFFFFFF;	/* OFS1 register */

/***********************************************************************************************************************
* Function Name: r_undefined_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_undefined_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_reserved_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_reserved_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_nmi_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_nmi_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_brk_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_brk_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_privileged_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_privileged_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_access_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_access_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_floatingpoint_exception
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	r_floatingpoint_exception(void)
{
	/* Start user code. Do not edit comment generated here */
#ifdef	__USE_DEBUG_NOP_FOR_BREAKPOINTS
	__asm("nop");		// as the break point for debugging
#endif	// __USE_DEBUG_NOP_FOR_BREAKPOINTS
	/* End user code. Do not edit comment generated here */
}

#define EXVECT_SECT    __attribute__ ((section (".exvectors")))

const void	*ExceptVectors[] EXVECT_SECT  = {
	r_reserved_exception,					/* 0xffffff80  Reserved */
	r_reserved_exception,					/* 0xffffff84  Reserved */
	r_reserved_exception,					/* 0xffffff88  Reserved */
	r_reserved_exception,					/* 0xffffff8C  Reserved */
	r_reserved_exception,					/* 0xffffff90  Reserved */
	r_reserved_exception,					/* 0xffffff94  Reserved */
	r_reserved_exception,					/* 0xffffff98  Reserved */
	r_reserved_exception,					/* 0xffffff9C  Reserved */
	r_reserved_exception,					/* 0xffffffA0  Reserved */
	r_reserved_exception,					/* 0xffffffA4  Reserved */
	r_reserved_exception,					/* 0xffffffA8  Reserved */
	r_reserved_exception,					/* 0xffffffAC  Reserved */
	r_reserved_exception,					/* 0xffffffB0  Reserved */
	r_reserved_exception,					/* 0xffffffB4  Reserved */
	r_reserved_exception,					/* 0xffffffB8  Reserved */
	r_reserved_exception,					/* 0xffffffBC  Reserved */
	r_reserved_exception,					/* 0xffffffC0  Reserved */
	r_reserved_exception,					/* 0xffffffC4  Reserved */
	r_reserved_exception,					/* 0xffffffC8  Reserved */
	r_reserved_exception,					/* 0xffffffCC  Reserved */
	r_privileged_exception,					/* 0xffffffd0  Exception(Supervisor Instruction) */
	r_access_exception,						/* 0xffffffd4  Exception(Access Instruction) */
	r_reserved_exception,					/* 0xffffffd8  Reserved */
	r_undefined_exception,					/* 0xffffffdc  Exception(Undefined Instruction) */
	r_reserved_exception,					/* 0xffffffe0  Reserved */
	r_floatingpoint_exception,				/* 0xffffffe4  Exception(Floating Point) */
	r_undefined_exception,					/* 0xffffffe8  Reserved */
	r_undefined_exception,					/* 0xffffffec  Reserved */
	r_undefined_exception,					/* 0xfffffff0  Reserved */
	r_undefined_exception,					/* 0xfffffff4  Reserved */
	r_nmi_exception							/* 0xfffffff8  NMI */
};

#define FVECT_SECT    __attribute__ ((section (".fvectors")))

extern void		_start(void);				// defined in rx65n_head.S

const void	*HardwareVectors[]	FVECT_SECT = {	/* 0xfffffffc  RESET */
	/* <<VECTOR DATA START (POWER ON RESET)>> */
	_start										/* Power On Reset PC */
	/* <<VECTOR DATA END (POWER ON RESET)>> */
};

#define RVECT_SECT __attribute__ ((section (".rvectors")))

const void	*RelocatableVectors[256] RVECT_SECT  = { 0 };
