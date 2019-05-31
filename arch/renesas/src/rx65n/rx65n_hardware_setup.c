/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "rx65n_macrodriver.h"
#include "rx65n_cgc.h"
#include "rx65n_icu.h"
#include "rx65n_port.h"
#include "rx65n_sci.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "rx_65n.h"

//#define 	USE_DEBUG_COMMENT_OUTPUT

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

int HardwareSetup(void);
static void R_Systeminit(void);

#ifdef	USE_DEBUG_COMMENT_OUTPUT
void	test_message(
	unsigned char	*msg1,
	unsigned char	len1
)
{
	for( int i = 0; i < len1; i++ ) {
		SCI1.TDR = msg1[i];
		while(0 == SCI1.SSR.BIT.TDRE); // waiting for completing tx
	}
}
#endif	// USE_DEBUG_COMMENT_OUTPUT

/***********************************************************************************************************************
* Function Name: R_Systeminit
* Description  : This function initializes every macro.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Systeminit(void)
{
	/* Enable writing to registers related to operating modes, LPC, CGC and software reset */
	SYSTEM.PRCR.WORD = 0xA50BU;

	/* Enable writing to MPC pin function control registers */
	MPC.PWPR.BIT.B0WI = 0U;
	MPC.PWPR.BIT.PFSWE = 1U;
	
	/* Set peripheral settings */
	R_CGC_Create();
	R_ICU_Create();
	R_PORT_Create();
#ifdef	CONFIG_RX65N_SCI0
	R_SCI0_Create();
	R_SCI0_Start();
	SCI0.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI1
	R_SCI1_Create();
	R_SCI1_Start();
	SCI1.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI2
	R_SCI2_Create();
	R_SCI2_Start();
	SCI2.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI3
	R_SCI3_Create();
	R_SCI3_Start();
	SCI3.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI4
	R_SCI4_Create();
	R_SCI4_Start();
	SCI4.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI5
	R_SCI5_Create();
	R_SCI5_Start();
	SCI5.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI6
	R_SCI6_Create();
	R_SCI6_Start();
	SCI6.SCR.BIT.TE = 1U;
#endif
#ifdef	CONFIG_RX65N_SCI8
	R_SCI8_Create();
	R_SCI8_Start();
	SCI8.SCR.BIT.TE = 1U;
#endif
	/* Disable writing to MPC pin function control registers */
	MPC.PWPR.BIT.PFSWE = 0U;
	MPC.PWPR.BIT.B0WI = 1U;

	/* Enable protection */
	SYSTEM.PRCR.WORD = 0xA500U;

	R_Config_ICU_Software_Start();
	R_Config_ICU_Software2_Start();

#ifdef	USE_DEBUG_COMMENT_OUTPUT
	test_message("test\n", 5 );
	__asm("nop");  // for debug (as break-point)
#endif	// USE_DEBUG_COMMENT_OUTPUT
}
/***********************************************************************************************************************
* Function Name: HardwareSetup
* Description  : This function initializes hardware setting.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
int HardwareSetup(void)
{
	R_Systeminit();

	return (1U);
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */


