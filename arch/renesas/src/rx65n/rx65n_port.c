/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "rx65n_macrodriver.h"
#include "rx65n_port.h"
/* Start user code for include. Do not edit comment generated here */
#include "arch/board/board.h"
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_PORT_Create
* Description  : This function initializes the Port I/O.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	R_PORT_Create(void)
{
#if		defined(CONFIG_ARCH_BOARD_RX65N_RSK1MB)
	//LED_PORTINIT(0); /* Commented by TATA due to compilation error */
	
	PORT0.PODR.BYTE = _04_Pm2_OUTPUT_1 | _08_Pm3_OUTPUT_1 | _20_Pm5_OUTPUT_1;
	PORT5.PODR.BYTE = _40_Pm6_OUTPUT_1;
	PORT7.PODR.BYTE = _08_Pm3_OUTPUT_1;
	PORT9.PODR.BYTE = _08_Pm3_OUTPUT_1;
	PORTJ.PODR.BYTE = _20_Pm5_OUTPUT_1;
	PORT0.DSCR.BYTE = _00_Pm2_HIDRV_OFF;
	PORT0.DSCR2.BYTE = _00_Pm2_HISPEED_OFF;
	PORT5.DSCR.BYTE = _20_Pm5_HIDRV_ON | _00_Pm6_HIDRV_OFF;
	PORT5.DSCR2.BYTE = _00_Pm5_HISPEED_OFF | _00_Pm6_HISPEED_OFF;
	PORT7.DSCR2.BYTE = _00_Pm3_HISPEED_OFF;
	PORT9.DSCR.BYTE = _00_Pm3_HIDRV_OFF;
	PORT9.DSCR2.BYTE = _00_Pm3_HISPEED_OFF;
	PORT0.PMR.BYTE = 0x00U;
	PORT0.PDR.BYTE = _04_Pm2_MODE_OUTPUT | _08_Pm3_MODE_OUTPUT | _20_Pm5_MODE_OUTPUT | _50_PDR0_DEFAULT;
	PORT5.PMR.BYTE = 0x00U;
	PORT5.PDR.BYTE = _20_Pm5_MODE_OUTPUT | _40_Pm6_MODE_OUTPUT | _80_PDR5_DEFAULT;
	PORT7.PMR.BYTE = 0x00U;
	PORT7.PDR.BYTE = _08_Pm3_MODE_OUTPUT;
	PORT9.PMR.BYTE = 0x00U;
	PORT9.PDR.BYTE = _08_Pm3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
	PORTJ.PMR.BYTE = 0x00U;
	PORTJ.PDR.BYTE = _20_Pm5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
#elif	defined (CONFIG_ARCH_BOARD_RX65N_RSK2MB)
//	LED_PORTINIT(0);
	PORT0.PODR.BYTE = _04_Pm2_OUTPUT_1 | _08_Pm3_OUTPUT_1 | _20_Pm5_OUTPUT_1;
	PORT5.PODR.BYTE = _40_Pm6_OUTPUT_1;
	PORT7.PODR.BYTE = _08_Pm3_OUTPUT_1;
	PORT9.PODR.BYTE = _08_Pm3_OUTPUT_1;
	PORTJ.PODR.BYTE = _20_Pm5_OUTPUT_1;
	PORT0.DSCR.BYTE = _00_Pm2_HIDRV_OFF;
	PORT0.DSCR2.BYTE = _00_Pm2_HISPEED_OFF;
	PORT5.DSCR.BYTE = _20_Pm5_HIDRV_ON | _00_Pm6_HIDRV_OFF;
	PORT5.DSCR2.BYTE = _00_Pm5_HISPEED_OFF | _00_Pm6_HISPEED_OFF;
	PORT7.DSCR2.BYTE = _00_Pm3_HISPEED_OFF;
	PORT9.DSCR.BYTE = _00_Pm3_HIDRV_OFF;
	PORT9.DSCR2.BYTE = _00_Pm3_HISPEED_OFF;
	PORT0.PMR.BYTE = 0x00U;
	PORT0.PDR.BYTE = _04_Pm2_MODE_OUTPUT | _08_Pm3_MODE_OUTPUT | _20_Pm5_MODE_OUTPUT | _50_PDR0_DEFAULT;
	PORT5.PMR.BYTE = 0x00U;
	PORT5.PDR.BYTE = _20_Pm5_MODE_OUTPUT | _40_Pm6_MODE_OUTPUT | _80_PDR5_DEFAULT;
	PORT7.PMR.BYTE = 0x00U;
	PORT7.PDR.BYTE = _08_Pm3_MODE_OUTPUT;
	PORT9.PMR.BYTE = 0x00U;
	PORT9.PDR.BYTE = _08_Pm3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
	PORTJ.PMR.BYTE = 0x00U;
	PORTJ.PDR.BYTE = _20_Pm5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
#elif	defined(_CONFIGS_RX65N_GRROSE_BOARD_H)
	LED_PORTINIT(0);
	PORT2.PODR.BIT.B2 = 0;	PORT2.PMR.BIT.B2 = 0;	PORT2.PDR.BIT.B2 = 1;	// SCI0(UART)  direction
	PORT1.PODR.BIT.B4 = 0;	PORT1.PMR.BIT.B4 = 0;	PORT1.PDR.BIT.B4 = 1;	// SCI2(UART)  direction
	PORTC.PODR.BIT.B4 = 0;	PORTC.PMR.BIT.B4 = 0;	PORTC.PDR.BIT.B4 = 1;	// SCI5(UART)  direction
	PORT3.PODR.BIT.B4 = 0;	PORT3.PMR.BIT.B4 = 0;	PORT3.PDR.BIT.B4 = 1;	// SCI6(UART)  direction
	PORTC.PODR.BIT.B5 = 0;	PORTC.PMR.BIT.B5 = 0;	PORTC.PDR.BIT.B5 = 1;	// SCI8(RS485) direction
#else
	// nothing to do
#endif
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
