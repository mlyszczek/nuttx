/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "rx65n_macrodriver.h"
#include "rx65n_cgc.h"
/* Start user code for include. Do not edit comment generated here */
#include "arch/board/board.h"
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_CGC_Create
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CGC_Create(void)
{
#if	((24 * RX_CLK_1MHz) == RX_RESONATOR)
	/* Set main clock control registers */
	SYSTEM.MOFCR.BYTE = _00_CGC_MAINOSC_RESONATOR | _00_CGC_MAINOSC_UNDER24M;
	SYSTEM.MOSCWTCR.BYTE = _5C_CGC_MOSCWTCR_VALUE;

	/* Set main clock operation */
	SYSTEM.MOSCCR.BIT.MOSTP = 0U;

	/* Wait for main clock oscillator wait counter overflow */
	while (1U != SYSTEM.OSCOVFSR.BIT.MOOVF)
	{
		/* Do nothing */
	}

	/* Set system clock */
	SYSTEM.SCKCR.LONG = _00000002_CGC_PCLKD_DIV_4 | _00000020_CGC_PCLKC_DIV_4 | _00000200_CGC_PCLKB_DIV_4 |
						_00001000_CGC_PCLKA_DIV_2 | _00010000_CGC_BCLK_DIV_2 | _01000000_CGC_ICLK_DIV_2 |
						_20000000_CGC_FCLK_DIV_4;

	/* Set PLL circuit */
	SYSTEM.PLLCR.WORD = _0000_CGC_PLL_FREQ_DIV_1 | _0000_CGC_PLL_SOURCE_MAIN | _1300_CGC_PLL_FREQ_MUL_10_0;
	SYSTEM.PLLCR2.BIT.PLLEN = 0U;

	/* Wait for PLL wait counter overflow */
	while (1U != SYSTEM.OSCOVFSR.BIT.PLOVF)
	{
		/* Do nothing */
	}

	/* Stop sub-clock */
	RTC.RCR3.BIT.RTCEN = 0U;

	/* Wait for the register modification to complete */
	while (0U != RTC.RCR3.BIT.RTCEN)
	{
		/* Do nothing */
	}

	/* Stop sub-clock */
	SYSTEM.SOSCCR.BIT.SOSTP = 1U;

	/* Wait for the register modification to complete */
	while (1U != SYSTEM.SOSCCR.BIT.SOSTP)
	{
		/* Do nothing */
	}

	/* Wait for sub-clock oscillation stopping */
	while (0U != SYSTEM.OSCOVFSR.BIT.SOOVF)
	{
		/* Do nothing */
	}

	/* Set UCLK */
	SYSTEM.SCKCR2.WORD = _0040_CGC_UCLK_DIV_5 | _0001_SCKCR2_BIT0;

	/* Set ROM wait cycle */
	SYSTEM.ROMWT.BYTE = _02_CGC_ROMWT_CYCLE_2;

	/* Set SDCLK */
	SYSTEM.SCKCR.BIT.PSTOP0 = 1U;

	/* Set clock source */
	SYSTEM.SCKCR3.WORD = _0400_CGC_CLOCKSOURCE_PLL;

	/* Set LOCO */
	SYSTEM.LOCOCR.BIT.LCSTP = 1U;
#elif	((12 * RX_CLK_1MHz) == RX_RESONATOR)
	SYSTEM.MOFCR.BIT.MOFXIN = 0;
	SYSTEM.MOFCR.BIT.MOSEL = 0;
	if(1 == SYSTEM.HOCOCR.BIT.HCSTP) {
		SYSTEM.HOCOPCR.BYTE = 0x01;
	}
	else {
		while(0 == SYSTEM.OSCOVFSR.BIT.HCOVF);
	}
	SYSTEM.MOFCR.BIT.MODRV2 = 2;
	SYSTEM.MOSCWTCR.BYTE = 0x53;
	SYSTEM.MOSCCR.BYTE = 0x00;
	if(0x00 ==  SYSTEM.MOSCCR.BYTE) {
		__asm("nop");
	}
	while(0 == SYSTEM.OSCOVFSR.BIT.MOOVF);
	if(0 == SYSTEM.RSTSR1.BIT.CWSF) {
		volatile uint8_t i;
		volatile uint8_t dummy;

		RTC.RCR4.BIT.RCKSEL = 0;
		for (i = 0; i < 4; i++) {
			dummy = RTC.RCR4.BYTE;
		}
		if (0 != RTC.RCR4.BIT.RCKSEL) {
			__asm("nop");
		}
		RTC.RCR3.BIT.RTCEN = 0;
		for (i = 0; i < 4; i++) {
			dummy = RTC.RCR3.BYTE;
		}
		if (0 != RTC.RCR3.BIT.RTCEN) {
			__asm("nop");
		}
		SYSTEM.SOSCCR.BYTE = 0x01;
		if (0x01 != SYSTEM.SOSCCR.BYTE) {
			__asm("nop");
		}
		while (0 != SYSTEM.OSCOVFSR.BIT.SOOVF);
	}
	else {
		SYSTEM.SOSCCR.BYTE = 0x01;
		if (0x01 != SYSTEM.SOSCCR.BYTE) {
			__asm("nop");
		}
		while (0 != SYSTEM.OSCOVFSR.BIT.SOOVF);
	}
	SYSTEM.PLLCR.BIT.PLIDIV = 0;
	SYSTEM.PLLCR.BIT.PLLSRCSEL = 0;
	SYSTEM.PLLCR.BIT.STC = (20 * 2) - 1;
	SYSTEM.PLLCR2.BYTE = 0x00;
	while(0 == SYSTEM.OSCOVFSR.BIT.PLOVF);
	SYSTEM.ROMWT.BYTE = 0x02;
	if(0x02 == SYSTEM.ROMWT.BYTE) {
		__asm("nop");
	}
	//
	SYSTEM.SCKCR.LONG = 0x21C11222;
	SYSTEM.SCKCR2.WORD = 0x0011;
	SYSTEM.SCKCR3.WORD = 4U << 8;	// BSP_CFG_CLOCK_SOURCE
	SYSTEM.LOCOCR.BYTE = 0x01;
#else
#error "RX_RESONATOR is not defined in board.h"
#endif
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
