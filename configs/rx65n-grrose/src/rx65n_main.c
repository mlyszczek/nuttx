/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "rx65n_macrodriver.h"
#include "rx65n_cgc.h"
#include "rx65n_icu.h"
#include "rx65n_port.h"
#include "rx65n_sci.h"
#include "arch/board/board.h"
#include "rx65n_rsk_def.h"
void board_autoled_on(int led);
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* Prototype declaration for led_display_count */



static void R_MAIN_UserInit(void);
void	board_autoled1_on(int led)
{
	LED0=LED_ON;
}
void	board_autoled2_on(int led)
{
	LED1=LED_ON;
}
void	board_autoled1_off(int led)
{
	LED0=LED_OFF;
}
void	board_autoled2_off(int led)
{
	LED1=LED_OFF;
}
void	board_autoled_off(int led)
{
	LED0=LED_OFF;
	LED1=LED_OFF;
}
/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	main(void)
{
	R_MAIN_UserInit();
	/* Start user code. Do not edit comment generated here */
	/* Initialize the switch module */
//    R_SWITCH_Init();
	/* Set the call back function when SW1 or SW2 is pressed */
//    R_SWITCH_SetPressCallback(cb_switch_press);
	/* Start the A/D converter */
	//R_S12AD0_Start();
	/* Set up SCI2 receive buffer and callback function */
//    R_SCI2_Serial_Receive((uint8_t *)&g_rx_char, 10);
	/* Enable SCI2 operations */
//    R_SCI2_Start();
#if 0
	while (1U)
	{

		uint16_t adc_result;
		/* Wait for user requested A/D conversion flag to be set (SW1 or SW2) */
		if (TRUE == g_adc_trigger)
		{
		/* Call the function to perform an A/D conversion */
		adc_result = get_adc();
		/* Display the result on the LCD */
//lcd_display_adc(adc_result);
		/* Increment the adc_count */
		if (16 == (++adc_count))
		{
		adc_count = 0;
		}

		led_display_count(adc_count);
		/* Send the result to the UART */
		uart_display_adc(adc_count, adc_result);
		/* Reset the flag */
		g_adc_trigger = FALSE;
		}
		/* SW3 is directly wired into the ADTRG0n pin so will
		cause the interrupt to fire */
		else if (TRUE == g_adc_complete)
		{
		/* Get the result of the A/D conversion */
		R_S12AD0_Get_ValueResult(ADCHANNEL0, &adc_result);
		/* Display the result on the LCD */
   // 	lcd_display_adc(adc_result);
		/* Increment the adc_count */
		if (16 == (++adc_count))
		{
		adc_count = 0;
		}
		/* Send the result to the UART */
		uart_display_adc(adc_count, adc_result);
		/* Reset the flag */
		g_adc_complete = FALSE;
		}
		else
		{
		/* do nothing */
		}
		;
	}
#endif
	/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
	/* Start user code. Do not edit comment generated here */
	/* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/******************************************************************************
* Function Name : cb_switch_press
* Description : Switch press callback function. Sets g_adc_trigger flag.
* Argument : none
* Return value : none
******************************************************************************/

#if 0
static void cb_switch_press (void)
{
/* Check if switch 1 or 2 was pressed */
if (g_switch_flag & (SWITCHPRESS_1 | SWITCHPRESS_2))
{
	/* set the flag indicating a user requested A/D conversion is required */
	g_adc_trigger = TRUE;
	/* Clear flag */
	g_switch_flag = 0x0;
	}
}

#endif
	/******************************************************************************
	* End of function cb_switch_press
	******************************************************************************/
	/******************************************************************************
	* Function Name : get_adc
	* Description : Reads the ADC result, converts it to a string and displays
	* it on the LCD panel.
	* Argument : none
	* Return value : uint16_t adc value
	******************************************************************************/
#if 0
		static uint16_t get_adc (void)
	{
	/* A variable to retrieve the adc result */
	uint16_t adc_result;
	/* Stop the A/D converter being triggered from the pin ADTRG0n */
	R_S12AD0_Stop();
	/* Start a conversion */
	R_S12AD0_SWTriggerStart();
	/* Wait for the A/D conversion to complete */
	while (FALSE == g_adc_complete)
	{
	/* Wait */
	}
	/* Stop conversion */
	R_S12AD0_SWTriggerStop();
	/* Clear ADC flag */
	g_adc_complete = FALSE;
	R_S12AD0_Get_ValueResult(ADCHANNEL0, &adc_result);
	/* Set AD conversion start trigger source back to ADTRG0n pin */
	R_S12AD0_Start();
	return (adc_result);
	}
	/******************************************************************************
	* End of function get_adc
	******************************************************************************/
#endif

/******************************************************************************
* Function Name : led_display_count
* Description : Converts count to binary and displays on 4 LEDS0-3
* Argument : uint8_t count
* Return value : none
******************************************************************************/
#if 0
static void led_display_count (const uint8_t count)
{
/* Set LEDs according to lower nibble of count parameter */
LED0 = (uint8_t)((count & 0x01) ? LED_ON : LED_OFF);
LED1 = (uint8_t)((count & 0x02) ? LED_ON : LED_OFF);
//LED2 = (uint8_t)((count & 0x04) ? LED_ON : LED_OFF);
//LED3 = (uint8_t)((count & 0x08) ? LED_ON : LED_OFF);
}
#endif

/******************************************************************************
* End of function led_display_count
******************************************************************************/
/******************************************************************************
* Function Name : uart_display_adc
* Description : Converts adc result to a string and sends it to the UART1.
* Argument : uint8_t : adc_count
* uint16_t: adc result
* Return value : none
******************************************************************************/

#if 0
static void uart_display_adc (const uint8_t adc_count, const uint16_t adc_result)
{
/* Declare a temporary variable */
char a;
/* Declare temporary character string */
static char uart_buffer[] = "ADC xH Value: xxxH\r\n";
/* Convert ADC result into a character string, and store in the local.
Casting to ensure use of correct data type. */
a = (char)(adc_count & 0x000F);
uart_buffer[4] = (char)((a < 0x0A) ? (a + 0x30) : (a + 0x37));
a = (char)((adc_result & 0x0F00) >> 8);
uart_buffer[14] = (char)((a < 0x0A) ? (a + 0x30) : (a + 0x37));
a = (char)((adc_result & 0x00F0) >> 4);
uart_buffer[15] = (char)((a < 0x0A) ? (a + 0x30) : (a + 0x37));
a = (char)(adc_result & 0x000F);
uart_buffer[16] = (char)((a < 0x0A) ? (a + 0x30) : (a + 0x37));
/* Send the string to the UART */
R_DEBUG_Print(uart_buffer);

}

#endif
/******************************************************************************
* End of function uart_display_adc
******************************************************************************/
/* End user code. Do not edit comment generated here */
//void up_putc(){

//}
void board_autoled_on(int led){
//LED3= LED_ON;
//sleep(5);
//LED2= LED_ON;
//LED3=LED_OFF;
//sleep(50);
//LED1=LED_ON;
//LED2=LED_ON;
//LED3=LED_ON;
}

//void board_autoled_off(int led){
//}
