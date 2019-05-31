/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "rx65n_macrodriver.h"
#include "rx65n_icu.h"


/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/


/***********************************************************************************************************************
* Function Name: R_ICU_Create
* Description  : This function initializes ICU module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_Create(void)
{
    /* Disable IRQ interrupts */
    ICU.IER[0x08].BYTE = _00_ICU_IRQ0_DISABLE | _00_ICU_IRQ1_DISABLE | _00_ICU_IRQ2_DISABLE | _00_ICU_IRQ3_DISABLE |
                         _00_ICU_IRQ4_DISABLE | _00_ICU_IRQ5_DISABLE | _00_ICU_IRQ6_DISABLE | _00_ICU_IRQ7_DISABLE;
    ICU.IER[0x09].BYTE = _00_ICU_IRQ8_DISABLE | _00_ICU_IRQ9_DISABLE | _00_ICU_IRQ10_DISABLE | _00_ICU_IRQ11_DISABLE |
                         _00_ICU_IRQ12_DISABLE | _00_ICU_IRQ13_DISABLE | _00_ICU_IRQ14_DISABLE | _00_ICU_IRQ15_DISABLE;

    /* Disable group interrupts */
    IEN(ICU,GROUPBL0) = 0U;
    /* Set IRQ settings */
    ICU.IRQCR[8].BYTE = _04_ICU_IRQ_EDGE_FALLING;
    ICU.IRQCR[9].BYTE = _04_ICU_IRQ_EDGE_FALLING;
    /* Set IRQ8 priority level */
    IPR(ICU,IRQ8) = _0F_ICU_PRIORITY_LEVEL15;
    /* Set IRQ9 priority level */
    IPR(ICU,IRQ9) = _0F_ICU_PRIORITY_LEVEL15;
    /* Set Group BL0 priority level */
    IPR(ICU,GROUPBL0) = _0F_ICU_PRIORITY_LEVEL15;
    /* Enable group BL0 interrupt */
    IEN(ICU,GROUPBL0) = 1U;

    /* Disable software interrupt */
    IEN(ICU,SWINT) = 0U;
    IEN(ICU,SWINT2) = 0U;
    /* Set SWINT priority level */
    IPR(ICU,SWINT) = _0F_ICU_PRIORITY_LEVEL15;
    /* Set SWINT2 priority level */
    IPR(ICU,SWINT2) = _0F_ICU_PRIORITY_LEVEL15;

    /* Set IRQ8 pin */
    MPC.P00PFS.BYTE = 0x40U;
    PORT0.PDR.BYTE &= 0xFEU;
    PORT0.PMR.BYTE &= 0xFEU;

    /* Set IRQ9 pin */
    MPC.P01PFS.BYTE = 0x40U;
    PORT0.PDR.BYTE &= 0xFDU;
    PORT0.PMR.BYTE &= 0xFDU;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ8_Start
* Description  : This function enables IRQ8 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ8_Start(void)
{
    /* Enable IRQ8 interrupt */
    IEN(ICU,IRQ8) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ8_Stop
* Description  : This function disables IRQ8 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ8_Stop(void)
{
    /* Disable IRQ8 interrupt */
    IEN(ICU,IRQ8) = 0U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ9_Start
* Description  : This function enables IRQ9 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ9_Start(void)
{
    /* Enable IRQ9 interrupt */
    IEN(ICU,IRQ9) = 1U;
}
/***********************************************************************************************************************
* Function Name: R_ICU_IRQ9_Stop
* Description  : This function disables IRQ9 interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ICU_IRQ9_Stop(void)
{
    /* Disable IRQ9 interrupt */
    IEN(ICU,IRQ9) = 0U;
}


/***********************************************************************************************************************
* Function Name: R_Config_ICU_Software_Start
* Description  : This function enables SWINT interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_Software_Start(void)
{
    /* Enable software interrupt */
    IEN(ICU,SWINT) = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_ICU_SoftwareInterrupt_Generate
* Description  : This function generates SWINT interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_SoftwareInterrupt_Generate(void)
{
    /* Generate software interrupt */
    ICU.SWINTR.BIT.SWINT = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_ICU_Software_Stop
* Description  : This function disables SWINT interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_Software_Stop(void)
{
    /* Disable software interrupt */
    IEN(ICU,SWINT) = 0U;
}

/***********************************************************************************************************************
* Function Name: R_Config_ICU_Software2_Start
* Description  : This function enables SWINT2 interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_Software2_Start(void)
{
    /* Enable software interrupt 2 */
    IEN(ICU,SWINT2) = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_ICU_SoftwareInterrupt2_Generate
* Description  : This function generates SWINT2 interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_SoftwareInterrupt2_Generate(void)
{
    /* Generate software interrupt 2 */
    ICU.SWINT2R.BIT.SWINT2 = 1U;
}

/***********************************************************************************************************************
* Function Name: R_Config_ICU_Software2_Stop
* Description  : This function disables SWINT2 interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_ICU_Software2_Stop(void)
{
    /* Disable software interrupt 2 */
    IEN(ICU,SWINT2) = 0U;
}

/* Start user code for adding. Do not edit comment generated here */
/*******************************************************************************
* Function Name: R_ICU_IRQIsFallingEdge
* Description : This function returns 1 if the specified ICU_IRQ is set to
* falling edge triggered, otherwise 0.
* Arguments : uint8_t irq_no
* Return Value : 1 if falling edge triggered, 0 if not
*******************************************************************************/
uint8_t R_ICU_IRQIsFallingEdge (const uint8_t irq_no)
{
uint8_t falling_edge_trig = 0x0;
if (ICU.IRQCR[irq_no].BYTE & _04_ICU_IRQ_EDGE_FALLING)
{
falling_edge_trig = 1;
}
return (falling_edge_trig);
}
/*******************************************************************************
* End of function R_ICU_IRQIsFallingEdge
*******************************************************************************/
/*******************************************************************************
* Function Name: R_ICU_IRQSetFallingEdge
* Description : This function sets/clears the falling edge trigger for the
* specified ICU_IRQ.
* Arguments : uint8_t irq_no
* uint8_t set_f_edge, 1 if setting falling edge triggered, 0 if
* clearing
* Return Value : None
*******************************************************************************/
void R_ICU_IRQSetFallingEdge (const uint8_t irq_no, const uint8_t set_f_edge)
{
if (1 == set_f_edge)
{
ICU.IRQCR[irq_no].BYTE |= _04_ICU_IRQ_EDGE_FALLING;
}
else
{
ICU.IRQCR[irq_no].BYTE &= (uint8_t) ~_04_ICU_IRQ_EDGE_FALLING;
}
}
/******************************************************************************
* End of function R_ICU_IRQSetFallingEdge
*******************************************************************************/
/*******************************************************************************
* Function Name: R_ICU_IRQSetRisingEdge
* Description : This function sets/clear the rising edge trigger for the
* specified ICU_IRQ.
* Arguments : uint8_t irq_no
* uint8_t set_r_edge, 1 if setting rising edge triggered, 0 if
* clearing
* Return Value : None
*******************************************************************************/
void R_ICU_IRQSetRisingEdge (const uint8_t irq_no, const uint8_t set_r_edge)
{
if (1 == set_r_edge)
{
ICU.IRQCR[irq_no].BYTE |= _08_ICU_IRQ_EDGE_RISING;
}
else
{
ICU.IRQCR[irq_no].BYTE &= (uint8_t) ~_08_ICU_IRQ_EDGE_RISING;
}
}
/******************************************************************************
* End of function R_ICU_IRQSetRisingEdge
*******************************************************************************/
/* End user code. Do not edit comment generated here */
