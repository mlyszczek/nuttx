/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "sys/types.h"
#include "rx65n_macrodriver.h"
#include "rx65n_sci.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "rx_65n.h"

#define debug 0

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

static inline
void	RX_MPC_enable( void )
{
	/* Enable writing to registers related to operating modes, LPC, CGC and software reset */
	SYSTEM.PRCR.WORD = 0xA50BU;
	/* Enable writing to MPC pin function control registers */
	MPC.PWPR.BIT.B0WI = 0U;
	MPC.PWPR.BIT.PFSWE = 1U;
}

static inline
void	RX_MPC_disable( void )
{
	/* Disable writing to MPC pin function control registers */
	MPC.PWPR.BIT.PFSWE = 0U;
	MPC.PWPR.BIT.B0WI = 1U;
	/* Enable protection */
	SYSTEM.PRCR.WORD = 0xA500U;
}

/* Start user code for global. Do not edit comment generated here */
#ifdef	CONFIG_RX65N_SCI0
volatile uint8_t	*gp_sci0_tx_address;			/* SCI0 transmit buffer address		*/
volatile uint8_t	*gp_sci0_rx_address;			/* SCI0 receive buffer address		*/
volatile uint16_t	g_sci0_tx_count;				/* SCI0 transmit data number		*/
volatile uint16_t	g_sci0_rx_count;				/* SCI0 receive data number			*/
volatile uint16_t	g_sci0_rx_length;				/* SCI0 receive data length			*/

static inline
void	SCI0_init_port( void )
{
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK1MB
	/* Set RXD0 pin (PXX) */
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PMR.BIT.BX	= 1U;
	/* Set TXD0 pin (PXX) */
//	PORTX.PODR.BIT.BX	= 1U;
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PDR.BIT.BX	= 1U;
//	PORTX.PMR.BIT.BX	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK1MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK2MB
	/* Set RXD0 pin (PXX) */
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PMR.BIT.BX	= 1U;
	/* Set TXD0 pin (PXX) */
//	PORTX.PODR.BIT.BX	= 1U;
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PDR.BIT.BX	= 1U;
//	PORTX.PMR.BIT.BX	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK2MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_GRROSE
	/* Set RXD0 pin (P21) */
	MPC.P21PFS.BYTE		= 0x0AU;
	PORT2.PMR.BIT.B1	= 1U;
	/* Set TXD0 pin (P20) */
	PORT2.PODR.BIT.B0	= 1U;
	MPC.P20PFS.BYTE		= 0x0AU;
	PORT2.PDR.BIT.B0	= 1U;
	PORT2.PMR.BIT.B0	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_GRROSE
}
#endif
#ifdef	CONFIG_RX65N_SCI1
volatile uint8_t	*gp_sci1_tx_address;			/* SCI1 transmit buffer address		*/
volatile uint8_t	*gp_sci1_rx_address;			/* SCI1 receive buffer address		*/
volatile uint16_t	g_sci1_tx_count;				/* SCI1 transmit data number		*/
volatile uint16_t	g_sci1_rx_count;				/* SCI1 receive data number			*/
volatile uint16_t	g_sci1_rx_length;				/* SCI1 receive data length			*/

static inline
void	SCI1_init_port( void )
{
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK1MB
	/* Set RXD1 pin (PXX) */
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PMR.BIT.BX	= 1U;
	/* Set TXD1 pin (PXX) */
//	PORTX.PODR.BIT.BX	= 1U;
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PDR.BIT.BX	= 1U;
//	PORTX.PMR.BIT.BX	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK1MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK2MB
	/* Set RXD1 pin (PF2) */
	MPC.PF2PFS.BYTE		= 0x0AU;
	PORTF.PMR.BIT.B2	= 1U;
	/* Set TXD1 pin (PF1) */
	PORTF.PODR.BIT.B1	= 1U;
	MPC.PF1PFS.BYTE		= 0x0AU;
	PORTF.PDR.BIT.B1	= 1U;
	PORTF.PMR.BIT.B1	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK2MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_GRROSE
	/* Set RXD1 pin (P30) */
	MPC.P30PFS.BYTE		= 0x0AU;
	PORT3.PMR.BIT.B0	= 1U;
	/* Set TXD1 pin (P26) */
	PORT2.PODR.BIT.B6	= 1U;
	MPC.P26PFS.BYTE		= 0x0AU;
	PORT2.PDR.BIT.B6	= 1U;
	PORT2.PMR.BIT.B6	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_GRROSE
}
#endif
#ifdef	CONFIG_RX65N_SCI2
volatile uint8_t	*gp_sci2_tx_address;			/* SCI2 transmit buffer address		*/
volatile uint8_t	*gp_sci2_rx_address;			/* SCI2 receive buffer address		*/
volatile uint16_t	g_sci2_tx_count;				/* SCI2 transmit data number		*/
volatile uint16_t	g_sci2_rx_count;				/* SCI2 receive data number			*/
volatile uint16_t	g_sci2_rx_length;				/* SCI2 receive data length			*/

static inline
void	SCI2_init_port( void )
{
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK1MB
	/* Set RXD2 pin (P52) */
	MPC.P52PFS.BYTE		= 0x0AU;
	PORT5.PMR.BIT.B2	= 1U;
	/* Set TXD2 pin (P50) */
	PORT5.PODR.BIT.B0	= 1U;
	MPC.P50PFS.BYTE		= 0x0AU;
	PORT5.PDR.BIT.B0	= 1U;
	PORT5.PMR.BIT.B0	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK1MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK2MB
	/* Set RXD2 pin (P52) */
	MPC.P52PFS.BYTE		= 0x0AU;
	PORT5.PMR.BIT.B2	= 1U;
	/* Set TXD2 pin (P50) */
	PORT5.PODR.BIT.B0	= 1U;
	MPC.P50PFS.BYTE		= 0x0AU;
	PORT5.PDR.BIT.B0	= 1U;
	PORT5.PMR.BIT.B0	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK2MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_GRROSE
#if 1	// for debugging
	/* Set RXD2 pin (P52) */
	MPC.P52PFS.BYTE		= 0x0AU;
	PORT5.PMR.BIT.B2	= 1U;
	/* Set TXD2 pin (P50) */
	PORT5.PODR.BIT.B0	= 1U;
	MPC.P50PFS.BYTE		= 0x0AU;
	PORT5.PDR.BIT.B0	= 1U;
	PORT5.PMR.BIT.B0	= 1U;
#else	// release
	/* Set RXD2 pin (P12) */
	MPC.P12PFS.BYTE		= 0x0AU;
	PORT1.PMR.BIT.B2	= 1U;
	/* Set TXD2 pin (P13) */
	PORT1.PODR.BIT.B3	= 1U;
	MPC.P13PFS.BYTE		= 0x0AU;
	PORT1.PDR.BIT.B3	= 1U;
	PORT1.PMR.BIT.B3	= 1U;
#endif
#endif	// CONFIG_ARCH_BOARD_RX65N_GRROSE
}
#endif
#ifdef	CONFIG_RX65N_SCI3
volatile uint8_t	*gp_sci3_tx_address;			/* SCI3 transmit buffer address		*/
volatile uint8_t	*gp_sci3_rx_address;			/* SCI3 receive buffer address		*/
volatile uint16_t	g_sci3_tx_count;				/* SCI3 transmit data number		*/
volatile uint16_t	g_sci3_rx_count;				/* SCI3 receive data number			*/
volatile uint16_t	g_sci3_rx_length;				/* SCI3 receive data length			*/

static inline
void	SCI3_init_port( void )
{
	/* Set RXD3 pin (PXX) */
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PMR.BIT.BX	= 1U;
	/* Set TXD3 pin (PXX) */
//	PORTX.PODR.BIT.BX	= 1U;
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PDR.BIT.BX	= 1U;
//	PORTX.PMR.BIT.BX	= 1U;
}
#endif
#ifdef	CONFIG_RX65N_SCI4
volatile uint8_t	*gp_sci4_tx_address;			/* SCI4 transmit buffer address		*/
volatile uint8_t	*gp_sci4_rx_address;			/* SCI4 receive buffer address		*/
volatile uint16_t	g_sci4_tx_count;				/* SCI4 transmit data number		*/
volatile uint16_t	g_sci4_rx_count;				/* SCI4 receive data number			*/
volatile uint16_t	g_sci4_rx_length;				/* SCI4 receive data length			*/

static inline
void	SCI4_init_port( void )
{
	/* Set RXD4 pin (PXX) */
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PMR.BIT.BX	= 1U;
	/* Set TXD4 pin (PXX) */
//	PORTX.PODR.BIT.BX	= 1U;
//	MPC.PXXPFS.BYTE		= 0x0AU;
//	PORTX.PDR.BIT.BX	= 1U;
//	PORTX.PMR.BIT.BX	= 1U;
}
#endif
#ifdef	CONFIG_RX65N_SCI5
volatile uint8_t	*gp_sci5_tx_address;			/* SCI5 transmit buffer address		*/
volatile uint8_t	*gp_sci5_rx_address;			/* SCI5 receive buffer address		*/
volatile uint16_t	g_sci5_tx_count;				/* SCI5 transmit data number		*/
volatile uint16_t	g_sci5_rx_count;				/* SCI5 receive data number			*/
volatile uint16_t	g_sci5_rx_length;				/* SCI5 receive data length			*/

static inline
void	SCI5_init_port( void )
{
#ifdef	CONFIG_ARCH_BOARD_RX65N_GRROSE
	/* Set RXD3 pin (PC2) */
	MPC.PC2PFS.BYTE		= 0x0AU;
	PORTC.PMR.BIT.B2	= 1U;
	/* Set TXD3 pin (PC3) */
	PORTC.PODR.BIT.B3	= 1U;
	MPC.PC3PFS.BYTE		= 0x0AU;
	PORTC.PDR.BIT.B3	= 1U;
	PORTC.PMR.BIT.B3	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_GRROSE
}
#endif
#ifdef	CONFIG_RX65N_SCI6
volatile uint8_t	*gp_sci6_tx_address;			/* SCI6 transmit buffer address		*/
volatile uint8_t	*gp_sci6_rx_address;			/* SCI6 receive buffer address		*/
volatile uint16_t	g_sci6_tx_count;				/* SCI6 transmit data number		*/
volatile uint16_t	g_sci6_rx_count;				/* SCI6 receive data number			*/
volatile uint16_t	g_sci6_rx_length;				/* SCI6 receive data length			*/

static inline
void	SCI6_init_port( void )
{
#ifdef	CONFIG_ARCH_BOARD_RX65N_GRROSE
	/* Set RXD6 pin (P33) */
	MPC.P33PFS.BYTE		= 0x0AU;
	PORT3.PMR.BIT.B3	= 1U;
	/* Set TXD6 pin (P32) */
	PORT3.PODR.BIT.B2	= 1U;
	MPC.P32PFS.BYTE		= 0x0AU;
	PORT3.PDR.BIT.B2	= 1U;
	PORT3.PMR.BIT.B2	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_GRROSE
}
#endif
#ifdef	CONFIG_RX65N_SCI8
volatile uint8_t	*gp_sci8_tx_address;			/* SCI8 transmit buffer address		*/
volatile uint8_t	*gp_sci8_rx_address;			/* SCI8 receive buffer address		*/
volatile uint16_t	g_sci8_tx_count;				/* SCI8 transmit data number		*/
volatile uint16_t	g_sci8_rx_count;				/* SCI8 receive data number			*/
volatile uint16_t	g_sci8_rx_length;				/* SCI8 receive data length			*/

static inline
void	SCI8_init_port( void )
{
#ifdef	CONFIG_ARCH_BOARD_RX65N_RSK2MB
	/* Set RXD8 pin (PJ1) */
	MPC.PJ1PFS.BYTE		= 0x0AU;
	PORTJ.PMR.BIT.B1	= 1U;
	/* Set TXD8 pin (PJ2) */
	PORTJ.PODR.BIT.B2	= 1U;
	MPC.PJ2PFS.BYTE		= 0x0AU;
	PORTJ.PDR.BIT.B2	= 1U;
	PORTJ.PMR.BIT.B2	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_RSK2MB
#ifdef	CONFIG_ARCH_BOARD_RX65N_GRROSE
	/* Set RXD8 pin (PC6) */
	MPC.PC6PFS.BYTE		= 0x0AU;
	PORTC.PMR.BIT.B6	= 1U;
	/* Set TXD8 pin (PC7) */
	PORTC.PODR.BIT.B7	= 1U;
	MPC.PC7PFS.BYTE		= 0x0AU;
	PORTC.PDR.BIT.B7	= 1U;
	PORTC.PMR.BIT.B7	= 1U;
#endif	// CONFIG_ARCH_BOARD_RX65N_GRROSE
}
#endif

volatile uint8_t *gp_sci7_tx_address;               /* SCI7 transmit buffer address */
volatile uint16_t g_sci7_tx_count;                  /* SCI7 transmit data number */
/* End user code. Do not edit comment generated here */


#ifdef	CONFIG_RX65N_SCI0
/***********************************************************************************************************************
 * Function Name: R_SCI0_Create
 * Description  : This function initializes SCI0.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI0_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI0)			= 0U;			/* Cancel SCI0 module stop state */
	IPR(SCI0,RXI0)		= 15;			/* Set interrupt priority */
	IPR(SCI0,TXI0)		= 15;			/* Set interrupt priority */
	SCI0.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI0.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI0.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI0.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI0.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI0.SPMR.BYTE		= _00_SCI_RTS;
	SCI0.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI0.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI0.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI0.BRR			= 0x40U;
	/* Set SCI0 pin */
	SCI0_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI0_Start
 * Description  : This function starts SCI0.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI0_Start(void)
{
	IR(SCI0,TXI0)		= 0U;	/* Clear interrupt flag */
	IR(SCI0,RXI0)		= 0U;	/* Clear interrupt flag */
	IEN(SCI0,TXI0)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN0	= 1U;
	IEN(SCI0,RXI0)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN1	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI0_Stop
 * Description  : This function stops SCI0.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI0_Stop(void)
{
	SCI0.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI0.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI0.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI0.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI0,TXI0)		= 0U;
	ICU.GENBL0.BIT.EN0	= 0U;
	IR(SCI0,TXI0)		= 0U;
	IEN(SCI0,RXI0)		= 0U;
	ICU.GENBL0.BIT.EN1	= 0U;
	IR(SCI0,RXI0)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI0_Serial_Receive
 * Description  : This function receives SCI0 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI0_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci0_rx_count		= 0U;
	g_sci0_rx_length	= rx_num;
	gp_sci0_rx_address	= rx_buf;
	SCI0.SCR.BIT.RIE	= 1U;
	SCI0.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI0_Serial_Send
 * Description  : This function transmits SCI0 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI0_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci0_tx_address	= tx_buf;
	g_sci0_tx_count		= tx_num;
	/* Set TXD0 pin */
	SCI0.SCR.BIT.TIE	= 1U;
	SCI0.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI1
/***********************************************************************************************************************
 * Function Name: R_SCI1_Create
 * Description  : This function initializes SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI1_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI1)			= 0U;			/* Cancel SCI1 module stop state */
	IPR(SCI1,RXI1)		= 15;			/* Set interrupt priority */
	IPR(SCI1,TXI1)		= 15;			/* Set interrupt priority */
	SCI1.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI1.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI1.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI1.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI1.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI1.SPMR.BYTE		= _00_SCI_RTS;
	SCI1.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI1.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI1.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI1.BRR			= 0x40U;
	/* Set SCI1 pin */
	SCI1_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI1_Start
 * Description  : This function starts SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI1_Start(void)
{
	IR(SCI1,TXI1)		= 0U;	/* Clear interrupt flag */
	IR(SCI1,RXI1)		= 0U;	/* Clear interrupt flag */
	IEN(SCI1,TXI1)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN2	= 1U;
	IEN(SCI1,RXI1)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN3	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI1_Stop
 * Description  : This function stops SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI1_Stop(void)
{
	SCI1.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI1.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI1.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI1.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI1,TXI1)		= 0U;
	ICU.GENBL0.BIT.EN2	= 0U;
	IR(SCI1,TXI1)		= 0U;
	IEN(SCI1,RXI1)		= 0U;
	ICU.GENBL0.BIT.EN3	= 0U;
	IR(SCI1,RXI1)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI1_Serial_Receive
 * Description  : This function receives SCI1 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI1_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci1_rx_count		= 0U;
	g_sci1_rx_length	= rx_num;
	gp_sci1_rx_address	= rx_buf;
	SCI1.SCR.BIT.RIE	= 1U;
	SCI1.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI1_Serial_Send
 * Description  : This function transmits SCI1 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI1_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci1_tx_address	= tx_buf;
	g_sci1_tx_count		= tx_num;
	/* Set TXD1 pin */
	SCI1.SCR.BIT.TIE	= 1U;
	SCI1.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI2
/***********************************************************************************************************************
 * Function Name: R_SCI2_Create
 * Description  : This function initializes SCI2.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI2_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI2)			= 0U;			/* Cancel SCI2 module stop state */
	IPR(SCI2,RXI2)		= 15;			/* Set interrupt priority */
	IPR(SCI2,TXI2)		= 15;			/* Set interrupt priority */
	SCI2.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI2.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI2.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI2.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI2.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI2.SPMR.BYTE		= _00_SCI_RTS;
	SCI2.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI2.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI2.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI2.BRR			= 0x40U;
	/* Set SCI2 pin */
	SCI2_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI2_Start
 * Description  : This function starts SCI2.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI2_Start(void)
{
	IR(SCI2,TXI2)		= 0U;	/* Clear interrupt flag */
	IR(SCI2,RXI2)		= 0U;	/* Clear interrupt flag */
	IEN(SCI2,TXI2)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN4	= 1U;
	IEN(SCI2,RXI2)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN5	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI2_Stop
 * Description  : This function stops SCI2.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI2_Stop(void)
{
//	PORT5.PMR.BIT.B0	= 0U;	/* reset TXD2 pin(P50) */
	SCI2.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI2.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI2.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI2.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI2,TXI2)		= 0U;
	ICU.GENBL0.BIT.EN4	= 0U;
	IR(SCI2,TXI2)		= 0U;
	IEN(SCI2,RXI2)		= 0U;
	ICU.GENBL0.BIT.EN5	= 0U;
	IR(SCI2,RXI2)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI2_Serial_Receive
 * Description  : This function receives SCI2 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI2_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci2_rx_count		= 0U;
	g_sci2_rx_length	= rx_num;
	gp_sci2_rx_address	= rx_buf;
	SCI2.SCR.BIT.RIE	= 1U;
	SCI2.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI2_Serial_Send
 * Description  : This function transmits SCI2 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI2_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci2_tx_address	= tx_buf;
	g_sci2_tx_count		= tx_num;
	/* Set TXD2 pin */
	SCI2.SCR.BIT.TIE	= 1U;
	SCI2.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI3
/***********************************************************************************************************************
 * Function Name: R_SCI3_Create
 * Description  : This function initializes SCI3.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI3_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI3)			= 0U;			/* Cancel SCI3 module stop state */
	IPR(SCI3,RXI3)		= 15;			/* Set interrupt priority */
	IPR(SCI3,TXI3)		= 15;			/* Set interrupt priority */
	SCI3.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI3.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI3.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI3.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI3.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI3.SPMR.BYTE		= _00_SCI_RTS;
	SCI3.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI3.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI3.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI3.BRR			= 0x40U;
	/* Set SCI3 pin */
	SCI3_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI3_Start
 * Description  : This function starts SCI3.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI3_Start(void)
{
	IR(SCI3,TXI3)		= 0U;	/* Clear interrupt flag */
	IR(SCI3,RXI3)		= 0U;	/* Clear interrupt flag */
	IEN(SCI3,TXI3)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN6	= 1U;
	IEN(SCI3,RXI3)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN7	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI3_Stop
 * Description  : This function stops SCI3.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI3_Stop(void)
{
	SCI3.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI3.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI3.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI3.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI3,TXI3)		= 0U;
	ICU.GENBL0.BIT.EN6	= 0U;
	IR(SCI3,TXI3)		= 0U;
	IEN(SCI3,RXI3)		= 0U;
	ICU.GENBL0.BIT.EN7	= 0U;
	IR(SCI3,RXI3)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI3_Serial_Receive
 * Description  : This function receives SCI3 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI3_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci3_rx_count		= 0U;
	g_sci3_rx_length	= rx_num;
	gp_sci3_rx_address	= rx_buf;
	SCI3.SCR.BIT.RIE	= 1U;
	SCI3.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI3_Serial_Send
 * Description  : This function transmits SCI3 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI3_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci3_tx_address	= tx_buf;
	g_sci3_tx_count		= tx_num;
	/* Set TXD3 pin */
	SCI3.SCR.BIT.TIE	= 1U;
	SCI3.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI4
/***********************************************************************************************************************
 * Function Name: R_SCI4_Create
 * Description  : This function initializes SCI4.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI4_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI4)			= 0U;			/* Cancel SCI4 module stop state */
	IPR(SCI4,RXI4)		= 15;			/* Set interrupt priority */
	IPR(SCI4,TXI4)		= 15;			/* Set interrupt priority */
	SCI4.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI4.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI4.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI4.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI4.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI4.SPMR.BYTE		= _00_SCI_RTS;
	SCI4.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI4.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI4.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI4.BRR			= 0x40U;
	/* Set SCI4 pin */
	SCI4_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI4_Start
 * Description  : This function starts SCI4.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI4_Start(void)
{
	RX_MPC_enable();
	IR(SCI4,TXI4)		= 0U;	/* Clear interrupt flag */
	IR(SCI4,RXI4)		= 0U;	/* Clear interrupt flag */
	IEN(SCI4,TXI4)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN8	= 1U;
	IEN(SCI4,RXI4)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN9	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI4_Stop
 * Description  : This function stops SCI4.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI4_Stop(void)
{
	SCI4.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI4.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI4.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI4.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI4,TXI4)		= 0U;
	ICU.GENBL0.BIT.EN8	= 0U;
	IR(SCI4,TXI4)		= 0U;
	IEN(SCI4,RXI4)		= 0U;
	ICU.GENBL0.BIT.EN9	= 0U;
	IR(SCI4,RXI4)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI4_Serial_Receive
 * Description  : This function receives SCI4 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI4_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci4_rx_count		= 0U;
	g_sci4_rx_length	= rx_num;
	gp_sci4_rx_address	= rx_buf;
	SCI4.SCR.BIT.RIE	= 1U;
	SCI4.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI4_Serial_Send
 * Description  : This function transmits SCI4 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI4_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci4_tx_address	= tx_buf;
	g_sci4_tx_count		= tx_num;
	/* Set TXD4 pin */
	SCI4.SCR.BIT.TIE	= 1U;
	SCI4.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI5
/***********************************************************************************************************************
 * Function Name: R_SCI5_Create
 * Description  : This function initializes SCI5.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI5_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI5)			= 0U;			/* Cancel SCI0 module stop state */
	IPR(SCI5,RXI5)		= 15;			/* Set interrupt priority */
	IPR(SCI5,TXI5)		= 15;			/* Set interrupt priority */
	SCI5.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI5.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI5.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI5.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI5.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI5.SPMR.BYTE		= _00_SCI_RTS;
	SCI5.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI5.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI5.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI5.BRR			= 0x40U;
	/* Set SCI5 pin */
	SCI5_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI5_Start
 * Description  : This function starts SCI5.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI5_Start(void)
{
	IR(SCI5,TXI5)		= 0U;	/* Clear interrupt flag */
	IR(SCI5,RXI5)		= 0U;	/* Clear interrupt flag */
	IEN(SCI5,TXI5)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN10	= 1U;
	IEN(SCI5,RXI5)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN11	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI5_Stop
 * Description  : This function stops SCI5.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI5_Stop(void)
{
	SCI5.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI5.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI5.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI5.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI5,TXI5)		= 0U;
	ICU.GENBL0.BIT.EN10	= 0U;
	IR(SCI5,TXI5)		= 0U;
	IEN(SCI5,RXI5)		= 0U;
	ICU.GENBL0.BIT.EN11	= 0U;
	IR(SCI5,RXI5)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI5_Serial_Receive
 * Description  : This function receives SCI5 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI5_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci5_rx_count		= 0U;
	g_sci5_rx_length	= rx_num;
	gp_sci5_rx_address	= rx_buf;
	SCI5.SCR.BIT.RIE	= 1U;
	SCI5.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI5_Serial_Send
 * Description  : This function transmits SCI5 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI5_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci5_tx_address	= tx_buf;
	g_sci5_tx_count		= tx_num;
	/* Set TXD5 pin */
	SCI5.SCR.BIT.TIE	= 1U;
	SCI5.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI6
/***********************************************************************************************************************
 * Function Name: R_SCI6_Create
 * Description  : This function initializes SCI6.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI6_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI6)			= 0U;			/* Cancel SCI0 module stop state */
	IPR(SCI6,RXI6)		= 15;			/* Set interrupt priority */
	IPR(SCI6,TXI6)		= 15;			/* Set interrupt priority */
	SCI6.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI6.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI6.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI6.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI6.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI6.SPMR.BYTE		= _00_SCI_RTS;
	SCI6.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI6.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI6.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI6.BRR			= 0x40U;
	/* Set SCI6 pin */
	SCI6_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI6_Start
 * Description  : This function starts SCI6.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI6_Start(void)
{
	IR(SCI6,TXI6)		= 0U;	/* Clear interrupt flag */
	IR(SCI6,RXI6)		= 0U;	/* Clear interrupt flag */
	IEN(SCI6,TXI6)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN12	= 1U;
	IEN(SCI6,RXI6)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN13	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI6_Stop
 * Description  : This function stops SCI6.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI6_Stop(void)
{
	SCI6.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI6.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI6.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI6.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI6,TXI6)		= 0U;
	ICU.GENBL0.BIT.EN12	= 0U;
	IR(SCI6,TXI6)		= 0U;
	IEN(SCI6,RXI6)		= 0U;
	ICU.GENBL0.BIT.EN13	= 0U;
	IR(SCI6,RXI6)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI6_Serial_Receive
 * Description  : This function receives SCI6 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI6_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci6_rx_count		= 0U;
	g_sci6_rx_length	= rx_num;
	gp_sci6_rx_address	= rx_buf;
	SCI6.SCR.BIT.RIE	= 1U;
	SCI6.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI6_Serial_Send
 * Description  : This function transmits SCI6 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI6_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci6_tx_address	= tx_buf;
	g_sci6_tx_count		= tx_num;
	/* Set TXD0 pin */
	SCI6.SCR.BIT.TIE	= 1U;
	SCI6.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX65N_SCI8
/***********************************************************************************************************************
 * Function Name: R_SCI8_Create
 * Description  : This function initializes SCI8.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI8_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI8)			= 0U;			/* Cancel SCI8 module stop state */
	IPR(SCI8,RXI8)		= 15;			/* Set interrupt priority */
	IPR(SCI8,TXI8)		= 15;			/* Set interrupt priority */
	SCI8.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI8.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI8.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI8.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI8.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI8.SPMR.BYTE		= _00_SCI_RTS;
	SCI8.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI8.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI8.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _00_SCI_NOISE_FILTER_DISABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _00_SCI_BIT_MODULATION_DISABLE;
	/* Set bit rate */
	SCI8.BRR			= 0x40U;
	/* Set SCI8 pin */
	SCI8_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI8_Start
 * Description  : This function starts SCI8.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI8_Start(void)
{
	IR(SCI8,TXI8)		= 0U;	/* Clear interrupt flag */
	IR(SCI8,RXI8)		= 0U;	/* Clear interrupt flag */
	IEN(SCI8,TXI8)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN24	= 1U;
	IEN(SCI8,RXI8)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN25	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI8_Stop
 * Description  : This function stops SCI8.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI8_Stop(void)
{
	SCI8.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI8.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI8.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI8.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI8,TXI8)		= 0U;
	ICU.GENBL1.BIT.EN24	= 0U;
	IR(SCI8,TXI8)		= 0U;
	IEN(SCI8,RXI8)		= 0U;
	ICU.GENBL1.BIT.EN25	= 0U;
	IR(SCI8,RXI8)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI8_Serial_Receive
 * Description  : This function receives SCI8 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI8_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci8_rx_count		= 0U;
	g_sci8_rx_length	= rx_num;
	gp_sci8_rx_address	= rx_buf;
	SCI8.SCR.BIT.RIE	= 1U;
	SCI8.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI8_Serial_Send
 * Description  : This function transmits SCI8 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI8_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci8_tx_address	= tx_buf;
	g_sci8_tx_count		= tx_num;
	/* Set TXD0 pin */
	SCI8.SCR.BIT.TIE	= 1U;
	SCI8.SCR.BIT.TE		= 1U;
	return OK;
}
#endif


/***********************************************************************************************************************
* Function Name: R_SCI7_Create
* Description  : This function initializes SCI7.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SCI7_Create(void)
{
	RX_MPC_enable();

	/* Cancel SCI7 module stop state */
	MSTP(SCI7) = 0U;

	/* Set interrupt priority */
	IPR(SCI7,TXI7) = _0F_SCI_PRIORITY_LEVEL15;

	/* Clear the control register */
	SCI7.SCR.BYTE = 0x00U;

	/* Set clock enable */
	SCI7.SCR.BYTE |= _01_SCI_INTERNAL_SCK_OUTPUT;

	/* Clear the SIMR1.IICM */
	SCI7.SIMR1.BIT.IICM = 0U;

	/* Set control registers */
	SCI7.SPMR.BYTE = _00_SCI_SS_PIN_DISABLE | _00_SCI_SPI_MASTER | _00_SCI_CLOCK_NOT_INVERTED |
					 _00_SCI_CLOCK_NOT_DELAYED;
	SCI7.SMR.BYTE = _80_SCI_CLOCK_SYNCHRONOUS_OR_SPI_MODE | _00_SCI_CLOCK_PCLK;
	SCI7.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_INVERT_NONE | _08_SCI_DATA_MSB_FIRST | _62_SCI_SCMR_DEFAULT;
	SCI7.SEMR.BYTE = _00_SCI_BIT_MODULATION_DISABLE;

	/* Set bit rate */
	SCI7.BRR = 0x00U;

	/* Set SMOSI7 pin */
	MPC.P90PFS.BYTE = 0x0AU;
	PORT9.PMR.BYTE |= 0x01U;

	/* Set SCK7 pin */
	MPC.P91PFS.BYTE = 0x0AU;
	PORT9.PMR.BYTE |= 0x02U;

	RX_MPC_disable();
}
/***********************************************************************************************************************
* Function Name: R_SCI7_Start
* Description  : This function starts SCI7.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SCI7_Start(void)
{
	/* Enable TXI and TEI interrupt */
	IR(SCI7,TXI7) = 0U;
	IEN(SCI7,TXI7) = 1U;
	ICU.GENBL0.BIT.EN14 = 1U;
}
/***********************************************************************************************************************
* Function Name: R_SCI7_Stop
* Description  : This function stops SCI7.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_SCI7_Stop(void)
{
	/* Set SMOSI7 pin */
	PORT9.PMR.BYTE &= 0xFEU;

	/* Disable serial transmit and receive */
	SCI7.SCR.BYTE &= 0xCFU;

	/* Disable TXI and TEI interrupt */
	IEN(SCI7,TXI7) = 0U;
	ICU.GENBL0.BIT.EN14 = 0U;

	/* Clear interrupt flags */
	IR(SCI7,TXI7) = 0U;
	IR(SCI7,RXI7) = 0U;
}
/***********************************************************************************************************************
* Function Name: R_SCI7_SPI_Master_Send
* Description  : This function sends SPI7 data to slave device.
* Arguments    : tx_buf -
*                    transfer buffer pointer (Not used when transmit data handled by DTC or DMAC)
*                tx_num -
*                    buffer size (Not used when receive data handled by DTC or DMAC)
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_SCI7_SPI_Master_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	MD_STATUS status = MD_OK;

	if (1U > tx_num)
	{
		status = MD_ARGERROR;
	}
	else
	{
		gp_sci7_tx_address = tx_buf;
		g_sci7_tx_count = tx_num;

		/* Set SMOSI7 pin */
		PORT9.PMR.BYTE |= 0x01U;

		SCI7.SCR.BIT.TIE = 1U;
		SCI7.SCR.BIT.TE = 1U;
	}

	return (status);
}
