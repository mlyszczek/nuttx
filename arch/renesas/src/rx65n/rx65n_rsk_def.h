/* Multiple inclusion prevention macro */
#ifndef RSKRX65N_H
#define RSKRX65N_H

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/* General Values */
#define LED_ON          (0)
#define LED_OFF         (1)
#define SET_BIT_HIGH    (1)
#define SET_BIT_LOW     (0)
#define SET_BYTE_HIGH   (0xFF)
#define SET_BYTE_LOW    (0x00)

/* Switches */
#define SW1             (PORT0.PIDR.BIT.B0)
#define SW2             (PORT0.PIDR.BIT.B1)
#define SW3             (PORT0.PIDR.BIT.B7)

/* LED port settings */
//#define LED0            (PORT0.PODR.BIT.B3)
//#define LED1            (PORT0.PODR.BIT.B5)
//#define LED2            (PORT7.PODR.BIT.B3)
//#define LED3            (PORTJ.PODR.BIT.B5)

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global variables
***********************************************************************************************************************/

/***********************************************************************************************************************
Exported global functions (to be accessed by other files)
***********************************************************************************************************************/

#endif

