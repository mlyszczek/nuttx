/************************************************************************************
 * configs/metro-m4/include/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIG_METRO_M4_INCLUDE_BOARD_H
#define __CONFIG_METRO_M4_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* LED definitions ******************************************************************/

/* Alternate function pin selections ************************************************/

/* SERCOM definitions ***************************************************************/
/* The SERCOM bus clock (CLK_SERCOMx_APB) can be enabled and disabled in the Main
 * Clock Controller.  The SERCOM uses two generic clocks: GCLK_SERCOMx_CORE and
 * GCLK_SERCOMx_SLOW. The core clock (GCLK_SERCOMx_CORE) is required to clock the
 * SERCOM while working as a master.  The slow clock (GCLK_SERCOMx_SLOW) is only
 * required for certain functions.
 *
 * These clocks must be configured and enabled in the Generic Clock Controller (GCLK)
 * before using the SERCOM.
 */

/* SERCOM3
 *
 * An Arduino compatible serial Shield is assumed (or equivalently, an external
 * RS-232 or serial-to-USB adapter connected on Arduino pins D0 and D1):
 *
 *   ------ ----------------- ---------
 *   SHIELD SAMD5E5           FUNCTION
 *   ------ ----------------- ---------
 *   D0     PA23 SERCOM3 PAD2 RXD
 *   D1     PA22 SERCOM3 PAD0 TXD
 *
 * NOTES:
 *   USART_CTRLA_TXPAD0_1: TxD=PAD0 XCK=PAD1 RTS/TE=N/A CTS=N/A
 *   USART_CTRLA_RXPAD2:   RxD=PAD2
 */

#define BOARD_SERCOM3_MUXCONFIG      (USART_CTRLA_TXPAD0_1 | USART_CTRLA_RXPAD2)
#define BOARD_SERCOM3_PINMAP_PAD0    PORT_SERCOM3_PAD0_1 /* USART TX */
#define BOARD_SERCOM3_PINMAP_PAD1    PORT_SERCOM3_PAD2_1 /* USART RX */
#define BOARD_SERCOM3_PINMAP_PAD2    0                   /* (not used) */
#define BOARD_SERCOM3_PINMAP_PAD3    0                   /* (not used) */

#define BOARD_SERCOM3_FREQUENCY      BOARD_GCLK0_FREQUENCY

#endif  /* __CONFIG_METRO_M4_INCLUDE_BOARD_H */
