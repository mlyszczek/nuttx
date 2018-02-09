/****************************************************************************************************
 * arch/arm/src/stm32h7/chip/stm32h7x3xx_rcc.h
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_RCC_H
#define __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_RCC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

// TODO: Complete comments

#define STM32_RCC_CR_OFFSET          0x0000  /* Clock control register */
#define STM32_RCC_ICSCR_OFFSET       0x0004  /*  */
#define STM32_RCC_CRRCR_OFFSET       0x0008  /*  */
#define STM32_RCC_CFGR_OFFSET        0x0010  /* Clock configuration register */
#define STM32_RCC_D1CFGR_OFFSET      0x0018  /*  */
#define STM32_RCC_D2CFGR_OFFSET      0x001c  /*  */
#define STM32_RCC_D3CFGR_OFFSET      0x0020  /*  */
#define STM32_RCC_PLLCKSELR_OFFSET   0x0028  /*  */
#define STM32_RCC_PLLCFGR_OFFSET     0x002c  /*  */
#define STM32_RCC_PLL1DIVR_OFFSET    0x0030  /*  */
#define STM32_RCC_PLL1FRACR_OFFSET   0x0034  /*  */
#define STM32_RCC_PLL2DIVR_OFFSET    0x0038  /*  */
#define STM32_RCC_PLL2FRACR_OFFSET   0x003c  /*  */
#define STM32_RCC_PLL3DIVR_OFFSET    0x0040  /*  */
#define STM32_RCC_PLL3FRACR_OFFSET   0x0044  /*  */
#define STM32_RCC_D1CCIPR_OFFSET     0x004c  /*  */
#define STM32_RCC_D2CCIP1R_OFFSET    0x0050  /*  */
#define STM32_RCC_D2CCIP2R_OFFSET    0x0054  /*  */
#define STM32_RCC_D3CCIPR_OFFSET     0x0058  /*  */
#define STM32_RCC_CIER_OFFSET        0x0060  /*  */
#define STM32_RCC_CIFR_OFFSET        0x0064  /*  */
#define STM32_RCC_CICR_OFFSET        0x0068  /*  */
#define STM32_RCC_BDCR_OFFSET        0x0070  /*  */
#define STM32_RCC_CSR_OFFSET         0x0074  /*  */
#define STM32_RCC_AHB1RSTR_OFFSET    0x0080  /* AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR_OFFSET    0x0084  /* AHB2 peripheral reset register */
#define STM32_RCC_AHB3RSTR_OFFSET    0x007c  /* AHB3 peripheral reset register */
#define STM32_RCC_AHB4RSTR_OFFSET    0x0088  /* AHB4 peripheral reset register */
#define STM32_RCC_APB1LRSTR_OFFSET   0x0090  /* APB1 L Peripheral reset register */
#define STM32_RCC_APB1HRSTR_OFFSET   0x0094  /* APB1 H Peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET    0x0098  /* APB2 Peripheral reset register */
#define STM32_RCC_APB3RSTR_OFFSET    0x008c  /* APB3 Peripheral reset register */
#define STM32_RCC_APB4RSTR_OFFSET    0x009c  /* APB4 Peripheral reset register */
#define STM32_RCC_GCR_OFFSET         0x00a0  /*  */
#define STM32_RCC_D3AMR_OFFSET       0x00a8  /*  */
#define STM32_RCC_RSR_OFFSET         0x00d0  /*  */
#define STM32_RCC_AHB1ENR_OFFSET     0x00d8  /* AHB1 Peripheral Clock enable register */
#define STM32_RCC_AHB2ENR_OFFSET     0x00dc  /* AHB2 Peripheral Clock enable register */
#define STM32_RCC_AHB3ENR_OFFSET     0x00d4  /* AHB3 Peripheral Clock enable register */
#define STM32_RCC_AHB4ENR_OFFSET     0x00e0  /* AHB4 Peripheral Clock enable register */
#define STM32_RCC_APB1LENR_OFFSET    0x00e8  /* APB1 L Peripheral Clock enable register */
#define STM32_RCC_APB1HENR_OFFSET    0x00ec  /* APB1 H Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET     0x00f0  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB3ENR_OFFSET     0x00e4  /* APB3 Peripheral Clock enable register */
#define STM32_RCC_APB4ENR_OFFSET     0x00f4  /* APB4 Peripheral Clock enable register */
#define STM32_RCC_AHB1LPENR_OFFSET   0x0100  /* RCC AHB1 low power mode peripheral clock enable register */
#define STM32_RCC_AHB2LPENR_OFFSET   0x0104  /* RCC AHB2 low power mode peripheral clock enable register */
#define STM32_RCC_AHB3LPENR_OFFSET   0x00fc  /* RCC AHB3 low power mode peripheral clock enable register */
#define STM32_RCC_AHB4LPENR_OFFSET   0x0108  /* RCC AHB4 low power mode peripheral clock enable register */
#define STM32_RCC_APB1LLPENR_OFFSET  0x0110  /* RCC APB1 L low power mode peripheral clock enable register */
#define STM32_RCC_APB1HLPENR_OFFSET  0x0114  /* RCC APB1 H low power mode peripheral clock enable register */
#define STM32_RCC_APB2LPENR_OFFSET   0x0118  /* RCC APB2 low power mode peripheral clock enable register */
#define STM32_RCC_APB3LPENR_OFFSET   0x010c  /* RCC APB3 low power mode peripheral clock enable register */
#define STM32_RCC_APB4LPENR_OFFSET   0x011c  /* RCC APB4 low power mode peripheral clock enable register */

/* Register Addresses *******************************************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_ICSCR             (STM32_RCC_BASE+STM32_RCC_ICSCR_OFFSET)
#define STM32_RCC_CRRCR             (STM32_RCC_BASE+STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_D1CFGR            (STM32_RCC_BASE+STM32_RCC_D1CFGR_OFFSET)
#define STM32_RCC_D2CFGR            (STM32_RCC_BASE+STM32_RCC_D2CFGR_OFFSET)
#define STM32_RCC_D3CFGR            (STM32_RCC_BASE+STM32_RCC_D3CFGR_OFFSET)
#define STM32_RCC_PLLCKSELR         (STM32_RCC_BASE+STM32_RCC_PLLCKSELR_OFFSET)
#define STM32_RCC_PLLCFGR           (STM32_RCC_BASE+STM32_RCC_PLLCFGR_OFFSET)
#define STM32_RCC_PLL1DIVR          (STM32_RCC_BASE+STM32_RCC_PLL1DIVR_OFFSET)
#define STM32_RCC_PLL1FRACR         (STM32_RCC_BASE+STM32_RCC_PLL1FRACR_OFFSET)
#define STM32_RCC_PLL2DIVR          (STM32_RCC_BASE+STM32_RCC_PLL2DIVR_OFFSET)
#define STM32_RCC_PLL2FRACR         (STM32_RCC_BASE+STM32_RCC_PLL2FRACR_OFFSET)
#define STM32_RCC_PLL3DIVR          (STM32_RCC_BASE+STM32_RCC_PLL3DIVR_OFFSET)
#define STM32_RCC_PLL3FRACR         (STM32_RCC_BASE+STM32_RCC_PLL3FRACR_OFFSET)
#define STM32_RCC_D1CCIPR           (STM32_RCC_BASE+STM32_RCC_D1CCIPR_OFFSET)
#define STM32_RCC_D2CCIP1R          (STM32_RCC_BASE+STM32_RCC_D2CCIP1R_OFFSET)
#define STM32_RCC_D2CCIP2R          (STM32_RCC_BASE+STM32_RCC_D2CCIP2R_OFFSET)
#define STM32_RCC_D3CCIPR           (STM32_RCC_BASE+STM32_RCC_D3CCIPR_OFFSET)
#define STM32_RCC_CIER              (STM32_RCC_BASE+STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR              (STM32_RCC_BASE+STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR              (STM32_RCC_BASE+STM32_RCC_CICR_OFFSET)
#define STM32_RCC_BDCR              (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)
#define STM32_RCC_AHB1RSTR          (STM32_RCC_BASE+STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR          (STM32_RCC_BASE+STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB3RSTR          (STM32_RCC_BASE+STM32_RCC_AHB3RSTR_OFFSET)
#define STM32_RCC_AHB4RSTR          (STM32_RCC_BASE+STM32_RCC_AHB4RSTR_OFFSET)
#define STM32_RCC_APB1LRSTR         (STM32_RCC_BASE+STM32_RCC_APB1LRSTR_OFFSET)
#define STM32_RCC_APB1HRSTR         (STM32_RCC_BASE+STM32_RCC_APB1HRSTR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB3RSTR          (STM32_RCC_BASE+STM32_RCC_APB3RSTR_OFFSET)
#define STM32_RCC_APB4RSTR          (STM32_RCC_BASE+STM32_RCC_APB4RSTR_OFFSET)
#define STM32_RCC_GCR               (STM32_RCC_BASE+STM32_RCC_GCR_OFFSET)
#define STM32_RCC_D3AMR             (STM32_RCC_BASE+STM32_RCC_D3AMR_OFFSET)
#define STM32_RCC_RSR               (STM32_RCC_BASE+STM32_RCC_RSR_OFFSET)
#define STM32_RCC_AHB1ENR           (STM32_RCC_BASE+STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR           (STM32_RCC_BASE+STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB3ENR           (STM32_RCC_BASE+STM32_RCC_AHB3ENR_OFFSET)
#define STM32_RCC_AHB4ENR           (STM32_RCC_BASE+STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB1LENR          (STM32_RCC_BASE+STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB1HENR          (STM32_RCC_BASE+STM32_RCC_APB1HENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR           (STM32_RCC_BASE+STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_APB4ENR           (STM32_RCC_BASE+STM32_RCC_APB4ENR_OFFSET)
#define STM32_RCC_AHB1LPENR         (STM32_RCC_BASE+STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR         (STM32_RCC_BASE+STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB3LPENR         (STM32_RCC_BASE+STM32_RCC_AHB3LPENR_OFFSET)
#define STM32_RCC_AHB4LPENR         (STM32_RCC_BASE+STM32_RCC_AHB4LPENR_OFFSET)
#define STM32_RCC_APB1LLPENR        (STM32_RCC_BASE+STM32_RCC_APB1LLPENR_OFFSET)
#define STM32_RCC_APB1HLPENR        (STM32_RCC_BASE+STM32_RCC_APB1HLPENR_OFFSET)
#define STM32_RCC_APB2LPENR         (STM32_RCC_BASE+STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB3LPENR         (STM32_RCC_BASE+STM32_RCC_APB3LPENR_OFFSET)
#define STM32_RCC_APB4LPENR         (STM32_RCC_BASE+STM32_RCC_APB4LPENR_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* Source Control Register */

#define RCC_CR_HSION_Pos                       (0U)
#define RCC_CR_HSION_Msk                       (0x1U << RCC_CR_HSION_Pos)      /*!< 0x00000001 */
#define RCC_CR_HSION                           RCC_CR_HSION_Msk                /*!< Internal High Speed clock enable */
#define RCC_CR_HSIKERON_Pos                    (1U)
#define RCC_CR_HSIKERON_Msk                    (0x1U << RCC_CR_HSIKERON_Pos)   /*!< 0x00000002 */
#define RCC_CR_HSIKERON                        RCC_CR_HSIKERON_Msk             /*!< Internal High Speed clock enable for some IPs Kernel */
#define RCC_CR_HSIRDY_Pos                      (2U)
#define RCC_CR_HSIRDY_Msk                      (0x1U << RCC_CR_HSIRDY_Pos)     /*!< 0x00000004 */
#define RCC_CR_HSIRDY                          RCC_CR_HSIRDY_Msk               /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSIDIV_Pos                      (3U)
#define RCC_CR_HSIDIV_Msk                      (0x3U << RCC_CR_HSIDIV_Pos)     /*!< 0x00000018 */
#define RCC_CR_HSIDIV                          RCC_CR_HSIDIV_Msk               /*!< Internal High Speed clock divider selection */
#define RCC_CR_HSIDIV_1                        (0x0U << RCC_CR_HSIDIV_Pos)     /*!< 0x00000000 */
#define RCC_CR_HSIDIV_2                        (0x1U << RCC_CR_HSIDIV_Pos)     /*!< 0x00000008 */
#define RCC_CR_HSIDIV_4                        (0x2U << RCC_CR_HSIDIV_Pos)     /*!< 0x00000010 */
#define RCC_CR_HSIDIV_8                        (0x3U << RCC_CR_HSIDIV_Pos)     /*!< 0x00000018 */

#define RCC_CR_HSIDIVF_Pos                     (5U)
#define RCC_CR_HSIDIVF_Msk                     (0x1U << RCC_CR_HSIDIVF_Pos)    /*!< 0x00000020 */
#define RCC_CR_HSIDIVF                         RCC_CR_HSIDIVF_Msk              /*!< HSI Divider flag */
#define RCC_CR_CSION_Pos                       (7U)
#define RCC_CR_CSION_Msk                       (0x1U << RCC_CR_CSION_Pos)      /*!< 0x00000080 */
#define RCC_CR_CSION                           RCC_CR_CSION_Msk                /*!< The Internal RC 4MHz oscillator clock enable */
#define RCC_CR_CSIRDY_Pos                      (8U)
#define RCC_CR_CSIRDY_Msk                      (0x1U << RCC_CR_CSIRDY_Pos)     /*!< 0x00000100 */
#define RCC_CR_CSIRDY                          RCC_CR_CSIRDY_Msk               /*!< The Internal RC 4MHz oscillator clock ready */
#define RCC_CR_CSIKERON_Pos                    (9U)
#define RCC_CR_CSIKERON_Msk                    (0x1U << RCC_CR_CSIKERON_Pos)   /*!< 0x00000200 */
#define RCC_CR_CSIKERON                        RCC_CR_CSIKERON_Msk             /*!< Internal RC 4MHz oscillator clock enable for some IPs Kernel */
#define RCC_CR_HSI48ON_Pos                     (12U)
#define RCC_CR_HSI48ON_Msk                     (0x1U << RCC_CR_HSI48ON_Pos)    /*!< 0x00001000 */
#define RCC_CR_HSI48ON                         RCC_CR_HSI48ON_Msk              /*!< HSI48 clock enable clock enable  */
#define RCC_CR_HSI48RDY_Pos                    (13U)
#define RCC_CR_HSI48RDY_Msk                    (0x1U << RCC_CR_HSI48RDY_Pos)   /*!< 0x00002000 */
#define RCC_CR_HSI48RDY                        RCC_CR_HSI48RDY_Msk             /*!< HSI48 clock ready */

#define RCC_CR_D1CKRDY_Pos                     (14U)
#define RCC_CR_D1CKRDY_Msk                     (0x1U << RCC_CR_D1CKRDY_Pos)    /*!< 0x00004000 */
#define RCC_CR_D1CKRDY                         RCC_CR_D1CKRDY_Msk              /*!< D1 domain clocks ready flag  */
#define RCC_CR_D2CKRDY_Pos                     (15U)
#define RCC_CR_D2CKRDY_Msk                     (0x1U << RCC_CR_D2CKRDY_Pos)    /*!< 0x00008000 */
#define RCC_CR_D2CKRDY                         RCC_CR_D2CKRDY_Msk              /*!< D2 domain clocks ready flag */

#define RCC_CR_HSEON_Pos                       (16U)
#define RCC_CR_HSEON_Msk                       (0x1U << RCC_CR_HSEON_Pos)      /*!< 0x00010000 */
#define RCC_CR_HSEON                           RCC_CR_HSEON_Msk                /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos                      (17U)
#define RCC_CR_HSERDY_Msk                      (0x1U << RCC_CR_HSERDY_Pos)     /*!< 0x00020000 */
#define RCC_CR_HSERDY                          RCC_CR_HSERDY_Msk               /*!< External High Speed clock ready */
#define RCC_CR_HSEBYP_Pos                      (18U)
#define RCC_CR_HSEBYP_Msk                      (0x1U << RCC_CR_HSEBYP_Pos)     /*!< 0x00040000 */
#define RCC_CR_HSEBYP                          RCC_CR_HSEBYP_Msk               /*!< External High Speed clock Bypass */
#define RCC_CR_CSSHSEON_Pos                    (19U)
#define RCC_CR_CSSHSEON_Msk                    (0x1U << RCC_CR_CSSHSEON_Pos)   /*!< 0x00080000 */
#define RCC_CR_CSSHSEON                        RCC_CR_CSSHSEON_Msk             /*!< HSE Clock security System enable */

#define RCC_CR_PLL1ON_Pos                      (24U)
#define RCC_CR_PLL1ON_Msk                      (0x1U << RCC_CR_PLL1ON_Pos)     /*!< 0x01000000 */
#define RCC_CR_PLL1ON                          RCC_CR_PLL1ON_Msk               /*!< System PLL1 clock enable */
#define RCC_CR_PLL1RDY_Pos                     (25U)
#define RCC_CR_PLL1RDY_Msk                     (0x1U << RCC_CR_PLL1RDY_Pos)    /*!< 0x02000000 */
#define RCC_CR_PLL1RDY                         RCC_CR_PLL1RDY_Msk              /*!< System PLL1 clock ready */
#define RCC_CR_PLL2ON_Pos                      (26U)
#define RCC_CR_PLL2ON_Msk                      (0x1U << RCC_CR_PLL2ON_Pos)     /*!< 0x04000000 */
#define RCC_CR_PLL2ON                          RCC_CR_PLL2ON_Msk               /*!< System PLL2 clock enable */
#define RCC_CR_PLL2RDY_Pos                     (27U)
#define RCC_CR_PLL2RDY_Msk                     (0x1U << RCC_CR_PLL2RDY_Pos)    /*!< 0x08000000 */
#define RCC_CR_PLL2RDY                         RCC_CR_PLL2RDY_Msk              /*!< System PLL2 clock ready */
#define RCC_CR_PLL3ON_Pos                      (28U)
#define RCC_CR_PLL3ON_Msk                      (0x1U << RCC_CR_PLL3ON_Pos)     /*!< 0x10000000 */
#define RCC_CR_PLL3ON                          RCC_CR_PLL3ON_Msk               /*!< System PLL3 clock enable */
#define RCC_CR_PLL3RDY_Pos                     (29U)
#define RCC_CR_PLL3RDY_Msk                     (0x1U << RCC_CR_PLL3RDY_Pos)    /*!< 0x20000000 */
#define RCC_CR_PLL3RDY                         RCC_CR_PLL3RDY_Msk              /*!< System PLL3 clock ready */

/* Internal Clock Source Calibration Register */

/*!< HSICAL configuration */
#define RCC_ICSCR_HSICAL_Pos                   (0U)
#define RCC_ICSCR_HSICAL_Msk                   (0xFFFU << RCC_ICSCR_HSICAL_Pos) /*!< 0x00000FFF */
#define RCC_ICSCR_HSICAL                       RCC_ICSCR_HSICAL_Msk            /*!< HSICAL[11:0] bits */

/*!< HSITRIM configuration */
#define RCC_ICSCR_HSITRIM_Pos                  (12U)
#define RCC_ICSCR_HSITRIM_Msk                  (0x3FU << RCC_ICSCR_HSITRIM_Pos) /*!< 0x0003F000 */
#define RCC_ICSCR_HSITRIM                      RCC_ICSCR_HSITRIM_Msk           /*!< HSITRIM[5:0] bits */

/*!< CSICAL configuration */
#define RCC_ICSCR_CSICAL_Pos                   (18U)
#define RCC_ICSCR_CSICAL_Msk                   (0xFFU << RCC_ICSCR_CSICAL_Pos) /*!< 0x03FC0000 */
#define RCC_ICSCR_CSICAL                       RCC_ICSCR_CSICAL_Msk            /*!< CSICAL[7:0] bits */

/*!< CSITRIM configuration */
#define RCC_ICSCR_CSITRIM_Pos                  (26U)
#define RCC_ICSCR_CSITRIM_Msk                  (0x1FU << RCC_ICSCR_CSITRIM_Pos) /*!< 0x7C000000 */
#define RCC_ICSCR_CSITRIM                      RCC_ICSCR_CSITRIM_Msk           /*!< CSITRIM[4:0] bits */

/* Clock Recovery RC Register */

/*!< HSI48CAL configuration */
#define RCC_CRRCR_HSI48CAL_Pos                 (0U)
#define RCC_CRRCR_HSI48CAL_Msk                 (0x3FFU << RCC_CRRCR_HSI48CAL_Pos) /*!< 0x000003FF */
#define RCC_CRRCR_HSI48CAL                     RCC_CRRCR_HSI48CAL_Msk          /*!< HSI48CAL[9:0] bits */

/* Clock Configuration Register */

/*!< SW configuration */
#define RCC_CFGR_SW                            ((uint32_t)0x00000007)          /*!< SW[2:0] bits (System clock Switch) */
#define RCC_CFGR_SW_HSI                        ((uint32_t)0x00000000)          /*!< HSI selection as system clock */
#define RCC_CFGR_SW_CSI                        ((uint32_t)0x00000001)          /*!< CSI selection as system clock */
#define RCC_CFGR_SW_HSE                        ((uint32_t)0x00000002)          /*!< HSE selection as system clock */
#define RCC_CFGR_SW_PLL1                       ((uint32_t)0x00000003)          /*!< PLL1 selection as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS                           ((uint32_t)0x00000038)          /*!< SWS[2:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_HSI                       ((uint32_t)0x00000000)          /*!< HSI used as system clock */
#define RCC_CFGR_SWS_CSI                       ((uint32_t)0x00000008)          /*!< CSI used as system clock */
#define RCC_CFGR_SWS_HSE                       ((uint32_t)0x00000010)          /*!< HSE used as system clock */
#define RCC_CFGR_SWS_PLL1                      ((uint32_t)0x00000018)          /*!< PLL1 used as system clock */

#define RCC_CFGR_STOPWUCK                      ((uint32_t)0x00000040)          /*!< Wake Up from stop and CSS backup clock selection */
#define RCC_CFGR_STOPKERWUCK                   ((uint32_t)0x00000080)          /*!< Kernel Clock Selection after a Wake Up from STOP */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE(x)                     (((uint32_t)(x)) << 8)          /* HSE division factor for RTC clock (2 - 63) */

#define RCC_CFGR_HRTIMSEL                      ((uint32_t)0x00004000)
#define RCC_CFGR_TIMPRE                        ((uint32_t)0x00008000)

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_SHIFT                    (22)      /* Bits 22-24: Microcontroller Clock Output 1 */
#define RCC_CFGR_MCO1_MASK                     (7ul << RCC_CFGR_MCO1_SHIFT)
#define RCC_CFGR_MCO1_HSI                      (0ul << RCC_CFGR_MCO1_SHIFT)     /* 000: HSI clock selected */
#define RCC_CFGR_MCO1_LSE                      (1ul << RCC_CFGR_MCO1_SHIFT)     /* 001: LSE oscillator selected */
#define RCC_CFGR_MCO1_HSE                      (2ul << RCC_CFGR_MCO1_SHIFT)     /* 010: HSE oscillator clock selected */
#define RCC_CFGR_MCO1_PLL                      (3ul << RCC_CFGR_MCO1_SHIFT)     /* 011: PLL clock selected */
#define RCC_CFGR_MCO1_PLL                      (4ul << RCC_CFGR_MCO1_SHIFT)     /* 100: HSI48 clock selected */
#define RCC_CFGR_MCO1PRE(x)                    (((uint32_t)(x)) << 18)          /* MCO1 prescaler */

/*!< MCO2 configuration */
#define RCC_CFGR_MCO2_SHIFT                    (29)      /* Bits 29-31: Microcontroller Clock Output 2 */
#define RCC_CFGR_MCO2_MASK                     (7ul << RCC_CFGR_MCO1_SHIFT)
#define RCC_CFGR_MCO2_HSI                      (0ul << RCC_CFGR_MCO1_SHIFT)     /* 000: HSI clock selected */
#define RCC_CFGR_MCO2_LSE                      (1ul << RCC_CFGR_MCO1_SHIFT)     /* 001: LSE oscillator selected */
#define RCC_CFGR_MCO2_HSE                      (2ul << RCC_CFGR_MCO1_SHIFT)     /* 010: HSE oscillator clock selected */
#define RCC_CFGR_MCO2_PLL                      (3ul << RCC_CFGR_MCO1_SHIFT)     /* 011: PLL clock selected */
#define RCC_CFGR_MCO2_PLL                      (4ul << RCC_CFGR_MCO1_SHIFT)     /* 100: HSI48 clock selected */
#define RCC_CFGR_MCO2PRE(x)                    (((uint32_t)(x)) << 25)          /* MCO2 prescaler */

/********************  Bit definition for RCC_PLLCKSELR register  *************/

#define RCC_PLLCKSELR_PLLSRC_Pos               (0U)
#define RCC_PLLCKSELR_PLLSRC_Msk               (0x3U << RCC_PLLCKSELR_PLLSRC_Pos) /* 0x00000003 */
#define RCC_PLLCKSELR_PLLSRC                   RCC_PLLCKSELR_PLLSRC_Msk

#define RCC_PLLCKSELR_PLLSRC_HSI               ((uint32_t)0x00000000)          /* HSI source clock selected */
#define RCC_PLLCKSELR_PLLSRC_CSI               ((uint32_t)0x00000001)          /* CSI source clock selected */
#define RCC_PLLCKSELR_PLLSRC_HSE               ((uint32_t)0x00000002)          /* HSE source clock selected */
#define RCC_PLLCKSELR_PLLSRC_NONE              ((uint32_t)0x00000003)          /* No source clock selected  */

#define RCC_PLLCKSELR_DIVM1_Pos                (4U)
#define RCC_PLLCKSELR_DIVM1(x)                 ((x) << RCC_PLLCKSELR_DIVM1_Pos) /* Prescaler for PLL1: 1 - 63, 0 = disabled */
#define RCC_PLLCKSELR_DIVM2_Pos                (12U)
#define RCC_PLLCKSELR_DIVM2(x)                 ((x) << RCC_PLLCKSELR_DIVM2_Pos) /* Prescaler for PLL2: 1 - 63, 0 = disabled */
#define RCC_PLLCKSELR_DIVM3_Pos                (20U)
#define RCC_PLLCKSELR_DIVM3(x)                 ((x) << RCC_PLLCKSELR_DIVM3_Pos) /* Prescaler for PLL3: 1 - 63, 0 = disabled */

/********************  Bit definition for RCC_PLLCFGR register  ***************/

#define RCC_PLLCFGR_RESET                      ((uint32_t)0x01FF0000)

#define RCC_PLLCFGR_PLL1FRACEN_Pos             (0U)
#define RCC_PLLCFGR_PLL1FRACEN_Msk             (0x1U << RCC_PLLCFGR_PLL1FRACEN_Pos)
#define RCC_PLLCFGR_PLL1FRACEN                 RCC_PLLCFGR_PLL1FRACEN_Msk        /* Fractional latch enable */
#define RCC_PLLCFGR_PLL1VCOSEL_Pos             (1U)
#define RCC_PLLCFGR_PLL1VCOSEL_Msk             (0x1U << RCC_PLLCFGR_PLL1VCOSEL_Pos)
#define RCC_PLLCFGR_PLL1VCOSEL                 RCC_PLLCFGR_PLL1VCOSEL_Msk        /* VCO frequency range: 1 = Medium VCO range: 150 to 420 MHz, 0 = Wide VCO range: 192 to 836 MHz */
#define RCC_PLLCFGR_PLL1VCOSEL_WIDE            (0U)                              /* VCO frequency range: Wide VCO range: 192 to 836 MHz, input clock >= 2 MHz */
#define RCC_PLLCFGR_PLL1VCOSEL_MEDIUM          RCC_PLLCFGR_PLL3VCOSEL            /* VCO frequency range: Medium VCO range: 150 to 420 MHz, input clock <= 2 MHz */
#define RCC_PLLCFGR_PLL1RGE_Pos                (2U)
#define RCC_PLLCFGR_PLL1RGE_Msk                (0x3U << RCC_PLLCFGR_PLL1RGE_Pos)
#define RCC_PLLCFGR_PLL1RGE                    RCC_PLLCFGR_PLL1RGE_Msk
#define RCC_PLLCFGR_PLL1RGE_1_2_MHZ            (0x0U << RCC_PLLCFGR_PLL1RGE_Pos) /* The PLL input clock range frequency is between 1 and 2 MHz */
#define RCC_PLLCFGR_PLL1RGE_2_4_MHZ            (0x1U << RCC_PLLCFGR_PLL1RGE_Pos) /* The PLL input clock range frequency is between 2 and 4 MHz */
#define RCC_PLLCFGR_PLL1RGE_4_8_MHZ            (0x2U << RCC_PLLCFGR_PLL1RGE_Pos) /* The PLL input clock range frequency is between 4 and 8 MHz */
#define RCC_PLLCFGR_PLL1RGE_8_16_MHZ           (0x3U << RCC_PLLCFGR_PLL1RGE_Pos) /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLLCFGR_PLL2FRACEN_Pos             (0U)
#define RCC_PLLCFGR_PLL2FRACEN_Msk             (0x1U << RCC_PLLCFGR_PLL2FRACEN_Pos)
#define RCC_PLLCFGR_PLL2FRACEN                 RCC_PLLCFGR_PLL2FRACEN_Msk        /* Fractional latch enable */
#define RCC_PLLCFGR_PLL2VCOSEL_Pos             (1U)
#define RCC_PLLCFGR_PLL2VCOSEL_Msk             (0x1U << RCC_PLLCFGR_PLL2VCOSEL_Pos)
#define RCC_PLLCFGR_PLL2VCOSEL                 RCC_PLLCFGR_PLL2VCOSEL_Msk        /* VCO frequency range: 1 = Medium VCO range: 150 to 420 MHz, 0 = Wide VCO range: 192 to 836 MHz */
#define RCC_PLLCFGR_PLL2VCOSEL_WIDE            (0U)                              /* VCO frequency range: Wide VCO range: 192 to 836 MHz, input clock >= 2 MHz */
#define RCC_PLLCFGR_PLL2VCOSEL_MEDIUM          RCC_PLLCFGR_PLL3VCOSEL            /* VCO frequency range: Medium VCO range: 150 to 420 MHz, input clock <= 2 MHz */
#define RCC_PLLCFGR_PLL2RGE_Pos                (2U)
#define RCC_PLLCFGR_PLL2RGE_Msk                (0x3U << RCC_PLLCFGR_PLL2RGE_Pos)
#define RCC_PLLCFGR_PLL2RGE                    RCC_PLLCFGR_PLL2RGE_Msk
#define RCC_PLLCFGR_PLL2RGE_1_2_MHZ            (0x0U << RCC_PLLCFGR_PLL2RGE_Pos) /* The PLL input clock range frequency is between 1 and 2 MHz */
#define RCC_PLLCFGR_PLL2RGE_2_4_MHZ            (0x1U << RCC_PLLCFGR_PLL2RGE_Pos) /* The PLL input clock range frequency is between 2 and 4 MHz */
#define RCC_PLLCFGR_PLL2RGE_4_8_MHZ            (0x2U << RCC_PLLCFGR_PLL2RGE_Pos) /* The PLL input clock range frequency is between 4 and 8 MHz */
#define RCC_PLLCFGR_PLL2RGE_8_16_MHZ           (0x3U << RCC_PLLCFGR_PLL2RGE_Pos) /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLLCFGR_PLL3FRACEN_Pos             (0U)
#define RCC_PLLCFGR_PLL3FRACEN_Msk             (0x1U << RCC_PLLCFGR_PLL3FRACEN_Pos)
#define RCC_PLLCFGR_PLL3FRACEN                 RCC_PLLCFGR_PLL3FRACEN_Msk        /* Fractional latch enable */
#define RCC_PLLCFGR_PLL3VCOSEL_Pos             (1U)
#define RCC_PLLCFGR_PLL3VCOSEL_Msk             (0x1U << RCC_PLLCFGR_PLL3VCOSEL_Pos)
#define RCC_PLLCFGR_PLL3VCOSEL                 RCC_PLLCFGR_PLL3VCOSEL_Msk        /* VCO frequency range: 1 = Medium VCO range: 150 to 420 MHz, 0 = Wide VCO range: 192 to 836 MHz */
#define RCC_PLLCFGR_PLL3VCOSEL_WIDE            (0U)                              /* VCO frequency range: Wide VCO range: 192 to 836 MHz, input clock >= 2 MHz */
#define RCC_PLLCFGR_PLL3VCOSEL_MEDIUM          RCC_PLLCFGR_PLL3VCOSEL            /* VCO frequency range: Medium VCO range: 150 to 420 MHz, input clock <= 2 MHz */
#define RCC_PLLCFGR_PLL3RGE_Pos                (2U)
#define RCC_PLLCFGR_PLL3RGE_Msk                (0x3U << RCC_PLLCFGR_PLL3RGE_Pos)
#define RCC_PLLCFGR_PLL3RGE                    RCC_PLLCFGR_PLL3RGE_Msk
#define RCC_PLLCFGR_PLL3RGE_1_2_MHZ            (0x0U << RCC_PLLCFGR_PLL3RGE_Pos) /* The PLL input clock range frequency is between 1 and 2 MHz */
#define RCC_PLLCFGR_PLL3RGE_2_4_MHZ            (0x1U << RCC_PLLCFGR_PLL3RGE_Pos) /* The PLL input clock range frequency is between 2 and 4 MHz */
#define RCC_PLLCFGR_PLL3RGE_4_8_MHZ            (0x2U << RCC_PLLCFGR_PLL3RGE_Pos) /* The PLL input clock range frequency is between 4 and 8 MHz */
#define RCC_PLLCFGR_PLL3RGE_8_16_MHZ           (0x3U << RCC_PLLCFGR_PLL3RGE_Pos) /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLLCFGR_DIVP1EN_Pos                (16U)
#define RCC_PLLCFGR_DIVP1EN_Msk                (0x1U << RCC_PLLCFGR_DIVP1EN_Pos) /*!< 0x00010000 */
#define RCC_PLLCFGR_DIVP1EN                    RCC_PLLCFGR_DIVP1EN_Msk
#define RCC_PLLCFGR_DIVQ1EN_Pos                (17U)
#define RCC_PLLCFGR_DIVQ1EN_Msk                (0x1U << RCC_PLLCFGR_DIVQ1EN_Pos) /*!< 0x00020000 */
#define RCC_PLLCFGR_DIVQ1EN                    RCC_PLLCFGR_DIVQ1EN_Msk
#define RCC_PLLCFGR_DIVR1EN_Pos                (18U)
#define RCC_PLLCFGR_DIVR1EN_Msk                (0x1U << RCC_PLLCFGR_DIVR1EN_Pos) /*!< 0x00040000 */
#define RCC_PLLCFGR_DIVR1EN                    RCC_PLLCFGR_DIVR1EN_Msk

#define RCC_PLLCFGR_DIVP2EN_Pos                (19U)
#define RCC_PLLCFGR_DIVP2EN_Msk                (0x1U << RCC_PLLCFGR_DIVP2EN_Pos) /*!< 0x00080000 */
#define RCC_PLLCFGR_DIVP2EN                    RCC_PLLCFGR_DIVP2EN_Msk
#define RCC_PLLCFGR_DIVQ2EN_Pos                (20U)
#define RCC_PLLCFGR_DIVQ2EN_Msk                (0x1U << RCC_PLLCFGR_DIVQ2EN_Pos) /*!< 0x00100000 */
#define RCC_PLLCFGR_DIVQ2EN                    RCC_PLLCFGR_DIVQ2EN_Msk
#define RCC_PLLCFGR_DIVR2EN_Pos                (21U)
#define RCC_PLLCFGR_DIVR2EN_Msk                (0x1U << RCC_PLLCFGR_DIVR2EN_Pos) /*!< 0x00200000 */
#define RCC_PLLCFGR_DIVR2EN                    RCC_PLLCFGR_DIVR2EN_Msk

#define RCC_PLLCFGR_DIVP3EN_Pos                (22U)
#define RCC_PLLCFGR_DIVP3EN_Msk                (0x1U << RCC_PLLCFGR_DIVP3EN_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_DIVP3EN                    RCC_PLLCFGR_DIVP3EN_Msk
#define RCC_PLLCFGR_DIVQ3EN_Pos                (23U)
#define RCC_PLLCFGR_DIVQ3EN_Msk                (0x1U << RCC_PLLCFGR_DIVQ3EN_Pos) /*!< 0x00800000 */
#define RCC_PLLCFGR_DIVQ3EN                    RCC_PLLCFGR_DIVQ3EN_Msk
#define RCC_PLLCFGR_DIVR3EN_Pos                (24U)
#define RCC_PLLCFGR_DIVR3EN_Msk                (0x1U << RCC_PLLCFGR_DIVR3EN_Pos) /*!< 0x01000000 */
#define RCC_PLLCFGR_DIVR3EN                    RCC_PLLCFGR_DIVR3EN_Msk

/********************  Bit definition for RCC_PLL1DIVR register  ***************/

#define RCC_PLL1DIVR_N1_Pos                    (0U)
#define RCC_PLL1DIVR_N1(x)                     (((x) - 1) << RCC_PLL1DIVR_N1_Pos) /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL1DIVR_P1_Pos                    (9U)
#define RCC_PLL1DIVR_P1(x)                     (((x) - 1) << RCC_PLL1DIVR_P1_Pos) /* DIVP division factor: 2 - 128, must be even */
#define RCC_PLL1DIVR_Q1_Pos                    (16U)
#define RCC_PLL1DIVR_Q1(x)                     (((x) - 1) << RCC_PLL1DIVR_Q1_Pos) /* DIVQ division factor: 2 - 128 */
#define RCC_PLL1DIVR_R1_Pos                    (24U)
#define RCC_PLL1DIVR_R1(x)                     (((x) - 1) << RCC_PLL1DIVR_R1_Pos) /* DIVR division factor: 2 - 128 */

/********************  Bit definition for RCC_PLL1FRACR register  ***************/

#define RCC_PLL1FRACR_FRACN1_Pos               (3U)
#define RCC_PLL1FRACR_FRACN1_Msk               (0x1FFFU << RCC_PLL1FRACR_FRACN1_Pos) /*!< 0x0000FFF8 */
#define RCC_PLL1FRACR_FRACN1                   RCC_PLL1FRACR_FRACN1_Msk

/********************  Bit definition for RCC_PLL2DIVR register  ***************/

#define RCC_PLL2DIVR_N2_Pos                    (0U)
#define RCC_PLL2DIVR_N2(x)                     (((x) - 1) << RCC_PLL2DIVR_N2_Pos) /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL2DIVR_P2_Pos                    (9U)
#define RCC_PLL2DIVR_P2(x)                     (((x) - 1) << RCC_PLL2DIVR_P2_Pos) /* DIVP division factor: 2 - 128 */
#define RCC_PLL2DIVR_Q2_Pos                    (16U)
#define RCC_PLL2DIVR_Q2(x)                     (((x) - 1) << RCC_PLL2DIVR_Q2_Pos) /* DIVQ division factor: 2 - 128 */
#define RCC_PLL2DIVR_R2_Pos                    (24U)
#define RCC_PLL2DIVR_R2(x)                     (((x) - 1) << RCC_PLL2DIVR_R2_Pos) /* DIVR division factor: 2 - 128 */

/********************  Bit definition for RCC_PLL2FRACR register  ***************/

#define RCC_PLL2FRACR_FRACN2_Pos               (3U)
#define RCC_PLL2FRACR_FRACN2_Msk               (0x1FFFU << RCC_PLL2FRACR_FRACN2_Pos) /*!< 0x0000FFF8 */
#define RCC_PLL2FRACR_FRACN2                   RCC_PLL2FRACR_FRACN2_Msk

/********************  Bit definition for RCC_PLL3DIVR register  ***************/

#define RCC_PLL3DIVR_N3_Pos                    (0U)
#define RCC_PLL3DIVR_N3(x)                     (((x) - 1) << RCC_PLL3DIVR_N3_Pos) /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL3DIVR_P3_Pos                    (9U)
#define RCC_PLL3DIVR_P3(x)                     (((x) - 1) << RCC_PLL3DIVR_P3_Pos) /* DIVP division factor: 2 - 128 */
#define RCC_PLL3DIVR_Q3_Pos                    (16U)
#define RCC_PLL3DIVR_Q3(x)                     (((x) - 1) << RCC_PLL3DIVR_Q3_Pos) /* DIVQ division factor: 2 - 128 */
#define RCC_PLL3DIVR_R3_Pos                    (24U)
#define RCC_PLL3DIVR_R3(x)                     (((x) - 1) << RCC_PLL3DIVR_R3_Pos) /* DIVR division factor: 2 - 128 */

/********************  Bit definition for RCC_PLL3FRACR register  ***************/

#define RCC_PLL3FRACR_FRACN3_Pos               (3U)
#define RCC_PLL3FRACR_FRACN3_Msk               (0x1FFFU << RCC_PLL3FRACR_FRACN3_Pos) /*!< 0x0000FFF8 */
#define RCC_PLL3FRACR_FRACN3                   RCC_PLL3FRACR_FRACN3_Msk

/* ==========================================================  CSR  ========================================================== */

#define RCC_CSR_LSION_Pos                 (0UL)                     /*!< RCC CSR: LSION (Bit 0)                                */
#define RCC_CSR_LSION                     (0x1UL)                   /*!< RCC CSR: LSION (Bitfield-Mask: 0x01)                  */
#define RCC_CSR_LSIRDY_Pos                (1UL)                     /*!< RCC CSR: LSIRDY (Bit 1)                               */
#define RCC_CSR_LSIRDY                    (0x2UL)                   /*!< RCC CSR: LSIRDY (Bitfield-Mask: 0x01)                 */

/* AHB3 peripheral reset register */

#define RCC_AHB3RSTR_MDMARST_Pos          (0UL)                     /*!< RCC AHB3RSTR: MDMARST (Bit 0)                         */
#define RCC_AHB3RSTR_MDMARST              (0x1UL)                   /*!< RCC AHB3RSTR: MDMARST (Bitfield-Mask: 0x01)           */
#define RCC_AHB3RSTR_DMA2DRST_Pos         (4UL)                     /*!< RCC AHB3RSTR: DMA2DRST (Bit 4)                        */
#define RCC_AHB3RSTR_DMA2DRST             (0x10UL)                  /*!< RCC AHB3RSTR: DMA2DRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB3RSTR_JPGDECRST_Pos        (5UL)                     /*!< RCC AHB3RSTR: JPGDECRST (Bit 5)                       */
#define RCC_AHB3RSTR_JPGDECRST            (0x20UL)                  /*!< RCC AHB3RSTR: JPGDECRST (Bitfield-Mask: 0x01)         */
#define RCC_AHB3RSTR_FMCRST_Pos           (12UL)                    /*!< RCC AHB3RSTR: FMCRST (Bit 12)                         */
#define RCC_AHB3RSTR_FMCRST               (0x1000UL)                /*!< RCC AHB3RSTR: FMCRST (Bitfield-Mask: 0x01)            */
#define RCC_AHB3RSTR_QSPIRST_Pos          (14UL)                    /*!< RCC AHB3RSTR: QSPIRST (Bit 14)                        */
#define RCC_AHB3RSTR_QSPIRST              (0x4000UL)                /*!< RCC AHB3RSTR: QSPIRST (Bitfield-Mask: 0x01)           */
#define RCC_AHB3RSTR_SDMMC1RST_Pos        (16UL)                    /*!< RCC AHB3RSTR: SDMMC1RST (Bit 16)                      */
#define RCC_AHB3RSTR_SDMMC1RST            (0x10000UL)               /*!< RCC AHB3RSTR: SDMMC1RST (Bitfield-Mask: 0x01)         */
#define RCC_AHB3RSTR_CPURST_Pos           (31UL)                    /*!< RCC AHB3RSTR: CPURST (Bit 31)                         */
#define RCC_AHB3RSTR_CPURST               (0x80000000UL)            /*!< RCC AHB3RSTR: CPURST (Bitfield-Mask: 0x01)            */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_DMA1RST_Pos          (0UL)                     /*!< RCC AHB1RSTR: DMA1RST (Bit 0)                         */
#define RCC_AHB1RSTR_DMA1RST              (0x1UL)                   /*!< RCC AHB1RSTR: DMA1RST (Bitfield-Mask: 0x01)           */
#define RCC_AHB1RSTR_DMA2RST_Pos          (1UL)                     /*!< RCC AHB1RSTR: DMA2RST (Bit 1)                         */
#define RCC_AHB1RSTR_DMA2RST              (0x2UL)                   /*!< RCC AHB1RSTR: DMA2RST (Bitfield-Mask: 0x01)           */
#define RCC_AHB1RSTR_ADC12RST_Pos         (5UL)                     /*!< RCC AHB1RSTR: ADC12RST (Bit 5)                        */
#define RCC_AHB1RSTR_ADC12RST             (0x20UL)                  /*!< RCC AHB1RSTR: ADC12RST (Bitfield-Mask: 0x01)          */
#define RCC_AHB1RSTR_ETH1MACRST_Pos       (15UL)                    /*!< RCC AHB1RSTR: ETH1MACRST (Bit 15)                     */
#define RCC_AHB1RSTR_ETH1MACRST           (0x8000UL)                /*!< RCC AHB1RSTR: ETH1MACRST (Bitfield-Mask: 0x01)        */
#define RCC_AHB1RSTR_USB1OTGRST_Pos       (25UL)                    /*!< RCC AHB1RSTR: USB1OTGRST (Bit 25)                     */
#define RCC_AHB1RSTR_USB1OTGRST           (0x2000000UL)             /*!< RCC AHB1RSTR: USB1OTGRST (Bitfield-Mask: 0x01)        */
#define RCC_AHB1RSTR_USB2OTGRST_Pos       (27UL)                    /*!< RCC AHB1RSTR: USB2OTGRST (Bit 27)                     */
#define RCC_AHB1RSTR_USB2OTGRST           (0x8000000UL)             /*!< RCC AHB1RSTR: USB2OTGRST (Bitfield-Mask: 0x01)        */

/* AHB2 peripheral reset register */

#define RCC_AHB2RSTR_CAMITFRST_Pos        (0UL)                     /*!< RCC AHB2RSTR: CAMITFRST (Bit 0)                       */
#define RCC_AHB2RSTR_CAMITFRST            (0x1UL)                   /*!< RCC AHB2RSTR: CAMITFRST (Bitfield-Mask: 0x01)         */
#define RCC_AHB2RSTR_CRYPTRST_Pos         (4UL)                     /*!< RCC AHB2RSTR: CRYPTRST (Bit 4)                        */
#define RCC_AHB2RSTR_CRYPTRST             (0x10UL)                  /*!< RCC AHB2RSTR: CRYPTRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB2RSTR_HASHRST_Pos          (5UL)                     /*!< RCC AHB2RSTR: HASHRST (Bit 5)                         */
#define RCC_AHB2RSTR_HASHRST              (0x20UL)                  /*!< RCC AHB2RSTR: HASHRST (Bitfield-Mask: 0x01)           */
#define RCC_AHB2RSTR_RNGRST_Pos           (6UL)                     /*!< RCC AHB2RSTR: RNGRST (Bit 6)                          */
#define RCC_AHB2RSTR_RNGRST               (0x40UL)                  /*!< RCC AHB2RSTR: RNGRST (Bitfield-Mask: 0x01)            */
#define RCC_AHB2RSTR_SDMMC2RST_Pos        (9UL)                     /*!< RCC AHB2RSTR: SDMMC2RST (Bit 9)                       */
#define RCC_AHB2RSTR_SDMMC2RST            (0x200UL)                 /*!< RCC AHB2RSTR: SDMMC2RST (Bitfield-Mask: 0x01)         */

/* AHB4 peripheral reset register */

#define RCC_AHB4RSTR_GPIOARST_Pos         (0UL)                     /*!< RCC AHB4RSTR: GPIOARST (Bit 0)                        */
#define RCC_AHB4RSTR_GPIOARST             (0x1UL)                   /*!< RCC AHB4RSTR: GPIOARST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOBRST_Pos         (1UL)                     /*!< RCC AHB4RSTR: GPIOBRST (Bit 1)                        */
#define RCC_AHB4RSTR_GPIOBRST             (0x2UL)                   /*!< RCC AHB4RSTR: GPIOBRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOCRST_Pos         (2UL)                     /*!< RCC AHB4RSTR: GPIOCRST (Bit 2)                        */
#define RCC_AHB4RSTR_GPIOCRST             (0x4UL)                   /*!< RCC AHB4RSTR: GPIOCRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIODRST_Pos         (3UL)                     /*!< RCC AHB4RSTR: GPIODRST (Bit 3)                        */
#define RCC_AHB4RSTR_GPIODRST             (0x8UL)                   /*!< RCC AHB4RSTR: GPIODRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOERST_Pos         (4UL)                     /*!< RCC AHB4RSTR: GPIOERST (Bit 4)                        */
#define RCC_AHB4RSTR_GPIOERST             (0x10UL)                  /*!< RCC AHB4RSTR: GPIOERST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOFRST_Pos         (5UL)                     /*!< RCC AHB4RSTR: GPIOFRST (Bit 5)                        */
#define RCC_AHB4RSTR_GPIOFRST             (0x20UL)                  /*!< RCC AHB4RSTR: GPIOFRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOGRST_Pos         (6UL)                     /*!< RCC AHB4RSTR: GPIOGRST (Bit 6)                        */
#define RCC_AHB4RSTR_GPIOGRST             (0x40UL)                  /*!< RCC AHB4RSTR: GPIOGRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOHRST_Pos         (7UL)                     /*!< RCC AHB4RSTR: GPIOHRST (Bit 7)                        */
#define RCC_AHB4RSTR_GPIOHRST             (0x80UL)                  /*!< RCC AHB4RSTR: GPIOHRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOIRST_Pos         (8UL)                     /*!< RCC AHB4RSTR: GPIOIRST (Bit 8)                        */
#define RCC_AHB4RSTR_GPIOIRST             (0x100UL)                 /*!< RCC AHB4RSTR: GPIOIRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOJRST_Pos         (9UL)                     /*!< RCC AHB4RSTR: GPIOJRST (Bit 9)                        */
#define RCC_AHB4RSTR_GPIOJRST             (0x200UL)                 /*!< RCC AHB4RSTR: GPIOJRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_GPIOKRST_Pos         (10UL)                    /*!< RCC AHB4RSTR: GPIOKRST (Bit 10)                       */
#define RCC_AHB4RSTR_GPIOKRST             (0x400UL)                 /*!< RCC AHB4RSTR: GPIOKRST (Bitfield-Mask: 0x01)          */
#define RCC_AHB4RSTR_CRCRST_Pos           (19UL)                    /*!< RCC AHB4RSTR: CRCRST (Bit 19)                         */
#define RCC_AHB4RSTR_CRCRST               (0x80000UL)               /*!< RCC AHB4RSTR: CRCRST (Bitfield-Mask: 0x01)            */
#define RCC_AHB4RSTR_BDMARST_Pos          (21UL)                    /*!< RCC AHB4RSTR: BDMARST (Bit 21)                        */
#define RCC_AHB4RSTR_BDMARST              (0x200000UL)              /*!< RCC AHB4RSTR: BDMARST (Bitfield-Mask: 0x01)           */
#define RCC_AHB4RSTR_ADC3RST_Pos          (24UL)                    /*!< RCC AHB4RSTR: ADC3RST (Bit 24)                        */
#define RCC_AHB4RSTR_ADC3RST              (0x1000000UL)             /*!< RCC AHB4RSTR: ADC3RST (Bitfield-Mask: 0x01)           */
#define RCC_AHB4RSTR_HSEMRST_Pos          (25UL)                    /*!< RCC AHB4RSTR: HSEMRST (Bit 25)                        */
#define RCC_AHB4RSTR_HSEMRST              (0x2000000UL)             /*!< RCC AHB4RSTR: HSEMRST (Bitfield-Mask: 0x01)           */

/* APB3 peripheral reset register */

#define RCC_APB3RSTR_LTDCRST_Pos          (3UL)                     /*!< RCC APB3RSTR: LTDCRST (Bit 3)                         */
#define RCC_APB3RSTR_LTDCRST              (0x8UL)                   /*!< RCC APB3RSTR: LTDCRST (Bitfield-Mask: 0x01)           */

/* APB3 L peripheral reset register */

#define RCC_APB1LRSTR_TIM2RST_Pos         (0UL)                     /*!< RCC APB1LRSTR: TIM2RST (Bit 0)                        */
#define RCC_APB1LRSTR_TIM2RST             (0x1UL)                   /*!< RCC APB1LRSTR: TIM2RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_TIM3RST_Pos         (1UL)                     /*!< RCC APB1LRSTR: TIM3RST (Bit 1)                        */
#define RCC_APB1LRSTR_TIM3RST             (0x2UL)                   /*!< RCC APB1LRSTR: TIM3RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_TIM4RST_Pos         (2UL)                     /*!< RCC APB1LRSTR: TIM4RST (Bit 2)                        */
#define RCC_APB1LRSTR_TIM4RST             (0x4UL)                   /*!< RCC APB1LRSTR: TIM4RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_TIM5RST_Pos         (3UL)                     /*!< RCC APB1LRSTR: TIM5RST (Bit 3)                        */
#define RCC_APB1LRSTR_TIM5RST             (0x8UL)                   /*!< RCC APB1LRSTR: TIM5RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_TIM6RST_Pos         (4UL)                     /*!< RCC APB1LRSTR: TIM6RST (Bit 4)                        */
#define RCC_APB1LRSTR_TIM6RST             (0x10UL)                  /*!< RCC APB1LRSTR: TIM6RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_TIM7RST_Pos         (5UL)                     /*!< RCC APB1LRSTR: TIM7RST (Bit 5)                        */
#define RCC_APB1LRSTR_TIM7RST             (0x20UL)                  /*!< RCC APB1LRSTR: TIM7RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_TIM12RST_Pos        (6UL)                     /*!< RCC APB1LRSTR: TIM12RST (Bit 6)                       */
#define RCC_APB1LRSTR_TIM12RST            (0x40UL)                  /*!< RCC APB1LRSTR: TIM12RST (Bitfield-Mask: 0x01)         */
#define RCC_APB1LRSTR_TIM13RST_Pos        (7UL)                     /*!< RCC APB1LRSTR: TIM13RST (Bit 7)                       */
#define RCC_APB1LRSTR_TIM13RST            (0x80UL)                  /*!< RCC APB1LRSTR: TIM13RST (Bitfield-Mask: 0x01)         */
#define RCC_APB1LRSTR_TIM14RST_Pos        (8UL)                     /*!< RCC APB1LRSTR: TIM14RST (Bit 8)                       */
#define RCC_APB1LRSTR_TIM14RST            (0x100UL)                 /*!< RCC APB1LRSTR: TIM14RST (Bitfield-Mask: 0x01)         */
#define RCC_APB1LRSTR_LPTIM1RST_Pos       (9UL)                     /*!< RCC APB1LRSTR: LPTIM1RST (Bit 9)                      */
#define RCC_APB1LRSTR_LPTIM1RST           (0x200UL)                 /*!< RCC APB1LRSTR: LPTIM1RST (Bitfield-Mask: 0x01)        */
#define RCC_APB1LRSTR_SPI2RST_Pos         (14UL)                    /*!< RCC APB1LRSTR: SPI2RST (Bit 14)                       */
#define RCC_APB1LRSTR_SPI2RST             (0x4000UL)                /*!< RCC APB1LRSTR: SPI2RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_SPI3RST_Pos         (15UL)                    /*!< RCC APB1LRSTR: SPI3RST (Bit 15)                       */
#define RCC_APB1LRSTR_SPI3RST             (0x8000UL)                /*!< RCC APB1LRSTR: SPI3RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_SPDIFRXRST_Pos      (16UL)                    /*!< RCC APB1LRSTR: SPDIFRXRST (Bit 16)                    */
#define RCC_APB1LRSTR_SPDIFRXRST          (0x10000UL)               /*!< RCC APB1LRSTR: SPDIFRXRST (Bitfield-Mask: 0x01)       */
#define RCC_APB1LRSTR_USART2RST_Pos       (17UL)                    /*!< RCC APB1LRSTR: USART2RST (Bit 17)                     */
#define RCC_APB1LRSTR_USART2RST           (0x20000UL)               /*!< RCC APB1LRSTR: USART2RST (Bitfield-Mask: 0x01)        */
#define RCC_APB1LRSTR_USART3RST_Pos       (18UL)                    /*!< RCC APB1LRSTR: USART3RST (Bit 18)                     */
#define RCC_APB1LRSTR_USART3RST           (0x40000UL)               /*!< RCC APB1LRSTR: USART3RST (Bitfield-Mask: 0x01)        */
#define RCC_APB1LRSTR_UART4RST_Pos        (19UL)                    /*!< RCC APB1LRSTR: UART4RST (Bit 19)                      */
#define RCC_APB1LRSTR_UART4RST            (0x80000UL)               /*!< RCC APB1LRSTR: UART4RST (Bitfield-Mask: 0x01)         */
#define RCC_APB1LRSTR_UART5RST_Pos        (20UL)                    /*!< RCC APB1LRSTR: UART5RST (Bit 20)                      */
#define RCC_APB1LRSTR_UART5RST            (0x100000UL)              /*!< RCC APB1LRSTR: UART5RST (Bitfield-Mask: 0x01)         */
#define RCC_APB1LRSTR_I2C1RST_Pos         (21UL)                    /*!< RCC APB1LRSTR: I2C1RST (Bit 21)                       */
#define RCC_APB1LRSTR_I2C1RST             (0x200000UL)              /*!< RCC APB1LRSTR: I2C1RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_I2C2RST_Pos         (22UL)                    /*!< RCC APB1LRSTR: I2C2RST (Bit 22)                       */
#define RCC_APB1LRSTR_I2C2RST             (0x400000UL)              /*!< RCC APB1LRSTR: I2C2RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_I2C3RST_Pos         (23UL)                    /*!< RCC APB1LRSTR: I2C3RST (Bit 23)                       */
#define RCC_APB1LRSTR_I2C3RST             (0x800000UL)              /*!< RCC APB1LRSTR: I2C3RST (Bitfield-Mask: 0x01)          */
#define RCC_APB1LRSTR_HDMICECRST_Pos      (27UL)                    /*!< RCC APB1LRSTR: HDMICECRST (Bit 27)                    */
#define RCC_APB1LRSTR_HDMICECRST          (0x8000000UL)             /*!< RCC APB1LRSTR: HDMICECRST (Bitfield-Mask: 0x01)       */
#define RCC_APB1LRSTR_DAC12RST_Pos        (29UL)                    /*!< RCC APB1LRSTR: DAC12RST (Bit 29)                      */
#define RCC_APB1LRSTR_DAC12RST            (0x20000000UL)            /*!< RCC APB1LRSTR: DAC12RST (Bitfield-Mask: 0x01)         */
#define RCC_APB1LRSTR_USART7RST_Pos       (30UL)                    /*!< RCC APB1LRSTR: USART7RST (Bit 30)                     */
#define RCC_APB1LRSTR_USART7RST           (0x40000000UL)            /*!< RCC APB1LRSTR: USART7RST (Bitfield-Mask: 0x01)        */
#define RCC_APB1LRSTR_USART8RST_Pos       (31UL)                    /*!< RCC APB1LRSTR: USART8RST (Bit 31)                     */
#define RCC_APB1LRSTR_USART8RST           (0x80000000UL)            /*!< RCC APB1LRSTR: USART8RST (Bitfield-Mask: 0x01)        */

/* APB1 H peripheral reset register */

#define RCC_APB1HRSTR_CRSRST_Pos          (1UL)                     /*!< RCC APB1HRSTR: CRSRST (Bit 1)                         */
#define RCC_APB1HRSTR_CRSRST              (0x2UL)                   /*!< RCC APB1HRSTR: CRSRST (Bitfield-Mask: 0x01)           */
#define RCC_APB1HRSTR_SWPRST_Pos          (2UL)                     /*!< RCC APB1HRSTR: SWPRST (Bit 2)                         */
#define RCC_APB1HRSTR_SWPRST              (0x4UL)                   /*!< RCC APB1HRSTR: SWPRST (Bitfield-Mask: 0x01)           */
#define RCC_APB1HRSTR_OPAMPRST_Pos        (4UL)                     /*!< RCC APB1HRSTR: OPAMPRST (Bit 4)                       */
#define RCC_APB1HRSTR_OPAMPRST            (0x10UL)                  /*!< RCC APB1HRSTR: OPAMPRST (Bitfield-Mask: 0x01)         */
#define RCC_APB1HRSTR_MDIOSRST_Pos        (5UL)                     /*!< RCC APB1HRSTR: MDIOSRST (Bit 5)                       */
#define RCC_APB1HRSTR_MDIOSRST            (0x20UL)                  /*!< RCC APB1HRSTR: MDIOSRST (Bitfield-Mask: 0x01)         */
#define RCC_APB1HRSTR_FDCANRST_Pos        (8UL)                     /*!< RCC APB1HRSTR: FDCANRST (Bit 8)                       */
#define RCC_APB1HRSTR_FDCANRST            (0x100UL)                 /*!< RCC APB1HRSTR: FDCANRST (Bitfield-Mask: 0x01)         */

/* APB2 peripheral reset register */

#define RCC_APB2RSTR_TIM1RST_Pos          (0UL)                     /*!< RCC APB2RSTR: TIM1RST (Bit 0)                         */
#define RCC_APB2RSTR_TIM1RST              (0x1UL)                   /*!< RCC APB2RSTR: TIM1RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_TIM8RST_Pos          (1UL)                     /*!< RCC APB2RSTR: TIM8RST (Bit 1)                         */
#define RCC_APB2RSTR_TIM8RST              (0x2UL)                   /*!< RCC APB2RSTR: TIM8RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_USART1RST_Pos        (4UL)                     /*!< RCC APB2RSTR: USART1RST (Bit 4)                       */
#define RCC_APB2RSTR_USART1RST            (0x10UL)                  /*!< RCC APB2RSTR: USART1RST (Bitfield-Mask: 0x01)         */
#define RCC_APB2RSTR_USART6RST_Pos        (5UL)                     /*!< RCC APB2RSTR: USART6RST (Bit 5)                       */
#define RCC_APB2RSTR_USART6RST            (0x20UL)                  /*!< RCC APB2RSTR: USART6RST (Bitfield-Mask: 0x01)         */
#define RCC_APB2RSTR_SPI1RST_Pos          (12UL)                    /*!< RCC APB2RSTR: SPI1RST (Bit 12)                        */
#define RCC_APB2RSTR_SPI1RST              (0x1000UL)                /*!< RCC APB2RSTR: SPI1RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_SPI4RST_Pos          (13UL)                    /*!< RCC APB2RSTR: SPI4RST (Bit 13)                        */
#define RCC_APB2RSTR_SPI4RST              (0x2000UL)                /*!< RCC APB2RSTR: SPI4RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_TIM15RST_Pos         (16UL)                    /*!< RCC APB2RSTR: TIM15RST (Bit 16)                       */
#define RCC_APB2RSTR_TIM15RST             (0x10000UL)               /*!< RCC APB2RSTR: TIM15RST (Bitfield-Mask: 0x01)          */
#define RCC_APB2RSTR_TIM16RST_Pos         (17UL)                    /*!< RCC APB2RSTR: TIM16RST (Bit 17)                       */
#define RCC_APB2RSTR_TIM16RST             (0x20000UL)               /*!< RCC APB2RSTR: TIM16RST (Bitfield-Mask: 0x01)          */
#define RCC_APB2RSTR_TIM17RST_Pos         (18UL)                    /*!< RCC APB2RSTR: TIM17RST (Bit 18)                       */
#define RCC_APB2RSTR_TIM17RST             (0x40000UL)               /*!< RCC APB2RSTR: TIM17RST (Bitfield-Mask: 0x01)          */
#define RCC_APB2RSTR_SPI5RST_Pos          (20UL)                    /*!< RCC APB2RSTR: SPI5RST (Bit 20)                        */
#define RCC_APB2RSTR_SPI5RST              (0x100000UL)              /*!< RCC APB2RSTR: SPI5RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_SAI1RST_Pos          (22UL)                    /*!< RCC APB2RSTR: SAI1RST (Bit 22)                        */
#define RCC_APB2RSTR_SAI1RST              (0x400000UL)              /*!< RCC APB2RSTR: SAI1RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_SAI2RST_Pos          (23UL)                    /*!< RCC APB2RSTR: SAI2RST (Bit 23)                        */
#define RCC_APB2RSTR_SAI2RST              (0x800000UL)              /*!< RCC APB2RSTR: SAI2RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_SAI3RST_Pos          (24UL)                    /*!< RCC APB2RSTR: SAI3RST (Bit 24)                        */
#define RCC_APB2RSTR_SAI3RST              (0x1000000UL)             /*!< RCC APB2RSTR: SAI3RST (Bitfield-Mask: 0x01)           */
#define RCC_APB2RSTR_DFSDM1RST_Pos        (28UL)                    /*!< RCC APB2RSTR: DFSDM1RST (Bit 28)                      */
#define RCC_APB2RSTR_DFSDM1RST            (0x10000000UL)            /*!< RCC APB2RSTR: DFSDM1RST (Bitfield-Mask: 0x01)         */
#define RCC_APB2RSTR_HRTIMRST_Pos         (29UL)                    /*!< RCC APB2RSTR: HRTIMRST (Bit 29)                       */
#define RCC_APB2RSTR_HRTIMRST             (0x20000000UL)            /*!< RCC APB2RSTR: HRTIMRST (Bitfield-Mask: 0x01)          */

/* APB4 peripheral reset register */

#define RCC_APB4RSTR_SYSCFGRST_Pos        (1UL)                     /*!< RCC APB4RSTR: SYSCFGRST (Bit 1)                       */
#define RCC_APB4RSTR_SYSCFGRST            (0x2UL)                   /*!< RCC APB4RSTR: SYSCFGRST (Bitfield-Mask: 0x01)         */
#define RCC_APB4RSTR_LPUART1RST_Pos       (3UL)                     /*!< RCC APB4RSTR: LPUART1RST (Bit 3)                      */
#define RCC_APB4RSTR_LPUART1RST           (0x8UL)                   /*!< RCC APB4RSTR: LPUART1RST (Bitfield-Mask: 0x01)        */
#define RCC_APB4RSTR_SPI6RST_Pos          (5UL)                     /*!< RCC APB4RSTR: SPI6RST (Bit 5)                         */
#define RCC_APB4RSTR_SPI6RST              (0x20UL)                  /*!< RCC APB4RSTR: SPI6RST (Bitfield-Mask: 0x01)           */
#define RCC_APB4RSTR_I2C4RST_Pos          (7UL)                     /*!< RCC APB4RSTR: I2C4RST (Bit 7)                         */
#define RCC_APB4RSTR_I2C4RST              (0x80UL)                  /*!< RCC APB4RSTR: I2C4RST (Bitfield-Mask: 0x01)           */
#define RCC_APB4RSTR_LPTIM2RST_Pos        (9UL)                     /*!< RCC APB4RSTR: LPTIM2RST (Bit 9)                       */
#define RCC_APB4RSTR_LPTIM2RST            (0x200UL)                 /*!< RCC APB4RSTR: LPTIM2RST (Bitfield-Mask: 0x01)         */
#define RCC_APB4RSTR_LPTIM3RST_Pos        (10UL)                    /*!< RCC APB4RSTR: LPTIM3RST (Bit 10)                      */
#define RCC_APB4RSTR_LPTIM3RST            (0x400UL)                 /*!< RCC APB4RSTR: LPTIM3RST (Bitfield-Mask: 0x01)         */
#define RCC_APB4RSTR_LPTIM4RST_Pos        (11UL)                    /*!< RCC APB4RSTR: LPTIM4RST (Bit 11)                      */
#define RCC_APB4RSTR_LPTIM4RST            (0x800UL)                 /*!< RCC APB4RSTR: LPTIM4RST (Bitfield-Mask: 0x01)         */
#define RCC_APB4RSTR_LPTIM5RST_Pos        (12UL)                    /*!< RCC APB4RSTR: LPTIM5RST (Bit 12)                      */
#define RCC_APB4RSTR_LPTIM5RST            (0x1000UL)                /*!< RCC APB4RSTR: LPTIM5RST (Bitfield-Mask: 0x01)         */
#define RCC_APB4RSTR_COMP12RST_Pos        (14UL)                    /*!< RCC APB4RSTR: COMP12RST (Bit 14)                      */
#define RCC_APB4RSTR_COMP12RST            (0x4000UL)                /*!< RCC APB4RSTR: COMP12RST (Bitfield-Mask: 0x01)         */
#define RCC_APB4RSTR_VREFRST_Pos          (15UL)                    /*!< RCC APB4RSTR: VREFRST (Bit 15)                        */
#define RCC_APB4RSTR_VREFRST              (0x8000UL)                /*!< RCC APB4RSTR: VREFRST (Bitfield-Mask: 0x01)           */
#define RCC_APB4RSTR_SAI4RST_Pos          (21UL)                    /*!< RCC APB4RSTR: SAI4RST (Bit 21)                        */
#define RCC_APB4RSTR_SAI4RST              (0x200000UL)              /*!< RCC APB4RSTR: SAI4RST (Bitfield-Mask: 0x01)           */

/* AHB3 Peripheral Clock enable register */

#define RCC_AHB3ENR_MDMAEN_Pos            (0UL)                     /*!< RCC AHB3ENR: MDMAEN (Bit 0)                           */
#define RCC_AHB3ENR_MDMAEN                (0x1UL)                   /*!< RCC AHB3ENR: MDMAEN (Bitfield-Mask: 0x01)             */
#define RCC_AHB3ENR_DMA2DEN_Pos           (4UL)                     /*!< RCC AHB3ENR: DMA2DEN (Bit 4)                          */
#define RCC_AHB3ENR_DMA2DEN               (0x10UL)                  /*!< RCC AHB3ENR: DMA2DEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB3ENR_JPGDECEN_Pos          (5UL)                     /*!< RCC AHB3ENR: JPGDECEN (Bit 5)                         */
#define RCC_AHB3ENR_JPGDECEN              (0x20UL)                  /*!< RCC AHB3ENR: JPGDECEN (Bitfield-Mask: 0x01)           */
#define RCC_AHB3ENR_FMCEN_Pos             (12UL)                    /*!< RCC AHB3ENR: FMCEN (Bit 12)                           */
#define RCC_AHB3ENR_FMCEN                 (0x1000UL)                /*!< RCC AHB3ENR: FMCEN (Bitfield-Mask: 0x01)              */
#define RCC_AHB3ENR_QSPIEN_Pos            (14UL)                    /*!< RCC AHB3ENR: QSPIEN (Bit 14)                          */
#define RCC_AHB3ENR_QSPIEN                (0x4000UL)                /*!< RCC AHB3ENR: QSPIEN (Bitfield-Mask: 0x01)             */
#define RCC_AHB3ENR_SDMMC1EN_Pos          (16UL)                    /*!< RCC AHB3ENR: SDMMC1EN (Bit 16)                        */
#define RCC_AHB3ENR_SDMMC1EN              (0x10000UL)               /*!< RCC AHB3ENR: SDMMC1EN (Bitfield-Mask: 0x01)           */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_DMA1EN_Pos            (0UL)                     /*!< RCC AHB1ENR: DMA1EN (Bit 0)                           */
#define RCC_AHB1ENR_DMA1EN                (0x1UL)                   /*!< RCC AHB1ENR: DMA1EN (Bitfield-Mask: 0x01)             */
#define RCC_AHB1ENR_DMA2EN_Pos            (1UL)                     /*!< RCC AHB1ENR: DMA2EN (Bit 1)                           */
#define RCC_AHB1ENR_DMA2EN                (0x2UL)                   /*!< RCC AHB1ENR: DMA2EN (Bitfield-Mask: 0x01)             */
#define RCC_AHB1ENR_ADC12EN_Pos           (5UL)                     /*!< RCC AHB1ENR: ADC12EN (Bit 5)                          */
#define RCC_AHB1ENR_ADC12EN               (0x20UL)                  /*!< RCC AHB1ENR: ADC12EN (Bitfield-Mask: 0x01)            */
#define RCC_AHB1ENR_ETH1MACEN_Pos         (15UL)                    /*!< RCC AHB1ENR: ETH1MACEN (Bit 15)                       */
#define RCC_AHB1ENR_ETH1MACEN             (0x8000UL)                /*!< RCC AHB1ENR: ETH1MACEN (Bitfield-Mask: 0x01)          */
#define RCC_AHB1ENR_ETH1TXEN_Pos          (16UL)                    /*!< RCC AHB1ENR: ETH1TXEN (Bit 16)                        */
#define RCC_AHB1ENR_ETH1TXEN              (0x10000UL)               /*!< RCC AHB1ENR: ETH1TXEN (Bitfield-Mask: 0x01)           */
#define RCC_AHB1ENR_ETH1RXEN_Pos          (17UL)                    /*!< RCC AHB1ENR: ETH1RXEN (Bit 17)                        */
#define RCC_AHB1ENR_ETH1RXEN              (0x20000UL)               /*!< RCC AHB1ENR: ETH1RXEN (Bitfield-Mask: 0x01)           */
#define RCC_AHB1ENR_USB1OTGEN_Pos         (25UL)                    /*!< RCC AHB1ENR: USB1OTGEN (Bit 25)                       */
#define RCC_AHB1ENR_USB1OTGEN             (0x2000000UL)             /*!< RCC AHB1ENR: USB1OTGEN (Bitfield-Mask: 0x01)          */
#define RCC_AHB1ENR_USB1ULPIEN_Pos        (26UL)                    /*!< RCC AHB1ENR: USB1ULPIEN (Bit 26)                      */
#define RCC_AHB1ENR_USB1ULPIEN            (0x4000000UL)             /*!< RCC AHB1ENR: USB1ULPIEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB1ENR_USB2OTGEN_Pos         (27UL)                    /*!< RCC AHB1ENR: USB2OTGEN (Bit 27)                       */
#define RCC_AHB1ENR_USB2OTGEN             (0x8000000UL)             /*!< RCC AHB1ENR: USB2OTGEN (Bitfield-Mask: 0x01)          */
#define RCC_AHB1ENR_USB2ULPIEN_Pos        (28UL)                    /*!< RCC AHB1ENR: USB2ULPIEN (Bit 28)                      */
#define RCC_AHB1ENR_USB2ULPIEN            (0x10000000UL)            /*!< RCC AHB1ENR: USB2ULPIEN (Bitfield-Mask: 0x01)         */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_CAMITFEN_Pos          (0UL)                     /*!< RCC AHB2ENR: CAMITFEN (Bit 0)                         */
#define RCC_AHB2ENR_CAMITFEN              (0x1UL)                   /*!< RCC AHB2ENR: CAMITFEN (Bitfield-Mask: 0x01)           */
#define RCC_AHB2ENR_CRYPTEN_Pos           (4UL)                     /*!< RCC AHB2ENR: CRYPTEN (Bit 4)                          */
#define RCC_AHB2ENR_CRYPTEN               (0x10UL)                  /*!< RCC AHB2ENR: CRYPTEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB2ENR_HASHEN_Pos            (5UL)                     /*!< RCC AHB2ENR: HASHEN (Bit 5)                           */
#define RCC_AHB2ENR_HASHEN                (0x20UL)                  /*!< RCC AHB2ENR: HASHEN (Bitfield-Mask: 0x01)             */
#define RCC_AHB2ENR_RNGEN_Pos             (6UL)                     /*!< RCC AHB2ENR: RNGEN (Bit 6)                            */
#define RCC_AHB2ENR_RNGEN                 (0x40UL)                  /*!< RCC AHB2ENR: RNGEN (Bitfield-Mask: 0x01)              */
#define RCC_AHB2ENR_SDMMC2EN_Pos          (9UL)                     /*!< RCC AHB2ENR: SDMMC2EN (Bit 9)                         */
#define RCC_AHB2ENR_SDMMC2EN              (0x200UL)                 /*!< RCC AHB2ENR: SDMMC2EN (Bitfield-Mask: 0x01)           */
#define RCC_AHB2ENR_SRAM1EN_Pos           (29UL)                    /*!< RCC AHB2ENR: SRAM1EN (Bit 29)                         */
#define RCC_AHB2ENR_SRAM1EN               (0x20000000UL)            /*!< RCC AHB2ENR: SRAM1EN (Bitfield-Mask: 0x01)            */
#define RCC_AHB2ENR_SRAM2EN_Pos           (30UL)                    /*!< RCC AHB2ENR: SRAM2EN (Bit 30)                         */
#define RCC_AHB2ENR_SRAM2EN               (0x40000000UL)            /*!< RCC AHB2ENR: SRAM2EN (Bitfield-Mask: 0x01)            */
#define RCC_AHB2ENR_SRAM3EN_Pos           (31UL)                    /*!< RCC AHB2ENR: SRAM3EN (Bit 31)                         */
#define RCC_AHB2ENR_SRAM3EN               (0x80000000UL)            /*!< RCC AHB2ENR: SRAM3EN (Bitfield-Mask: 0x01)            */

/* AHB4 Peripheral Clock enable register */

#define RCC_AHB4ENR_GPIOAEN_Pos           (0UL)                     /*!< RCC AHB4ENR: GPIOAEN (Bit 0)                          */
#define RCC_AHB4ENR_GPIOAEN               (0x1UL)                   /*!< RCC AHB4ENR: GPIOAEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOBEN_Pos           (1UL)                     /*!< RCC AHB4ENR: GPIOBEN (Bit 1)                          */
#define RCC_AHB4ENR_GPIOBEN               (0x2UL)                   /*!< RCC AHB4ENR: GPIOBEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOCEN_Pos           (2UL)                     /*!< RCC AHB4ENR: GPIOCEN (Bit 2)                          */
#define RCC_AHB4ENR_GPIOCEN               (0x4UL)                   /*!< RCC AHB4ENR: GPIOCEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIODEN_Pos           (3UL)                     /*!< RCC AHB4ENR: GPIODEN (Bit 3)                          */
#define RCC_AHB4ENR_GPIODEN               (0x8UL)                   /*!< RCC AHB4ENR: GPIODEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOEEN_Pos           (4UL)                     /*!< RCC AHB4ENR: GPIOEEN (Bit 4)                          */
#define RCC_AHB4ENR_GPIOEEN               (0x10UL)                  /*!< RCC AHB4ENR: GPIOEEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOFEN_Pos           (5UL)                     /*!< RCC AHB4ENR: GPIOFEN (Bit 5)                          */
#define RCC_AHB4ENR_GPIOFEN               (0x20UL)                  /*!< RCC AHB4ENR: GPIOFEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOGEN_Pos           (6UL)                     /*!< RCC AHB4ENR: GPIOGEN (Bit 6)                          */
#define RCC_AHB4ENR_GPIOGEN               (0x40UL)                  /*!< RCC AHB4ENR: GPIOGEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOHEN_Pos           (7UL)                     /*!< RCC AHB4ENR: GPIOHEN (Bit 7)                          */
#define RCC_AHB4ENR_GPIOHEN               (0x80UL)                  /*!< RCC AHB4ENR: GPIOHEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOIEN_Pos           (8UL)                     /*!< RCC AHB4ENR: GPIOIEN (Bit 8)                          */
#define RCC_AHB4ENR_GPIOIEN               (0x100UL)                 /*!< RCC AHB4ENR: GPIOIEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOJEN_Pos           (9UL)                     /*!< RCC AHB4ENR: GPIOJEN (Bit 9)                          */
#define RCC_AHB4ENR_GPIOJEN               (0x200UL)                 /*!< RCC AHB4ENR: GPIOJEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_GPIOKEN_Pos           (10UL)                    /*!< RCC AHB4ENR: GPIOKEN (Bit 10)                         */
#define RCC_AHB4ENR_GPIOKEN               (0x400UL)                 /*!< RCC AHB4ENR: GPIOKEN (Bitfield-Mask: 0x01)            */
#define RCC_AHB4ENR_CRCEN_Pos             (19UL)                    /*!< RCC AHB4ENR: CRCEN (Bit 19)                           */
#define RCC_AHB4ENR_CRCEN                 (0x80000UL)               /*!< RCC AHB4ENR: CRCEN (Bitfield-Mask: 0x01)              */
#define RCC_AHB4ENR_BDMAEN_Pos            (21UL)                    /*!< RCC AHB4ENR: BDMAEN (Bit 21)                          */
#define RCC_AHB4ENR_BDMAEN                (0x200000UL)              /*!< RCC AHB4ENR: BDMAEN (Bitfield-Mask: 0x01)             */
#define RCC_AHB4ENR_ADC3EN_Pos            (24UL)                    /*!< RCC AHB4ENR: ADC3EN (Bit 24)                          */
#define RCC_AHB4ENR_ADC3EN                (0x1000000UL)             /*!< RCC AHB4ENR: ADC3EN (Bitfield-Mask: 0x01)             */
#define RCC_AHB4ENR_HSEMEN_Pos            (25UL)                    /*!< RCC AHB4ENR: HSEMEN (Bit 25)                          */
#define RCC_AHB4ENR_HSEMEN                (0x2000000UL)             /*!< RCC AHB4ENR: HSEMEN (Bitfield-Mask: 0x01)             */
#define RCC_AHB4ENR_BKPRAMEN_Pos          (28UL)                    /*!< RCC AHB4ENR: BKPRAMEN (Bit 28)                        */
#define RCC_AHB4ENR_BKPRAMEN              (0x10000000UL)            /*!< RCC AHB4ENR: BKPRAMEN (Bitfield-Mask: 0x01)           */

/* APB3 Peripheral Clock enable register */

#define RCC_APB3ENR_LTDCEN_Pos            (3UL)                     /*!< RCC APB3ENR: LTDCEN (Bit 3)                           */
#define RCC_APB3ENR_LTDCEN                (0x8UL)                   /*!< RCC APB3ENR: LTDCEN (Bitfield-Mask: 0x01)             */
#define RCC_APB3ENR_WWDG1EN_Pos           (6UL)                     /*!< RCC APB3ENR: WWDG1EN (Bit 6)                          */
#define RCC_APB3ENR_WWDG1EN               (0x40UL)                  /*!< RCC APB3ENR: WWDG1EN (Bitfield-Mask: 0x01)            */

/* APB1 L Peripheral Clock enable register */

#define RCC_APB1LENR_TIM2EN_Pos           (0UL)                     /*!< RCC APB1LENR: TIM2EN (Bit 0)                          */
#define RCC_APB1LENR_TIM2EN               (0x1UL)                   /*!< RCC APB1LENR: TIM2EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_TIM3EN_Pos           (1UL)                     /*!< RCC APB1LENR: TIM3EN (Bit 1)                          */
#define RCC_APB1LENR_TIM3EN               (0x2UL)                   /*!< RCC APB1LENR: TIM3EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_TIM4EN_Pos           (2UL)                     /*!< RCC APB1LENR: TIM4EN (Bit 2)                          */
#define RCC_APB1LENR_TIM4EN               (0x4UL)                   /*!< RCC APB1LENR: TIM4EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_TIM5EN_Pos           (3UL)                     /*!< RCC APB1LENR: TIM5EN (Bit 3)                          */
#define RCC_APB1LENR_TIM5EN               (0x8UL)                   /*!< RCC APB1LENR: TIM5EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_TIM6EN_Pos           (4UL)                     /*!< RCC APB1LENR: TIM6EN (Bit 4)                          */
#define RCC_APB1LENR_TIM6EN               (0x10UL)                  /*!< RCC APB1LENR: TIM6EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_TIM7EN_Pos           (5UL)                     /*!< RCC APB1LENR: TIM7EN (Bit 5)                          */
#define RCC_APB1LENR_TIM7EN               (0x20UL)                  /*!< RCC APB1LENR: TIM7EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_TIM12EN_Pos          (6UL)                     /*!< RCC APB1LENR: TIM12EN (Bit 6)                         */
#define RCC_APB1LENR_TIM12EN              (0x40UL)                  /*!< RCC APB1LENR: TIM12EN (Bitfield-Mask: 0x01)           */
#define RCC_APB1LENR_TIM13EN_Pos          (7UL)                     /*!< RCC APB1LENR: TIM13EN (Bit 7)                         */
#define RCC_APB1LENR_TIM13EN              (0x80UL)                  /*!< RCC APB1LENR: TIM13EN (Bitfield-Mask: 0x01)           */
#define RCC_APB1LENR_TIM14EN_Pos          (8UL)                     /*!< RCC APB1LENR: TIM14EN (Bit 8)                         */
#define RCC_APB1LENR_TIM14EN              (0x100UL)                 /*!< RCC APB1LENR: TIM14EN (Bitfield-Mask: 0x01)           */
#define RCC_APB1LENR_LPTIM1EN_Pos         (9UL)                     /*!< RCC APB1LENR: LPTIM1EN (Bit 9)                        */
#define RCC_APB1LENR_LPTIM1EN             (0x200UL)                 /*!< RCC APB1LENR: LPTIM1EN (Bitfield-Mask: 0x01)          */
#define RCC_APB1LENR_SPI2EN_Pos           (14UL)                    /*!< RCC APB1LENR: SPI2EN (Bit 14)                         */
#define RCC_APB1LENR_SPI2EN               (0x4000UL)                /*!< RCC APB1LENR: SPI2EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_SPI3EN_Pos           (15UL)                    /*!< RCC APB1LENR: SPI3EN (Bit 15)                         */
#define RCC_APB1LENR_SPI3EN               (0x8000UL)                /*!< RCC APB1LENR: SPI3EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_SPDIFRXEN_Pos        (16UL)                    /*!< RCC APB1LENR: SPDIFRXEN (Bit 16)                      */
#define RCC_APB1LENR_SPDIFRXEN            (0x10000UL)               /*!< RCC APB1LENR: SPDIFRXEN (Bitfield-Mask: 0x01)         */
#define RCC_APB1LENR_USART2EN_Pos         (17UL)                    /*!< RCC APB1LENR: USART2EN (Bit 17)                       */
#define RCC_APB1LENR_USART2EN             (0x20000UL)               /*!< RCC APB1LENR: USART2EN (Bitfield-Mask: 0x01)          */
#define RCC_APB1LENR_USART3EN_Pos         (18UL)                    /*!< RCC APB1LENR: USART3EN (Bit 18)                       */
#define RCC_APB1LENR_USART3EN             (0x40000UL)               /*!< RCC APB1LENR: USART3EN (Bitfield-Mask: 0x01)          */
#define RCC_APB1LENR_UART4EN_Pos          (19UL)                    /*!< RCC APB1LENR: UART4EN (Bit 19)                        */
#define RCC_APB1LENR_UART4EN              (0x80000UL)               /*!< RCC APB1LENR: UART4EN (Bitfield-Mask: 0x01)           */
#define RCC_APB1LENR_UART5EN_Pos          (20UL)                    /*!< RCC APB1LENR: UART5EN (Bit 20)                        */
#define RCC_APB1LENR_UART5EN              (0x100000UL)              /*!< RCC APB1LENR: UART5EN (Bitfield-Mask: 0x01)           */
#define RCC_APB1LENR_I2C1EN_Pos           (21UL)                    /*!< RCC APB1LENR: I2C1EN (Bit 21)                         */
#define RCC_APB1LENR_I2C1EN               (0x200000UL)              /*!< RCC APB1LENR: I2C1EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_I2C2EN_Pos           (22UL)                    /*!< RCC APB1LENR: I2C2EN (Bit 22)                         */
#define RCC_APB1LENR_I2C2EN               (0x400000UL)              /*!< RCC APB1LENR: I2C2EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_I2C3EN_Pos           (23UL)                    /*!< RCC APB1LENR: I2C3EN (Bit 23)                         */
#define RCC_APB1LENR_I2C3EN               (0x800000UL)              /*!< RCC APB1LENR: I2C3EN (Bitfield-Mask: 0x01)            */
#define RCC_APB1LENR_HDMICECEN_Pos        (27UL)                    /*!< RCC APB1LENR: HDMICECEN (Bit 27)                      */
#define RCC_APB1LENR_HDMICECEN            (0x8000000UL)             /*!< RCC APB1LENR: HDMICECEN (Bitfield-Mask: 0x01)         */
#define RCC_APB1LENR_DAC12EN_Pos          (29UL)                    /*!< RCC APB1LENR: DAC12EN (Bit 29)                        */
#define RCC_APB1LENR_DAC12EN              (0x20000000UL)            /*!< RCC APB1LENR: DAC12EN (Bitfield-Mask: 0x01)           */
#define RCC_APB1LENR_USART7EN_Pos         (30UL)                    /*!< RCC APB1LENR: USART7EN (Bit 30)                       */
#define RCC_APB1LENR_USART7EN             (0x40000000UL)            /*!< RCC APB1LENR: USART7EN (Bitfield-Mask: 0x01)          */
#define RCC_APB1LENR_USART8EN_Pos         (31UL)                    /*!< RCC APB1LENR: USART8EN (Bit 31)                       */
#define RCC_APB1LENR_USART8EN             (0x80000000UL)            /*!< RCC APB1LENR: USART8EN (Bitfield-Mask: 0x01)          */

/* APB1 H Peripheral Clock enable register */

#define RCC_APB1HENR_CRSEN_Pos            (1UL)                     /*!< RCC APB1HENR: CRSEN (Bit 1)                           */
#define RCC_APB1HENR_CRSEN                (0x2UL)                   /*!< RCC APB1HENR: CRSEN (Bitfield-Mask: 0x01)             */
#define RCC_APB1HENR_SWPEN_Pos            (2UL)                     /*!< RCC APB1HENR: SWPEN (Bit 2)                           */
#define RCC_APB1HENR_SWPEN                (0x4UL)                   /*!< RCC APB1HENR: SWPEN (Bitfield-Mask: 0x01)             */
#define RCC_APB1HENR_OPAMPEN_Pos          (4UL)                     /*!< RCC APB1HENR: OPAMPEN (Bit 4)                         */
#define RCC_APB1HENR_OPAMPEN              (0x10UL)                  /*!< RCC APB1HENR: OPAMPEN (Bitfield-Mask: 0x01)           */
#define RCC_APB1HENR_MDIOSEN_Pos          (5UL)                     /*!< RCC APB1HENR: MDIOSEN (Bit 5)                         */
#define RCC_APB1HENR_MDIOSEN              (0x20UL)                  /*!< RCC APB1HENR: MDIOSEN (Bitfield-Mask: 0x01)           */
#define RCC_APB1HENR_FDCANEN_Pos          (8UL)                     /*!< RCC APB1HENR: FDCANEN (Bit 8)                         */
#define RCC_APB1HENR_FDCANEN              (0x100UL)                 /*!< RCC APB1HENR: FDCANEN (Bitfield-Mask: 0x01)           */

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_TIM1EN_Pos            (0UL)                     /*!< RCC APB2ENR: TIM1EN (Bit 0)                           */
#define RCC_APB2ENR_TIM1EN                (0x1UL)                   /*!< RCC APB2ENR: TIM1EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_TIM8EN_Pos            (1UL)                     /*!< RCC APB2ENR: TIM8EN (Bit 1)                           */
#define RCC_APB2ENR_TIM8EN                (0x2UL)                   /*!< RCC APB2ENR: TIM8EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_USART1EN_Pos          (4UL)                     /*!< RCC APB2ENR: USART1EN (Bit 4)                         */
#define RCC_APB2ENR_USART1EN              (0x10UL)                  /*!< RCC APB2ENR: USART1EN (Bitfield-Mask: 0x01)           */
#define RCC_APB2ENR_USART6EN_Pos          (5UL)                     /*!< RCC APB2ENR: USART6EN (Bit 5)                         */
#define RCC_APB2ENR_USART6EN              (0x20UL)                  /*!< RCC APB2ENR: USART6EN (Bitfield-Mask: 0x01)           */
#define RCC_APB2ENR_SPI1EN_Pos            (12UL)                    /*!< RCC APB2ENR: SPI1EN (Bit 12)                          */
#define RCC_APB2ENR_SPI1EN                (0x1000UL)                /*!< RCC APB2ENR: SPI1EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_SPI4EN_Pos            (13UL)                    /*!< RCC APB2ENR: SPI4EN (Bit 13)                          */
#define RCC_APB2ENR_SPI4EN                (0x2000UL)                /*!< RCC APB2ENR: SPI4EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_TIM16EN_Pos           (17UL)                    /*!< RCC APB2ENR: TIM16EN (Bit 17)                         */
#define RCC_APB2ENR_TIM16EN               (0x20000UL)               /*!< RCC APB2ENR: TIM16EN (Bitfield-Mask: 0x01)            */
#define RCC_APB2ENR_TIM15EN_Pos           (16UL)                    /*!< RCC APB2ENR: TIM15EN (Bit 16)                         */
#define RCC_APB2ENR_TIM15EN               (0x10000UL)               /*!< RCC APB2ENR: TIM15EN (Bitfield-Mask: 0x01)            */
#define RCC_APB2ENR_TIM17EN_Pos           (18UL)                    /*!< RCC APB2ENR: TIM17EN (Bit 18)                         */
#define RCC_APB2ENR_TIM17EN               (0x40000UL)               /*!< RCC APB2ENR: TIM17EN (Bitfield-Mask: 0x01)            */
#define RCC_APB2ENR_SPI5EN_Pos            (20UL)                    /*!< RCC APB2ENR: SPI5EN (Bit 20)                          */
#define RCC_APB2ENR_SPI5EN                (0x100000UL)              /*!< RCC APB2ENR: SPI5EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_SAI1EN_Pos            (22UL)                    /*!< RCC APB2ENR: SAI1EN (Bit 22)                          */
#define RCC_APB2ENR_SAI1EN                (0x400000UL)              /*!< RCC APB2ENR: SAI1EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_SAI2EN_Pos            (23UL)                    /*!< RCC APB2ENR: SAI2EN (Bit 23)                          */
#define RCC_APB2ENR_SAI2EN                (0x800000UL)              /*!< RCC APB2ENR: SAI2EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_SAI3EN_Pos            (24UL)                    /*!< RCC APB2ENR: SAI3EN (Bit 24)                          */
#define RCC_APB2ENR_SAI3EN                (0x1000000UL)             /*!< RCC APB2ENR: SAI3EN (Bitfield-Mask: 0x01)             */
#define RCC_APB2ENR_DFSDM1EN_Pos          (28UL)                    /*!< RCC APB2ENR: DFSDM1EN (Bit 28)                        */
#define RCC_APB2ENR_DFSDM1EN              (0x10000000UL)            /*!< RCC APB2ENR: DFSDM1EN (Bitfield-Mask: 0x01)           */
#define RCC_APB2ENR_HRTIMEN_Pos           (29UL)                    /*!< RCC APB2ENR: HRTIMEN (Bit 29)                         */
#define RCC_APB2ENR_HRTIMEN               (0x20000000UL)            /*!< RCC APB2ENR: HRTIMEN (Bitfield-Mask: 0x01)            */

/* APB4 Peripheral Clock enable register */

#define RCC_APB4ENR_SYSCFGEN_Pos          (1UL)                     /*!< RCC APB4ENR: SYSCFGEN (Bit 1)                         */
#define RCC_APB4ENR_SYSCFGEN              (0x2UL)                   /*!< RCC APB4ENR: SYSCFGEN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_LPUART1EN_Pos         (3UL)                     /*!< RCC APB4ENR: LPUART1EN (Bit 3)                        */
#define RCC_APB4ENR_LPUART1EN             (0x8UL)                   /*!< RCC APB4ENR: LPUART1EN (Bitfield-Mask: 0x01)          */
#define RCC_APB4ENR_SPI6EN_Pos            (5UL)                     /*!< RCC APB4ENR: SPI6EN (Bit 5)                           */
#define RCC_APB4ENR_SPI6EN                (0x20UL)                  /*!< RCC APB4ENR: SPI6EN (Bitfield-Mask: 0x01)             */
#define RCC_APB4ENR_I2C4EN_Pos            (7UL)                     /*!< RCC APB4ENR: I2C4EN (Bit 7)                           */
#define RCC_APB4ENR_I2C4EN                (0x80UL)                  /*!< RCC APB4ENR: I2C4EN (Bitfield-Mask: 0x01)             */
#define RCC_APB4ENR_LPTIM2EN_Pos          (9UL)                     /*!< RCC APB4ENR: LPTIM2EN (Bit 9)                         */
#define RCC_APB4ENR_LPTIM2EN              (0x200UL)                 /*!< RCC APB4ENR: LPTIM2EN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_LPTIM3EN_Pos          (10UL)                    /*!< RCC APB4ENR: LPTIM3EN (Bit 10)                        */
#define RCC_APB4ENR_LPTIM3EN              (0x400UL)                 /*!< RCC APB4ENR: LPTIM3EN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_LPTIM4EN_Pos          (11UL)                    /*!< RCC APB4ENR: LPTIM4EN (Bit 11)                        */
#define RCC_APB4ENR_LPTIM4EN              (0x800UL)                 /*!< RCC APB4ENR: LPTIM4EN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_LPTIM5EN_Pos          (12UL)                    /*!< RCC APB4ENR: LPTIM5EN (Bit 12)                        */
#define RCC_APB4ENR_LPTIM5EN              (0x1000UL)                /*!< RCC APB4ENR: LPTIM5EN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_COMP12EN_Pos          (14UL)                    /*!< RCC APB4ENR: COMP12EN (Bit 14)                        */
#define RCC_APB4ENR_COMP12EN              (0x4000UL)                /*!< RCC APB4ENR: COMP12EN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_VREFEN_Pos            (15UL)                    /*!< RCC APB4ENR: VREFEN (Bit 15)                          */
#define RCC_APB4ENR_VREFEN                (0x8000UL)                /*!< RCC APB4ENR: VREFEN (Bitfield-Mask: 0x01)             */
#define RCC_APB4ENR_RTCAPBEN_Pos          (16UL)                    /*!< RCC APB4ENR: RTCAPBEN (Bit 16)                        */
#define RCC_APB4ENR_RTCAPBEN              (0x10000UL)               /*!< RCC APB4ENR: RTCAPBEN (Bitfield-Mask: 0x01)           */
#define RCC_APB4ENR_SAI4EN_Pos            (21UL)                    /*!< RCC APB4ENR: SAI4EN (Bit 21)                          */
#define RCC_APB4ENR_SAI4EN                (0x200000UL)              /*!< RCC APB4ENR: SAI4EN (Bitfield-Mask: 0x01)             */

/* AHB3 low power mode peripheral clock enable register */

#define RCC_AHB3LPENR_MDMALPEN_Pos        (0UL)                     /*!< RCC AHB3LPENR: MDMALPEN (Bit 0)                       */
#define RCC_AHB3LPENR_MDMALPEN            (0x1UL)                   /*!< RCC AHB3LPENR: MDMALPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB3LPENR_DMA2DLPEN_Pos       (4UL)                     /*!< RCC AHB3LPENR: DMA2DLPEN (Bit 4)                      */
#define RCC_AHB3LPENR_DMA2DLPEN           (0x10UL)                  /*!< RCC AHB3LPENR: DMA2DLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB3LPENR_JPGDECLPEN_Pos      (5UL)                     /*!< RCC AHB3LPENR: JPGDECLPEN (Bit 5)                     */
#define RCC_AHB3LPENR_JPGDECLPEN          (0x20UL)                  /*!< RCC AHB3LPENR: JPGDECLPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB3LPENR_FLITFLPEN_Pos       (8UL)                     /*!< RCC AHB3LPENR: FLITFLPEN (Bit 8)                      */
#define RCC_AHB3LPENR_FLITFLPEN           (0x100UL)                 /*!< RCC AHB3LPENR: FLITFLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB3LPENR_FMCLPEN_Pos         (12UL)                    /*!< RCC AHB3LPENR: FMCLPEN (Bit 12)                       */
#define RCC_AHB3LPENR_FMCLPEN             (0x1000UL)                /*!< RCC AHB3LPENR: FMCLPEN (Bitfield-Mask: 0x01)          */
#define RCC_AHB3LPENR_QSPILPEN_Pos        (14UL)                    /*!< RCC AHB3LPENR: QSPILPEN (Bit 14)                      */
#define RCC_AHB3LPENR_QSPILPEN            (0x4000UL)                /*!< RCC AHB3LPENR: QSPILPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB3LPENR_SDMMC1LPEN_Pos      (16UL)                    /*!< RCC AHB3LPENR: SDMMC1LPEN (Bit 16)                    */
#define RCC_AHB3LPENR_SDMMC1LPEN          (0x10000UL)               /*!< RCC AHB3LPENR: SDMMC1LPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB3LPENR_D1DTCM1LPEN_Pos     (28UL)                    /*!< RCC AHB3LPENR: D1DTCM1LPEN (Bit 28)                   */
#define RCC_AHB3LPENR_D1DTCM1LPEN         (0x10000000UL)            /*!< RCC AHB3LPENR: D1DTCM1LPEN (Bitfield-Mask: 0x01)      */
#define RCC_AHB3LPENR_DTCM2LPEN_Pos       (29UL)                    /*!< RCC AHB3LPENR: DTCM2LPEN (Bit 29)                     */
#define RCC_AHB3LPENR_DTCM2LPEN           (0x20000000UL)            /*!< RCC AHB3LPENR: DTCM2LPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB3LPENR_ITCMLPEN_Pos        (30UL)                    /*!< RCC AHB3LPENR: ITCMLPEN (Bit 30)                      */
#define RCC_AHB3LPENR_ITCMLPEN            (0x40000000UL)            /*!< RCC AHB3LPENR: ITCMLPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB3LPENR_AXISRAMLPEN_Pos     (31UL)                    /*!< RCC AHB3LPENR: AXISRAMLPEN (Bit 31)                   */
#define RCC_AHB3LPENR_AXISRAMLPEN         (0x80000000UL)            /*!< RCC AHB3LPENR: AXISRAMLPEN (Bitfield-Mask: 0x01)      */

/* AHB1 low power mode peripheral clock enable register */

#define RCC_AHB1LPENR_DMA1LPEN_Pos        (0UL)                     /*!< RCC AHB1LPENR: DMA1LPEN (Bit 0)                       */
#define RCC_AHB1LPENR_DMA1LPEN            (0x1UL)                   /*!< RCC AHB1LPENR: DMA1LPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB1LPENR_DMA2LPEN_Pos        (1UL)                     /*!< RCC AHB1LPENR: DMA2LPEN (Bit 1)                       */
#define RCC_AHB1LPENR_DMA2LPEN            (0x2UL)                   /*!< RCC AHB1LPENR: DMA2LPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB1LPENR_ADC12LPEN_Pos       (5UL)                     /*!< RCC AHB1LPENR: ADC12LPEN (Bit 5)                      */
#define RCC_AHB1LPENR_ADC12LPEN           (0x20UL)                  /*!< RCC AHB1LPENR: ADC12LPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB1LPENR_ETH1MACLPEN_Pos     (15UL)                    /*!< RCC AHB1LPENR: ETH1MACLPEN (Bit 15)                   */
#define RCC_AHB1LPENR_ETH1MACLPEN         (0x8000UL)                /*!< RCC AHB1LPENR: ETH1MACLPEN (Bitfield-Mask: 0x01)      */
#define RCC_AHB1LPENR_ETH1TXLPEN_Pos      (16UL)                    /*!< RCC AHB1LPENR: ETH1TXLPEN (Bit 16)                    */
#define RCC_AHB1LPENR_ETH1TXLPEN          (0x10000UL)               /*!< RCC AHB1LPENR: ETH1TXLPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB1LPENR_ETH1RXLPEN_Pos      (17UL)                    /*!< RCC AHB1LPENR: ETH1RXLPEN (Bit 17)                    */
#define RCC_AHB1LPENR_ETH1RXLPEN          (0x20000UL)               /*!< RCC AHB1LPENR: ETH1RXLPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB1LPENR_USB1OTGLPEN_Pos     (25UL)                    /*!< RCC AHB1LPENR: USB1OTGLPEN (Bit 25)                   */
#define RCC_AHB1LPENR_USB1OTGLPEN         (0x2000000UL)             /*!< RCC AHB1LPENR: USB1OTGLPEN (Bitfield-Mask: 0x01)      */
#define RCC_AHB1LPENR_USB1ULPILPEN_Pos    (26UL)                    /*!< RCC AHB1LPENR: USB1ULPILPEN (Bit 26)                  */
#define RCC_AHB1LPENR_USB1ULPILPEN        (0x4000000UL)             /*!< RCC AHB1LPENR: USB1ULPILPEN (Bitfield-Mask: 0x01)     */
#define RCC_AHB1LPENR_USB2OTGLPEN_Pos     (27UL)                    /*!< RCC AHB1LPENR: USB2OTGLPEN (Bit 27)                   */
#define RCC_AHB1LPENR_USB2OTGLPEN         (0x8000000UL)             /*!< RCC AHB1LPENR: USB2OTGLPEN (Bitfield-Mask: 0x01)      */
#define RCC_AHB1LPENR_USB2ULPILPEN_Pos    (28UL)                    /*!< RCC AHB1LPENR: USB2ULPILPEN (Bit 28)                  */
#define RCC_AHB1LPENR_USB2ULPILPEN        (0x10000000UL)            /*!< RCC AHB1LPENR: USB2ULPILPEN (Bitfield-Mask: 0x01)     */


/* AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2LPENR_CAMITFLPEN_Pos      (0UL)                     /*!< RCC AHB2LPENR: CAMITFLPEN (Bit 0)                     */
#define RCC_AHB2LPENR_CAMITFLPEN          (0x1UL)                   /*!< RCC AHB2LPENR: CAMITFLPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB2LPENR_CRYPTLPEN_Pos       (4UL)                     /*!< RCC AHB2LPENR: CRYPTLPEN (Bit 4)                      */
#define RCC_AHB2LPENR_CRYPTLPEN           (0x10UL)                  /*!< RCC AHB2LPENR: CRYPTLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB2LPENR_HASHLPEN_Pos        (5UL)                     /*!< RCC AHB2LPENR: HASHLPEN (Bit 5)                       */
#define RCC_AHB2LPENR_HASHLPEN            (0x20UL)                  /*!< RCC AHB2LPENR: HASHLPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB2LPENR_SDMMC2LPEN_Pos      (9UL)                     /*!< RCC AHB2LPENR: SDMMC2LPEN (Bit 9)                     */
#define RCC_AHB2LPENR_SDMMC2LPEN          (0x200UL)                 /*!< RCC AHB2LPENR: SDMMC2LPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB2LPENR_RNGLPEN_Pos         (6UL)                     /*!< RCC AHB2LPENR: RNGLPEN (Bit 6)                        */
#define RCC_AHB2LPENR_RNGLPEN             (0x40UL)                  /*!< RCC AHB2LPENR: RNGLPEN (Bitfield-Mask: 0x01)          */
#define RCC_AHB2LPENR_SRAM1LPEN_Pos       (29UL)                    /*!< RCC AHB2LPENR: SRAM1LPEN (Bit 29)                     */
#define RCC_AHB2LPENR_SRAM1LPEN           (0x20000000UL)            /*!< RCC AHB2LPENR: SRAM1LPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB2LPENR_SRAM2LPEN_Pos       (30UL)                    /*!< RCC AHB2LPENR: SRAM2LPEN (Bit 30)                     */
#define RCC_AHB2LPENR_SRAM2LPEN           (0x40000000UL)            /*!< RCC AHB2LPENR: SRAM2LPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB2LPENR_SRAM3LPEN_Pos       (31UL)                    /*!< RCC AHB2LPENR: SRAM3LPEN (Bit 31)                     */
#define RCC_AHB2LPENR_SRAM3LPEN           (0x80000000UL)            /*!< RCC AHB2LPENR: SRAM3LPEN (Bitfield-Mask: 0x01)        */

/* AHB4 low power mode peripheral clock enable register*/

#define RCC_AHB4LPENR_GPIOALPEN_Pos       (0UL)                     /*!< RCC AHB4LPENR: GPIOALPEN (Bit 0)                      */
#define RCC_AHB4LPENR_GPIOALPEN           (0x1UL)                   /*!< RCC AHB4LPENR: GPIOALPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOBLPEN_Pos       (1UL)                     /*!< RCC AHB4LPENR: GPIOBLPEN (Bit 1)                      */
#define RCC_AHB4LPENR_GPIOBLPEN           (0x2UL)                   /*!< RCC AHB4LPENR: GPIOBLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOCLPEN_Pos       (2UL)                     /*!< RCC AHB4LPENR: GPIOCLPEN (Bit 2)                      */
#define RCC_AHB4LPENR_GPIOCLPEN           (0x4UL)                   /*!< RCC AHB4LPENR: GPIOCLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIODLPEN_Pos       (3UL)                     /*!< RCC AHB4LPENR: GPIODLPEN (Bit 3)                      */
#define RCC_AHB4LPENR_GPIODLPEN           (0x8UL)                   /*!< RCC AHB4LPENR: GPIODLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOELPEN_Pos       (4UL)                     /*!< RCC AHB4LPENR: GPIOELPEN (Bit 4)                      */
#define RCC_AHB4LPENR_GPIOELPEN           (0x10UL)                  /*!< RCC AHB4LPENR: GPIOELPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOFLPEN_Pos       (5UL)                     /*!< RCC AHB4LPENR: GPIOFLPEN (Bit 5)                      */
#define RCC_AHB4LPENR_GPIOFLPEN           (0x20UL)                  /*!< RCC AHB4LPENR: GPIOFLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOGLPEN_Pos       (6UL)                     /*!< RCC AHB4LPENR: GPIOGLPEN (Bit 6)                      */
#define RCC_AHB4LPENR_GPIOGLPEN           (0x40UL)                  /*!< RCC AHB4LPENR: GPIOGLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOHLPEN_Pos       (7UL)                     /*!< RCC AHB4LPENR: GPIOHLPEN (Bit 7)                      */
#define RCC_AHB4LPENR_GPIOHLPEN           (0x80UL)                  /*!< RCC AHB4LPENR: GPIOHLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOILPEN_Pos       (8UL)                     /*!< RCC AHB4LPENR: GPIOILPEN (Bit 8)                      */
#define RCC_AHB4LPENR_GPIOILPEN           (0x100UL)                 /*!< RCC AHB4LPENR: GPIOILPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOJLPEN_Pos       (9UL)                     /*!< RCC AHB4LPENR: GPIOJLPEN (Bit 9)                      */
#define RCC_AHB4LPENR_GPIOJLPEN           (0x200UL)                 /*!< RCC AHB4LPENR: GPIOJLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_GPIOKLPEN_Pos       (10UL)                    /*!< RCC AHB4LPENR: GPIOKLPEN (Bit 10)                     */
#define RCC_AHB4LPENR_GPIOKLPEN           (0x400UL)                 /*!< RCC AHB4LPENR: GPIOKLPEN (Bitfield-Mask: 0x01)        */
#define RCC_AHB4LPENR_CRCLPEN_Pos         (19UL)                    /*!< RCC AHB4LPENR: CRCLPEN (Bit 19)                       */
#define RCC_AHB4LPENR_CRCLPEN             (0x80000UL)               /*!< RCC AHB4LPENR: CRCLPEN (Bitfield-Mask: 0x01)          */
#define RCC_AHB4LPENR_BDMALPEN_Pos        (21UL)                    /*!< RCC AHB4LPENR: BDMALPEN (Bit 21)                      */
#define RCC_AHB4LPENR_BDMALPEN            (0x200000UL)              /*!< RCC AHB4LPENR: BDMALPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB4LPENR_ADC3LPEN_Pos        (24UL)                    /*!< RCC AHB4LPENR: ADC3LPEN (Bit 24)                      */
#define RCC_AHB4LPENR_ADC3LPEN            (0x1000000UL)             /*!< RCC AHB4LPENR: ADC3LPEN (Bitfield-Mask: 0x01)         */
#define RCC_AHB4LPENR_BKPRAMLPEN_Pos      (28UL)                    /*!< RCC AHB4LPENR: BKPRAMLPEN (Bit 28)                    */
#define RCC_AHB4LPENR_BKPRAMLPEN          (0x10000000UL)            /*!< RCC AHB4LPENR: BKPRAMLPEN (Bitfield-Mask: 0x01)       */
#define RCC_AHB4LPENR_SRAM4LPEN_Pos       (29UL)                    /*!< RCC AHB4LPENR: SRAM4LPEN (Bit 29)                     */
#define RCC_AHB4LPENR_SRAM4LPEN           (0x20000000UL)            /*!< RCC AHB4LPENR: SRAM4LPEN (Bitfield-Mask: 0x01)        */

/* APB3 low power mode peripheral clock enable register */

#define RCC_APB3LPENR_LTDCLPEN_Pos        (3UL)                     /*!< RCC APB3LPENR: LTDCLPEN (Bit 3)                       */
#define RCC_APB3LPENR_LTDCLPEN            (0x8UL)                   /*!< RCC APB3LPENR: LTDCLPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB3LPENR_WWDG1LPEN_Pos       (6UL)                     /*!< RCC APB3LPENR: WWDG1LPEN (Bit 6)                      */
#define RCC_APB3LPENR_WWDG1LPEN           (0x40UL)                  /*!< RCC APB3LPENR: WWDG1LPEN (Bitfield-Mask: 0x01)        */

/* APB1 L low power mode peripheral clock enable register */

#define RCC_APB1LLPENR_TIM2LPEN_Pos       (0UL)                     /*!< RCC APB1LLPENR: TIM2LPEN (Bit 0)                      */
#define RCC_APB1LLPENR_TIM2LPEN           (0x1UL)                   /*!< RCC APB1LLPENR: TIM2LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_TIM3LPEN_Pos       (1UL)                     /*!< RCC APB1LLPENR: TIM3LPEN (Bit 1)                      */
#define RCC_APB1LLPENR_TIM3LPEN           (0x2UL)                   /*!< RCC APB1LLPENR: TIM3LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_TIM4LPEN_Pos       (2UL)                     /*!< RCC APB1LLPENR: TIM4LPEN (Bit 2)                      */
#define RCC_APB1LLPENR_TIM4LPEN           (0x4UL)                   /*!< RCC APB1LLPENR: TIM4LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_TIM5LPEN_Pos       (3UL)                     /*!< RCC APB1LLPENR: TIM5LPEN (Bit 3)                      */
#define RCC_APB1LLPENR_TIM5LPEN           (0x8UL)                   /*!< RCC APB1LLPENR: TIM5LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_TIM6LPEN_Pos       (4UL)                     /*!< RCC APB1LLPENR: TIM6LPEN (Bit 4)                      */
#define RCC_APB1LLPENR_TIM6LPEN           (0x10UL)                  /*!< RCC APB1LLPENR: TIM6LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_TIM7LPEN_Pos       (5UL)                     /*!< RCC APB1LLPENR: TIM7LPEN (Bit 5)                      */
#define RCC_APB1LLPENR_TIM7LPEN           (0x20UL)                  /*!< RCC APB1LLPENR: TIM7LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_TIM12LPEN_Pos      (6UL)                     /*!< RCC APB1LLPENR: TIM12LPEN (Bit 6)                     */
#define RCC_APB1LLPENR_TIM12LPEN          (0x40UL)                  /*!< RCC APB1LLPENR: TIM12LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1LLPENR_TIM13LPEN_Pos      (7UL)                     /*!< RCC APB1LLPENR: TIM13LPEN (Bit 7)                     */
#define RCC_APB1LLPENR_TIM13LPEN          (0x80UL)                  /*!< RCC APB1LLPENR: TIM13LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1LLPENR_TIM14LPEN_Pos      (8UL)                     /*!< RCC APB1LLPENR: TIM14LPEN (Bit 8)                     */
#define RCC_APB1LLPENR_TIM14LPEN          (0x100UL)                 /*!< RCC APB1LLPENR: TIM14LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1LLPENR_LPTIM1LPEN_Pos     (9UL)                     /*!< RCC APB1LLPENR: LPTIM1LPEN (Bit 9)                    */
#define RCC_APB1LLPENR_LPTIM1LPEN         (0x200UL)                 /*!< RCC APB1LLPENR: LPTIM1LPEN (Bitfield-Mask: 0x01)      */
#define RCC_APB1LLPENR_SPI2LPEN_Pos       (14UL)                    /*!< RCC APB1LLPENR: SPI2LPEN (Bit 14)                     */
#define RCC_APB1LLPENR_SPI2LPEN           (0x4000UL)                /*!< RCC APB1LLPENR: SPI2LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_SPI3LPEN_Pos       (15UL)                    /*!< RCC APB1LLPENR: SPI3LPEN (Bit 15)                     */
#define RCC_APB1LLPENR_SPI3LPEN           (0x8000UL)                /*!< RCC APB1LLPENR: SPI3LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_SPDIFRXLPEN_Pos    (16UL)                    /*!< RCC APB1LLPENR: SPDIFRXLPEN (Bit 16)                  */
#define RCC_APB1LLPENR_SPDIFRXLPEN        (0x10000UL)               /*!< RCC APB1LLPENR: SPDIFRXLPEN (Bitfield-Mask: 0x01)     */
#define RCC_APB1LLPENR_USART2LPEN_Pos     (17UL)                    /*!< RCC APB1LLPENR: USART2LPEN (Bit 17)                   */
#define RCC_APB1LLPENR_USART2LPEN         (0x20000UL)               /*!< RCC APB1LLPENR: USART2LPEN (Bitfield-Mask: 0x01)      */
#define RCC_APB1LLPENR_USART3LPEN_Pos     (18UL)                    /*!< RCC APB1LLPENR: USART3LPEN (Bit 18)                   */
#define RCC_APB1LLPENR_USART3LPEN         (0x40000UL)               /*!< RCC APB1LLPENR: USART3LPEN (Bitfield-Mask: 0x01)      */
#define RCC_APB1LLPENR_UART4LPEN_Pos      (19UL)                    /*!< RCC APB1LLPENR: UART4LPEN (Bit 19)                    */
#define RCC_APB1LLPENR_UART4LPEN          (0x80000UL)               /*!< RCC APB1LLPENR: UART4LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1LLPENR_UART5LPEN_Pos      (20UL)                    /*!< RCC APB1LLPENR: UART5LPEN (Bit 20)                    */
#define RCC_APB1LLPENR_UART5LPEN          (0x100000UL)              /*!< RCC APB1LLPENR: UART5LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1LLPENR_I2C1LPEN_Pos       (21UL)                    /*!< RCC APB1LLPENR: I2C1LPEN (Bit 21)                     */
#define RCC_APB1LLPENR_I2C1LPEN           (0x200000UL)              /*!< RCC APB1LLPENR: I2C1LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_I2C2LPEN_Pos       (22UL)                    /*!< RCC APB1LLPENR: I2C2LPEN (Bit 22)                     */
#define RCC_APB1LLPENR_I2C2LPEN           (0x400000UL)              /*!< RCC APB1LLPENR: I2C2LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_I2C3LPEN_Pos       (23UL)                    /*!< RCC APB1LLPENR: I2C3LPEN (Bit 23)                     */
#define RCC_APB1LLPENR_I2C3LPEN           (0x800000UL)              /*!< RCC APB1LLPENR: I2C3LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB1LLPENR_HDMICECLPEN_Pos    (27UL)                    /*!< RCC APB1LLPENR: HDMICECLPEN (Bit 27)                  */
#define RCC_APB1LLPENR_HDMICECLPEN        (0x8000000UL)             /*!< RCC APB1LLPENR: HDMICECLPEN (Bitfield-Mask: 0x01)     */
#define RCC_APB1LLPENR_DAC12LPEN_Pos      (29UL)                    /*!< RCC APB1LLPENR: DAC12LPEN (Bit 29)                    */
#define RCC_APB1LLPENR_DAC12LPEN          (0x20000000UL)            /*!< RCC APB1LLPENR: DAC12LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1LLPENR_USART7LPEN_Pos     (30UL)                    /*!< RCC APB1LLPENR: USART7LPEN (Bit 30)                   */
#define RCC_APB1LLPENR_USART7LPEN         (0x40000000UL)            /*!< RCC APB1LLPENR: USART7LPEN (Bitfield-Mask: 0x01)      */
#define RCC_APB1LLPENR_USART8LPEN_Pos     (31UL)                    /*!< RCC APB1LLPENR: USART8LPEN (Bit 31)                   */
#define RCC_APB1LLPENR_USART8LPEN         (0x80000000UL)            /*!< RCC APB1LLPENR: USART8LPEN (Bitfield-Mask: 0x01)      */


/* APB1 H low power mode peripheral clock enable register */

#define RCC_APB1HLPENR_CRSLPEN_Pos        (1UL)                     /*!< RCC APB1HLPENR: CRSLPEN (Bit 1)                       */
#define RCC_APB1HLPENR_CRSLPEN            (0x2UL)                   /*!< RCC APB1HLPENR: CRSLPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB1HLPENR_SWPLPEN_Pos        (2UL)                     /*!< RCC APB1HLPENR: SWPLPEN (Bit 2)                       */
#define RCC_APB1HLPENR_SWPLPEN            (0x4UL)                   /*!< RCC APB1HLPENR: SWPLPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB1HLPENR_OPAMPLPEN_Pos      (4UL)                     /*!< RCC APB1HLPENR: OPAMPLPEN (Bit 4)                     */
#define RCC_APB1HLPENR_OPAMPLPEN          (0x10UL)                  /*!< RCC APB1HLPENR: OPAMPLPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1HLPENR_MDIOSLPEN_Pos      (5UL)                     /*!< RCC APB1HLPENR: MDIOSLPEN (Bit 5)                     */
#define RCC_APB1HLPENR_MDIOSLPEN          (0x20UL)                  /*!< RCC APB1HLPENR: MDIOSLPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB1HLPENR_FDCANLPEN_Pos      (8UL)                     /*!< RCC APB1HLPENR: FDCANLPEN (Bit 8)                     */
#define RCC_APB1HLPENR_FDCANLPEN          (0x100UL)                 /*!< RCC APB1HLPENR: FDCANLPEN (Bitfield-Mask: 0x01)       */

/* APB2 low power mode peripheral clock enable register */

#define RCC_APB2LPENR_TIM1LPEN_Pos        (0UL)                     /*!< RCC APB2LPENR: TIM1LPEN (Bit 0)                       */
#define RCC_APB2LPENR_TIM1LPEN            (0x1UL)                   /*!< RCC APB2LPENR: TIM1LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_TIM8LPEN_Pos        (1UL)                     /*!< RCC APB2LPENR: TIM8LPEN (Bit 1)                       */
#define RCC_APB2LPENR_TIM8LPEN            (0x2UL)                   /*!< RCC APB2LPENR: TIM8LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_USART1LPEN_Pos      (4UL)                     /*!< RCC APB2LPENR: USART1LPEN (Bit 4)                     */
#define RCC_APB2LPENR_USART1LPEN          (0x10UL)                  /*!< RCC APB2LPENR: USART1LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB2LPENR_USART6LPEN_Pos      (5UL)                     /*!< RCC APB2LPENR: USART6LPEN (Bit 5)                     */
#define RCC_APB2LPENR_USART6LPEN          (0x20UL)                  /*!< RCC APB2LPENR: USART6LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB2LPENR_SPI1LPEN_Pos        (12UL)                    /*!< RCC APB2LPENR: SPI1LPEN (Bit 12)                      */
#define RCC_APB2LPENR_SPI1LPEN            (0x1000UL)                /*!< RCC APB2LPENR: SPI1LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_SPI4LPEN_Pos        (13UL)                    /*!< RCC APB2LPENR: SPI4LPEN (Bit 13)                      */
#define RCC_APB2LPENR_SPI4LPEN            (0x2000UL)                /*!< RCC APB2LPENR: SPI4LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_TIM15LPEN_Pos       (16UL)                    /*!< RCC APB2LPENR: TIM15LPEN (Bit 16)                     */
#define RCC_APB2LPENR_TIM15LPEN           (0x10000UL)               /*!< RCC APB2LPENR: TIM15LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB2LPENR_TIM16LPEN_Pos       (17UL)                    /*!< RCC APB2LPENR: TIM16LPEN (Bit 17)                     */
#define RCC_APB2LPENR_TIM16LPEN           (0x20000UL)               /*!< RCC APB2LPENR: TIM16LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB2LPENR_TIM17LPEN_Pos       (18UL)                    /*!< RCC APB2LPENR: TIM17LPEN (Bit 18)                     */
#define RCC_APB2LPENR_TIM17LPEN           (0x40000UL)               /*!< RCC APB2LPENR: TIM17LPEN (Bitfield-Mask: 0x01)        */
#define RCC_APB2LPENR_SPI5LPEN_Pos        (20UL)                    /*!< RCC APB2LPENR: SPI5LPEN (Bit 20)                      */
#define RCC_APB2LPENR_SPI5LPEN            (0x100000UL)              /*!< RCC APB2LPENR: SPI5LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_SAI1LPEN_Pos        (22UL)                    /*!< RCC APB2LPENR: SAI1LPEN (Bit 22)                      */
#define RCC_APB2LPENR_SAI1LPEN            (0x400000UL)              /*!< RCC APB2LPENR: SAI1LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_SAI2LPEN_Pos        (23UL)                    /*!< RCC APB2LPENR: SAI2LPEN (Bit 23)                      */
#define RCC_APB2LPENR_SAI2LPEN            (0x800000UL)              /*!< RCC APB2LPENR: SAI2LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_SAI3LPEN_Pos        (24UL)                    /*!< RCC APB2LPENR: SAI3LPEN (Bit 24)                      */
#define RCC_APB2LPENR_SAI3LPEN            (0x1000000UL)             /*!< RCC APB2LPENR: SAI3LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB2LPENR_DFSDM1LPEN_Pos      (28UL)                    /*!< RCC APB2LPENR: DFSDM1LPEN (Bit 28)                    */
#define RCC_APB2LPENR_DFSDM1LPEN          (0x10000000UL)            /*!< RCC APB2LPENR: DFSDM1LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB2LPENR_HRTIMLPEN_Pos       (29UL)                    /*!< RCC APB2LPENR: HRTIMLPEN (Bit 29)                     */
#define RCC_APB2LPENR_HRTIMLPEN           (0x20000000UL)            /*!< RCC APB2LPENR: HRTIMLPEN (Bitfield-Mask: 0x01)        */

/* APB4 low power mode peripheral clock enable register */

#define RCC_APB4LPENR_SYSCFGLPEN_Pos      (1UL)                     /*!< RCC APB4LPENR: SYSCFGLPEN (Bit 1)                     */
#define RCC_APB4LPENR_SYSCFGLPEN          (0x2UL)                   /*!< RCC APB4LPENR: SYSCFGLPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_LPUART1LPEN_Pos     (3UL)                     /*!< RCC APB4LPENR: LPUART1LPEN (Bit 3)                    */
#define RCC_APB4LPENR_LPUART1LPEN         (0x8UL)                   /*!< RCC APB4LPENR: LPUART1LPEN (Bitfield-Mask: 0x01)      */
#define RCC_APB4LPENR_SPI6LPEN_Pos        (5UL)                     /*!< RCC APB4LPENR: SPI6LPEN (Bit 5)                       */
#define RCC_APB4LPENR_SPI6LPEN            (0x20UL)                  /*!< RCC APB4LPENR: SPI6LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB4LPENR_I2C4LPEN_Pos        (7UL)                     /*!< RCC APB4LPENR: I2C4LPEN (Bit 7)                       */
#define RCC_APB4LPENR_I2C4LPEN            (0x80UL)                  /*!< RCC APB4LPENR: I2C4LPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB4LPENR_LPTIM2LPEN_Pos      (9UL)                     /*!< RCC APB4LPENR: LPTIM2LPEN (Bit 9)                     */
#define RCC_APB4LPENR_LPTIM2LPEN          (0x200UL)                 /*!< RCC APB4LPENR: LPTIM2LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_LPTIM3LPEN_Pos      (10UL)                    /*!< RCC APB4LPENR: LPTIM3LPEN (Bit 10)                    */
#define RCC_APB4LPENR_LPTIM3LPEN          (0x400UL)                 /*!< RCC APB4LPENR: LPTIM3LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_LPTIM4LPEN_Pos      (11UL)                    /*!< RCC APB4LPENR: LPTIM4LPEN (Bit 11)                    */
#define RCC_APB4LPENR_LPTIM4LPEN          (0x800UL)                 /*!< RCC APB4LPENR: LPTIM4LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_LPTIM5LPEN_Pos      (12UL)                    /*!< RCC APB4LPENR: LPTIM5LPEN (Bit 12)                    */
#define RCC_APB4LPENR_LPTIM5LPEN          (0x1000UL)                /*!< RCC APB4LPENR: LPTIM5LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_COMP12LPEN_Pos      (14UL)                    /*!< RCC APB4LPENR: COMP12LPEN (Bit 14)                    */
#define RCC_APB4LPENR_COMP12LPEN          (0x4000UL)                /*!< RCC APB4LPENR: COMP12LPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_VREFLPEN_Pos        (15UL)                    /*!< RCC APB4LPENR: VREFLPEN (Bit 15)                      */
#define RCC_APB4LPENR_VREFLPEN            (0x8000UL)                /*!< RCC APB4LPENR: VREFLPEN (Bitfield-Mask: 0x01)         */
#define RCC_APB4LPENR_RTCAPBLPEN_Pos      (16UL)                    /*!< RCC APB4LPENR: RTCAPBLPEN (Bit 16)                    */
#define RCC_APB4LPENR_RTCAPBLPEN          (0x10000UL)               /*!< RCC APB4LPENR: RTCAPBLPEN (Bitfield-Mask: 0x01)       */
#define RCC_APB4LPENR_SAI4LPEN_Pos        (21UL)                    /*!< RCC APB4LPENR: SAI4LPEN (Bit 21)                      */
#define RCC_APB4LPENR_SAI4LPEN            (0x200000UL)              /*!< RCC APB4LPENR: SAI4LPEN (Bitfield-Mask: 0x01)         */

#endif /* __ARCH_ARM_SRC_STM32H7_CHIP_STM32H7X3XX_RCC_H */
