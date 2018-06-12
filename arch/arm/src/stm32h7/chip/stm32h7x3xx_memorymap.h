/************************************************************************************
 * arch/arm/src/stm3fr2/chip/stm32h7x3xx_memorymap.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_CHIP_STM32H77X3XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32H7_CHIP_STM32H77X3XX_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32H77X3XX Address Blocks *******************************************/

// TODO: Check all of this an make complete ...

//#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
//#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
//#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb AHB1-2 peripheral blocks */
//#define STM32_FMC_BASE12     0x60000000     /* 0x60000000-0x7fffffff: 512Mb FMC bank1&2 block */
//#  define STM32_FMC_BANK1    0x60000000     /* 0x60000000-0x6fffffff:       256Mb NOR/SRAM */
//#  define STM32_FMC_BANK2    0x70000000     /* 0x70000000-0x7fffffff:       256Mb NAND FLASH */
//#define STM32_FMC_BASE34     0x80000000     /* 0x80000000-0x8fffffff: 512Mb FMC bank3&4 block */
//#  define STM32_FMC_BANK3    0x80000000     /* 0x80000000-0x8fffffff:       256Mb NAND FLASH */
//#  define STM32_FMC_BANK4    0x90000000     /* 0x90000000-0x9fffffff:       256Mb PC CARD */
//#define STM32_FMC_BASE5      0xc0000000     /* 0xc0000000-0xcfffffff: 256Mb FMC */
//#define STM32_FMC_BASE6      0xd0000000     /* 0xd0000000-0xdfffffff: 256Mb FMC */
//#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M7 block */
//
//#define STM32_REGION_MASK    0xf0000000
//#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
//#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FMC_BANK1)

/* Code Base Addresses **************************************************************/

//#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
//#define STM32_INSTRAM_BASE   0x00000000     /* 0x00000000-0x00003fff: Instruction RAM (ITCM-RAM) */
//#define STM32_SYSMEM_ICTM    0x00100000     /* 0x00100000-0x0010edbf: System memory (ITCM) */
//#define STM32_FLASH_ITCM     0x00200000     /* 0x00200000-0x003fffff: FLASH memory (ITCM) */
//#define STM32_LOADER_BASE    0x01000000     /* 0x01000000-            Bootloader */
//#define STM32_FLASH_AXIM     0x08000000     /* 0x08000000-0x081fffff: FLASH memory (AXIM) */
//#define STM32_OPTIONS_BASE   0x1fff0000     /* 0x1ff00000-0x1fff001f: OTP (AXIM) */

/* Information Addresses ************************************************************/

//#define STM32_SYSMEM_AXIM    0x1ff00000     /* 0x1ff00000-0x1ff0edbf: System memory (AXIM) */
//#define STM32_SYSMEM_UID     0x1ff0f420     /* The 96-bit unique device identifier */
//#define STM32_OTP_ICTM       0x0010f000     /* 0x0010f000-0x0010edbf: OTP (ITCM) */
//#define STM32_OTP_AXIM       0x1ff0f000     /* 0x1ff00000-0x1ff0f41f: OTP (AXIM) */

/* SRAM Base Addresses **************************************************************/

#define STM32_DTCRAM_BASE    0x20000000     /* 0x20000000-0x2001ffff: DTCM-RAM on TCM interface */
#define STM32_SRAM_BASE      0x24000000     /* 0x24000000-0x247fffff: System SRAM */
#define STM32_SRAM1_BASE     0x30000000     /* 0x30000000-0x3001ffff: System SRAM1 */
#define STM32_SRAM2_BASE     0x30020000     /* 0x30020000-0x3003ffff: System SRAM2 */
#define STM32_SRAM3_BASE     0x3004c000     /* 0x30040000-0x30047fff: System SRAM3 */
#define STM32_SRAM123_BASE   0x30000000     /* 0x30000000-0x30047fff: System SRAM123 */
#define STM32_SRAM4_BASE     0x38000000     /* 0x38000000-0x3800ffff: System SRAM4 */
#define STM32_BBRAM_BASE     0x38800000     /* 0x38800000-0x38800fff: System BBRAM */

/* Peripheral Base Addresses ********************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x40007fff: APB1 */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x40016bff: APB2 */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x4007ffff: APB1 */
#define STM32_AHB2_BASE      0x48020000     /* 0x50000000-0x48022bff: AHB2 */
#define STM32_APB3_BASE      0x50000000     /* 0x60000000-0x50003fff: APB3 */
#define STM32_AHB3_BASE      0x51000000     /* 0x51000000-0x52008fff: AHB3 */
#define STM32_APB4_BASE      0x58000000     /* 0x60000000-0x58006bff: APB4 */
#define STM32_AHB4_BASE      0x58020000     /* 0x58020000-0x580267ff: AHB4 */

/* APB1 Base Addresses **************************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 */
#define STM32_TIM12_BASE     0x40001800     /* 0x40001800-0x40001bff: TIM12 */
#define STM32_TIM13_BASE     0x40001c00     /* 0x40001c00-0x40001fff: TIM13 */
#define STM32_TIM14_BASE     0x40002000     /* 0x40002000-0x400023ff: TIM14 */
#define STM32_LPTIM1_BASE    0x40002400     /* 0x40002400-0x400027ff: LPTIM1 */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define STM32_I2S2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define STM32_I2S3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define STM32_SPDIFRX_BASE   0x40004000     /* 0x40004000-0x400043ff: SPDIFRX */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005bff: I2C2 */
#define STM32_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define STM32_HDMICEC_BASE   0x40006c00     /* 0x40006c00-0x40006fff: HDMI-CEC */
#define STM32_DAC1_BASE      0x40007400     /* 0x40007400-0x400077ff: DAC */
#define STM32_UART7_BASE     0x40007800     /* 0x40007800-0x40007bff: UART7 */
#define STM32_UART8_BASE     0x40007c00     /* 0x40007c00-0x40007fff: UART8 */
#define STM32_CRS_BASE       0x40008400     /* 0x40007c00-0x40007fff: CRS */
#define STM32_SWPMI_BASE     0x40008800     /* 0x40008800 - 0x40008bff SWPMI Section */
#define STM32_OPAMP_BASE     0x40009000     /* 0x40009000 - 0x400093ff OPAMP Section */
#define STM32_MDIOS_BASE     0x4000a000     /* 0x40009400 - 0x400097ff MDIOS Section */
#define STM32_FDCAN1_BASE    0x40008400     /* 0x4000a000 - 0x4000a3ff FDCAN1 Section */
#define STM32_FDCAN2_BASE    0x4000a400     /* 0x4000a400 - 0x4000A7ff FDCAN2 Section */
#define STM32_CANCCU_BASE    0x4000a800     /* 0x4000a800 - 0x4000abff CAN CCU Section */
#define STM32_CANRAM_BASE    0x4000ac00     /* 0x4000ac00 - 0x4000d3ff CAN Message RAM */

/* APB2 Base Addresses **************************************************************/

//#define STM32_TIM1_BASE      0x40010000     /* 0x40010000-0x400103ff: TIM1 */
//#define STM32_TIM8_BASE      0x40010400     /* 0x40010400-0x400107ff: TIM8 */
//#define STM32_USART1_BASE    0x40011000     /* 0x40011000-0x400113ff: USART1 */
//#define STM32_USART6_BASE    0x40011400     /* 0x40011400-0x400117ff: USART6 */
//#define STM32_SDMMC2_BASE    0x40011c00     /* 0x40011c00-0x40011fff: SDMMC2 */
//#define STM32_ADC_BASE       0x40012000     /* 0x40012000-0x400123ff: ADC1 - ADC2 - ADC3 */
//#  define STM32_ADC1_BASE    0x40012000     /*                        ADC1 */
//#  define STM32_ADC2_BASE    0x40012100     /*                        ADC2 */
//#  define STM32_ADC3_BASE    0x40012200     /*                        ADC3 */
//#  define STM32_ADCCMN_BASE  0x40012300     /*                        Common */
//#define STM32_SDMMC1_BASE    0x40012c00     /* 0x40012c00-0x40012fff: SDMMC1 */
//#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
//#define STM32_SPI4_BASE      0x40013400     /* 0x40013400-0x400137ff: SPI4 */
//#define STM32_SYSCFG_BASE    0x40013800     /* 0x40013800-0x40013bff: SYSCFG */
//#define STM32_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
//#define STM32_TIM9_BASE      0x40014000     /* 0x40014000-0x400143ff: TIM9 */
//#define STM32_TIM10_BASE     0x40014400     /* 0x40014400-0x400147ff: TIM10 */
//#define STM32_TIM11_BASE     0x40014800     /* 0x40014800-0x40014bff: TIM11 */
//#define STM32_SPI5_BASE      0x40015000     /* 0x40015000-0x400153ff: SPI5 */
//#define STM32_SPI6_BASE      0x40015400     /* 0x40015400-0x400157ff: SPI6 */
//#define STM32_SAI1_BASE      0x40015800     /* 0x40015800-0x40015bff: SAI1 */
//#define STM32_SAI2_BASE      0x40015c00     /* 0x40015c00-0x40015fff: SAI2 */
//#define STM32_LTDC_BASE      0x40016800     /* 0x40016800-0x40016bff: LCD-TFT */
//#define STM32_DSIHOST_BASE   0x40016c00     /* 0x40016c00-0x400173ff: DSI Host */
//#define STM32_DFSDM1_BASE    0x40017400     /* 0x40017400-0x400174ff: DFSDM1 */
//#define STM32_MDIOS_BASE     0x40017800     /* 0x40017800-0x40017bff: MDIOS */

/* AHB1 Base Addresses **************************************************************/

//#define STM32_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff: GPIOA */
//#define STM32_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff: GPIOB */
//#define STM32_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff: GPIOC */
//#define STM32_GPIOD_BASE     0x40020c00     /* 0x40020c00-0x40020fff: GPIOD */
//#define STM32_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff: GPIOE */
//#define STM32_GPIOF_BASE     0x40021400     /* 0x40021400-0x400217ff: GPIOF */
//#define STM32_GPIOG_BASE     0x40021800     /* 0x40021800-0x40021bff: GPIOG */
//#define STM32_GPIOH_BASE     0x40021c00     /* 0x40021c00-0x40021fff: GPIOH */
//#define STM32_GPIOI_BASE     0x40022000     /* 0x40022000-0x400223ff: GPIOI */
//#define STM32_GPIOJ_BASE     0x40022400     /* 0x40022400-0x400227ff: GPIOJ */
//#define STM32_GPIOK_BASE     0x40022800     /* 0x40022800-0x40022bff: GPIOK */
//#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
//#define STM32_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff: Flash interface */
//#define STM32_BKPSRAM_BASE   0x40024000     /* 0x40024000-0x40024fff: BKPSRAM */
//#define STM32_DMA1_BASE      0x40026000     /* 0x40026000-0x400263ff: DMA1 */
//#define STM32_DMA2_BASE      0x40026400     /* 0x40026400-0x400267ff: DMA2 */
//#define STM32_ETHMAC_BASE    0x40028000     /* 0x40028000-0x400293ff: ETHERNET MAC */
//#define STM32_DMA2D_BASE     0x4002b000     /* 0x4002b000-0x4002Bbff: Chrom-ART (DMA2D) */
//#define STM32_USBOTGHS_BASE  0x40040000     /* 0x40040000-0x4007ffff: USB OTG HS */

/* AHB2 Base Addresses **************************************************************/

//#define STM32_USBOTGFS_BASE  0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS */
//#define STM32_DCMI_BASE      0x50050000     /* 0x50050000-0x500503ff: DCMI */
//#define STM32_JPEG_BASE      0x50051000     /* 0x50051000-0x500511ff: JPEG */
//#define STM32_CRYP_BASE      0x50060000     /* 0x50060000-0x500603ff: CRYP */
//#define STM32_HASH_BASE      0x50060400     /* 0x50060400-0x500607ff: HASH */
//#define STM32_RNG_BASE       0x50060800     /* 0x50060800-0x50060bff: RNG */

/* APB3 Base Addresses **************************************************************/

/* AHB3 Base Addresses **************************************************************/

//#define STM32_FMCBANK1_BASE  0x60000000     /* 0x60000000-0x6fffffff: FMC bank 1 */
//#define STM32_FMCBANK2_BASE  0x70000000     /* 0x70000000-0x7fffffff: FMC bank 2 */
//#define STM32_FMCBANK3_BASE  0x80000000     /* 0x80000000-0x8fffffff: FMC bank 3 */
//#define STM32_FMCBANK4_BASE  0x90000000     /* 0x90000000-0x9fffffff: FMC bank 4 */
//#define STM32_FMC_BASE       0xa0000000     /* 0xa0000000-0xa0000fff: FMC control registers */
//#define STM32_QUADSPI_BASE   0xa0001000     /* 0xa0001000-0xa0001fff: QuadSPI Control */
//#define STM32_FMCBANK5_BASE  0xc0000000     /* 0xc0000000-0xcfffffff: FMC bank 5 */
//#define STM32_FMCBANK6_BASE  0xd0000000     /* 0xd0000000-0xdfffffff: FMC bank 6 */

/* APB4 Base Addresses **************************************************************/

#define STM32_EXTI_BASE      0x58000000     /* 0x58000000-0x580003ff EXTI */
#define STM32_SYSCFG_BASE    0x58000400     /* 0x58000400-0x580007ff SYSCFG */
#define STM32_LPUART1_BASE   0x58000c00     /* 0x58000c00-0x58000fff LPUART1 */
#define STM32_SPI6_BASE      0x58001400     /* 0x58001400-0x580017ff SPI6 */
#define STM32_I2C4_BASE      0x58001c00     /* 0x58001c00-0x58001fff I2C4 */
#define STM32_LPTIM2_BASE    0x58002400     /* 0x58002400-0x580027ff LPTIM2 */
#define STM32_LPTIM3_BASE    0x58002800     /* 0x58002800-0x58002Bff LPTIM3 */
#define STM32_LPTIM4_BASE    0x58002c00     /* 0x58002c00-0x58002fff LPTIM4 */
#define STM32_LPTIM5_BASE    0x58003000     /* 0x58003000-0x580033ff LPTIM5 */
#define STM32_COMP12_BASE    0x58003800     /* 0x58003800-0x58003bff COMP1-COMP2 */
#define STM32_VREF_BASE      0x58003c00     /* 0x58003c00-0x58003fff VREF */
#define STM32_RTC_BASE       0x58004000     /* 0x58004000-0x580043ff RTC & BKP registers */
#define STM32_IWDG1_BASE     0x58004800     /* 0x58004800-0x58004bff IWDG1 */
#define STM32_SAI4_BASE      0x58005400     /* 0x58005400-0x580057ff SAI4 */

/* AHB4 Base Addresses **************************************************************/

#define STM32_GPIOA_BASE     0x58005400     /* 0x58020000-0x580203ff GPIOA */
#define STM32_GPIOB_BASE     0x58005400     /* 0x58020400-0x580207ff GPIOB */
#define STM32_GPIOC_BASE     0x58005400     /* 0x58020800-0x58020bff GPIOC */
#define STM32_GPIOD_BASE     0x58005400     /* 0x58020c00-0x58020fff GPIOD */
#define STM32_GPIOE_BASE     0x58005400     /* 0x58021000-0x580213ff GPIOE */
#define STM32_GPIOF_BASE     0x58005400     /* 0x58021400-0x580217ff GPIOF */
#define STM32_GPIOG_BASE     0x58005400     /* 0x58021800-0x58021bff GPIOG */
#define STM32_GPIOH_BASE     0x58005400     /* 0x58021c00-0x58021fff GPIOH */
#define STM32_GPIOI_BASE     0x58005400     /* 0x58022000-0x580223ff GPIOI */
#define STM32_GPIOJ_BASE     0x58005400     /* 0x58022400-0x580227ff GPIOJ */
#define STM32_GPIOK_BASE     0x58005400     /* 0x58022800-0x58022bff GPIOK */
#define STM32_RCC_BASE       0x58005400     /* 0x58024400-0x580247ff RCC */
#define STM32_PWR_BASE       0x58005400     /* 0x58024800-0x58024bff PWR */
#define STM32_CRC_BASE       0x58005400     /* 0x58024c00-0x58024fff CRC */
#define STM32_BDMA_BASE      0x58005400     /* 0x58025400-0x580257ff BDMA */
#define STM32_DMAMUX2_BASE   0x58005400     /* 0x58025800-0x58025bff DMAMUX2 */
#define STM32_ADC3_BASE      0x58005400     /* 0x58026000-0x580263ff ADC3 */
#define STM32_HSEM_BASE      0x58005400     /* 0x58026400-0x580267ff HSEM */

/* Cortex-M7 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range
 */

//#define STM32_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_STM32H7_CHIP_STM32H77X3XX_MEMORYMAP_H */
