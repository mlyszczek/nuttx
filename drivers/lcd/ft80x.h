/*******************************************************************************************
 * drivers/lcd/ft80x.h
 * Definitions for the FTDI FT80x GUI
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *  - Document No.: FT_000792, "FT800 Embedded Video Engine", Datasheet Version 1.1,
 *    Clearance No.: FTDI# 334, Future Technology Devices International Ltd.
 *  - Document No.: FT_000986, "FT801 Embedded Video Engine Datasheet", Version 1.0,
 *    Clearance No.: FTDI#376, Future Technology Devices International Ltd.

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
 *******************************************************************************************/

#ifndef __DRIVERS_LCD_FT80X_H
#define __DRIVERS_LCD_FT80X_H

/*******************************************************************************************
 * Included Files
 *******************************************************************************************/

/*******************************************************************************************
 * Pre-processor Definitions
 *******************************************************************************************/

/* FT80x Memory Map ************************************************************************/

#define FT80X_RAM_G                0x000000  /* Main graphics RAM (256Kb) */
#define FT80X_ROM_CHIPID           0x0c0000  /* FT80x chip identification and revision
                                              * information (4b):
                                              * Byte [0:1] Chip ID: 0800 or 0801
                                              * Byte [2:3] Version ID: 0100 */
#define FT80X_ROM_FONT             0x0bb23c  /* Font table and bitmap (275Kb) */
#define FT80X_ROM_FONT_ADDR        0x0ffffc  /* Font table pointer address (4b) */
#define FT80X_RAM_DL               0x100000  /* Display List RAM (8Kb) */
#define FT80X_RAM_PAL              0x102000  /* Palette RAM (1Kb) */
#define FT80X_REG                  0x102400  /* Registers (380b) */
#define FT80X_RAM_CMD              0x108000  /* Command Buffer (4Kb) */

#ifdef CONFIG_LCD_FT801
#  define FT80X_RAM_SCREENSHOT     0x108000  /* Screenshot readout buffer  (2Kb) */
#endif

/* FT80x Registers *************************************************************************/

#define FT80X_ID                   0x102400  /* Identification register, always reads as 7c */
#define FT80X_FRAMES               0x102404  /* Frame counter, since reset */
#define FT80X_CLOCK                0x102408  /* Clock cycles, since reset */
#define FT80X_FREQUENCY            0x10240c  /* Main clock frequency */

#if defined(CONFIG_LCD_FT800)
#  define FT80X_RENDERMODE         0x102410  /* Rendering mode: 0 = normal, 1 = single-line */
#  define FT80X_SNAPY              0x102414  /* Scan line select for RENDERMODE 1 */
#  define FT80X_SNAPSHOT           0x102418  /* Trigger for RENDERMODE 1 */
#elif defined(CONFIG_LCD_FT801)
#  define FT80X_SCREENSHOT_EN      0x102410  /* Set to enable screenshot mode */
#  define FT80X_SCREENSHOT_Y       0x102414  /* Y line number for screenshot */
#  define FT80X_SCREENSHOT_START   0x102418  /* Screenshot start trigger */
#endif

#define FT80X_CPURESET             0x10241c  /* Graphics, audio and touch engines reset
                                              * control */
#define FT80X_TAP_CRC              0x102420  /* Live video tap crc. Frame CRC is computed
                                              * every DL SWAP. */
#define FT80X_TAP_MASK             0x102424  /* Live video tap mask */
#define FT80X_HCYCLE               0x102428  /* Horizontal total cycle count */
#define FT80X_HOFFSET              0x10242c  /* Horizontal display start offset */
#define FT80X_HSIZE                0x102430  /* Horizontal display pixel count */
#define FT80X_HSYNC0               0x102434  /* Horizontal sync fall offset */
#define FT80X_HSYNC1               0x102438  /* Horizontal sync rise offset */
#define FT80X_VCYCLE               0x10243c  /* Vertical total cycle count */
#define FT80X_VOFFSET              0x102440  /* Vertical display start offset */
#define FT80X_VSIZE                0x102444  /* Vertical display line count */
#define FT80X_VSYNC0               0x102448  /* Vertical sync fall offset */
#define FT80X_VSYNC1               0x10244c  /* Vertical sync rise offset */
#define FT80X_DLSWAP               0x102450  /* Display list swap control */
#define FT80X_ROTATE               0x102454  /* Screen 180 degree rotate */
#define FT80X_OUTBITS              0x102458  /* Output bit resolution, 3x3x3 bits */
#define FT80X_DITHER               0x10245c  /* Output dither enable */
#define FT80X_SWIZZLE              0x102460  /* Output RGB signal swizzle */
#define FT80X_CSPREAD              0x102464  /* Output clock spreading enable */
#define FT80X_PCLK_POL             0x102468  /* PCLK polarity: 0=rising edge, 1= falling edge */
#define FT80X_PCLK                 0x10246c  /* PCLK frequency divider, 0 = disable */
#define FT80X_TAG_X                0x102470  /* Tag query X coordinate */
#define FT80X_TAG_Y                0x102474  /* Tag query Y coordinate */
#define FT80X_TAG                  0x102478  /* Tag query result */
#define FT80X_VOL_PB               0x10247c  /* Volume for playback */
#define FT80X_VOL_SOUND            0x102480  /* Volume for synthesizer sound */
#define FT80X_SOUND                0x102484  /* Sound effect select */
#define FT80X_PLAY                 0x102488  /* Start effect playback */
#define FT80X_GPIO_DIR             0x10248c  /* GPIO pin direction, 0=input, 1=output */
#define FT80X_GPIO                 0x102490  /* Pin value (bits 0,1,7); Drive strength
                                              * (bits 2-6) */
                                             /* 0x102494 Reserved */
#define FT80X_INT_FLAGS            0x102498  /* Interrupt flags, clear by read */
#define FT80X_INT_EN               0x10249c  /* Global interrupt enable */
#define FT80X_INT_MASK             0x1024a0  /* Interrupt enable mask */
#define FT80X_PLAYBACK_START       0x1024a4  /* Audio playback RAM start address */
#define FT80X_PLAYBACK_LENGTH      0x1024a8  /* Audio playback sample length (bytes) */
#define FT80X_PLAYBACK_READPTR     0x1024ac  /* Audio playback current read pointer */
#define FT80X_PLAYBACK_FREQ        0x1024b0  /* Audio playback sampling frequency (Hz) */
#define FT80X_PLAYBACK_FORMAT      0x1024b4  /* Audio playback format */
#define FT80X_PLAYBACK_LOOP        0x1024b8  /* Audio playback loop enable */
#define FT80X_PLAYBACK_PLAY        0x1024bc  /* Start audio playback */
#define FT80X_PWM_HZ               0x1024c0  /* BACKLIGHT PWM output frequency (Hz) */
#define FT80X_PWM_DUTY             0x1024c4  /* BACKLIGHT PWM output duty cycle 0=0%,
                                              * 128=100% */
#define FT80X_MACRO_0              0x1024c8  /* Display list macro command 0 */
#define FT80X_MACRO_1              0x1024cc  /* Display list macro command 1 */

#if defined(CONFIG_LCD_FT800)
                                             /* 0x1024d0 – 0x1024e0 Reserved */
#elif defined(CONFIG_LCD_FT801)
                                             /* 0x1024d0 – 0x1024d4 Reserved */
#  define FT80X_SCREENSHOT_BUSY    0x1024d8  /* Screenshot ready flags */
                                             /* 0x1024e0 Reserved */
#endif

#define FT80X_CMD_READ             0x1024e4  /* Command buffer read pointer */
#define FT80X_CMD_WRITE            0x1024e8  /* Command buffer write pointer */
#define FT80X_CMD_DL               0x1024ec  /* Command display list offset */
#define FT80X_TOUCH_MODE           0x1024f0  /* Touch-screen sampling mode */

#if defined(CONFIG_LCD_FT800)
#  define FT80X_TOUCH_ADC_MODE     0x1024f4  /* Select single ended (low power) or
                                              * differential (accurate) sampling */
#  define FT80X_TOUCH_CHARGE       0x1024f8  /* Touch-screen charge time, units of 6 clocks */
#  define FT80X_TOUCH_SETTLE       0x1024fc  /* Touch-screen settle time, units of 6 clocks */
#  define FT80X_TOUCH_OVERSAMPLE   0x102500  /* Touch-screen oversample factor */
#  define FT80X_TOUCH_RZTHRESH     0x102504  /* Touch-screen resistance threshold */
#  define FT80X_TOUCH_RAW_XY       0x102508  /* Touch-screen raw (x-MSB16; y-LSB16) */
#  define FT80X_TOUCH_RZ           0x10250c  /* Touch-screen resistance */
#  define FT80X_TOUCH_SCREEN_XY    0x102510  /* Touch-screen screen (x-MSB16; y-LSB16) */
#  define FT80X_TOUCH_TAG_XY       0x102514  /* Touch-screen screen (x-MSB16; y-LSB16)
                                              * used for tag lookup */
#  define FT80X_TOUCH_TAG          0x102518  /* Touch-screen tag result */
#  define FT80X_TOUCH_TRANSFORM_A  0x10251c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_TOUCH_TRANSFORM_B  0x102520  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_TOUCH_TRANSFORM_C  0x102524  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_TOUCH_TRANSFORM_D  0x102528  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_TOUCH_TRANSFORM_E  0x10252c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_TOUCH_TRANSFORM_F  0x102530  /* Touch-screen transform coefficient (s15.16) */
                                             /* 0x102534 – 0x102470 Reserved */
#  define FT80X_TOUCH_DIRECT_XY    0x102574  /* Touch screen direct (x-MSB16; y-LSB16)
                                              * conversions */
#  define FT80X_TOUCH_DIRECT_Z1Z2  0x102578  /* Touch screen direct (z1-MSB16; z2-LSB16)
                                              * conversions */
#elif defined(CONFIG_LCD_FT801)
                                             /* 0x1024d0 – 0x1024d4 Reserved */
#  define FT80X_CTOUCH_EXTENDED    0x1024f4  /* Set capacitive touch operation mode:
                                              * 0: extended mode (multi-touch)
                                              * 0: 1: FT800 compatibility mode (single touch) */
#  define FT80X_CTOUCH_REG         0x1024f8  /* CTPM configure register write
                                              * Bits [7:0]: configure register address
                                              * Bits [15:8]: configure register value */
                                             /* 0x1024fc - 0x102504 Reserved */
#  define FT80X_CTOUCH_RAW_XY      0x102508  /* Compatibility mode: touch-screen raw
                                              * (x-MSB16; y-LSB16) */
#  define FT80X_CTOUCH_TOUCH1_XY   0x102508  /* Extended mode: touch-screen screen data for touch 1
                                              * (x-MSB16; y-LSB16) */
#  define FT80X_CTOUCH_TOUCH4_XY   0x10250c  /* Extended mode: touch-screen screen Y data for touch 4 */
#  define FT80X_CTOUCH_SCREEN_XY   0x102510  /* Compatibility mode: touch-screen screen
                                              * (x-MSB16; y-LSB16) */
#  define FT80X_CTOUCH_TOUCH0_XY   0x102510  /* Extended mode: touch-screen screen data for touch 0
                                              * (x-MSB16; y-LSB16) */
#  define FT80X_CTOUCH_TAG_XY      0x102514  /* Touch-screen screen (x-MSB16; y-LSB16)
                                              * used for tag lookup */
#  define FT80X_CTOUCH_TAG         0x102518  /* Touch-screen tag result */
#  define FT80X_CTOUCH_TRANSFORM_A 0x10251c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_CTOUCH_TRANSFORM_B 0x102520  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_CTOUCH_TRANSFORM_C 0x102524  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_CTOUCH_TRANSFORM_D 0x102528  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_CTOUCH_TRANSFORM_E 0x10252c  /* Touch-screen transform coefficient (s15.16) */
#  define FT80X_CTOUCH_TRANSFORM_F 0x102530  /* Touch-screen transform coefficient (s15.16) */
                                             /* 0x102534 Reserved */
#  define FT80X_CTOUCH_TOUCH4_X    0x102538  /* Extended mode: touch-screen screen X data for
                                              * touch 4 */
                                             /* 0x10253c – 0x102450 Reserved */
#  define FT80X_SCREENSHOT_READ    0x102554  /* Set to enable readout of the screenshot of the
                                              * selected Y line */
                                             /* 0x10253c – 0x102468 Reserved */
#  define FT80X_TRIM               0x10256c  /* Internal relaxation clock trimming */
                                             /* 0x102570 Reserved */
#  define FT80X_CTOUCH_DIRECT_XY   0x102574  /* Compatibility mode: Touch screen direct
                                              * (x-MSB16; y-LSB16) conversions */
#  define FT80X_CTOUCH_TOUCH2_XY   0x102574  /* Extended mode: touch-screen screen data for
                                              * touch 2 (x-MSB16; y-LSB16) */
#  define FT80X_CTOUCH_DIRECT_Z1Z2 0x102578  /* Compatibility mode: Touch screen direct
                                              * (z1-MSB16; z2-LSB16) conversions */
#  define FT80X_CTOUCH_TOUCH3_XY   0x102578  /* Extended mode: touch-screen screen data for
                                              * touch 3 (x-MSB16; y-LSB16) */
#endif

#define FT80X_TRACKER              0x109000  /* Track register (Track value – MSB16;
                                              * Tag value - LSB8) */

/*******************************************************************************************
 * Public Types
 *******************************************************************************************/

/* For SPI memory read transaction, the host sends two zero bits, followed by the 22-bit
 * address. This is followed by a dummy byte. After the dummy byte, the FT80x responds to
 *  each host byte with read data bytes.
 *
 * For I2C memory read transaction, bytes are packed in the I2C protocol as follow:
 *
 *   [start] <DEVICE ADDRESS + write bit>
 *   <00b+Address[21:16]>
 *   <Address[15:8]>
 *   <Address[7:0]>
 *   [restart] <DEVICE ADDRESS + read bit>
 *   <Read data byte 0>
 *   ....
 *   <Read data byte n> [stop]
 */

struct ft80x_spiread_s
{
  uint8_t addrh;   /* Bits 6-7: 00, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
  uint8_t dummmy;  /* Dummy byte */
};

struct ft80x_i2cread_s
{
  uint8_t addrh;   /* Bits 6-7: 00, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
};

/* For SPI memory write transaction, the host sends a ‘1’ bit and ‘0’ bit, followed by the
 * 22-bit address. This is followed by the write data.
 *
 * For I2C memory write transaction, bytes are packed in the I2C protocol as follow:
 *
 *  [start] <DEVICE ADDRESS + write bit>
 *  <10b,Address[21:16]>
 *  <Address[15:8]>
 *  <Address[7:0]>
 *  <Write data byte 0>
 *  ....
 *  <Write data byte n> [stop]
 */

struct ft80x_spiwrite_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
                   /* Write data follows */
};

struct ft80x_i2cwrite_s
{
  uint8_t addrh;   /* Bits 6-7: 10, Bits 0-5: Address[21:16] */
  uint8_t addrm;   /* Address[15:8] */
  uint8_t addrl;   /* Address[7:0] */
                   /* Write data follows */
};

#endif /* __DRIVERS_LCD_FT80X_H */