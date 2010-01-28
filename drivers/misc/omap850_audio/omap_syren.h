/* drivers/android/omap730_syren.h
** 
** Copyright 2005-2006, Google Inc.
** Author: Ficus Kirkpatrick
**
** This file is dual licensed.  It may be redistributed and/or modified
** under the terms of the Apache 2.0 License OR version 2 of the GNU
** General Public License.
*/

#ifndef __OMAP_730_SYREN_H
#define __OMAP_730_SYREN_H

#define SYREN_PAGE_0					0
#define SYREN_PAGE_1					1
#define SYREN_READ_REG					1
#define SYREN_WRITE_REG					0
#define SYREN_DELAY_USEC				157 // Syren internal bus needs time to put value on its bus

#define SYREN_PAGE						0x01 // Page 0 & 1
#define SYREN_TOGBR1					0x04 // Page 0
#define SYREN_TOGBR2					0x05 // Page 0
#define SYREN_VBDCTRL					0x06 // Page 0
#define SYREN_PWDNRG					0x09 // Page 1
#define SYREN_VAUDCTRL					0x0F // Page 1
#define SYREN_VAUSCTRL					0x10 // Page 1
#define SYREN_VAUOCTRL					0x11 // Page 1
#define SYREN_VAUDPLL					0x12 // Page 1
#define SYREN_VRPCSIMR					0x17 // Page 1

#define SYREN_TOGBR2_ACTS				0x008 // Activate MCLK of PWDNRG register
#define SYREN_TOGBR2_AUDS				0x100 // Activate AUDON of PWDNRG register
#define SYREN_TOGBR2_AUDR				0x080 // Deactivate AUDON of PWDNRG register
#define SYREN_VAUDCTRL_SR_MASK			(0x7<<5)	// Sample rate bits in VAUDCTRL
#define SYREN_VAUDCTRL_SRW_441			(2<<5)// Sample rate frequency = 44.1Khz
#define SYREN_VAUOCTRL_AUDIO_MASK		0x02aa	// Output bits pertaining to DMA audio (not voice)
#define SYREN_VAUOCTRL_HSOR				0x002 // Enable Audio stereo Right on HSOR output
#define SYREN_VAUOCTRL_HSOL				0x008 // Enable Audio stereo Left on HSOL output
#define SYREN_VAUOCTRL_HS				(SYREN_VAUOCTRL_HSOR | SYREN_VAUOCTRL_HSOL)
#define SYREN_VAUOCTRL_SPK				0x020 // enable audio mono on speaker output
#define SYREN_VAUDPLL_I2SON				0x100 // Set the power on of the I2S serial interface
#define SYREN_VAUDPLL_AUPLLON				0x002 // Audio PLL power up
#define SYREN_VAUSCTRL_ODB				0x000 // Gain = 0dB on left and right channel

#define SYREN_VRPCSIMR_RUSBEN			(1<<5)



#define OMAP_SYREN_SPI_BASE 0xFFFC1800

#define SPI_SET1    (OMAP_SYREN_SPI_BASE+0x00)
#define SPI_SET2    (OMAP_SYREN_SPI_BASE+0x02)
#define SPI_CTRL    (OMAP_SYREN_SPI_BASE+0x04)
#define SPI_STATUS  (OMAP_SYREN_SPI_BASE+0x06)
#define SPI_TX_LSB  (OMAP_SYREN_SPI_BASE+0x08)
#define SPI_TX_MSB  (OMAP_SYREN_SPI_BASE+0x0A)
#define SPI_RX_LSB  (OMAP_SYREN_SPI_BASE+0x0C)
#define SPI_RX_MSB  (OMAP_SYREN_SPI_BASE+0x0E)

#define SPI_CTRL_RD     0x0001
#define SPI_CTRL_WR     0x0002
#define SPI_CTRL_16BIT  0x003C
#define SPI_CTRL_32BIT  0x007C

#define SPI_STATUS_WE   0x0002
#define SPI_STATUS_RD   0x0001

// OMAP730 SYREN SPI contant definitions (16 bits)
#define SYREN_SPI_SET1_CLOCK_ON			0x0001
#define SYREN_SPI_SET1_CLOCK_OFF		0x0000
#define SYREN_SPI_SET1_IT_RD_ON			0x0020
#define SYREN_SPI_SET1_IT_RD_OFF		0x0000
#define SYREN_SPI_SET1_IT_WR_ON			0x0010
#define SYREN_SPI_SET1_IT_WR_OFF		0x0000
#define SYREN_SPI_SET1_PTV1				0x0000
#define SYREN_SPI_SET2_C_HIGH			0x001F
#define SYREN_SPI_SET2_C_LOW			0x0000
#define SYREN_SPI_SET2_P_HIGH			0x03E0
#define SYREN_SPI_SET2_P_LOW			0x0000
#define SYREN_SPI_SET2_L_HIGH			0x7C00
#define SYREN_SPI_SET2_L_LOW			0x0000
#define SYREN_SPI_CTRL_STOP				0x0000
#define SYREN_SPI_CTRL_RD				0x0001
#define SYREN_SPI_CTRL_WR				0x0002
#define SYREN_SPI_CTRL_16BIT			0x003C
#define SYREN_SPI_CTRL_32BIT			0x007C
#define SYREN_SPI_CTRL_AD				0x0000
#define SYREN_SPI_MSK_WE_BIT			0x0002
#define SYREN_SPI_MSK_RD_BIT			0x0001

#endif
