/* linux/include/asm-arm/arch-omap/omap850.h
 *
 * Hardware definitions for TI OMAP850 processor.
 *
 * Cleanup for Linux-2.6 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Copied from omap730.h by Zebediah C. McClure <zmc@centrixpr.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP850_H
#define __ASM_ARCH_OMAP850_H

/*
 * ----------------------------------------------------------------------------
 * Base addresses
 * ----------------------------------------------------------------------------
 */

/* Syntax: XX_BASE = Virtual base address, XX_START = Physical base address */

#define OMAP850_DSP_BASE	0xE0000000
#define OMAP850_DSP_SIZE	0x50000
#define OMAP850_DSP_START	0xE0000000

#define OMAP850_DSPREG_BASE	0xE1000000
#define OMAP850_DSPREG_SIZE	SZ_128K
#define OMAP850_DSPREG_START	0xE1000000

#define OMAP850_SPI1_BASE	0xfffc0800
#define OMAP850_SPI2_BASE	0xfffc1000
/*
 * ----------------------------------------------------------------------------
 * OMAP850 specific configuration registers
 * ----------------------------------------------------------------------------
 */
#define OMAP850_CONFIG_BASE	0xfffe1000
#define OMAP850_IO_CONF_0	0xfffe1070
#define OMAP850_IO_CONF_1	0xfffe1074
#define OMAP850_IO_CONF_2	0xfffe1078
#define OMAP850_IO_CONF_3	0xfffe107c
#define OMAP850_IO_CONF_4	0xfffe1080
#define OMAP850_IO_CONF_5	0xfffe1084
#define OMAP850_IO_CONF_6	0xfffe1088
#define OMAP850_IO_CONF_7	0xfffe108c
#define OMAP850_IO_CONF_8	0xfffe1090
#define OMAP850_IO_CONF_9	0xfffe1094
#define OMAP850_IO_CONF_10	0xfffe1098
#define OMAP850_IO_CONF_11	0xfffe109c
#define OMAP850_IO_CONF_12	0xfffe10a0
#define OMAP850_IO_CONF_13	0xfffe10a4

#define OMAP850_MODE_1		0xfffe1010
#define OMAP850_MODE_2		0xfffe1014

#define OMAP850_DAGON_MODE	0xfffe10f8

/* CSMI specials: in terms of base + offset */
#define OMAP850_MODE2_OFFSET	0x14

/*
 * ----------------------------------------------------------------------------
 * OMAP850 traffic controller configuration registers
 * ----------------------------------------------------------------------------
 */
#define OMAP850_FLASH_CFG_0	0xfffecc10
#define OMAP850_FLASH_ACFG_0	0xfffecc50
#define OMAP850_FLASH_CFG_1	0xfffecc14
#define OMAP850_FLASH_ACFG_1	0xfffecc54

/*
 * ----------------------------------------------------------------------------
 * OMAP850 DSP control registers
 * ----------------------------------------------------------------------------
 */
#define OMAP850_ICR_BASE	0xfffbb800
#define OMAP850_DSP_M_CTL	0xfffbb804
#define OMAP850_DSP_MMU_BASE	0xfffed200

/*
 * ----------------------------------------------------------------------------
 * OMAP850 PCC_UPLD configuration registers
 * ----------------------------------------------------------------------------
 */
#define OMAP850_PCC_UPLD_CTRL_BASE	(0xfffe0900)
#define OMAP850_PCC_UPLD_CTRL		(OMAP850_PCC_UPLD_CTRL_BASE + 0x00)


#define OMAP850_EAC_BASE           0xFFFBB000                 // VA
#define OMAP850_EAC_START          OMAP850_EAC_BASE           // PA
#define OMAP850_EAC_SIZE           SZ_4K                      // size

/* ---------------------------------------------------------------------------
 *  OMAP730 EAC (Audio) Register definitions
 * ---------------------------------------------------------------------------
 */
#define EAC_CPCFR1			(OMAP850_EAC_BASE + 0x00)
#define EAC_CPCFR2			(OMAP850_EAC_BASE + 0x02)
#define EAC_CPCFR3			(OMAP850_EAC_BASE + 0x04)
#define EAC_CPCFR4			(OMAP850_EAC_BASE + 0x06)
#define EAC_CPTCTL			(OMAP850_EAC_BASE + 0x08)
#define EAC_CPTTADR			(OMAP850_EAC_BASE + 0x0A)
#define EAC_CPTDATL			(OMAP850_EAC_BASE + 0x0C)
#define EAC_CPTDATH			(OMAP850_EAC_BASE + 0x0E)
#define EAC_CPTVSLL			(OMAP850_EAC_BASE + 0x10)
#define EAC_CPTVSLH			(OMAP850_EAC_BASE + 0x12)
#define EAC_MPCTR			(OMAP850_EAC_BASE + 0x20)
#define EAC_MPMCCFR			(OMAP850_EAC_BASE + 0x22)
#define EAC_MPACCFR			(OMAP850_EAC_BASE + 0x24)
#define EAC_MPADLTR			(OMAP850_EAC_BASE + 0x26)
#define EAC_MPADMTR			(OMAP850_EAC_BASE + 0x28)
#define EAC_MPADLRR			(OMAP850_EAC_BASE + 0x2A)
#define EAC_MPADMRR			(OMAP850_EAC_BASE + 0x2C)
#define EAC_BPCTR			(OMAP850_EAC_BASE + 0x30)
#define EAC_BPMCCFR			(OMAP850_EAC_BASE + 0x32)
#define EAC_BPACCFR			(OMAP850_EAC_BASE + 0x34)
#define EAC_BPADLTR			(OMAP850_EAC_BASE + 0x36)
#define EAC_BPADMTR			(OMAP850_EAC_BASE + 0x38)
#define EAC_BPADLRR			(OMAP850_EAC_BASE + 0x3A)
#define EAC_BPADMRR			(OMAP850_EAC_BASE + 0x3C)
#define EAC_AMSCFR			(OMAP850_EAC_BASE + 0x40)
#define EAC_AMVCTR			(OMAP850_EAC_BASE + 0x42)
#define EAC_AM1VCTR			(OMAP850_EAC_BASE + 0x44)
#define EAC_AM2VCTR			(OMAP850_EAC_BASE + 0x46)
#define EAC_AM3VCTR			(OMAP850_EAC_BASE + 0x48)
#define EAC_ASTCTR			(OMAP850_EAC_BASE + 0x4A)
#define EAC_APD1LCR			(OMAP850_EAC_BASE + 0x4C)
#define EAC_APD1RCR			(OMAP850_EAC_BASE + 0x4E)
#define EAC_APD2LCR			(OMAP850_EAC_BASE + 0x50)
#define EAC_APD2RCR			(OMAP850_EAC_BASE + 0x52)
#define EAC_APD3LCR			(OMAP850_EAC_BASE + 0x54)
#define EAC_APD3RCR			(OMAP850_EAC_BASE + 0x56)
#define EAC_APD4R			(OMAP850_EAC_BASE + 0x58)
#define EAC_ADWDR			(OMAP850_EAC_BASE + 0x5A)
#define EAC_ADRDR			(OMAP850_EAC_BASE + 0x5C)
#define EAC_AGCFR			(OMAP850_EAC_BASE + 0x5E)
#define EAC_AGCTR			(OMAP850_EAC_BASE + 0x60)
#define EAC_AGCFR2			(OMAP850_EAC_BASE + 0x62)

#define MIXER_x_A_MASK			0x007F
#define MIXER_x_B_MASK			0x7F00
#define	MIXER_x_A_GAIN_OFFSET		0
#define	MIXER_x_B_GAIN_OFFSET		8

#define EAC_AGCTR_RESERVED		0x07F0
#define	EAC_AGCTR_EACPWD		0x0001
#define	EAC_AGCTR_AUDEN			0x0002
#define EAC_AGCTR_MCLK_EN		0x0008
#define EAC_AGCTR_DMAWEN		0x0800
#define EAC_AGCTR_DMAREN		0x1000

#define EAC_AGCFR_RESERVED		0xF800
#define EAC_AGCFR_B8_16			0x0200
#define EAC_AGCFR_MN_ST			0x0400
#define	EAC_AGCFR_AUD_CKSRC_12MHZ	0x0010
#define	EAC_AGCFR_AUD_CKSRC_13MHZ	0x002C
#define EAC_AGCFR_FSINT_MASK		0x00C0
#define EAC_AGCFR_FSINT_8KHZ		0x0000
#define EAC_AGCFR_FSINT_11KHZ		0x0040
#define EAC_AGCFR_FSINT_22KHZ		0x0080
#define EAC_AGCFR_FSINT_44KHZ		0x00C0

#define	EAC_AMSCFR_DEFAULT_SWITCHES	0x0BE7
//#define	EAC_AMSCFR_DEFAULT_SWITCHES	0x00A0

#define	EAC_AMVCTR_RD_DMA_OFFSET	0
#define	EAC_AMVCTR_WR_DMA_OFFSET	8

#define EAC_ASTCTR_ATTEN		0x0001

#define EAC_CPTCTL_RESERVED		0xFF00
#define EAC_CPTCTL_CRST			0x0001
#define EAC_CPTCTL_CPEN			0x0008
#define EAC_CPTCTL_TXE			0x0020
#define EAC_CPTCTL_RXF			0x0080

#define EAC_CPCFR1_MODE_I2S		0x000C

#define	EAC_CPCFR2_I2S_20BITS		0x001B

#define	EAC_CPCFR3_I2S_INPUT		0x00EB

#define EAC_CPCFR4_I2S_DIV7		0x0007

#define	EAC_MPMCCFR_DEFAULT_MASTER_NOCOMP_16BITS	0x01EF

#define EAC_MPCTR_DISABLEALL		0x0000
#define	EAC_MPCTR_PRE_MC_16		0x0008
#define	EAC_MPCTR_MC_EN			0x0080
#define	EAC_MPCTR_CKEN			0x0001

#define EAC_BPCTR_DISABLEALL		0x0000

#define EAC_BPMCCFR_DEFAULT_SLAVE_NOCOMP_16BITS		0x00EF

#define	EAC_BPCTR_PRE_MC_16		0x0008
#define	EAC_BPCTR_MC_EN			0x0080
#define	EAC_BPCTR_CKEN			0x0001

#define	SOFT_REQ_REG_EAC12M_DPLL_REQ	0x4000

#define PCC_PERIPH_SOURCE_EAC_CLK_SOURCE		0x0010
#define	CAM_CLK_CTRL_SYSTEM_CLK_EN	0x0004

/*
 * ---------------------------------------------------------------------------
 * OMAP730 PCC UPLD Register Definitions
 * ---------------------------------------------------------------------------
 */
#define OMAP730_PCC_ULPD_BASE      0xFFFE0800                 // VA
#define OMAP730_PCC_ULPD_START     OMAP730_PCC_ULPD_BASE      // PA
#define OMAP730_PCC_ULPD_SIZE      SZ_2K                      // size


#define CLOCK_CTRL_REG             (OMAP730_PCC_ULPD_BASE + 0x030)
#define SOFT_REQ_REG               (OMAP730_PCC_ULPD_BASE + 0x034)
#define SOFT_DISABLE_REQ_REG       (OMAP730_PCC_ULPD_BASE + 0x068)
#define CAM_CLK_CTRL               (OMAP730_PCC_ULPD_BASE + 0x07C)
#define PCC_CTRL_REG               (OMAP730_PCC_ULPD_BASE + 0x100)
#define PCC_PERIPH_CLOCK_SOURCE_SEL (OMAP730_PCC_ULPD_BASE + 0x108)
#define PCC_DBB_STATUS		   (OMAP730_PCC_ULPD_BASE + 0x120)
#define SOFT_REQ_REG               (OMAP730_PCC_ULPD_BASE + 0x034)


#endif /*  __ASM_ARCH_OMAP850_H */

