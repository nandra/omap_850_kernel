/* drivers/android/sspi.c
** 
** Copyright 2005-2006, Google Inc.
** Author: Ficus Kirkpatrick
**
** This file is dual licensed.  It may be redistributed and/or modified
** under the terms of the Apache 2.0 License OR version 2 of the GNU
** General Public License.
*/

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>

//#include "eac.h"
//#include "omap730_syren.h"
//#include <linux/omap_csmi.h>
//#include "omap_csmi_debug.h"
//#include "omap_regs_icr.h"
#include "syren_spi.h"

void set_bit_range_32(unsigned long reg, int high, int low, int value)
{
         unsigned int mask = ((0xffffffff) >> (32 - (high - low) - 1)) << low;
         unsigned int cur = omap_readl(reg) & ~mask;
         cur |= (value << low);
         omap_writel(cur, reg);
}
void set_bit_range_16(unsigned long reg, int high, int low, int value)
{
         unsigned int mask = ((0xffff) >> (16 - (high - low) - 1)) << low;
         unsigned int cur = omap_readw(reg) & ~mask;
	printk("addr:%x, cur=%x, mask=%x\n", reg, cur, mask);
	 cur |= (value << low);
        printk("cru=%x\n", cur);
	 omap_writew(cur, reg);
}



/* XXX */
//uint32_t omap_csmi_debug_level = OMAP_CSMI_PRINT_ERRORS | OMAP_CSMI_PRINT_INIT_STATUS | OMAP_CSMI_PRINT_INFO;
//EXPORT_SYMBOL(omap_csmi_debug_level);

u_short syren_direct_io(u_char page, u_char addr, u_short value, int write)
{ 
	int timeout;
	int i;
	u_short ret;

	syren_spi_lock();

	/* stop SSPI */
	omap_writew(SYREN_SPI_CTRL_STOP, SPI_CTRL);

	/* turn clock off so we can set PTV */
	omap_writew(SYREN_SPI_SET1_CLOCK_OFF, SPI_SET1);

	/* set PTV div = 1, wr int disable */
	// set_bit_range_16(SPI_SET1, 3, 1, 0);  - optimized out
	// set_bit_range_16(SPI_SET1, 5, 5, 0);  - optimized out
	set_bit_range_16(SPI_SET1, 4, 4, 1);

	/* turn the clock back on */
	set_bit_range_16(SPI_SET1, 0, 0, 1);

	/* active clock edge is rising for each device */
	omap_writew(0x1f, SPI_SET2);

	/* select register page */
	page &= 1;
	omap_writew((SYREN_PAGE << 1)|(1<<page)<<6, SPI_TX_MSB);
	omap_writew(0, SPI_TX_LSB);
	omap_writew(SYREN_SPI_CTRL_AD
			  | SYREN_SPI_CTRL_16BIT
			  | SYREN_SPI_CTRL_WR, SPI_CTRL);

	timeout = 1000000;
	for (;;) {
		if (omap_readw(SPI_STATUS) & SPI_STATUS_WE)
			break;
		if (--timeout == 0) {
			printk("spi: timeout page selecting\n");
			goto bail;
		}
	}
	udelay(SYREN_DELAY_USEC);

	/* read or write the value */

	/* stop SSPI */
	omap_writew(SYREN_SPI_CTRL_STOP, SPI_CTRL);
	addr &= 0x1f;
	value &= 0x3ff;

	if (write) {
		omap_writew((value << 6) | (addr << 1) | SYREN_WRITE_REG, SPI_TX_MSB);
		omap_writew(SYREN_SPI_CTRL_AD
				  | SYREN_SPI_CTRL_16BIT
				  | SYREN_SPI_CTRL_WR, SPI_CTRL);
		timeout = 100000;
		for (;;) {
			if (omap_readw(SPI_STATUS) & SPI_STATUS_WE)
				break;
			if (--timeout == 0) {
				printk("spi: timeout writing\n");
				goto bail;
			}
		}
		udelay(SYREN_DELAY_USEC);
	} else {
		omap_writew((value << 6) | (addr << 1) | SYREN_READ_REG, SPI_TX_MSB);
		for (i = 0; i < 2; i++) {
			omap_writew(SYREN_SPI_CTRL_AD
					  | SYREN_SPI_CTRL_16BIT
					  | SYREN_SPI_CTRL_RD, SPI_CTRL);
			timeout = 100000;
			for (;;) {
				if (omap_readw(SPI_STATUS) & SPI_STATUS_RD)
					break;
				if (--timeout == 0) {
					printk("spi: timeout reading (%d)\n", i);
					goto bail;
				}
			}
			udelay(SYREN_DELAY_USEC);
			omap_writew((value << 6) | (addr << 1) | SYREN_READ_REG, SPI_TX_MSB);
		}
	}

	ret = omap_readw(SPI_RX_LSB) >> 6;
	syren_spi_unlock();
	return ret;

bail:
	syren_spi_unlock();
	return 0;
}

#define SPI_TAS (OMAP730_ICR_BASE+0x12)
#define OMAP730_UPLD_PCC_PERIPH_CLOCK_SOURCE_SEL	(OMAP730_PCC_UPLD_CTRL_BASE+0x108)

void syren_spi_init(void)
{
	/* p.647 - Get Syren SPI output signals - MCUDI, MCUDO, MCUEN */
    set_bit_range_32(OMAP730_IO_CONF_2, 3, 1, 0);

	/* p.1643 - syren slicer output XXX don't understand this */
	set_bit_range_32(OMAP730_UPLD_PCC_PERIPH_CLOCK_SOURCE_SEL, 9, 9, 0);
}

void syren_spi_lock(void)
{
	int timeout;
	u_short val;

	/* p.619 - poll the SPI TAS to claim the SYREN SPI interface */
	for (timeout = 0; timeout < 100000; timeout++) {
		if ((val = omap_readw(SPI_TAS)) & 0x1)
			return;
	}

	printk("syren_spi_lock: timed out! last=%04x\n", val);
	// *(char *)0 = 0; /* crash */
}

void syren_spi_unlock(void)
{
	omap_writew(0x0001, SPI_TAS);
}

