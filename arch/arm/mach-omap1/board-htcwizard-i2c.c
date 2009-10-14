/*
 * linux/arch/arm/mach-omap1/board-htcwizard-i2c.c
 *
 * Copyright (C) 2008 linwizard.sourceforge.net
 * Copyright (C) 2008 Angelo Arrifano <miknix@gmail.com>
 *
 * This  program is  free  software; you  can  redistribute it  and/or
 * modify  it under the  terms of  the GNU  General Public  License as
 * published by the Free Software  Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT  ANY  WARRANTY;  without   even  the  implied  warranty  of
 * MERCHANTABILITY or  FITNESS FOR A PARTICULAR PURPOSE.   See the GNU
 * General Public License for more details.
 * 
 * You should have  received a copy of the  GNU General Public License
 * along  with  this program;  if  not,  write  to the  Free  Software
 * Foundation,  Inc.,  51 Franklin  Street,  Fifth  Floor, Boston,  MA
 * 02110-1301, USA. 
 *
 */

#include <asm/arch/common.h>
#include <asm/arch/gpio.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/i2c-htcpld.h>

#define HTCWIZARD_GIRQ_BTNS 141

/* 
 * CPLD Logic
 *

Address 0x03
----------------+--------------------------
Function/Bits   | 7  6  5  4     3  2  1  0
DPAD Light 	    | x  x  x  x     x  x  x  1
SoundDev        | x  x  x  x     1  x  x  x

Address 0x04
----------------+--------------------------
Function/Bits   | 7  6  5  4     3  2  1  0
Keyboard Light  | x  x  x  x     x  x  x  1
LCD Brigh. 4    | x  x  x  x     1  1  1  x
LCD Brigh. 3    | x  x  x  x     1  0  1  x
LCD Brigh. 2    | x  x  x  x     1  1  0  x
LCD Brigh. 1    | x  x  x  x     1  0  0  x
LCD Brigh. 0    | x  x  x  x     0  0  0  x
LCD FB          | 1  1  1  1     x  x  x  x
LCD white       | 0  0  0  0     x  x  x  x

Address 0x05
----------------+--------------------------
Function/Bits   | 7  6  5  4     3  2  1  0
Low bat LED     | x  x  x  0     x  1  x  x
Bat charged LED | x  x  x  1     x  0  x  x
Bat charging LED| x  x  x  1     x  1  x  x
Bluetooth LED 	 | x  1  x  x     x  x  x  x
Wifi LED        | x  x  1  x     x  x  x  x
GSM LED         | x  x  x  x     1  x  x  x
GSM Event LED 	 | x  x  x  x     x  x  1  x
Bit 4: USB Power plugged in
Bit 2: Battery not full

Address 0x06
----------------+--------------------------
Function/Bits   | 7  6  5  4     3  2  1  0
Vibrator        | x  x  x  x     1  x  x  x
Camera light    | x  x  1  x     x  x  x  x
DKB & DPAD OFF  | x  x  x  x     x  x  1  x
*/

struct htcpld_chip_platform_data i2c_chip3 = {
	.reset = 0xf6,
};
struct htcpld_chip_platform_data i2c_chip4 = {
	.reset = 0xfa,
};
struct htcpld_chip_platform_data i2c_chip5 = {
	.reset = 0x81,
};
struct htcpld_chip_platform_data i2c_chip6 = {
	.reset = 0xd5,
};

static struct i2c_board_info __initdata htcwizard_i2cboardinfo[] = {
	{
		.driver_name = "i2c-htcpld",
		.addr = 0x03,
		.platform_data = &i2c_chip3,
	},
	{
		.driver_name = "i2c-htcpld",
		.addr = 0x04,
		.platform_data = &i2c_chip4,
	},
	{
		.driver_name = "i2c-htcpld",
		.addr = 0x05,
		.platform_data = &i2c_chip5,
	},
	{
		.driver_name = "i2c-htcpld",
		.addr = 0x06,
		.platform_data = &i2c_chip6,
	},
};

static struct resource i2c_btns_resources[] = {
	[0] = {
		.start  = OMAP_GPIO_IRQ(HTCWIZARD_GIRQ_BTNS),
		.end    = OMAP_GPIO_IRQ(HTCWIZARD_GIRQ_BTNS),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct htcpld_btns_platform_data i2c_btns_pfdata = {
	.poll_interval = 5,
};

static struct platform_device i2c_btns_pfdevice = {
	.name           = "i2c-htcpld-btns",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(i2c_btns_resources),
	.resource       = i2c_btns_resources,
	.dev	 = {
		.platform_data	= &i2c_btns_pfdata,
	},
};

static struct platform_device i2c_leds_pfdevice = {
	.name = "leds-htcwiz",
	.id	= -1,
};

static struct platform_device *i2c_devices[] __initdata = {
 	&i2c_btns_pfdevice,
	&i2c_leds_pfdevice,
};

#if defined(CONFIG_I2C_OMAP) || defined(CONFIG_I2C_OMAP_MODULE)

void __init htcwizard_i2c_init(void)
{
	platform_add_devices(i2c_devices, ARRAY_SIZE(i2c_devices));

	/* Use I2C_SCK and I2_SDA */
	omap_writel(omap_readl(OMAP850_IO_CONF_5) & ~0x000000FF, OMAP850_IO_CONF_5);
	
	omap_register_i2c_bus(1, 100, htcwizard_i2cboardinfo,
	                      ARRAY_SIZE(htcwizard_i2cboardinfo));
}

#else

static struct i2c_gpio_platform_data i2cgpio_pfdata = {
	.sda_pin		= 69,
	.scl_pin		= 70,
	.udelay		= 2,		/* ~100 kHz */
};

static struct platform_device i2cgpio_device = {
	.name			= "i2c-gpio",
	.id			= -1,
	.dev	 = {
		.platform_data	= &i2cgpio_pfdata,
	},
};

void __init htcwizard_i2c_init(void)
{
	platform_add_devices(i2c_devices, ARRAY_SIZE(i2c_devices));

	/* Set GPIOS 70 and 69 */
	omap_writel(omap_readl(OMAP850_IO_CONF_5) |  0x000000CC, OMAP850_IO_CONF_5);
	omap_writel(omap_readl(OMAP850_IO_CONF_5) & ~0x00000033, OMAP850_IO_CONF_5);

	platform_device_register(&i2cgpio_device);
	i2c_register_board_info(0, htcwizard_i2cboardinfo,
	                        ARRAY_SIZE(htcwizard_i2cboardinfo));
}

#endif


