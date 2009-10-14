/*
 * File: drivers/video/omap/lcd-wizard.c
 *
 * LCD panel support for the HTC Wizard
 *
 * Copyright (C) linwizard.sourceforge.net
 * Author: Angelo Arrifano <miknix@gmail.com>
 * Based on lcd_h4 by Imre Deak <imre.deak@nokia.com>
 * Based on htcartemis_lcd.c by Vivien Chappelier <vivien.chappelier@free.fr>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/arch/gpio.h>
#include <asm/arch/omapfb.h>

#define HTCWIZARD_GPIO_PANEL_ID0 44 /* LCD Panel identification */
#define HTCWIZARD_GPIO_PANEL_ID1 55 /* LCD Panel identification */

static int htcwizard_panel_init(struct lcd_panel *panel,
                                struct omapfb_device *fbdev)
{
	return 0;
}

static void htcwizard_panel_cleanup(struct lcd_panel *panel)
{
}

static int htcwizard_panel_enable(struct lcd_panel *panel)
{

	return 0;
}

static void htcwizard_panel_disable(struct lcd_panel *panel)
{
}

static unsigned long htcwizard_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

/* Unknown */
struct lcd_panel htcwizard_panel = {
	.name		= "lcd_wiz",
	.config		= OMAP_LCDC_PANEL_TFT |
	              OMAP_LCDC_INV_VSYNC |
	              OMAP_LCDC_INV_HSYNC |
	              OMAP_LCDC_HSVS_OPPOSITE,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 240,
	.y_res		= 320,
	.pixel_clock	= 5500,
	.hsw		= 10,
	.hfp		= 10,
	.hbp		= 20,
	.vsw		= 2,
	.vfp		= 2,
	.vbp		= 3,

	.init		= htcwizard_panel_init,
	.cleanup	= htcwizard_panel_cleanup,
	.enable		= htcwizard_panel_enable,
	.disable	= htcwizard_panel_disable,
	.get_caps	= htcwizard_panel_get_caps,
};

/* Found on WIZ200 (miknix) */
struct lcd_panel htcwizard_panel3 = {
	.name		= "lcd_wiz3",
	.config		= OMAP_LCDC_PANEL_TFT |
	              OMAP_LCDC_INV_HSYNC |
					  OMAP_LCDC_INV_PIX_CLOCK,
	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 240,
	.y_res		= 320,
	.pixel_clock = 6093,
	.hsw		= 10,
	.hfp		= 10,
	.hbp		= 20,
	.vsw		= 3,
	.vfp		= 2,
	.vbp		= 2,

	.init		= htcwizard_panel_init,
	.cleanup	= htcwizard_panel_cleanup,
	.enable		= htcwizard_panel_enable,
	.disable	= htcwizard_panel_disable,
	.get_caps	= htcwizard_panel_get_caps,
};

/* Found on HTC Elfs */
struct lcd_panel htcwizard_panel_elf = {
	.name		= "lcd_elf",
	.config		= OMAP_LCDC_PANEL_TFT |
	              OMAP_LCDC_INV_HSYNC |
					  OMAP_LCDC_INV_PIX_CLOCK,
	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 240,
	.y_res		= 320,
	.pcd = 18,
	.hsw		= 4,
	.hfp		= 8,
	.hbp		= 4,
	.vsw		= 3,
	.vfp		= 8,
	.vbp		= 5,

	.init		= htcwizard_panel_init,
	.cleanup	= htcwizard_panel_cleanup,
	.enable		= htcwizard_panel_enable,
	.disable	= htcwizard_panel_disable,
	.get_caps	= htcwizard_panel_get_caps,
};


static int htcwizard_panel_probe(struct platform_device *pdev)
{
	int paneltype;

	omap_set_gpio_direction(HTCWIZARD_GPIO_PANEL_ID0, 1);
	omap_set_gpio_direction(HTCWIZARD_GPIO_PANEL_ID1, 1);

	paneltype = (omap_get_gpio_datain(HTCWIZARD_GPIO_PANEL_ID1) << 1) |
	            omap_get_gpio_datain(HTCWIZARD_GPIO_PANEL_ID0);

	printk("LCD: Panel type (%d)\n", paneltype);
	
	if (paneltype == 3)
		omapfb_register_panel(&htcwizard_panel3);
	else
		omapfb_register_panel(&htcwizard_panel);

	return 0;
}

static int htcwizard_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int htcwizard_panel_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int htcwizard_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver htcwizard_panel_driver = {
	.probe		= htcwizard_panel_probe,
	.remove		= htcwizard_panel_remove,
	.suspend	= htcwizard_panel_suspend,
	.resume		= htcwizard_panel_resume,
	.driver		= {
		.name	= "lcd_htcwizard",
		.owner	= THIS_MODULE,
	},
};

static int htcwizard_panel_drv_init(void)
{
	return platform_driver_register(&htcwizard_panel_driver);
}

static void htcwizard_panel_drv_cleanup(void)
{
	platform_driver_unregister(&htcwizard_panel_driver);
}

module_init(htcwizard_panel_drv_init);
module_exit(htcwizard_panel_drv_cleanup);

