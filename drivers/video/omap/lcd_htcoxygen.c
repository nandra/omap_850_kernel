/*
 * File: drivers/video/omap/lcd_htcoxygen.c
 * 
 *
 * LCD panel support for the HTC Oxygen
 * modified from lcd-wizard.c
 *
 * Copyright (C) linwizard.sourceforge.net
 * Copyright (C) 2009 Marek Belisko <marek.belisko@open-nandra
 * Author: Angelo Arrifano <miknix@gmail.com>
 * Based on lcd_h4 by Imre Deak <imre.deak@nokia.com>
 * Based on htcartemis_lcd.c by Vivien Chappelier <vivien.chap
 *
 * This program is free software; you can redistribute it and/
 * under the terms of the GNU General Public License as publis
 * Free Software Foundation; either version 2 of the License, 
 * option) any later version.
 *
 * This program is distributed in the hope that it will be use
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See t
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public L
 * with this program; if not, write to the Free Software Found
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/arch/omapfb.h>

static int htcoxygen_panel_init(struct lcd_panel *panel,
                                struct omapfb_device *fbdev)
{
	return 0;
}

static void htcoxygen_panel_cleanup(struct lcd_panel *panel)
{
}

static int htcoxygen_panel_enable(struct lcd_panel *panel)
{

	return 0;
}

static void htcoxygen_panel_disable(struct lcd_panel *panel)
{
}

static unsigned long htcoxygen_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}


struct lcd_panel htcoxygen_panel = {
	.name		= "lcd_oxygen",
	.config		= OMAP_LCDC_PANEL_TFT |
	              OMAP_LCDC_INV_VSYNC |
	              OMAP_LCDC_INV_HSYNC |
	              OMAP_LCDC_HSVS_OPPOSITE,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 176,
	.y_res		= 220,
	.pixel_clock	= 3362,
	.hsw		= 10,
	.hfp		= 45,
	.hbp		= 9,
	.vsw		= 3,
	.vfp		= 3,
	.vbp		= 3,
	.pcd            = 0,

	.init		= htcoxygen_panel_init,
	.cleanup	= htcoxygen_panel_cleanup,
	.enable		= htcoxygen_panel_enable,
	.disable	= htcoxygen_panel_disable,
	.get_caps	= htcoxygen_panel_get_caps,
};

static int htcoxygen_panel_probe(struct platform_device *pdev)
{
	
	omapfb_register_panel(&htcoxygen_panel);
	
	return 0;
}

static int htcoxygen_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int htcoxygen_panel_suspend(struct platform_device *pde)
{
	return 0;
}

static int htcoxygen_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver htcoxygen_panel_driver = {
	.probe		= htcoxygen_panel_probe,
	.remove		= htcoxygen_panel_remove,
	.suspend	= htcoxygen_panel_suspend,
	.resume		= htcoxygen_panel_resume,
	.driver		= {
		.name	= "lcd_htcoxygen",
		.owner	= THIS_MODULE,
	},
};

static int htcoxygen_panel_drv_init(void)
{
	return platform_driver_register(&htcoxygen_panel_driver);
}

static void htcoxygen_panel_drv_cleanup(void)
{
	platform_driver_unregister(&htcoxygen_panel_driver);
}

module_init(htcoxygen_panel_drv_init);
module_exit(htcoxygen_panel_drv_cleanup);