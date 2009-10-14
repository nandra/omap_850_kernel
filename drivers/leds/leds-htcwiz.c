/*
 * LEDs driver for the htc-i2c-cpld interface.
 *
 * Copyright 2008 (C) Angelo Arrifano <miknix@gmail.com>
 *  - Update for using the i2c-htcpld driver interface.
 * Copyright 2008 (C) Justin Verel <justin@marverinc.nl>
 *
 *	based on leds-tosa.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <linux/i2c/i2c-htcpld.h>

static void ledshtcwiz_batcharging_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) | 0x14);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xeb);
}
static struct led_classdev ledshtcwiz_batcharging_led = {
	.name					= "htcwiz:bat-charging",
	.brightness_set	= ledshtcwiz_batcharging_set,
};


static void ledshtcwiz_batcharged_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, (htcpld_chip_get(0x05) | 0x10) & 0xfb);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xeb);
}
static struct led_classdev ledshtcwiz_batcharged_led = {
	.name					= "htcwiz:bat-charged",
	.brightness_set	= ledshtcwiz_batcharged_set,
};


static void ledshtcwiz_batlow_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, (htcpld_chip_get(0x05) | 0x04) & 0xef);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xeb);
}
static struct led_classdev ledshtcwiz_batlow_led = {
	.name					= "htcwiz:bat-low",
	.brightness_set	= ledshtcwiz_batlow_set,
};


static void ledshtcwiz_wifi_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) | 0x20);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xdf);
}
static struct led_classdev ledshtcwiz_wifi_led = {
	.name					= "htcwiz:wifi",
	.brightness_set	= ledshtcwiz_wifi_set,
};


static void ledshtcwiz_bluetooth_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) | 0x40);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xbf);
}
static struct led_classdev ledshtcwiz_bluetooth_led = {
	.name					= "htcwiz:bluetooth",
	.brightness_set	= ledshtcwiz_bluetooth_set,
};


static void ledshtcwiz_gsm_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) | 0x08);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xf7);
}
static struct led_classdev ledshtcwiz_gsm_led = {
	.name					= "htcwiz:gsm",
	.brightness_set	= ledshtcwiz_gsm_set,
	.default_trigger  = "heartbeat",
};


static void ledshtcwiz_gsmevent_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) | 0x02);
	else
		htcpld_chip_set(0x05, htcpld_chip_get(0x05) & 0xfd);
}
static struct led_classdev ledshtcwiz_gsmevent_led = {
	.name					= "htcwiz:gsm-event",
	.brightness_set	= ledshtcwiz_gsmevent_set,
};


static void ledshtcwiz_vibrator_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x06, htcpld_chip_get(0x06) | 0x08);
	else
		htcpld_chip_set(0x06, htcpld_chip_get(0x06) & 0xf7);
}
static struct led_classdev ledshtcwiz_vibrator = {
	.name					= "htcwiz:vibrator",
	.brightness_set	= ledshtcwiz_vibrator_set,
};


static void ledshtcwiz_camera_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x06, htcpld_chip_get(0x06) | 0x20);
	else
		htcpld_chip_set(0x06, htcpld_chip_get(0x06) & 0xdf);
}
static struct led_classdev ledshtcwiz_camera_led = {
	.name					= "htcwiz:camera",
	.brightness_set	= ledshtcwiz_camera_set,
};


static void ledshtcwiz_dpad_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x03, htcpld_chip_get(0x03) | 0x01);
	else
		htcpld_chip_set(0x03, htcpld_chip_get(0x03) & 0xfe);
}
static struct led_classdev ledshtcwiz_dpad_led = {
	.name					= "htcwiz:dpad",
	.brightness_set	= ledshtcwiz_dpad_set,
};


static void ledshtcwiz_keyboard_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value)
		htcpld_chip_set(0x04, htcpld_chip_get(0x04) | 0x01);
	else
		htcpld_chip_set(0x04, htcpld_chip_get(0x04) & 0xfe);
}
static struct led_classdev ledshtcwiz_keyboard_led = {
	.name					= "htcwiz:keyboard",
	.brightness_set	= ledshtcwiz_keyboard_set,
};


static int ledshtcwiz_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_batcharging_led);
	if (ret < 0)
		return ret;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_batcharged_led);
	if (ret < 0)
		goto undo_batcharged;
	
	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_batlow_led);
	if (ret < 0)
		goto undo_batlow;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_wifi_led);
	if (ret < 0)
		goto undo_wifi;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_bluetooth_led);
	if (ret < 0)
		goto undo_bluetooth;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_gsm_led);
	if (ret < 0)
		goto undo_gsm;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_gsmevent_led);
	if (ret < 0)
		goto undo_gsmevent;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_vibrator);
	if (ret < 0)
		goto undo_vibrator;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_camera_led);
	if (ret < 0)
		goto undo_camera;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_dpad_led);
	if (ret < 0)
		goto undo_dpad;

	ret = led_classdev_register(&pdev->dev, &ledshtcwiz_keyboard_led);
	if (ret < 0)
		goto undo_keyboard;

	return ret;

undo_keyboard:
		led_classdev_unregister(&ledshtcwiz_dpad_led);
undo_dpad:
		led_classdev_unregister(&ledshtcwiz_camera_led);
undo_camera:
		led_classdev_unregister(&ledshtcwiz_vibrator);
undo_vibrator:
		led_classdev_unregister(&ledshtcwiz_gsmevent_led);
undo_gsmevent:
		led_classdev_unregister(&ledshtcwiz_gsm_led);
undo_gsm:
		led_classdev_unregister(&ledshtcwiz_bluetooth_led);
undo_bluetooth:
		led_classdev_unregister(&ledshtcwiz_wifi_led);
undo_wifi:
		led_classdev_unregister(&ledshtcwiz_batlow_led);
undo_batlow:
		led_classdev_unregister(&ledshtcwiz_batcharged_led);
undo_batcharged:
		led_classdev_unregister(&ledshtcwiz_batcharging_led);

	return ret;
}

static int ledshtcwiz_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&ledshtcwiz_keyboard_led);
	led_classdev_unregister(&ledshtcwiz_dpad_led);
	led_classdev_unregister(&ledshtcwiz_camera_led);
	led_classdev_unregister(&ledshtcwiz_vibrator);
	led_classdev_unregister(&ledshtcwiz_gsmevent_led);
	led_classdev_unregister(&ledshtcwiz_gsm_led);
	led_classdev_unregister(&ledshtcwiz_bluetooth_led);
	led_classdev_unregister(&ledshtcwiz_wifi_led);
	led_classdev_unregister(&ledshtcwiz_batlow_led);
	led_classdev_unregister(&ledshtcwiz_batcharged_led);
	led_classdev_unregister(&ledshtcwiz_batcharging_led);
	return 0;
}

#ifdef CONFIG_PM
static int ledshtcwiz_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&ledshtcwiz_keyboard_led);
	led_classdev_suspend(&ledshtcwiz_dpad_led);
	led_classdev_suspend(&ledshtcwiz_camera_led);
	led_classdev_suspend(&ledshtcwiz_vibrator);
	led_classdev_suspend(&ledshtcwiz_gsmevent_led);
	led_classdev_suspend(&ledshtcwiz_gsm_led);
	led_classdev_suspend(&ledshtcwiz_bluetooth_led);
	led_classdev_suspend(&ledshtcwiz_wifi_led);
	led_classdev_suspend(&ledshtcwiz_batlow_led);
	led_classdev_suspend(&ledshtcwiz_batcharged_led);
	led_classdev_suspend(&ledshtcwiz_batcharging_led);
	return 0;
}

static int ledshtcwiz_resume(struct platform_device *dev)
{
	led_classdev_resume(&ledshtcwiz_keyboard_led);
	led_classdev_resume(&ledshtcwiz_dpad_led);
	led_classdev_resume(&ledshtcwiz_camera_led);
	led_classdev_resume(&ledshtcwiz_vibrator);
	led_classdev_resume(&ledshtcwiz_gsmevent_led);
	led_classdev_resume(&ledshtcwiz_gsm_led);
	led_classdev_resume(&ledshtcwiz_bluetooth_led);
	led_classdev_resume(&ledshtcwiz_wifi_led);
	led_classdev_resume(&ledshtcwiz_batlow_led);
	led_classdev_resume(&ledshtcwiz_batcharged_led);
	led_classdev_resume(&ledshtcwiz_batcharging_led);
	return 0;
}
#endif


static struct platform_driver ledshtcwiz_driver = {
	.probe		= ledshtcwiz_probe,
	.remove		= ledshtcwiz_remove,
#ifdef CONFIG_PM
	.suspend    = ledshtcwiz_suspend,
	.resume		= ledshtcwiz_resume,
#endif
	.driver		= {
		.name		= "leds-htcwiz",
		.owner		= THIS_MODULE,
	},
};

static int __init ledshtcwiz_init(void)
{
	printk("leds-htcwiz: HTC WIZARD LEDs driver\n");
	return platform_driver_register(&ledshtcwiz_driver);
}

static void __exit ledshtcwiz_exit(void)
{
 	platform_driver_unregister(&ledshtcwiz_driver);
}

module_init(ledshtcwiz_init);
module_exit(ledshtcwiz_exit);

MODULE_AUTHOR("Justin Verel <justin@marverinc.nl>");
MODULE_DESCRIPTION("HTC Wizard LEDs");
MODULE_LICENSE("GPL");
