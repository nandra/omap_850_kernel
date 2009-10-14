/*
 *  i2c-htcpld.c
 *  Chip driver for a unknown cpld found on HTC boards with omap850.
 *  The cpld is located on the i2c bus and controls backlight, leds,
 *  vibrator and other power devices. The cpld also returns buttons status
 *  of the directional pads found on this HTC devices.
 *
 *  Copyright (C) 2008 Angelo Arrifano <miknix@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <linux/i2c/i2c-htcpld.h>

#define HTCPLD_ADDR_MAX 16

struct htcpld_chip_data {
	u8 reset;
	u8 cache;
};

static struct i2c_client *htcpld_chip[HTCPLD_ADDR_MAX]; 
static struct i2c_client *htcpld_dkbd_chip;
static struct i2c_client *htcpld_dpad_chip;

static const unsigned char htcpld_btns_map[] = {
	KEY_RIGHT,
	KEY_UP,
	KEY_LEFT,
	KEY_DOWN,
	KEY_ENTER
};

struct htcpld_btns_dev {
	unsigned char keycode[ARRAY_SIZE(htcpld_btns_map)];
	char dpad[ARRAY_SIZE(htcpld_btns_map)];
	char dkbd[ARRAY_SIZE(htcpld_btns_map)];	
	int irq;
	int poll_interval;
	struct input_dev *input;
	struct timer_list timer;
};

void htcpld_chip_set(u8 chip_addr, u8 val)
{
	struct htcpld_chip_data *chip_data;

	if (chip_addr > HTCPLD_ADDR_MAX)
		return;

	if (! htcpld_chip[chip_addr])
		return;

	chip_data = i2c_get_clientdata(htcpld_chip[chip_addr]);
	if (! chip_data)
		return;
	
	i2c_smbus_read_byte_data(htcpld_chip[chip_addr],
	                         (chip_data->cache = val));
}

u8 htcpld_chip_get(u8 chip_addr)
{
	struct htcpld_chip_data *chip_data;

	if (! htcpld_chip[chip_addr])
		return 0;

	chip_data = i2c_get_clientdata(htcpld_chip[chip_addr]);
	if (! chip_data)
		return 0;
	
	return chip_data->cache;
}


void htcpld_chip_reset(u8 chip_addr)
{
	struct htcpld_chip_data *chip_data;

	if (chip_addr > HTCPLD_ADDR_MAX)
		return;

	if (! htcpld_chip[chip_addr])
		return;

	chip_data = i2c_get_clientdata(htcpld_chip[chip_addr]);
	if (! chip_data)
		return;

	i2c_smbus_read_byte_data(htcpld_chip[chip_addr],
	  (chip_data->cache = chip_data->reset));
}


/*
 * Driver handling
 */

static inline u8 _htcpld_chip_cache_read(struct i2c_client *client)
{
	return i2c_smbus_read_byte_data(client,
	  ((struct htcpld_chip_data *)i2c_get_clientdata(client))->cache);
}

static int htcpld_probe(struct i2c_client *client)
{
	struct htcpld_chip_data *chip_data;
	struct device *dev = &client->dev;
	struct htcpld_chip_platform_data *pdata;

	if (!dev)
		return -ENODEV;

	pdata = (struct htcpld_chip_platform_data *)dev->platform_data;
	if (!pdata) {
		printk("Platform data not found for chip at 0x%x!\n", client->addr);
		return -ENXIO;
	}

	if (!i2c_check_functionality(client->adapter,
	                             I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -ENODEV;

	if (client->addr > HTCPLD_ADDR_MAX) {
		printk("Address above range.\n");
		return -ENOMEM;
	}

	if (htcpld_chip[client->addr]) {
		printk("Address already on use.\n");
		return -EINVAL;
	}

	chip_data = kzalloc(sizeof(struct htcpld_chip_data), GFP_KERNEL);
	if (! chip_data)
		return -ENOMEM;
	
	chip_data->reset = pdata->reset;	
	i2c_set_clientdata(client, chip_data);
	snprintf(client->name, I2C_NAME_SIZE, "Chip_0x%x", client->addr);
	htcpld_chip[client->addr] = client;

	/* Even addresses are read as directional pad events,
	 *  odd addresses as directional keyboard events. */
	if (client->addr % 2 == 0)
		htcpld_dpad_chip = client;
	else
		htcpld_dkbd_chip = client;

	printk("i2c-htcpld: Detected chip at 0x%x\n", client->addr);
	htcpld_chip_reset(client->addr);

	return 0;
}

static int htcpld_remove(struct i2c_client *client)
{
	htcpld_chip[client->addr] = NULL;
	kfree(i2c_get_clientdata(client));

	return 0;
}

static irqreturn_t htcpld_btns_irq(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct device *dev = &pdev->dev;
	struct htcpld_btns_dev *bdev = dev_get_drvdata(dev);
	struct htcpld_btns_platform_data *pdata = dev->platform_data;
	u8 dpad_status, dkbd_status;
	int i;
	
	dkbd_status = _htcpld_chip_cache_read(htcpld_dkbd_chip);
	dpad_status = _htcpld_chip_cache_read(htcpld_dpad_chip);

	/* Send key press events */
	for (i=0; i<ARRAY_SIZE(htcpld_btns_map); i++) {
		if (!((dpad_status >> (7 - i)) & 0x01)) {
			bdev->dpad[i] = 1;
			input_report_key(bdev->input, bdev->keycode[i], 1);
		}
		if (!((dkbd_status >> (7 - i)) & 0x01)) {
			bdev->dkbd[i] = 1;
			input_report_key(bdev->input, bdev->keycode[i], 1);	
		}
	}
	input_sync(bdev->input);

	/* Poll for key release */
	/* FIXME: We really should disable this irq before polling */
	mod_timer(&bdev->timer, jiffies + pdata->poll_interval);

	return IRQ_HANDLED;
}

static void htcpld_btns_poll(unsigned long data)
{
	struct platform_device *pdev = (struct platform_device *)data;
	struct device *dev = &pdev->dev;
	struct htcpld_btns_dev *bdev = dev_get_drvdata(dev);
	struct htcpld_btns_platform_data *pdata = dev->platform_data;
	u8 dpad_status, dkbd_status;
	int i, we_continue = 0;
	
	dkbd_status = _htcpld_chip_cache_read(htcpld_dkbd_chip);
	dpad_status = _htcpld_chip_cache_read(htcpld_dpad_chip);

	/* Send key release events */
	for (i=0; i<ARRAY_SIZE(htcpld_btns_map); i++) {
		if (bdev->dpad[i]) {
			if ((dpad_status >> (7 - i)) & 0x01) {
				bdev->dpad[i] = 0;
				input_report_key(bdev->input, bdev->keycode[i], 0);
			} else {
				we_continue = 1;
			}
		}
		if (bdev->dkbd[i]) {
			if ((dkbd_status >> (7 - i)) & 0x01) {
				bdev->dkbd[i] = 0;
				input_report_key(bdev->input, bdev->keycode[i], 0);	
			} else
				we_continue = 1;
		}
	}
	input_sync(bdev->input);

	if (we_continue) {
		/* Poll again */
		mod_timer(&bdev->timer, jiffies + pdata->poll_interval);
		return;
	}
}

static int __devinit htcpld_btns_probe(struct platform_device *pdev)
{
	struct htcpld_btns_dev *bdev;
	struct htcpld_btns_platform_data *pdata;
	struct input_dev *input;
	struct resource *res;
	int error, i;

	if (!htcpld_dkbd_chip || !htcpld_dpad_chip)
		return -ENODEV;

	bdev = kzalloc(sizeof(struct htcpld_btns_dev), GFP_KERNEL);
	input = input_allocate_device();
	if (!bdev || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	platform_set_drvdata(pdev, bdev);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		printk("Platform data not found for buttons device!\n");
		error = -ENXIO;
		goto err_free_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		error = -EBUSY;
		goto err_free_mem;
	}

	bdev->irq = res->start;
	bdev->input = input;
	bdev->poll_interval = HZ / pdata->poll_interval;
	memcpy(bdev->keycode, htcpld_btns_map, sizeof(bdev->keycode));

	input->name = "HTC I2C CPLD Buttons";
	input->keycode = bdev->keycode;
	input->keycodemax = ARRAY_SIZE(bdev->keycode);
	input->keycodesize = sizeof(unsigned char);

	set_bit(EV_KEY, input->evbit);
	set_bit(EV_REP, input->evbit);
	for (i = 0; i < ARRAY_SIZE(htcpld_btns_map); i++)
		set_bit(bdev->keycode[i], input->keybit);

	error = input_register_device(input);
	if (error)
		goto err_free_mem;

	setup_timer(&bdev->timer, htcpld_btns_poll, (unsigned long)pdev);

	error = request_irq(bdev->irq, htcpld_btns_irq,
	         IRQF_TRIGGER_FALLING | IRQF_SAMPLE_RANDOM, pdev->name, pdev);
	if (error)
		goto err_unreg;

	printk("i2c-htcpld: HTC I2C CPLD Buttons\n");
	printk("i2c-htcpld: Using IRQ: %d\n", bdev->irq);
	printk("i2c-htcpld: DKbd chip at 0x%x\n", htcpld_dkbd_chip->addr);
	printk("i2c-htcpld: DPad chip at 0x%x\n", htcpld_dpad_chip->addr);

	return 0;

 err_unreg:
	input_unregister_device(input);
 err_free_mem:
	input_free_device(input);
	kfree(bdev);
	dev_set_drvdata(&pdev->dev, NULL);
	return error;
}

static int __devexit htcpld_btns_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct htcpld_btns_dev *bdev = dev_get_drvdata(dev);

	del_timer_sync(&bdev->timer);
	input_unregister_device(bdev->input);
	input_free_device(bdev->input);
	free_irq(bdev->irq, pdev);
	kfree(bdev);
	dev_set_drvdata(dev, NULL);

	return 0;
}

static struct i2c_driver htcpld_driver = {
	.driver = {
		.name	= "i2c-htcpld",
	},
	.probe	= htcpld_probe,
	.remove	= htcpld_remove,
};

static struct platform_driver htcpld_btns_driver = {
	.probe	= htcpld_btns_probe,
	.remove	= __devexit_p(htcpld_btns_remove),
	.driver	= {
		.name	= "i2c-htcpld-btns",
		.owner	= THIS_MODULE,
	},
};

static int __init htcpld_init(void)
{
	int ret;	

	ret = i2c_add_driver(&htcpld_driver);
	if (ret)
		return ret;	
		
	ret = platform_driver_register(&htcpld_btns_driver);
	if (ret)
		goto fail_drv_register;

	return 0;

fail_drv_register:
	i2c_del_driver(&htcpld_driver);

	return ret;
}

static void __exit htcpld_exit(void)
{
	i2c_del_driver(&htcpld_driver);

	platform_driver_unregister(&htcpld_btns_driver);
}

module_init(htcpld_init);
module_exit(htcpld_exit);

EXPORT_SYMBOL_GPL(htcpld_chip_set);
EXPORT_SYMBOL_GPL(htcpld_chip_get);
EXPORT_SYMBOL_GPL(htcpld_chip_reset);

MODULE_AUTHOR("Angelo Arrifano <miknix@gmail.com>");
MODULE_DESCRIPTION("HTC I2C CPLD Driver");
MODULE_LICENSE("GPL");

