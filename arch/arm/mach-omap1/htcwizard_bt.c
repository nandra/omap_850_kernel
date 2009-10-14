/* Phone interface driver for GPS SIRF-III
 *
 * 2007-12-12 Robert Jarzmik
 *
 * This code is licenced under the GPLv2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>
#include <asm/arch/omap850.h>

static int status=0; /* Default bt off */
static int pm=1; /* Default to power management */

void bt_init_gpios()
{
	// Configure gpios
	/* configure bt UART clock*/
	//omap_writel(omap_readl(0xfffe0834) | 0x200, 0xfffe0834);
}

#define EGPIO_5_0_BT 22

void bt_on()
{
	/* configure SIRF.III GPIOS */
//	gpio_set_value(EGPIO_5_0_BT,1);
        gpio_set_value(125,1);
	mdelay(1000);
	/* configure bt UART clock*/
	omap_writel(omap_readl(0xfffe0834) | 0x200, 0xfffe0834);
	
	gpio_set_value(42,0);
	gpio_set_value(43,0);
	gpio_set_value(40,0);
	mdelay(100);
	gpio_set_value(40,1);
	gpio_set_value(42,1);
	gpio_set_value(43,1);
	printk("HTC Artemis Bluetooth: Bluetooth turned on.\n");

}

void bt_off()
{
	/* configure bt UART clock*/
	omap_writel(omap_readl(0xfffe0834) & 0xfffffdff, 0xfffe0834);
	/* configure SIRF.III GPIOS */
        gpio_set_value(125,0);
//	gpio_set_value(EGPIO_5_0_BT,0);
	gpio_set_value(42,0);
	gpio_set_value(43,0);
	mdelay(40);
	gpio_set_value(42,1);
	gpio_set_value(43,1);
	printk("HTC Artemis Bluetooth: Bluetooth turned off.\n");
}

void bt_reset()
{
	bt_off();
	bt_on();
}

static void htcartemis_bt_configure(int state)
{
	switch (state) {
	
	case 0:
		bt_off();
		status = 0;
		break;

	case 1:
                bt_on();
		status = 1;
		break;
	default:
		break;
	}
}


static ssize_t power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",status); 
}

static ssize_t power_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
        unsigned int power;
        power = simple_strtoul(buf, NULL, 10);
        status = power != 0 ? 1 : 0;
	htcartemis_bt_configure(status);
	return count;
}

static DEVICE_ATTR(bt_power, 0644, power_show, power_store);

static ssize_t pm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",pm); 
}

static ssize_t pm_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
        unsigned int pmng;
        pmng = simple_strtoul(buf, NULL, 10);
        pm = pmng != 0 ? 1 : 0;
	return count;
}

static DEVICE_ATTR(skip_pm, 0644, pm_show, pm_store);

static int htcartemis_bt_probe(struct platform_device *pdev)
{
	bt_init_gpios();
	device_create_file(&pdev->dev, &dev_attr_bt_power);
	device_create_file(&pdev->dev, &dev_attr_skip_pm);
	return 0;
}

static int htcartemis_bt_remove(struct platform_device *pdev)
{
	bt_off();
	device_remove_file(&pdev->dev, &dev_attr_bt_power);
	device_remove_file(&pdev->dev, &dev_attr_skip_pm);
	return 0;
}

static int htcartemis_bt_suspend(struct platform_device *dev, pm_message_t state)
{
	if (!pm)
		bt_off();
	return 0;
}

static int htcartemis_bt_resume(struct platform_device *dev, pm_message_t state)
{
	if (!pm)
	    if (status)
		bt_on();
	return 0;
}

static struct platform_driver bt_driver = {
	.driver   = {
		.name     = "htcartemis-bt",
	},
	.probe    = htcartemis_bt_probe,
	.remove   = htcartemis_bt_remove,
	.suspend  = htcartemis_bt_suspend,
	.resume   = htcartemis_bt_resume,
};

static int __init htcartemis_bt_init(void)
{
	printk(KERN_NOTICE "HTC Artemis Bluetooth Driver\n");
	platform_driver_register( &bt_driver );
        return 0;
}

static void __exit htcartemis_bt_exit(void)
{
	platform_driver_unregister( &bt_driver );
}

module_init(htcartemis_bt_init);
module_exit(htcartemis_bt_exit);

MODULE_AUTHOR("Crohas Fabrice");
MODULE_DESCRIPTION("HTC Artemis Bluetooth Support Driver");
MODULE_LICENSE("GPL");
