#ifndef __LINUX_I2C_HTCPLD_H
#define __LINUX_I2C_HTCPLD_H

struct htcpld_chip_platform_data {
	u8 reset;
};

struct htcpld_btns_platform_data {
	int poll_interval;
};

u8 htcpld_chip_get(u8 chip_addr);
void htcpld_chip_set(u8 chip_addr, u8 val);
void htcpld_chip_reset(u8 chip_addr);

#endif /* __LINUX_I2C_HTCPLD_H */
