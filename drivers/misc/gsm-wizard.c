/*
 *  gsm-wizard.c
 *  This driver provides support for the GSM module on the
 *  HTC Wizard.
 *
 *  Copyright (C) 2008 Justin Verel <justin@marverinc.nl>
 *  Copyright (C) 2009 Alistair Buxton <a.j.buxton@gmail.com>
 *
 *  Based on gsm-typhoon.c
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/err.h>

#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include <asm/uaccess.h>
#include <asm/arch/omap850.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/tc.h>

#define FW_CODE_NAME "gsmcode.bin"
#define FW_DATA_NAME "gsmdata.bin"

/*
 * DSP MMU
 */
#define DSPMMU_PREFETCH                 (OMAP850_DSP_MMU_BASE + 0x00)
#define DSPMMU_WALKING_ST               (OMAP850_DSP_MMU_BASE + 0x04)
#define DSPMMU_CNTL                     (OMAP850_DSP_MMU_BASE + 0x08)
#define DSPMMU_FAULT_AD_H               (OMAP850_DSP_MMU_BASE + 0x0c)
#define DSPMMU_FAULT_AD_L               (OMAP850_DSP_MMU_BASE + 0x10)
#define DSPMMU_FAULT_ST                 (OMAP850_DSP_MMU_BASE + 0x14)
#define DSPMMU_IT_ACK                   (OMAP850_DSP_MMU_BASE + 0x18)
#define DSPMMU_TTB_H                    (OMAP850_DSP_MMU_BASE + 0x1c)
#define DSPMMU_TTB_L                    (OMAP850_DSP_MMU_BASE + 0x20)
#define DSPMMU_LOCK                     (OMAP850_DSP_MMU_BASE + 0x24)
#define DSPMMU_LD_TLB                   (OMAP850_DSP_MMU_BASE + 0x28)
#define DSPMMU_CAM_H                    (OMAP850_DSP_MMU_BASE + 0x2c)
#define DSPMMU_CAM_L                    (OMAP850_DSP_MMU_BASE + 0x30)
#define DSPMMU_RAM_H                    (OMAP850_DSP_MMU_BASE + 0x34)
#define DSPMMU_RAM_L                    (OMAP850_DSP_MMU_BASE + 0x38)
#define DSPMMU_GFLUSH                   (OMAP850_DSP_MMU_BASE + 0x3c)
#define DSPMMU_FLUSH_ENTRY              (OMAP850_DSP_MMU_BASE + 0x40)
#define DSPMMU_READ_CAM_H               (OMAP850_DSP_MMU_BASE + 0x44)
#define DSPMMU_READ_CAM_L               (OMAP850_DSP_MMU_BASE + 0x48)
#define DSPMMU_READ_RAM_H               (OMAP850_DSP_MMU_BASE + 0x4c)
#define DSPMMU_READ_RAM_L               (OMAP850_DSP_MMU_BASE + 0x50)

#define DSPMMU_CNTL_BURST_16MNGT_EN     0x0020
#define DSPMMU_CNTL_WTL_EN              0x0004
#define DSPMMU_CNTL_MMU_EN              0x0002
#define DSPMMU_CNTL_RESET_SW            0x0001

#define DSPMMU_FAULT_AD_H_DP            0x0100
#define DSPMMU_FAULT_AD_H_ADR_MASK      0x00ff

#define DSPMMU_FAULT_ST_PREF            0x0008
#define DSPMMU_FAULT_ST_PERM            0x0004
#define DSPMMU_FAULT_ST_TLB_MISS        0x0002
#define DSPMMU_FAULT_ST_TRANS           0x0001

#define DSPMMU_IT_ACK_IT_ACK            0x0001

#define DSPMMU_LOCK_BASE_MASK           0xfc00
#define DSPMMU_LOCK_BASE_SHIFT          10
#define DSPMMU_LOCK_VICTIM_MASK         0x03f0
#define DSPMMU_LOCK_VICTIM_SHIFT        4

#define DSPMMU_CAM_H_VA_TAG_H_MASK              0x0003

#define DSPMMU_CAM_L_VA_TAG_L1_MASK             0xc000
#define DSPMMU_CAM_L_VA_TAG_L2_MASK_1MB         0x0000
#define DSPMMU_CAM_L_VA_TAG_L2_MASK_64KB        0x3c00
#define DSPMMU_CAM_L_VA_TAG_L2_MASK_4KB         0x3fc0
#define DSPMMU_CAM_L_VA_TAG_L2_MASK_1KB         0x3ff0
#define DSPMMU_CAM_L_P                          0x0008
#define DSPMMU_CAM_L_V                          0x0004
#define DSPMMU_CAM_L_SLST_MASK                  0x0003
#define DSPMMU_CAM_L_SLST_1MB                   0x0000
#define DSPMMU_CAM_L_SLST_64KB                  0x0001
#define DSPMMU_CAM_L_SLST_4KB                   0x0002
#define DSPMMU_CAM_L_SLST_1KB                   0x0003

#define DSPMMU_RAM_L_RAM_LSB_MASK       0xfc00
#define DSPMMU_RAM_L_AP_MASK            0x0300
#define DSPMMU_RAM_L_AP_NA              0x0000
#define DSPMMU_RAM_L_AP_RO              0x0200
#define DSPMMU_RAM_L_AP_FA              0x0300

#define DSPMMU_GFLUSH_GFLUSH            0x0001

#define DSPMMU_FLUSH_ENTRY_FLUSH_ENTRY  0x0001

#define DSPMMU_LD_TLB_RD                0x0002
#define DSPMMU_LD_TLB_LD                0x0001

#define WIZARD_GSM_CODE_PHYS           0x13c00000
#define WIZARD_GSM_CODE_SIZE           0x002C0000
#define WIZARD_GSM_DATA_PHYS           0x13f10000
#define WIZARD_GSM_DATA_SIZE           0x00004000
#define WIZARD_FIFO_BASE_PHYS          0x13f00000

// port usage enum
#define CSMI_PORT_INDEX_SYSTEM		0			//mailbox
#define CSMI_PORT_INDEX_GSMCTRL		1			//mailbox
#define CSMI_PORT_INDEX_OTP		2			//mailbox
#define CSMI_PORT_INDEX_UNUSED_1	2			//mailbox
#define CSMI_PORT_INDEX_UNUSED_2	3			//mailbox
#define CSMI_PORT_INDEX_LOW_BATTERY_WARNING	3		
#define CSMI_PORT_INDEX_UNUSED_3	4			//mailbox
#define CSMI_PORT_INDEX_AT_1		5			//stream
#define CSMI_PORT_INDEX_AT_2		6			//stream
#define CSMI_PORT_INDEX_TRACE		7			//stream
#define CSMI_PORT_INDEX_PACKET		8			//stream
#define CSMI_PORT_INDEX_AUDIO		9			//stream

#define NR_PORTS		10
#define NR_STREAM_PORTS		5
#define FIRST_STREAM_PORT 	5
#define NR_TTY_PORTS		5
#define FIRST_TTY_PORT		5

#define FIFO_SIZE       0x1000
#define FIFO_MASK       (FIFO_SIZE - 1)

#define PORT_CTRL_BASE                  0xfffbbc00
#define PORT_CTRL_SIZE                  0x10
#define PORT_CTRL(port)	(PORT_CTRL_BASE + ((port) * PORT_CTRL_SIZE))

#define FIFO_CTRL_TX_HW_PTR_LO          0x00
#define FIFO_CTRL_TX_HW_PTR_HI          0x02
#define FIFO_CTRL_RX_HW_PTR_LO          0x04
#define FIFO_CTRL_RX_HW_PTR_HI          0x06
#define FIFO_CTRL_RX_SW_PTR_LO          0x08
#define FIFO_CTRL_RX_SW_PTR_HI          0x0a
#define FIFO_CTRL_TX_SW_PTR_LO          0x0c
#define FIFO_CTRL_TX_SW_PTR_HI          0x0e

/* inter system communication */
#define ICR_MPU2GSM     (OMAP850_ICR_BASE + 0x00)
#define ICR_GSM2MPU     (OMAP850_ICR_BASE + 0x02)
#define ICR_MPUCTRL     (OMAP850_ICR_BASE + 0x04)
#define ICR_GSMCTRL     (OMAP850_ICR_BASE + 0x06)
#define ICR_CODEBASE    (OMAP850_ICR_BASE + 0x0a)
#define ICR_DATABASE    (OMAP850_ICR_BASE + 0x0c)
#define ICR_RAMBASE     (OMAP850_ICR_BASE + 0x0e)
#define ICR_LOCK_OFFSET (OMAP850_ICR_BASE + 0x10)
#define ICR_SPITAS      (OMAP850_ICR_BASE + 0x12)

#define INT_STATUS_CTRL    (1 << 1)
#define INT_STATUS_PORT(port) (1 << (port))

/* MPU Control register */
#define MPUCTRL_BLOCK_SIZE_64K  0UL
#define MPUCTRL_BLOCK_SIZE_128K 1UL
#define MPUCTRL_BLOCK_SIZE_256K 2UL
#define MPUCTRL_BLOCK_SIZE_512K 3UL
#define MPUCTRL_BLOCK_SIZE_1M   4UL
#define MPUCTRL_BLOCK_SIZE_2M   5UL
#define MPUCTRL_BLOCK_SIZE_4M   6UL
#define MPUCTRL_BLOCK_SIZE_8M   7UL

#define MPUCTRL_GSM_INTERRUPT_ENABLE    (1UL << 11)
#define MPUCTRL_MPU_INTERRUPT_ENABLE    (1UL << 10)
#define MPUCTRL_RAM_BLOCK_SIZE(x)       ((x) << 7)
#define MPUCTRL_DATA_BLOCK_SIZE(x)      ((x) << 4)
#define MPUCTRL_CODE_BLOCK_SIZE(x)      ((x) << 1)
#define MPUCTRL_GSM_ENABLE              (1UL << 0)

/* GSM control register */
#define GSMCTRL_GSM_INTERRUPT_ENABLE    (1UL << 2)
#define GSMCTRL_MPU_INTERRUPT_ENABLE    (1UL << 1)

/* Sound GPIO */
#define WIZARD_SNDDEV_GPIO     176

struct csmi_stream_priv {
	void *fifo_base;
	int port;
        struct mutex io_mutex_read;
        struct mutex io_mutex_write;
        struct mutex io_mutex_open_close;
        struct completion io_comp_read;
        struct completion io_comp_write;
        spinlock_t fifo_lock;
	void *data;
};

static void csmi_tty_n_routine(unsigned long arg);
DECLARE_TASKLET(csmi_5_tasklet, csmi_tty_n_routine, 0);
DECLARE_TASKLET(csmi_6_tasklet, csmi_tty_n_routine, 1);
DECLARE_TASKLET(csmi_7_tasklet, csmi_tty_n_routine, 2);
DECLARE_TASKLET(csmi_8_tasklet, csmi_tty_n_routine, 3);
DECLARE_TASKLET(csmi_9_tasklet, csmi_tty_n_routine, 4);

struct csmi_priv {
	void *fifo_base;
	struct csmi_stream_priv csmi_ttys[NR_TTY_PORTS];
	struct tasklet *tasklets[NR_PORTS];	
};

static struct csmi_priv csmi_priv_data;

/* module params */

int noreset = 0;
module_param(noreset, int, 0);
MODULE_PARM_DESC(noreset, "Do not attempt to reset the DSP. Implies not loading any firmware.");

int loadcode = 0;
module_param(loadcode, int, 0);
MODULE_PARM_DESC(noload, "Lode the GSM code (aka the radio rom.)");

int loaddata = 0;
module_param(loaddata, int, 0);
MODULE_PARM_DESC(noload, "Load the GSM data (unique to the phone.)");


enum gsm_audio_path {
        GSM_AUDIO_HANDHOLD = 0,
        GSM_AUDIO_SPEAKER = 1,
        GSM_AUDIO_HEADPHONE = 2,
	GSM_AUDIO_CARKIT = 3,
	GSM_AUDIO_BLUETOOTH = 4,
	GSM_PATH_BLUETOOTH = 5,
        GSM_PATH_VSP = 6,
        GSM_PATH_DAI_ENABLE = 7,
        GSM_PATH_DAI_DISABLE = 0xe0,
        GSM_MMI_ACE_TEST = 0xff,
};

/* FIFO Code
 * FIFOs are only available for ports 5-9 (stream ports)
 */

/* initialize the DSP<->CPU fifos */
static int fifo_init(struct csmi_priv *priv)
{
	priv->fifo_base = ioremap_nocache(WIZARD_FIFO_BASE_PHYS, 
					NR_STREAM_PORTS * 2 * FIFO_SIZE);

	if (!priv->fifo_base) {
		printk(KERN_ERR "CSMI: Could not remap FIFO shared memory buffers.\n");
		return -ENOMEM;
	}

	memset_io(priv->fifo_base, 0, NR_STREAM_PORTS * 2 * FIFO_SIZE);

	return 0;
}

static void fifo_cleanup(struct csmi_priv *priv)
{
	// TODO: check for error
        if (priv->fifo_base)
                iounmap(priv->fifo_base);
}

/* poll a DSP<->CPU fifo */
static int fifo_poll_read(struct csmi_stream_priv *priv)
{
        void *base;
        void *ctrl;
        int hw_ptr, sw_ptr;
        int avail;
        unsigned long flags;

	base = (void *) priv->fifo_base;
        ctrl = (void *) PORT_CTRL(priv->port);

        spin_lock_irqsave(&priv->fifo_lock, flags);
        hw_ptr = (omap_readw(ctrl + FIFO_CTRL_RX_HW_PTR_HI) << 16) |
  	          omap_readw(ctrl + FIFO_CTRL_RX_HW_PTR_LO);
        sw_ptr = (omap_readw(ctrl + FIFO_CTRL_RX_SW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_RX_SW_PTR_LO);
        spin_unlock_irqrestore(&priv->fifo_lock, flags);

	avail = (hw_ptr + FIFO_SIZE - sw_ptr) & FIFO_MASK;

        return avail;
}

static int fifo_poll_write(struct csmi_stream_priv *priv)
{
        void *base;
        void *ctrl;
        int hw_ptr, sw_ptr;
        int avail;
        unsigned long flags;

	base = (void *) priv->fifo_base + FIFO_SIZE;
        ctrl = (void *) PORT_CTRL(priv->port);

        spin_lock_irqsave(&priv->fifo_lock, flags);
        hw_ptr = (omap_readw(ctrl + FIFO_CTRL_TX_HW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_TX_HW_PTR_LO);
        sw_ptr = (omap_readw(ctrl + FIFO_CTRL_TX_SW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_TX_SW_PTR_LO);
        spin_unlock_irqrestore(&priv->fifo_lock, flags);

        avail = (hw_ptr + FIFO_SIZE - 1 - sw_ptr) & FIFO_MASK;

        return avail;
}

/* read from a DSP<->CPU fifo */
static int fifo_read(struct csmi_stream_priv *priv, void *data, int len)
{
        void *base;
        void *ctrl;
        int hw_ptr, sw_ptr;
        int avail;
        unsigned long flags;

	base = (void *) priv->fifo_base;
        ctrl = (void *) PORT_CTRL(priv->port);

	spin_lock_irqsave(&priv->fifo_lock, flags);
        hw_ptr = (omap_readw(ctrl + FIFO_CTRL_RX_HW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_RX_HW_PTR_LO);
        sw_ptr = (omap_readw(ctrl + FIFO_CTRL_RX_SW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_RX_SW_PTR_LO);
        spin_unlock_irqrestore(&priv->fifo_lock, flags);

        avail = (hw_ptr + FIFO_SIZE - sw_ptr) & FIFO_MASK;

	if (len > avail)
 	       len = avail;

        if (sw_ptr + len > FIFO_SIZE) 
	{
		memcpy_fromio(data, base + sw_ptr, FIFO_SIZE - sw_ptr);
                memcpy_fromio(data + (FIFO_SIZE - sw_ptr), base,
                              len - (FIFO_SIZE - sw_ptr));
	} else {
                memcpy_fromio(data, base + sw_ptr, len);
        }

	sw_ptr += len;
        sw_ptr &= FIFO_MASK;

	spin_lock_irqsave(&priv->fifo_lock, flags);
        omap_writew(sw_ptr & 0xffff, ctrl + FIFO_CTRL_RX_SW_PTR_LO);
        omap_writew(sw_ptr >> 16,    ctrl + FIFO_CTRL_RX_SW_PTR_HI);
        omap_writew(INT_STATUS_PORT(priv->port), ICR_MPU2GSM);
        spin_unlock_irqrestore(&priv->fifo_lock, flags);

	return len;
}

/* write to a DSP<->CPU fifo */
static int fifo_write(struct csmi_stream_priv *priv, const void *data, int len)
{
        void *base;
        void *ctrl;
        int hw_ptr, sw_ptr;
        int avail;
        unsigned long flags;
					
	base = (void *) priv->fifo_base + FIFO_SIZE;
        ctrl = (void *) PORT_CTRL(priv->port);
					
        spin_lock_irqsave(&priv->fifo_lock, flags);
        hw_ptr = (omap_readw(ctrl + FIFO_CTRL_TX_HW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_TX_HW_PTR_LO);
        sw_ptr = (omap_readw(ctrl + FIFO_CTRL_TX_SW_PTR_HI) << 16) |
                  omap_readw(ctrl + FIFO_CTRL_TX_SW_PTR_LO);
        spin_unlock_irqrestore(&priv->fifo_lock, flags);
														    
        avail = (hw_ptr + FIFO_SIZE - 1 - sw_ptr) & FIFO_MASK;
															    
        if (len > avail)
	        len = avail;
									    
        if (sw_ptr + len > FIFO_SIZE) {
                memcpy_toio(base + sw_ptr, data, FIFO_SIZE - sw_ptr);
	        memcpy_toio(base, data + (FIFO_SIZE - sw_ptr),
                len - (FIFO_SIZE - sw_ptr));
        } else {
                memcpy_toio(base + sw_ptr, data, len);
        }

	sw_ptr += len;
        sw_ptr &= FIFO_MASK;
	
	spin_lock_irqsave(&priv->fifo_lock, flags);
        omap_writew(sw_ptr & 0xffff, ctrl + FIFO_CTRL_TX_SW_PTR_LO);
        omap_writew(sw_ptr >> 16,    ctrl + FIFO_CTRL_TX_SW_PTR_HI);
        omap_writew(INT_STATUS_PORT(priv->port), ICR_MPU2GSM);
        spin_unlock_irqrestore(&priv->fifo_lock, flags);
																			
	return len;
}

/* End - FIFO Code
 */

#define SYREN_SPI_BASE          0xfffc1800
#define SYREN_SPI_SETUP1        (SYREN_SPI_BASE + 0x00)
#define SYREN_SPI_SETUP2        (SYREN_SPI_BASE + 0x02)
#define SYREN_SPI_CTRL          (SYREN_SPI_BASE + 0x04)
#define SYREN_SPI_STATUS        (SYREN_SPI_BASE + 0x06)
#define SYREN_SPI_TX_LSB        (SYREN_SPI_BASE + 0x08)
#define SYREN_SPI_TX_MSB        (SYREN_SPI_BASE + 0x0a)
#define SYREN_SPI_RX_LSB        (SYREN_SPI_BASE + 0x0c)
#define SYREN_SPI_RX_MSB        (SYREN_SPI_BASE + 0x0e)

#define SPI_SETUP1_INT_READ_ENABLE      (1UL << 5)
#define SPI_SETUP1_INT_WRITE_ENABLE     (1UL << 4)
#define SPI_SETUP1_CLOCK_DIVISOR(x)     ((x) << 1)
#define SPI_SETUP1_CLOCK_ENABLE         (1UL << 0)

#define SPI_SETUP2_ACTIVE_EDGE_FALLING  (0UL << 0)
#define SPI_SETUP2_ACTIVE_EDGE_RISING   (1UL << 0)
#define SPI_SETUP2_NEGATIVE_LEVEL       (0UL << 5)
#define SPI_SETUP2_POSITIVE_LEVEL       (1UL << 5)
#define SPI_SETUP2_LEVEL_TRIGGER        (0UL << 10)
#define SPI_SETUP2_EDGE_TRIGGER         (1UL << 10)

#define SPI_CTRL_SEN(x)                 ((x) << 7)
#define SPI_CTRL_WORD_SIZE(x)           (((x) - 1) << 2)
#define SPI_CTRL_WR                     (1UL << 1)
#define SPI_CTRL_RD                     (1UL << 0)

#define SPI_STATUS_WE                   (1UL << 1)
#define SPI_STATUS_RD                   (1UL << 0)

#define SYREN_WRITE 0
#define SYREN_READ  1

/* Syren registers are split in two pages.
 *    [0x01] = page selection (1 or 2)
 *    [0x1e]
 *    [0x1f]
*/

static void syren_spi_write_reg(int reg, int data)
{
        unsigned int val;

        /* enable SPI */
        val = omap_readw(SYREN_SPI_SETUP1);
        val |= SPI_SETUP1_CLOCK_ENABLE;
        omap_writew(val, SYREN_SPI_SETUP1);

	/* write 16-bit word */
        omap_writew((data << 6) | (reg << 1) | SYREN_WRITE, SYREN_SPI_TX_MSB);
        omap_writew(0, SYREN_SPI_TX_LSB);
        omap_writew(SPI_CTRL_SEN(0) |
                    SPI_CTRL_WORD_SIZE(16) |
                    SPI_CTRL_WR,
                    SYREN_SPI_CTRL);

	while((omap_readw(SYREN_SPI_STATUS) & SPI_STATUS_WE) != SPI_STATUS_WE);
        udelay(1000);

	/* disable SPI */
        val = omap_readw(SYREN_SPI_SETUP1);
        val &= ~SPI_SETUP1_CLOCK_ENABLE;
        omap_writew(val, SYREN_SPI_SETUP1);
}

static int syren_spi_read_reg(int reg)
{       
        unsigned int val;
        int data;

        /* enable SPI */
        val = omap_readw(SYREN_SPI_SETUP1);
        val |= SPI_SETUP1_CLOCK_ENABLE;
        omap_writew(val, SYREN_SPI_SETUP1);

        /* read/write 16-bit word */
        omap_writew((reg << 1) | SYREN_READ, SYREN_SPI_TX_MSB);
        omap_writew(0, SYREN_SPI_TX_LSB);

	omap_writew(SPI_CTRL_SEN(0) |
		    SPI_CTRL_WORD_SIZE(16) |
                    SPI_CTRL_RD,
                    SYREN_SPI_CTRL);

        while((omap_readw(SYREN_SPI_STATUS) & SPI_STATUS_RD) != SPI_STATUS_RD);
        udelay(1000);

        /* read 16-bit word */
        omap_writew((reg << 1) | SYREN_READ, SYREN_SPI_TX_MSB);
        omap_writew(0, SYREN_SPI_TX_LSB);

        omap_writew(SPI_CTRL_SEN(0) |
                    SPI_CTRL_WORD_SIZE(16) |
                    SPI_CTRL_RD,
                    SYREN_SPI_CTRL);

        while((omap_readw(SYREN_SPI_STATUS) & SPI_STATUS_RD) != SPI_STATUS_RD);
        udelay(1000);

	data = omap_readw(SYREN_SPI_RX_LSB);

        /* disable SPI */
        val = omap_readw(SYREN_SPI_SETUP1);
        val &= ~SPI_SETUP1_CLOCK_ENABLE;
        omap_writew(val, SYREN_SPI_SETUP1);

        return data >> 6;
}

static void syren_spi_open(void)
{       
        /* get control of SPI */
        while (omap_readw(ICR_SPITAS) == 0);

        /* configure clock and interrupts */
        omap_writew(SPI_SETUP1_INT_READ_ENABLE |
                    SPI_SETUP1_INT_WRITE_ENABLE |
                    SPI_SETUP1_CLOCK_DIVISOR(0), SYREN_SPI_SETUP1);
        omap_writew(SPI_SETUP2_ACTIVE_EDGE_RISING |
                    SPI_SETUP2_NEGATIVE_LEVEL |
                    SPI_SETUP2_LEVEL_TRIGGER, SYREN_SPI_SETUP2);
}

static void syren_spi_close(void)
{       
        /* reset to page 0 */
        syren_spi_write_reg(0x01, 0x0001);

        /* release SPI */
        omap_writew(0, ICR_SPITAS);
}

#define GSM_CTRL_BASE                   0xfffbbc10
#define GSM_CTRL_RESP_LO                (GSM_CTRL_BASE + 0x00)
#define GSM_CTRL_RESP_HI                (GSM_CTRL_BASE + 0x02)
#define GSM_CTRL_STATUS_LO              (GSM_CTRL_BASE + 0x04)
#define GSM_CTRL_STATUS_HI              (GSM_CTRL_BASE + 0x06)
#define GSM_CTRL_COMMAND_LO             (GSM_CTRL_BASE + 0x08)
#define GSM_CTRL_COMMAND_HI             (GSM_CTRL_BASE + 0x0a)
#define GSM_CTRL_ARG_LO                 (GSM_CTRL_BASE + 0x0c)
#define GSM_CTRL_ARG_HI                 (GSM_CTRL_BASE + 0x0e)

/* gsm control commands */
#define GSM_CTRL_CMD_READ_AUDIO_PATH    0x17
#define GSM_CTRL_CMD_WRITE_AUDIO_PATH   0x18
#define GSM_CTRL_CMD_READ_SYREN         0x19
#define GSM_CTRL_CMD_WRITE_SYREN        0x20
#define GSM_CTRL_CMD_READ_UNK0          0x21
#define GSM_CTRL_CMD_WRITE_UNK0         0x22
#define GSM_CTRL_CMD_WRITE_ABB          0x23
#define GSM_CTRL_CMD_READ_ABB           0x24
#define GSM_CTRL_CMD_READ_UNK2          0x25
#define GSM_CTRL_CMD_WRITE_UNK2         0x26
#define GSM_CTRL_CMD_READ_UNK1          0x27
#define GSM_CTRL_CMD_WRITE_UNK1         0x28

/* expected return codes */
#define GSM_CTRL_ACK_READ_AUDIO_PATH    0x1008
#define GSM_CTRL_ACK_WRITE_AUDIO_PATH   0x1009
#define GSM_CTRL_ACK_READ_SYREN         0x100a
#define GSM_CTRL_ACK_WRITE_SYREN        0x100b
#define GSM_CTRL_ACK_READ_UNK0          0x100c
#define GSM_CTRL_ACK_WRITE_UNK0         0x100d
#define GSM_CTRL_ACK_WRITE_ABB          0x100e
#define GSM_CTRL_ACK_READ_ABB           0x100f
#define GSM_CTRL_ACK_READ_UNK2          0x1010
#define GSM_CTRL_ACK_WRITE_UNK2         0x1011
#define GSM_CTRL_ACK_READ_UNK1          0x1012
#define GSM_CTRL_ACK_WRITE_UNK1         0x1013

/* analog baseband registers */
#define ABB_REG_VBUCTRL 0x00
#define ABB_REG_VBDCTRL 0x01
#define ABB_REG_VBCTRL1 0x02
#define ABB_REG_VBCTRL2 0x03
#define ABB_REG_VAUOCR  0x04
#define ABB_REG_VAUSCR  0x05

/* Following functions are equivelent to csmi read/write mailbox.
 * Mailboxe ports are similar to stream ports in that they have 0x10 bytes
 * of shared data at PORT_CTRL(port). Unlike streams they have no FIFO
 * memory and the PORT_CTRL bytes are used arbitrarily
 */

static int gsm_ctrl_wait_resp(u32 resp)
{
        int ret;

        /* wait for ack */
        while (!(omap_readw(ICR_GSM2MPU) & INT_STATUS_CTRL))
        udelay(1000);

        ret = (omap_readw(GSM_CTRL_RESP_HI) << 16) |
               omap_readw(GSM_CTRL_RESP_LO);

	/* ack */
        omap_writew(INT_STATUS_CTRL, ICR_GSM2MPU);

        if (ret != resp) {
	        printk("unexpected answer %04x\n", ret);
                return -1;
        }

        return 0;
}

static void gsm_ctrl_send_cmd(u32 cmd, u8 page, u8 reg, u16 data)
{
        /* send command */
        omap_writew(cmd & 0xffff, GSM_CTRL_COMMAND_LO);
        omap_writew(cmd >> 16, GSM_CTRL_COMMAND_HI);
        omap_writew((reg << 8) | page, GSM_CTRL_ARG_LO);
        omap_writew(data, GSM_CTRL_ARG_HI);

        /* signal change */
        omap_writew(INT_STATUS_CTRL, ICR_MPU2GSM);
}

static u32 gsm_ctrl_read_status(void)
{
        return (omap_readw(GSM_CTRL_STATUS_HI) << 16) |
                omap_readw(GSM_CTRL_STATUS_LO);
}

static u16 abb_read_reg(u8 reg)
{       
        /* send read ABB register command */
        gsm_ctrl_send_cmd(GSM_CTRL_CMD_READ_ABB, 1, reg, 0);

        /* wait answer */
        gsm_ctrl_wait_resp(GSM_CTRL_ACK_READ_ABB);

	return (u16) gsm_ctrl_read_status();
}

static void abb_write_reg(u8 reg, u16 val)
{       
        /* send write ABB register command */
        gsm_ctrl_send_cmd(GSM_CTRL_CMD_WRITE_ABB, 1, reg, val);

        /* wait answer */
        gsm_ctrl_wait_resp(GSM_CTRL_ACK_WRITE_ABB);
}

static u16 syren_read_reg(u8 page, u8 reg)
{       
        /* send read syren register command */
        gsm_ctrl_send_cmd(GSM_CTRL_CMD_READ_SYREN, page, reg, 0);

        /* wait answer */
        gsm_ctrl_wait_resp(GSM_CTRL_ACK_READ_SYREN);

	return (u16) gsm_ctrl_read_status();
}

static void syren_write_reg(u8 page, u8 reg, u16 val, u16 mask)
{
#define GSM_SYREN_BASE                  0xfffbbd10
#define GSM_SYREN_MASK                  (GSM_SYREN_BASE + 0x08)
#define GSM_SYREN_DATA                  (GSM_SYREN_BASE + 0x0c)
        /* set mask */
        omap_writew(mask, GSM_SYREN_MASK);
        omap_writew(val,  GSM_SYREN_DATA);

        /* send write syren register command */
        gsm_ctrl_send_cmd(GSM_CTRL_CMD_WRITE_SYREN, page, reg, val);

        /* wait answer */
        gsm_ctrl_wait_resp(GSM_CTRL_ACK_WRITE_SYREN);
}

static void typhoon_gsm_set_audio_2(void)
{       
        int low;
        u16 syren6;
        u16 syren7;

        syren6 = syren_read_reg(0, 6);
        syren6 = 0x25 | (syren6 & 0xff00);

	abb_write_reg(ABB_REG_VBDCTRL, syren6);
	
	syren_write_reg(0, 6, syren6, 0);
	syren6 = syren_read_reg(0, 6);

        syren7 = syren_read_reg(0, 7);
        syren7 = 0xd3;
//      syren7 = 0x1e6;

        abb_write_reg(ABB_REG_VBUCTRL, syren7);

        syren_write_reg(0, 7, syren7, 0);
        syren7 = syren_read_reg(0, 7);

        low = 0x100;

        omap_writew(GSM_CTRL_CMD_WRITE_AUDIO_PATH, GSM_CTRL_COMMAND_LO);
        omap_writew(                            0, GSM_CTRL_COMMAND_HI);
        omap_writew(                          low, GSM_CTRL_ARG_LO);
	omap_writew(                            0, GSM_CTRL_ARG_HI);

	/* signal change in the audio registers */
	omap_writew(INT_STATUS_CTRL, ICR_MPU2GSM);

	/* wait answer */
	gsm_ctrl_wait_resp(GSM_CTRL_ACK_WRITE_AUDIO_PATH);

	printk("typhoon_gsm_set_audio: done\n");
}

static void typhoon_gsm_set_audio(enum gsm_audio_path path)
{
        int low;

        syren_spi_open();

        switch(path) {
        case GSM_AUDIO_HANDHOLD: /* RECEIVE */
                low  = 0x0000;
                syren_spi_write_reg(0x01, 0x0002); /* page 1 */
                syren_spi_write_reg(0x08, 0x0100);
                syren_spi_write_reg(0x11, 0x0100);
                syren_spi_write_reg(0x0b, 0x0000);
		break;
        case GSM_AUDIO_SPEAKER: /* SPEAKER */
                low  = 0x0001;
                syren_spi_write_reg(0x01, 0x0002); /* page 1 */
                syren_spi_write_reg(0x08, 0x0100);
                syren_spi_write_reg(0x11, 0x0010);
                break;
        case GSM_AUDIO_HEADPHONE: /* EARPHONE */
                low  = 0x0002;
                syren_spi_write_reg(0x01, 0x0002); /* page 1 */
                syren_spi_write_reg(0x08, 0x0108);
                syren_spi_write_reg(0x11, 0x0005);
                break;
        case GSM_AUDIO_CARKIT: /* HEADSET */
                low  = 0x0003;
                syren_spi_write_reg(0x01, 0x0002); /* page 1 */
                syren_spi_write_reg(0x08, 0x0100);
                syren_spi_write_reg(0x11, 0x0005);
                break;
        case GSM_PATH_DAI_ENABLE:
                low  = 0x0107;
                syren_spi_write_reg(0x01, 0x0002); /* page 1 */
                syren_spi_write_reg(0x08, 0x0100);
                syren_spi_write_reg(0x11, 0x0010);
                break;
        case GSM_PATH_DAI_DISABLE:
                low  = 0x0007;                                                                                                                                    
                break;
        default:
                printk("typhoon_gsm_set_audio: unknown audio path\n");
                return;
        }

        syren_spi_close();

        omap_writew(GSM_CTRL_CMD_WRITE_AUDIO_PATH, GSM_CTRL_COMMAND_LO);
        omap_writew(                            0, GSM_CTRL_COMMAND_HI);
        omap_writew(                          low, GSM_CTRL_ARG_LO);
        omap_writew(                         0x56, GSM_CTRL_ARG_HI);

	/* signal change in the audio registers */
        omap_writew(INT_STATUS_CTRL, ICR_MPU2GSM);

        /* wait answer */
        gsm_ctrl_wait_resp(GSM_CTRL_ACK_WRITE_AUDIO_PATH);

        printk("typhoon_gsm_set_audio: done\n");
}

static irqreturn_t rf_interrupt(int irq, void *dev_id)
{
        /* XXX: what should we do here? */
	/*      printk(KERN_INFO "RF interrupt\n"); */

        return IRQ_HANDLED;
}

static irqreturn_t icr_interrupt(int irq, void *dev_id)
{
        int status;

        status = omap_readw(ICR_GSM2MPU);

	if (status & INT_STATUS_PORT(5)) {
		tasklet_schedule(&csmi_5_tasklet);
		omap_writew(INT_STATUS_PORT(5), ICR_GSM2MPU);
	}
	if (status & INT_STATUS_PORT(6)) {
		tasklet_schedule(&csmi_6_tasklet);
		omap_writew(INT_STATUS_PORT(6), ICR_GSM2MPU);
	}
	if (status & INT_STATUS_PORT(7)) {
		tasklet_schedule(&csmi_7_tasklet);
		omap_writew(INT_STATUS_PORT(7), ICR_GSM2MPU);
	}
	if (status & INT_STATUS_PORT(8)) {
		tasklet_schedule(&csmi_8_tasklet);
		omap_writew(INT_STATUS_PORT(8), ICR_GSM2MPU);
	}
	if (status & INT_STATUS_PORT(9)) {
		tasklet_schedule(&csmi_9_tasklet);
		omap_writew(INT_STATUS_PORT(9), ICR_GSM2MPU);
	}

	// TODO: handle the other interrupts
	omap_writew(0xFFFF, ICR_GSM2MPU);

	//printk("CSMI: Interrupt %x\n", status);
        return IRQ_HANDLED;
}


/*
 *  * TTY Driver
 *   */

static struct tty_driver *csmi_tty_driver;

static void csmi_tty_n_routine(unsigned long arg){
	//struct gsm_priv *priv = (struct gsm_priv *) &gsm;
	struct csmi_stream_priv *priv = &csmi_priv_data.csmi_ttys[arg];
	struct tty_struct *tty = (struct tty_struct *)priv->data;

	int len;
	u_char readbuf[FIFO_SIZE+1];

	if (mutex_lock_interruptible(&priv->io_mutex_read))
		return;

	/* see if there's anything to read */
	len = fifo_read(priv, readbuf, FIFO_SIZE);

	//printk("CSMI: Interrupt on port %lu, len = %d\n", arg, len);

	if(tty && len>0) {
		tty_insert_flip_string(tty, readbuf, len);
		tty_flip_buffer_push(tty);
	}

	mutex_unlock(&priv->io_mutex_read);
	return;
}

static void csmi_tty_put_char(struct tty_struct *tty, u_char ch) {}

static int csmi_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static void csmi_tty_flush_buffer(struct tty_struct *tty) {}
static inline void mpu_set_cflag(int cflag) {}
static void csmi_tty_set_termios(struct tty_struct *tty, struct ktermios *old) {}
static void csmi_tty_stop(struct tty_struct *tty) {}
static void csmi_tty_start(struct tty_struct *tty) {}
static void csmi_tty_wait_until_sent(struct tty_struct *tty, int timeout) {}
static void csmi_tty_send_xchar(struct tty_struct *tty, char ch) {}
static void csmi_tty_throttle(struct tty_struct *tty) {}
static void csmi_tty_unthrottle(struct tty_struct *tty) {}

static int csmi_tty_open(struct tty_struct *tty, struct file * filp)
{
	struct csmi_stream_priv *priv;

	if(tty->index >= NR_TTY_PORTS) {
		return -ENODEV;
	}

	priv = &csmi_priv_data.csmi_ttys[tty->index];

	// TODO: allow multiple accesses
	if(priv->data != NULL)
		return -EBUSY;
	priv->data = tty;
	tty->driver_data = priv;
	return 0;
}

static void csmi_tty_close(struct tty_struct *tty, struct file *filp) 
{
	struct csmi_stream_priv *priv = 
		(struct csmi_stream_priv *)tty->driver_data;
	tty->driver_data = NULL;
	priv->data = NULL;
}

static void csmi_tty_flush_chars(struct tty_struct *tty) {}
static void csmi_tty_hangup(struct tty_struct *tty) {}
static void csmi_tty_break(struct tty_struct *tty, int break_state) {}
static int csmi_tty_ioctl(struct tty_struct *tty, struct file * file,
		    unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int csmi_tty_read_proc(char *page, char **start, off_t off, int count,
			int *eof, void *data)
{
	//TODO: implement me
	return 0;
}

static int csmi_tty_write_room(struct tty_struct *tty)
{
	//TODO: implement me
	int free = FIFO_SIZE-1;
	return free;
}

static int csmi_tty_write(struct tty_struct *tty, const u_char *buf, int count)
{
	struct csmi_stream_priv *priv = 
		(struct csmi_stream_priv *)tty->driver_data;
        int len;

	if (mutex_lock_interruptible(&priv->io_mutex_read))
		return -ERESTARTSYS;	

	len = fifo_write(priv, buf, count);

	mutex_unlock(&priv->io_mutex_read);
        return len;

}

static int csmi_tty_tiocmget(struct tty_struct *tty, struct file *file) 
{
	return 0;
}

static int csmi_tty_tiocmset(struct tty_struct *tty, struct file *file,
		       unsigned int set, unsigned int clear) 
{
	return 0;
}


static const struct tty_operations csmi_tty_ops = {
	.open = csmi_tty_open,
	.close = csmi_tty_close,
	.write = csmi_tty_write,
	.put_char = csmi_tty_put_char,
	.flush_chars = csmi_tty_flush_chars,
	.write_room = csmi_tty_write_room,
	.chars_in_buffer = csmi_tty_chars_in_buffer,
	.flush_buffer = csmi_tty_flush_buffer,
	.ioctl = csmi_tty_ioctl,
	.throttle = csmi_tty_throttle,
	.unthrottle = csmi_tty_unthrottle,
	.set_termios = csmi_tty_set_termios,
	.stop = csmi_tty_stop,
	.start = csmi_tty_start,
	.hangup = csmi_tty_hangup,
	.break_ctl = csmi_tty_break,
	.send_xchar = csmi_tty_send_xchar,
	.wait_until_sent = csmi_tty_wait_until_sent,
	.read_proc = csmi_tty_read_proc,
	.tiocmget = csmi_tty_tiocmget,
	.tiocmset = csmi_tty_tiocmset,
};

/* END TTY Driver */

/* GSM-S Booting functions */

/* load the firmware to GSM reserved memory */
static int gsm_load_firmware(struct device *dev, char const *name,
                                     unsigned long addr, unsigned int size)
{
	int ret=0;
        const struct firmware *fw;
	void *ptr;

	/* ask firmware_class module to get the boot firmware off disk */
	ret = request_firmware(&fw, name, dev);
	if (ret < 0) { 
		printk("CSMI: %s load failed: %d\n", name, ret);
		goto err_req;
        }

        if (fw->size != size) {
	        printk("CSMI: incorrect firmware size (%d)\n", fw->size);
                ret = -EINVAL;
		goto err_size;
        }

        printk("CSMI: Loaded firmware %s (%zd bytes)\n", name, fw->size);

        ptr = ioremap(addr, size);

        if (ptr == NULL) {
	        printk("CSMI: Could not remap GSM region.\n");
                ret = -ENOMEM;
		goto err_size;
        }

        memcpy_toio(ptr, fw->data, fw->size);

        iounmap(ptr);

err_size:
        release_firmware(fw);
err_req:
        return ret;
}

/* load an entry in the DSP TLB */
static void gsm_load_tlb_entry(int entry,
			       unsigned long virt_addr,
			       unsigned long phys_addr)
{
	omap_writew(virt_addr >> 22, DSPMMU_CAM_H);
        omap_writew((virt_addr >> 6) &
 	       (DSPMMU_CAM_L_VA_TAG_L1_MASK |
                DSPMMU_CAM_L_VA_TAG_L2_MASK_1MB), DSPMMU_CAM_L);
        omap_writew(phys_addr >> 16, DSPMMU_RAM_H);
        omap_writew((phys_addr & DSPMMU_RAM_L_RAM_LSB_MASK) |
                     DSPMMU_RAM_L_AP_FA, DSPMMU_RAM_L);
        omap_writew((entry << DSPMMU_LOCK_BASE_SHIFT) |
                    (entry << DSPMMU_LOCK_VICTIM_SHIFT), DSPMMU_LOCK);
        omap_writew(DSPMMU_LD_TLB_LD, DSPMMU_LD_TLB);
}

/* put GSM into reset and reset all control registers */
static int gsm_reset(void)
{
        unsigned int val;
	void *ctrl_base;

        /* hold in reset */
        val = omap_readw(ICR_MPUCTRL);
        val &= ~MPUCTRL_GSM_ENABLE;
        omap_writew(val, ICR_MPUCTRL);

        /* turn clocks on */
	omap_writew(omap_readw(ARM_CKCTL) | 0x6000, ARM_CKCTL);
	omap_writew(0x1601, ARM_IDLECT1);

        /* disable GSM protection */
        omap_writew(omap_readw(0xfffe1800) & ~0x32000000, 0xfffe1800);

        /* disable MMU */
        val = omap_readw(DSPMMU_CNTL);
        val &= ~DSPMMU_CNTL_MMU_EN;
        omap_writew(val, DSPMMU_CNTL);

	/* mux */
        val = omap_readl(OMAP850_MODE_2);
        val |= 1 << 5;
        omap_writel(val, OMAP850_MODE_2);

        /* set code, data and ram address */
        omap_writew(0x00, ICR_CODEBASE);
        omap_writew(0x40, ICR_DATABASE);
        omap_writew(0x40, ICR_RAMBASE);

	/* reset MMU */
        val = omap_readw(DSPMMU_CNTL);
        val |= DSPMMU_CNTL_RESET_SW;
        omap_writew(val, DSPMMU_CNTL);

	/* enable MMU */
        val = omap_readw(DSPMMU_CNTL);
        val |= DSPMMU_CNTL_MMU_EN;
        omap_writew(val, DSPMMU_CNTL);

        /* load the DSP mmu with:
         *   wizard:
         *     0: 0x00000000 -> 0x13c00000
         *     1: 0x00100000 -> 0x13d00000
         *     2: 0x00200000 -> 0x13e00000
         *     3: 0x00400000 -> 0x13f00000
         */
        omap_writew(DSPMMU_GFLUSH_GFLUSH, DSPMMU_GFLUSH);
        gsm_load_tlb_entry(0, 0x00000000, 0x13c00000);
        gsm_load_tlb_entry(1, 0x00100000, 0x13d00000);
        gsm_load_tlb_entry(2, 0x00200000, 0x13e00000);
        gsm_load_tlb_entry(3, 0x00400000, 0x13f00000);

        /* setup DSP */
        omap_writew(MPUCTRL_CODE_BLOCK_SIZE(MPUCTRL_BLOCK_SIZE_4M) |
                    MPUCTRL_DATA_BLOCK_SIZE(MPUCTRL_BLOCK_SIZE_1M) |
                    MPUCTRL_RAM_BLOCK_SIZE(MPUCTRL_BLOCK_SIZE_1M) |
                    MPUCTRL_GSM_INTERRUPT_ENABLE |
                    MPUCTRL_MPU_INTERRUPT_ENABLE,
                    ICR_MPUCTRL);

	/* clear interrupts */
	omap_writew(0xffff, ICR_GSM2MPU);

	/* clear ctrl port shared ram */
	ctrl_base = ioremap_nocache(PORT_CTRL_BASE, 0x280);

	if(ctrl_base) {
		memset_io(ctrl_base, 0, 0x280);
		iounmap(ctrl_base);
	} else {
		printk("CSMI: Could not remap control port shared memory.\n");
		return -ENOMEM;
	}

#ifdef CONFIG_ARCH_OMAP850
	/* Select Internal DAGON - important if you want it to actually work */
	omap_writel((omap_readl(OMAP850_DAGON_MODE)) | 0x1, OMAP850_DAGON_MODE);
#endif
	return 0;
}

static int gsm_loadfw(struct device *dev) {
	int error;

	if(loadcode || loaddata)
		printk("CSMI: Loading firmware, please wait...");

	if (loadcode)
		if ((error = gsm_load_firmware(dev, FW_CODE_NAME, 
						WIZARD_GSM_CODE_PHYS, 
						WIZARD_GSM_CODE_SIZE)) < 0)
			return error;

	if (loaddata)
		if ((error = gsm_load_firmware(dev, FW_DATA_NAME, 
						WIZARD_GSM_DATA_PHYS, 
						WIZARD_GSM_DATA_SIZE)) < 0)
			return error;

	return 0;
}

/* take GSM out of reset  */
void gsm_boot(void) {
	unsigned int val;
        /* enable DSP */
        val = omap_readw(ICR_MPUCTRL);
        val |= MPUCTRL_GSM_ENABLE;
        omap_writew(val, ICR_MPUCTRL);
}

/* End GSM-S Booting functions */

static int __init wizard_gsm_probe(struct platform_device *pdev)
{
	struct csmi_priv *priv = &csmi_priv_data;
        struct device *dev;
        int error,n;

	dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	/* initialize the GSM<->CPU fifos */
        if((error = fifo_init(priv))) {
		goto err_fifo;
	}

	for(n=0;n<NR_TTY_PORTS;n++) {
		priv->csmi_ttys[n].fifo_base = priv->fifo_base + (n * 2 * FIFO_SIZE);
		priv->csmi_ttys[n].port = n+FIRST_TTY_PORT;
		mutex_init(&priv->csmi_ttys[n].io_mutex_read);
	        mutex_init(&priv->csmi_ttys[n].io_mutex_write);
	        mutex_init(&priv->csmi_ttys[n].io_mutex_open_close);
	        init_completion(&priv->csmi_ttys[n].io_comp_read);
	        init_completion(&priv->csmi_ttys[n].io_comp_write);
	        spin_lock_init(&priv->csmi_ttys[n].fifo_lock);
		priv->csmi_ttys[n].data = NULL;
	}

	csmi_tty_driver = alloc_tty_driver(1);
	if (!csmi_tty_driver) {
		error = -ENOMEM;
		goto err_tty;
	}

	/* Initialize the tty_driver structure */

	csmi_tty_driver->owner = THIS_MODULE;
	csmi_tty_driver->driver_name = "TI CSMI GTI";
	csmi_tty_driver->name = "ttyCSMI";
	csmi_tty_driver->major = 204;
	csmi_tty_driver->minor_start = 0;
	csmi_tty_driver->num=NR_TTY_PORTS;
	csmi_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	csmi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	csmi_tty_driver->init_termios = tty_std_termios;
	csmi_tty_driver->init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	csmi_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	tty_set_operations(csmi_tty_driver, &csmi_tty_ops);

	/* register tty driver */
	if ((error = tty_register_driver(csmi_tty_driver))) {
		printk("CSMI: Couldn't register CSMI driver (%d.)\n", error);
		goto err_tty_reg;
	}

	/* register ICR interrupt */
        if ((error = request_irq(INT_850_ICR, icr_interrupt,
	                       0, "GSM ICR", dev)) < 0) {
	        printk("Failed to register ICR interrupt (%d)\n", error);
                goto err_irq_icr;
        }

	/* register RF interrupt */
        if ((error = request_irq(INT_850_DBB_RF_EN, rf_interrupt,
                               0, "GSM RF", dev)) < 0) {
                printk("Failed to register RF interrupt (%d)\n", error);
                error = -ENODEV;
                goto err_irq_rf;
        }

        /* start the GSM program */
        if (!noreset) {
	        if((error = gsm_reset()) || (error = gsm_loadfw(dev)))
			goto err_fw;
	        gsm_boot();
	}
	/* enable sound */
	//omap_set_gpio_direction(WIZARD_SNDDEV_GPIO, 0);
	//omap_set_gpio_dataout(WIZARD_SNDDEV_GPIO, 0);

        return 0;

// Error unwinding
err_fw:
        free_irq(INT_850_DBB_RF_EN, dev);
err_irq_rf:
        free_irq(INT_850_ICR, dev);
err_irq_icr:
	if ((error = tty_unregister_driver(csmi_tty_driver)))
		printk("CSMI: Failed to unregister CSMI driver (%d.)\n", error);
err_tty_reg:
	put_tty_driver(csmi_tty_driver);
err_tty:
	fifo_cleanup(priv);
err_fifo:
	return error;
}

static int wizard_gsm_remove(struct platform_device *pdev)
{
	struct csmi_priv *priv = platform_get_drvdata(pdev);
        struct device *dev;
	int error;

	dev = &pdev->dev;

        free_irq(INT_850_ICR, dev);
        free_irq(INT_850_DBB_RF_EN, dev);

	if ((error = tty_unregister_driver(csmi_tty_driver)))
		printk("CSMI: Failed to unregister CSMI driver (%d.)\n", error);

	put_tty_driver(csmi_tty_driver);

        fifo_cleanup(priv);

	/* disable sound */
	//omap_set_gpio_direction(WIZARD_SNDDEV_GPIO, 0);
	//omap_set_gpio_dataout(WIZARD_SNDDEV_GPIO, 0);

        return 0;
}
							
static struct platform_driver wizard_gsm_driver = {
        .probe          = wizard_gsm_probe,
        .remove         = wizard_gsm_remove,
        .driver         = {
                .name   = "wizard-gsm",
        },
};

static int __init wizard_gsm_init(void)
{
        return platform_driver_register(&wizard_gsm_driver);
}

static void __exit wizard_gsm_exit(void)
{
        platform_driver_unregister(&wizard_gsm_driver);
}

module_init(wizard_gsm_init);
module_exit(wizard_gsm_exit);

MODULE_AUTHOR("Justin Verel <justin@marverinc.nl>");
MODULE_DESCRIPTION("HTC Wizard GSM module");
MODULE_LICENSE("GPL");
