#ifndef __ARCH_ARM_MACH_OMAP2_PRM_H
#define __ARCH_ARM_MACH_OMAP2_PRM_H

/*
 * OMAP2/3 Power/Reset Management (PRM) register definitions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2008 Nokia Corporation
 *
 * Written by Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/io.h>
#include <asm/bitops.h>

#include "prcm-common.h"

#ifndef __ASSEMBLER__
#define OMAP_PRM_REGADDR(module, reg)					\
	(void __iomem *)IO_ADDRESS(OMAP2_PRM_BASE + (module) + (reg))
#else
#define OMAP2420_PRM_REGADDR(module, reg)				\
			IO_ADDRESS(OMAP2420_PRM_BASE + (module) + (reg))
#define OMAP2430_PRM_REGADDR(module, reg)				\
			IO_ADDRESS(OMAP2430_PRM_BASE + (module) + (reg))
#define OMAP34XX_PRM_REGADDR(module, reg)				\
			IO_ADDRESS(OMAP3430_PRM_BASE + (module) + (reg))
#endif

/*
 * Architecture-specific global PRM registers
 * Use prm_{read,write}_reg() with these registers.
 *
 * With a few exceptions, these are the register names beginning with
 * PRCM_* on 24xx, and PRM_* on 34xx.  (The exceptions are the
 * IRQSTATUS and IRQENABLE bits.)
 *
 */

#define OMAP24XX_PRCM_REVISION		OMAP_PRM_REGADDR(OCP_MOD, 0x0000)
#define OMAP24XX_PRCM_SYSCONFIG		OMAP_PRM_REGADDR(OCP_MOD, 0x0010)

#define OMAP24XX_PRCM_IRQSTATUS_MPU	OMAP_PRM_REGADDR(OCP_MOD, 0x0018)
#define OMAP24XX_PRCM_IRQENABLE_MPU	OMAP_PRM_REGADDR(OCP_MOD, 0x001c)

#define OMAP24XX_PRCM_VOLTCTRL		OMAP_PRM_REGADDR(OCP_MOD, 0x0050)
#define OMAP24XX_PRCM_VOLTST		OMAP_PRM_REGADDR(OCP_MOD, 0x0054)
#define OMAP24XX_PRCM_CLKSRC_CTRL	OMAP_PRM_REGADDR(OCP_MOD, 0x0060)
#define OMAP24XX_PRCM_CLKOUT_CTRL	OMAP_PRM_REGADDR(OCP_MOD, 0x0070)
#define OMAP24XX_PRCM_CLKEMUL_CTRL	OMAP_PRM_REGADDR(OCP_MOD, 0x0078)
#define OMAP24XX_PRCM_CLKCFG_CTRL	OMAP_PRM_REGADDR(OCP_MOD, 0x0080)
#define OMAP24XX_PRCM_CLKCFG_STATUS	OMAP_PRM_REGADDR(OCP_MOD, 0x0084)
#define OMAP24XX_PRCM_VOLTSETUP		OMAP_PRM_REGADDR(OCP_MOD, 0x0090)
#define OMAP24XX_PRCM_CLKSSETUP		OMAP_PRM_REGADDR(OCP_MOD, 0x0094)
#define OMAP24XX_PRCM_POLCTRL		OMAP_PRM_REGADDR(OCP_MOD, 0x0098)

#define OMAP3430_PRM_REVISION		OMAP_PRM_REGADDR(OCP_MOD, 0x0004)
#define OMAP3430_PRM_SYSCONFIG		OMAP_PRM_REGADDR(OCP_MOD, 0x0014)

#define OMAP3430_PRM_IRQSTATUS_MPU	OMAP_PRM_REGADDR(OCP_MOD, 0x0018)
#define OMAP3430_PRM_IRQENABLE_MPU	OMAP_PRM_REGADDR(OCP_MOD, 0x001c)

#define OMAP3430_PRM_VC_SMPS_SA		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0020)
#define OMAP3430_PRM_VC_SMPS_VOL_RA	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0024)
#define OMAP3430_PRM_VC_SMPS_CMD_RA	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0028)
#define OMAP3430_PRM_VC_CMD_VAL_0	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x002c)
#define OMAP3430_PRM_VC_CMD_VAL_1	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0030)
#define OMAP3430_PRM_VC_CH_CONF		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0034)
#define OMAP3430_PRM_VC_I2C_CFG		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0038)
#define OMAP3430_PRM_VC_BYPASS_VAL	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x003c)
#define OMAP3430_PRM_RSTCTRL		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0050)
#define OMAP3430_PRM_RSTTIME		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0054)
#define OMAP3430_PRM_RSTST		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0058)
#define OMAP3430_PRM_VOLTCTRL		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0060)
#define OMAP3430_PRM_SRAM_PCHARGE	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0064)
#define OMAP3430_PRM_CLKSRC_CTRL	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0070)
#define OMAP3430_PRM_VOLTSETUP1		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0090)
#define OMAP3430_PRM_VOLTOFFSET		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0094)
#define OMAP3430_PRM_CLKSETUP		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x0098)
#define OMAP3430_PRM_POLCTRL		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x009c)
#define OMAP3430_PRM_VOLTSETUP2		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00a0)
#define OMAP3430_PRM_VP1_CONFIG		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00b0)
#define OMAP3430_PRM_VP1_VSTEPMIN	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00b4)
#define OMAP3430_PRM_VP1_VSTEPMAX	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00b8)
#define OMAP3430_PRM_VP1_VLIMITTO	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00bc)
#define OMAP3430_PRM_VP1_VOLTAGE	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00c0)
#define OMAP3430_PRM_VP1_STATUS		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00c4)
#define OMAP3430_PRM_VP2_CONFIG		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00d0)
#define OMAP3430_PRM_VP2_VSTEPMIN	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00d4)
#define OMAP3430_PRM_VP2_VSTEPMAX	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00d8)
#define OMAP3430_PRM_VP2_VLIMITTO	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00dc)
#define OMAP3430_PRM_VP2_VOLTAGE	OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00e0)
#define OMAP3430_PRM_VP2_STATUS		OMAP_PRM_REGADDR(OMAP3430_GR_MOD, 0x00e4)

#define OMAP3430_PRM_CLKSEL		OMAP_PRM_REGADDR(OMAP3430_CCR_MOD, 0x0040)
#define OMAP3430_PRM_CLKOUT_CTRL	OMAP_PRM_REGADDR(OMAP3430_CCR_MOD, 0x0070)

#ifndef __ASSEMBLER__

/* Read-modify-write bits in a PRM register */
static __inline__ u32 __attribute__((unused)) prm_rmw_reg_bits(u32 mask,
						u32 bits, void __iomem *va)
{
	u32 v;

	v = __raw_readl(va);
	v &= ~mask;
	v |= bits;
	__raw_writel(v, va);

	return v;
}

#endif

/*
 * Module specific PRM registers from PRM_BASE + domain offset
 *
 * Use prm_{read,write}_mod_reg() with these registers.
 *
 * With a few exceptions, these are the register names beginning with
 * {PM,RM}_* on both architectures.  (The exceptions are the IRQSTATUS
 * and IRQENABLE bits.)
 *
 */

/* Registers appearing on both 24xx and 34xx */

#define RM_RSTCTRL					0x0050
#define RM_RSTTIME					0x0054
#define RM_RSTST					0x0058

#define PM_WKEN						0x00a0
#define PM_WKEN1					PM_WKEN
#define PM_WKST						0x00b0
#define PM_WKST1					PM_WKST
#define PM_WKDEP					0x00c8
#define PM_EVGENCTRL					0x00d4
#define PM_EVGENONTIM					0x00d8
#define PM_EVGENOFFTIM					0x00dc
#define PM_PWSTCTRL					0x00e0
#define PM_PWSTST					0x00e4

#define OMAP3430_PM_MPUGRPSEL				0x00a4
#define OMAP3430_PM_MPUGRPSEL1				OMAP3430_PM_MPUGRPSEL

#define OMAP3430_PM_IVAGRPSEL				0x00a8
#define OMAP3430_PM_IVAGRPSEL1				OMAP3430_PM_IVAGRPSEL

#define OMAP3430_PM_PREPWSTST				0x00e8

#define OMAP3430_PRM_IRQSTATUS_IVA2			0x00f8
#define OMAP3430_PRM_IRQENABLE_IVA2			0x00fc

/* Read-modify-write bits in a PRM register (by domain) */
static u32 __attribute__((unused)) prm_rmw_mod_reg_bits(u32 mask, u32 bits,
							s16 module, s16 idx)
{
	return prm_rmw_reg_bits(mask, bits, OMAP_PRM_REGADDR(module, idx));
}

static u32 __attribute__((unused)) prm_set_mod_reg_bits(u32 bits, s16 module, s16 idx)
{
	return prm_rmw_mod_reg_bits(bits, bits, module, idx);
}

static u32 __attribute__((unused)) prm_clear_mod_reg_bits(u32 bits, s16 module, s16 idx)
{
	return prm_rmw_mod_reg_bits(bits, 0x0, module, idx);
}

/* Architecture-specific registers */

#define OMAP24XX_PM_WKEN2				0x00a4
#define OMAP24XX_PM_WKST2				0x00b4

#define OMAP24XX_PRCM_IRQSTATUS_DSP			0x00f0	/* IVA mod */
#define OMAP24XX_PRCM_IRQENABLE_DSP			0x00f4	/* IVA mod */
#define OMAP24XX_PRCM_IRQSTATUS_IVA			0x00f8
#define OMAP24XX_PRCM_IRQENABLE_IVA			0x00fc

#ifndef __ASSEMBLER__

/* Power/reset management domain register get/set */

static __inline__ void __attribute__((unused)) prm_write_mod_reg(u32 val,
							s16 module, s16 idx)
{
	__raw_writel(val, OMAP_PRM_REGADDR(module, idx));
}

static __inline__ u32 __attribute__((unused)) prm_read_mod_reg(s16 module,
							s16 idx)
{
	return __raw_readl(OMAP_PRM_REGADDR(module, idx));
}

#endif

/*
 * Bits common to specific registers
 *
 * The 3430 register and bit names are generally used,
 * since they tend to make more sense
 */

/* PM_EVGENONTIM_MPU */
/* Named PM_EVEGENONTIM_MPU on the 24XX */
#define OMAP_ONTIMEVAL_SHIFT				0
#define OMAP_ONTIMEVAL_MASK				(0xffffffff << 0)

/* PM_EVGENOFFTIM_MPU */
/* Named PM_EVEGENOFFTIM_MPU on the 24XX */
#define OMAP_OFFTIMEVAL_SHIFT				0
#define OMAP_OFFTIMEVAL_MASK				(0xffffffff << 0)

/* PRM_CLKSETUP and PRCM_VOLTSETUP */
/* Named PRCM_CLKSSETUP on the 24XX */
#define OMAP_SETUP_TIME_SHIFT				0
#define OMAP_SETUP_TIME_MASK				(0xffff << 0)

/* PRM_CLKSRC_CTRL */
/* Named PRCM_CLKSRC_CTRL on the 24XX */
#define OMAP_SYSCLKDIV_SHIFT				6
#define OMAP_SYSCLKDIV_MASK				(0x3 << 6)
#define OMAP_AUTOEXTCLKMODE_SHIFT			3
#define OMAP_AUTOEXTCLKMODE_MASK			(0x3 << 3)
#define OMAP_SYSCLKSEL_SHIFT				0
#define OMAP_SYSCLKSEL_MASK				(0x3 << 0)

/* PM_EVGENCTRL_MPU */
#define OMAP_OFFLOADMODE_SHIFT				3
#define OMAP_OFFLOADMODE_MASK				(0x3 << 3)
#define OMAP_ONLOADMODE_SHIFT				1
#define OMAP_ONLOADMODE_MASK				(0x3 << 1)
#define OMAP_ENABLE					(1 << 0)

/* PRM_RSTTIME */
/* Named RM_RSTTIME_WKUP on the 24xx */
#define OMAP_RSTTIME2_SHIFT				8
#define OMAP_RSTTIME2_MASK				(0x1f << 8)
#define OMAP_RSTTIME1_SHIFT				0
#define OMAP_RSTTIME1_MASK				(0xff << 0)


/* PRM_RSTCTRL */
/* Named RM_RSTCTRL_WKUP on the 24xx */
/* 2420 calls RST_DPLL3 'RST_DPLL' */
#define OMAP_RST_DPLL3					(1 << 2)
#define OMAP_RST_GS					(1 << 1)


/*
 * Bits common to module-shared registers
 *
 * Not all registers of a particular type support all of these bits -
 * check TRM if you are unsure
 */

/*
 * 24XX: PM_PWSTST_CORE, PM_PWSTST_GFX, PM_PWSTST_MPU, PM_PWSTST_DSP
 *
 * 2430: PM_PWSTST_MDM
 *
 * 3430: PM_PWSTST_IVA2, PM_PWSTST_MPU, PM_PWSTST_CORE, PM_PWSTST_GFX,
 *	 PM_PWSTST_DSS, PM_PWSTST_CAM, PM_PWSTST_PER, PM_PWSTST_EMU,
 *	 PM_PWSTST_NEON
 */
#define OMAP_INTRANSITION				(1 << 20)


/*
 * 24XX: PM_PWSTST_GFX, PM_PWSTST_DSP
 *
 * 2430: PM_PWSTST_MDM
 *
 * 3430: PM_PWSTST_IVA2, PM_PWSTST_MPU, PM_PWSTST_CORE, PM_PWSTST_GFX,
 *	 PM_PWSTST_DSS, PM_PWSTST_CAM, PM_PWSTST_PER, PM_PWSTST_EMU,
 *	 PM_PWSTST_NEON
 */
#define OMAP_POWERSTATEST_SHIFT				0
#define OMAP_POWERSTATEST_MASK				(0x3 << 0)

/*
 * 24XX: RM_RSTST_MPU and RM_RSTST_DSP - on 24XX, 'COREDOMAINWKUP_RST' is
 *	 called 'COREWKUP_RST'
 *
 * 3430: RM_RSTST_IVA2, RM_RSTST_MPU, RM_RSTST_GFX, RM_RSTST_DSS,
 *	 RM_RSTST_CAM, RM_RSTST_PER, RM_RSTST_NEON
 */
#define OMAP_COREDOMAINWKUP_RST				(1 << 3)

/*
 * 24XX: RM_RSTST_MPU, RM_RSTST_GFX, RM_RSTST_DSP
 *
 * 2430: RM_RSTST_MDM
 *
 * 3430: RM_RSTST_CORE, RM_RSTST_EMU
 */
#define OMAP_DOMAINWKUP_RST				(1 << 2)

/*
 * 24XX: RM_RSTST_MPU, RM_RSTST_WKUP, RM_RSTST_DSP
 *	 On 24XX, 'GLOBALWARM_RST' is called 'GLOBALWMPU_RST'.
 *
 * 2430: RM_RSTST_MDM
 *
 * 3430: RM_RSTST_CORE, RM_RSTST_EMU
 */
#define OMAP_GLOBALWARM_RST				(1 << 1)
#define OMAP_GLOBALCOLD_RST				(1 << 0)

/*
 * 24XX: PM_WKDEP_GFX, PM_WKDEP_MPU, PM_WKDEP_CORE, PM_WKDEP_DSP
 *	 2420 TRM sometimes uses "EN_WAKEUP" instead of "EN_WKUP"
 *
 * 2430: PM_WKDEP_MDM
 *
 * 3430: PM_WKDEP_IVA2, PM_WKDEP_GFX, PM_WKDEP_DSS, PM_WKDEP_CAM,
 *	 PM_WKDEP_PER
 */
#define OMAP_EN_WKUP					(1 << 4)

/*
 * 24XX: PM_PWSTCTRL_MPU, PM_PWSTCTRL_CORE, PM_PWSTCTRL_GFX,
 *	 PM_PWSTCTRL_DSP
 *
 * 2430: PM_PWSTCTRL_MDM
 *
 * 3430: PM_PWSTCTRL_IVA2, PM_PWSTCTRL_CORE, PM_PWSTCTRL_GFX,
 *	 PM_PWSTCTRL_DSS, PM_PWSTCTRL_CAM, PM_PWSTCTRL_PER,
 *	 PM_PWSTCTRL_NEON
 */
#define OMAP_LOGICRETSTATE				(1 << 2)

/*
 * 24XX: PM_PWSTCTRL_MPU, PM_PWSTCTRL_CORE, PM_PWSTCTRL_GFX,
 *       PM_PWSTCTRL_DSP, PM_PWSTST_MPU
 *
 * 2430: PM_PWSTCTRL_MDM shared bits
 *
 * 3430: PM_PWSTCTRL_IVA2, PM_PWSTCTRL_MPU, PM_PWSTCTRL_CORE,
 *	 PM_PWSTCTRL_GFX, PM_PWSTCTRL_DSS, PM_PWSTCTRL_CAM, PM_PWSTCTRL_PER,
 *	 PM_PWSTCTRL_NEON shared bits
 */
#define OMAP_POWERSTATE_SHIFT				0
#define OMAP_POWERSTATE_MASK				(0x3 << 0)


#endif
