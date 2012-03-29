/*
 * (C) Copyright 2011
 * Logic Product Development, <www.logicpd.com>
 *
 * Author :
 *	Peter Barada <peterb@logicpd.com>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <netdev.h>
#include <nand.h>
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/omap_gpmc.h>
#include <asm/arch/gpio.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include <twl4030.h>
#include "logic.h"
#include "product_id.h"

DECLARE_GLOBAL_DATA_PTR;

#define MUX_LOGIC_HSUSB0_D5_GPIO_MUX()					\
 MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M4)) /*GPIO_189*/

/* Safe mode HSUSB0_DATA5 */
#define MUX_LOGIC_HSUSB0_D5_SAFE_MODE()					\
 MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | EN  | M7)) /*HSUSB0.SAFE_MODE*/
/*
 * Routine: logic_identify
 * Description: Detect if we are running on a Logic or Torpedo.
 *              This can be done by GPIO_189. If its low after driving it high,
 *              then its a SOM LV, else Torpedo.
 */
unsigned int logic_identify(void)
{
	unsigned int val = 0;
	int i;

	MUX_LOGIC_HSUSB0_D5_GPIO_MUX();

	if (!omap_request_gpio(189)) {

		omap_set_gpio_direction(189, 0);
		omap_set_gpio_dataout(189, 1);

		/* Let it soak for a bit */
		for (i=0; i<0x100; ++i)
			asm("nop");

		omap_set_gpio_direction(189, 1);
		val = omap_get_gpio_datain(189);
		omap_free_gpio(189);

		printf("Board: ");
		if (val) {
			printf("Torpedo\n");
			val = MACH_TYPE_DM3730_TORPEDO;
		} else {
			printf("LV SOM\n");
			val = MACH_TYPE_DM3730_SOM_LV;
		}
	}

	MUX_LOGIC_HSUSB0_D5_SAFE_MODE();

	return val;
}

static void setup_nand_settings(void);
static void fix_flash_sync(void);
static void setup_isp1760_chip(void);
static void setup_cf_gpmc(void);

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */

	/* Update NAND settings */
	setup_nand_settings();

	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_DM3730_TORPEDO;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

/* Turn on VAUX1 voltage to 3.0 volts to drive level shifters and
 * power 3.0v parts (tsc2004 and Product ID chip) */
#define I2C_TRITON2 0x4b /* Address of Triton power group */

void init_vaux1_voltage(void)
{
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	unsigned char data;
	unsigned short msg;

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	/* Select the output voltage */
	data = 0x04;
	i2c_write(I2C_TRITON2, 0x72, 1, &data, 1);
	/* Select the Processor resource group */
	data = 0x20;
	i2c_write(I2C_TRITON2, 0x72, 1, &data, 1);
	/* Enable I2C access to the Power bus */
	data = 0x02;
	i2c_write(I2C_TRITON2, 0x4a, 1, &data, 1);
	/* Send message MSB */
	msg = (1<<13) | (1<<4) | (0xd<<0); /* group(process_grp1):resource(vaux1):res_active; */
	data = msg >> 8;
	i2c_write(I2C_TRITON2, 0x4b, 1, &data, 1);
	/* Send message LSB */
	data = msg & 0xff;
	i2c_write(I2C_TRITON2, 0x4c, 1, &data, 1);
#endif
}

/*
 * Check _SYSCONFIG registers and fisup bootrom leaving them in
 * non forced-idle/smart-stdby mode */

static void check_sysconfig_regs(void)
{
	unsigned int temp, temp2;

	temp = *(unsigned int *)OTG_SYSCONFIG;
	temp2 = OTG_SYSCONFIG_MIDLEMODE_SMART_STDBY
		| OTG_SYSCONFIG_SIDLEMODE_FORCE_IDLE
		| OTG_SYSCONFIG_AUTOIDLE;
	if (temp != temp2) {
		printf("OTG_SYSCONFIG: %08x - needs to be %08x\n", temp, temp2);
		*(unsigned int *)OTG_SYSCONFIG = temp2;
	}
}

/*
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 */
int misc_init_r(void)
{

#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

	/* Turn on vaux1 to make sure voltage is to the product ID chip.
	 * Extract production data from ID chip, used to selectively
	 * initialize portions of the system */
	init_vaux1_voltage();
	fetch_production_data();

#if defined(CONFIG_CMD_NET)
	setup_net_chip();
#endif


	gd->bd->bi_arch_number = logic_identify();

	/* Setup the ISP1763 on CS6 if we're a Torpedo */
	if (gd->bd->bi_arch_number == MACH_TYPE_DM3730_TORPEDO)
		setup_isp1760_chip();

	/* Setup CompactFlash on CS3 if we're a SOM LV */
	if (gd->bd->bi_arch_number == MACH_TYPE_DM3730_SOM_LV)
		setup_cf_gpmc();

	/* Fix the flash sync */
	fix_flash_sync();

	dieid_num_r();

	check_sysconfig_regs();
	return 0;
}

/* Called after the default environment has been copied to memory */
int board_default_env_init(void)
{
	struct mtd_info *mtd;
	struct nand_chip *nand;
	char *p;

	mtd = &nand_info[nand_curr_device];
	nand = mtd->priv;
	switch(nand->ecc.mode) {
	case NAND_ECC_SOFT:
		p = "sw";
		break;
	case NAND_ECC_HW:
		p = "hw";
		break;
	case NAND_ECC_CHIP:
		p = "chip";
		break;
	default:
		p = NULL;
		break;
	}
	if (p)
		setenv("defaultecc", p);
	return 0;
}

int board_late_init(void)
{
	unsigned char enetaddr[6];

	dump_production_data();

	/* Fetch the ethaddr of the LAN */
	board_get_nth_enetaddr(enetaddr, 0, 0);
#ifdef CONFIG_HAS_ETH1
	/* Fetch the ethaddr of the WiFi */
	board_get_nth_enetaddr(enetaddr, 1, 1);
#endif

#ifdef CONFIG_CMD_NAND_LOCK_UNLOCK
	nand_unlock(&nand_info[0], 0x0, nand_info[0].size);
#endif

	/* Setup environment based on hardware found */
	board_default_env_init();

#ifdef CONFIG_ENABLE_TWL4030_CHARGING
	/* Enable charging on Torpedo unless $disablecharging == yes */
	if (gd->bd->bi_arch_number == MACH_TYPE_DM3730_TORPEDO) {
		char *str;
		str = getenv("disablecharging");
		if (!str || strcmp(str, "yes") != 0) {
			printf("Torpedo: Enabling battery charging\n");
			twl4030_enable_charging();
		}
	}
#endif

	return 0;
}

/*
 * Routine: setup_net_chip
 * Description: Setting up the configuration GPMC registers specific to the
 *		Ethernet hardware.
 */
static void setup_net_chip(void)
{
	struct ctrl *ctrl_base = (struct ctrl *)OMAP34XX_CTRL_BASE;

	/* Configure GPMC registers */
	writel(LOGIC_NET_GPMC_CONFIG1, &gpmc_cfg->cs[1].config1);
	writel(LOGIC_NET_GPMC_CONFIG2, &gpmc_cfg->cs[1].config2);
	writel(LOGIC_NET_GPMC_CONFIG3, &gpmc_cfg->cs[1].config3);
	writel(LOGIC_NET_GPMC_CONFIG4, &gpmc_cfg->cs[1].config4);
	writel(LOGIC_NET_GPMC_CONFIG5, &gpmc_cfg->cs[1].config5);
	writel(LOGIC_NET_GPMC_CONFIG6, &gpmc_cfg->cs[1].config6);
	writel(LOGIC_NET_GPMC_CONFIG7, &gpmc_cfg->cs[1].config7);

	/* Enable off mode for NWE in PADCONF_GPMC_NWE register */
	writew(readw(&ctrl_base ->gpmc_nwe) | 0x0E00, &ctrl_base->gpmc_nwe);
	/* Enable off mode for NOE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_noe) | 0x0E00, &ctrl_base->gpmc_noe);
	/* Enable off mode for ALE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_nadv_ale) | 0x0E00,
		&ctrl_base->gpmc_nadv_ale);
}

#define LOGIC_NAND_GPMC_CONFIG1	0x00001800
#define LOGIC_NAND_GPMC_CONFIG2	0x00090900
#define LOGIC_NAND_GPMC_CONFIG3	0x00090902
#define LOGIC_NAND_GPMC_CONFIG4	0x07020702
#define LOGIC_NAND_GPMC_CONFIG5	0x00080909
#define LOGIC_NAND_GPMC_CONFIG6	0x000002CF
#define LOGIC_NAND_GPMC_CONFIG7	0x00000C70

static void setup_nand_settings(void)
{
	/* struct ctrl *ctrl_base = (struct ctrl *)OMAP34XX_CTRL_BASE; */

	/* Configure GPMC registers */
	writel(0x00000000, &gpmc_cfg->cs[0].config7);
	sdelay(1000);
	writel(LOGIC_NAND_GPMC_CONFIG1, &gpmc_cfg->cs[0].config1);
	writel(LOGIC_NAND_GPMC_CONFIG2, &gpmc_cfg->cs[0].config2);
	writel(LOGIC_NAND_GPMC_CONFIG3, &gpmc_cfg->cs[0].config3);
	writel(LOGIC_NAND_GPMC_CONFIG4, &gpmc_cfg->cs[0].config4);
	writel(LOGIC_NAND_GPMC_CONFIG5, &gpmc_cfg->cs[0].config5);
	writel(LOGIC_NAND_GPMC_CONFIG6, &gpmc_cfg->cs[0].config6);
	writel(LOGIC_NAND_GPMC_CONFIG7, &gpmc_cfg->cs[0].config7);
	sdelay(2000);
}

#define LOGIC_CF_GPMC_CONFIG1	0x00001210
#define LOGIC_CF_GPMC_CONFIG2	0x00131000
#define LOGIC_CF_GPMC_CONFIG3	0x001f1f01
#define LOGIC_CF_GPMC_CONFIG4	0x10030e03
#define LOGIC_CF_GPMC_CONFIG5	0x010f1411
#define LOGIC_CF_GPMC_CONFIG6	0x80030600
#define LOGIC_CF_GPMC_CONFIG7	0x00000f58

static void setup_cf_gpmc(void)
{
	/* Configure GPMC registers */
	writel(0x00000000, &gpmc_cfg->cs[3].config7);
	sdelay(1000);
	writel(LOGIC_CF_GPMC_CONFIG1, &gpmc_cfg->cs[3].config1);
	writel(LOGIC_CF_GPMC_CONFIG2, &gpmc_cfg->cs[3].config2);
	writel(LOGIC_CF_GPMC_CONFIG3, &gpmc_cfg->cs[3].config3);
	writel(LOGIC_CF_GPMC_CONFIG4, &gpmc_cfg->cs[3].config4);
	writel(LOGIC_CF_GPMC_CONFIG5, &gpmc_cfg->cs[3].config5);
	writel(LOGIC_CF_GPMC_CONFIG6, &gpmc_cfg->cs[3].config6);
	writel(LOGIC_CF_GPMC_CONFIG7, &gpmc_cfg->cs[3].config7);
	sdelay(2000);
}

// GPMC settings for LOGIC 1760 chip
#define LOGIC_ISP1760_GPMC_CONFIG1  0x00001000
#define LOGIC_ISP1760_GPMC_CONFIG2  0x00090900
#define LOGIC_ISP1760_GPMC_CONFIG3  0x00000000
#define LOGIC_ISP1760_GPMC_CONFIG4  0x05000900
#define LOGIC_ISP1760_GPMC_CONFIG5  0x0007090c
#define LOGIC_ISP1760_GPMC_CONFIG6  0x04010200
#define LOGIC_ISP1760_GPMC_CONFIG7  0x00000f5c

/*
 * Routine: setup_isp1760_chip
 * Description: Setting up the configuration GPMC registers specific to the
 *		ISP1760 USB hardware.
 */
static void setup_isp1760_chip(void)
{
	/* Configure GPMC registers */
	writel(LOGIC_ISP1760_GPMC_CONFIG1, &gpmc_cfg->cs[6].config1);
	writel(LOGIC_ISP1760_GPMC_CONFIG2, &gpmc_cfg->cs[6].config2);
	writel(LOGIC_ISP1760_GPMC_CONFIG3, &gpmc_cfg->cs[6].config3);
	writel(LOGIC_ISP1760_GPMC_CONFIG4, &gpmc_cfg->cs[6].config4);
	writel(LOGIC_ISP1760_GPMC_CONFIG5, &gpmc_cfg->cs[6].config5);
	writel(LOGIC_ISP1760_GPMC_CONFIG6, &gpmc_cfg->cs[6].config6);
	writel(LOGIC_ISP1760_GPMC_CONFIG7, &gpmc_cfg->cs[6].config7);
}

#if 1
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG1	0x00001210
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG2	0x00101001
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG3	0x00020201
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG4	0x0f031003
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG5	0x000f1111
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG6	0x0f030080
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG7	0x00000c50

#define LOGIC_STNOR_SYNC_GPMC_CONFIG1	0x68411213
#define LOGIC_STNOR_SYNC_GPMC_CONFIG2	0x000C1502
#define LOGIC_STNOR_SYNC_GPMC_CONFIG3	0x00040402
#define LOGIC_STNOR_SYNC_GPMC_CONFIG4	0x0B051505
#define LOGIC_STNOR_SYNC_GPMC_CONFIG5	0x020E0C15
#define LOGIC_STNOR_SYNC_GPMC_CONFIG6	0x0B0603C3
#define LOGIC_STNOR_SYNC_GPMC_CONFIG7	0x00000c50
#else
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG1	0x00001211
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG2	0x00080901
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG3	0x00020201
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG4	0x08010901
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG5	0x0008090a
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG6	0x08030200
#define LOGIC_STNOR_ASYNC_GPMC_CONFIG7	0x00000c50

#define LOGIC_STNOR_SYNC_GPMC_CONFIG1	0x68411213
#define LOGIC_STNOR_SYNC_GPMC_CONFIG2	0x000C1502
#define LOGIC_STNOR_SYNC_GPMC_CONFIG3	0x00040402
#define LOGIC_STNOR_SYNC_GPMC_CONFIG4	0x0B051505
#define LOGIC_STNOR_SYNC_GPMC_CONFIG5	0x020E0C15
#define LOGIC_STNOR_SYNC_GPMC_CONFIG6	0x0B0603C3
#define LOGIC_STNOR_SYNC_GPMC_CONFIG7	0x00000c50
#endif

#define LOGIC_FLASH_BASE 0x10000000

/* These are bit definitions for the RCR register of the NOR flash       */
/* 28FxxxP30 device.  This register sets the bus configration for reads. */
/* settings, located on address pins A[15:0].                            */
#define FLASH_28FxxxP30_RCR_RM    0x8000
#define FLASH_28FxxxP30_RCR_R     0x4000
#define FLASH_28FxxxP30_RCR_LC(x) ((x & 0x7) << 11)
#define FLASH_28FxxxP30_RCR_WP    0x0400
#define FLASH_28FxxxP30_RCR_DH    0x0200
#define FLASH_28FxxxP30_RCR_WD    0x0100
#define FLASH_28FxxxP30_RCR_BS    0x0080
#define FLASH_28FxxxP30_RCR_CE    0x0040
#define FLASH_28FxxxP30_RCR_BW    0x0008
#define FLASH_28FxxxP30_RCR_BL(x) ((x & 0x7) << 0)
#define FLASH_28FxxxP30_BL_4      0x1
#define FLASH_28FxxxP30_BL_8      0x2
#define FLASH_28FxxxP30_BL_16     0x3
#define FLASH_28FxxxP30_BL_CONT   0x7

/*
 * Routine: fix_flash_sync
 * Description: Setting up the configuration GPMC registers specific to the
 *		NOR flash (and place in sync mode if not done).
 */
static void fix_flash_sync(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	int arch_number;
	u16 rcrval;

	/* Check the arch_number - Torpedo doesn't have NOR flash */
	arch_number = gd->bd->bi_arch_number;
	if (arch_number == MACH_TYPE_DM3730_TORPEDO)
		return;

	/* Check CS2 config, if its not in sync, or not valid then configure it */
	if (!(readl(&gpmc_cfg->cs[2].config1) & TYPE_READTYPE) ||
		!(readl(&gpmc_cfg->cs[2].config7) & 0x40)) {

		/* Invalidate in case its set valud */
		writel(0x00000000, &gpmc_cfg->cs[2].config7);

		/* clear WAIT1 polarity */
		writel(readl(&gpmc_cfg->config) & ~0x200, &gpmc_cfg->config);

		/* clear GPMC_TIMEOUT */
		writel(0x0, &gpmc_cfg->timeout_control);

		/* Configure GPMC registers for async */
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG1, &gpmc_cfg->cs[2].config1);
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG2, &gpmc_cfg->cs[2].config2);
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG3, &gpmc_cfg->cs[2].config3);
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG4, &gpmc_cfg->cs[2].config4);
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG5, &gpmc_cfg->cs[2].config5);
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG6, &gpmc_cfg->cs[2].config6);
		writel(LOGIC_STNOR_ASYNC_GPMC_CONFIG7, &gpmc_cfg->cs[2].config7);

		/* Test if this NOR flash is connected */
		*(volatile u16 *)LOGIC_FLASH_BASE = 0x0070; /* Read status reg */
		if (*(volatile u16 *)LOGIC_FLASH_BASE != 0x0080) {
			/* Its invalid, disable CS */
			writel(0x00000000, &gpmc_cfg->cs[2].config7);
			puts("NOR: no flash device detected\n");
			return;
		}

		puts("NOR: initialize in sync mode\n");

		/* 1st NOR cycle, send read config register setup 0x60 */
		*(volatile u16 *)LOGIC_FLASH_BASE = 0x0060;

		/* 2nd NOR cycle, send 0x03 to latch in read
		 * configuration register setttings, located on A[15:0] */
		rcrval = FLASH_28FxxxP30_RCR_LC(4) | FLASH_28FxxxP30_RCR_WP |
		  FLASH_28FxxxP30_RCR_BS | FLASH_28FxxxP30_RCR_CE |
		  FLASH_28FxxxP30_RCR_BW | FLASH_28FxxxP30_RCR_BL(FLASH_28FxxxP30_BL_4);
		*(volatile u16 *)(LOGIC_FLASH_BASE | (rcrval << 1)) = 0x0003;

		/* Give a chance for accesses to finish... */
		sdelay(2000);

		/* Third, set GPMC for sync. */
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG1, &gpmc_cfg->cs[2].config1);
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG2, &gpmc_cfg->cs[2].config2);
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG3, &gpmc_cfg->cs[2].config3);
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG4, &gpmc_cfg->cs[2].config4);
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG5, &gpmc_cfg->cs[2].config5);
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG6, &gpmc_cfg->cs[2].config6);
		writel(LOGIC_STNOR_SYNC_GPMC_CONFIG7, &gpmc_cfg->cs[2].config7);
		/* And lastly, set the WAIT1 polarity high */
		writel(readl(&gpmc_cfg->config) | 0x200, &gpmc_cfg->config);
	} else
		puts ("NOR: Already initialized in sync mode\n");
}

int board_eth_init(bd_t *bis)
{
	int rc = 0;
#ifdef CONFIG_SMC911X
	rc = smc911x_initialize(0, CONFIG_SMC911X_BASE);
#endif
	return rc;
}

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
#define	SAFE_MODE_PINS_1
#define	SAFE_MODE_PINS_2
#define	SAFE_MODE_PINS_3
#define	SAFE_MODE_PINS_4
#undef	SAFE_MODE_PINS_5
#define	SAFE_MODE_PINS_5A
#define	SAFE_MODE_PINS_6
void set_muxconf_regs(void)
{
 /*SDRC*/
	MUX_VAL(CP(SDRC_D0),		(IEN  | PTD | DIS | M0)); /*SDRC_D0*/
	MUX_VAL(CP(SDRC_D1),		(IEN  | PTD | DIS | M0)); /*SDRC_D1*/
	MUX_VAL(CP(SDRC_D2),		(IEN  | PTD | DIS | M0)); /*SDRC_D2*/
	MUX_VAL(CP(SDRC_D3),		(IEN  | PTD | DIS | M0)); /*SDRC_D3*/
	MUX_VAL(CP(SDRC_D4),		(IEN  | PTD | DIS | M0)); /*SDRC_D4*/
	MUX_VAL(CP(SDRC_D5),		(IEN  | PTD | DIS | M0)); /*SDRC_D5*/
	MUX_VAL(CP(SDRC_D6),		(IEN  | PTD | DIS | M0)); /*SDRC_D6*/
	MUX_VAL(CP(SDRC_D7),		(IEN  | PTD | DIS | M0)); /*SDRC_D7*/
	MUX_VAL(CP(SDRC_D8),		(IEN  | PTD | DIS | M0)); /*SDRC_D8*/
	MUX_VAL(CP(SDRC_D9),		(IEN  | PTD | DIS | M0)); /*SDRC_D9*/
	MUX_VAL(CP(SDRC_D10),		(IEN  | PTD | DIS | M0)); /*SDRC_D10*/
	MUX_VAL(CP(SDRC_D11),		(IEN  | PTD | DIS | M0)); /*SDRC_D11*/
	MUX_VAL(CP(SDRC_D12),		(IEN  | PTD | DIS | M0)); /*SDRC_D12*/
	MUX_VAL(CP(SDRC_D13),		(IEN  | PTD | DIS | M0)); /*SDRC_D13*/
	MUX_VAL(CP(SDRC_D14),		(IEN  | PTD | DIS | M0)); /*SDRC_D14*/
	MUX_VAL(CP(SDRC_D15),		(IEN  | PTD | DIS | M0)); /*SDRC_D15*/
	MUX_VAL(CP(SDRC_D16),		(IEN  | PTD | DIS | M0)); /*SDRC_D16*/
	MUX_VAL(CP(SDRC_D17),		(IEN  | PTD | DIS | M0)); /*SDRC_D17*/
	MUX_VAL(CP(SDRC_D18),		(IEN  | PTD | DIS | M0)); /*SDRC_D18*/
	MUX_VAL(CP(SDRC_D19),		(IEN  | PTD | DIS | M0)); /*SDRC_D19*/
	MUX_VAL(CP(SDRC_D20),		(IEN  | PTD | DIS | M0)); /*SDRC_D20*/
	MUX_VAL(CP(SDRC_D21),		(IEN  | PTD | DIS | M0)); /*SDRC_D21*/
	MUX_VAL(CP(SDRC_D22),		(IEN  | PTD | DIS | M0)); /*SDRC_D22*/
	MUX_VAL(CP(SDRC_D23),		(IEN  | PTD | DIS | M0)); /*SDRC_D23*/
	MUX_VAL(CP(SDRC_D24),		(IEN  | PTD | DIS | M0)); /*SDRC_D24*/
	MUX_VAL(CP(SDRC_D25),		(IEN  | PTD | DIS | M0)); /*SDRC_D25*/
	MUX_VAL(CP(SDRC_D26),		(IEN  | PTD | DIS | M0)); /*SDRC_D26*/
	MUX_VAL(CP(SDRC_D27),		(IEN  | PTD | DIS | M0)); /*SDRC_D27*/
	MUX_VAL(CP(SDRC_D28),		(IEN  | PTD | DIS | M0)); /*SDRC_D28*/
	MUX_VAL(CP(SDRC_D29),		(IEN  | PTD | DIS | M0)); /*SDRC_D29*/
	MUX_VAL(CP(SDRC_D30),		(IEN  | PTD | DIS | M0)); /*SDRC_D30*/
	MUX_VAL(CP(SDRC_D31),		(IEN  | PTD | DIS | M0)); /*SDRC_D31*/
	MUX_VAL(CP(SDRC_CLK),		(IEN  | PTD | DIS | M0)); /*SDRC_CLK*/
	MUX_VAL(CP(SDRC_DQS0),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS0*/
	MUX_VAL(CP(SDRC_DQS1),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS1*/
	MUX_VAL(CP(SDRC_DQS2),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS2*/
	MUX_VAL(CP(SDRC_DQS3),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS3*/
 /*GPMC*/
	MUX_VAL(CP(GPMC_A1),		(IDIS | PTU | EN  | M0)); /*GPMC_A1*/
	MUX_VAL(CP(GPMC_A2),		(IDIS | PTU | EN  | M0)); /*GPMC_A2*/
	MUX_VAL(CP(GPMC_A3),		(IDIS | PTU | EN  | M0)); /*GPMC_A3*/
	MUX_VAL(CP(GPMC_A4),		(IDIS | PTU | EN  | M0)); /*GPMC_A4*/
	MUX_VAL(CP(GPMC_A5),		(IDIS | PTU | EN  | M0)); /*GPMC_A5*/
	MUX_VAL(CP(GPMC_A6),		(IDIS | PTU | EN  | M0)); /*GPMC_A6*/
	MUX_VAL(CP(GPMC_A7),		(IDIS | PTU | EN  | M0)); /*GPMC_A7*/
	MUX_VAL(CP(GPMC_A8),		(IDIS | PTU | EN  | M0)); /*GPMC_A8*/
	MUX_VAL(CP(GPMC_A9),		(IDIS | PTU | EN  | M0)); /*GPMC_A9*/
	MUX_VAL(CP(GPMC_A10),		(IDIS | PTU | EN  | M0)); /*GPMC_A10*/
	MUX_VAL(CP(GPMC_D0),		(IEN  | PTU | EN  | M0)); /*GPMC_D0*/
	MUX_VAL(CP(GPMC_D1),		(IEN  | PTU | EN  | M0)); /*GPMC_D1*/
	MUX_VAL(CP(GPMC_D2),		(IEN  | PTU | EN  | M0)); /*GPMC_D2*/
	MUX_VAL(CP(GPMC_D3),		(IEN  | PTU | EN  | M0)); /*GPMC_D3*/
	MUX_VAL(CP(GPMC_D4),		(IEN  | PTU | EN  | M0)); /*GPMC_D4*/
	MUX_VAL(CP(GPMC_D5),		(IEN  | PTU | EN  | M0)); /*GPMC_D5*/
	MUX_VAL(CP(GPMC_D6),		(IEN  | PTU | EN  | M0)); /*GPMC_D6*/
	MUX_VAL(CP(GPMC_D7),		(IEN  | PTU | EN  | M0)); /*GPMC_D7*/
	MUX_VAL(CP(GPMC_D8),		(IEN  | PTU | EN  | M0)); /*GPMC_D8*/
	MUX_VAL(CP(GPMC_D9),		(IEN  | PTU | EN  | M0)); /*GPMC_D9*/
	MUX_VAL(CP(GPMC_D10),		(IEN  | PTU | EN  | M0)); /*GPMC_D10*/
	MUX_VAL(CP(GPMC_D11),		(IEN  | PTU | EN  | M0)); /*GPMC_D11*/
	MUX_VAL(CP(GPMC_D12),		(IEN  | PTU | EN  | M0)); /*GPMC_D12*/
	MUX_VAL(CP(GPMC_D13),		(IEN  | PTU | EN  | M0)); /*GPMC_D13*/
	MUX_VAL(CP(GPMC_D14),		(IEN  | PTU | EN  | M0)); /*GPMC_D14*/
	MUX_VAL(CP(GPMC_D15),		(IEN  | PTU | EN  | M0)); /*GPMC_D15*/
	MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS0*/
	MUX_VAL(CP(GPMC_NCS1),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS1*/
	MUX_VAL(CP(GPMC_NCS2),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS2*/
	MUX_VAL(CP(GPMC_NCS3),		(IDIS | PTD | DIS | M0)); /*GPMC_nCS3*/
	MUX_VAL(CP(GPMC_NCS4),		(IEN  | PTU | EN  | M0)); /*GPMC_nCS4*/
	MUX_VAL(CP(GPMC_NCS5),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS5*/
	MUX_VAL(CP(GPMC_NCS6),		(IEN  | PTD | DIS | M0)); /*GPMC_nCS6*/
	MUX_VAL(CP(GPMC_NCS7),		(IDIS | PTD | DIS | M1)); /*GPMC_IO_DIR*/
	MUX_VAL(CP(GPMC_CLK),		(IEN  | PTD | DIS | M0)); /*GPMC_CLK*/
	MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)); /*GPMC_nADV_ALE*/
	MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)); /*GPMC_nOE*/
	MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)); /*GPMC_nWE*/
	MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTU | EN  | M0)); /*GPMC_nBE0_CLE*/
	MUX_VAL(CP(GPMC_NBE1),		(IEN  | PTU | EN  | M0)); /*GPMC_nBE1*/
	MUX_VAL(CP(GPMC_NWP),		(IEN  | PTD | DIS | M0)); /*GPMC_nWP*/
	MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT0*/
	MUX_VAL(CP(GPMC_WAIT1),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT1*/
#ifdef SAFE_MODE_PINS_1
	MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTD | EN  | M7)); /*safe mode */
#else
	MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTU | EN  | M4)); /*GPIO_64*/
								 /* - ETH_nRESET*/
#endif
	MUX_VAL(CP(GPMC_WAIT3),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT3*/
 /*DSS*/
#ifdef SAFE_MODE_PINS_2
	MUX_VAL(CP(DSS_PCLK),		(IEN  | PTD | EN  | M7)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC),		(IEN  | PTD | EN  | M7)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC),		(IEN  | PTD | EN  | M7)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS),		(IEN  | PTD | EN  | M7)); /*DSS_ACBIAS*/
	MUX_VAL(CP(DSS_DATA0),		(IEN  | PTD | EN  | M7)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA1),		(IEN  | PTD | EN  | M7)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA2),		(IEN  | PTD | EN  | M7)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA3),		(IEN  | PTD | EN  | M7)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA4),		(IEN  | PTD | EN  | M7)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA5),		(IEN  | PTD | EN  | M7)); /*DSS_DATA5*/
	MUX_VAL(CP(DSS_DATA6),		(IEN  | PTD | EN  | M7)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7),		(IEN  | PTD | EN  | M7)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8),		(IEN  | PTD | EN  | M7)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9),		(IEN  | PTD | EN  | M7)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10),		(IEN  | PTD | EN  | M7)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11),		(IEN  | PTD | EN  | M7)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12),		(IEN  | PTD | EN  | M7)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13),		(IEN  | PTD | EN  | M7)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14),		(IEN  | PTD | EN  | M7)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15),		(IEN  | PTD | EN  | M7)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16),		(IEN  | PTD | EN  | M7)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17),		(IEN  | PTD | EN  | M7)); /*DSS_DATA17*/
	MUX_VAL(CP(DSS_DATA18),		(IEN  | PTD | EN  | M7)); /*DSS_DATA18*/
	MUX_VAL(CP(DSS_DATA19),		(IEN  | PTD | EN  | M7)); /*DSS_DATA19*/
	MUX_VAL(CP(DSS_DATA20),		(IEN  | PTD | EN  | M7)); /*DSS_DATA20*/
	MUX_VAL(CP(DSS_DATA21),		(IEN  | PTD | EN  | M7)); /*DSS_DATA21*/
	MUX_VAL(CP(DSS_DATA22),		(IEN  | PTD | EN  | M7)); /*DSS_DATA22*/
	MUX_VAL(CP(DSS_DATA23),		(IEN  | PTD | EN  | M7)); /*DSS_DATA23*/
 /*CAMERA*/
	MUX_VAL(CP(CAM_HS),		(IEN  | PTD | EN  | M7)); /*CAM_HS */
	MUX_VAL(CP(CAM_VS),		(IEN  | PTD | EN  | M7)); /*CAM_VS */
	MUX_VAL(CP(CAM_XCLKA),		(IEN  | PTD | EN  | M7)); /*CAM_XCLKA*/
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTD | EN  | M7)); /*CAM_PCLK*/
	MUX_VAL(CP(CAM_FLD),		(IEN  | PTD | EN  | M7)); /*GPIO_98*/
								 /* - CAM_RESET*/
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | EN  | M7)); /*CAM_D0*/
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | EN  | M7)); /*CAM_D1*/
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | EN  | M7)); /*CAM_D2*/
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | EN  | M7)); /*CAM_D3*/
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | EN  | M7)); /*CAM_D4*/
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | EN  | M7)); /*CAM_D5*/
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | EN  | M7)); /*CAM_D6*/
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | EN  | M7)); /*CAM_D7*/
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | EN  | M7)); /*CAM_D8*/
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | EN  | M7)); /*CAM_D9*/
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | EN  | M7)); /*CAM_D10*/
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | EN  | M7)); /*CAM_D11*/
	MUX_VAL(CP(CAM_XCLKB),		(IEN  | PTD | EN  | M7)); /*CAM_XCLKB*/
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | EN  | M7)); /*GPIO_167*/
	MUX_VAL(CP(CAM_STROBE),		(IEN  | PTD | EN  | M7)); /*CAM_STROBE*/
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M7)); /*CSI2_DX0*/
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M7)); /*CSI2_DY0*/
	MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | EN  | M7)); /*CSI2_DX1*/
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M7)); /*CSI2_DY1*/
 /*Audio Interface */
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | EN  | M7)); /*McBSP2_FSX*/
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | EN  | M7)); /*McBSP2_CLKX*/
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | EN  | M7)); /*McBSP2_DR*/
	MUX_VAL(CP(MCBSP2_DX),		(IEN  | PTD | EN  | M7)); /*McBSP2_DX*/
#else
	MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | EN  | M0)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC),		(IDIS | PTD | EN  | M0)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC),		(IDIS | PTD | EN  | M0)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS),		(IDIS | PTD | EN  | M0)); /*DSS_ACBIAS*/
	MUX_VAL(CP(DSS_DATA0),		(IDIS | PTD | EN  | M0)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA1),		(IDIS | PTD | EN  | M0)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA2),		(IDIS | PTD | EN  | M0)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA3),		(IDIS | PTD | EN  | M0)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA4),		(IDIS | PTD | EN  | M0)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | EN  | M0)); /*DSS_DATA5*/
	MUX_VAL(CP(DSS_DATA6),		(IDIS | PTD | EN  | M0)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7),		(IDIS | PTD | EN  | M0)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8),		(IDIS | PTD | EN  | M0)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9),		(IDIS | PTD | EN  | M0)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10),		(IDIS | PTD | EN  | M0)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11),		(IDIS | PTD | EN  | M0)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12),		(IDIS | PTD | EN  | M0)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13),		(IDIS | PTD | EN  | M0)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14),		(IDIS | PTD | EN  | M0)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15),		(IDIS | PTD | EN  | M0)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16),		(IDIS | PTD | EN  | M0)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17),		(IDIS | PTD | EN  | M0)); /*DSS_DATA17*/
	MUX_VAL(CP(DSS_DATA18),		(IDIS | PTD | EN  | M0)); /*DSS_DATA18*/
	MUX_VAL(CP(DSS_DATA19),		(IDIS | PTD | EN  | M0)); /*DSS_DATA19*/
	MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | EN  | M0)); /*DSS_DATA20*/
	MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | EN  | M0)); /*DSS_DATA21*/
	MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | EN  | M0)); /*DSS_DATA22*/
	MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | EN  | M0)); /*DSS_DATA23*/
 /*CAMERA*/
	MUX_VAL(CP(CAM_HS),		(IEN  | PTD | EN  | M0)); /*CAM_HS */
	MUX_VAL(CP(CAM_VS),		(IEN  | PTD | EN  | M0)); /*CAM_VS */
	MUX_VAL(CP(CAM_XCLKA),		(IDIS | PTD | EN  | M0)); /*CAM_XCLKA*/
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTD | EN  | M0)); /*CAM_PCLK*/
	MUX_VAL(CP(CAM_FLD),		(IDIS | PTD | EN  | M4)); /*GPIO_98*/
								 /* - CAM_RESET*/
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | EN  | M0)); /*CAM_D0*/
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | EN  | M0)); /*CAM_D1*/
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | EN  | M0)); /*CAM_D2*/
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | EN  | M0)); /*CAM_D3*/
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | EN  | M0)); /*CAM_D4*/
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | EN  | M0)); /*CAM_D5*/
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | EN  | M0)); /*CAM_D6*/
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | EN  | M0)); /*CAM_D7*/
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | EN  | M0)); /*CAM_D8*/
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | EN  | M0)); /*CAM_D9*/
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | EN  | M0)); /*CAM_D10*/
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | EN  | M0)); /*CAM_D11*/
	MUX_VAL(CP(CAM_XCLKB),		(IDIS | PTD | EN  | M0)); /*CAM_XCLKB*/
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | EN  | M4)); /*GPIO_167*/
	MUX_VAL(CP(CAM_STROBE),		(IDIS | PTD | EN  | M0)); /*CAM_STROBE*/
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M0)); /*CSI2_DX0*/
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M0)); /*CSI2_DY0*/
	MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | EN  | M0)); /*CSI2_DX1*/
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M0)); /*CSI2_DY1*/
 /*Audio Interface */
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | EN  | M0)); /*McBSP2_FSX*/
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | EN  | M0)); /*McBSP2_CLKX*/
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | EN  | M0)); /*McBSP2_DR*/
	MUX_VAL(CP(MCBSP2_DX),		(IDIS | PTD | EN  | M0)); /*McBSP2_DX*/
#endif
 /*Expansion card  */
	MUX_VAL(CP(MMC1_CLK),		(IDIS | PTU | EN  | M0)); /*MMC1_CLK*/
	MUX_VAL(CP(MMC1_CMD),		(IEN  | PTU | EN  | M0)); /*MMC1_CMD*/
	MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT0*/
	MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT1*/
	MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT2*/
	MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT3*/
#ifdef SAFE_MODE_PINS_3
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT4*/
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT5*/
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT6*/
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTD | EN  | M7)); /*MMC1_DAT7*/
 /*Wireless LAN */
	MUX_VAL(CP(MMC2_CLK),		(IEN  | PTD | EN  | M7)); /*MMC2_CLK*/
	MUX_VAL(CP(MMC2_CMD),		(IEN  | PTD | EN  | M7)); /*MMC2_CMD*/
	MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT0*/
	MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT1*/
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT2*/
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT3*/
	MUX_VAL(CP(MMC2_DAT4),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT4*/
	MUX_VAL(CP(MMC2_DAT5),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT5*/
	MUX_VAL(CP(MMC2_DAT6),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT6 */
	MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTD | EN  | M7)); /*MMC2_DAT7*/
 /*Bluetooth*/
	MUX_VAL(CP(MCBSP3_DX),		(IEN  | PTD | EN  | M7)); /*McBSP3_DX*/
	MUX_VAL(CP(MCBSP3_DR),		(IEN  | PTD | EN  | M7)); /*McBSP3_DR*/
	MUX_VAL(CP(MCBSP3_CLKX),	(IEN  | PTD | EN  | M7)); /*McBSP3_CLKX  */
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | EN  | M7)); /*McBSP3_FSX*/
	MUX_VAL(CP(UART2_CTS),		(IEN  | PTD | EN  | M7)); /*UART2_CTS*/
	MUX_VAL(CP(UART2_RTS),		(IEN  | PTD | EN  | M7)); /*UART2_RTS*/
	MUX_VAL(CP(UART2_TX),		(IEN  | PTD | EN  | M7)); /*UART2_TX*/
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | EN  | M7)); /*UART2_RX*/
#else
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT4*/
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT5*/
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT6*/
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT7*/
 /*Wireless LAN */
	MUX_VAL(CP(MMC2_CLK),		(IEN  | PTD | DIS | M0)); /*MMC2_CLK*/
	MUX_VAL(CP(MMC2_CMD),		(IEN  | PTU | EN  | M0)); /*MMC2_CMD*/
	MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT0*/
	MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT1*/
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT2*/
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT3*/
	MUX_VAL(CP(MMC2_DAT4),		(IDIS | PTD | DIS | M0)); /*MMC2_DAT4*/
	MUX_VAL(CP(MMC2_DAT5),		(IDIS | PTD | DIS | M0)); /*MMC2_DAT5*/
	MUX_VAL(CP(MMC2_DAT6),		(IDIS | PTD | DIS | M0)); /*MMC2_DAT6 */
	MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT7*/
 /*Bluetooth*/
	MUX_VAL(CP(MCBSP3_DX),		(IDIS | PTD | DIS | M0)); /*McBSP3_DX*/
	MUX_VAL(CP(MCBSP3_DR),		(IEN  | PTD | DIS | M0)); /*McBSP3_DR*/
	MUX_VAL(CP(MCBSP3_CLKX),	(IEN  | PTD | DIS | M0)); /*McBSP3_CLKX  */
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | DIS | M0)); /*McBSP3_FSX*/
	MUX_VAL(CP(UART2_CTS),		(IEN  | PTU | EN  | M0)); /*UART2_CTS*/
	MUX_VAL(CP(UART2_RTS),		(IDIS | PTD | DIS | M0)); /*UART2_RTS*/
	MUX_VAL(CP(UART2_TX),		(IDIS | PTD | DIS | M0)); /*UART2_TX*/
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | DIS | M0)); /*UART2_RX*/
#endif
 /*Modem Interface */
	MUX_VAL(CP(UART1_TX),		(IDIS | PTD | DIS | M0)); /*UART1_TX*/
	MUX_VAL(CP(UART1_RTS),		(IDIS | PTD | DIS | M0)); /*UART1_RTS*/
	MUX_VAL(CP(UART1_CTS),		(IEN  | PTU | DIS | M0)); /*UART1_CTS*/
	MUX_VAL(CP(UART1_RX),		(IEN  | PTD | DIS | M0)); /*UART1_RX*/
#ifdef SAFE_MODE_PINS_4
	MUX_VAL(CP(MCBSP4_CLKX),	(IEN  | PTD | EN  | M7)); /*GPIO_152*/
								 /* - LCD_INI*/
	MUX_VAL(CP(MCBSP4_DR),		(IEN  | PTD | EN  | M7)); /*GPIO_153*/
								 /* - LCD_ENVDD */
	MUX_VAL(CP(MCBSP4_DX),		(IEN  | PTD | EN  | M7)); /*GPIO_154*/
								 /* - LCD_QVGA/nVGA */
	MUX_VAL(CP(MCBSP4_FSX),		(IEN  | PTD | EN  | M7)); /*GPIO_155*/
								 /* - LCD_RESB */
	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | EN  | M7)); /*MCBSP1_CLKR  */
	MUX_VAL(CP(MCBSP1_FSR),		(IEN  | PTD | EN  | M7)); /*MCBSP1_FSR*/
	MUX_VAL(CP(MCBSP1_DX),		(IEN  | PTD | EN  | M7)); /*MCBSP1_DX*/
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | EN  | M7)); /*MCBSP1_DR*/
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTD | EN  | M7)); /*MCBSP_CLKS  */
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | EN  | M7)); /*MCBSP1_FSX*/
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | EN  | M7)); /*MCBSP1_CLKX  */
 /*Serial Interface*/
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M7)); /*UART3_CTS_*/
								 /* RCTX*/
	MUX_VAL(CP(UART3_RTS_SD),	(IEN  | PTD | EN  | M7)); /*UART3_RTS_SD */
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | EN  | M7)); /*UART3_RX_IRRX*/
	MUX_VAL(CP(UART3_TX_IRTX),	(IEN  | PTD | EN  | M7)); /*UART3_TX_IRTX*/
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | EN  | M7)); /*HSUSB0_CLK*/
	MUX_VAL(CP(HSUSB0_STP),		(IEN  | PTD | EN  | M7)); /*HSUSB0_STP*/
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | EN  | M7)); /*HSUSB0_DIR*/
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | EN  | M7)); /*HSUSB0_NXT*/
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA0*/
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA1*/
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA2*/
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA3*/
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA4*/
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA5*/
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA6*/
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | EN  | M7)); /*HSUSB0_DATA7*/
#else
	MUX_VAL(CP(MCBSP4_CLKX),	(IDIS | PTD | DIS | M4)); /*GPIO_152*/
								 /* - LCD_INI*/
	MUX_VAL(CP(MCBSP4_DR),		(IDIS | PTD | DIS | M4)); /*GPIO_153*/
								 /* - LCD_ENVDD */
	MUX_VAL(CP(MCBSP4_DX),		(IDIS | PTD | DIS | M4)); /*GPIO_154*/
								 /* - LCD_QVGA/nVGA */
	MUX_VAL(CP(MCBSP4_FSX),		(IDIS | PTD | DIS | M4)); /*GPIO_155*/
								 /* - LCD_RESB */
	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | DIS | M0)); /*MCBSP1_CLKR  */
	MUX_VAL(CP(MCBSP1_FSR),		(IDIS | PTU | EN  | M0)); /*MCBSP1_FSR*/
	MUX_VAL(CP(MCBSP1_DX),		(IDIS | PTD | DIS | M0)); /*MCBSP1_DX*/
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | DIS | M0)); /*MCBSP1_DR*/
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTU | DIS | M0)); /*MCBSP_CLKS  */
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | DIS | M0)); /*MCBSP1_FSX*/
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | DIS | M0)); /*MCBSP1_CLKX  */
 /*Serial Interface*/
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M0)); /*UART3_CTS_*/
								 /* RCTX*/
	MUX_VAL(CP(UART3_RTS_SD),	(IDIS | PTD | DIS | M0)); /*UART3_RTS_SD */
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | DIS | M0)); /*UART3_RX_IRRX*/
	MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)); /*UART3_TX_IRTX*/
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | DIS | M0)); /*HSUSB0_CLK*/
	MUX_VAL(CP(HSUSB0_STP),		(IDIS | PTU | EN  | M0)); /*HSUSB0_STP*/
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | DIS | M0)); /*HSUSB0_DIR*/
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | DIS | M0)); /*HSUSB0_NXT*/
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA0*/
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA1*/
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA2*/
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA3*/
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA4*/
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA5*/
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA6*/
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA7*/
#endif
	MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)); /*I2C1_SCL*/
	MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)); /*I2C1_SDA*/
#ifdef SAFE_MODE_PINS_5
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTD | EN  | M7)); /*I2C2_SCL*/
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTD | EN  | M7)); /*I2C2_SDA*/
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTD | EN  | M7)); /*I2C3_SCL*/
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTD | EN  | M7)); /*I2C3_SDA*/
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTD | EN  | M7)); /*I2C4_SCL*/
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTD | EN  | M7)); /*I2C4_SDA*/
#else
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | EN  | M0)); /*I2C2_SCL*/
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | EN  | M0)); /*I2C2_SDA*/
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M0)); /*I2C3_SCL*/
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M0)); /*I2C3_SDA*/
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)); /*I2C4_SCL*/
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)); /*I2C4_SDA*/
#endif
	MUX_VAL(CP(HDQ_SIO),		(IEN  | PTU | EN  | M0)); /*HDQ_SIO*/
#ifdef SAFE_MODE_PINS_5A
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | EN  | M7)); /*McSPI1_CLK*/
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | EN  | M7)); /*McSPI1_SIMO  */
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | EN  | M7)); /*McSPI1_SOMI  */
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M7)); /*McSPI1_CS0*/
	MUX_VAL(CP(MCSPI1_CS1),		(IEN  | PTD | EN  | M7)); /*GPIO_175*/
								 /* TS_PEN_IRQ */
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTD | EN  | M7)); /*GPIO_176*/
								 /* - LAN_INTR*/
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | EN  | M7)); /*McSPI1_CS3*/
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | EN  | M7)); /*McSPI2_CLK*/
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | EN  | M7)); /*McSPI2_SIMO*/
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | EN  | M7)); /*McSPI2_SOMI*/
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | EN  | M7)); /*McSPI2_CS0*/
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | EN  | M7)); /*McSPI2_CS1*/
#else
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | DIS | M0)); /*McSPI1_CLK*/
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | DIS | M0)); /*McSPI1_SIMO  */
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | DIS | M0)); /*McSPI1_SOMI  */
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M0)); /*McSPI1_CS0*/
	MUX_VAL(CP(MCSPI1_CS1),		(IEN  | PTD | EN  | M4)); /*GPIO_175*/
								 /* TS_PEN_IRQ */
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTU | DIS | M4)); /*GPIO_176*/
								 /* - LAN_INTR*/
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | EN  | M0)); /*McSPI1_CS3*/
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | DIS | M0)); /*McSPI2_CLK*/
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | DIS | M0)); /*McSPI2_SIMO*/
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | DIS | M0)); /*McSPI2_SOMI*/
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | EN  | M0)); /*McSPI2_CS0*/
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | EN  | M0)); /*McSPI2_CS1*/
#endif
 /*Control and debug */
	MUX_VAL(CP(SYS_32K),		(IEN  | PTD | DIS | M0)); /*SYS_32K*/
	MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)); /*SYS_CLKREQ*/
	MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTU | EN  | M0)); /*SYS_nIRQ*/
#ifdef SAFE_MODE_PINS_6
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | EN  | M7)); /*GPIO_2*/
								 /* - PEN_IRQ */
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | EN  | M7)); /*GPIO_3 */
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | EN  | M7)); /*GPIO_4*/
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | EN  | M7)); /*GPIO_5*/
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | EN  | M7)); /*GPIO_6*/
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | EN  | M7)); /*GPIO_7*/
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | EN  | M7)); /*GPIO_8*/
#else
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)); /*GPIO_2*/
								 /* - PEN_IRQ */
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)); /*GPIO_3 */
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M4)); /*GPIO_4*/
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M4)); /*GPIO_5*/
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M4)); /*GPIO_6*/
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M4)); /*GPIO_7*/
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | DIS | M4)); /*GPIO_8*/
#endif								 /* - VIO_1V8*/
	MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)); /*SYS_OFF_MODE*/
	MUX_VAL(CP(SYS_CLKOUT1),	(IEN  | PTD | DIS | M0)); /*SYS_CLKOUT1*/
	MUX_VAL(CP(SYS_CLKOUT2),	(IEN  | PTU | EN  | M0)); /*SYS_CLKOUT2*/
	MUX_VAL(CP(JTAG_nTRST),		(IEN  | PTD | DIS | M0)); /*JTAG_nTRST*/
	MUX_VAL(CP(JTAG_TCK),		(IEN  | PTD | DIS | M0)); /*JTAG_TCK*/
	MUX_VAL(CP(JTAG_TMS),		(IEN  | PTD | DIS | M0)); /*JTAG_TMS*/
	MUX_VAL(CP(JTAG_TDI),		(IEN  | PTD | DIS | M0)); /*JTAG_TDI*/
	MUX_VAL(CP(JTAG_EMU0),		(IEN  | PTD | DIS | M0)); /*JTAG_EMU0*/
	MUX_VAL(CP(JTAG_EMU1),		(IEN  | PTD | DIS | M0)); /*JTAG_EMU1*/
	MUX_VAL(CP(ETK_CLK_ES2),	(IDIS | PTU | EN  | M0)); /*ETK_CLK*/
	MUX_VAL(CP(ETK_CTL_ES2),	(IDIS | PTD | DIS | M0)); /*ETK_CTL*/
	MUX_VAL(CP(ETK_D0_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D0*/
	MUX_VAL(CP(ETK_D1_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D1*/
	MUX_VAL(CP(ETK_D2_ES2 ),	(IEN  | PTD | EN  | M0)); /*ETK_D2*/
	MUX_VAL(CP(ETK_D3_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D3*/
	MUX_VAL(CP(ETK_D4_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D4*/
	MUX_VAL(CP(ETK_D5_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D5*/
	MUX_VAL(CP(ETK_D6_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D6*/
	MUX_VAL(CP(ETK_D7_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D7*/
	MUX_VAL(CP(ETK_D8_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D8*/
	MUX_VAL(CP(ETK_D9_ES2 ),	(IEN  | PTD | DIS | M0)); /*ETK_D9*/
	MUX_VAL(CP(ETK_D10_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D10*/
	MUX_VAL(CP(ETK_D11_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D11*/
	MUX_VAL(CP(ETK_D12_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D12*/
	MUX_VAL(CP(ETK_D13_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D13*/
	MUX_VAL(CP(ETK_D14_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D14*/
	MUX_VAL(CP(ETK_D15_ES2),	(IEN  | PTD | DIS | M0)); /*ETK_D15*/
 /*Die to Die */
	MUX_VAL(CP(D2D_MCAD1),		(IEN  | PTD | EN  | M0)); /*d2d_mcad1*/
	MUX_VAL(CP(D2D_MCAD2),		(IEN  | PTD | EN  | M0)); /*d2d_mcad2*/
	MUX_VAL(CP(D2D_MCAD3),		(IEN  | PTD | EN  | M0)); /*d2d_mcad3*/
	MUX_VAL(CP(D2D_MCAD4),		(IEN  | PTD | EN  | M0)); /*d2d_mcad4*/
	MUX_VAL(CP(D2D_MCAD5),		(IEN  | PTD | EN  | M0)); /*d2d_mcad5*/
	MUX_VAL(CP(D2D_MCAD6),		(IEN  | PTD | EN  | M0)); /*d2d_mcad6*/
	MUX_VAL(CP(D2D_MCAD7),		(IEN  | PTD | EN  | M0)); /*d2d_mcad7*/
	MUX_VAL(CP(D2D_MCAD8),		(IEN  | PTD | EN  | M0)); /*d2d_mcad8*/
	MUX_VAL(CP(D2D_MCAD9),		(IEN  | PTD | EN  | M0)); /*d2d_mcad9*/
	MUX_VAL(CP(D2D_MCAD10),		(IEN  | PTD | EN  | M0)); /*d2d_mcad10*/
	MUX_VAL(CP(D2D_MCAD11),		(IEN  | PTD | EN  | M0)); /*d2d_mcad11*/
	MUX_VAL(CP(D2D_MCAD12),		(IEN  | PTD | EN  | M0)); /*d2d_mcad12*/
	MUX_VAL(CP(D2D_MCAD13),		(IEN  | PTD | EN  | M0)); /*d2d_mcad13*/
	MUX_VAL(CP(D2D_MCAD14),		(IEN  | PTD | EN  | M0)); /*d2d_mcad14*/
	MUX_VAL(CP(D2D_MCAD15),		(IEN  | PTD | EN  | M0)); /*d2d_mcad15*/
	MUX_VAL(CP(D2D_MCAD16),		(IEN  | PTD | EN  | M0)); /*d2d_mcad16*/
	MUX_VAL(CP(D2D_MCAD17),		(IEN  | PTD | EN  | M0)); /*d2d_mcad17*/
	MUX_VAL(CP(D2D_MCAD18),		(IEN  | PTD | EN  | M0)); /*d2d_mcad18*/
	MUX_VAL(CP(D2D_MCAD19),		(IEN  | PTD | EN  | M0)); /*d2d_mcad19*/
	MUX_VAL(CP(D2D_MCAD20),		(IEN  | PTD | EN  | M0)); /*d2d_mcad20*/
	MUX_VAL(CP(D2D_MCAD21),		(IEN  | PTD | EN  | M0)); /*d2d_mcad21*/
	MUX_VAL(CP(D2D_MCAD22),		(IEN  | PTD | EN  | M0)); /*d2d_mcad22*/
	MUX_VAL(CP(D2D_MCAD23),		(IEN  | PTD | EN  | M0)); /*d2d_mcad23*/
	MUX_VAL(CP(D2D_MCAD24),		(IEN  | PTD | EN  | M0)); /*d2d_mcad24*/
	MUX_VAL(CP(D2D_MCAD25),		(IEN  | PTD | EN  | M0)); /*d2d_mcad25*/
	MUX_VAL(CP(D2D_MCAD26),		(IEN  | PTD | EN  | M0)); /*d2d_mcad26*/
	MUX_VAL(CP(D2D_MCAD27),		(IEN  | PTD | EN  | M0)); /*d2d_mcad27*/
	MUX_VAL(CP(D2D_MCAD28),		(IEN  | PTD | EN  | M0)); /*d2d_mcad28*/
	MUX_VAL(CP(D2D_MCAD29),		(IEN  | PTD | EN  | M0)); /*d2d_mcad29*/
	MUX_VAL(CP(D2D_MCAD30),		(IEN  | PTD | EN  | M0)); /*d2d_mcad30*/
	MUX_VAL(CP(D2D_MCAD31),		(IEN  | PTD | EN  | M0)); /*d2d_mcad31*/
	MUX_VAL(CP(D2D_MCAD32),		(IEN  | PTD | EN  | M0)); /*d2d_mcad32*/
	MUX_VAL(CP(D2D_MCAD33),		(IEN  | PTD | EN  | M0)); /*d2d_mcad33*/
	MUX_VAL(CP(D2D_MCAD34),		(IEN  | PTD | EN  | M0)); /*d2d_mcad34*/
	MUX_VAL(CP(D2D_MCAD35),		(IEN  | PTD | EN  | M0)); /*d2d_mcad35*/
	MUX_VAL(CP(D2D_MCAD36),		(IEN  | PTD | EN  | M0)); /*d2d_mcad36*/
	MUX_VAL(CP(D2D_CLK26MI),	(IEN  | PTD | DIS | M0)); /*d2d_clk26mi*/
	MUX_VAL(CP(D2D_NRESPWRON),	(IEN  | PTD | EN  | M0)); /*d2d_nrespwron*/
	MUX_VAL(CP(D2D_NRESWARM),	(IEN  | PTU | EN  | M0)); /*d2d_nreswarm */
	MUX_VAL(CP(D2D_ARM9NIRQ),	(IEN  | PTD | DIS | M0)); /*d2d_arm9nirq */
	MUX_VAL(CP(D2D_UMA2P6FIQ),	(IEN  | PTD | DIS | M0)); /*d2d_uma2p6fiq*/
	MUX_VAL(CP(D2D_SPINT),		(IEN  | PTD | EN  | M0)); /*d2d_spint*/
	MUX_VAL(CP(D2D_FRINT),		(IEN  | PTD | EN  | M0)); /*d2d_frint*/
	MUX_VAL(CP(D2D_DMAREQ0),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq0*/
	MUX_VAL(CP(D2D_DMAREQ1),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq1*/
	MUX_VAL(CP(D2D_DMAREQ2),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq2*/
	MUX_VAL(CP(D2D_DMAREQ3),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq3*/
	MUX_VAL(CP(D2D_N3GTRST),	(IEN  | PTD | DIS | M0)); /*d2d_n3gtrst*/
	MUX_VAL(CP(D2D_N3GTDI),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtdi*/
	MUX_VAL(CP(D2D_N3GTDO),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtdo*/
	MUX_VAL(CP(D2D_N3GTMS),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtms*/
	MUX_VAL(CP(D2D_N3GTCK),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtck*/
	MUX_VAL(CP(D2D_N3GRTCK),	(IEN  | PTD | DIS | M0)); /*d2d_n3grtck*/
	MUX_VAL(CP(D2D_MSTDBY),		(IEN  | PTU | EN  | M0)); /*d2d_mstdby*/
	MUX_VAL(CP(D2D_SWAKEUP),	(IEN  | PTD | EN  | M0)); /*d2d_swakeup*/
	MUX_VAL(CP(D2D_IDLEREQ),	(IEN  | PTD | DIS | M0)); /*d2d_idlereq*/
	MUX_VAL(CP(D2D_IDLEACK),	(IEN  | PTU | EN  | M0)); /*d2d_idleack*/
	MUX_VAL(CP(D2D_MWRITE),		(IEN  | PTD | DIS | M0)); /*d2d_mwrite*/
	MUX_VAL(CP(D2D_SWRITE),		(IEN  | PTD | DIS | M0)); /*d2d_swrite*/
	MUX_VAL(CP(D2D_MREAD),		(IEN  | PTD | DIS | M0)); /*d2d_mread*/
	MUX_VAL(CP(D2D_SREAD),		(IEN  | PTD | DIS | M0)); /*d2d_sread*/
	MUX_VAL(CP(D2D_MBUSFLAG),	(IEN  | PTD | DIS | M0)); /*d2d_mbusflag*/
	MUX_VAL(CP(D2D_SBUSFLAG),	(IEN  | PTD | DIS | M0)); /*d2d_sbusflag*/
	MUX_VAL(CP(SDRC_CKE0),		(IDIS | PTU | EN  | M0)); /*sdrc_cke0*/
	MUX_VAL(CP(SDRC_CKE1),		(IDIS | PTD | DIS | M7)); /*sdrc_cke1*/
}
