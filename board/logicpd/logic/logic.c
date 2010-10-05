/*
 * (C) Copyright 2009
 * Logic Product Development, <www.logicpd.com>
 *
 * Author :
 *	Peter Barada <peterb@logicpd.com>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
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
#include <twl4030.h>
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/gpio.h>
#include <asm/mach-types.h>
#include "logic.h"
#include "product_id.h"
#include <nand.h>

#define MUX_LOGIC_HSUSB0_D5_GPIO_MUX()					\
 MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M4)) /*GPIO_189*/

#define MUX_LOGIC_HSUSB0_D5_DATA5()					\
 MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA5*/

/*
 * Routine: logic_identify
 * Description: Detect if we are running on a Logic or Torpedo.
 *              This can be done by GPIO_189. If its low after driving it high,
 *              then its an LOGIC, else Torpedo.
 */
unsigned int logic_identify(void)
{
	unsigned int val = 0;
	int i;

	MUX_LOGIC_HSUSB0_D5_GPIO_MUX();

	if (!omap_request_gpio(189)) {

		omap_set_gpio_direction(189, 0);
		omap_set_gpio_dataout(189, 1);

		// Let it soak for a bit
		for (i=0; i<0x100; ++i)
			asm("nop");

		omap_set_gpio_direction(189, 1);
		val = omap_get_gpio_datain(189);
		omap_free_gpio(189);

		printf("Board: ");
		if (val) {
			printf("Torpedo\n");
			val = MACH_TYPE_OMAP3_TORPEDO;
		} else {
			printf("LV SOM\n");
			val = MACH_TYPE_OMAP3530_LV_SOM;
		}
	}
	return val;
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

static void setup_cf_gpmc_setup(void)
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

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */

	/* Update NAND settings */
	setup_nand_settings();

#if 0
	/* Update CF settings */
	setup_cf_gpmc_setup();
#endif

	/* board id for Linux (placeholder until can ID board) */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3530_LV_SOM;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

static void setup_net_chip(void);
static void setup_isp1760_chip(void);
static void fix_flash_sync(void);

/* Turn on VAUX1 voltage to 3.0 volts to drive level shifters and
 * power 3.0v parts (tsc2004 and Product ID chip) */
#define I2C_TRITON2 0x4b /* Address of Triton power group */

void init_vaux1_voltage(void)
{
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	unsigned char data;
	unsigned short msg;

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	// Select the output voltage
	data = 0x04;
	i2c_write(TWL4030_CHIP_PM_MASTER, 0x72, 1, &data, 1);
	// Select the Processor resource group
	data = 0x20;
	i2c_write(TWL4030_CHIP_PM_MASTER, 0x72, 1, &data, 1);
	// Enable I2C access to the Power bus
	data = 0x02;
	i2c_write(TWL4030_CHIP_PM_MASTER, 0x4a, 1, &data, 1);
	// Send message MSB
	msg = (1<<13) | (1<<4) | (0xd<<0); // group(process_grp1):resource(vaux1):res_active;
	data = msg >> 8;
	i2c_write(TWL4030_CHIP_PM_MASTER, 0x4b, 1, &data, 1);
	// Send message LSB
	data = msg & 0xff;
	i2c_write(TWL4030_CHIP_PM_MASTER, 0x4c, 1, &data, 1);
#endif
}

static void enable_charger()
{
	u8 val;
	twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER, val, TWL4030_PM_MASTER_BOOT_BCI);
	val |= 0x37;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER, val, TWL4030_PM_MASTER_BOOT_BCI);

	puts("Charger enabled\n");
}

/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */
int misc_init_r(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	struct gpio *gpio5_base = (struct gpio *)OMAP34XX_GPIO5_BASE;
	struct gpio *gpio6_base = (struct gpio *)OMAP34XX_GPIO6_BASE;

#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
//	enable_charger();

	/* Turn on vaux1 to make sure voltage is to the product ID chip.
	 * Extract production data from ID chip, used to selectively
	 * initialize portions of the system */
	init_vaux1_voltage();
	fetch_production_data();

	twl4030_power_init();

#if defined(CONFIG_CMD_NET)
	setup_net_chip();
#endif

	/* Setup access to the isp1760 chip on CS6 */
	setup_isp1760_chip();

	twl4030_led_init(TWL4030_LED_LEDEN_LEDAON | TWL4030_LED_LEDEN_LEDBON);

	/* Configure GPIOs to output */
	writel(~(GPIO23 | GPIO10 | GPIO8 | GPIO2 | GPIO1), &gpio6_base->oe);
	writel(~(GPIO31 | GPIO30 | GPIO29 | GPIO28 | GPIO22 | GPIO21 |
		GPIO15 | GPIO14 | GPIO13 | GPIO12), &gpio5_base->oe);

	/* Set GPIOs */
	writel(GPIO23 | GPIO10 | GPIO8 | GPIO2 | GPIO1,
		&gpio6_base->setdataout);
	writel(GPIO31 | GPIO30 | GPIO29 | GPIO28 | GPIO22 | GPIO21 |
		GPIO15 | GPIO14 | GPIO13 | GPIO12, &gpio5_base->setdataout);

	gd->bd->bi_arch_number = logic_identify();

	/* Fix the flash sync */
	fix_flash_sync();

	dieid_num_r();

	return 0;
}


/******************************************************************************
 * Routine: late_board_init
 * Description: Late hardware init.
 *****************************************************************************/
int board_late_init(void)
{
	unsigned char enetaddr[6];

	// DECLARE_GLOBAL_DATA_PTR;

	dump_production_data(); // Dump production data

	// Fetch the ethaddr of the LAN
	board_get_nth_enetaddr(enetaddr, 0, 0);
#ifdef CONFIG_HAS_ETH1
	// Fetch the ethaddr of the WiFi
	board_get_nth_enetaddr(enetaddr, 1, 1);
#endif

#ifdef CONFIG_CMD_NAND_LOCK_UNLOCK
	// Unlock the whole chip
	nand_unlock(&nand_info[0], 0x0, nand_info[0].size);
#endif
	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
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

	/*SDRC*/
	MUX_VAL(CP(SDRC_D0), (IEN  | PTD | DIS | M0)); /*SDRC_D0*/
	MUX_VAL(CP(SDRC_D1), (IEN  | PTD | DIS | M0)); /*SDRC_D1*/
	MUX_VAL(CP(SDRC_D2), (IEN  | PTD | DIS | M0)); /*SDRC_D2*/
	MUX_VAL(CP(SDRC_D3), (IEN  | PTD | DIS | M0)); /*SDRC_D3*/
	MUX_VAL(CP(SDRC_D4), (IEN  | PTD | DIS | M0)); /*SDRC_D4*/
	MUX_VAL(CP(SDRC_D5), (IEN  | PTD | DIS | M0)); /*SDRC_D5*/
	MUX_VAL(CP(SDRC_D6), (IEN  | PTD | DIS | M0)); /*SDRC_D6*/
	MUX_VAL(CP(SDRC_D7), (IEN  | PTD | DIS | M0)); /*SDRC_D7*/
	MUX_VAL(CP(SDRC_D8), (IEN  | PTD | DIS | M0)); /*SDRC_D8*/
	MUX_VAL(CP(SDRC_D9), (IEN  | PTD | DIS | M0)); /*SDRC_D9*/
	MUX_VAL(CP(SDRC_D10), (IEN  | PTD | DIS | M0)); /*SDRC_D10*/
	MUX_VAL(CP(SDRC_D11), (IEN  | PTD | DIS | M0)); /*SDRC_D11*/
	MUX_VAL(CP(SDRC_D12), (IEN  | PTD | DIS | M0)); /*SDRC_D12*/
	MUX_VAL(CP(SDRC_D13), (IEN  | PTD | DIS | M0)); /*SDRC_D13*/
	MUX_VAL(CP(SDRC_D14), (IEN  | PTD | DIS | M0)); /*SDRC_D14*/
	MUX_VAL(CP(SDRC_D15), (IEN  | PTD | DIS | M0)); /*SDRC_D15*/
	MUX_VAL(CP(SDRC_D16), (IEN  | PTD | DIS | M0)); /*SDRC_D16*/
	MUX_VAL(CP(SDRC_D17), (IEN  | PTD | DIS | M0)); /*SDRC_D17*/
	MUX_VAL(CP(SDRC_D18), (IEN  | PTD | DIS | M0)); /*SDRC_D18*/
	MUX_VAL(CP(SDRC_D19), (IEN  | PTD | DIS | M0)); /*SDRC_D19*/
	MUX_VAL(CP(SDRC_D20), (IEN  | PTD | DIS | M0)); /*SDRC_D20*/
	MUX_VAL(CP(SDRC_D21), (IEN  | PTD | DIS | M0)); /*SDRC_D21*/
	MUX_VAL(CP(SDRC_D22), (IEN  | PTD | DIS | M0)); /*SDRC_D22*/
	MUX_VAL(CP(SDRC_D23), (IEN  | PTD | DIS | M0)); /*SDRC_D23*/
	MUX_VAL(CP(SDRC_D24), (IEN  | PTD | DIS | M0)); /*SDRC_D24*/
	MUX_VAL(CP(SDRC_D25), (IEN  | PTD | DIS | M0)); /*SDRC_D25*/
	MUX_VAL(CP(SDRC_D26), (IEN  | PTD | DIS | M0)); /*SDRC_D26*/
	MUX_VAL(CP(SDRC_D27), (IEN  | PTD | DIS | M0)); /*SDRC_D27*/
	MUX_VAL(CP(SDRC_D28), (IEN  | PTD | DIS | M0)); /*SDRC_D28*/
	MUX_VAL(CP(SDRC_D29), (IEN  | PTD | DIS | M0)); /*SDRC_D29*/
	MUX_VAL(CP(SDRC_D30), (IEN  | PTD | DIS | M0)); /*SDRC_D30*/
	MUX_VAL(CP(SDRC_D31), (IEN  | PTD | DIS | M0)); /*SDRC_D31*/
	MUX_VAL(CP(SDRC_CLK), (IEN  | PTD | DIS | M0)); /*SDRC_CLK*/
	MUX_VAL(CP(SDRC_DQS0), (IEN  | PTD | DIS | M0)); /*SDRC_DQS0*/
	MUX_VAL(CP(SDRC_DQS1), (IEN  | PTD | DIS | M0)); /*SDRC_DQS1*/
	MUX_VAL(CP(SDRC_DQS2), (IEN  | PTD | DIS | M0)); /*SDRC_DQS2*/
	MUX_VAL(CP(SDRC_DQS3), (IEN  | PTD | DIS | M0)); /*SDRC_DQS3*/
	/*GPMC*/
	MUX_VAL(CP(GPMC_A1), (IDIS | PTD | DIS | M0)); /*GPMC_A1*/
	MUX_VAL(CP(GPMC_A2), (IDIS | PTD | DIS | M0)); /*GPMC_A2*/
	MUX_VAL(CP(GPMC_A3), (IDIS | PTD | DIS | M0)); /*GPMC_A3*/
	MUX_VAL(CP(GPMC_A4), (IDIS | PTD | DIS | M0)); /*GPMC_A4*/
	MUX_VAL(CP(GPMC_A5), (IDIS | PTD | DIS | M0)); /*GPMC_A5*/
	MUX_VAL(CP(GPMC_A6), (IDIS | PTD | DIS | M0)); /*GPMC_A6*/
	MUX_VAL(CP(GPMC_A7), (IDIS | PTD | DIS | M0)); /*GPMC_A7*/
	MUX_VAL(CP(GPMC_A8), (IDIS | PTD | DIS | M0)); /*GPMC_A8*/
	MUX_VAL(CP(GPMC_A9), (IDIS | PTD | DIS | M0)); /*GPMC_A9*/
	MUX_VAL(CP(GPMC_A10), (IDIS | PTD | DIS | M0)); /*GPMC_A10*/
	MUX_VAL(CP(GPMC_D0), (IEN  | PTD | DIS | M0)); /*GPMC_D0*/
	MUX_VAL(CP(GPMC_D1), (IEN  | PTD | DIS | M0)); /*GPMC_D1*/
	MUX_VAL(CP(GPMC_D2), (IEN  | PTD | DIS | M0)); /*GPMC_D2*/
	MUX_VAL(CP(GPMC_D3), (IEN  | PTD | DIS | M0)); /*GPMC_D3*/
	MUX_VAL(CP(GPMC_D4), (IEN  | PTD | DIS | M0)); /*GPMC_D4*/
	MUX_VAL(CP(GPMC_D5), (IEN  | PTD | DIS | M0)); /*GPMC_D5*/
	MUX_VAL(CP(GPMC_D6), (IEN  | PTD | DIS | M0)); /*GPMC_D6*/
	MUX_VAL(CP(GPMC_D7), (IEN  | PTD | DIS | M0)); /*GPMC_D7*/
	MUX_VAL(CP(GPMC_D8), (IEN  | PTD | DIS | M0)); /*GPMC_D8*/
	MUX_VAL(CP(GPMC_D9), (IEN  | PTD | DIS | M0)); /*GPMC_D9*/
	MUX_VAL(CP(GPMC_D10), (IEN  | PTD | DIS | M0)); /*GPMC_D10*/
	MUX_VAL(CP(GPMC_D11), (IEN  | PTD | DIS | M0)); /*GPMC_D11*/
	MUX_VAL(CP(GPMC_D12), (IEN  | PTD | DIS | M0)); /*GPMC_D12*/
	MUX_VAL(CP(GPMC_D13), (IEN  | PTD | DIS | M0)); /*GPMC_D13*/
	MUX_VAL(CP(GPMC_D14), (IEN  | PTD | DIS | M0)); /*GPMC_D14*/
	MUX_VAL(CP(GPMC_D15), (IEN  | PTD | DIS | M0)); /*GPMC_D15*/
	MUX_VAL(CP(GPMC_NCS0), (IDIS | PTU | EN  | M0)); /*GPMC_nCS0*/
	MUX_VAL(CP(GPMC_NCS1), (IDIS | PTU | EN  | M0)); /*GPMC_nCS1*/
	MUX_VAL(CP(GPMC_NCS2), (IDIS | PTU | EN  | M0)); /*GPMC_nCS2*/
	MUX_VAL(CP(GPMC_NCS3), (IEN  | PTD | DIS | M0)); /*GPMC_nCS3*/
	MUX_VAL(CP(GPMC_NCS4), (IDIS | PTU | EN  | M7)); /*GPMC_nCS4*/
	MUX_VAL(CP(GPMC_NCS5), (IDIS | PTD | DIS | M7)); /*GPMC_nCS5*/
	MUX_VAL(CP(GPMC_NCS6), (IDIS | PTU | EN  | M0)); /*GPMC_nCS6*/
	MUX_VAL(CP(GPMC_NCS7), (IEN  | PTU | EN  | M1)); /*GPMC_IO_DIR*/
	MUX_VAL(CP(GPMC_NBE1), (IDIS | PTU | EN  | M0)); /*GPMC_nBE1*/
	MUX_VAL(CP(GPMC_WAIT0), (IEN  | PTU | EN  | M0)); /*GPMC_WAIT0*/
	MUX_VAL(CP(GPMC_WAIT1), (IEN  | PTU | EN  | M0)); /*GPMC_WAIT1*/
	MUX_VAL(CP(GPMC_WAIT2), (IEN  | PTU | EN  | M0)); /*GPMC_WAIT2*/
	MUX_VAL(CP(GPMC_WAIT3), (IEN  | PTU | EN  | M1)); /*uP_DREQ1*/
	MUX_VAL(CP(GPMC_CLK), (IEN  | PTD | DIS | M0)); /*GPMC_CLK*/
	MUX_VAL(CP(GPMC_NADV_ALE), (IDIS | PTD | DIS | M0)); /*GPMC_nADV_ALE*/
	MUX_VAL(CP(GPMC_NOE), (IDIS | PTD | DIS | M0)); /*GPMC_nOE*/
	MUX_VAL(CP(GPMC_NWE), (IDIS | PTD | DIS | M0)); /*GPMC_nWE*/
	MUX_VAL(CP(GPMC_NWP), (IEN  | PTU | EN  | M0)); /*GPMC_nWP*/
	MUX_VAL(CP(GPMC_NBE0_CLE), (IEN  | PTU | EN  | M0)); /*GPMC_nBE0_CLE*/
	/*DSS*/
	MUX_VAL(CP(DSS_PCLK), (IDIS | PTD | DIS | M0)); /*DSS_PCLK*/
	MUX_VAL(CP(DSS_HSYNC), (IDIS | PTD | DIS | M0)); /*DSS_HSYNC*/
	MUX_VAL(CP(DSS_VSYNC), (IDIS | PTD | DIS | M0)); /*DSS_VSYNC*/
	MUX_VAL(CP(DSS_ACBIAS), (IDIS | PTD | DIS | M0)); /*DSS_ACBIAS*/
	MUX_VAL(CP(DSS_DATA0), (IDIS | PTD | DIS | M0)); /*DSS_DATA0*/
	MUX_VAL(CP(DSS_DATA1), (IDIS | PTD | DIS | M0)); /*DSS_DATA1*/
	MUX_VAL(CP(DSS_DATA2), (IDIS | PTD | DIS | M0)); /*DSS_DATA2*/
	MUX_VAL(CP(DSS_DATA3), (IDIS | PTD | DIS | M0)); /*DSS_DATA3*/
	MUX_VAL(CP(DSS_DATA4), (IDIS | PTD | DIS | M0)); /*DSS_DATA4*/
	MUX_VAL(CP(DSS_DATA5), (IDIS | PTD | DIS | M0)); /*DSS_DATA5*/
	MUX_VAL(CP(DSS_DATA6), (IDIS | PTD | DIS | M0)); /*DSS_DATA6*/
	MUX_VAL(CP(DSS_DATA7), (IDIS | PTD | DIS | M0)); /*DSS_DATA7*/
	MUX_VAL(CP(DSS_DATA8), (IDIS | PTD | DIS | M0)); /*DSS_DATA8*/
	MUX_VAL(CP(DSS_DATA9), (IDIS | PTD | DIS | M0)); /*DSS_DATA9*/
	MUX_VAL(CP(DSS_DATA10), (IDIS | PTD | DIS | M0)); /*DSS_DATA10*/
	MUX_VAL(CP(DSS_DATA11), (IDIS | PTD | DIS | M0)); /*DSS_DATA11*/
	MUX_VAL(CP(DSS_DATA12), (IDIS | PTD | DIS | M0)); /*DSS_DATA12*/
	MUX_VAL(CP(DSS_DATA13), (IDIS | PTD | DIS | M0)); /*DSS_DATA13*/
	MUX_VAL(CP(DSS_DATA14), (IDIS | PTD | DIS | M0)); /*DSS_DATA14*/
	MUX_VAL(CP(DSS_DATA15), (IDIS | PTD | DIS | M0)); /*DSS_DATA15*/
	MUX_VAL(CP(DSS_DATA16), (IDIS | PTD | DIS | M0)); /*DSS_DATA16*/
	MUX_VAL(CP(DSS_DATA17), (IDIS | PTD | DIS | M0)); /*DSS_DATA17*/
	MUX_VAL(CP(DSS_DATA18), (IDIS | PTD | DIS | M0)); /*DSS_DATA18*/
	MUX_VAL(CP(DSS_DATA19), (IDIS | PTD | DIS | M0)); /*DSS_DATA19*/
	MUX_VAL(CP(DSS_DATA20), (IDIS | PTD | DIS | M0)); /*DSS_DATA20*/
	MUX_VAL(CP(DSS_DATA21), (IDIS | PTD | DIS | M0)); /*DSS_DATA21*/
	MUX_VAL(CP(DSS_DATA22), (IDIS | PTD | DIS | M0)); /*DSS_DATA22*/
	MUX_VAL(CP(DSS_DATA23), (IDIS | PTD | DIS | M0)); /*DSS_DATA23*/
	/*CAMERA*/
	MUX_VAL(CP(CAM_HS), (IEN  | PTU | EN  | M0)); /*CAM_HS */
	MUX_VAL(CP(CAM_VS), (IEN  | PTU | EN  | M0)); /*CAM_VS */
	MUX_VAL(CP(CAM_XCLKA), (IDIS | PTD | DIS | M0)); /*CAM_XCLKA*/
	MUX_VAL(CP(CAM_PCLK), (IEN  | PTU | EN  | M0)); /*CAM_PCLK*/
	MUX_VAL(CP(CAM_FLD), (IDIS | PTD | DIS | M4)); /*GPIO_98*/
	MUX_VAL(CP(CAM_D0), (IEN  | PTD | DIS | M0)); /*CAM_D0*/
	MUX_VAL(CP(CAM_D1), (IEN  | PTD | DIS | M0)); /*CAM_D1*/
	MUX_VAL(CP(CAM_D2), (IEN  | PTD | DIS | M0)); /*CAM_D2*/
	MUX_VAL(CP(CAM_D3), (IEN  | PTD | DIS | M0)); /*CAM_D3*/
	MUX_VAL(CP(CAM_D4), (IEN  | PTD | DIS | M0)); /*CAM_D4*/
	MUX_VAL(CP(CAM_D5), (IEN  | PTD | DIS | M0)); /*CAM_D5*/
	MUX_VAL(CP(CAM_D6), (IEN  | PTD | DIS | M0)); /*CAM_D6*/
	MUX_VAL(CP(CAM_D7), (IEN  | PTD | DIS | M0)); /*CAM_D7*/
	MUX_VAL(CP(CAM_D8), (IEN  | PTD | DIS | M0)); /*CAM_D8*/
	MUX_VAL(CP(CAM_D9), (IEN  | PTD | DIS | M0)); /*CAM_D9*/
	MUX_VAL(CP(CAM_D10), (IEN  | PTD | DIS | M0)); /*CAM_D10*/
	MUX_VAL(CP(CAM_D11), (IEN  | PTD | DIS | M0)); /*CAM_D11*/
	MUX_VAL(CP(CAM_XCLKB), (IDIS | PTD | DIS | M0)); /*CAM_XCLKB*/
	MUX_VAL(CP(CAM_WEN), (IEN  | PTD | DIS | M4)); /*GPIO_167*/
	MUX_VAL(CP(CAM_STROBE), (IDIS | PTD | DIS | M0)); /*CAM_STROBE*/
	MUX_VAL(CP(CSI2_DX0), (IEN  | PTD | DIS | M0)); /*CSI2_DX0*/
	MUX_VAL(CP(CSI2_DY0), (IEN  | PTD | DIS | M0)); /*CSI2_DY0*/
	MUX_VAL(CP(CSI2_DX1), (IEN  | PTD | DIS | M0)); /*CSI2_DX1*/
	MUX_VAL(CP(CSI2_DY1), (IEN  | PTD | DIS | M0)); /*CSI2_DY1*/
	/*Audio Interface */
	MUX_VAL(CP(MCBSP2_FSX), (IEN  | PTD | DIS | M0)); /*McBSP2_FSX*/
	MUX_VAL(CP(MCBSP2_CLKX), (IEN  | PTD | DIS | M0)); /*McBSP2_CLKX*/
	MUX_VAL(CP(MCBSP2_DR), (IEN  | PTD | DIS | M0)); /*McBSP2_DR*/
	MUX_VAL(CP(MCBSP2_DX), (IDIS | PTD | DIS | M0)); /*McBSP2_DX*/
	/*Expansion card */
	MUX_VAL(CP(MMC1_CLK), (IDIS | PTU | EN  | M0)); /*MMC1_CLK*/
	MUX_VAL(CP(MMC1_CMD), (IEN  | PTU | EN  | M0)); /*MMC1_CMD*/
	MUX_VAL(CP(MMC1_DAT0), (IEN  | PTU | EN  | M0)); /*MMC1_DAT0*/
	MUX_VAL(CP(MMC1_DAT1), (IEN  | PTU | EN  | M0)); /*MMC1_DAT1*/
	MUX_VAL(CP(MMC1_DAT2), (IEN  | PTU | EN  | M0)); /*MMC1_DAT2*/
	MUX_VAL(CP(MMC1_DAT3), (IEN  | PTU | EN  | M0)); /*MMC1_DAT3*/
	MUX_VAL(CP(MMC1_DAT4), (IEN  | PTU | EN  | M0)); /*MMC1_DAT4*/
	MUX_VAL(CP(MMC1_DAT5), (IEN  | PTU | EN  | M0)); /*MMC1_DAT5*/
	MUX_VAL(CP(MMC1_DAT6), (IEN  | PTU | EN  | M0)); /*MMC1_DAT6*/
	MUX_VAL(CP(MMC1_DAT7), (IEN  | PTU | EN  | M0)); /*MMC1_DAT7*/
	/*Wireless LAN */
#if 1
	/* HSUSB0_DATA1/2, not GPIO_130 */
#else
	MUX_VAL(CP(MMC2_CLK), (IEN  | PTU | EN  | M4)); /*GPIO_130*/
	MUX_VAL(CP(MMC2_CMD), (IEN  | PTU | EN  | M4)); /*GPIO_131*/
#endif
#if 1
	/* SDMMC2_DAT0-3 */
#else
	MUX_VAL(CP(MMC2_DAT0), (IEN  | PTU | EN  | M4)); /*GPIO_132*/
	MUX_VAL(CP(MMC2_DAT1), (IEN  | PTU | EN  | M4)); /*GPIO_133*/
	MUX_VAL(CP(MMC2_DAT2), (IEN  | PTU | EN  | M4)); /*GPIO_134*/
	MUX_VAL(CP(MMC2_DAT3), (IEN  | PTU | EN  | M4)); /*GPIO_135*/
#endif
#if 1
	/* SDMMC3_DAT0-3 */
#else
	MUX_VAL(CP(MMC2_DAT4), (IEN  | PTU | EN  | M4)); /*GPIO_136*/
	MUX_VAL(CP(MMC2_DAT5), (IEN  | PTU | EN  | M4)); /*GPIO_137*/
	MUX_VAL(CP(MMC2_DAT6), (IEN  | PTU | EN  | M4)); /*GPIO_138*/
	MUX_VAL(CP(MMC2_DAT7), (IEN  | PTU | EN  | M4)); /*GPIO_139*/
#endif

	/*Bluetooth*/
#if 1
	MUX_VAL(CP(MCBSP3_DX), (IDIS  | PTD | DIS | M0)); /*McBSP3_DX*/
	MUX_VAL(CP(MCBSP3_DR), (IDIS | PTD | DIS | M0)); /*McBSP3_DR*/
	MUX_VAL(CP(MCBSP3_CLKX), (IDIS | PTD | DIS | M0)); /*McBSP3_CLKX*/
	MUX_VAL(CP(MCBSP3_FSX), (IEN  | PTD | DIS | M0)); /*McBSP3_FSX*/

#else
	MUX_VAL(CP(MCBSP3_DX), (IEN  | PTD | DIS | M1)); /*UART2_CTS*/
	MUX_VAL(CP(MCBSP3_DR), (IDIS | PTD | DIS | M1)); /*UART2_RTS*/
	MUX_VAL(CP(MCBSP3_CLKX), (IDIS | PTD | DIS | M1)); /*UART2_TX*/
	MUX_VAL(CP(MCBSP3_FSX), (IEN  | PTD | DIS | M1)); /*UART2_RX*/
	MUX_VAL(CP(UART2_CTS), (IEN  | PTD | DIS | M4)); /*GPIO_144*/
	MUX_VAL(CP(UART2_RTS), (IEN  | PTD | DIS | M4)); /*GPIO_145*/
	MUX_VAL(CP(UART2_TX), (IEN  | PTD | DIS | M4)); /*GPIO_146*/
	MUX_VAL(CP(UART2_RX), (IEN  | PTD | DIS | M4)); /*GPIO_147*/
#endif
	/*Modem Interface */
#if 1
	MUX_VAL(CP(UART1_TX), (IDIS | PTD | DIS | M0)); /*UART1_TX*/
	MUX_VAL(CP(UART1_RTS), (IDIS | PTD | DIS | M0)); /*UART1_RTS*/
	MUX_VAL(CP(UART1_CTS), (IEN  | PTU | EN  | M0)); /*UART1_CTS*/
	MUX_VAL(CP(UART1_RX), (IEN  | PTD | DIS | M0)); /*UART1_RX*/
#else
	MUX_VAL(CP(UART1_TX), (IDIS | PTD | DIS | M0)); /*UART1_TX*/
	MUX_VAL(CP(UART1_RTS), (IDIS | PTD | DIS | M4)); /*GPIO_149*/ 
	MUX_VAL(CP(UART1_CTS), (IDIS | PTD | DIS | M4)); /*GPIO_150*/ 
	MUX_VAL(CP(UART1_RX), (IEN  | PTD | DIS | M0)); /*UART1_RX*/
#endif
	MUX_VAL(CP(MCBSP4_CLKX), (IEN  | PTD | DIS | M1)); /*SSI1_DAT_RX*/
	MUX_VAL(CP(MCBSP4_DR), (IEN  | PTD | DIS | M1)); /*SSI1_FLAG_RX*/
	MUX_VAL(CP(MCBSP4_DX), (IEN  | PTD | DIS | M1)); /*SSI1_RDY_RX*/
	MUX_VAL(CP(MCBSP4_FSX), (IEN  | PTD | DIS | M1)); /*SSI1_WAKE*/
	MUX_VAL(CP(MCBSP1_CLKR), (IDIS | PTD | DIS | M4)); /*GPIO_156*/
	MUX_VAL(CP(MCBSP1_FSR), (IDIS | PTU | EN  | M4)); /*GPIO_157*/
	MUX_VAL(CP(MCBSP1_DX), (IDIS | PTD | DIS | M4)); /*GPIO_158*/
	MUX_VAL(CP(MCBSP1_DR), (IDIS | PTD | DIS | M4)); /*GPIO_159*/
	MUX_VAL(CP(MCBSP_CLKS), (IEN  | PTU | DIS | M0)); /*McBSP_CLKS*/
	MUX_VAL(CP(MCBSP1_FSX), (IDIS | PTD | DIS | M4)); /*GPIO_161*/
	MUX_VAL(CP(MCBSP1_CLKX), (IDIS | PTD | DIS | M4)); /*GPIO_162*/
	/*Serial Interface*/
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN  | PTD | EN  | M0)); /*UART3_CTS_RCTX*/
	MUX_VAL(CP(UART3_RTS_SD), (IDIS | PTD | DIS | M0)); /*UART3_RTS_SD */
	MUX_VAL(CP(UART3_RX_IRRX), (IEN  | PTD | DIS | M0)); /*UART3_RX_IRRX*/
	MUX_VAL(CP(UART3_TX_IRTX), (IDIS | PTD | DIS | M0)); /*UART3_TX_IRTX*/
	MUX_VAL(CP(HSUSB0_CLK), (IEN  | PTD | DIS | M0)); /*HSUSB0_CLK*/
	MUX_VAL(CP(HSUSB0_STP), (IDIS | PTU | EN  | M0)); /*HSUSB0_STP*/
	MUX_VAL(CP(HSUSB0_DIR), (IEN  | PTD | DIS | M0)); /*HSUSB0_DIR*/
	MUX_VAL(CP(HSUSB0_NXT), (IEN  | PTD | DIS | M0)); /*HSUSB0_NXT*/
	MUX_VAL(CP(HSUSB0_DATA0), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA0*/
	MUX_VAL(CP(HSUSB0_DATA1), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA1*/
	MUX_VAL(CP(HSUSB0_DATA2), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA2*/
	MUX_VAL(CP(HSUSB0_DATA3), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA3*/
	MUX_VAL(CP(HSUSB0_DATA4), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA4*/
	MUX_VAL(CP(HSUSB0_DATA5), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA5*/
	MUX_VAL(CP(HSUSB0_DATA6), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA6*/
	MUX_VAL(CP(HSUSB0_DATA7), (IEN  | PTD | DIS | M0)); /*HSUSB0_DATA7*/
	MUX_VAL(CP(I2C1_SCL), (IEN  | PTU | EN  | M0)); /*I2C1_SCL*/
	MUX_VAL(CP(I2C1_SDA), (IEN  | PTU | EN  | M0)); /*I2C1_SDA*/
#if 1
	MUX_VAL(CP(I2C2_SCL), (IEN  | PTU | EN  | M0)); /*I2C2_SCL*/
	MUX_VAL(CP(I2C2_SDA), (IEN  | PTU | EN  | M0)); /*I2C2_SDA*/
#else
	MUX_VAL(CP(I2C2_SCL), (IEN  | PTU | EN  | M4)); /*GPIO_168*/
	MUX_VAL(CP(I2C2_SDA), (IEN  | PTU | EN  | M4)); /*GPIO_183*/
#endif
	MUX_VAL(CP(I2C3_SCL), (IEN  | PTU | EN  | M0)); /*I2C3_SCL*/
	MUX_VAL(CP(I2C3_SDA), (IEN  | PTU | EN  | M0)); /*I2C3_SDA*/
	MUX_VAL(CP(I2C4_SCL), (IEN  | PTU | EN  | M0)); /*I2C4_SCL*/
	MUX_VAL(CP(I2C4_SDA), (IEN  | PTU | EN  | M0)); /*I2C4_SDA*/
#if 1
	/* This is BATT_LINE, should it be a GPIO??? */
	MUX_VAL(CP(HDQ_SIO), (IDIS | PTU | EN  | M4)); /*GPIO_170*/
#else
	MUX_VAL(CP(HDQ_SIO), (IDIS | PTU | EN  | M4)); /*GPIO_170*/
#endif
#if 1
	MUX_VAL(CP(MCSPI1_CLK), (IEN  | PTD | DIS | M0)); /*McSPI1_CLK*/
	MUX_VAL(CP(MCSPI1_SIMO), (IEN  | PTD | DIS  | M0)); /*McSPI1_SIMO*/
	MUX_VAL(CP(MCSPI1_SOMI), (IEN  | PTD | DIS | M0)); /*McSPI1_SOMI*/
	MUX_VAL(CP(MCSPI1_CS0), (IDIS  | PTU | EN  | M0)); /*McSPI1_CS0*/
	MUX_VAL(CP(MCSPI1_CS1), (IDIS | PTU | EN  | M0)); /*McSPI1_CS1*/
	MUX_VAL(CP(MCSPI1_CS2), (IDIS | PTU | EN | M0)); /*McSPI1_CS2*/
#else
	MUX_VAL(CP(MCSPI1_CLK), (IEN  | PTU | EN  | M4)); /*GPIO_171*/
	MUX_VAL(CP(MCSPI1_SIMO), (IEN  | PTU | EN  | M4)); /*GPIO_172*/
	MUX_VAL(CP(MCSPI1_SOMI), (IEN  | PTD | DIS | M0)); /*McSPI1_SOMI*/
	MUX_VAL(CP(MCSPI1_CS0), (IEN  | PTD | EN  | M0)); /*McSPI1_CS0*/
	MUX_VAL(CP(MCSPI1_CS1), (IDIS | PTD | EN  | M0)); /*McSPI1_CS1*/
	MUX_VAL(CP(MCSPI1_CS2), (IDIS | PTD | DIS | M4)); /*GPIO_176*/
#endif
	/* USB EHCI (port 2) */
#if 1
	MUX_VAL(CP(MCSPI1_CS3), (IDIS  | PTU | EN | M0)); /*McSPI1_CS3*/
#else
	MUX_VAL(CP(MCSPI1_CS3), (IEN  | PTU | EN | M3)); /*HSUSB2_DATA2*/
#endif
	MUX_VAL(CP(MCSPI2_CLK), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA7*/
	MUX_VAL(CP(MCSPI2_SIMO), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA4*/
	MUX_VAL(CP(MCSPI2_SOMI), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA5*/
	MUX_VAL(CP(MCSPI2_CS0), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA6*/
	MUX_VAL(CP(MCSPI2_CS1), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA3*/
	MUX_VAL(CP(ETK_D10_ES2), (IDIS | PTU | DIS | M3)); /*HSUSB2_CLK*/
	MUX_VAL(CP(ETK_D11_ES2), (IDIS | PTU | DIS | M3)); /*HSUSB2_STP*/
	MUX_VAL(CP(ETK_D12_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB2_DIR*/
	MUX_VAL(CP(ETK_D13_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB2_NXT*/
	MUX_VAL(CP(ETK_D14_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA0*/
	MUX_VAL(CP(ETK_D15_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB2_DATA1*/
	/*Control and debug */
	MUX_VAL(CP(SYS_32K), (IEN  | PTD | DIS | M0)); /*SYS_32K*/
	MUX_VAL(CP(SYS_CLKREQ), (IEN  | PTD | DIS | M0)); /*SYS_CLKREQ*/
	MUX_VAL(CP(SYS_NIRQ), (IEN  | PTU | EN  | M0)); /*SYS_nIRQ*/
	MUX_VAL(CP(SYS_BOOT0), (IEN  | PTD | DIS | M4)); /*GPIO_2*/
	MUX_VAL(CP(SYS_BOOT1), (IEN  | PTD | DIS | M4)); /*GPIO_3*/
	MUX_VAL(CP(SYS_BOOT2), (IEN  | PTD | DIS | M4)); /*GPIO_4 - MMC1_WP*/
	MUX_VAL(CP(SYS_BOOT3), (IEN  | PTD | DIS | M4)); /*GPIO_5*/
	MUX_VAL(CP(SYS_BOOT4), (IEN  | PTD | DIS | M4)); /*GPIO_6*/
	MUX_VAL(CP(SYS_BOOT5), (IEN  | PTD | DIS | M4)); /*GPIO_7*/
	MUX_VAL(CP(SYS_BOOT6), (IDIS | PTD | DIS | M4)); /*GPIO_8*/ 
	MUX_VAL(CP(SYS_OFF_MODE), (IEN  | PTD | DIS | M0)); /*SYS_OFF_MODE*/
	MUX_VAL(CP(SYS_CLKOUT1), (IEN  | PTD | DIS | M0)); /*SYS_CLKOUT1*/
	MUX_VAL(CP(SYS_CLKOUT2), (IEN  | PTU | EN  | M4)); /*GPIO_186*/
	MUX_VAL(CP(ETK_CLK_ES2), (IDIS | PTU | EN  | M3)); /*HSUSB1_STP*/
	MUX_VAL(CP(ETK_CTL_ES2), (IDIS | PTU | DIS | M3)); /*HSUSB1_CLK*/
	MUX_VAL(CP(ETK_D0_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA0*/
	MUX_VAL(CP(ETK_D1_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA1*/
	MUX_VAL(CP(ETK_D2_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA2*/
	MUX_VAL(CP(ETK_D3_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA7*/
	MUX_VAL(CP(ETK_D4_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA4*/
	MUX_VAL(CP(ETK_D5_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA5*/
	MUX_VAL(CP(ETK_D6_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA6*/
	MUX_VAL(CP(ETK_D7_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DATA3*/
	MUX_VAL(CP(ETK_D8_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_DIR*/
	MUX_VAL(CP(ETK_D9_ES2), (IEN  | PTU | DIS | M3)); /*HSUSB1_NXT*/
	MUX_VAL(CP(D2D_MCAD1), (IEN  | PTD | EN  | M0)); /*d2d_mcad1*/
	MUX_VAL(CP(D2D_MCAD2), (IEN  | PTD | EN  | M0)); /*d2d_mcad2*/
	MUX_VAL(CP(D2D_MCAD3), (IEN  | PTD | EN  | M0)); /*d2d_mcad3*/
	MUX_VAL(CP(D2D_MCAD4), (IEN  | PTD | EN  | M0)); /*d2d_mcad4*/
	MUX_VAL(CP(D2D_MCAD5), (IEN  | PTD | EN  | M0)); /*d2d_mcad5*/
	MUX_VAL(CP(D2D_MCAD6), (IEN  | PTD | EN  | M0)); /*d2d_mcad6*/
	MUX_VAL(CP(D2D_MCAD7), (IEN  | PTD | EN  | M0)); /*d2d_mcad7*/
	MUX_VAL(CP(D2D_MCAD8), (IEN  | PTD | EN  | M0)); /*d2d_mcad8*/
	MUX_VAL(CP(D2D_MCAD9), (IEN  | PTD | EN  | M0)); /*d2d_mcad9*/
	MUX_VAL(CP(D2D_MCAD10), (IEN  | PTD | EN  | M0)); /*d2d_mcad10*/
	MUX_VAL(CP(D2D_MCAD11), (IEN  | PTD | EN  | M0)); /*d2d_mcad11*/
	MUX_VAL(CP(D2D_MCAD12), (IEN  | PTD | EN  | M0)); /*d2d_mcad12*/
	MUX_VAL(CP(D2D_MCAD13), (IEN  | PTD | EN  | M0)); /*d2d_mcad13*/
	MUX_VAL(CP(D2D_MCAD14), (IEN  | PTD | EN  | M0)); /*d2d_mcad14*/
	MUX_VAL(CP(D2D_MCAD15), (IEN  | PTD | EN  | M0)); /*d2d_mcad15*/
	MUX_VAL(CP(D2D_MCAD16), (IEN  | PTD | EN  | M0)); /*d2d_mcad16*/
	MUX_VAL(CP(D2D_MCAD17), (IEN  | PTD | EN  | M0)); /*d2d_mcad17*/
	MUX_VAL(CP(D2D_MCAD18), (IEN  | PTD | EN  | M0)); /*d2d_mcad18*/
	MUX_VAL(CP(D2D_MCAD19), (IEN  | PTD | EN  | M0)); /*d2d_mcad19*/
	MUX_VAL(CP(D2D_MCAD20), (IEN  | PTD | EN  | M0)); /*d2d_mcad20*/
	MUX_VAL(CP(D2D_MCAD21), (IEN  | PTD | EN  | M0)); /*d2d_mcad21*/
	MUX_VAL(CP(D2D_MCAD22), (IEN  | PTD | EN  | M0)); /*d2d_mcad22*/
	MUX_VAL(CP(D2D_MCAD23), (IEN  | PTD | EN  | M0)); /*d2d_mcad23*/
	MUX_VAL(CP(D2D_MCAD24), (IEN  | PTD | EN  | M0)); /*d2d_mcad24*/
	MUX_VAL(CP(D2D_MCAD25), (IEN  | PTD | EN  | M0)); /*d2d_mcad25*/
	MUX_VAL(CP(D2D_MCAD26), (IEN  | PTD | EN  | M0)); /*d2d_mcad26*/
	MUX_VAL(CP(D2D_MCAD27), (IEN  | PTD | EN  | M0)); /*d2d_mcad27*/
	MUX_VAL(CP(D2D_MCAD28), (IEN  | PTD | EN  | M0)); /*d2d_mcad28*/
	MUX_VAL(CP(D2D_MCAD29), (IEN  | PTD | EN  | M0)); /*d2d_mcad29*/
	MUX_VAL(CP(D2D_MCAD30), (IEN  | PTD | EN  | M0)); /*d2d_mcad30*/
	MUX_VAL(CP(D2D_MCAD31), (IEN  | PTD | EN  | M0)); /*d2d_mcad31*/
	MUX_VAL(CP(D2D_MCAD32), (IEN  | PTD | EN  | M0)); /*d2d_mcad32*/
	MUX_VAL(CP(D2D_MCAD33), (IEN  | PTD | EN  | M0)); /*d2d_mcad33*/
	MUX_VAL(CP(D2D_MCAD34), (IEN  | PTD | EN  | M0)); /*d2d_mcad34*/
	MUX_VAL(CP(D2D_MCAD35), (IEN  | PTD | EN  | M0)); /*d2d_mcad35*/
	MUX_VAL(CP(D2D_MCAD36), (IEN  | PTD | EN  | M0)); /*d2d_mcad36*/
	MUX_VAL(CP(D2D_CLK26MI), (IEN  | PTD | DIS | M0)); /*d2d_clk26mi*/
	MUX_VAL(CP(D2D_NRESPWRON), (IEN  | PTD | EN  | M0)); /*d2d_nrespwron*/
	MUX_VAL(CP(D2D_NRESWARM), (IEN  | PTU | EN  | M0)); /*d2d_nreswarm */
	MUX_VAL(CP(D2D_ARM9NIRQ), (IEN  | PTD | DIS | M0)); /*d2d_arm9nirq */
	MUX_VAL(CP(D2D_UMA2P6FIQ), (IEN  | PTD | DIS | M0)); /*d2d_uma2p6fiq*/
	MUX_VAL(CP(D2D_SPINT), (IEN  | PTD | EN  | M0)); /*d2d_spint*/
	MUX_VAL(CP(D2D_FRINT), (IEN  | PTD | EN  | M0)); /*d2d_frint*/
	MUX_VAL(CP(D2D_DMAREQ0), (IEN  | PTD | DIS | M0)); /*d2d_dmareq0*/
	MUX_VAL(CP(D2D_DMAREQ1), (IEN  | PTD | DIS | M0)); /*d2d_dmareq1*/
	MUX_VAL(CP(D2D_DMAREQ2), (IEN  | PTD | DIS | M0)); /*d2d_dmareq2*/
	MUX_VAL(CP(D2D_DMAREQ3), (IEN  | PTD | DIS | M0)); /*d2d_dmareq3*/
	MUX_VAL(CP(D2D_N3GTRST), (IEN  | PTD | DIS | M0)); /*d2d_n3gtrst*/
	MUX_VAL(CP(D2D_N3GTDI), (IEN  | PTD | DIS | M0)); /*d2d_n3gtdi*/
	MUX_VAL(CP(D2D_N3GTDO), (IEN  | PTD | DIS | M0)); /*d2d_n3gtdo*/
	MUX_VAL(CP(D2D_N3GTMS), (IEN  | PTD | DIS | M0)); /*d2d_n3gtms*/
	MUX_VAL(CP(D2D_N3GTCK), (IEN  | PTD | DIS | M0)); /*d2d_n3gtck*/
	MUX_VAL(CP(D2D_N3GRTCK), (IEN  | PTD | DIS | M0)); /*d2d_n3grtck*/
	MUX_VAL(CP(D2D_MSTDBY), (IEN  | PTU | EN  | M0)); /*d2d_mstdby*/
	MUX_VAL(CP(D2D_SWAKEUP), (IEN  | PTD | EN  | M0)); /*d2d_swakeup*/
	MUX_VAL(CP(D2D_IDLEREQ), (IEN  | PTD | DIS | M0)); /*d2d_idlereq*/
	MUX_VAL(CP(D2D_IDLEACK), (IEN  | PTU | EN  | M0)); /*d2d_idleack*/
	MUX_VAL(CP(D2D_MWRITE), (IEN  | PTD | DIS | M0)); /*d2d_mwrite*/
	MUX_VAL(CP(D2D_SWRITE), (IEN  | PTD | DIS | M0)); /*d2d_swrite*/
	MUX_VAL(CP(D2D_MREAD), (IEN  | PTD | DIS | M0)); /*d2d_mread*/
	MUX_VAL(CP(D2D_SREAD), (IEN  | PTD | DIS | M0)); /*d2d_sread*/
	MUX_VAL(CP(D2D_MBUSFLAG), (IEN  | PTD | DIS | M0)); /*d2d_mbusflag*/
	MUX_VAL(CP(D2D_SBUSFLAG), (IEN  | PTD | DIS | M0)); /*d2d_sbusflag*/
	MUX_VAL(CP(SDRC_CKE0), (IDIS | PTU | EN  | M0)); /*sdrc_cke0*/
	MUX_VAL(CP(SDRC_CKE1), (IDIS | PTU | EN  | M0)); /*sdrc_cke1*/
}



// GPMC settings for LV SOM Ethernet chip
#define LOGIC_NET_GPMC_CONFIG1  0x00001000
#define LOGIC_NET_GPMC_CONFIG2  0x00080701
#define LOGIC_NET_GPMC_CONFIG3  0x00000000
#define LOGIC_NET_GPMC_CONFIG4  0x08010702
#define LOGIC_NET_GPMC_CONFIG5  0x00080a0a
#define LOGIC_NET_GPMC_CONFIG6  0x03000280
#define LOGIC_NET_GPMC_CONFIG7  0x00000f48

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

// GPMC settings for LOGIC 1760 chip
#define LOGIC_ISP1760_GPMC_CONFIG1  0x00001200
#define LOGIC_ISP1760_GPMC_CONFIG2  0x00090901
#define LOGIC_ISP1760_GPMC_CONFIG3  0x00091001
#define LOGIC_ISP1760_GPMC_CONFIG4  0x07031002
#define LOGIC_ISP1760_GPMC_CONFIG5  0x00080c0a
#define LOGIC_ISP1760_GPMC_CONFIG6  0x08030208
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
	if (arch_number == MACH_TYPE_OMAP3_TORPEDO)
		return;

	/* If no NOR in product, then return */
	if (productID_has_NOR_flash() <= 0) {
		printf("NOR: None installed\n");
		return;
	}

	/* Check CS2 config, if its already in sync, then return */
	if (!(readl(&gpmc_cfg->cs[2].config1) & TYPE_READTYPE)) {
		puts("NOR: initialize in sync mode\n");

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
