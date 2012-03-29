/*
 * am3517evm.c - board file for TI's AM3517 family of devices.
 *
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Based on ti/evm/evm.c
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
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

#include <common.h>
#include <netdev.h>
#include <net.h>
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/emac_defs.h>
#include <asm/arch/gpio.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include "am3517evm.h"


#define AM3517_IP_SW_RESET	0x48002598
#define CPGMACSS_SW_RST		(1 << 1)
#define CONTROL_EFUSE_EMAC_LSB  0x48002380
#define CONTROL_EFUSE_EMAC_MSB  0x48002384



/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3517EVM;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

/*
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 */
int misc_init_r(void)
{
	volatile unsigned int ctr;
	u32 reset;


#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

	dieid_num_r();


#if defined(CONFIG_DRIVER_TI_EMAC)

	omap_request_gpio(30);
	omap_set_gpio_direction(30,0);
	omap_set_gpio_dataout(30,0);
	ctr  = 0;
	do{
		udelay(1000);
		ctr++;
		}while (ctr <300);
	omap_set_gpio_dataout(30,1);
	ctr =0;
	/* allow the PHY to stabilize and settle down */
	do{
		udelay(1000);
		ctr++;
		}while (ctr <300);

	/*ensure that the module is out of reset*/
	reset = readl(AM3517_IP_SW_RESET);
	reset &= (~CPGMACSS_SW_RST);
	writel(reset,AM3517_IP_SW_RESET);

#endif


	return 0;
}


/*
 * Initializes on-chip ethernet controllers.
 * to override, implement board_eth_init()
 */
int cpu_eth_init(bd_t *bis)
{
#if defined(CONFIG_DRIVER_TI_EMAC)
extern void davinci_eth_set_mac_addr(const u_int8_t *addr);

	u_int8_t mac_addr[6];
	u_int32_t mac_hi, mac_lo;

	printf("davinci_emac_initialize\n");

	if (!eth_getenv_enetaddr("ethaddr", mac_addr)) {
		printf("<ethaddr> not set. Reading from e-fuse\n");
		mac_lo = __raw_readl(CONTROL_EFUSE_EMAC_LSB);
		mac_hi = __raw_readl(CONTROL_EFUSE_EMAC_MSB);
		mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
		mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
		mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
		mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
		mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
		mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);
		/*set the ethaddr variable with the detected Addr */
		eth_setenv_enetaddr("ethaddr", mac_addr);
	}

	if (is_valid_ether_addr(mac_addr)) {
		printf("Detected MACID:%x:%x:%x:%x:%x:%x\n", mac_addr[0],
			mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
			mac_addr[5]);
		davinci_eth_set_mac_addr(mac_addr);
	} else {
		printf("caution: no valid MACID!! Set <ethaddr> variable \n");
	}

	davinci_emac_initialize();
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
	MUX_AM3517EVM();
}
