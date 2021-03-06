/*
 * Tarr 3/2012 - Updated for the Aircell Cloudsurfer and the DM3730
 */

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
 * but WITHOUT ANY WARRANTY; without even the implied warranty of * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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
#include <asm/arch-omap3/i2c2.h>
#include <asm/mach-types.h>
#include <twl4030.h>
#include "cloudsurfer.h"
#include "cloudsurfer-gpio.h"
#include "product_id.h"

DECLARE_GLOBAL_DATA_PTR;

#define MUX_LOGIC_HSUSB0_D5_GPIO_MUX()					\
 MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M4)) /*GPIO_189*/

/* Safe mode HSUSB0_DATA5 */
#define MUX_LOGIC_HSUSB0_D5_SAFE_MODE()					\
 MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | EN  | M7)) /*HSUSB0.SAFE_MODE*/
//Aircell Hardware detect stuff
#define TWL4030_I2C_BUS			0
#define CLKGEN1_ADDR 0x49
#define MADC_ADDR 0x4a
#define ACIN2_P3_THRESHOLD 50
#define MAX_CONV_DELAY 300

static int i2c_write_byte(u8 devaddr, u8 regoffset, u8 value) {
	int i;
	for(i=0; i<10; i++) {
		if (i2c_write(devaddr, regoffset, 1, &value, 1) == 0) {
			return 0;
		} else {
			if (i < 9) {
				printf("i2c_write_byte failed, addr=0x%x, offset 0x%x: retry %d/9\n", devaddr, regoffset, i+1);
			}
		}
		udelay(50);
	}
	return -1; // failure
}

static int i2c_read_byte(u8 devaddr, u8 regoffset, u8 * value) {
	int i;
	for(i=0; i<10; i++) {
		if (i2c_read(devaddr, regoffset, 1, value, 1) == 0) {
			return 0;
		} else {
			if (i < 9) {
				printf("i2c_read_byte failed, addr=0x%x, offset 0x%x: retry %d/9\n", devaddr, regoffset, i+1);
			}
		}
		udelay(50);
	}
	return -1; // failure
}

/* Routine: detect_cloudsurfer_rev_A
 * Description: Aircell board detection (p3/rev-A)
 *
 * Hardware designers have provided a means to detect the board revision
 * /by examining the SOM pin J2-196 (ACIN2 to the TPS65950) which is a
 * BFG input on Rev 0 boards, and grounded on Rev A.
 */
static int detect_cloudsurfer_rev(void) {
	u8 msb, lsb;
	int val; // ADC counts
	int max = 10; // maximum wait cycles for ADC conversion

	//	i2c_set_bus_num(TWL4030_I2C_BUS);
	/*We assume that there is only one I2C bus */
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);	
	// Enable clocks
	i2c_write_byte(CLKGEN1_ADDR, 0x91, 0xd5);
	// power on MADC (CTRL1)
	i2c_write_byte(MADC_ADDR, 0x00, 0x01);
	// Enable CH2 for conversion: SW1SELECT_LSB, bit 2
	i2c_write_byte(MADC_ADDR, 0x06, 0x04);
	// Turn on averaging (4 reads per conversion) for CH2
	i2c_write_byte(MADC_ADDR, 0x08, 0x04);
	// Request start of conversion: CTRL_SW1, start all-channel conversion
	i2c_write_byte(MADC_ADDR, 0x12, 0x20);
	// Wait for conversion
	//   160 = 39 * 4 plus a bit. See timing table 9-3 in TPS65950 Data Manual
	//udelay(160);

	// No, really wait until it's ready
	do
	{
		udelay(50);
		i2c_read_byte(MADC_ADDR, 0x12, &lsb);
		max--;
		if ((lsb & 0x01) == 0x01) {
			printf("Waiting for board_detect ADC conversion\n");
		}
	}
	while((lsb & 0x01) == 0x01 && max > 0); // bit goes low when conversion is complete

	// Read the voltage
	i2c_read_byte(MADC_ADDR, 0x3b, &lsb);
	printf(" %s:lsb is:0x%x \n",__FUNCTION__,lsb);
	i2c_read_byte(MADC_ADDR, 0x3c, &msb);
	printf(" %s:msb is:0x%x \n",__FUNCTION__,msb);
	val = (((int)lsb & 0xc0) >> 6) | ((int)msb << 2);
	printf("detect_cloudsurfer_rev_A: ACIN2 on TPS65950 reads %d\n", val);

	// Power off MADC
	i2c_write_byte(MADC_ADDR, 0x00, 0x00);

	if (val < ACIN2_P3_THRESHOLD) {
		// Pin is grounded on RevA
		return 1;
	} else {
		// Logic-high on P3
		return 0;
	}
}


/*
 * Routine: logic_identify
 * Description: Detect if we are running on a Logic or Torpedo.
 *              This can be done by GPIO_189. If its low after driving it high,
 *              then its a SOM LV, else Torpedo.
 */
#define ID_CHECK_GPIO	189

unsigned int logic_identify(void)
{
	unsigned int val = 0;
	int i;

	MUX_LOGIC_HSUSB0_D5_GPIO_MUX();

	if (!omap_request_gpio(ID_CHECK_GPIO)) {

		omap_set_gpio_direction(ID_CHECK_GPIO, 0);
		omap_set_gpio_dataout(ID_CHECK_GPIO, 1);

		/* Let it soak for a bit */
		for (i=0; i<0x100; ++i)
			asm("nop");

		omap_set_gpio_direction(ID_CHECK_GPIO, 1);
		val = omap_get_gpio_datain(ID_CHECK_GPIO);
		omap_free_gpio(ID_CHECK_GPIO);

		if ( val ) {
			printf("Wrong SOM install!\n\n!");
			val = 0;
		}	
		//val = MACH_TYPE_DM3730_SOM_LV;
		printf(" AKA Cloudsurfer\n");
		if (detect_cloudsurfer_rev())
		{
			printf(" (Rev B)\n");
			val = MACH_TYPE_CLOUDSURFER_REVA;
		}
		else
		{
			printf(" (P3 = REV A)\n");
			val = MACH_TYPE_CLOUDSURFER_P3;
		}
	}

	MUX_LOGIC_HSUSB0_D5_SAFE_MODE();

	return val;
}

static void setup_nand_settings(void);
static void fix_flash_sync(void);
static void setup_isp1760_chip(void);
static void setup_cf_gpmc(void);
void init_vaux1_voltage(void);

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */

	/* Update NAND settings */
	setup_nand_settings();

	/* Setup the GPIO pins Cloudsurfer - Doing it here to shut
     * some stuff off by default */
	cloudsurfer_gpios();

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

#define CHARGER_ADDR			0x41
#define BQ2750_ADDR				0x55
#define MINIMUM_VOLTAGE			3400	/* in mV */
#define PCA9626B_ADDR			0x70
#define LEDOUT0					0x1d
#define PWM0_NO_AUTOINCREMENT	0x02


static void turn_on_keypad_leds(void)
{//PCA9626B
	unsigned char data[32];
	i2c2_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	data[0] = 0x01;
	i2c2_write(PCA9626B_ADDR, 0x00, 1, &data[0], 1);	// Bring LED driver chip out of reset
	udelay(600);	//	delay min 500us

	data[0] = 0xaa;
	i2c2_write(PCA9626B_ADDR, LEDOUT0 + 0, 1, &data[0], 1);
	i2c2_write(PCA9626B_ADDR, LEDOUT0 + 1, 1, &data[0], 1);
	i2c2_write(PCA9626B_ADDR, LEDOUT0 + 2, 1, &data[0], 1);
	i2c2_write(PCA9626B_ADDR, LEDOUT0 + 3, 1, &data[0], 1);
	i2c2_write(PCA9626B_ADDR, LEDOUT0 + 4, 1, &data[0], 1);
	i2c2_write(PCA9626B_ADDR, LEDOUT0 + 5, 1, &data[0], 1);

	data[0] = 40;
	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  0, 1, &data[0], 1);	// Write brightness to zone-5
	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  1, 1, &data[0], 1);	// Write brightness to zone-8
	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  2, 1, &data[0], 1);	// Write brightness to zone-2
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  3, 1, &data[0], 1);	// Write brightness to zone-6
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  4, 1, &data[0], 1);	// Write brightness to zone-3
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  5, 1, &data[0], 1);	// Write brightness to zone-menu
	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  6, 1, &data[0], 1);	// Write brightness to zone-home
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 11, 1, &data[0], 1);	// Write brightness to zone-red
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 12, 1, &data[0], 1);	// Write brightness to zone-back
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 13, 1, &data[0], 1);	// Write brightness to zone-9
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 14, 1, &data[0], 1);	// Write brightness to zone-pound
	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 15, 1, &data[0], 1);	// Write brightness to zone-0
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 16, 1, &data[0], 1);	// Write brightness to zone-astrix
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 17, 1, &data[0], 1);	// Write brightness to zone-7
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 18, 1, &data[0], 1);	// Write brightness to zone-4
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 19, 1, &data[0], 1);	// Write brightness to zone-1
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 20, 1, &data[0], 1);	// Write brightness to zone-S1
	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 21, 1, &data[0], 1);	// Write brightness to zone-S2
//	i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT + 22, 1, &data[0], 1);	// Write brightness to zone-S3
}

/* Check the current battery voltage via the fuel guage
 * Return 1 if OK, otherwise 0 
 */
unsigned short check_voltage(void)
{
	unsigned short sdata;
	
	i2c2_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	if ( i2c2_read(BQ2750_ADDR,0x08,1,(unsigned char*)&sdata,2) )
		return 0;
	printf("Current Voltage is %d\n",sdata);
	return sdata;

}

int enable_fast_charge(int enable) 
{
	unsigned char data;
	data = 0;
	i2c2_write(CHARGER_ADDR,0x03,1,&data,1);
	if ( enable ){
		printf("Enabling");
		data = 0x02;
	} else {
		printf("Disabling");
		data = 0x03;
	}
	printf(" Fast Charging\n");
	i2c2_write(CHARGER_ADDR,0x01,1,&data,1);
	return 1;
}
/* Determine if we have enough umph to do anything useful */
void check_energy(void) 
{	
	int charging = 0; 
	unsigned short voltage;
	int cradled = 0;	
	unsigned char data;
	unsigned int count = 0;

	if ( (cradled = !omap_get_gpio_datain(AIRCELL_POWER_APPLIED_DETECT))) {
		printf("In Cradle\n");
	} else {
		printf("Not in Cradle\n");
	}
	voltage = check_voltage();
	if ( cradled ) {
		if ( voltage < MINIMUM_VOLTAGE )  {
			if ( !(charging = enable_fast_charge(1)) ) {
				printf("Battery low and can't charge - Loop forever\n");
				while ( 1 ) ;
			}
			/* Loop till we get energy to do stuff */
			while ( check_voltage() < MINIMUM_VOLTAGE ) {
				udelay(1000000);
				count++;
				if ( count & 0x0001 ) 
					data = 0;
				else 
					data = 40; 	
				i2c2_write(PCA9626B_ADDR, PWM0_NO_AUTOINCREMENT +  6, 1, &data, 1);	// Write brightness to zone-home
			}
		 	enable_fast_charge(0);
		}
	} else {
		if ( voltage < MINIMUM_VOLTAGE )  {
			printf("He's dead, Jim!\n");
			while ( 1 ) ;
		}
	}
}
/*
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 */
int misc_init_r(void)
{
	printf("Cloudsurfer P3\n");
	
	turn_on_keypad_leds();

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	/* Turn on vaux1 to make sure voltage is to the product ID chip.
	 * Extract production data from ID chip, used to selectively
	 * initialize portions of the system */
	init_vaux1_voltage();
	fetch_production_data();
	if ( is_dm3730_som() == 0 ) {
		printf("This is not a supported SOM!\n");
		while ( 1 ) ;
	}

	/* If battery powered, check to make sure we have sufficient energy
     * to startup...
	 */
 //Currently We don't need this feature with Aircell phone.
 // This prevents device to boot with low voltage. Enable it if required.
#if 0
    if ( omap_get_gpio_datain(AIRCELL_BATTERY_POWERED) != 0 ) {
		check_energy();
	}	
#endif
	setup_net_chip();

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

	if (!omap_request_gpio(AIRCELL_WAKE_ON_LAN)) {
		omap_set_gpio_direction(AIRCELL_WAKE_ON_LAN, 1);
	} else {
		printf("Could not request GPIO 10\n");
	}

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
	/* Fetch the ethaddr of the WiFi */
	board_get_nth_enetaddr(enetaddr, 1, 1);

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

	puts("Cloudsurfer uses ");
	/* Determine if the is a corded phone */
	if ( omap_get_gpio_datain(AIRCELL_BATTERY_POWERED) != 0 ) {
		/* power off the ethernet chip */
		puts("WiFi\n");
		rc = smc911x_powerdown(CONFIG_SMC911X_BASE);
	} else {
		puts("Ethernet\n");
		rc = smc911x_initialize(0, CONFIG_SMC911X_BASE);
	}
	return rc;
}
