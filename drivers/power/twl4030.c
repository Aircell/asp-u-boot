/*
 * Copyright (c) 2009 Wind River Systems, Inc.
 * Tom Rix <Tom.Rix at windriver.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * twl4030_power_reset_init is derived from code on omapzoom,
 * git://git.omapzoom.com/repo/u-boot.git
 *
 * Copyright (C) 2007-2009 Texas Instruments, Inc.
 *
 * twl4030_power_init is from cpu/omap3/common.c, power_init_r
 *
 * (C) Copyright 2004-2008
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Sunil Kumar <sunilsaini05 at gmail.com>
 *	Shashi Ranjan <shashiranjanmca05 at gmail.com>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2 at ti.com>
 *	Syed Mohammed Khasim <khasim at ti.com>
 *
 */

#include <twl4030.h>

/*
 * Power Reset
 */
void twl4030_power_reset_init(void)
{
	u8 val = 0;
	if (twl4030_i2c_read_u8(TWL4030_CHIP_PM_MASTER, &val,
				TWL4030_PM_MASTER_P1_SW_EVENTS)) {
		printf("Error:TWL4030: failed to read the power register\n");
		printf("Could not initialize hardware reset\n");
	} else {
		val |= TWL4030_PM_MASTER_SW_EVENTS_STOPON_PWRON;
		if (twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER, val,
					 TWL4030_PM_MASTER_P1_SW_EVENTS)) {
			printf("Error:TWL4030: failed to write the power register\n");
			printf("Could not initialize hardware reset\n");
		}
	}
}


/*
 * Power Init
 */
#define DEV_GRP_P1		0x20
#define VAUX3_VSEL_28		0x03
#define DEV_GRP_ALL		0xE0
#define VPLL2_VSEL_18		0x05
#define VDAC_VSEL_18		0x03

void twl4030_power_init(void)
{
	unsigned char byte;

	/* set VAUX3 to 2.8V */
	byte = DEV_GRP_P1;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VAUX3_DEV_GRP);
	byte = VAUX3_VSEL_28;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VAUX3_DEDICATED);

	/* set VPLL2 to 1.8V */
	byte = DEV_GRP_ALL;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VPLL2_DEV_GRP);
	byte = VPLL2_VSEL_18;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VPLL2_DEDICATED);

	/* set VDAC to 1.8V */
	byte = DEV_GRP_P1;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VDAC_DEV_GRP);
	byte = VDAC_VSEL_18;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VDAC_DEDICATED);
}

#define VMMC1_VSEL_30		0x02

void twl4030_power_mmc_init(void)
{
	unsigned char byte;

	byte = DEV_GRP_P1;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VMMC1_DEV_GRP);

	/* 3 Volts */
	byte = VMMC1_VSEL_30;
	twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte,
			     TWL4030_PM_RECEIVER_VMMC1_DEDICATED);
}

int twl4030_enable_charging(void)
{
	u8 val = 0;
	/* write 0x57 to 0x4a, 0x85 (BCIMFKEY)*/
	/* Enabel access to BCIMFEN1 register access */
	if (twl4030_i2c_write_u8(TWL4030_CHIP_MAIN_CHARGE, 0x57,
					TWL4030_MAIN_CHARGE_BCIMFKEY)) {
		printf("Error:TWL4030: failed to write BCIMFKEY\n");
		return 1;
	}

	/* read 0x4a, 0x86 */
	if (twl4030_i2c_read_u8(TWL4030_CHIP_MAIN_CHARGE, &val,
					TWL4030_MAIN_CHARGE_BCIMFEN1)) {
		printf("Error:TWL4030: failed to read BCIMFEN1\n");
		return 1;
	}

	/* or in 0x80 */
	/* write to 0x4a, 0x86 (TWL_BCIMFEN1) */
	val |= TWL4030_MAIN_CHARGE_BCIMFEN1_VBATOV1EN;
	if (twl4030_i2c_write_u8(TWL4030_CHIP_MAIN_CHARGE, val,
					TWL4030_MAIN_CHARGE_BCIMFEN1)) {
		printf("Error:TWL4030: failed to write BCIMFEN1\n");
		return 1;
	}

	/* write 0xd2 to 0x4a, 0x85 (BCIMFKEY) */
	val = TWL4030_MAIN_CHARGE_BCIMFKEY_MFKEY5;
	if (twl4030_i2c_write_u8(TWL4030_CHIP_MAIN_CHARGE,
					val, 
					TWL4030_MAIN_CHARGE_BCIMFKEY)) {
		printf("Error:TWL4030: failed to write BCIMFKEY\n");
		return 1;
	}

	/* clear low four bits in 0x4a, 0x8a (BCIMFTH1) */
	if (twl4030_i2c_read_u8(TWL4030_CHIP_MAIN_CHARGE, &val,
					TWL4030_MAIN_CHARGE_BCIMFTH1)) {
		printf("Error:TWL4030: failed to read BCIMFTH1\n");
		return 1;
	}
	val = (val & ~TWL4030_MAIN_CHARGE_BCIMFTH1_VBATOV1TH_MASK) |
		TWL4030_MAIN_CHARGE_BCIMFTH1_VBATOV1TH_2636_mV;
	/* clear low four bits in 0x4a, 0x8a (BCIMFTH1) */
	if (twl4030_i2c_write_u8(TWL4030_CHIP_MAIN_CHARGE, val,
					TWL4030_MAIN_CHARGE_BCIMFTH1)) {
		printf("Error:TWL4030: failed to write BCIMFTH1\n");
		return 1;
	}

	/* read 0x4a, 0x86; 0x4a, 0x8a (???) */

	/* Turn on AC charging */
	/* write 0x90 to 0x49, 0x91 (MADC_HFCLK_EN, DEFAULTMADC_CLK_EN) */
	val = TWL4030_INTBR_GPBR1_MADC_HFCLK_EN;
	val |= TWL4030_INTBR_GPBR1_DEFAULT_MADC_CLK_EN;
	if (twl4030_i2c_write_u8(TWL4030_CHIP_INTBR, val,
					TWL4030_INTBR_GPBR1)) {
		printf("Error:TWL4030: failed to write BCIMFTH1\n");
		return 1;
	}
	return 0;
}


struct {
	u8 idx;
	u8 reg_base;
	char *str;
	u16 mul;
} adc_regs[] = {
	{
		0,
		0x37,
		"(GP analog input/Bat type)",
		1466
	},
	{
		1,
		0x39,
		"(GP analog input/Bat temp)",
		1446
	},
	{
		2,
		0x3b,
		"(GP analog input)\t",
		2444,
	},
	{
		3,
		0x3d,
		"(GP analog input)\t",
		2444,
	},
	{
		4,
		0x3f,
		"(GP analog input)\t",
		2444,
	},
	{
		5,
		0x41,
		"(GP analog input)\t",
		2444,
	},
	{
		6,
		0x43,
		"(GP analog input)\t",
		2444,
	},
	{
		7,
		0x45,
		"(GP analog input)\t",
		2444,
	},
	{
		8,
		0x47,
		"(VBUS voltage)\t",
		6843,
	},
	{
		9,
		0x49,
		"(Charger backup bat volt)",
		4399,
	},
	{
		11,
		0x4d,
		"(Battery charger voltage)",
		9775,
	},
	{
		12,
		0x4f,
		"(Main battery voltage)",
		5865,
	},
	{
		15,
		0x55,
		"(VRUSB supply/spkr pol)",
		3225,
	},
};

struct 
{
	u8 val;
	char *str;
	u8 pr_info;
} charge_state[] = {
	{
		0x00,
		"No charging device",
		0,
	},
	{
		0x01,
		"Off mode",
		0,
	},
	{
		0x02,
		"Standby mode",
		0,
	},
	{
		0x03,
		"Open bat or USB not debounced",
		0,
	},
	{
		0x21,
		"Constant voltage AC",
		1,
	},
	{
		0x22,
		"Quick charge AC 1",
		1,
	},
	{
		0x23,
		"Quick charge AC 2",
		1,
	},
	{
		0x24,
		"Quick charge AC 3",
		1,
	},
	{
		0x25,
		"Quick charge AC 4",
		1,
	},
	{
		0x26,
		"Quick charge AC 5",
		1,
	},
	{
		0x27,
		"Quick charge AC 6",
		1,
	},
	{
		0x28,
		"Charge stop AC 1",
		1,
	},
	{
		0x29,
		"Charge stop AC 1",
		1,
	},
	{
		0x2a,
		"Charge stop AC 1",
		1,
	},
	{
		0x2b,
		"Charge AC comp 1",
		1,
	},
	{
		0x2c,
		"Charge AC comp 2",
		1,
	},
	{
		0x2d,
		"Charge AC comp 3",
		1,
	},
	{
		0x2e,
		"Charge AC comp 4",
		1,
	},
	{
		0x2f,
		"AC adapter overvoltage",
		0,
	},
	{
		0x12,
		"Quick charge USB 1",
		1,
	},
	{
		0x13,
		"Quick charge USB 2",
		1,
	},
	{
		0x14,
		"Quick charge USB 3",
		1,
	},
	{
		0x15,
		"Quick charge USB 4",
		1,
	},
	{
		0x16,
		"Quick charge USB 5",
		1,
	},
	{
		0x17,
		"Quick charge USB 6",
		1,
	},
	{
		0x18,
		"Charge stop USB 1",
		1,
	},
	{
		0x19,
		"Charge stop USB 2",
		1,
	},
	{
		0x1a,
		"Charge stop USB 3",
		1,
	},
	{
		0x1b,
		"Charge USB comp 1",
		1,
	},
	{
		0x1c,
		"Charge USB comp 2",
		1,
	},
	{
		0x1d,
		"Charge USB comp 3",
		1,
	},
	{
		0x1e,
		"Charge USB comp 4",
		1,
	},
	{
		0x1f,
		"USB adapter overvoltage",
		0,
	},
};

int read_madc_msblsb(u8 reg)
{
	u8 val;
	int temp;

	twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &val, reg+1);
	temp = ((int)(val & 0x3)) << 8;
	twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &val, reg);
	return temp | val;
}

#define VOLT_STEP_SIZE	588
#define VOLT_PSR_R	100
	
int pm_battery_voltage(void)
{
	int volt = read_madc_msblsb(0x78);
	return (volt * VOLT_STEP_SIZE) / VOLT_PSR_R;
}

#define CURR_SLOPE	1194

int pm_battery_current(void)
{
	int curr = read_madc_msblsb(0x7c);
	u8 val;

	twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &val, 0x98);
	if (val & 0x20)
		return (curr * 1000) / (2 * CURR_SLOPE);
	else
		return (curr * 1000) / (1 * CURR_SLOPE);
}

static int batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,  9280,  8950,  8620,  8310,
8020,  7730,  7460,  7200,  6950,  6710,  6470,  6250,  6040,  5830,
5640,  5450,  5260,  5090,  4920,  4760,  4600,  4450,  4310,  4170,
4040,  3910,  3790,  3670,  3550
};

#define TEMP_STEP_SIZE	147
#define TEMP_PSR_R	100

int pm_battery_temp(void)
{
	u8 val;
	int temp, curr, volt, res, ret;

	ret = read_madc_msblsb(0x7a);

	volt = (ret * TEMP_STEP_SIZE) / TEMP_PSR_R;

	twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &val, 0x98);
	curr = ((val & 0x07) + 1) * 10;
	res = volt * 1000 / curr;
	for (temp = 58; temp >= 0; temp--) {
		int actual = batt_table[temp];
		if ((actual - res) >= 0)
			break;
	}

	if (temp < 3) {
		if (temp == 2)
			temp = -1;
		else if (temp == 1)
			temp = -2;
		else
			temp = -3;
	}
	return temp + 1;
}

void print_charge_info(void)
{
	printf("\n");

	printf("Charger main battery voltage\t\t      %4u mV\n", pm_battery_voltage());

	printf("Charger current\t\t\t\t      %4u mA\n", pm_battery_current());

	printf("\n");

	printf("Battery temperature (if using Logic battery)\t%2u deg C\n", pm_battery_temp());

}
		
int twl4030_info(void)
{
	u8 val;
	int i;
	
	twl4030_i2c_write_u8(TWL4030_CHIP_INTBR, 0x90, TWL4030_INTBR_GPBR1);

	twl4030_i2c_write_u8(TWL4030_CHIP_MADC, 0x01, 0x00);

	twl4030_i2c_write_u8(TWL4030_CHIP_MADC, 0xff, 0x06);

	twl4030_i2c_write_u8(TWL4030_CHIP_MADC, 0xff, 0x07);

	twl4030_i2c_write_u8(TWL4030_CHIP_MADC, 0x02, 0x97);

	twl4030_i2c_write_u8(TWL4030_CHIP_MADC, 0x20, 0x12);

	udelay(200000);

	for (i=0; i<sizeof(adc_regs)/sizeof(adc_regs[0]); ++i) {
		u8 lsb, msb;
		u32 adc, v;
		
		twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &lsb, adc_regs[i].reg_base);
		twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &msb, adc_regs[i].reg_base+1);
		adc = lsb / 64 + msb * 4;
		v = (adc * adc_regs[i].mul) / 1000;
		printf("ADC%d %s\tvalue 0x%03x = %4u mV\n", adc_regs[i].idx, adc_regs[i].str, adc, v);
	}

	twl4030_i2c_read_u8(TWL4030_CHIP_MADC, &val, 0x76);
	
	for (i=0; i<sizeof(charge_state)/sizeof(charge_state[0]);++i) {
		if (val == charge_state[i].val) {
			printf("Charge state \t\t\tvalue 0x%02x  = %s\n", val, charge_state[i].str);
			if (charge_state[i].pr_info) {
				print_charge_info();
			}
			break;
		}
	}
	if (i >= sizeof(charge_state)/sizeof(charge_state[0])) {
			printf("Charge state \t\t\tvalue 0x%02x  = %s\n", val, "Unknown charge state");
	}
	return 0;
}


int adc_cmd(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	char *cmd;

	if (argc < 2)
		goto usage;

	cmd = argv[1];
	if (strcmp(cmd, "enable") == 0)
		return twl4030_enable_charging();
	if (strcmp(cmd, "info") == 0)
		return twl4030_info();
usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	madc, 2, 1, adc_cmd,
	"MADC subsytem",
	"madc enable - enable charging\n"
	"madc info - print information on ADC registers/charging state"
);
