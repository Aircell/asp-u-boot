/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *     Manikandan Pillai <mani.pillai@ti.com>
 *
 * Initial Code from:
 *     Richard Woodruff <r-woodruff2@ti.com>
 *     Syed Mohammed Khasim <khasim@ti.com>
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
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <command.h>

/*
 * Only One NAND allowed on board at a time.
 * The GPMC CS Base for the same
 */
unsigned int boot_flash_base;
unsigned int boot_flash_off;
unsigned int boot_flash_sec;
unsigned int boot_flash_type;
volatile unsigned int boot_flash_env_addr;

struct gpmc *gpmc_cfg;

#if defined(CONFIG_CMD_NAND)
static const u32 gpmc_m_nand[GPMC_MAX_REG] = {
	M_NAND_GPMC_CONFIG1,
	M_NAND_GPMC_CONFIG2,
	M_NAND_GPMC_CONFIG3,
	M_NAND_GPMC_CONFIG4,
	M_NAND_GPMC_CONFIG5,
	M_NAND_GPMC_CONFIG6, 0
};

#if defined(CONFIG_ENV_IS_IN_NAND)
#define GPMC_CS 0
#else
#define GPMC_CS 1
#endif

#endif

#if defined(CONFIG_CMD_ONENAND)
static const u32 gpmc_onenand[GPMC_MAX_REG] = {
	ONENAND_GPMC_CONFIG1,
	ONENAND_GPMC_CONFIG2,
	ONENAND_GPMC_CONFIG3,
	ONENAND_GPMC_CONFIG4,
	ONENAND_GPMC_CONFIG5,
	ONENAND_GPMC_CONFIG6, 0
};

#if defined(CONFIG_ENV_IS_IN_ONENAND)
#define GPMC_CS 0
#else
#define GPMC_CS 1
#endif

#endif

static struct sdrc *sdrc_base = (struct sdrc *)OMAP34XX_SDRC_BASE;

/**************************************************************************
 * make_cs1_contiguous() - for es2 and above remap cs1 behind cs0 to allow
 *  command line mem=xyz use all memory with out discontinuous support
 *  compiled in.  Could do it at the ATAG, but there really is two banks...
 * Called as part of 2nd phase DDR init.
 **************************************************************************/
void make_cs1_contiguous(void)
{
	u32 size, a_add_low, a_add_high;

	size = get_sdr_cs_size(CS0);
	size >>= 25;	/* divide by 32 MiB to find size to offset CS1 */
	a_add_high = (size & 3) << 8;	/* set up low field */
	a_add_low = (size & 0x3C) >> 2;	/* set up high field */
	writel((a_add_high | a_add_low), &sdrc_base->cs_cfg);

}

/********************************************************
 *  mem_ok() - test used to see if timings are correct
 *             for a part. Helps in guessing which part
 *             we are currently using.
 *******************************************************/
u32 mem_ok(u32 cs)
{
	u32 val1, val2, addr;
	u32 pattern = 0x12345678;

	addr = OMAP34XX_SDRC_CS0 + get_sdr_cs_offset(cs);

	writel(0x0, addr + 0x400);	/* clear pos A */
	writel(pattern, addr);		/* pattern to pos B */
	writel(0x0, addr + 4);		/* remove pattern off the bus */
	val1 = readl(addr + 0x400);	/* get pos A value */
	val2 = readl(addr);		/* get val2 */

	if ((val1 != 0) || (val2 != pattern))	/* see if pos A val changed */
		return 0;
	else
		return 1;
}

void enable_gpmc_cs_config(const u32 *gpmc_config, struct gpmc_cs *cs, u32 base,
			u32 size)
{
	writel(0, &cs->config7);
	sdelay(1000);
	/* Delay for settling */
	writel(gpmc_config[0], &cs->config1);
	writel(gpmc_config[1], &cs->config2);
	writel(gpmc_config[2], &cs->config3);
	writel(gpmc_config[3], &cs->config4);
	writel(gpmc_config[4], &cs->config5);
	writel(gpmc_config[5], &cs->config6);
	/* Enable the config */
	writel((((size & 0xF) << 8) | ((base >> 24) & 0x3F) |
		(1 << 6)), &cs->config7);
	sdelay(2000);
}

/*****************************************************
 * gpmc_init(): init gpmc bus
 * Init GPMC for x16, MuxMode (SDRAM in x32).
 * This code can only be executed from SRAM or SDRAM.
 *****************************************************/
void gpmc_init(void)
{
	/* putting a blanket check on GPMC based on ZeBu for now */
	gpmc_cfg = (struct gpmc *)GPMC_BASE;
#if defined(CONFIG_CMD_NAND) || defined(CONFIG_CMD_ONENAND)
	const u32 *gpmc_config = NULL;
	u32 base = 0;
	u32 size = 0;
#if defined(CONFIG_ENV_IS_IN_NAND) || defined(CONFIG_ENV_IS_IN_ONENAND)
	u32 f_off = CONFIG_SYS_MONITOR_LEN;
	u32 f_sec = 0;
#endif
#endif
	u32 config = 0;

	/* global settings */
	writel(0, &gpmc_cfg->irqenable); /* isr's sources masked */
	writel(0, &gpmc_cfg->timeout_control);/* timeout disable */

	config = readl(&gpmc_cfg->config);
	config &= (~0xf00);
	writel(config, &gpmc_cfg->config);

	/*
	 * Disable the GPMC0 config set by ROM code
	 * It conflicts with our MPDB (both at 0x08000000)
	 */
	writel(0, &gpmc_cfg->cs[0].config7);
	sdelay(1000);

#if defined(CONFIG_CMD_NAND)	/* CS 0 */
	gpmc_config = gpmc_m_nand;

	base = PISMO1_NAND_BASE;
	size = PISMO1_NAND_SIZE;
	enable_gpmc_cs_config(gpmc_config, &gpmc_cfg->cs[0], base, size);
#if defined(CONFIG_ENV_IS_IN_NAND)
	f_off = SMNAND_ENV_OFFSET;
	f_sec = (128 << 10);	/* 128 KiB */
	/* env setup */
	boot_flash_base = base;
	boot_flash_off = f_off;
	boot_flash_sec = f_sec;
	boot_flash_env_addr = f_off;
#endif
#endif

#if defined(CONFIG_CMD_ONENAND)
	gpmc_config = gpmc_onenand;
	base = PISMO1_ONEN_BASE;
	size = PISMO1_ONEN_SIZE;
	enable_gpmc_cs_config(gpmc_config, &gpmc_cfg->cs[0], base, size);
#if defined(CONFIG_ENV_IS_IN_ONENAND)
	f_off = ONENAND_ENV_OFFSET;
	f_sec = (128 << 10);	/* 128 KiB */
	/* env setup */
	boot_flash_base = base;
	boot_flash_off = f_off;
	boot_flash_sec = f_sec;
	boot_flash_env_addr = f_off;
#endif
#endif
}

#ifdef CONFIG_CMD_GPMC_CONFIG
int do_dump_gpmc(cmd_tbl_t * cmdtp, int flag, int argc, char *const argv[])
{
	gpmc_cfg = (struct gpmc *)GPMC_BASE;
	int i;
	printf("GPMC_SYSCONFIG: %08x\n", gpmc_cfg->sysconfig);
	printf("GPMC_CONFIG: %08x\n", gpmc_cfg->config);
	for (i=0; i<8; ++i) {
		struct gpmc_cs *p = &gpmc_cfg->cs[i];
		if (p->config7 & (1<<6)) {
			printf("GPMC%d: %08x %08x %08x %08x\n", i,
				p->config1, p->config2, p->config3, p->config4);
			printf("       %08x %08x %03x\n",
				p->config5, p->config6, p->config7);
		}
	}
	return 1;
}

U_BOOT_CMD(
	dump_gpmc, 1, 1, do_dump_gpmc,
	"dump_gpmc - dump GPMC settings",
	"dump GPMC settings for valid GPMC setup"
);
#endif

#ifdef CONFIG_CMD_MUX_CONFIG
int do_dump_mux_config(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	int i,j;
	u16 val;
	struct {
		u16 start;
		u16 stop;
	} mux_offsets[] = {
		{
			CONTROL_PADCONF_SDRC_D0,	/* 0x0030 */
			CONTROL_PADCONF_SDRC_CKE1,	/* 0x0264 */
		},
		{
			CONTROL_PADCONF_SYS_32K,	/* 0x0A04 */
			CONTROL_PADCONF_D2D_SWAKEUP,	/* 0x0A4C */
		},
		{
			CONTROL_PADCONF_ETK_CLK_ES2,	/* 0x05D8 */
			CONTROL_PADCONF_ETK_D15_ES2,	/* 0x05FA */
		},
	};

	for (i=0; i<ARRAY_SIZE(mux_offsets); ++i) {
		for (j=mux_offsets[i].start; j<mux_offsets[i].stop; j+= sizeof(u16)) {
			val = readw(OMAP34XX_CTRL_BASE + j);
			if ((val & M7) != M7) {
				printf("%04x: %04x (",
					j, val);
				if (val & IEN)
					printf("IEN ");
				else
					printf("IDIS");
				if (val & PTU)
					printf(" | PTU");
				else
					printf(" | PTD");
				if (val & EN)
					printf(" | EN ");
				else
					printf(" | DIS");
				printf(" | M%d)\n", (val & M7));
			}
		}
	}
	return 0;
}

U_BOOT_CMD(mux_config, 1, 1, do_dump_mux_config,
	"mux_config - dump active mux registers",
	"dump active mux configuration"
);
#endif
