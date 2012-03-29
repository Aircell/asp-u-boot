/*
 * Functions related to SDRC.
 *
 * This file has been created after exctracting and consolidating
 * the SDRC related content from mem.c and board.c.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
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
#include <asm/arch/sys_proto.h>

extern omap3_sysinfo sysinfo;

static struct sdrc *sdrc_base = (struct sdrc *)OMAP34XX_SDRC_BASE;

/**
 * is_mem_sdr() - return 1 if mem type in use is SDR
 */
u32 is_mem_sdr(void)
{
	if (readl(&sdrc_base->cs[CS0].mr) == SDRC_MR_0_SDR)
		return 1;
	return 0;
}

/**
 * get_sdr_cs_size() - get size of chip select 0/1
 */
u32 get_sdr_cs_size(u32 cs)
{
	u32 size;

	/* get ram size field */
	size = readl(&sdrc_base->cs[cs].mcfg) >> 8;
	size &= 0x3FF;		/* remove unwanted bits */
	size <<= 21;		/* multiply by 2 MiB to find size in MB */
	return size;
}

/**
 * get_sdr_cs_offset() - get offset of cs from cs0 start
 */
u32 get_sdr_cs_offset(u32 cs)
{
	u32 offset;

	if (!cs)
		return 0;

	offset = readl(&sdrc_base->cs_cfg);
	offset = (offset & 15) << 27 | (offset & 0x30) >> 17;

	return offset;
}

/**
 * do_sdrc_init(): initialize the SDRAM for use.
 *  -Sets up SDRAM basic SDRC timings for CS0
 *  -Optimal settings can be placed here, or redone after i2c
 *      inspection of board info
 *
 *  - code called once in C-Stack only context for CS0 and a possible 2nd
 *    time depending on memory configuration from stack+global context
 */
void do_sdrc_init(u32 cs, u32 early)
{
	struct sdrc_actim *sdrc_actim_base;

	if(cs)
		sdrc_actim_base = (struct sdrc_actim *)SDRC_ACTIM_CTRL1_BASE;
	else
		sdrc_actim_base = (struct sdrc_actim *)SDRC_ACTIM_CTRL0_BASE;

	if (early) {
		/* reset sdrc controller */
		writel(SOFTRESET, &sdrc_base->sysconfig);
		wait_on_value(RESETDONE, RESETDONE, &sdrc_base->status,
			      12000000);
		writel(0, &sdrc_base->sysconfig);

		/* setup sdrc to ball mux */
		writel(SDRC_SHARING, &sdrc_base->sharing);

		/* Disable Power Down of CKE cuz of 1 CKE on combo part */
		writel(WAKEUPPROC | SRFRONRESET | PAGEPOLICY_HIGH,
				&sdrc_base->power);

		writel(ENADLL | DLLPHASE_90, &sdrc_base->dlla_ctrl);
		sdelay(0x20000);
	}

	writel(RASWIDTH_13BITS | CASWIDTH_10BITS | ADDRMUXLEGACY |
		RAMSIZE_128 | BANKALLOCATION | B32NOT16 | B32NOT16 |
		DEEPPD | DDR_SDRAM, &sdrc_base->cs[cs].mcfg);
	writel(ARCV | ARE_ARCV_1, &sdrc_base->cs[cs].rfr_ctrl);
	if (is_cpu_family(CPU_OMAP36XX)) {
		writel(V_ACTIMA_200, &sdrc_actim_base->ctrla);
		writel(V_ACTIMB_200, &sdrc_actim_base->ctrlb);
	} else {
		writel(V_ACTIMA_165, &sdrc_actim_base->ctrla);
		writel(V_ACTIMB_165, &sdrc_actim_base->ctrlb);
	}

	writel(CMD_NOP, &sdrc_base ->cs[cs].manual);
	writel(CMD_PRECHARGE, &sdrc_base->cs[cs].manual);
	writel(CMD_AUTOREFRESH, &sdrc_base->cs[cs].manual);
	writel(CMD_AUTOREFRESH, &sdrc_base->cs[cs].manual);

	/*
	 * CAS latency 3, Write Burst = Read Burst, Serial Mode,
	 * Burst length = 4
	 */
	writel(CASL3 | BURSTLENGTH4, &sdrc_base->cs[cs].mr);

	if (!mem_ok(cs))
		writel(0, &sdrc_base->cs[cs].mcfg);
}

/**
 * dram_init - Sets uboots idea of sdram size
 */
int dram_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	unsigned int size0 = 0, size1 = 0;

	size0 = get_sdr_cs_size(CS0);
	/*
	 * If a second bank of DDR is attached to CS1 this is
	 * where it can be started.  Early init code will init
	 * memory on CS0.
	 */
	if ((sysinfo.mtype == DDR_COMBO) || (sysinfo.mtype == DDR_STACKED)) {
		do_sdrc_init(CS1, NOT_EARLY);
		make_cs1_contiguous();

		size1 = get_sdr_cs_size(CS1);
	}

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = size0;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_1 + get_sdr_cs_offset(CS1);
	gd->bd->bi_dram[1].size = size1;

	return 0;
}

/**
 * mem_init() - init the sdrc chip selects CS0 and CS1
 *  - early init routines, called from flash or SRAM.
 */
void mem_init(void)
{
	/* only init up first bank here */
	do_sdrc_init(CS0, EARLY_INIT);
}

#ifdef CONFIG_CMD_SDRC_CONFIG
static void dump_sdram_config(int cs)
{
	struct sdrc_actim *sdrc_actim_base;

	if(cs)
		sdrc_actim_base = (struct sdrc_actim *)SDRC_ACTIM_CTRL1_BASE;
	else
		sdrc_actim_base = (struct sdrc_actim *)SDRC_ACTIM_CTRL0_BASE;

	printf("cs%d: mcfg  %08x mr    %08x rfr_ctrl %08x emr2 %08x\n",
		cs, readl(&sdrc_base->cs[cs].mcfg),
		readl(&sdrc_base->cs[cs].mr),
		readl(&sdrc_base->cs[cs].rfr_ctrl),
		readl(&sdrc_base->cs[cs].emr2));
	printf("cs%d: ctrla %08x ctrlb %08x\n",
		cs, 
		readl(&sdrc_actim_base->ctrla),
		readl(&sdrc_actim_base->ctrlb));
}

int do_dump_sdram_config(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	printf("sysconfig %08x sharing %08x dlla_ctrl %08x",
		readl(&sdrc_base->sysconfig),
		readl(&sdrc_base->sharing),
		readl(&sdrc_base->dlla_ctrl));
	printf(" cs_cfg %08x\n",
		readl(&sdrc_base->cs_cfg));
		
	dump_sdram_config(0);
	dump_sdram_config(1);
	return 0;
}

U_BOOT_CMD(sdram_config, 1, 1, do_dump_sdram_config,
	"sdram_config - dump SDRC registers",
	""
);
#endif
