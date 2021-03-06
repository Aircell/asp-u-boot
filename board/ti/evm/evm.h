/*
 * (C) Copyright 2008
 * Nishanth Menon <menon.nishanth@gmail.com>
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
#ifndef _EVM_H_
#define _EVM_H_

const omap3_sysinfo sysinfo = {
	DDR_DISCRETE,
	"OMAP3 EVM board",
#if defined(CONFIG_ENV_IS_IN_ONENAND)
	"OneNAND",
#else
	"NAND",
#endif
};

/*
 * OMAP35x EVM revision
 * Run time detection of EVM revision is done by reading Ethernet
 * PHY ID -
 *      GEN_1   = 0x01150000
 *      GEN_2   = 0x92200000
 */
enum {
	OMAP3EVM_BOARD_GEN_1 = 0,	/* EVM Rev between  A - D */
	OMAP3EVM_BOARD_GEN_2,		/* EVM Rev >= Rev E */
};

u8 get_omap3_evm_rev(void);

static void setup_net_chip(void);

#endif
