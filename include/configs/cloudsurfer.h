/* Tarr- 3/2012
 *
 * Used the LogicPD DM3730 config file as a starting point
 */

/*
 * (C) Copyright 2006-2008
 * Texas Instruments.
 * Author :
 *	Manikandan Pillai <mani.pillai@ti.com>
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 * Manikandan Pillai <mani.pillai@ti.com>
 *
 * Configuration settings for the TI OMAP3 EVM board.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

// #undef DEBUG		/* disabe debug() output */

/*
 * High Level Configuration Options
 */
#define CONFIG_ARMCORTEXA8	1	/* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1	/* in a TI OMAP core */
#define CONFIG_OMAP34XX		1	/* which is a 34XX */
#define CONFIG_OMAP3430		1	/* which is in a 3430 */
#define CONFIG_DM3730LOGIC      1	/* working with DM3730LOGIC board */

#define CONFIG_SDRC	/* The chip has SDRC controller */

#include <asm/arch/cpu.h>	/* get chip and board defs */
#include <asm/arch/omap3.h>

/*
 * Display CPU and Board information
 */
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_DISPLAY_BOARDINFO	1

/* Clock Defines */
#define V_OSCK			26000000	/* Clock output from T2 */
#define V_SCLK			(V_OSCK >> 1)

#undef CONFIG_USE_IRQ			/* no support for IRQs */
#define CONFIG_MISC_INIT_R

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_REVISION_TAG		1

#define CONFIG_CMDLINE_EDITING		1	/* cmd line edit/history */
#define CONFIG_ZERO_BOOTDELAY_CHECK	1	/* Check if keypress with zero bootdelay */
/*
 * Size of malloc() pool
 */
#define CONFIG_ENV_SIZE			(128 << 10)	/* 128 KiB */
						/* Sector */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (128 << 10))
#define CONFIG_SYS_GBL_DATA_SIZE	128	/* bytes reserved for */
						/* initial data */
/*
 * Hardware drivers
 */

/*
 * NS16550 Configuration
 */
#define V_NS16550_CLK			48000000	/* 48MHz (APLL96/2) */

#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		V_NS16550_CLK

/*
 * select serial console configuration
 */
#define CONFIG_CONS_INDEX		1
#define CONFIG_SYS_NS16550_COM1		OMAP34XX_UART1
#define CONFIG_SERIAL1			1	/* UART1 on OMAP3 EVM */

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
					115200}
#define CONFIG_MMC			1
#define CONFIG_OMAP3_MMC		1
#define CONFIG_DOS_PARTITION		1

/* DDR - I use Micron DDR */
#define CONFIG_OMAP3_MICRON_DDR		1

/* USB
 * Enable CONFIG_MUSB_HCD for Host functionalities MSC, keyboard
 * Enable CONFIG_MUSB_UDD for Device functionalities.
 */
#define CONFIG_USB_OMAP3		1
#define CONFIG_MUSB_HCD			1
/* #define CONFIG_MUSB_UDC		1 */

#ifdef CONFIG_USB_OMAP3

#ifdef CONFIG_MUSB_HCD
#define CONFIG_CMD_USB

#define CONFIG_USB_STORAGE
#define CONGIG_CMD_STORAGE
#define CONFIG_CMD_FAT
#define CONFIG_CMD_MTDPARTS    /* Enable MTD parts commands */
#define CONFIG_MTD_DEVICE      /* needed for mtdparts commands */
#define MTDIDS_DEFAULT                 "nand0=nand"
#define MTDPARTS_DEFAULT               "mtdparts=nand:512k(x-loader),"\
                                       "1920k(u-boot),128k(u-boot-env),"\
                                       "4m(kernel),160m(system),"\
                                       "84m(cache),4m(recovery),"\
                                       "-(userdata)"

#ifdef CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_PREBOOT "usb start"
#endif /* CONFIG_USB_KEYBOARD */

#endif /* CONFIG_MUSB_HCD */

#ifdef CONFIG_MUSB_UDC
/* USB device configuration */
#define CONFIG_USB_DEVICE		1
#define CONFIG_USB_TTY			1
#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1
/* Change these to suit your needs */
#define CONFIG_USBD_VENDORID		0x0451
#define CONFIG_USBD_PRODUCTID		0x5678
#define CONFIG_USBD_MANUFACTURER	"Texas Instruments"
#define CONFIG_USBD_PRODUCT_NAME	"LogicPD 3730 SOM"
#endif /* CONFIG_MUSB_UDC */

#endif /* CONFIG_USB_OMAP3 */

/* commands to include */
#include <config_cmd_default.h>

#define CONFIG_CMD_EXT2		/* EXT2 Support			*/
#define CONFIG_CMD_FAT		/* FAT support			*/
#define CONFIG_CMD_JFFS2	/* JFFS2 Support		*/

#define CONFIG_CMD_I2C		/* I2C serial bus support	*/
#define CONFIG_CMD_MMC		/* MMC support			*/
#define CONFIG_CMD_NAND		/* NAND support			*/
#define CONFIG_CMD_NAND_YAFFS	/* Add "nand write.yaffs" command */
#define CONFIG_CMD_NAND_LOCK_UNLOCK	/* nand (un)lock commands	*/
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PING

#define BOARD_LATE_INIT

#undef CONFIG_CMD_FLASH		/* flinfo, erase, protect	*/
#undef CONFIG_CMD_FPGA		/* FPGA configuration Support	*/
#undef CONFIG_CMD_IMI		/* iminfo			*/
#undef CONFIG_CMD_IMLS		/* List all found images	*/

#define CONFIG_CMD_ASKENV	/* askenv support */

#define CONFIG_CMD_GPMC_CONFIG	/* gpmc_config */
#define CONFIG_CMD_MUX_CONFIG	/* mux_config */
#define CONFIG_CMD_SDRC_CONFIG	/* mux_config */

#define CONFIG_SYS_NO_FLASH
#define CONFIG_HARD_I2C			1
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_SYS_I2C_BUS		0
#define CONFIG_SYS_I2C_BUS_SELECT	1
#define CONFIG_DRIVER_OMAP34XX_I2C	1

/*
 * TWL4030
 */
#define CONFIG_TWL4030_POWER		1
#undef  CONFIG_ENABLE_TWL4030_CHARGING	/* DM3730 Torpedo uses external
					   charging circuit; not twl4030 */

/*
 * Board NAND Info.
 */
#define CONFIG_SYS_NAND_ADDR		NAND_BASE	/* physical address */
							/* to access nand */
#define CONFIG_SYS_NAND_BASE		NAND_BASE	/* physical address */
							/* to access */
							/* nand at CS0 */

#define CONFIG_SYS_MAX_NAND_DEVICE	1		/* Max number of */
							/* NAND devices */
#define CONFIG_SYS_64BIT_VSPRINTF		/* needed for nand_util.c */

#define CONFIG_JFFS2_NAND
/* nand device jffs2 lives on */
#define CONFIG_JFFS2_DEV		"nand0"
/* start of jffs2 partition */
#define CONFIG_JFFS2_PART_OFFSET	0x680000
#define CONFIG_JFFS2_PART_SIZE		0xf980000	/* sz of jffs2 part */

/* Environment information */
#define CONFIG_BOOTDELAY	6

#define CONFIG_BOOTFILE		uImage

#define CONFIG_PREBOOT \
    "setenv preboot \"echo current boot target:; printenv bootcmd;\";"    \
    "saveenv;" 


#define CONFIG_EXTRA_ENV_SETTINGS \
    "rotate_type=1\0" \
	"vram=10M\0" \
	"fbram=1:8M\0" \
    "rotate=2\0" \
    "loadaddr=0x81000000\0"                     \
    "rootfsaddr=0x81300000\0"                   \
    "consoledev=ttyS0\0"                        \
    "rootpath=/opt/nfs-exports/ltib-omap\0"             \
    "rootdev=/dev/mtdblock5\0"                        \
    "kernelimage=uImage\0"                      \
    "stable=bankA\0"                      \
    "kernelblock=280000\0"                      \
    "nfsoptions=,wsize=1500,rsize=1500\0"               \
    "mem1=mem=126M@0x80000000\0" \
    "mem2=mem=128M@0x88000000\0" \
    "memsize=126M\0" \
    "rekern=bootp; nand erase ${kernelblock} 400000; nand write ${loadaddr} ${kernelblock} 400000; boot\0" \
    "nfsboot=bootp; setenv bootargs ${mem1} ${mem2} omapfb.vrfb=${rotate_type} vram=${vram} omapfb.rotate=${rotate} omapfb.vram=${fbram} omapfb.debug=${display_debug} console=${consoledev},${baudrate} root=/dev/nfs rw nfsroot=${rootpath}${nfsoptions} ip=dhcp init=/init androidboot.console=${consoledev} ${otherbootargs};bootm ${loadaddr}\0" \
    "flashboot=setenv bootargs ${mem1} ${mem2} omapfb.vrfb=${rotate_type} vram=${vram} omapfb.rotate=${rotate} omapfb.vram=${fbram} omapfb.debug=${display_debug} console=${consoledev},${baudrate} root=${rootdev} rootfstype=yaffs2 rw init=/init androidboot.console=${consoledev} ${otherbootargs}; nand read ${loadaddr} ${kernelblock} 400000; bootm ${loadaddr}\0" \
    "install=mmc init; fatload mmc 0 80000000 install.img; source\0" \
    "restart=nand read 80000000 1fe80000 20000; source\0" \
    "reboot=run restart\0" \
    "sdboot=setenv bootargs mem=${memsize} vram=${vram} omapfb.vra,=${fbram} omapfb.vrfb=${rotate_type} omapfb.rotate=${rotate} console=${consoledev},${baudrate} root=/dev/mmcblk0p2 rootfstype=ext3 rw rootwait init=/init; mmc init; fatload mmc 0 ${loadaddr} ${kernelimage}; bootm ${loadaddr}\0"

#define CONFIG_BOOTCOMMAND \
	"run restart"

#define CONFIG_AUTO_COMPLETE	1
/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
#define CONFIG_SYS_HUSH_PARSER		/* use "hush" command parser */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_PROMPT		"=> "
#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		64	/* max number of command */
						/* args */
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		(CONFIG_SYS_CBSIZE)
/* memtest works on */
#define CONFIG_SYS_MEMTEST_START	(OMAP34XX_SDRC_CS0)
#define CONFIG_SYS_MEMTEST_END		(OMAP34XX_SDRC_CS0 + \
					0x01F00000) /* 31MB */

#define CONFIG_SYS_LOAD_ADDR		(OMAP34XX_SDRC_CS0) /* default load */
								/* address */

/*
 * OMAP3 has 12 GP timers, they can be driven by the system clock
 * (12/13/16.8/19.2/38.4MHz) or by 32KHz clock. We use 13MHz (V_SCLK).
 * This rate is divided by a local divisor.
 */
#define CONFIG_SYS_TIMERBASE		OMAP34XX_GPT2
#define CONFIG_SYS_PTV			2	/* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ			1000

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 << 10)	/* regular stack 128 KiB */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4 << 10)	/* IRQ stack 4 KiB */
#define CONFIG_STACKSIZE_FIQ	(4 << 10)	/* FIQ stack 4 KiB */
#endif

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	2	/* CS1 may or may not be populated */
#define PHYS_SDRAM_1		OMAP34XX_SDRC_CS0
#define PHYS_SDRAM_1_SIZE	(32 << 20)	/* at least 32 MiB */
#define PHYS_SDRAM_2		OMAP34XX_SDRC_CS1

/* SDRAM Bank Allocation method */
#define SDRC_R_B_C		1

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

/* **** PISMO SUPPORT *** */

/* Configure the PISMO */
#define PISMO1_NAND_SIZE		GPMC_SIZE_128M
#define PISMO1_ONEN_SIZE		GPMC_SIZE_128M

#define CONFIG_SYS_MONITOR_LEN		(256 << 10)	/* Reserve 2 sectors */

#define CONFIG_SYS_FLASH_BASE		boot_flash_base

/* Monitor at start of flash */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_ONENAND_BASE		ONENAND_MAP

#if defined(CONFIG_CMD_NAND)
#define CONFIG_NAND_OMAP_GPMC
#define GPMC_NAND_ECC_LP_x16_LAYOUT	1
#define CONFIG_ENV_IS_IN_NAND
#elif defined(CONFIG_CMD_ONENAND)
#define CONFIG_ENV_IS_IN_ONENAND	1
#endif
#define ONENAND_ENV_OFFSET		0x260000 /* environment starts here */
#define SMNAND_ENV_OFFSET		0x260000 /* environment starts here */

#define CONFIG_SYS_ENV_SECT_SIZE	boot_flash_sec
#define CONFIG_ENV_OFFSET		boot_flash_off
#define CONFIG_ENV_ADDR			boot_flash_env_addr

#define CONFIG_MTD_DEBUG		1
#define CONFIG_MTD_DEBUG_VERBOSE	-1
#define CONFIG_MTD_SKIP_BBTSCAN		1	/* Skip NAND bad block scan */
#define CONFIG_TOOL_SIGNGP		1

#ifndef __ASSEMBLY__
extern unsigned int boot_flash_base;
extern volatile unsigned int boot_flash_env_addr;
extern unsigned int boot_flash_off;
extern unsigned int boot_flash_sec;
extern unsigned int boot_flash_type;
#endif

/*
 * Support for relocation
 */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define CONFIG_SYS_INIT_RAM_ADDR	0x4020f800
#define CONFIG_SYS_INIT_RAM_SIZE	0x800
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + \
					 CONFIG_SYS_INIT_RAM_SIZE - \
					 GENERATED_GBL_DATA_SIZE)

/*----------------------------------------------------------------------------
 * SMSC9115 Ethernet from SMSC9118 family
 *----------------------------------------------------------------------------
 */
#if defined(CONFIG_CMD_NET)

#define CONFIG_NET_MULTI
#define CONFIG_SMC911X
#define CONFIG_SMC911X_16_BIT
#define CONFIG_SMC911X_BASE	0x08000000

#endif /* (CONFIG_CMD_NET) */

/*
 * BOOTP fields
 */

#define CONFIG_BOOTP_SUBNETMASK		0x00000001
#define CONFIG_BOOTP_GATEWAY		0x00000002
#define CONFIG_BOOTP_HOSTNAME		0x00000004
#define CONFIG_BOOTP_BOOTPATH		0x00000010

#endif /* __CONFIG_H */
