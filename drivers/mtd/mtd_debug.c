/*
 * MTD verbose debug 
 *
 * (C) 2011 Peter Barada <peter.barada@logicpd.com>
 *
 * This code is GPL
 */

#include <linux/mtd/mtd.h>
#include <linux/mtd/compat.h>
#include <ubi_uboot.h>

#ifdef CONFIG_MTD_DEBUG
#ifndef CONFIG_MTD_DEBUG_VERBOSE
#define CONFIG_MTD_DEBUG_VERBOSE -1
#endif
int mtd_debug_verbose = CONFIG_MTD_DEBUG_VERBOSE;
#endif
