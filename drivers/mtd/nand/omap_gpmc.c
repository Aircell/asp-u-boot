/*
 * (C) Copyright 2004-2008 Texas Instruments, <www.ti.com>
 * Rohit Choraria <rohitkc@ti.com>
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
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/mem.h>
#include <asm/arch/omap_gpmc.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/omap_bch_soft.h>
#include <linux/mtd/nand_ecc.h>
#include <nand.h>

uint8_t cs;
static struct nand_ecclayout hw_nand_oob = GPMC_NAND_HW_ECC_LAYOUT;
static struct nand_ecclayout chip_nand_oob = GPMC_NAND_CHIP_ECC_LAYOUT;

/*
 * omap_nand_hwcontrol - Set the address pointers corretly for the
 *			following address/data/command operation
 */
static void omap_nand_hwcontrol(struct mtd_info *mtd, int32_t cmd,
				uint32_t ctrl)
{
	register struct nand_chip *this = mtd->priv;

	/*
	 * Point the IO_ADDR to DATA and ADDRESS registers instead
	 * of chip address
	 */
	switch (ctrl) {
	case NAND_CTRL_CHANGE | NAND_CTRL_CLE:
		this->IO_ADDR_W = (void __iomem *)&gpmc_cfg->cs[cs].nand_cmd;
		break;
	case NAND_CTRL_CHANGE | NAND_CTRL_ALE:
		this->IO_ADDR_W = (void __iomem *)&gpmc_cfg->cs[cs].nand_adr;
		break;
	case NAND_CTRL_CHANGE | NAND_NCE:
		this->IO_ADDR_W = (void __iomem *)&gpmc_cfg->cs[cs].nand_dat;
		break;
	}

	if (cmd != NAND_CMD_NONE) {
#ifdef CONFIG_MTD_DEBUG
		if (this->IO_ADDR_W == &gpmc_cfg->cs[cs].nand_cmd)
			MTDDEBUG(MTD_DEBUG_LEVEL4, "NAND C: %02x\n", cmd & 0xff);
		else if (this->IO_ADDR_W == &gpmc_cfg->cs[cs].nand_adr)
			MTDDEBUG(MTD_DEBUG_LEVEL4, "NAND A: %02x\n", cmd & 0xff);
		else if (this->IO_ADDR_W == &gpmc_cfg->cs[cs].nand_dat)
			MTDDEBUG(MTD_DEBUG_LEVEL4, "NAND D: %02x\n", cmd & 0xff);
#endif
		writeb(cmd, this->IO_ADDR_W);
	}
}

/*
 * omap_hwecc_init - Initialize the Hardware ECC for NAND flash in
 *                   GPMC controller
 * @mtd:        MTD device structure
 *
 */
static void omap_hwecc_init(struct nand_chip *chip)
{
	/*
	 * Init ECC Control Register
	 * Clear all ECC | Enable Reg1
	 */
	writel(ECCCLEAR | ECCRESULTREG1, &gpmc_cfg->ecc_control);
	writel(ECCSIZE1 | ECCSIZE0 | ECCSIZE0SEL, &gpmc_cfg->ecc_size_config);
}

/*
 * gen_true_ecc - This function will generate true ECC value, which
 * can be used when correcting data read from NAND flash memory core
 *
 * @ecc_buf:	buffer to store ecc code
 *
 * @return:	re-formatted ECC value
 */
static uint32_t gen_true_ecc(uint8_t *ecc_buf)
{
	return ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) |
		((ecc_buf[2] & 0x0F) << 8);
}



/*
 * omap_correct_data - Compares the ecc read from nand spare area with ECC
 * registers values and corrects one bit error if it has occured
 * Further details can be had from OMAP TRM and the following selected links:
 * http://en.wikipedia.org/wiki/Hamming_code
 * http://www.cs.utexas.edu/users/plaxton/c/337/05f/slides/ErrorCorrection-4.pdf
 *
 * @mtd:		 MTD device structure
 * @dat:		 page data
 * @read_ecc:		 ecc read from nand flash
 * @calc_ecc:		 ecc read from ECC registers
 *
 * @return 0 if data is OK or corrected, else returns -1
 */
static int omap_correct_data(struct mtd_info *mtd, uint8_t *dat,
				uint8_t *read_ecc, uint8_t *calc_ecc)
{
	uint32_t orig_ecc, new_ecc, res, hm;
	uint16_t parity_bits, byte;
	uint8_t bit;

	/* Regenerate the orginal ECC */
	orig_ecc = gen_true_ecc(read_ecc);
	new_ecc = gen_true_ecc(calc_ecc);
	/* Get the XOR of real ecc */
	res = orig_ecc ^ new_ecc;
	if (res) {
		/* Get the hamming width */
		hm = hweight32(res);
		/* Single bit errors can be corrected! */
		if (hm == 12) {
			/* Correctable data! */
			parity_bits = res >> 16;
			bit = (parity_bits & 0x7);
			byte = (parity_bits >> 3) & 0x1FF;
			/* Flip the bit to correct */
			dat[byte] ^= (0x1 << bit);
		} else if (hm == 1) {
			printf("Error: Ecc is wrong\n");
			/* ECC itself is corrupted */
			return 2;
		} else {
			/*
			 * hm distance != parity pairs OR one, could mean 2 bit
			 * error OR potentially be on a blank page..
			 * orig_ecc: contains spare area data from nand flash.
			 * new_ecc: generated ecc while reading data area.
			 * Note: if the ecc = 0, all data bits from which it was
			 * generated are 0xFF.
			 * The 3 byte(24 bits) ecc is generated per 512byte
			 * chunk of a page. If orig_ecc(from spare area)
			 * is 0xFF && new_ecc(computed now from data area)=0x0,
			 * this means that data area is 0xFF and spare area is
			 * 0xFF. A sure sign of a erased page!
			 */
			if ((orig_ecc == 0x0FFF0FFF) && (new_ecc == 0x00000000))
				return 0;
			printf("Error: Bad compare! failed\n");
			/* detected 2 bit error */
			return -1;
		}
	}
	return 0;
}

static int omap_correct_chip_hwecc(struct mtd_info *mtd, u_char *dat,
				u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *chip;

	chip = mtd->priv;

	printf("%s: ecc_status %02x\n", __func__, chip->ecc_status);
	/* We stored the read status in info->ecc_status in the read.
	   If bit 0 is set, then there was an uncorrectable ECC error.
	   If bit 3 is set, then there was a correctable error (up to
	   four bits of correction). */
	if (chip->ecc_status & 0x01)
		return -1;
	if (chip->ecc_status & 0x08)
		return 4;
	return 0;
}

/*
 *  omap_calculate_ecc - Generate non-inverted ECC bytes.
 *
 *  Using noninverted ECC can be considered ugly since writing a blank
 *  page ie. padding will clear the ECC bytes. This is no problem as
 *  long nobody is trying to write data on the seemingly unused page.
 *  Reading an erased page will produce an ECC mismatch between
 *  generated and read ECC bytes that has to be dealt with separately.
 *  E.g. if page is 0xFF (fresh erased), and if HW ECC engine within GPMC
 *  is used, the result of read will be 0x0 while the ECC offsets of the
 *  spare area will be 0xFF which will result in an ECC mismatch.
 *  @mtd:	MTD structure
 *  @dat:	unused
 *  @ecc_code:	ecc_code buffer
 */
static int omap_calculate_ecc(struct mtd_info *mtd, const uint8_t *dat,
				uint8_t *ecc_code)
{
	u_int32_t val;

	/* Start Reading from HW ECC1_Result = 0x200 */
	val = readl(&gpmc_cfg->ecc1_result);

	ecc_code[0] = val & 0xFF;
	ecc_code[1] = (val >> 16) & 0xFF;
	ecc_code[2] = ((val >> 8) & 0x0F) | ((val >> 20) & 0xF0);

	/*
	 * Stop reading anymore ECC vals and clear old results
	 * enable will be called if more reads are required
	 */
	writel(0x000, &gpmc_cfg->ecc_config);

	return 0;
}

static int omap_calculate_chip_hwecc(struct mtd_info *mtd, const u_char *dat,
				u_char *ecc_code)
{
	MTDDEBUG(MTD_DEBUG_LEVEL3, "%s:\n", __func__);
	return 0;
}

/*
 * omap_enable_ecc - This function enables the hardware ecc functionality
 * @mtd:        MTD device structure
 * @mode:       Read/Write mode
 */
static void omap_enable_hwecc(struct mtd_info *mtd, int32_t mode)
{
	struct nand_chip *chip = mtd->priv;
	uint32_t val, dev_width = (chip->options & NAND_BUSWIDTH_16) >> 1;

	switch (mode) {
	case NAND_ECC_READ:
	case NAND_ECC_WRITE:
		/* Clear the ecc result registers, select ecc reg as 1 */
		writel(ECCCLEAR | ECCRESULTREG1, &gpmc_cfg->ecc_control);

		/*
		 * Size 0 = 0xFF, Size1 is 0xFF - both are 512 bytes
		 * tell all regs to generate size0 sized regs
		 * we just have a single ECC engine for all CS
		 */
		writel(ECCSIZE1 | ECCSIZE0 | ECCSIZE0SEL,
			&gpmc_cfg->ecc_size_config);
		val = (dev_width << 7) | (cs << 1) | (0x1);
		writel(val, &gpmc_cfg->ecc_config);
		break;
	default:
		printf("Error: Unrecognized Mode[%d]!\n", mode);
		break;
	}
}

static void omap_enable_chip_hwecc(struct mtd_info *mtd, int mode)
{
	MTDDEBUG(MTD_DEBUG_LEVEL3, "%s:\n", __func__);
}

/*
 * omap_nand_chip_has_ecc - return true if chip has internal ECC
 */
int omap_nand_chip_has_ecc(void)
{
	struct nand_chip *chip;
	struct mtd_info *mtd;
	int		i;
	uint8_t		ident[5];

	if (nand_curr_device < 0 ||
	    nand_curr_device >= CONFIG_SYS_MAX_NAND_DEVICE ||
	    !nand_info[nand_curr_device].name) {
		printf("Error: Can't switch ecc, no devices available\n");
		return 0;
	}

	mtd = &nand_info[nand_curr_device];
	chip = mtd->priv;

#if 1
	chip->select_chip(mtd, 0);
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Wait for the chip to get the ID ready */
	ndelay(100);

	for (i=0; i<2; ++i)
		ident[i] = chip->read_byte(mtd);

	MTDDEBUG(MTD_DEBUG_LEVEL3, "%s:%d %02x %02x\n", __FUNCTION__, __LINE__, ident[0], ident[1]);
	if (ident[0] == NAND_MFR_MICRON) {
		for (i=2; i<5; ++i)
			ident[i] = chip->read_byte(mtd);
		MTDDEBUG(MTD_DEBUG_LEVEL3, "%s:%d %02x %02x %02x\n", __FUNCTION__, __LINE__, ident[2], ident[3], ident[4]);
		if (ident[4] & 0x3)
			chip->has_chip_ecc = 1;
	}
	MTDDEBUG(MTD_DEBUG_LEVEL3, "%s: has_chip_ecc %d\n", __FUNCTION__, chip->has_chip_ecc);
#else
	if (nand->maf_id == NAND_MFR_MICRON) {
		switch(nand->dev_id) {
		case 0x2c:
		case 0xdc:
		case 0xcc:
		case 0xac:
		case 0xbc:
		case 0xa3:
		case 0xb3:
		case 0xd3:
		case 0xc3:
			nand->has_chip_ecc = 1;
			return 1;
		default:
			break;
		}
	}
#endif
	return chip->has_chip_ecc;
}


static void micron_set_chip_ecc(struct mtd_info *mtd, int enable)
{
	uint8_t params[4];

	MTDDEBUG(MTD_DEBUG_LEVEL3,"%s:%d enable %d\n", __FUNCTION__, __LINE__, enable);

	memset(params, 0x00, sizeof(params));
	if (enable)
		params[0] = 0x08;
	nand_set_features(mtd, 0x90, params);

#if 1
	nand_get_features(mtd, 0x90, params);
	MTDDEBUG(MTD_DEBUG_LEVEL4,"%s: %02x %02x %02x %02x\n", __FUNCTION__, params[0], params[1], params[2], params[3]);
#endif
}

/**
 * nand_read_oob_chipecc - read; get status to seeif chip ECC error
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @sndcmd:	flag whether to issue read command or not
 */
static int omap_read_oob_chipecc(struct mtd_info *mtd, struct nand_chip *chip,
			     int page, int sndcmd)
{
	struct nand_chip *nand;

	nand = mtd->priv;

	MTDDEBUG(MTD_DEBUG_LEVEL3, "%s: page = %d, len = %i\n",
			__func__, page, mtd->oobsize);

	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}

	/* Send the status command */
	omap_nand_hwcontrol(mtd, NAND_CMD_STATUS,
		NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
	/* Switch to data access */
	omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
		NAND_NCE | NAND_CTRL_CHANGE);
	chip->ecc_status = chip->read_byte(mtd);
	MTDDEBUG(MTD_DEBUG_LEVEL3, "%s: ecc_status %02x\n", __func__, chip->ecc_status);
	if (chip->ecc_status & (0x8|0x1)) {
		MTDDEBUG(MTD_DEBUG_LEVEL3, "%s:%d page %d ecc_status %02x\n", __FUNCTION__, __LINE__, page, chip->ecc_status);
		if (chip->ecc_status & 0x1)
			mtd->ecc_stats.failed++;
		else if (chip->ecc_status & 0x80)
			mtd->ecc_stats.corrected += 4;
	}

	/* Send the read prefix */
	omap_nand_hwcontrol(mtd, NAND_CMD_READ0,
		NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
	/* Switch to data access */
	omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
		NAND_NCE | NAND_CTRL_CHANGE);
	
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	return sndcmd;
}

/**
 * omap_nand_command_lp - Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void omap_nand_command_lp(struct mtd_info *mtd, unsigned int command,
			    int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	omap_nand_hwcontrol(mtd, command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			omap_nand_hwcontrol(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			omap_nand_hwcontrol(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			omap_nand_hwcontrol(mtd, page_addr, ctrl);
			omap_nand_hwcontrol(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				omap_nand_hwcontrol(mtd, page_addr >> 16,
					       NAND_NCE | NAND_ALE);
		}
	}
	omap_nand_hwcontrol(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

		/*
		 * read error status commands require only a short delay
		 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		omap_nand_hwcontrol(mtd, NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY)) ;
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		omap_nand_hwcontrol(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:

		/* Send the read start */
		omap_nand_hwcontrol(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			goto ready_exit;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	nand_wait_ready(mtd);

ready_exit:
	/* If the chip has internal ECC, then we need to read the status
	   to determin if there's an ECC error - capture it for handling by
	   omap_nand_correct_chip_hwecc() later */
	if (command == NAND_CMD_READ0) {
		if (chip->has_chip_ecc) {

			/* Send the status command */
			omap_nand_hwcontrol(mtd, NAND_CMD_STATUS,
				NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
			/* Switch to data access */
			omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
				NAND_NCE | NAND_CTRL_CHANGE);
			chip->ecc_status = chip->read_byte(mtd);
			MTDDEBUG(MTD_DEBUG_LEVEL3, "%s: ecc_status %02x\n", __func__, chip->ecc_status);
#if 0
			if (chip->ecc_status & (0x8|0x1))
				printk("%s:%d page %d column %d ecc_status %02x\n", __FUNCTION__, __LINE__, page_addr, column, chip->ecc_status);
#endif

			/* Send the read prefix */
			omap_nand_hwcontrol(mtd, NAND_CMD_READ0,
				NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
			/* Switch to data access */
			omap_nand_hwcontrol(mtd, NAND_CMD_NONE,
				NAND_NCE | NAND_CTRL_CHANGE);

		}
	}
		
}

/*
 * omap_nand_switch_ecc - switch the ECC operation b/w h/w ecc and s/w ecc.
 * The default is to come up on s/w ecc
 *
 * @hardware - 1 -switch to h/w ecc, 0 - s/w ecc, 2 - chip ecc
 *
 */
void omap_nand_switch_ecc(enum omap_nand_ecc_mode mode)
{
  struct nand_chip *nand;
  struct mtd_info *mtd;
  uint32_t dev_width;

  if (nand_curr_device < 0 ||
      nand_curr_device >= CONFIG_SYS_MAX_NAND_DEVICE ||
      !nand_info[nand_curr_device].name) {
    printf("Error: Can't switch ecc, no devices available\n");
    return;
  }

  mtd = &nand_info[nand_curr_device];
  nand = mtd->priv;

  nand->options |= NAND_OWN_BUFFERS;

  /* Reset ecc interface */
  nand->ecc.read_page = NULL;
  nand->ecc.write_page = NULL;
  nand->ecc.read_oob = NULL;
  nand->ecc.write_oob = NULL;
  nand->ecc.hwctl = NULL;
  nand->ecc.correct = NULL;
  nand->ecc.calculate = NULL;

  dev_width = (nand->options & NAND_BUSWIDTH_16) >> 1;

  switch(mode)
    {
    case NAND_ECC_HW:
      nand->ecc.mode = NAND_ECC_HW;
      nand->ecc.layout = &hw_nand_oob;	
      nand->ecc.hwctl = omap_enable_hwecc;
      nand->ecc.size = 512;
      nand->ecc.bytes = 3;
      nand->ecc.correct = omap_correct_data;
      nand->ecc.calculate = omap_calculate_ecc;
      omap_hwecc_init(nand);
      if (nand->has_chip_ecc)
	      micron_set_chip_ecc(mtd, 0);
      printf("HW ECC selected\n");
      break;
    case NAND_ECC_SOFT:
      nand->ecc.mode = NAND_ECC_SOFT;
      /* Use mtd default settings */
      nand->ecc.layout = NULL;
      printf("SW ECC selected\n");
      if (nand->has_chip_ecc)
	      micron_set_chip_ecc(mtd, 0);
      break;
    case NAND_ECC_CHIP:
	    if (!nand->has_chip_ecc) {
		    printf("NAND: Chip does not have internal ECC!\n");
		    return;
	    }
	    nand->ecc.bytes = 0;
	    nand->ecc.size = 2048;
	    nand->ecc.calculate = omap_calculate_chip_hwecc;
	    nand->ecc.hwctl = omap_enable_chip_hwecc;
	    nand->ecc.correct = omap_correct_chip_hwecc;
	    nand->ecc.read_oob = omap_read_oob_chipecc;
	    nand->ecc.mode = NAND_ECC_CHIP; /* internal to chip */
	    nand->ecc.layout = &chip_nand_oob;
	    if (nand->options & NAND_BUSWIDTH_16)
		    nand->cmdfunc = omap_nand_command_lp;
	    else
		    printf("%s: Huh? not 16-bit wide\n", __FUNCTION__);
	    micron_set_chip_ecc(mtd, 1);
	    printf("NAND: Internal to NAND ECC selected\n");
	    break;

    case NAND_ECC_4BIT_SOFT:
      nand->ecc.mode = mode;
      nand->ecc.layout = omap_get_ecc_layout_bch(dev_width, 4);
      nand->ecc.hwctl = omap_enable_hwecc_bch4;
      nand->ecc.size = 2048;
      nand->ecc.bytes = 28;
      nand->ecc.calculate = omap_calculate_ecc_bch4;
      nand->ecc.correct = omap_correct_data_bch4;
      omap_hwecc_init_bch(nand);
      if (nand->has_chip_ecc)
	      micron_set_chip_ecc(mtd, 0);
      printf("4 BIT SW ECC selected\n");
      break;
    case NAND_ECC_8BIT_SOFT:
      nand->ecc.mode = mode;
      nand->ecc.layout = omap_get_ecc_layout_bch(dev_width, 8);
      nand->ecc.hwctl = omap_enable_hwecc_bch8;
      nand->ecc.size = 2048;
      nand->ecc.bytes = 52;
      nand->ecc.calculate = omap_calculate_ecc_bch8;
      nand->ecc.correct = omap_correct_data_bch8;
      omap_hwecc_init_bch(nand);
      if (nand->has_chip_ecc)
	      micron_set_chip_ecc(mtd, 0);
      printf("8 BIT SW ECC selected\n");
      break;
    default:
	    printk("Unknown ECC method selected!\n");
	    break;
    }

  /* Update NAND handling after ECC mode switch */
  nand_scan_tail(mtd);

  nand->options &= ~NAND_OWN_BUFFERS;
}

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific:
 * - IO_ADDR_R: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W: address to write the 8 I/O lines of the flash device
 * - cmd_ctrl: hardwarespecific function for accesing control-lines
 * - waitfunc: hardwarespecific function for accesing device ready/busy line
 * - ecc.hwctl: function to enable (reset) hardware ecc generator
 * - ecc.mode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 */
int board_nand_init(struct nand_chip *nand)
{
  int32_t gpmc_config = 0;
  cs = 0;

  /*
   * xloader/Uboot's gpmc configuration would have configured GPMC for
   * nand type of memory. The following logic scans and latches on to the
   * first CS with NAND type memory.
   * TBD: need to make this logic generic to handle multiple CS NAND
   * devices.
   */
  while (cs < GPMC_MAX_CS) {
    /* Check if NAND type is set */
    if ((readl(&gpmc_cfg->cs[cs].config1) & 0xC00) == 0x800) {
      /* Found it!! */
      break;
    }
    cs++;
  }
  if (cs >= GPMC_MAX_CS) {
    printf("NAND: Unable to find NAND settings in "
	   "GPMC Configuration - quitting\n");
    return -ENODEV;
  }

  gpmc_config = readl(&gpmc_cfg->config);
  /* Disable Write protect */
  gpmc_config |= 0x10;
  writel(gpmc_config, &gpmc_cfg->config);

  nand->IO_ADDR_R = (void __iomem *)&gpmc_cfg->cs[cs].nand_dat;
  nand->IO_ADDR_W = (void __iomem *)&gpmc_cfg->cs[cs].nand_cmd;

  nand->cmd_ctrl = omap_nand_hwcontrol;
  nand->options = NAND_NO_PADDING | NAND_CACHEPRG | NAND_NO_AUTOINCR;
  /* If we are 16 bit dev, our gpmc config tells us that */
  if ((readl(&gpmc_cfg->cs[cs].config1) & 0x3000) == 0x1000)
    nand->options |= NAND_BUSWIDTH_16;

#ifdef CONFIG_MTD_SKIP_BBTSCAN
	/* Skip the bad block scan */
	nand->options |= NAND_SKIP_BBTSCAN;
#endif

  nand->chip_delay = 100;

  nand->ecc.mode = NAND_ECC_4BIT_SOFT;

  return 0;
}
