// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2013 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2023 by Max A. Dednev                                   *
 *   m.dednev@yandex.ru                                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/bits.h>
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/riscv/riscv.h>

/* Define USE_MDR_READ = 1 to use hardware-specific flash controller read function */
#define USE_MDR_READ 0

/* Reset and clock controller base address for generic ARM/RISC-V controllers */
#define MD_RST_CLK		(0x40020000)
/* Reset and clock controller base address for MDR1206F RISC-V controllers */
#define MDR1206_RST_CLK		(0x50020000)

/* ARM-based MCU definitions */
#define MD_PER_CLOCK		(MD_RST_CLK + 0x1C)
#define MD_PER_CLOCK_FLASH	BIT(3)
#define MD_PER_CLOCK_RST_CLK	BIT(4)

/* RISCV-based MCU definitions (f.e. MDR32F02FI) */
#define MD_PER2_CLOCK		(MD_RST_CLK + 0x1C)
#define MD_PER2_CLOCK_FLASH	BIT(3)
#define MD_PER2_CLOCK_RST_CLK	BIT(4)

/* MDR1206 RISCV-based MCU definitions */
#define MDR1206_PER2_CLOCK		(MDR1206_RST_CLK + 0x1C)
#define MDR1206_PER2_CLOCK_FLASH	BIT(3)
#define MDR1206_PER2_CLOCK_RST_CLK	BIT(4)

/* Flash memory controller base address for generic ARM/RISC-V controllers */
#define MD_FLASH_REG_BASE	0x40018000
/* Reset and clock controller base address for MDR1206F RISC-V controllers */
#define MDR1206_FLASH_REG_BASE	0x50018000

#define MD_FLASH_CMD		(mdr_info->flash_base + 0x00)
#define MD_FLASH_ADR		(mdr_info->flash_base + 0x04)
#define MD_FLASH_DI		(mdr_info->flash_base + 0x08)
#define MD_FLASH_DO		(mdr_info->flash_base + 0x0C)
#define MD_FLASH_KEY		(mdr_info->flash_base + 0x10)
#define MD_FLASH_CTRL		(mdr_info->flash_base + 0x14) /* Only for MDR1206FI CHIP_ID = 215 or 217 */
#define MD_CHIP_ID_CTRL		(mdr_info->flash_base + 0x18) /* Only for MDR1206FI CHIP_ID = 215 or 217 */

#define MD_FLASH_TMR		BIT(14)
#define MD_FLASH_NVSTR		BIT(13)
#define MD_FLASH_PROG		BIT(12)
#define MD_FLASH_MAS1		BIT(11)
#define MD_FLASH_ERASE		BIT(10)
#define MD_FLASH_IFREN		BIT(9)
#define MD_FLASH_SE		BIT(8)
#define MD_FLASH_YE		BIT(7)
#define MD_FLASH_XE		BIT(6)
#define MD_FLASH_RD		BIT(2)
#define MD_FLASH_WR		BIT(1)
#define MD_FLASH_CON		BIT(0)
#define MD_FLASH_DELAY_MASK	(7 << 3)

/* MDR1206FI CHIP_ID = 215 specific definitions */
#define MD_FLASH_TMEN_215	BIT(14)
#define MD_FLASH_PROG2_215	BIT(13)
#define MD_FLASH_PROG_215	BIT(12)
#define MD_FLASH_CHIP_215	BIT(11)
#define MD_FLASH_ERASE_215	BIT(10)
#define MD_FLASH_NVR_215	BIT(9)
#define MD_FLASH_RE_215		BIT(8)
#define MD_FLASH_WE_215		BIT(7)
#define MD_FLASH_CE_215		BIT(6)
#define MD_FLASH_CON_215	BIT(0)

/* Protection key */
#define FLASH_KEY_UNLOCK_KEY	(0x8AAA5551U)
#define FLASH_KEY_LOCK_KEY	(0x0U)

/* Operation timing in microseconds (us) */
#define MD_FLASH_T_NVS		(mdr_info->t_nvs)
#define MD_FLASH_T_ERASE	(mdr_info->t_erase)
#define MD_FLASH_T_ME		(mdr_info->t_me)
#define MD_FLASH_T_NVH		(mdr_info->t_nvh)
#define MD_FLASH_T_NVH1		(mdr_info->t_nvh1)
#define MD_FLASH_T_PROG		(mdr_info->t_prog)
#define MD_FLASH_T_PGS		(mdr_info->t_pgs)
#define MD_FLASH_T_RCV		(mdr_info->t_rcv)

/* MDR32F02 and MDR1206FI FLASH parameters with a safe margin of ~7% (HSI spread). */
#define FLASH_TNVS_US   (6)
#define FLASH_TNVH_US   (6)
#define FLASH_TNVH1_US  (107)
#define FLASH_TPGS_US   (11)
#define FLASH_TRCV_US   (11)
#define FLASH_THV_US    (16000)

#define FLASH_TPROG_US  (30)
#define FLASH_TERASE_US (30000)
#define FLASH_TME_US    (30000)

/* MDR1206AFI FLASH parameters with a safe margin of ~7% (HSI spread). */
#define FLASH_TNVS_PROGRAM_US        (22)
#define FLASH_TRCV_PROGRAM_US        (54)
#define FLASH_TRW_PROGRAM_US         (1)
#define FLASH_TADH_PROGRAM_US        (1)
#define FLASH_TADS_PROGRAM_US        (1)
#define FLASH_TPGH_PROGRAM_US        (1)
#define FLASH_TPGS_PROGRAM_US        (54)
#define FLASH_TPROG_PROGRAM_US       (5)

#define FLASH_TNVS_SECTOR_ERASE_US   (22)
#define FLASH_TRCV_SECTOR_ERASE_US   (54)
#define FLASH_TRW_SECTOR_ERASE_US    (1)
#define FLASH_TERASE_SECTOR_ERASE_US (2500)

#define FLASH_TNVS_CHIP_ERASE_US     (86)
#define FLASH_TRCV_CHIP_ERASE_US     (214)
#define FLASH_TRW_CHIP_ERASE_US      (11)
#define FLASH_TERASE_CHIP_ERASE_US   (35000)

struct mdr_flash_bank {
	bool          probed;
	bool          riscv;
	uint32_t      flash_base;
	uint32_t      per_clock;
	uint32_t      per_clock_flash_en;
	uint32_t      per_clock_rst_clk;
	uint32_t      chip_id;
	uint32_t      calib_offs;
	size_t        calib_size;
	uint32_t      calib_values[8];

	unsigned int  mem_type;
	unsigned int  bank_count;
	unsigned int  sect_count;
	unsigned int  page_size;

	uint32_t      ext_flags;
	const uint8_t *flash_write_code;
	size_t        flash_write_code_size;

	unsigned int  t_nvs;
	unsigned int  t_erase;
	unsigned int  t_me;
	unsigned int  t_nvh;
	unsigned int  t_nvh1;
	unsigned int  t_prog;
	unsigned int  t_pgs;
	unsigned int  t_rcv;
};

/* see contrib/loaders/flash/mdr32fx.S for src */
static const uint8_t mdr32fx_flash_write_code[] = {
	0x07, 0x68, 0x16, 0x68, 0x00, 0x2e, 0x2e, 0xd0, 0x55, 0x68, 0xb5, 0x42,
	0xf9, 0xd0, 0x2e, 0x68, 0x44, 0x60, 0x86, 0x60, 0x17, 0x4e, 0x37, 0x43,
	0x07, 0x60, 0x05, 0x26, 0x00, 0xf0, 0x25, 0xf8, 0x15, 0x4e, 0x37, 0x43,
	0x07, 0x60, 0x0d, 0x26, 0x00, 0xf0, 0x1f, 0xf8, 0x80, 0x26, 0x37, 0x43,
	0x07, 0x60, 0x3d, 0x26, 0x00, 0xf0, 0x19, 0xf8, 0x80, 0x26, 0xb7, 0x43,
	0x07, 0x60, 0x0f, 0x4e, 0xb7, 0x43, 0x07, 0x60, 0x05, 0x26, 0x00, 0xf0,
	0x10, 0xf8, 0x0d, 0x4e, 0xb7, 0x43, 0x07, 0x60, 0x04, 0x35, 0x04, 0x34,
	0x9d, 0x42, 0x01, 0xd3, 0x15, 0x46, 0x08, 0x35, 0x55, 0x60, 0x01, 0x39,
	0x00, 0x29, 0x00, 0xd0, 0xcd, 0xe7, 0x30, 0x46, 0x00, 0xbe, 0x01, 0x3e,
	0x00, 0x2e, 0xfc, 0xd1, 0x70, 0x47, 0x00, 0x00, 0x40, 0x10, 0x00, 0x00,
	0x00, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00
};

/* see contrib/loaders/flash/milandr/mdr32f02fi_flash_write.c for src */
static const uint8_t mdr32f02fi_flash_write_code[] = {
#include "../../../contrib/loaders/flash/milandr/mdr32f02fi_flash_write.inc"
};

/* see contrib/loaders/flash/milandr/mdr1206afi_flash_write.c for src */
static const uint8_t mdr1206afi_flash_write_code[] = {
#include "../../../contrib/loaders/flash/milandr/mdr1206afi_flash_write.inc"
};

/* see contrib/loaders/flash/milandr/mdr1206fi_flash_write.c for src */
static const uint8_t mdr1206fi_flash_write_code[] = {
#include "../../../contrib/loaders/flash/milandr/mdr1206fi_flash_write.inc"
};

static int mdr_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count);

/* flash bank <name> mdr <base> <size> 0 0 <target#> <type> <bank_count> <sect_count> */
FLASH_BANK_COMMAND_HANDLER(mdr_flash_bank_command)
{
	struct mdr_flash_bank *mdr_info;

	if (CMD_ARGC < 9)
		return ERROR_COMMAND_SYNTAX_ERROR;

	mdr_info = malloc(sizeof(struct mdr_flash_bank));

	bank->driver_priv = mdr_info;
	mdr_info->probed = false;
	mdr_info->riscv = !strcmp(target_type_name(bank->target), "riscv");
	mdr_info->flash_base = MD_FLASH_REG_BASE;
	mdr_info->chip_id = 0;
	mdr_info->calib_offs = 0;
	mdr_info->calib_size = 0;
	memset(mdr_info->calib_values, 0xFF, sizeof(mdr_info->calib_values));

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[6], mdr_info->mem_type);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[7], mdr_info->bank_count);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[8], mdr_info->sect_count);

	mdr_info->page_size = (1 << 12); /* 4096B page size by default */

	if (!mdr_info->riscv) {
		/* ARM-based MCUs */
		mdr_info->per_clock = MD_PER_CLOCK;
		mdr_info->per_clock_flash_en = MD_PER_CLOCK_FLASH;
		mdr_info->per_clock_rst_clk = MD_PER_CLOCK_RST_CLK;
		mdr_info->ext_flags = 0;
		mdr_info->flash_write_code = mdr32fx_flash_write_code;
		mdr_info->flash_write_code_size = sizeof(mdr32fx_flash_write_code);
	} else {
		/* Default RISCV-based MCUs */
		mdr_info->per_clock = MD_PER2_CLOCK;
		mdr_info->per_clock_flash_en = MD_PER2_CLOCK_FLASH;
		mdr_info->per_clock_rst_clk = MD_PER2_CLOCK_RST_CLK;
		mdr_info->ext_flags = MD_FLASH_TMR;
		mdr_info->flash_write_code = mdr32f02fi_flash_write_code;
		mdr_info->flash_write_code_size = sizeof(mdr32f02fi_flash_write_code);

		/* Setup default timing parameters based on MDR32F02FI specification */
		mdr_info->t_nvs = FLASH_TNVS_US;
		mdr_info->t_erase = FLASH_TERASE_US;
		mdr_info->t_me = FLASH_TME_US;
		mdr_info->t_nvh = FLASH_TNVH_US;
		mdr_info->t_nvh1 = FLASH_TNVH1_US;
		mdr_info->t_prog = FLASH_TPROG_US;
		mdr_info->t_pgs = FLASH_TPGS_US;
		mdr_info->t_rcv = FLASH_TRCV_US;
	}

	return ERROR_OK;
}

static int mdr_flash_clock_enable(struct flash_bank* bank)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	int retval;
	uint32_t cur_per_clock;

	retval = target_read_u32(target, mdr_info->per_clock, &cur_per_clock);
	if (retval != ERROR_OK)
		return retval;

	if (!(cur_per_clock & mdr_info->per_clock_rst_clk)) {
		LOG_ERROR("Target needs reset before running flash operations");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	retval = target_write_u32(target, mdr_info->per_clock,
	                          cur_per_clock | mdr_info->per_clock_flash_en);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mdr_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	unsigned int bank_size = bank->size / mdr_info->bank_count;
	uint32_t flash_cmd;
	int retval;

	retval = target_read_u32(target, MD_FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < mdr_info->bank_count; i++) {
		if (!mdr_info->riscv || mdr_info->chip_id != 215) {
			retval = target_write_u32(target, MD_FLASH_ADR, i * bank_size);
			if (retval != ERROR_OK)
				return retval;

			flash_cmd |= MD_FLASH_XE | MD_FLASH_MAS1 | MD_FLASH_ERASE;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_NVS);

			flash_cmd |= MD_FLASH_NVSTR;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_ME);

			flash_cmd &= ~MD_FLASH_ERASE;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_NVH1);

			flash_cmd &= ~(MD_FLASH_XE | MD_FLASH_MAS1 | MD_FLASH_NVSTR);
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_RCV);
		} else {
			uint32_t address;
			address = i * (!mdr_info->mem_type ? BIT(18) : BIT(13));

			retval = target_write_u32(target, MD_FLASH_ADR, address);
			if (retval != ERROR_OK)
				return retval;

			flash_cmd |= MD_FLASH_CE_215 | MD_FLASH_CHIP_215 | MD_FLASH_ERASE_215;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_NVS);

			flash_cmd |= MD_FLASH_WE_215;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_ME);

			flash_cmd &= ~MD_FLASH_WE_215;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_NVH1);

			flash_cmd &= ~(MD_FLASH_CE_215 | MD_FLASH_CHIP_215 | MD_FLASH_ERASE_215);
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				return retval;

			jtag_sleep(MD_FLASH_T_RCV);
		}
	}

	for (unsigned int sect = 0; sect < mdr_info->sect_count; sect++) {
		bank->sectors[sect].is_erased = 1;
	}

	return retval;
}

static int mdr_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	int retval, retval2;
	uint32_t flash_cmd;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = mdr_flash_clock_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, MD_FLASH_KEY, FLASH_KEY_UNLOCK_KEY);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, MD_FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* Switch on register access */
	flash_cmd = (flash_cmd & MD_FLASH_DELAY_MASK) | MD_FLASH_CON | mdr_info->ext_flags;
	if (mdr_info->mem_type)
		flash_cmd |= MD_FLASH_IFREN;

	retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	if ((first == 0) && (last == (bank->num_sectors - 1)) &&
		!mdr_info->mem_type) {
		retval = mdr_mass_erase(bank);
		goto reset_pg_and_lock;
	}

	unsigned int sect_size = bank->size / mdr_info->sect_count;

	for (unsigned int sect = first; sect <= last; sect++) {
		if (!mdr_info->riscv || mdr_info->chip_id != 215) {
			retval = target_write_u32(target, MD_FLASH_ADR, sect * sect_size);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			flash_cmd |= MD_FLASH_XE | MD_FLASH_ERASE;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			jtag_sleep(MD_FLASH_T_NVS);

			flash_cmd |= MD_FLASH_NVSTR;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			jtag_sleep(MD_FLASH_T_ERASE);

			flash_cmd &= ~MD_FLASH_ERASE;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			jtag_sleep(MD_FLASH_T_NVH);

			flash_cmd &= ~(MD_FLASH_XE | MD_FLASH_NVSTR);
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			jtag_sleep(MD_FLASH_T_RCV);

			bank->sectors[sect].is_erased = 1;
		} else {
			for (unsigned int i = 0; i < 2; i++) {
				uint32_t address = (sect * sect_size) / 2;
				address |= i * (!mdr_info->mem_type ? BIT(18) : BIT(13));

				retval = target_write_u32(target, MD_FLASH_ADR, address);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				flash_cmd |= MD_FLASH_CE_215 | MD_FLASH_ERASE_215;
				retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				jtag_sleep(MD_FLASH_T_NVS);

				flash_cmd |= MD_FLASH_WE_215;
				retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				jtag_sleep(MD_FLASH_T_ERASE);

				flash_cmd &= ~MD_FLASH_WE_215;
				retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				jtag_sleep(MD_FLASH_T_RCV);

				flash_cmd &= ~(MD_FLASH_CE_215 | MD_FLASH_ERASE_215);
				retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				jtag_sleep(MD_FLASH_T_RCV);
			}
			bank->sectors[sect].is_erased = 1;
		}
	}

reset_pg_and_lock:
	flash_cmd &= MD_FLASH_DELAY_MASK;
	retval2 = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
	if (retval == ERROR_OK)
		retval = retval2;

	retval2 = target_write_u32(target, MD_FLASH_KEY, FLASH_KEY_LOCK_KEY);
	if (retval == ERROR_OK)
		retval = retval2;

	if (mdr_info->mem_type && (retval == ERROR_OK)
	    && (last == (bank->num_sectors - 1))
	    && mdr_info->calib_offs && mdr_info->calib_size
	    && mdr_info->riscv
	    && (mdr_info->chip_id == 215 || mdr_info->chip_id == 217)) {
		/* Restore calibration values in last sector of Boot/User flash memory */
		retval2 = mdr_write(bank, (const uint8_t*)mdr_info->calib_values,
				   bank->base + mdr_info->calib_offs,
				   mdr_info->calib_size);
		if (retval2) {
			LOG_WARNING("MDR32RV: failed to write calibration values in flash.");
			for (size_t i = 0; i < (mdr_info->calib_size + sizeof(uint32_t) - 1) / sizeof(uint32_t); ++i) {
				LOG_WARNING("MDR32RV: value @0x%08X = 0x%08X",
					  (uint32_t)(bank->base + mdr_info->calib_offs + i),
					  mdr_info->calib_values[i]);
			}
		} else {
			LOG_INFO("MDR32RV: restored erased calibration values in flash memory @0x%08X.",
				  (uint32_t)(bank->base + mdr_info->calib_offs));
		}
	}

	return retval;
}

static int mdr_write_block_hw(struct flash_bank *bank, const uint8_t *buffer,
			uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* flash write code */
	if (target_alloc_working_area(target, mdr_info->flash_write_code_size,
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			mdr_info->flash_write_code_size, mdr_info->flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	if (!mdr_info->riscv) {
		init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
		init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (32bit) */
		init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
		init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
		init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

		armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
		armv7m_info.core_mode = ARM_MODE_THREAD;

		buf_set_u32(reg_params[0].value, 0, 32, mdr_info->flash_base);
		buf_set_u32(reg_params[1].value, 0, 32, count);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
		buf_set_u32(reg_params[4].value, 0, 32, address);

		retval = target_run_flash_async_algorithm(target, buffer, count, 4,
				0, NULL,
				5, reg_params,
				source->address, source->size,
				write_algorithm->address, 0,
				!mdr_info->riscv ? &armv7m_info : NULL);

		if (retval == ERROR_FLASH_OPERATION_FAILED)
			LOG_ERROR("flash write failed at address 0x%"PRIx32,
					buf_get_u32(reg_params[4].value, 0, 32));
	} else {
		init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
		init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);	/* word_count (32bit) */
		init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);	/* buffer start */
		init_reg_param(&reg_params[3], "a3", 32, PARAM_OUT);	/* buffer end */
		init_reg_param(&reg_params[4], "a4", 32, PARAM_IN_OUT);	/* target address */

		while (count > 0) {
			size_t words_to_write = buffer_size/4 < count ? buffer_size/4 : count;

			retval = target_write_buffer(target,
					source->address,
					words_to_write * 4,
					buffer);
			if (retval != ERROR_OK) {
				LOG_ERROR("flash data buffer write failed at address 0x%"PRIx32,
						(uint32_t)source->address);
				break;
			}

			buf_set_u32(reg_params[0].value, 0, 32, mdr_info->flash_base);
			buf_set_u32(reg_params[1].value, 0, 32, words_to_write);
			buf_set_u32(reg_params[2].value, 0, 32, source->address);
			buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
			buf_set_u32(reg_params[4].value, 0, 32, address);

			LOG_DEBUG("MDR_RV: flash_base = 0x%"PRIx32", "
					  "word_count = 0x%"PRIx32", "
					  "start = 0x%"PRIx32", "
					  "end = 0x%"PRIx32", "
					  "address = 0x%"PRIx32,
					  buf_get_u32(reg_params[0].value, 0, 32),
					  buf_get_u32(reg_params[1].value, 0, 32),
					  buf_get_u32(reg_params[2].value, 0, 32),
					  buf_get_u32(reg_params[3].value, 0, 32),
					  buf_get_u32(reg_params[4].value, 0, 32));

			retval = target_run_algorithm(target,
				0, NULL, 5, reg_params,
				write_algorithm->address, 0,
				1000, NULL);

			LOG_DEBUG("MDR_RV: status = 0x%"PRIx32", "
					  "address = 0x%"PRIx32,
					  buf_get_u32(reg_params[0].value, 0, 32),
					  buf_get_u32(reg_params[4].value, 0, 32));

			if (retval != ERROR_OK) {
				LOG_ERROR("flash write failed at address 0x%"PRIx32,
						  buf_get_u32(reg_params[4].value, 0, 32));
				break;
			}

			address += words_to_write * 4;
			count   -= words_to_write;
			buffer  += words_to_write * 4;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int mdr_write_block_sw(struct flash_bank *bank, const uint8_t *buffer,
			uint32_t offset, uint32_t size)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	unsigned int sect_size = bank->size / bank->num_sectors;
	uint32_t flash_cmd;
	int retval;

	retval = target_read_u32(target, MD_FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto finish;

	while (size > 0) {
		unsigned int page_size = mdr_info->page_size;
		unsigned int page_mask = page_size - 1;
		unsigned int page_start = offset & ~page_mask;
		unsigned int page_write_size = page_start + page_size - offset;

		if (size < page_write_size)
			page_write_size = size;

		/* Latch MSB part of page address to be written in following cycle */
		retval = target_write_u32(target, MD_FLASH_ADR, offset);
		if (retval != ERROR_OK)
			goto finish;

		flash_cmd |= MD_FLASH_XE | MD_FLASH_PROG;
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_NVS);

		flash_cmd |= MD_FLASH_NVSTR;
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_PGS);

		for (unsigned int i = 0; i < page_write_size; i += 4) {
			/* Latch word address (LSB part) to be written */
			retval = target_write_u32(target, MD_FLASH_ADR, offset + i);
			if (retval != ERROR_OK)
				goto finish;

			uint32_t value = buf_get_u32(buffer + i, 0, 32);
			retval = target_write_u32(target, MD_FLASH_DI, value);
			if (retval != ERROR_OK)
				goto finish;

			flash_cmd |= MD_FLASH_YE;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto finish;

			jtag_sleep(MD_FLASH_T_PROG);

			flash_cmd &= ~MD_FLASH_YE;
			retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto finish;

			/* Wait for Tadh = 20 ns */
			jtag_sleep(1);

			bank->sectors[(offset + i) / sect_size].is_erased = 0;
		}

		buffer += page_write_size;
		offset += page_write_size;
		size   -= page_write_size;

		flash_cmd &= ~MD_FLASH_PROG;
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_NVH);

		flash_cmd &= ~(MD_FLASH_XE | MD_FLASH_NVSTR);
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_RCV);
	}
finish:
	return retval;
}

static uint32_t mdr_conv_addr_215(struct flash_bank *bank, uint32_t offset)
{
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	uint32_t address = (offset >> 1) & ~0x3;

	if (offset & BIT(2))
		address |= !mdr_info->mem_type ? BIT(18) : BIT(13);

	return address;
}

static int mdr_write_block_sw_215(struct flash_bank *bank, const uint8_t *buffer,
				uint32_t offset, uint32_t size)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	unsigned int sect_size = bank->size / bank->num_sectors;
	uint32_t flash_cmd;
	int retval;

	retval = target_read_u32(target, MD_FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto finish;

	while (size > 0) {
		unsigned int page_size = mdr_info->page_size;
		unsigned int page_mask = page_size - 1;
		unsigned int page_start = offset & ~page_mask;
		unsigned int page_write_size = page_start + page_size - offset;

		if (size < page_write_size)
			page_write_size = size;

		/* Latch MSB part of page address to be written in following cycle */
		retval = target_write_u32(target, MD_FLASH_ADR,
					mdr_conv_addr_215(bank, offset));
		if (retval != ERROR_OK)
			goto finish;

		flash_cmd |= MD_FLASH_CE_215 | MD_FLASH_PROG_215;
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_NVS);

		flash_cmd |= MD_FLASH_WE_215;
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_PGS);

		for (unsigned int i = 0; i < page_write_size; i += 4) {
			/* Latch word address (LSB part) to be written */
			retval = target_write_u32(target, MD_FLASH_ADR,
						mdr_conv_addr_215(bank, offset + i));
			if (retval != ERROR_OK)
				goto finish;

			uint32_t value = buf_get_u32(buffer + i, 0, 32);
			retval = target_write_u32(target, MD_FLASH_DI, value);
			if (retval != ERROR_OK)
				goto finish;

			for (unsigned int j = 0; j < 4; j++) {
				retval = target_write_u32(target, MD_FLASH_CTRL, BIT(j));
				if (retval != ERROR_OK)
					goto finish;

				/* Wait for Tads = 500 ns */
				jtag_sleep(1);

				flash_cmd |= MD_FLASH_PROG2_215;
				retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto finish;

				jtag_sleep(MD_FLASH_T_PROG);

				flash_cmd &= ~MD_FLASH_PROG2_215;
				retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto finish;

				/* Wait for Tadh = 500 ns */
				jtag_sleep(1);
			}

			bank->sectors[(offset + i) / sect_size].is_erased = 0;
		}

		buffer += page_write_size;
		offset += page_write_size;
		size   -= page_write_size;

		/* Wait for Tpgh = 500 ns */
		jtag_sleep(1);

		flash_cmd &= ~MD_FLASH_WE_215;
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		jtag_sleep(MD_FLASH_T_RCV);

		flash_cmd &= ~(MD_FLASH_CE_215 | MD_FLASH_PROG_215);
		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto finish;

		/* Wait for Trw = 500 ns */
		jtag_sleep(1);
	}
finish:
	return retval;
}

static int mdr_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	uint8_t *new_buffer = NULL;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */
	int rem = count % 4;
	if (rem) {
		new_buffer = malloc(count + rem);
		if (!new_buffer) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		while (rem--)
			new_buffer[count++] = 0xff;
	}

	/* Check if calibration values will be overwritten by requested write operation */
	if (mdr_info->mem_type
	    && mdr_info->calib_offs && mdr_info->calib_size
	    && mdr_info->riscv && (mdr_info->chip_id == 215 || mdr_info->chip_id == 217)
	    && (offset + count) > mdr_info->calib_offs) {
		if (! new_buffer) {
			new_buffer = malloc(count);
			if (!new_buffer) {
				LOG_ERROR("calibration bytes will be overwritten and no memory for padding buffer");
				return ERROR_FAIL;
			}
			buffer = memcpy(new_buffer, buffer, count);
		}
		size_t write_size = (offset + count) >= (mdr_info->calib_offs + mdr_info->calib_size) ?
			mdr_info->calib_size : (offset + count) - mdr_info->calib_offs;
		memcpy(new_buffer + mdr_info->calib_offs - offset,
		       mdr_info->calib_values,
		       write_size);
	}

	uint32_t flash_cmd;
	int retval, retval2;

	retval = mdr_flash_clock_enable(bank);
	if (retval != ERROR_OK)
		goto free_buffer;

	retval = target_write_u32(target, MD_FLASH_KEY, FLASH_KEY_UNLOCK_KEY);
	if (retval != ERROR_OK)
		goto free_buffer;

	retval = target_read_u32(target, MD_FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* Switch on register access */
	flash_cmd = (flash_cmd & MD_FLASH_DELAY_MASK) | MD_FLASH_CON | mdr_info->ext_flags;
	if (mdr_info->mem_type)
		flash_cmd |= MD_FLASH_IFREN;
	retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	if (mdr_info->riscv && (mdr_info->chip_id == 215 || mdr_info->chip_id == 217)) {
		retval = target_write_u32(target, MD_FLASH_CTRL, 0);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;
	}

	/* try using block write */
	retval = mdr_write_block_hw(bank, buffer, offset, count / 4);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("Can't use block writes, falling back to single memory accesses");

		if (!mdr_info->riscv || mdr_info->chip_id != 215)
			retval = mdr_write_block_sw(bank, buffer, offset, count);
		else
			retval = mdr_write_block_sw_215(bank, buffer, offset, count);
	}

reset_pg_and_lock:
	flash_cmd &= MD_FLASH_DELAY_MASK;
	retval2 = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
	if (retval == ERROR_OK)
		retval = retval2;

	retval2 = target_write_u32(target, MD_FLASH_KEY, FLASH_KEY_LOCK_KEY);
	if (retval == ERROR_OK)
		retval = retval2;

free_buffer:
	free(new_buffer);

	/* read some bytes bytes to flush buffer in flash accelerator.
	 * See errata for 1986VE1T and 1986VE3. Error 0007 */
	if ((retval == ERROR_OK) && (!mdr_info->mem_type)) {
		uint32_t tmp;
		target_checksum_memory(bank->target, bank->base, 64, &tmp);
	}

	return retval;
}

#if defined(USE_MDR_READ) && USE_MDR_READ
static int mdr_read(struct flash_bank *bank, uint8_t *buffer,
		    uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	int retval, retval2;

	if (!mdr_info->mem_type)
		return default_flash_read(bank, buffer, offset, count);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x3) {
		LOG_ERROR("count 0x%" PRIx32 " breaks required 4-byte alignment", count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	uint32_t flash_cmd;

	retval = mdr_flash_clock_enable(bank);
	if (retval != ERROR_OK)
		goto err;

	retval = target_write_u32(target, MD_FLASH_KEY, FLASH_KEY_UNLOCK_KEY);
	if (retval != ERROR_OK)
		goto err;

	retval = target_read_u32(target, MD_FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto err_lock;

	/* Switch on register access */
	flash_cmd = (flash_cmd & MD_FLASH_DELAY_MASK) | MD_FLASH_CON | mdr_info->ext_flags;
	if (mdr_info->mem_type)
		flash_cmd |= MD_FLASH_IFREN;
	retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	for (uint32_t i = 0; i < count; i += 4) {
		retval = target_write_u32(target, MD_FLASH_ADR, offset + i);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd |
					  MD_FLASH_XE | MD_FLASH_YE | MD_FLASH_SE);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		uint32_t buf;
		retval = target_read_u32(target, MD_FLASH_DO, &buf);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		buf_set_u32(buffer, i * 8, 32, buf);

		retval = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

	}

reset_pg_and_lock:
	flash_cmd &= MD_FLASH_DELAY_MASK;
	retval2 = target_write_u32(target, MD_FLASH_CMD, flash_cmd);
	if (retval == ERROR_OK)
		retval = retval2;

err_lock:
	retval2 = target_write_u32(target, MD_FLASH_KEY, FLASH_KEY_LOCK_KEY);
	if (retval == ERROR_OK)
		retval = retval2;

err:
	return retval;
}
#endif

static int mdr_probe(struct flash_bank *bank)
{
	int cur_debug_level = debug_level;
	int retval;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	struct mdr_flash_bank mdr_info_bckp = *mdr_info;
	unsigned int sect_count, sect_size;

	if (mdr_info->riscv) {
		/* Suppress error messages while trying to access invalid memory address
		 * while probing chip ID register */
		if (debug_level < LOG_LVL_DEBUG)
			debug_level = LOG_LVL_SILENT;

		mdr_info->flash_base = MDR1206_FLASH_REG_BASE;
		mdr_info->per_clock = MDR1206_PER2_CLOCK;
		mdr_info->per_clock_flash_en = MDR1206_PER2_CLOCK_FLASH;
		mdr_info->per_clock_rst_clk = MDR1206_PER2_CLOCK_RST_CLK;

		retval = mdr_flash_clock_enable(bank);
		if (retval == ERROR_OK)
			retval = target_read_u32(bank->target, MD_CHIP_ID_CTRL, &mdr_info->chip_id);

		/* Try to read/write MD_FLASH_KEY register to check it is accessible in current MCU */
		uint32_t flash_key = 0xDEADBEEF;
		if (retval == ERROR_OK)
			retval = target_write_u32(bank->target, MD_FLASH_KEY, 0xABADBABE);

		if (retval == ERROR_OK)
			retval = target_read_u32(bank->target, MD_FLASH_KEY, &flash_key);

		if (retval == ERROR_OK && flash_key != 0xABADBABE)
			retval = ERROR_FAIL;

		if (retval == ERROR_OK)
			retval = target_write_u32(bank->target, MD_FLASH_KEY, FLASH_KEY_LOCK_KEY);

		debug_level = cur_debug_level;

		if (retval) {
			LOG_INFO("MDR32RV: CHIP_ID_CTRL register is not accessible, using MDR32F02FI register map.");
			/* Restore saved default device context */
			*mdr_info = mdr_info_bckp;
		} else {
			mdr_info->chip_id = (mdr_info->chip_id >> 2) & 0xFF;
			const char *chip_name = NULL;
			switch (mdr_info->chip_id) {
			case 215:
				chip_name = "MDR1206AFI";
				break;
			case 217:
				chip_name = "MDR1206FI";
				break;
			default:
				chip_name = "Unknown chip";
				break;
			}
			LOG_INFO("MDR32RV: found %s with CHIP_ID = %d", chip_name, mdr_info->chip_id);
		}

		switch (mdr_info->chip_id) {
		case 0: /* Using default values */
			break;
		case 215: /* MDR1206AFI */
			if (!mdr_info->mem_type) {
				/* Main memory */
				bank->base = 0x10000000;
				bank->size = 512 * 1024;    /* 512 KB */
				mdr_info->sect_count = 512; /* 1 KB per sector */
				mdr_info->bank_count = 2;   /* 256 KB per bank */
				LOG_INFO("MDR32RV: setting flash bank type 0 size @0x%08X to %u KiB",
					 (unsigned int)bank->base, bank->size / 1024);
			} else {
				/* Boot/User memory */
				bank->base = 0x00020000;
				bank->size = 15 * 1024;    /* 15 KB */
				mdr_info->sect_count = 15; /* 1 KB per sector */
				mdr_info->bank_count = 2;  /* 15 KB per bank */
				mdr_info->calib_offs = 0x00003BE0;
				mdr_info->calib_size = 32; /* 28 calibration + 4 protection bytes */
				LOG_INFO("MDR32RV: setting flash bank type 1 size @0x%08X to %u KiB",
					 (unsigned int)bank->base, bank->size / 1024);
			}
			mdr_info->page_size = 128; /* 128B row (32 words) range for MDR1206AFI (ID 215) */
			mdr_info->ext_flags = MD_FLASH_TMEN_215;
			mdr_info->flash_write_code = mdr1206afi_flash_write_code;
			mdr_info->flash_write_code_size = sizeof(mdr1206afi_flash_write_code);

			/* Setup timing parameters based on MDR1206AFI (ID 215) specification */
			mdr_info->t_nvs = FLASH_TNVS_CHIP_ERASE_US;
			mdr_info->t_erase = FLASH_TERASE_SECTOR_ERASE_US;
			mdr_info->t_me = FLASH_TERASE_CHIP_ERASE_US;
			mdr_info->t_nvh = FLASH_TNVH_US;
			mdr_info->t_nvh1 = FLASH_TRCV_CHIP_ERASE_US;
			mdr_info->t_prog = FLASH_TPROG_PROGRAM_US;
			mdr_info->t_pgs = FLASH_TPGS_PROGRAM_US;
			mdr_info->t_rcv = FLASH_TRCV_PROGRAM_US;
			break;
		case 217: /* MDR1206FI */
			if (!mdr_info->mem_type) {
				/* Main memory */
				bank->base = 0x10000000;
				bank->size = 512 * 1024;    /* 512 KB */
				mdr_info->sect_count = 128; /* 4 KB per sector */
				mdr_info->bank_count = 2;   /* 256 KB per bank */
				LOG_INFO("MDR32RV: setting flash bank type 0 size @0x%08X to %u KiB",
					 (unsigned int)bank->base, bank->size / 1024);
			} else {
				/* Boot/User memory */
				bank->base = 0x00020000;
				bank->size = 16 * 1024;   /* 16 KB */
				mdr_info->sect_count = 4; /* 4 KB per sector */
				mdr_info->bank_count = 2; /* 8 KB per bank */
				mdr_info->calib_offs = 0x00003FE0;
				mdr_info->calib_size = 32; /* 28 calibration + 4 protection bytes */
				LOG_INFO("MDR32RV: setting flash bank type 1 size @0x%08X to %u KiB",
					 (unsigned int)bank->base, bank->size / 1024);
			}
			mdr_info->page_size = (1 << 9); /* YADR[8:2] (512B) range for MDR1206FI (ID 217) */
			mdr_info->ext_flags = MD_FLASH_TMR;
			mdr_info->flash_write_code = mdr1206fi_flash_write_code;
			mdr_info->flash_write_code_size = sizeof(mdr1206fi_flash_write_code);

			/* Setup timing parameters based on MDR1206FI (ID 217) specification */
			mdr_info->t_nvs = FLASH_TNVS_US;
			mdr_info->t_erase = FLASH_TERASE_US;
			mdr_info->t_me = FLASH_TME_US;
			mdr_info->t_nvh = FLASH_TNVH_US;
			mdr_info->t_nvh1 = FLASH_TNVH1_US;
			mdr_info->t_prog = FLASH_TPROG_US;
			mdr_info->t_pgs = FLASH_TPGS_US;
			mdr_info->t_rcv = FLASH_TRCV_US;
			break;
		default:
			LOG_INFO("MDR32RV: CHIP_ID = %d is unsupported yet.", mdr_info->chip_id);
			return ERROR_TARGET_INVALID;
		}
	}

	if (mdr_info->calib_offs) {
		retval = target_read_memory(bank->target,
				       bank->base + mdr_info->calib_offs,
				       1, mdr_info->calib_size,
				       (uint8_t*)mdr_info->calib_values);
		if (retval) {
			LOG_INFO("MDR32RV: failed to read CHIP_IDE = %d calibration values @0x%08X, size %u",
				 mdr_info->chip_id,
				 (uint32_t)(bank->base + mdr_info->calib_offs),
				 (unsigned)mdr_info->calib_size);
			return ERROR_TARGET_INVALID;
		}
		if (LOG_LEVEL_IS(LOG_LVL_DEBUG)) {
			LOG_DEBUG("Calibration base address = 0x%08X, size = %u",
				  (uint32_t)(bank->base + mdr_info->calib_offs),
				  (unsigned)mdr_info->calib_size);
			for (size_t i = 0; i < (mdr_info->calib_size + sizeof(uint32_t) - 1) / sizeof(uint32_t); ++i) {
				LOG_DEBUG("calib_value @0x%08X = 0x%08X",
					  (uint32_t)(bank->base + mdr_info->calib_offs + i),
					  mdr_info->calib_values[i]);
			}
		}
	}

	sect_count = mdr_info->sect_count;
	sect_size = bank->size / sect_count;

	free(bank->sectors);

	bank->num_sectors = sect_count;
	bank->sectors = alloc_block_array(0, sect_size, sect_count);
	if (!bank->sectors)
		return ERROR_FAIL;

	mdr_info->probed = true;

	return ERROR_OK;
}

static int mdr_auto_probe(struct flash_bank *bank)
{
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	if (mdr_info->probed)
		return ERROR_OK;
	return mdr_probe(bank);
}

static int get_mdr_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	command_print_sameline(cmd, "%s - %s",
			!mdr_info->riscv ? "MDR32Fx" : "MDR32RV",
			mdr_info->mem_type ? "info memory" : "main memory");

	return ERROR_OK;
}

const struct flash_driver mdr_flash = {
	.name = "mdr",
	.usage = "flash bank <name> mdr <base> <size> 0 0 <target#> <type> <bank_count> <sect_count>\n"
	"<type>: 0 for main memory, 1 for info memory\n"
	"<bank_count>: overall banks count to be erased separately during mass erase\n"
	"<sect_count>: minimum sized erase units count (erase sectors)",
	.flash_bank_command = mdr_flash_bank_command,
	.erase = mdr_erase,
	.write = mdr_write,
#if defined(USE_MDR_READ) && USE_MDR_READ
	.read = mdr_read,
#else
	.read = default_flash_read,
#endif
	.probe = mdr_probe,
	.auto_probe = mdr_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = get_mdr_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
