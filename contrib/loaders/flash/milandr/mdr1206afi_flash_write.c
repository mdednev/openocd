// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Max A. Dednev                                   *
 *   mdednev@yandex.ru                                                    *
 ***************************************************************************/

#include <stdint.h>

#ifndef CPU_FREQ_HZ
#define CPU_FREQ_HZ (8 * 1000 * 1000)
#endif

#ifndef __IO
#define __IO    volatile
#endif

typedef struct {
	__IO uint32_t CMD;          /*!< command register - 0x00 */
	__IO uint32_t ADR;          /*!< address register - 0x04 */
	__IO uint32_t DI;           /*!< input data register - 0x08 */
	__IO uint32_t DO;           /*!< output data register - 0x0C */
	__IO uint32_t KEY;          /*!< key register - 0x10 */
	__IO uint32_t CTRL;         /*!< control - 0x14 */
	__IO uint32_t CHIP_ID_CTRL; /*!< chip ID register - 0x18 */
} MDR_FLASH_CTRL_TYPEDEF;

/* protection key */
#define FLASH_KEY_UNLOCK_KEY		(0x8AAA5551U)
#define FLASH_KEY_LOCK_KEY		(0x0U)

/*******************  Bit definition for FLASH_CMD register *******************/
/* FLASH control (0 - normal mode, 1 - flash programming mode) */
#define FLASH_CMD_CON_Pos              (0U)
#define FLASH_CMD_CON_Msk              (0x1U << FLASH_CMD_CON_Pos)
#define FLASH_CMD_CON_NORMAL           (0x0U << FLASH_CMD_CON_Pos)
#define FLASH_CMD_CON_PROGRAMMING      (0x1U << FLASH_CMD_CON_Pos)
/* address [17:2] feeding enable (0 - disabled, 1 - enabled) */
#define FLASH_CMD_CE_Pos               (6U)
#define FLASH_CMD_CE_Msk               (0x1U << FLASH_CMD_CE_Pos)
#define FLASH_CMD_CE                   FLASH_CMD_CE_Msk
/* enable read/write operations (0 - disabled, 1 - enabled) */
#define FLASH_CMD_WE_Pos               (7U)
#define FLASH_CMD_WE_Msk               (0x1U << FLASH_CMD_WE_Pos)
#define FLASH_CMD_WE                   FLASH_CMD_WE_Msk
/* readout apmlifier (0 - disabled, 1 - enabled) */
#define FLASH_CMD_RE_Pos               (8U)
#define FLASH_CMD_RE_Msk               (0x1U << FLASH_CMD_RE_Pos)
#define FLASH_CMD_RE                   FLASH_CMD_RE_Msk
/* information block selection (0 - main block, 1 - information block) */
#define FLASH_CMD_NVR_Pos              (9U)
#define FLASH_CMD_NVR_Msk              (0x1U << FLASH_CMD_NVR_Pos)
#define FLASH_CMD_NVR                  FLASH_CMD_NVR_Msk
/* erase page (0 - do not erase, 1 - erase) (lower part of the address [8:2] do no sense here) */
#define FLASH_CMD_ERASE_Pos            (10U)
#define FLASH_CMD_ERASE_Msk            (0x1U << FLASH_CMD_ERASE_Pos)
#define FLASH_CMD_ERASE                FLASH_CMD_ERASE_Msk
/* mass erase whole memory (0 - do not erase, 1 - erase) */
#define FLASH_CMD_CHIP_Pos             (11U)
#define FLASH_CMD_CHIP_Msk             (0x1U << FLASH_CMD_CHIP_Pos)
#define FLASH_CMD_CHIP                 FLASH_CMD_CHIP_Msk
/* write data from FLASH_DI to 32 word rowj(0 - do not write, 1 - write) */
#define FLASH_CMD_PROG_Pos             (12U)
#define FLASH_CMD_PROG_Msk             (0x1U << FLASH_CMD_PROG_Pos)
#define FLASH_CMD_PROG                 FLASH_CMD_PROG_Msk
/* write data from FLASH_DI to address [17:2], (0 - do not write, 1 - write) */
#define FLASH_CMD_PROG2_Pos            (13U)
#define FLASH_CMD_PROG2_Msk            (0x1U << FLASH_CMD_PROG2_Pos)
#define FLASH_CMD_PROG2                FLASH_CMD_PROG2_Msk
/* FLASH test mode (0 - test enable, 1 - no test) */
#define FLASH_CMD_TMEN_Pos             (14U)
#define FLASH_CMD_TMEN_Msk             (0x1U << FLASH_CMD_TMEN_Pos)
#define FLASH_CMD_TMEN                 FLASH_CMD_TMR_Msk

/* FLASH parameters with a safe margin of ~7% (HSI spread). */
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

#define time_after(a, b) ((int32_t)(b) - (int32_t)(a) < 0)

__attribute__((always_inline))
static inline void delay_cycles(uint32_t cycles)
{
	register uint32_t last_cycle;
	asm volatile ("csrr %0, mcycle" : "=r" (last_cycle));
	last_cycle += cycles;

	while (1) {
		register uint32_t mcycle;
		asm volatile ("csrr %0, mcycle" : "=r" (mcycle));
		if (time_after(mcycle, last_cycle))
			break;
	}
}

#define delay_us(us) delay_cycles((CPU_FREQ_HZ / 1000 / 1000) * (us))

static uint32_t convert_address(int mem_type, uint32_t offset)
{
	uint32_t address = (offset >> 1) & ~0x3;

	if (offset & (1UL << 2))
		address |= !mem_type ? (1UL << 18) : (1UL << 13);

	return address;
}

__attribute__((naked))
void flash_write(uint32_t  flash_base,
		 uint32_t  word_count,
		 uint8_t   *buffer_start,
		 uint8_t   *buffer_end,
		 uint32_t  target_address);

void flash_write(uint32_t  flash_base,
		 uint32_t  word_count,
		 uint8_t   *buffer_start,
		 uint8_t   *buffer_end,
		 uint32_t  target_address)
{
	/* NOTE: Flash programming was unlocked by caller */
	MDR_FLASH_CTRL_TYPEDEF * const MDR_FLASH = (void *)flash_base;
	register uint32_t cmd = MDR_FLASH->CMD;
	register int mem_type = MDR_FLASH->CMD & FLASH_CMD_NVR;

	while (word_count > 0) {
		const unsigned int    page_size = 128; // 128B range for MDR1206AFI (ID 215)
		register unsigned int i, j;
		register unsigned int page_mask = page_size - 1;
		register unsigned int page_start = target_address & ~page_mask;
		register unsigned int page_write_size = (page_start + page_size - target_address) / 4;

		if (word_count < page_write_size)
			page_write_size = word_count;

		/* Latch MSB part of page address to be written in following cycle */
		MDR_FLASH->ADR = convert_address(mem_type, target_address);

		MDR_FLASH->CMD = cmd
			| FLASH_CMD_CE
			| FLASH_CMD_PROG;

		delay_us(FLASH_TNVS_PROGRAM_US);

		MDR_FLASH->CMD = cmd
			| FLASH_CMD_CE
			| FLASH_CMD_PROG
			| FLASH_CMD_WE;

		delay_us(FLASH_TPGS_PROGRAM_US);

		for (i = 0; i < page_write_size; i++) {
			/* Latch word address (LSB part) to be written */
			MDR_FLASH->ADR = convert_address(mem_type, target_address + i * 4);
			/* Latch data to be written */
			MDR_FLASH->DI  = *((uint32_t *)buffer_start + i);

#pragma GCC unroll 1
			for (j = 0; j < 4; j++) {
				MDR_FLASH->CTRL = (1UL << j);

				delay_us(FLASH_TADS_PROGRAM_US);

				MDR_FLASH->CMD = cmd
					| FLASH_CMD_CE
					| FLASH_CMD_PROG
					| FLASH_CMD_WE
					| FLASH_CMD_PROG2;

				delay_us(FLASH_TPROG_PROGRAM_US);

				MDR_FLASH->CMD = cmd
					| FLASH_CMD_CE
					| FLASH_CMD_PROG
					| FLASH_CMD_WE;

				delay_us(FLASH_TADH_PROGRAM_US);
			}

			delay_us(FLASH_TPGH_PROGRAM_US);
		}

		target_address += page_write_size * 4;
		buffer_start   += page_write_size * 4;
		word_count     -= page_write_size;

		MDR_FLASH->CMD = cmd
			| FLASH_CMD_CE
			| FLASH_CMD_PROG;

		delay_us(FLASH_TRCV_PROGRAM_US);

		MDR_FLASH->CMD = cmd;

		delay_us(FLASH_TRW_PROGRAM_US);
	}

	/* Succeeded */
	asm volatile ("li a0, 0");
	asm volatile ("mv a4, %0" : : "r" (target_address));

	asm volatile ("ebreak");
}
