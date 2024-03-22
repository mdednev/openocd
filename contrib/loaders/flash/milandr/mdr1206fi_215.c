// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023 by Max A. Dednev                                   *
 *   m.dednev@yandex.ru                                                    *
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
#define FLASH_CMD_CON_POS              (0U)
#define FLASH_CMD_CON_MSK              (0x1U << FLASH_CMD_CON_POS)            /*!< 0x00000001 */
#define FLASH_CMD_CON_NORMAL           (0x0U << FLASH_CMD_CON_POS)            /*!< 0x00000000 */
#define FLASH_CMD_CON_PROGRAMMING      (0x1U << FLASH_CMD_CON_POS)            /*!< 0x00000001 */
/* address [17:2] feeding enable (0 - disabled, 1 - enabled) */
#define FLASH_CMD_CE_POS               (6U)
#define FLASH_CMD_CE_MSK               (0x1U << FLASH_CMD_CE_POS)             /*!< 0x00000040 */
#define FLASH_CMD_CE                   FLASH_CMD_CE_MSK
/* enable read/write operations (0 - disabled, 1 - enabled) */
#define FLASH_CMD_WE_POS               (7U)
#define FLASH_CMD_WE_MSK               (0x1U << FLASH_CMD_WE_POS)             /*!< 0x00000080 */
#define FLASH_CMD_WE                   FLASH_CMD_WE_MSK
/* readout apmlifier (0 - disabled, 1 - enabled) */
#define FLASH_CMD_RE_POS               (8U)
#define FLASH_CMD_RE_MSK               (0x1U << FLASH_CMD_RE_POS)             /*!< 0x00000100 */
#define FLASH_CMD_RE                   FLASH_CMD_RE_MSK
/* information block selection (0 - main block, 1 - information block) */
#define FLASH_CMD_NVR_POS              (9U)
#define FLASH_CMD_NVR_MSK              (0x1U << FLASH_CMD_NVR_POS)            /*!< 0x00000200 */
#define FLASH_CMD_NVR                  FLASH_CMD_NVR_MSK
/* erase page (0 - do not erase, 1 - erase) (lower part of the address [8:2] do no sence here) */
#define FLASH_CMD_ERASE_POS            (10U)
#define FLASH_CMD_ERASE_MSK            (0x1U << FLASH_CMD_ERASE_POS)          /*!< 0x00000400 */
#define FLASH_CMD_ERASE                FLASH_CMD_ERASE_MSK
/* mass erase whole memory (0 - do not erase, 1 - erase) */
#define FLASH_CMD_CHIP_POS             (11U)
#define FLASH_CMD_CHIP_MSK             (0x1U << FLASH_CMD_CHIP_POS)           /*!< 0x00000800 */
#define FLASH_CMD_CHIP                 FLASH_CMD_CHIP_MSK
/* write data from FLASH_DI to 32 word rowj(0 - do not write, 1 - write) */
#define FLASH_CMD_PROG_POS             (12U)
#define FLASH_CMD_PROG_MSK             (0x1U << FLASH_CMD_PROG_POS)           /*!< 0x00001000 */
#define FLASH_CMD_PROG                 FLASH_CMD_PROG_MSK
/* write data from FLASH_DI to address [17:2], (0 - do not write, 1 - write) */
#define FLASH_CMD_PROG2_POS            (13U)
#define FLASH_CMD_PROG2_MSK            (0x1U << FLASH_CMD_PROG2_POS)          /*!< 0x00002000 */
#define FLASH_CMD_PROG2                FLASH_CMD_PROG2_MSK
/* FLASH test mode (0 - test enable, 1 - no test) */
#define FLASH_CMD_TMEN_POS             (14U)
#define FLASH_CMD_TMEN_MSK             (0x1U << FLASH_CMD_TMEN_POS)           /*!< 0x00004000 */
#define FLASH_CMD_TMEN                 FLASH_CMD_TMR_MSK

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
void write_flash(uint32_t  flash_base,
		 uint32_t  word_count,
		 uint8_t   *buffer_start,
		 uint8_t   *buffer_end,
		 uint32_t  target_address);

void write_flash(uint32_t  flash_base,
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
		const unsigned int    page_size = 128; // 128B range for MDR1206FI ID 215
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

		/* Tnvs delay 20 uS */
		delay_us(20);

		MDR_FLASH->CMD = cmd
			| FLASH_CMD_CE
			| FLASH_CMD_PROG
			| FLASH_CMD_WE;

		/* Tpgs delay 70 uS */
		delay_us(70);

		for (i = 0; i < page_write_size; i++) {
			/* Latch word address (LSB part) to be written */
			MDR_FLASH->ADR = convert_address(mem_type, target_address + i * 4);
			/* Latch data to be written */
			MDR_FLASH->DI  = *((uint32_t *)buffer_start + i);

#pragma GCC unroll 1
			for (j = 0; j < 4; j++) {
				MDR_FLASH->CTRL = (1UL << j);

				/* Tads delay 500 ns */
				delay_us(1);

				MDR_FLASH->CMD = cmd
					| FLASH_CMD_CE
					| FLASH_CMD_PROG
					| FLASH_CMD_WE
					| FLASH_CMD_PROG2;

				/* Tprog delay 7 uS */
				delay_us(7);

				MDR_FLASH->CMD = cmd
					| FLASH_CMD_CE
					| FLASH_CMD_PROG
					| FLASH_CMD_WE;

				/* Tadh delay 500 ns */
				delay_us(1);
			}

			/* Tpgh delay 500 ns */
			delay_us(1);
		}

		target_address += page_write_size * 4;
		buffer_start   += page_write_size * 4;
		word_count     -= page_write_size;

		MDR_FLASH->CMD = cmd
			| FLASH_CMD_CE
			| FLASH_CMD_PROG;

		/* Trcv delay 50 uS */
		delay_us(50);

		MDR_FLASH->CMD = cmd;

		/* Trw delay 1 ns */
		delay_us(1);
	}

	/* Succeeded */
	asm volatile ("li a0, 0");
	asm volatile ("mv a4, %0" : : "r" (target_address));

	asm volatile ("ebreak");
}
