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
	__IO uint32_t CMD; /*!< control register - 0x00 */
	__IO uint32_t ADR; /*!< address register - 0x04 */
	__IO uint32_t DI;  /*!< input data register - 0x08 */
	__IO uint32_t DO;  /*!< output data register - 0x0C */
	__IO uint32_t KEY; /*!< key register - 0x10 */
} MDR_EEPROM_CTRL_TYPEDEF;

/* protection key */
#define EEPROM_KEY_UNLOCK_KEY			(0x8AAA5551U)
#define EEPROM_KEY_LOCK_KEY			(0x0U)

/*******************  Bit definition for FLASH_CMD register *******************/
/* FLASH control (0 - normal mode, 1 - flash programming mode) */
#define EEPROM_CMD_CON_POS              (0U)
#define EEPROM_CMD_CON_MSK              (0x1U << EEPROM_CMD_CON_POS)            /*!< 0x00000001 */
#define EEPROM_CMD_CON_NORMAL           (0x0U << EEPROM_CMD_CON_POS)            /*!< 0x00000000 */
#define EEPROM_CMD_CON_PROGRAMMING      (0x1U << EEPROM_CMD_CON_POS)            /*!< 0x00000001 */
/* FLASH delay (maximum speed of the FLASH is 30 MHz, use delay = 1 for greater core speed) */
#define EEPROM_CMD_DELAY_POS            (3U)
#define EEPROM_CMD_DELAY_MSK            (0x7U << EEPROM_CMD_DELAY_POS)          /*!< 0x00000038 */
#define EEPROM_CMD_DELAY_0_CYCLE        (0x0U << EEPROM_CMD_DELAY_POS)          /*!< 0x00000000 */
#define EEPROM_CMD_DELAY_1_CYCLE        (0x1U << EEPROM_CMD_DELAY_POS)          /*!< 0x00000008 */
/* upper part of the address [17:9] feeding enable (0 - disabled, 1 - enabled) */
#define EEPROM_CMD_XE_POS               (6U)
#define EEPROM_CMD_XE_MSK               (0x1U << EEPROM_CMD_XE_POS)             /*!< 0x00000040 */
#define EEPROM_CMD_XE                   EEPROM_CMD_XE_MSK
/* lower part of the address [8:2] feeding enable (0 - disabled, 1 - enabled) */
#define EEPROM_CMD_YE_POS               (7U)
#define EEPROM_CMD_YE_MSK               (0x1U << EEPROM_CMD_YE_POS)             /*!< 0x00000080 */
#define EEPROM_CMD_YE                   EEPROM_CMD_YE_MSK
/* readout apmlifier (0 - disabled, 1 - enabled) */
#define EEPROM_CMD_SE_POS               (8U)
#define EEPROM_CMD_SE_MSK               (0x1U << EEPROM_CMD_SE_POS)             /*!< 0x00000100 */
#define EEPROM_CMD_SE                   EEPROM_CMD_SE_MSK
/* information block selection (0 - main block, 1 - information block) */
#define EEPROM_CMD_IFREN_POS            (9U)
#define EEPROM_CMD_IFREN_MSK            (0x1U << EEPROM_CMD_IFREN_POS)          /*!< 0x00000200 */
#define EEPROM_CMD_IFREN                EEPROM_CMD_IFREN_MSK
/* erase page (0 - do not erase, 1 - erase) (lower part of the address [8:2] do no sence here) */
#define EEPROM_CMD_ERASE_POS            (10U)
#define EEPROM_CMD_ERASE_MSK            (0x1U << EEPROM_CMD_ERASE_POS)          /*!< 0x00000400 */
#define EEPROM_CMD_ERASE                EEPROM_CMD_ERASE_MSK
/* mass erase whole memory (0 - do not erase, 1 - erase) */
#define EEPROM_CMD_MAS1_POS             (11U)
#define EEPROM_CMD_MAS1_MSK             (0x1U << EEPROM_CMD_MAS1_POS)           /*!< 0x00000800 */
#define EEPROM_CMD_MAS1                 EEPROM_CMD_MAS1_MSK
/* write data from EEPROM_DI to address [17:2], (0 - do not write, 1 - write) */
#define EEPROM_CMD_PROG_POS             (12U)
#define EEPROM_CMD_PROG_MSK             (0x1U << EEPROM_CMD_PROG_POS)           /*!< 0x00001000 */
#define EEPROM_CMD_PROG                 EEPROM_CMD_PROG_MSK
/* write / read operation (0 - read, 1 - write/erase) */
#define EEPROM_CMD_NVSTR_POS            (13U)
#define EEPROM_CMD_NVSTR_MSK            (0x1U << EEPROM_CMD_NVSTR_POS)          /*!< 0x00002000 */
#define EEPROM_CMD_NVSTR                EEPROM_CMD_NVSTR_MSK
/* EEPROM test mode (0 - test enable, 1 - no test) */
#define EEPROM_CMD_TMR_POS              (14U)
#define EEPROM_CMD_TMR_MSK              (0x1U << EEPROM_CMD_TMR_POS)            /*!< 0x00004000 */
#define EEPROM_CMD_TMR                  EEPROM_CMD_TMR_MSK

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

	MDR_EEPROM_CTRL_TYPEDEF * const MDR_EEPROM = (void *)flash_base;
	register uint32_t cmd = MDR_EEPROM->CMD;

	while (word_count--) {
		MDR_EEPROM->ADR = target_address;
		MDR_EEPROM->DI  = *((uint32_t *)buffer_start);

		MDR_EEPROM->CMD = cmd
			| EEPROM_CMD_XE
			| EEPROM_CMD_PROG;

		/* Tnvs delay 5 uS */
		delay_us(5);

		MDR_EEPROM->CMD = cmd
			| EEPROM_CMD_XE
			| EEPROM_CMD_PROG
			| EEPROM_CMD_NVSTR;

		/* Tpgs delay 10 uS */
		delay_us(10);

		MDR_EEPROM->CMD = cmd
			| EEPROM_CMD_XE
			| EEPROM_CMD_PROG
			| EEPROM_CMD_NVSTR
			| EEPROM_CMD_YE;

		/* Tprog delay 40 uS */
		delay_us(40);

		MDR_EEPROM->CMD = cmd
			| EEPROM_CMD_XE
			| EEPROM_CMD_PROG
			| EEPROM_CMD_NVSTR;

		target_address += 4;
		buffer_start   += 4;

		/* Tpgh delay 20 ns */
		delay_us(0);

		MDR_EEPROM->CMD = cmd
			| EEPROM_CMD_XE
			| EEPROM_CMD_NVSTR;

		/* Tnvh delay 5 uS */
		delay_us(5);

		MDR_EEPROM->CMD = cmd;
	}

	/* Trcv delay 10 uS */
	delay_us(10);

	/* Succeeded */
	asm volatile ("li a0, 0");
	asm volatile ("mv a4, %0" : : "r" (target_address));

	asm volatile ("ebreak");
}
