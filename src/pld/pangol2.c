// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023 by Max A. Dednev                                   *
 *   mdednev@yandex.ru                                                     *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "pangol2.h"
#include "pld.h"

/* IR commands from PG2L25H_MBG325.bsm Pango BSLD file */
#define PG2L_IDCODE          (0b1010000011)
#define PG2L_ISC_ENABLE      (0b0101000000)
#define PG2L_CFGI            (0b1010001011)
#define PG2L_EXTEST_TRAIN    (0b1001000001)
#define PG2L_BYPASS          (0b1111111111)
#define PG2L_USERCODE        (0b1010000100)
#define PG2L_EXTEST_PULSE    (0b1001000000)
#define PG2L_PRELOAD         (0b1010000000)
#define PG2L_SAMPLE          (0b1010000000)
#define PG2L_HIGHZ           (0b1010000101)
#define PG2L_EXTEST          (0b1010000001)
#define PG2L_ISC_PROGRAM     (0b0101000011)
#define PG2L_ISC_READ        (0b0101000100)
#define PG2L_ISC_DISABLE     (0b0101000001)
#define PG2L_JDRP            (0b1010001111)
#define PG2L_ISC_NOOP        (0b0101000010)

/* Undocumented commands from Verilog simulation model */
#define PG2L_CFGO            (0b1010001100)
#define PG2L_RDSR            (0b0101011001)
#define PG2L_PROGRAM_KEY     (0b0101000101)
#define PG2L_READ_KEY        (0b0101000110)
#define PG2L_PROGRAM_KEYLOCK (0b0101000111)
#define PG2L_READ_KEYLOCK    (0b0101001000)
#define PG2L_PROGRAM_FUSE    (0b0101001001)
#define PG2L_READ_FUSE       (0b0101001010)
#define PG2L_PROGRAM_UID     (0b0101001011)
#define PG2L_READ_UID        (0b0101001100)
#define PG2L_JRST            (0b1010001010)
#define PG2L_JWAKEUP         (0b1010001101)
#define PG2L_JWAKEDOWN       (0b1010001110)
#define PG2L_PROGRAM_FLASH   (0b0101001111)
#define PG2L_USER1           (0b1010000110) /* Access user-defined register 1 */
#define PG2L_USER2           (0b1010000111) /* Access user-defined register 2 */
#define PG2L_USER3           (0b1010001000) /* Access user-defined register 3 */
#define PG2L_USER4           (0b1010001001) /* Access user-defined register 4 */

/* First configuration bitstream word */
#define PG2L_SYNCH_WORD   (0x01332D94)

/* Command operation codes */
#define PG2L_NOP   (0)
#define PG2L_WRITE (1)
#define PG2L_READ  (2)

/* Register addresses */
#define PG2L_CRC        (0b00000) /* R/W, CRC register */
#define PG2L_IDR        (0b00001) /* R/W, Device Identification Register */
#define PG2L_CMDR       (0b00010) /* R/W, Command register */
#define PG2L_CTRL0R     (0b00011) /* R/W, Control register 0 */
#define PG2L_CTRL1R     (0b00100) /* R/W, Control register 1 */
#define PG2L_CMEMIR     (0b00101) /* W,   Frame Data Input Register */
#define PG2L_MFWRITER   (0b00110) /* W,   Multi-frame write register */
#define PG2L_CMEMOR     (0b00111) /* R,   Frame Data Output Register */
#define PG2L_IVR        (0b01000) /* W,   Initial vector register */
#define PG2L_STATUSR    (0b01001) /* R,   Status register */
#define PG2L_CHAINR     (0b01010) /* W,   Cascade register */
#define PG2L_ADRR       (0b01011) /* R/W, Frame address register */
#define PG2L_SBPIR      (0b01100) /* R/W, SBPI register */
#define PG2L_SEUR       (0b01101) /* R/W, SEUs control register */
#define PG2L_SEUSTATUSR (0b01110) /* R,   SEUs status register */
#define PG2L_IRSTCTRLR  (0b01111) /* R/W, Warm Start Control Register */
#define PG2L_IRSTADRR   (0b10000) /* R/W, Warm Start Address Register */
#define PG2L_WATCHDOGR  (0b10001) /* R/W, Watchdog register */
#define PG2L_HSTATUSR   (0b10010) /* R,   History Status Register */
#define PG2L_CMASKR     (0b10111) /* R/W, Control mask register */
#define PG2L_OPTION0R   (0b11001) /* R/W, OPTION REGISTER 0 */
#define PG2L_OPTION1R   (0b11010) /* R/W, OPTION REGISTER 1 */
#define PG2L_SEUADDR    (0b11101) /* R,   SEUs frame address register */
#define PG2L_SEUN_ADDR  (0b11111) /* R,   SEUs next frame address register */

#define PG2L_PKT_TYPE1(op, reg, cnt) \
	(((0b101)            << 29) | \
	 (((op)  &     0b11) << 27) | \
	 (((reg) &  0b11111) << 22) | \
	 (((cnt) & 0x3FFFFF) <<  0))

#define PG2L_PKT_TYPE2(op, cnt) \
	(((0b010)             << 29) | \
	 (((op)  &      0b11) << 27) | \
	 (((cnt) & 0x7FFFFFF) <<  0))

static int pangol2_set_instr(struct jtag_tap *tap, uint32_t new_instr)
{
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr) {
		struct scan_field field;

		field.num_bits = tap->ir_length;
		void *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);

		free(t);
	}

	return ERROR_OK;
}

static int pangol2_send_32(struct pld_device *pld_device,
	int num_words, uint32_t *words)
{
	struct pangol2_pld_device *pangol2_info = pld_device->driver_priv;
	struct scan_field scan_field;
	uint8_t *values;
	int i;

	values = malloc(num_words * 4);

	scan_field.num_bits = num_words * 32;
	scan_field.out_value = values;
	scan_field.in_value = NULL;

	for (i = 0; i < num_words; i++)
		buf_set_u32(values + 4 * i, 0, 32, flip_u32(*words++, 32));

	pangol2_set_instr(pangol2_info->tap, PG2L_CFGI);

	jtag_add_dr_scan(pangol2_info->tap, 1, &scan_field, TAP_DRPAUSE);

	free(values);

	return ERROR_OK;
}

static inline void pangoflip32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = flip_u32(le_to_h_u32(in), 32);
}

static int pangol2_receive_32(struct pld_device *pld_device,
	int num_words, uint32_t *words)
{
	struct pangol2_pld_device *pangol2_info = pld_device->driver_priv;
	struct scan_field scan_field;

	scan_field.num_bits = 32;
	scan_field.out_value = NULL;
	scan_field.in_value = NULL;

	pangol2_set_instr(pangol2_info->tap, PG2L_CFGO);

	while (num_words--) {
		scan_field.in_value = (uint8_t *)words;

		jtag_add_dr_scan(pangol2_info->tap, 1, &scan_field, TAP_DRPAUSE);

		jtag_add_callback(pangoflip32, (jtag_callback_data_t)words);

		words++;
	}

	return ERROR_OK;
}

static int pangol2_read_id(struct pld_device *pld_device, uint32_t *id)
{
	uint32_t data[5];

	jtag_add_tlr();

	data[0] = PG2L_SYNCH_WORD;
	data[1] = PG2L_PKT_TYPE1(PG2L_READ, PG2L_IDR, 1);
	data[2] = PG2L_PKT_TYPE1(PG2L_NOP, 0, 0); /* NOOP */
	data[3] = PG2L_PKT_TYPE1(PG2L_NOP, 0, 0); /* NOOP */
	data[4] = PG2L_PKT_TYPE1(PG2L_NOP, 0, 0); /* NOOP */
	pangol2_send_32(pld_device, 5, data);

	pangol2_receive_32(pld_device, 1, id);

	jtag_execute_queue();

	LOG_DEBUG("PG2L IDR: 0x%8.8" PRIx32 "", *id);

	return ERROR_OK;
}

static int pangol2_read_stat(struct pld_device *pld_device, uint32_t *status)
{
	uint32_t data[5];

	jtag_add_tlr();

	data[0] = PG2L_SYNCH_WORD;
	data[1] = PG2L_PKT_TYPE1(PG2L_READ, PG2L_STATUSR, 1);
	data[2] = PG2L_PKT_TYPE1(PG2L_NOP, 0, 0); /* NOOP */
	data[3] = PG2L_PKT_TYPE1(PG2L_NOP, 0, 0); /* NOOP */
	data[4] = PG2L_PKT_TYPE1(PG2L_NOP, 0, 0); /* NOOP */
	pangol2_send_32(pld_device, 5, data);

	pangol2_receive_32(pld_device, 1, status);

	jtag_execute_queue();

	LOG_DEBUG("PG2L STATUSR: 0x%8.8" PRIx32 "", *status);

	return ERROR_OK;
}

static int pangol2_load(struct pld_device *pld_device, const char *filename)
{
	struct pangol2_pld_device *pangol2_info = pld_device->driver_priv;
	FILE                      *input_file = NULL;
	long                      file_size = 0;
	size_t                    words_count = 0;
	size_t                    read_count = 0;
	uint8_t                   *data = NULL;
	int                       retval;
	unsigned int              i;
	struct scan_field         field;
	uint32_t                  status;

	if (!filename) {
		LOG_ERROR("file name not specified");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	input_file = fopen(filename, "rb");
	if (!input_file) {
		LOG_ERROR("couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	retval = fseek(input_file, 0, SEEK_END);
	if (retval) {
		LOG_ERROR("couldn't seek file %s end: %s", filename, strerror(errno));
		fclose(input_file);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	file_size = ftell(input_file);
	if (file_size < 0) {
		LOG_ERROR("couldn't get file %s size: %s", filename, strerror(errno));
		fclose(input_file);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	retval = fseek(input_file, 0, SEEK_SET);
	if (retval) {
		LOG_ERROR("couldn't seek file %s start: %s", filename, strerror(errno));
		fclose(input_file);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	words_count = (file_size + sizeof(uint32_t) - 1) / sizeof(uint32_t);
	data = calloc(words_count, sizeof(uint32_t));
	if (!data) {
		LOG_ERROR("couldn't allocate memory (%ld bytes): %s", file_size, strerror(errno));
		fclose(input_file);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	read_count = fread(data, sizeof(uint32_t), words_count, input_file);
	if (read_count != words_count) {
		LOG_ERROR("couldn't read file %s data: %s", filename, strerror(errno));
		free(data);
		fclose(input_file);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	field.in_value = NULL;

	pangol2_set_instr(pangol2_info->tap, PG2L_JRST);
	jtag_add_runtest(100, TAP_IDLE);

	pangol2_set_instr(pangol2_info->tap, PG2L_CFGI);
	jtag_add_sleep(500000);
	jtag_execute_queue();

	for (i = 0; i < words_count * sizeof(uint32_t); i++)
		data[i] = flip_u32(data[i], 8);

	field.num_bits = words_count * 32;
	field.out_value = data;

	jtag_add_dr_scan(pangol2_info->tap, 1, &field, TAP_DRPAUSE);

	/* Write IDCODE command to avoid TLR */
	pangol2_set_instr(pangol2_info->tap, PG2L_IDCODE);
	jtag_add_runtest(100, TAP_IDLE);

	if (!pangol2_info->no_jwakeup) {
		LOG_INFO("sending PG2L JWAKEUP command");
		pangol2_set_instr(pangol2_info->tap, PG2L_JWAKEUP);
	}
	jtag_add_runtest(100, TAP_IDLE);

	pangol2_set_instr(pangol2_info->tap, PG2L_BYPASS); /* BYPASS */
	jtag_add_runtest(100, TAP_IDLE);

	jtag_execute_queue();

	/* Read FPGA status register */
	pangol2_set_instr(pangol2_info->tap, PG2L_RDSR);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = (uint8_t *)&status;

	jtag_add_dr_scan(pangol2_info->tap, 1, &field, TAP_IDLE);

	jtag_execute_queue();

	LOG_INFO("PG2L STATUS = 0x%8.8" PRIx32 ", DONE = %d",
		 status, (status >> 12) & 1);
	if (!(status & (1 << 12))) {
		LOG_ERROR("no DONE bit after configuration, SR error bits:");
		LOG_OUTPUT("    timeout   = %d (Watchdog timeout)\n", (status >> 4) & 1);
		LOG_OUTPUT("    rbcrc_err = %d (Readback CRC Test results)\n", (status >> 3) & 1);
		LOG_OUTPUT("    aut_err   = %d (Certification result)\n", (status >> 2) & 1);
		LOG_OUTPUT("    crc_err   = %d (CRC Test results)\n", (status >> 1) & 1);
		LOG_OUTPUT("    id_err    = %d (ID Test results)\n", (status >> 0) & 1);
	}

	free(data);
	fclose(input_file);

	return ERROR_OK;
}

COMMAND_HANDLER(pangol2_handle_read_userid_command)
{
	struct pld_device *device;
	struct pangol2_pld_device *pangol2_info;
	struct scan_field scan_field;
	uint32_t usercode;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned int dev_id;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	pangol2_info = device->driver_priv;

	pangol2_set_instr(pangol2_info->tap, PG2L_USERCODE);

	scan_field.num_bits = 32;
	scan_field.out_value = NULL;
	scan_field.in_value = (uint8_t *)&usercode;

	jtag_add_dr_scan(pangol2_info->tap, 1, &scan_field, TAP_DRPAUSE);

	jtag_execute_queue();

	command_print(CMD, "PG2L USERCODE: 0x%8.8" PRIx32 "", usercode);

	return ERROR_OK;
}

COMMAND_HANDLER(pangol2_handle_read_id_command)
{
	struct pld_device *device;
	uint32_t id;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned int dev_id;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	pangol2_read_id(device, &id);

	command_print(CMD, "PG2L device identification register: 0x%8.8" PRIx32 "", id);

	return ERROR_OK;
}

COMMAND_HANDLER(pangol2_handle_read_stat_command)
{
	struct pld_device *device;
	uint32_t status;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned int dev_id;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	pangol2_read_stat(device, &status);

	command_print(CMD, "PG2L status register: 0x%8.8" PRIx32 "", status);
	command_print(CMD, "prcfg_over    = %d (Partial reconfiguration complete flag)",
		(status >> 29) & 1);
	command_print(CMD, "prcfg_err     = %d (Partial reconfiguration error flag)",
		(status >> 28) & 1);
	command_print(CMD, "over_temp     = %d (Over temperature flag)",
		(status >> 27) & 1);
	command_print(CMD, "flg_x32       = %d (Slave Parallel mode 32 Bit data width indication)",
		(status >> 26) & 1);
	command_print(CMD, "flg_x16       = %d (Slave Parallel mode 16 Bit data width indication)",
		(status >> 25) & 1);
	command_print(CMD, "flg_x8        = %d (Slave Parallel mode 8 Bit data width indication)",
		(status >> 24) & 1);
	command_print(CMD, "ipal_m[1:0]   = %d (Internal slave parallel interface data width selection)",
		(status >> 22) & 0b11);
	command_print(CMD, "fallback      = %d (Fallback indicator)",
		(status >> 21) & 1);
	command_print(CMD, "dci_match     = %d (DCI match flag)",
		(status >> 20) & 1);
	command_print(CMD, "pll_lock      = %d (PLLs lock sign)",
		(status >> 19) & 1);
	command_print(CMD, "gwen          = %d (Global write enable)",
		(status >> 18) & 1);
	command_print(CMD, "grsn          = %d (Global Register Set Reset)",
		(status >> 17) & 1);
	command_print(CMD, "go_out        = %d (Global I/O output enable)",
		(status >> 16) & 1);
	command_print(CMD, "glogen_fb     = %d (Global Logic Enable Feedback)",
		(status >> 15) & 1);
	command_print(CMD, "glogen        = %d (Global logic enable)",
		(status >> 14) & 1);
	command_print(CMD, "done_i        = %d (DONE pin input)",
		(status >> 13) & 1);
	command_print(CMD, "done          = %d (Device Wakeup Successful Flag)",
		(status >> 12) & 1);
	command_print(CMD, "init_n        = %d (INIT_FLAG_N pin input)",
		(status >> 11) & 1);
	command_print(CMD, "init_complete = %d (Initialization complete and configuration error indication)",
		(status >> 10) & 1);
	command_print(CMD, "m[2:0]        = %d (Mode selection)",
		(status >> 7) & 0b111);
	command_print(CMD, "wakedown_over = %d (Wake-up shutdown ends)",
		(status >> 6) & 1);
	command_print(CMD, "wakeup_over   = %d (End of wake up)",
		(status >> 5) & 1);
	command_print(CMD, "timeout       = %d (Watchdog timeout)",
		(status >> 4) & 1);
	command_print(CMD, "rbcrc_err     = %d (Readback CRC Test results)",
		(status >> 3) & 1);
	command_print(CMD, "aut_err       = %d (Certification result)",
		(status >> 2) & 1);
	command_print(CMD, "crc_err       = %d (CRC Test results)",
		(status >> 1) & 1);
	command_print(CMD, "id_err        = %d (ID Test results)",
		(status >> 0) & 1);

	return ERROR_OK;
}

PLD_DEVICE_COMMAND_HANDLER(pangol2_pld_device_command)
{
	struct jtag_tap *tap;

	struct pangol2_pld_device *pangol2_info;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	tap = jtag_tap_by_string(CMD_ARGV[1]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	pangol2_info = malloc(sizeof(struct pangol2_pld_device));
	pangol2_info->tap = tap;

	pangol2_info->no_jwakeup = 0;
	if (CMD_ARGC >= 3)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], pangol2_info->no_jwakeup);

	pld->driver_priv = pangol2_info;

	return ERROR_OK;
}

static const struct command_registration pangol2_exec_command_handlers[] = {
	{
		.name = "read_userid",
		.mode = COMMAND_EXEC,
		.handler = pangol2_handle_read_userid_command,
		.help = "read device user code value",
		.usage = "pld_num",
	},
	{
		.name = "read_id",
		.mode = COMMAND_EXEC,
		.handler = pangol2_handle_read_id_command,
		.help = "read device identification register",
		.usage = "pld_num",
	},
	{
		.name = "read_stat",
		.mode = COMMAND_EXEC,
		.handler = pangol2_handle_read_stat_command,
		.help = "read status register",
		.usage = "pld_num",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pangol2_command_handler[] = {
	{
		.name = "pangol2",
		.mode = COMMAND_ANY,
		.help = "PangoMicro Logos2 specific commands",
		.usage = "",
		.chain = pangol2_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct pld_driver pangol2_pld = {
	.name = "pangol2",
	.commands = pangol2_command_handler,
	.pld_device_command = &pangol2_pld_device_command,
	.load = &pangol2_load,
};
