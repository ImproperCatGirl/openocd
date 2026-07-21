// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <server/server.h>
#include <target/riscv/dmi.h>
#include <transport/transport.h>
#include <helper/binarybuffer.h>
#include <helper/log.h>
#include <helper/types.h>

#include "mpsse.h"

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define RVSWD_INITIAL_QUEUE_DEPTH 32
#define SWIO_INITIAL_QUEUE_DEPTH 16
#define RVSWD_READ_BITS 33
#define RVSWD_RESET_CLOCKS 100
#define RVSWD_INTEROP_HOLD_CYCLES 3

/*
 * SWIO timing is generated with repeated MPSSE SET_BITS_LOW GPIO commands.
 * Do not treat these counts as adapter-clock cycles. On the tested FTDI setup
 * each repeated GPIO command is roughly 250-300 ns, with additional fixed
 * overhead for the drive/release/read command itself. This GPIO-only method is
 * slower than MPSSE shift-clock timing, but avoids short glitches seen when
 * mixing native shift commands with GPIO direction changes on the same bus.
 */
#define SWIO_GPIO_ONE_LOW_CYCLES 0
#define SWIO_GPIO_ZERO_LOW_CYCLES 4
#define SWIO_GPIO_HIGH_CYCLES 0
#define SWIO_GPIO_READ_LOW_CYCLES 0
#define SWIO_GPIO_READ_SAMPLE_CYCLES 1
#define SWIO_GPIO_READ_POST_CYCLES 1
#define SWIO_STOP_CYCLES 20
#define SWIO_INTEROP_HOLD_CYCLES 3
#define SWIO_RESET_LOW_CYCLES 64
#define WCH_DMCONTROL 0x10
#define WCH_DM_CFGR 0x7d
#define WCH_DM_SHDWCFGR 0x7e
#define WCH_DM_DEBUG_OUTPUT_ENABLE 0x5aa50400

/*
 * MPSSE auto-flushes when its internal command buffers fill. That is safe for
 * normal shift commands, but SWIO DMI transactions are synthesized from many
 * GPIO commands and must not be split in the middle. Keep the SWIO transaction
 * queue small enough that only rvswd_run_queue() decides the flush boundary.
 */

enum ftdi_rvswd_protocol {
	FTDI_RVSWD_PROTOCOL_RVSWD,
	FTDI_RVSWD_PROTOCOL_SWIO,
};

static uint8_t ftdi_rvswd_channel;
static struct mpsse_ctx *rvswd_mpsse_ctx;
static uint16_t output;
static uint16_t direction;
static uint16_t output_init;
static uint16_t direction_init;
static enum ftdi_rvswd_protocol rvswd_protocol = FTDI_RVSWD_PROTOCOL_RVSWD;
static int queued_retval;

struct signal {
	const char *name;
	uint16_t data_mask;
	uint16_t input_mask;
	uint16_t oe_mask;
	bool invert_data;
	bool invert_input;
	bool invert_oe;
	struct signal *next;
};

static struct signal *signals;

struct rvswd_cmd_queue_entry {
	bool read;
	uint8_t addr;
	uint32_t data;
	uint32_t *dst;
	uint8_t read_low[RVSWD_READ_BITS];
	uint8_t read_high[RVSWD_READ_BITS];
};

static struct rvswd_cmd_queue_entry *rvswd_cmd_queue;
static size_t rvswd_cmd_queue_length;
static size_t rvswd_cmd_queue_alloced;

static int rvswd_run_queue(void);

static struct signal *find_signal_by_name(const char *name)
{
	for (struct signal *sig = signals; sig; sig = sig->next) {
		if (!strcmp(sig->name, name))
			return sig;
	}
	return NULL;
}

static struct signal *create_signal(const char *name)
{
	struct signal **psig = &signals;

	while (*psig)
		psig = &(*psig)->next;

	*psig = calloc(1, sizeof(**psig));
	if (!*psig)
		return NULL;

	(*psig)->name = strdup(name);
	if (!(*psig)->name) {
		free(*psig);
		*psig = NULL;
	}

	return *psig;
}

static int rvswd_set_signal(const struct signal *s, char value)
{
	bool data;
	bool oe;

	if (s->data_mask == 0 && s->oe_mask == 0) {
		LOG_ERROR("interface doesn't provide signal '%s'", s->name);
		return ERROR_FAIL;
	}

	switch (value) {
	case '0':
		data = s->invert_data;
		oe = !s->invert_oe;
		break;
	case '1':
		if (s->data_mask == 0) {
			LOG_ERROR("interface can't drive '%s' high", s->name);
			return ERROR_FAIL;
		}
		data = !s->invert_data;
		oe = !s->invert_oe;
		break;
	case 'z':
	case 'Z':
		if (s->oe_mask == 0) {
			LOG_ERROR("interface can't tri-state '%s'", s->name);
			return ERROR_FAIL;
		}
		data = s->invert_data;
		oe = s->invert_oe;
		break;
	default:
		LOG_ERROR("invalid signal level specifier '%c'(0x%02x)", value, value);
		return ERROR_FAIL;
	}

	uint16_t old_output = output;
	uint16_t old_direction = direction;

	output = data ? output | s->data_mask : output & ~s->data_mask;
	if (s->oe_mask == s->data_mask)
		direction = oe ? direction | s->oe_mask : direction & ~s->oe_mask;
	else
		output = oe ? output | s->oe_mask : output & ~s->oe_mask;

	if ((output & 0xff) != (old_output & 0xff) ||
			(direction & 0xff) != (old_direction & 0xff))
		mpsse_set_data_bits_low_byte(rvswd_mpsse_ctx, output & 0xff, direction & 0xff);
	if ((output >> 8) != (old_output >> 8) ||
			(direction >> 8) != (old_direction >> 8))
		mpsse_set_data_bits_high_byte(rvswd_mpsse_ctx, output >> 8, direction >> 8);

	return ERROR_OK;
}

static int rvswd_get_signal(const struct signal *s, uint16_t *value_out)
{
	uint8_t data_low = 0;
	uint8_t data_high = 0;

	if (s->input_mask == 0) {
		LOG_ERROR("interface doesn't provide signal '%s'", s->name);
		return ERROR_FAIL;
	}

	if (s->input_mask & 0xff)
		mpsse_read_data_bits_low_byte(rvswd_mpsse_ctx, &data_low);
	if (s->input_mask >> 8)
		mpsse_read_data_bits_high_byte(rvswd_mpsse_ctx, &data_high);

	int retval = mpsse_flush(rvswd_mpsse_ctx);
	if (retval != ERROR_OK)
		return retval;

	*value_out = (((uint16_t)data_high) << 8) | data_low;
	if (s->invert_input)
		*value_out = ~(*value_out);
	*value_out &= s->input_mask;

	return ERROR_OK;
}

static int rvswd_apply_signal_state(uint16_t *new_output, uint16_t *new_direction,
		const struct signal *s, bool data_level, bool drive)
{
	if (!s || !s->data_mask) {
		LOG_ERROR("interface doesn't provide signal '%s'", s ? s->name : "<null>");
		return ERROR_FAIL;
	}

	bool data = data_level ? !s->invert_data : s->invert_data;
	bool oe = drive ? !s->invert_oe : s->invert_oe;

	*new_output = data ? *new_output | s->data_mask : *new_output & ~s->data_mask;
	if (s->oe_mask == s->data_mask)
		*new_direction = oe ? *new_direction | s->oe_mask : *new_direction & ~s->oe_mask;
	else if (s->oe_mask)
		*new_output = oe ? *new_output | s->oe_mask : *new_output & ~s->oe_mask;
	else if (!drive)
		LOG_ERROR("interface can't tri-state '%s'", s->name);

	return ERROR_OK;
}

static int rvswd_apply_bus_state(bool clk_high, bool swdio_high, bool swdio_drive)
{
	struct signal *swclk = find_signal_by_name("SWCLK");
	struct signal *swdio = find_signal_by_name("SWDIO");
	if (!swclk || !swdio) {
		LOG_ERROR("FTDI RVSWD requires SWCLK and SWDIO layout signals.");
		return ERROR_FAIL;
	}

	uint16_t new_output = output;
	uint16_t new_direction = direction;

	if (rvswd_apply_signal_state(&new_output, &new_direction,
				swclk, clk_high, true) != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_apply_signal_state(&new_output, &new_direction,
				swdio, swdio_high, swdio_drive) != ERROR_OK)
		return ERROR_FAIL;

	if ((new_output & 0xff) != (output & 0xff) ||
			(new_direction & 0xff) != (direction & 0xff))
		mpsse_set_data_bits_low_byte(rvswd_mpsse_ctx, new_output & 0xff,
				new_direction & 0xff);
	if ((new_output >> 8) != (output >> 8) ||
			(new_direction >> 8) != (direction >> 8))
		mpsse_set_data_bits_high_byte(rvswd_mpsse_ctx, new_output >> 8,
				new_direction >> 8);

	output = new_output;
	direction = new_direction;
	return ERROR_OK;
}

static int rvswd_write_bit(bool bit)
{
	int retval = rvswd_apply_bus_state(false, bit, true);
	if (retval != ERROR_OK)
		return retval;

	return rvswd_apply_bus_state(true, bit, true);
}

static int rvswd_queue_read_bit(struct rvswd_cmd_queue_entry *entry,
		unsigned int bit)
{
	struct signal *swdio = find_signal_by_name("SWDIO");
	if (!swdio || !swdio->input_mask) {
		LOG_ERROR("FTDI RVSWD requires SWDIO input mask.");
		return ERROR_FAIL;
	}

	int retval = rvswd_apply_bus_state(false, false, false);
	if (retval != ERROR_OK)
		return retval;
	retval = rvswd_apply_bus_state(true, false, false);
	if (retval != ERROR_OK)
		return retval;

	if (swdio->input_mask & 0xff)
		mpsse_read_data_bits_low_byte(rvswd_mpsse_ctx, &entry->read_low[bit]);
	if (swdio->input_mask >> 8)
		mpsse_read_data_bits_high_byte(rvswd_mpsse_ctx, &entry->read_high[bit]);

	return ERROR_OK;
}

static int rvswd_write_addr_header(uint8_t addr, bool write)
{
	bool parity = false;

	for (int i = 6; i >= 0; i--) {
		bool bit = !!(addr & BIT(i));
		if (rvswd_write_bit(bit) != ERROR_OK)
			return ERROR_FAIL;
		parity ^= bit;
	}

	if (rvswd_write_bit(write) != ERROR_OK)
		return ERROR_FAIL;
	parity ^= write;
	if (rvswd_write_bit(parity) != ERROR_OK)
		return ERROR_FAIL;

	static const bool turnaround[] = { true, false, true, false, true };
	for (unsigned int i = 0; i < ARRAY_SIZE(turnaround); i++) {
		if (rvswd_write_bit(turnaround[i]) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int rvswd_write_data(uint32_t value)
{
	bool parity = false;

	for (int i = 31; i >= 0; i--) {
		bool bit = !!(value & BIT(i));
		if (rvswd_write_bit(bit) != ERROR_OK)
			return ERROR_FAIL;
		parity ^= bit;
	}

	return rvswd_write_bit(parity);
}

static int rvswd_write_tail(void)
{
	static const bool tail[] = { true, false, true, true, true };

	for (unsigned int i = 0; i < ARRAY_SIZE(tail); i++) {
		if (rvswd_write_bit(tail[i]) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int rvswd_start(void)
{
	int retval = rvswd_apply_bus_state(true, true, true);
	if (retval != ERROR_OK)
		return retval;
	return rvswd_apply_bus_state(true, false, true);
}

static int rvswd_stop(void)
{
	int retval = rvswd_apply_bus_state(true, false, true);
	if (retval != ERROR_OK)
		return retval;
	retval = rvswd_apply_bus_state(true, true, true);
	if (retval != ERROR_OK)
		return retval;
	return rvswd_apply_bus_state(true, true, false);
}

static int rvswd_force_bus_state(bool clk_high, bool swdio_high, bool swdio_drive)
{
	struct signal *swclk = find_signal_by_name("SWCLK");
	struct signal *swdio = find_signal_by_name("SWDIO");
	if (!swclk || !swdio) {
		LOG_ERROR("FTDI RVSWD requires SWCLK and SWDIO layout signals.");
		return ERROR_FAIL;
	}

	uint16_t new_output = output;
	uint16_t new_direction = direction;

	if (rvswd_apply_signal_state(&new_output, &new_direction,
				swclk, clk_high, true) != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_apply_signal_state(&new_output, &new_direction,
				swdio, swdio_high, swdio_drive) != ERROR_OK)
		return ERROR_FAIL;

	mpsse_set_data_bits_low_byte(rvswd_mpsse_ctx, new_output & 0xff,
			new_direction & 0xff);
	mpsse_set_data_bits_high_byte(rvswd_mpsse_ctx, new_output >> 8,
			new_direction >> 8);

	output = new_output;
	direction = new_direction;
	return ERROR_OK;
}

static int rvswd_idle_hold(unsigned int hold_cycles)
{
	for (unsigned int i = 0; i < hold_cycles; i++) {
		int retval = rvswd_force_bus_state(true, true, false);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int swio_set_pull_high(void)
{
	struct signal *pull = find_signal_by_name("SWIO_PULL");
	if (!pull)
		return ERROR_OK;

	return rvswd_set_signal(pull, '1');
}

static int swio_prepare_bus(void)
{
	int retval = swio_set_pull_high();
	if (retval != ERROR_OK)
		return retval;

	struct signal *swio = find_signal_by_name("SWIO");
	if (!swio) {
		LOG_ERROR("FTDI SWIO requires SWIO layout signal.");
		return ERROR_FAIL;
	}

	return rvswd_set_signal(swio, 'z');
}

static int swio_drive_low(void)
{
	struct signal *swio = find_signal_by_name("SWIO");
	if (!swio) {
		LOG_ERROR("FTDI SWIO requires SWIO layout signal.");
		return ERROR_FAIL;
	}

	return rvswd_set_signal(swio, '0');
}

static int swio_release(void)
{
	struct signal *swio = find_signal_by_name("SWIO");
	if (!swio) {
		LOG_ERROR("FTDI SWIO requires SWIO layout signal.");
		return ERROR_FAIL;
	}
	if (swio->data_mask == 0 || swio->oe_mask != swio->data_mask) {
		LOG_ERROR("FTDI SWIO requires a same-mask data/OE signal for open-drain emulation.");
		return ERROR_FAIL;
	}

	uint16_t old_output = output;
	uint16_t old_direction = direction;

	/* Release as high-Z, but preload the output latch high. */
	output = swio->invert_data ? output & ~swio->data_mask : output | swio->data_mask;
	direction = swio->invert_oe ? direction | swio->oe_mask : direction & ~swio->oe_mask;

	if ((output & 0xff) != (old_output & 0xff) ||
			(direction & 0xff) != (old_direction & 0xff))
		mpsse_set_data_bits_low_byte(rvswd_mpsse_ctx, output & 0xff, direction & 0xff);
	if ((output >> 8) != (old_output >> 8) ||
			(direction >> 8) != (old_direction >> 8))
		mpsse_set_data_bits_high_byte(rvswd_mpsse_ctx, output >> 8, direction >> 8);

	return ERROR_OK;
}

static void swio_delay_cycles(unsigned int cycles)
{
	for (unsigned int i = 0; i < cycles; i++) {
		mpsse_set_data_bits_low_byte(rvswd_mpsse_ctx,
				output & 0xff, direction & 0xff);
		if (output >> 8 || direction >> 8)
			mpsse_set_data_bits_high_byte(rvswd_mpsse_ctx,
					output >> 8, direction >> 8);
	}
}

static int swio_write_bit(bool bit)
{
	if (swio_drive_low() != ERROR_OK)
		return ERROR_FAIL;
	swio_delay_cycles(bit ? SWIO_GPIO_ONE_LOW_CYCLES :
			SWIO_GPIO_ZERO_LOW_CYCLES);
	if (swio_release() != ERROR_OK)
		return ERROR_FAIL;
	swio_delay_cycles(SWIO_GPIO_HIGH_CYCLES);
	return ERROR_OK;
}

static int swio_write_nibble(uint8_t value)
{
	for (int i = 3; i >= 0; i--) {
		if (swio_write_bit(!!(value & BIT(i))) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int swio_write_test_pattern_group(void)
{
	if (swio_write_nibble(0x0) != ERROR_OK)
		return ERROR_FAIL;
	if (swio_write_nibble(0xf) != ERROR_OK)
		return ERROR_FAIL;
	return swio_write_nibble(0xa);
}

static int swio_queue_read_bit(struct rvswd_cmd_queue_entry *entry,
		unsigned int bit)
{
	struct signal *swio = find_signal_by_name("SWIO");
	if (!swio || !swio->input_mask) {
		LOG_ERROR("FTDI SWIO requires SWIO input mask.");
		return ERROR_FAIL;
	}

	if (swio_drive_low() != ERROR_OK)
		return ERROR_FAIL;
	swio_delay_cycles(SWIO_GPIO_READ_LOW_CYCLES);
	if (swio_release() != ERROR_OK)
		return ERROR_FAIL;
	swio_delay_cycles(SWIO_GPIO_READ_SAMPLE_CYCLES);

	if (swio->input_mask & 0xff)
		mpsse_read_data_bits_low_byte(rvswd_mpsse_ctx, &entry->read_low[bit]);
	if (swio->input_mask >> 8)
		mpsse_read_data_bits_high_byte(rvswd_mpsse_ctx, &entry->read_high[bit]);

	swio_delay_cycles(SWIO_GPIO_READ_POST_CYCLES);
	return ERROR_OK;
}

static int swio_stop(void)
{
	if (swio_release() != ERROR_OK)
		return ERROR_FAIL;
	swio_delay_cycles(SWIO_STOP_CYCLES);
	return ERROR_OK;
}

static int swio_idle_hold(unsigned int hold_cycles)
{
	if (swio_release() != ERROR_OK)
		return ERROR_FAIL;
	swio_delay_cycles(hold_cycles);
	return ERROR_OK;
}

static int swio_write_header(uint8_t addr, bool write)
{
	if (swio_write_bit(true) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 6; i >= 0; i--) {
		if (swio_write_bit(!!(addr & BIT(i))) != ERROR_OK)
			return ERROR_FAIL;
	}

	return swio_write_bit(write);
}

static int swio_queue_read(uint8_t addr, uint32_t *value)
{
	LOG_DEBUG_IO("FTDI SWIO queue DMI read 0x%02" PRIx8, addr);

	if (rvswd_cmd_queue_length >= rvswd_cmd_queue_alloced) {
		int retval = rvswd_run_queue();
		if (retval != ERROR_OK)
			return retval;
	}

	if (!rvswd_cmd_queue_length) {
		int retval = swio_prepare_bus();
		if (retval != ERROR_OK)
			return retval;
	}

	struct rvswd_cmd_queue_entry *entry =
		&rvswd_cmd_queue[rvswd_cmd_queue_length++];
	memset(entry, 0, sizeof(*entry));
	entry->read = true;
	entry->addr = addr;
	entry->dst = value;

	if (swio_write_header(addr, false) != ERROR_OK)
		return ERROR_FAIL;
	for (unsigned int bit = 0; bit < 32; bit++) {
		if (swio_queue_read_bit(entry, bit) != ERROR_OK)
			return ERROR_FAIL;
	}

	int retval = swio_stop();
	if (retval != ERROR_OK)
		return retval;

	return swio_idle_hold(SWIO_INTEROP_HOLD_CYCLES);
}

static int swio_queue_write(uint8_t addr, uint32_t value)
{
	LOG_DEBUG_IO("FTDI SWIO queue DMI write 0x%02" PRIx8 " = 0x%08" PRIx32,
			addr, value);

	if (rvswd_cmd_queue_length >= rvswd_cmd_queue_alloced) {
		int retval = rvswd_run_queue();
		if (retval != ERROR_OK)
			return retval;
	}

	if (!rvswd_cmd_queue_length) {
		int retval = swio_prepare_bus();
		if (retval != ERROR_OK)
			return retval;
	}

	struct rvswd_cmd_queue_entry *entry =
		&rvswd_cmd_queue[rvswd_cmd_queue_length++];
	memset(entry, 0, sizeof(*entry));
	entry->addr = addr;
	entry->data = value;

	if (swio_write_header(addr, true) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 31; i >= 0; i--) {
		if (swio_write_bit(!!(value & BIT(i))) != ERROR_OK)
			return ERROR_FAIL;
	}

	int retval = swio_stop();
	if (retval != ERROR_OK)
		return retval;

	return swio_idle_hold(SWIO_INTEROP_HOLD_CYCLES);
}

static int swio_init_debug_interface(void)
{
	int retval = swio_queue_write(WCH_DM_SHDWCFGR,
			WCH_DM_DEBUG_OUTPUT_ENABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = swio_queue_write(WCH_DM_CFGR, WCH_DM_DEBUG_OUTPUT_ENABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = swio_queue_write(WCH_DMCONTROL, 0);
	if (retval != ERROR_OK)
		return retval;
	retval = swio_queue_write(WCH_DMCONTROL, 1);
	if (retval != ERROR_OK)
		return retval;

	return rvswd_run_queue();
}

static int rvswd_queue_read(uint8_t addr, uint32_t *value)
{
	LOG_DEBUG_IO("FTDI RVSWD queue DMI read 0x%02" PRIx8, addr);

	if (rvswd_cmd_queue_length >= rvswd_cmd_queue_alloced) {
		int retval = rvswd_run_queue();
		if (retval != ERROR_OK)
			return retval;
	}

	struct rvswd_cmd_queue_entry *entry =
		&rvswd_cmd_queue[rvswd_cmd_queue_length++];
	memset(entry, 0, sizeof(*entry));
	entry->read = true;
	entry->addr = addr;
	entry->dst = value;

	if (rvswd_start() != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_write_addr_header(addr, false) != ERROR_OK)
		return ERROR_FAIL;
	for (unsigned int bit = 0; bit < RVSWD_READ_BITS; bit++) {
		if (rvswd_queue_read_bit(entry, bit) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (rvswd_write_tail() != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_stop() != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_idle_hold(RVSWD_INTEROP_HOLD_CYCLES) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int rvswd_queue_write(uint8_t addr, uint32_t value)
{
	LOG_DEBUG_IO("FTDI RVSWD queue DMI write 0x%02" PRIx8 " = 0x%08" PRIx32,
			addr, value);

	if (rvswd_cmd_queue_length >= rvswd_cmd_queue_alloced) {
		int retval = rvswd_run_queue();
		if (retval != ERROR_OK)
			return retval;
	}

	struct rvswd_cmd_queue_entry *entry =
		&rvswd_cmd_queue[rvswd_cmd_queue_length++];
	memset(entry, 0, sizeof(*entry));
	entry->addr = addr;
	entry->data = value;

	if (rvswd_start() != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_write_addr_header(addr, true) != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_write_data(value) != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_write_tail() != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_stop() != ERROR_OK)
		return ERROR_FAIL;
	if (rvswd_idle_hold(RVSWD_INTEROP_HOLD_CYCLES) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int rvswd_run_queue(void)
{
	struct signal *led = find_signal_by_name("LED");

	if (queued_retval != ERROR_OK)
		goto done;

	if (led)
		rvswd_set_signal(led, '0');

	queued_retval = mpsse_flush(rvswd_mpsse_ctx);
	if (queued_retval != ERROR_OK)
		goto done;

	for (size_t i = 0; i < rvswd_cmd_queue_length; i++) {
		struct rvswd_cmd_queue_entry *entry = &rvswd_cmd_queue[i];
		if (!entry->read)
			continue;

		struct signal *data_signal = find_signal_by_name(
				rvswd_protocol == FTDI_RVSWD_PROTOCOL_SWIO ? "SWIO" : "SWDIO");
		if (!data_signal || !data_signal->input_mask) {
			queued_retval = ERROR_FAIL;
			goto done;
		}

		uint32_t data = 0;
		bool parity = false;
		for (unsigned int bit = 0; bit < 32; bit++) {
			uint16_t sample = entry->read_low[bit] |
				((uint16_t)entry->read_high[bit] << 8);
			if (data_signal->invert_input)
				sample = ~sample;
			bool value = !!(sample & data_signal->input_mask);
			if (value)
				data |= BIT(31 - bit);
			parity ^= value;
		}

		if (rvswd_protocol == FTDI_RVSWD_PROTOCOL_SWIO) {
			if (entry->dst)
				*entry->dst = data;
			continue;
		}

		uint16_t sample = entry->read_low[32] |
			((uint16_t)entry->read_high[32] << 8);
		if (data_signal->invert_input)
			sample = ~sample;
		bool got_parity = !!(sample & data_signal->input_mask);
		if (parity != got_parity) {
			LOG_ERROR("FTDI RVSWD read parity mismatch for DMI 0x%02" PRIx8
					": value 0x%08" PRIx32 ", expected parity %u, got %u",
					entry->addr, data, parity ? 1 : 0, got_parity ? 1 : 0);
			queued_retval = ERROR_FAIL;
			goto done;
		}

		if (entry->dst)
			*entry->dst = data;
	}

done:
	rvswd_cmd_queue_length = 0;
	int retval = queued_retval;
	queued_retval = ERROR_OK;

	if (led && retval == ERROR_OK) {
		retval = rvswd_set_signal(led, '1');
		if (retval == ERROR_OK)
			retval = mpsse_flush(rvswd_mpsse_ctx);
	}

	return retval;
}

static int rvswd_direct_read(uint32_t address, uint32_t *value)
{
	if (rvswd_protocol == FTDI_RVSWD_PROTOCOL_SWIO)
		return swio_queue_read(address & 0x7f, value);

	return rvswd_queue_read(address & 0x7f, value);
}

static int rvswd_direct_write(uint32_t address, uint32_t value)
{
	if (rvswd_protocol == FTDI_RVSWD_PROTOCOL_SWIO)
		return swio_queue_write(address & 0x7f, value);

	return rvswd_queue_write(address & 0x7f, value);
}

static int rvswd_direct_reset(void)
{
	if (rvswd_protocol == FTDI_RVSWD_PROTOCOL_SWIO) {
		LOG_DEBUG_IO("FTDI SWIO line reset");
		if (swio_set_pull_high() != ERROR_OK)
			return ERROR_FAIL;
		if (swio_drive_low() != ERROR_OK)
			return ERROR_FAIL;
		swio_delay_cycles(SWIO_RESET_LOW_CYCLES);
		if (swio_stop() != ERROR_OK)
			return ERROR_FAIL;
		int retval = mpsse_flush(rvswd_mpsse_ctx);
		if (retval != ERROR_OK)
			return retval;
		return swio_init_debug_interface();
	}

	LOG_DEBUG_IO("FTDI RVSWD line reset");
	if (rvswd_apply_bus_state(true, true, true) != ERROR_OK)
		return ERROR_FAIL;
	for (unsigned int i = 0; i < RVSWD_RESET_CLOCKS; i++) {
		if (rvswd_apply_bus_state(false, true, true) != ERROR_OK)
			return ERROR_FAIL;
		if (rvswd_apply_bus_state(true, true, true) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (rvswd_stop() != ERROR_OK)
		return ERROR_FAIL;
	return mpsse_flush(rvswd_mpsse_ctx);
}

static int rvswd_direct_batch_exec(void)
{
	return rvswd_run_queue();
}

static const struct riscv_dmi_direct_ops ftdi_rvswd_direct_ops = {
	.read = rvswd_direct_read,
	.write = rvswd_direct_write,
	.reset = rvswd_direct_reset,
	.batch_exec = rvswd_direct_batch_exec,
};

static int ftdi_rvswd_initialize(void)
{
	if (!adapter_usb_get_vids()[0] && !adapter_usb_get_pids()[0]) {
		LOG_ERROR("Please specify 'adapter usb vid_pid'");
		return ERROR_JTAG_INIT_FAILED;
	}

	rvswd_mpsse_ctx = mpsse_open(adapter_usb_get_vids(), adapter_usb_get_pids(),
		adapter_usb_get_product_name(), adapter_get_required_serial(),
		adapter_usb_get_location(), ftdi_rvswd_channel);
	if (!rvswd_mpsse_ctx)
		return ERROR_JTAG_INIT_FAILED;

	output = output_init;
	direction = direction_init;
	mpsse_set_data_bits_low_byte(rvswd_mpsse_ctx, output & 0xff, direction & 0xff);
	mpsse_set_data_bits_high_byte(rvswd_mpsse_ctx, output >> 8, direction >> 8);
	mpsse_loopback_config(rvswd_mpsse_ctx, false);
	mpsse_set_frequency(rvswd_mpsse_ctx, adapter_get_speed_khz() * 1000);

	rvswd_cmd_queue_alloced = rvswd_protocol == FTDI_RVSWD_PROTOCOL_SWIO ?
		SWIO_INITIAL_QUEUE_DEPTH : RVSWD_INITIAL_QUEUE_DEPTH;
	rvswd_cmd_queue = calloc(rvswd_cmd_queue_alloced, sizeof(*rvswd_cmd_queue));
	if (!rvswd_cmd_queue)
		return ERROR_JTAG_INIT_FAILED;

	int retval = riscv_dmi_direct_register_provider("ftdi_rvswd",
			&ftdi_rvswd_direct_ops);
	if (retval != ERROR_OK)
		return retval;

	retval = mpsse_flush(rvswd_mpsse_ctx);
	if (retval != ERROR_OK)
		return retval;

	return rvswd_direct_reset();
}

static int ftdi_rvswd_quit(void)
{
	free(rvswd_cmd_queue);
	rvswd_cmd_queue = NULL;
	rvswd_cmd_queue_length = 0;
	rvswd_cmd_queue_alloced = 0;

	if (rvswd_mpsse_ctx) {
		mpsse_close(rvswd_mpsse_ctx);
		rvswd_mpsse_ctx = NULL;
	}

	struct signal *sig = signals;
	while (sig) {
		struct signal *next = sig->next;
		free((void *)sig->name);
		free(sig);
		sig = next;
	}
	signals = NULL;

	return ERROR_OK;
}

static int ftdi_rvswd_reset(int trst, int srst)
{
	struct signal *sig_nsrst = find_signal_by_name("nSRST");

	LOG_DEBUG_IO("reset trst: %i srst %i", trst, srst);

	if (srst == 1) {
		if (sig_nsrst)
			rvswd_set_signal(sig_nsrst, '0');
		else
			LOG_ERROR("Can't assert SRST: nSRST signal is not defined");
	} else if (sig_nsrst && jtag_get_reset_config() & RESET_HAS_SRST &&
			srst == 0) {
		if (jtag_get_reset_config() & RESET_SRST_PUSH_PULL)
			rvswd_set_signal(sig_nsrst, '1');
		else
			rvswd_set_signal(sig_nsrst, 'z');
	}

	return mpsse_flush(rvswd_mpsse_ctx);
}

static int ftdi_rvswd_speed(int speed)
{
	int retval = mpsse_set_frequency(rvswd_mpsse_ctx, speed);

	if (retval < 0) {
		LOG_ERROR("couldn't set FTDI RVSWD speed");
		return retval;
	}

	return ERROR_OK;
}

static int ftdi_rvswd_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int ftdi_rvswd_khz(int khz, int *rvswd_speed)
{
	if (khz == 0 && !mpsse_is_high_speed(rvswd_mpsse_ctx)) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*rvswd_speed = khz * 1000;
	return ERROR_OK;
}

static int ftdi_rvswd_execute_queue(struct jtag_command *cmd_queue)
{
	return rvswd_run_queue();
}

COMMAND_HANDLER(ftdi_rvswd_handle_channel_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], ftdi_rvswd_channel);
	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_rvswd_handle_layout_init_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], output_init);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], direction_init);
	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_rvswd_handle_layout_signal_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bool invert_data = false;
	uint16_t data_mask = 0;
	bool invert_input = false;
	uint16_t input_mask = 0;
	bool invert_oe = false;
	uint16_t oe_mask = 0;

	for (unsigned int i = 1; i < CMD_ARGC; i += 2) {
		if (i + 1 == CMD_ARGC)
			return ERROR_COMMAND_SYNTAX_ERROR;
		if (!strcmp("-data", CMD_ARGV[i])) {
			invert_data = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], data_mask);
		} else if (!strcmp("-ndata", CMD_ARGV[i])) {
			invert_data = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], data_mask);
		} else if (!strcmp("-input", CMD_ARGV[i])) {
			invert_input = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], input_mask);
		} else if (!strcmp("-ninput", CMD_ARGV[i])) {
			invert_input = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], input_mask);
		} else if (!strcmp("-oe", CMD_ARGV[i])) {
			invert_oe = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], oe_mask);
		} else if (!strcmp("-noe", CMD_ARGV[i])) {
			invert_oe = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], oe_mask);
		} else if (!strcmp("-alias", CMD_ARGV[i]) ||
				!strcmp("-nalias", CMD_ARGV[i])) {
			if (!strcmp("-nalias", CMD_ARGV[i])) {
				invert_data = true;
				invert_input = true;
			}
			struct signal *sig = find_signal_by_name(CMD_ARGV[i + 1]);
			if (!sig) {
				LOG_ERROR("signal %s is not defined", CMD_ARGV[i + 1]);
				return ERROR_FAIL;
			}
			data_mask = sig->data_mask;
			input_mask = sig->input_mask;
			oe_mask = sig->oe_mask;
			invert_input ^= sig->invert_input;
			invert_oe = sig->invert_oe;
			invert_data ^= sig->invert_data;
		} else {
			LOG_ERROR("unknown option '%s'", CMD_ARGV[i]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	struct signal *sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig)
		sig = create_signal(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("failed to create signal %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	sig->invert_data = invert_data;
	sig->data_mask = data_mask;
	sig->invert_input = invert_input;
	sig->input_mask = input_mask;
	sig->invert_oe = invert_oe;
	sig->oe_mask = oe_mask;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_rvswd_handle_protocol_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!strcmp(CMD_ARGV[0], "rvswd")) {
		rvswd_protocol = FTDI_RVSWD_PROTOCOL_RVSWD;
		return ERROR_OK;
	}

	if (!strcmp(CMD_ARGV[0], "swio")) {
		rvswd_protocol = FTDI_RVSWD_PROTOCOL_SWIO;
		return ERROR_OK;
	}

	LOG_ERROR("unknown FTDI RVSWD protocol '%s'", CMD_ARGV[0]);
	return ERROR_COMMAND_ARGUMENT_INVALID;
}

COMMAND_HANDLER(ftdi_rvswd_handle_set_signal_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct signal *sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("interface configuration doesn't define signal '%s'", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	if (CMD_ARGV[1][1] != '\0')
		return ERROR_COMMAND_ARGUMENT_INVALID;

	int retval = rvswd_set_signal(sig, CMD_ARGV[1][0]);
	if (retval != ERROR_OK)
		return retval;

	return mpsse_flush(rvswd_mpsse_ctx);
}

COMMAND_HANDLER(ftdi_rvswd_handle_get_signal_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct signal *sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig) {
		command_print(CMD, "interface configuration doesn't define signal '%s'", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	uint16_t sig_data = 0;
	int retval = rvswd_get_signal(sig, &sig_data);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "%#06x", sig_data);
	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_rvswd_handle_dump_pins_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint8_t data_low = 0;
	uint8_t data_high = 0;

	mpsse_read_data_bits_low_byte(rvswd_mpsse_ctx, &data_low);
	mpsse_read_data_bits_high_byte(rvswd_mpsse_ctx, &data_high);

	int retval = mpsse_flush(rvswd_mpsse_ctx);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "cached data=%#06" PRIx16 " dir=%#06" PRIx16
			" sampled=%#06" PRIx16,
			output, direction, ((uint16_t)data_high << 8) | data_low);

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_rvswd_handle_test_pattern_command)
{
	uint32_t groups = 0;
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], groups);

	if (rvswd_protocol != FTDI_RVSWD_PROTOCOL_SWIO) {
		LOG_ERROR("FTDI RVSWD test_pattern is currently only implemented for SWIO");
		return ERROR_FAIL;
	}

	command_print(CMD, "emitting SWIO 0x0, 0xf, 0xa test pattern%s",
			groups ? "" : " until shutdown");

	int retval = swio_prepare_bus();
	if (retval != ERROR_OK)
		return retval;

	uint32_t emitted = 0;
	while (!openocd_is_shutdown_pending() && (!groups || emitted < groups)) {
		uint32_t chunk = 64;
		if (groups && chunk > groups - emitted)
			chunk = groups - emitted;

		for (uint32_t i = 0; i < chunk; i++) {
			retval = swio_write_test_pattern_group();
			if (retval != ERROR_OK)
				return retval;
		}

		retval = mpsse_flush(rvswd_mpsse_ctx);
		if (retval != ERROR_OK)
			return retval;

		emitted += chunk;
		keep_alive();
	}

	return ERROR_OK;
}

static const struct command_registration ftdi_rvswd_subcommand_handlers[] = {
	{
		.name = "channel",
		.handler = ftdi_rvswd_handle_channel_command,
		.mode = COMMAND_CONFIG,
		.help = "set the channel of the FTDI device that is used as RVSWD",
		.usage = "(0-3)",
	},
	{
		.name = "layout_init",
		.handler = ftdi_rvswd_handle_layout_init_command,
		.mode = COMMAND_CONFIG,
		.help = "initialize the FTDI GPIO signals used by RVSWD",
		.usage = "data direction",
	},
	{
		.name = "layout_signal",
		.handler = ftdi_rvswd_handle_layout_signal_command,
		.mode = COMMAND_ANY,
		.help = "define a layout-specific FTDI signal",
		.usage = "name [-data mask|-ndata mask] [-input mask|-ninput mask] [-oe mask|-noe mask] [-alias|-nalias name]",
	},
	{
		.name = "protocol",
		.handler = ftdi_rvswd_handle_protocol_command,
		.mode = COMMAND_CONFIG,
		.help = "select WCH direct-DMI wire protocol",
		.usage = "(rvswd|swio)",
	},
	{
		.name = "set_signal",
		.handler = ftdi_rvswd_handle_set_signal_command,
		.mode = COMMAND_EXEC,
		.help = "control a layout-specific signal",
		.usage = "name (1|0|z)",
	},
	{
		.name = "get_signal",
		.handler = ftdi_rvswd_handle_get_signal_command,
		.mode = COMMAND_EXEC,
		.help = "read a layout-specific signal",
		.usage = "name",
	},
	{
		.name = "dump_pins",
		.handler = ftdi_rvswd_handle_dump_pins_command,
		.mode = COMMAND_EXEC,
		.help = "print cached FTDI GPIO output/direction and sampled input pins",
		.usage = "",
	},
	{
		.name = "test_pattern",
		.handler = ftdi_rvswd_handle_test_pattern_command,
		.mode = COMMAND_EXEC,
		.help = "emit a repeated SWIO 0x0, 0xf, 0xa waveform test pattern",
		.usage = "[groups]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ftdi_rvswd_command_handlers[] = {
	{
		.name = "ftdi_rvswd",
		.mode = COMMAND_ANY,
		.help = "perform FTDI RVSWD management",
		.chain = ftdi_rvswd_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface ftdi_rvswd_interface = {
	.supported = 0,
	.execute_queue = ftdi_rvswd_execute_queue,
};

struct adapter_driver ftdi_rvswd_adapter_driver = {
	.name = "ftdi_rvswd",
	.transport_ids = TRANSPORT_DDMI,
	.transport_preferred_id = TRANSPORT_DDMI,
	.commands = ftdi_rvswd_command_handlers,

	.init = ftdi_rvswd_initialize,
	.quit = ftdi_rvswd_quit,
	.reset = ftdi_rvswd_reset,
	.speed = ftdi_rvswd_speed,
	.khz = ftdi_rvswd_khz,
	.speed_div = ftdi_rvswd_speed_div,

	.jtag_ops = &ftdi_rvswd_interface,
};
