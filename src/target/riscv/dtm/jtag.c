// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "internal.h"

#include "target/riscv/debug_defines.h"
#include "target/riscv/debug_reg_printer.h"
#include "target/riscv/field_helpers.h"
#include "target/riscv/riscv.h"

#include "helper/binarybuffer.h"
#include "helper/log.h"
#include "jtag/jtag.h"

#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>

#define DTM_DMI_MAX_ADDRESS_LENGTH	((1 << DTM_DTMCS_ABITS_LENGTH) - 1)
#define DMI_SCAN_MAX_BIT_LENGTH \
	(DTM_DMI_MAX_ADDRESS_LENGTH + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH)
#define DMI_SCAN_BUF_SIZE (DIV_ROUND_UP(DMI_SCAN_MAX_BIT_LENGTH, 8))

/* Reserve extra room in the batch for the final DMI NOP. */
#define BATCH_RESERVED_SCANS 1

enum riscv_scan_type {
	RISCV_SCAN_TYPE_INVALID,
	RISCV_SCAN_TYPE_NOP,
	RISCV_SCAN_TYPE_READ,
	RISCV_SCAN_TYPE_WRITE,
};

struct riscv_dmi_jtag_batch {
	size_t allocated_scans;
	size_t used_scans;

	uint8_t *data_out;
	uint8_t *data_in;
	struct scan_field *fields;
	enum riscv_scan_delay_class *delay_classes;

	riscv_bscan_tunneled_scan_context_t *bscan_ctxt;

	enum riscv_scan_type last_scan;
	size_t *read_keys;
	size_t read_keys_used;

	bool was_run;
	unsigned int last_scan_delay;
	bool finalized;
};

static struct riscv_dmi_jtag_batch *riscv_jtag_batch(struct riscv_batch *batch)
{
	return batch->backend_priv;
}

static const struct riscv_dmi_jtag_batch *riscv_jtag_batch_const(
		const struct riscv_batch *batch)
{
	return batch->backend_priv;
}

static unsigned int get_dmi_scan_length(const struct target *target)
{
	const unsigned int abits = riscv_get_dmi_address_bits(target);
	assert(abits > 0);
	assert(abits <= DTM_DMI_MAX_ADDRESS_LENGTH);

	return abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

static int riscv_dmi_get_info_jtag(struct target *target,
		struct riscv_dmi_info *info)
{
	uint32_t dtmcontrol;
	if (dtmcs_scan(target->tap, 0, &dtmcontrol) != ERROR_OK || dtmcontrol == 0) {
		LOG_TARGET_ERROR(target,
			"Could not read dtmcontrol. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTM_DTMCS_VERSION);
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);
	info->has_dtmcs = true;
	return ERROR_OK;
}

static int riscv_dmi_reset_jtag(struct target *target)
{
	return dtmcs_scan(target->tap, DTM_DTMCS_DMIRESET, NULL);
}

static int riscv_dmi_prepare_access_jtag(struct target *target)
{
	struct jtag_tap *tap = target->tap;

	if (bscan_tunnel_ir_width != 0) {
		select_dmi_via_bscan(tap);
		return ERROR_OK;
	}
	if (!tap->enabled)
		LOG_ERROR("BUG: Target's TAP '%s' is disabled!", jtag_tap_name(tap));

	bool need_ir_scan = false;
	for (struct jtag_tap *other_tap = jtag_tap_next_enabled(NULL);
			other_tap; other_tap = jtag_tap_next_enabled(other_tap)) {
		if (other_tap != tap) {
			if (!other_tap->bypass) {
				need_ir_scan = true;
				break;
			}
		} else if (!buf_eq(tap->cur_instr, select_dbus.out_value, tap->ir_length)) {
			need_ir_scan = true;
			break;
		}
	}

	if (need_ir_scan)
		jtag_add_ir_scan(tap, &select_dbus, TAP_IDLE);

	return ERROR_OK;
}

static void riscv_batch_add_nop_jtag(struct riscv_batch *batch);

static struct riscv_batch *riscv_batch_alloc_jtag(struct target *target,
		size_t scans)
{
	scans += BATCH_RESERVED_SCANS;
	struct riscv_batch *out = riscv_dmi_batch_alloc_common(target, scans,
			&riscv_dmi_jtag_backend);
	if (!out) {
		LOG_ERROR("Failed to allocate struct riscv_batch");
		return NULL;
	}

	struct riscv_dmi_jtag_batch *jtag = calloc(1, sizeof(*jtag));
	if (!jtag) {
		LOG_ERROR("Failed to allocate RISC-V JTAG batch state.");
		riscv_dmi_batch_free_common(out);
		return NULL;
	}

	out->backend_priv = jtag;
	jtag->allocated_scans = scans;
	jtag->last_scan = RISCV_SCAN_TYPE_INVALID;
	jtag->was_run = false;
	jtag->last_scan_delay = 0;

	/* FIXME: Allocate only enough buffer space for the target's real DMI
	 * scan length instead of the architectural maximum.
	 */
	jtag->data_out = malloc(sizeof(*jtag->data_out) * scans * DMI_SCAN_BUF_SIZE);
	if (!jtag->data_out) {
		LOG_ERROR("Failed to allocate data_out in RISC-V batch.");
		goto alloc_error;
	}
	jtag->data_in = malloc(sizeof(*jtag->data_in) * scans * DMI_SCAN_BUF_SIZE);
	if (!jtag->data_in) {
		LOG_ERROR("Failed to allocate data_in in RISC-V batch.");
		goto alloc_error;
	}
	jtag->fields = malloc(sizeof(*jtag->fields) * scans);
	if (!jtag->fields) {
		LOG_ERROR("Failed to allocate fields in RISC-V batch.");
		goto alloc_error;
	}
	jtag->delay_classes = malloc(sizeof(*jtag->delay_classes) * scans);
	if (!jtag->delay_classes) {
		LOG_ERROR("Failed to allocate delay_classes in RISC-V batch.");
		goto alloc_error;
	}
	if (bscan_tunnel_ir_width != 0) {
		jtag->bscan_ctxt = malloc(sizeof(*jtag->bscan_ctxt) * scans);
		if (!jtag->bscan_ctxt) {
			LOG_ERROR("Failed to allocate bscan_ctxt in RISC-V batch.");
			goto alloc_error;
		}
	}
	jtag->read_keys = malloc(sizeof(*jtag->read_keys) * scans);
	if (!jtag->read_keys) {
		LOG_ERROR("Failed to allocate read_keys in RISC-V batch.");
		goto alloc_error;
	}

	return out;

alloc_error:
	riscv_dmi_jtag_backend.batch_free(out);
	return NULL;
}

static void riscv_batch_free_jtag(struct riscv_batch *batch)
{
	struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch(batch);
	if (jtag) {
		free(jtag->data_in);
		free(jtag->data_out);
		free(jtag->fields);
		free(jtag->delay_classes);
		free(jtag->bscan_ctxt);
		free(jtag->read_keys);
		free(jtag);
	}
	riscv_dmi_batch_free_common(batch);
}

static bool riscv_batch_was_scan_busy(const struct riscv_batch *batch,
		size_t scan_idx)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	assert(jtag);
	assert(jtag->was_run);
	assert(scan_idx < jtag->used_scans);
	const struct scan_field *field = jtag->fields + scan_idx;
	assert(field->in_value);
	const uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	return get_field(in, DTM_DMI_OP) == DTM_DMI_OP_BUSY;
}

static void add_idle_before_batch(const struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	if (!jtag->was_run)
		return;

	const enum riscv_scan_delay_class delay_class = start_idx > 0
		? jtag->delay_classes[start_idx - 1]
		: RISCV_DELAY_BASE;
	const unsigned int new_delay = riscv_scan_get_delay(delays, delay_class);
	if (new_delay <= jtag->last_scan_delay)
		return;
	const unsigned int idle_change = new_delay - jtag->last_scan_delay;
	LOG_TARGET_DEBUG(batch->target, "Adding %u idle cycles before the batch.",
			idle_change);
	jtag_add_runtest(idle_change, TAP_IDLE);
}

static unsigned int get_delay(const struct riscv_batch *batch, size_t scan_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	assert(jtag);
	assert(scan_idx < jtag->used_scans);
	const bool delays_were_reset = resets_delays
		&& (scan_idx >= reset_delays_after);
	const enum riscv_scan_delay_class delay_class =
		jtag->delay_classes[scan_idx];
	const unsigned int delay = riscv_scan_get_delay(delays, delay_class);
	return delays_were_reset ? 0 : delay;
}

static unsigned int decode_dmi(const struct riscv_batch *batch, char *text,
		uint32_t address, uint32_t data)
{
	static const struct {
		uint32_t address;
		enum riscv_debug_reg_ordinal ordinal;
	} description[] = {
		{DM_DMCONTROL, DM_DMCONTROL_ORDINAL},
		{DM_DMSTATUS, DM_DMSTATUS_ORDINAL},
		{DM_ABSTRACTCS, DM_ABSTRACTCS_ORDINAL},
		{DM_COMMAND, DM_COMMAND_ORDINAL},
		{DM_SBCS, DM_SBCS_ORDINAL}
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(description); i++) {
		if (riscv_get_dmi_address(batch->target, description[i].address)
				== address) {
			const struct riscv_debug_reg_ctx context = {
				.XLEN = { .value = 0, .is_set = false },
				.DXLEN = { .value = 0, .is_set = false },
				.abits = { .value = 0, .is_set = false },
			};
			return riscv_debug_reg_to_s(text, description[i].ordinal,
					context, data, RISCV_DEBUG_REG_HIDE_ALL_0);
		}
	}
	if (text)
		text[0] = '\0';
	return 0;
}

static void log_dmi_decoded(const struct riscv_batch *batch, bool write,
		uint32_t address, uint32_t data)
{
	const size_t size = decode_dmi(batch, NULL, address, data) + 1;
	char * const decoded = malloc(size);
	if (!decoded) {
		LOG_ERROR("Not enough memory to allocate %zu bytes.", size);
		return;
	}
	decode_dmi(batch, decoded, address, data);
	LOG_DEBUG("%s: %s", write ? "write" : "read", decoded);
	free(decoded);
}

static void log_batch(const struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	if (!LOG_LEVEL_IS(LOG_LVL_DEBUG))
		return;

	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	const unsigned int abits = riscv_get_dmi_address_bits(batch->target);

	bool last_scan_was_read = false;
	uint32_t last_scan_address = (uint32_t)(-1);
	if (start_idx > 0) {
		const struct scan_field * const field = &jtag->fields[start_idx - 1];
		assert(field->out_value);
		last_scan_was_read = buf_get_u32(field->out_value, DTM_DMI_OP_OFFSET,
				DTM_DMI_OP_LENGTH) == DTM_DMI_OP_READ;
		last_scan_address = buf_get_u32(field->out_value,
				DTM_DMI_ADDRESS_OFFSET, abits);
	}

	for (size_t i = start_idx; i < jtag->used_scans; ++i) {
		static const char * const op_string[] = {"-", "r", "w", "?"};
		const unsigned int delay = get_delay(batch, i, delays, resets_delays,
				reset_delays_after);
		const struct scan_field * const field = &jtag->fields[i];

		assert(field->out_value);
		const unsigned int out_op = buf_get_u32(field->out_value,
				DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
		const uint32_t out_data = buf_get_u32(field->out_value,
				DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
		const uint32_t out_address = buf_get_u32(field->out_value,
				DTM_DMI_ADDRESS_OFFSET, abits);
		if (field->in_value) {
			static const char * const status_string[] = {
				"+", "?", "F", "b"
			};
			const unsigned int in_op = buf_get_u32(field->in_value,
					DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
			const uint32_t in_data = buf_get_u32(field->in_value,
					DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
			const uint32_t in_address = buf_get_u32(field->in_value,
					DTM_DMI_ADDRESS_OFFSET, abits);

			LOG_DEBUG("%db %s %08" PRIx32 " @%02" PRIx32
					" -> %s %08" PRIx32 " @%02" PRIx32 "; %ui",
					field->num_bits, op_string[out_op], out_data, out_address,
					status_string[in_op], in_data, in_address, delay);

			if (last_scan_was_read && in_op == DTM_DMI_OP_SUCCESS)
				log_dmi_decoded(batch, false, last_scan_address, in_data);
		} else {
			LOG_DEBUG("%db %s %08" PRIx32 " @%02" PRIx32 " -> ?; %ui",
					field->num_bits, op_string[out_op], out_data, out_address,
					delay);
		}

		if (out_op == DTM_DMI_OP_WRITE)
			log_dmi_decoded(batch, true, out_address, out_data);

		last_scan_was_read = out_op == DTM_DMI_OP_READ;
		last_scan_address = out_address;
	}
}

static int riscv_batch_run_from_jtag(struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch(batch);
	if (!jtag->finalized)
		riscv_batch_add_nop_jtag(batch);

	assert(jtag->used_scans);
	assert(start_idx < jtag->used_scans);
	assert(jtag->last_scan == RISCV_SCAN_TYPE_NOP);
	assert(!jtag->was_run || riscv_batch_was_scan_busy(batch, start_idx));
	assert(start_idx == 0 || !riscv_batch_was_scan_busy(batch, start_idx - 1));

	if (jtag->was_run)
		add_idle_before_batch(batch, start_idx, delays);

	LOG_TARGET_DEBUG(batch->target, "Running batch of scans [%zu, %zu)",
			start_idx, jtag->used_scans);

	unsigned int delay = 0;
	for (size_t i = start_idx; i < jtag->used_scans; ++i) {
		if (bscan_tunnel_ir_width != 0)
			riscv_add_bscan_tunneled_scan(batch->target->tap,
					jtag->fields + i, jtag->bscan_ctxt + i);
		else
			jtag_add_dr_scan(batch->target->tap, 1, jtag->fields + i,
					TAP_IDLE);

		delay = get_delay(batch, i, delays, resets_delays,
				reset_delays_after);
		if (delay > 0)
			jtag_add_runtest(delay, TAP_IDLE);
	}

	keep_alive();

	if (jtag_execute_queue() != ERROR_OK) {
		LOG_TARGET_ERROR(batch->target, "Unable to execute JTAG queue");
		return ERROR_FAIL;
	}

	keep_alive();

	if (bscan_tunnel_ir_width != 0) {
		for (size_t i = start_idx; i < jtag->used_scans; ++i) {
			if ((jtag->fields + i)->in_value)
				buffer_shr((jtag->fields + i)->in_value, DMI_SCAN_BUF_SIZE, 1);
		}
	}

	log_batch(batch, start_idx, delays, resets_delays, reset_delays_after);
	jtag->was_run = true;
	jtag->last_scan_delay = delay;
	return ERROR_OK;
}

static void riscv_batch_add_dmi_write_jtag(struct riscv_batch *batch,
		uint32_t address, uint32_t data, bool read_back,
		enum riscv_scan_delay_class delay_class)
{
	struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch(batch);
	assert(jtag->used_scans < jtag->allocated_scans);
	struct scan_field *field = jtag->fields + jtag->used_scans;

	field->num_bits = get_dmi_scan_length(batch->target);
	assert(field->num_bits <= DMI_SCAN_MAX_BIT_LENGTH);

	uint8_t *out_value = jtag->data_out + jtag->used_scans * DMI_SCAN_BUF_SIZE;
	uint8_t *in_value = jtag->data_in + jtag->used_scans * DMI_SCAN_BUF_SIZE;

	field->out_value = out_value;
	riscv_fill_dmi_write(batch->target, out_value, address, data);

	if (read_back) {
		field->in_value = in_value;
		riscv_fill_dm_nop(batch->target, in_value);
	} else {
		field->in_value = NULL;
	}

	jtag->delay_classes[jtag->used_scans] = delay_class;
	jtag->last_scan = RISCV_SCAN_TYPE_WRITE;
	jtag->used_scans++;
}

static size_t riscv_batch_add_dmi_read_jtag(struct riscv_batch *batch,
		uint32_t address, enum riscv_scan_delay_class delay_class)
{
	struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch(batch);
	assert(jtag->used_scans < jtag->allocated_scans);
	struct scan_field *field = jtag->fields + jtag->used_scans;

	field->num_bits = get_dmi_scan_length(batch->target);
	assert(field->num_bits <= DMI_SCAN_MAX_BIT_LENGTH);

	uint8_t *out_value = jtag->data_out + jtag->used_scans * DMI_SCAN_BUF_SIZE;
	uint8_t *in_value = jtag->data_in + jtag->used_scans * DMI_SCAN_BUF_SIZE;

	field->out_value = out_value;
	field->in_value = in_value;
	riscv_fill_dmi_read(batch->target, out_value, address);
	riscv_fill_dm_nop(batch->target, in_value);

	jtag->delay_classes[jtag->used_scans] = delay_class;
	jtag->last_scan = RISCV_SCAN_TYPE_READ;
	jtag->used_scans++;

	jtag->read_keys[jtag->read_keys_used] = jtag->used_scans;
	return jtag->read_keys_used++;
}

static uint32_t riscv_batch_get_dmi_read_op_jtag(
		const struct riscv_batch *batch, size_t key)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	assert(key < jtag->read_keys_used);
	size_t index = jtag->read_keys[key];
	assert(index < jtag->used_scans);
	uint8_t *base = jtag->data_in + DMI_SCAN_BUF_SIZE * index;
	return buf_get_u32(base, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

static uint32_t riscv_batch_get_dmi_read_data_jtag(
		const struct riscv_batch *batch, size_t key)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	assert(key < jtag->read_keys_used);
	size_t index = jtag->read_keys[key];
	assert(index < jtag->used_scans);
	uint8_t *base = jtag->data_in + DMI_SCAN_BUF_SIZE * index;
	return buf_get_u32(base, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
}

static void riscv_batch_add_nop_jtag(struct riscv_batch *batch)
{
	struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch(batch);
	if (jtag->finalized)
		return;

	assert(jtag->used_scans < jtag->allocated_scans);
	struct scan_field *field = jtag->fields + jtag->used_scans;

	field->num_bits = get_dmi_scan_length(batch->target);
	assert(field->num_bits <= DMI_SCAN_MAX_BIT_LENGTH);

	uint8_t *out_value = jtag->data_out + jtag->used_scans * DMI_SCAN_BUF_SIZE;
	uint8_t *in_value = jtag->data_in + jtag->used_scans * DMI_SCAN_BUF_SIZE;

	field->out_value = out_value;
	field->in_value = in_value;
	riscv_fill_dm_nop(batch->target, out_value);
	riscv_fill_dm_nop(batch->target, in_value);

	jtag->delay_classes[jtag->used_scans] = RISCV_DELAY_BASE;
	jtag->last_scan = RISCV_SCAN_TYPE_NOP;
	jtag->used_scans++;
	jtag->finalized = true;
}

static size_t riscv_batch_available_scans_jtag(struct riscv_batch *batch)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	assert(jtag->allocated_scans >= (jtag->used_scans + BATCH_RESERVED_SCANS));
	return jtag->allocated_scans - jtag->used_scans - BATCH_RESERVED_SCANS;
}

static bool riscv_batch_was_batch_busy_jtag(const struct riscv_batch *batch)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	assert(jtag->was_run);
	assert(jtag->used_scans);
	assert(jtag->last_scan == RISCV_SCAN_TYPE_NOP);
	return riscv_batch_was_scan_busy(batch, jtag->used_scans - 1);
}

static size_t riscv_batch_finished_scans_jtag(const struct riscv_batch *batch)
{
	const struct riscv_dmi_jtag_batch *jtag = riscv_jtag_batch_const(batch);
	if (!riscv_batch_was_batch_busy(batch))
		return jtag->used_scans;

	assert(jtag->used_scans);
	size_t first_busy = 0;
	while (!riscv_batch_was_scan_busy(batch, first_busy))
		++first_busy;
	return first_busy;
}

const struct riscv_dmi_backend_ops riscv_dmi_jtag_backend = {
	.name = "jtag",
	.get_info = riscv_dmi_get_info_jtag,
	.reset = riscv_dmi_reset_jtag,
	.prepare_access = riscv_dmi_prepare_access_jtag,
	.batch_alloc = riscv_batch_alloc_jtag,
	.batch_free = riscv_batch_free_jtag,
	.batch_run_from = riscv_batch_run_from_jtag,
	.batch_add_dmi_write = riscv_batch_add_dmi_write_jtag,
	.batch_add_dmi_read = riscv_batch_add_dmi_read_jtag,
	.batch_get_dmi_read_op = riscv_batch_get_dmi_read_op_jtag,
	.batch_get_dmi_read_data = riscv_batch_get_dmi_read_data_jtag,
	.batch_available_scans = riscv_batch_available_scans_jtag,
	.batch_was_busy = riscv_batch_was_batch_busy_jtag,
	.batch_finished_scans = riscv_batch_finished_scans_jtag,
};
