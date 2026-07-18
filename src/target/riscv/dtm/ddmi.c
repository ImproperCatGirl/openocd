// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "internal.h"

#include "target/riscv/debug_defines.h"
#include "target/riscv/field_helpers.h"

#include "helper/log.h"

#include <assert.h>
#include <stdlib.h>

struct riscv_batch *riscv_dmi_batch_alloc_common(struct target *target,
		size_t scans, const struct riscv_dmi_backend_ops *backend)
{
	struct riscv_batch *batch = calloc(1, sizeof(*batch));
	if (!batch)
		return NULL;

	batch->target = target;
	batch->backend = backend;
	return batch;
}

void riscv_dmi_batch_free_common(struct riscv_batch *batch)
{
	free(batch);
}

struct riscv_batch *riscv_dmi_direct_batch_alloc(struct target *target,
		size_t scans, const struct riscv_dmi_backend_ops *backend)
{
	struct riscv_batch *batch = riscv_dmi_batch_alloc_common(target, scans,
			backend);
	if (!batch)
		return NULL;

	struct riscv_dmi_direct_batch *direct = calloc(1, sizeof(*direct));
	if (!direct) {
		riscv_dmi_batch_free_common(batch);
		return NULL;
	}

	direct->allocated_ops = scans;
	direct->ops = calloc(scans, sizeof(*direct->ops));
	direct->read_keys = calloc(scans, sizeof(*direct->read_keys));
	if (!direct->ops || !direct->read_keys) {
		free(direct->ops);
		free(direct->read_keys);
		free(direct);
		riscv_dmi_batch_free_common(batch);
		return NULL;
	}

	batch->backend_priv = direct;
	return batch;
}

void riscv_dmi_direct_batch_free(struct riscv_batch *batch)
{
	struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	if (direct) {
		free(direct->ops);
		free(direct->read_keys);
		free(direct);
	}
	riscv_dmi_batch_free_common(batch);
}

void riscv_dmi_direct_batch_add_write(struct riscv_batch *batch,
		uint32_t address, uint32_t data, bool read_back,
		enum riscv_scan_delay_class delay_class)
{
	struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);
	assert(direct->used_ops < direct->allocated_ops);

	struct riscv_dmi_direct_op *op = &direct->ops[direct->used_ops++];
	op->opcode = RV_OP_WRITE;
	op->params.write.addr = address;
	op->params.write.data_to_target = data;
}

size_t riscv_dmi_direct_batch_add_read(struct riscv_batch *batch,
		uint32_t address, enum riscv_scan_delay_class delay_class)
{
	struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);
	assert(direct->used_ops < direct->allocated_ops);

	struct riscv_dmi_direct_op *op = &direct->ops[direct->used_ops++];
	op->opcode = RV_OP_READ;
	op->params.read.addr = address;
	op->params.read.data_from_target = 0;

	direct->read_keys[direct->read_keys_used] = direct->used_ops - 1;
	return direct->read_keys_used++;
}

uint32_t riscv_dmi_direct_batch_get_read_op(const struct riscv_batch *batch,
		size_t key)
{
	return DTM_DMI_OP_SUCCESS;
}

uint32_t riscv_dmi_direct_batch_get_read_data(const struct riscv_batch *batch,
		size_t key)
{
	const struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);
	assert(key < direct->read_keys_used);
	size_t index = direct->read_keys[key];
	assert(index < direct->used_ops);

	return direct->ops[index].params.read.data_from_target;
}

size_t riscv_dmi_direct_batch_available_scans(struct riscv_batch *batch)
{
	const struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);

	if (direct->used_ops >= direct->allocated_ops)
		return 0;
	return direct->allocated_ops - direct->used_ops;
}

bool riscv_dmi_direct_batch_was_busy(const struct riscv_batch *batch)
{
	return false;
}

size_t riscv_dmi_direct_batch_finished_scans(const struct riscv_batch *batch)
{
	const struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);

	return direct->used_ops;
}

static int riscv_dmi_get_info_direct(struct target *target,
		struct riscv_dmi_info *info)
{
	info->dtm_version = DTM_DTMCS_VERSION_1_0;
	info->abits = RISCV013_DTMCS_ABITS_MAX;
	info->idle = 0;
	info->has_dtmcs = false;
	return ERROR_OK;
}

static int riscv_dmi_reset_direct(struct target *target)
{
	return riscv_dmi_direct_reset(target);
}

static int riscv_dmi_prepare_access_direct(struct target *target)
{
	return ERROR_OK;
}

static struct riscv_batch *riscv_batch_alloc_direct(struct target *target,
		size_t scans)
{
	return riscv_dmi_direct_batch_alloc(target, scans,
			&riscv_dmi_direct_backend);
}

static int riscv_batch_run_from_direct(struct riscv_batch *batch,
		size_t start_idx, const struct riscv_scan_delays *delays,
		bool resets_delays, size_t reset_delays_after)
{
	assert(start_idx == 0);

	struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);

	keep_alive();
	for (size_t i = 0; i < direct->used_ops; ++i) {
		int result = ERROR_OK;

		if (direct->ops[i].opcode == RV_OP_READ) {
			result = riscv_dmi_direct_read(batch->target,
				direct->ops[i].params.read.addr,
				&direct->ops[i].params.read.data_from_target);
		}
		if (direct->ops[i].opcode == RV_OP_WRITE) {
			result = riscv_dmi_direct_write(batch->target,
				direct->ops[i].params.write.addr,
				direct->ops[i].params.write.data_to_target);
		}
		if (result != ERROR_OK)
			return result;
	}

	int result = riscv_dmi_direct_batch_exec(batch->target);
	if (result != ERROR_OK)
		return result;

	keep_alive();
	return ERROR_OK;
}

const struct riscv_dmi_backend_ops riscv_dmi_direct_backend = {
	.name = "ddmi",
	.get_info = riscv_dmi_get_info_direct,
	.reset = riscv_dmi_reset_direct,
	.prepare_access = riscv_dmi_prepare_access_direct,
	.batch_alloc = riscv_batch_alloc_direct,
	.batch_free = riscv_dmi_direct_batch_free,
	.batch_run_from = riscv_batch_run_from_direct,
	.batch_add_dmi_write = riscv_dmi_direct_batch_add_write,
	.batch_add_dmi_read = riscv_dmi_direct_batch_add_read,
	.batch_get_dmi_read_op = riscv_dmi_direct_batch_get_read_op,
	.batch_get_dmi_read_data = riscv_dmi_direct_batch_get_read_data,
	.batch_available_scans = riscv_dmi_direct_batch_available_scans,
	.batch_was_busy = riscv_dmi_direct_batch_was_busy,
	.batch_finished_scans = riscv_dmi_direct_batch_finished_scans,
};
