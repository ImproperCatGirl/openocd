// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "batch.h"
#include "dmi.h"

#include <assert.h>

struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans)
{
	const struct riscv_dmi_backend_ops *backend = riscv_dmi_backend(target);
	assert(backend);
	assert(backend->batch_alloc);

	return backend->batch_alloc(target, scans);
}

void riscv_batch_free(struct riscv_batch *batch)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_free);

	batch->backend->batch_free(batch);
}

bool riscv_batch_full(struct riscv_batch *batch)
{
	return riscv_batch_available_scans(batch) == 0;
}

int riscv_batch_run_from(struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_run_from);

	int result = riscv_dmi_prepare_access(batch->target);
	if (result != ERROR_OK)
		return result;

	return batch->backend->batch_run_from(batch, start_idx, delays,
		resets_delays, reset_delays_after);
}

void riscv_batch_add_dmi_write(struct riscv_batch *batch, uint32_t address,
		uint32_t data, bool read_back,
		enum riscv_scan_delay_class delay_class)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_add_dmi_write);

	batch->backend->batch_add_dmi_write(batch, address, data, read_back,
		delay_class);
}

size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, uint32_t address,
		enum riscv_scan_delay_class delay_class)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_add_dmi_read);

	return batch->backend->batch_add_dmi_read(batch, address, delay_class);
}

uint32_t riscv_batch_get_dmi_read_op(const struct riscv_batch *batch, size_t key)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_get_dmi_read_op);

	return batch->backend->batch_get_dmi_read_op(batch, key);
}

uint32_t riscv_batch_get_dmi_read_data(const struct riscv_batch *batch,
		size_t key)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_get_dmi_read_data);

	return batch->backend->batch_get_dmi_read_data(batch, key);
}

size_t riscv_batch_available_scans(struct riscv_batch *batch)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_available_scans);

	return batch->backend->batch_available_scans(batch);
}

bool riscv_batch_was_batch_busy(const struct riscv_batch *batch)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_was_busy);

	return batch->backend->batch_was_busy(batch);
}

size_t riscv_batch_finished_scans(const struct riscv_batch *batch)
{
	assert(batch);
	assert(batch->backend);
	assert(batch->backend->batch_finished_scans);

	return batch->backend->batch_finished_scans(batch);
}
