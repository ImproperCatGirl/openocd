// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "internal.h"

#include "target/riscv/debug_defines.h"
#include "target/riscv/riscv.h"

#include "helper/log.h"
#include "target/arm_adi_v5.h"

#include <assert.h>

static struct riscv_dmi_ap_config *riscv_dmi_ap_config(struct target *target)
{
	RISCV_INFO(r);

	if (!r->dtm)
		return NULL;
	return r->dtm->backend_priv;
}

static int riscv_dmi_get_info_ap(struct target *target,
		struct riscv_dmi_info *info)
{
	info->dtm_version = DTM_DTMCS_VERSION_1_0;
	info->abits = RISCV013_DTMCS_ABITS_MAX;
	info->idle = 0;
	info->has_dtmcs = false;
	return ERROR_OK;
}

static int riscv_dmi_reset_ap(struct target *target)
{
	return ERROR_OK;
}

static int riscv_dmi_prepare_access_ap(struct target *target)
{
	struct riscv_dmi_ap_config *config = riscv_dmi_ap_config(target);
	if (!config || !config->ap) {
		LOG_TARGET_ERROR(target, "RISC-V AP-backed DTM is not initialized.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static struct riscv_batch *riscv_batch_alloc_ap(struct target *target,
		size_t scans)
{
	return riscv_dmi_direct_batch_alloc(target, scans, &riscv_dmi_ap_backend);
}

static int riscv_batch_run_from_ap(struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	assert(start_idx == 0);

	struct riscv_dmi_ap_config *config = riscv_dmi_ap_config(batch->target);
	if (!config || !config->ap)
		return ERROR_FAIL;

	struct riscv_dmi_direct_batch *direct = batch->backend_priv;
	assert(direct);

	keep_alive();
	for (size_t i = 0; i < direct->used_ops; ++i) {
		int result = ERROR_OK;
		struct riscv_dmi_direct_op *op = &direct->ops[i];

		if (op->opcode == RV_OP_READ) {
			result = mem_ap_read_u32(config->ap, op->params.read.addr * 4,
				&op->params.read.data_from_target);
		}
		if (op->opcode == RV_OP_WRITE) {
			result = mem_ap_write_u32(config->ap, op->params.write.addr * 4,
				op->params.write.data_to_target);
		}
		if (result != ERROR_OK)
			return result;
	}

	int result = dap_run(config->ap->dap);
	if (result != ERROR_OK)
		return result;

	keep_alive();
	return ERROR_OK;
}

const struct riscv_dmi_backend_ops riscv_dmi_ap_backend = {
	.name = "ap",
	.get_info = riscv_dmi_get_info_ap,
	.reset = riscv_dmi_reset_ap,
	.prepare_access = riscv_dmi_prepare_access_ap,
	.batch_alloc = riscv_batch_alloc_ap,
	.batch_free = riscv_dmi_direct_batch_free,
	.batch_run_from = riscv_batch_run_from_ap,
	.batch_add_dmi_write = riscv_dmi_direct_batch_add_write,
	.batch_add_dmi_read = riscv_dmi_direct_batch_add_read,
	.batch_get_dmi_read_op = riscv_dmi_direct_batch_get_read_op,
	.batch_get_dmi_read_data = riscv_dmi_direct_batch_get_read_data,
	.batch_available_scans = riscv_dmi_direct_batch_available_scans,
	.batch_was_busy = riscv_dmi_direct_batch_was_busy,
	.batch_finished_scans = riscv_dmi_direct_batch_finished_scans,
};
