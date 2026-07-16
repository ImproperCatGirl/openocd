// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "dmi.h"
#include "dtm.h"
#include "riscv.h"

#include "helper/log.h"

#include <assert.h>
#include <string.h>

static const struct riscv_dmi_direct_ops *direct_ops;

const struct riscv_dmi_backend_ops *riscv_dmi_backend(struct target *target)
{
	RISCV_INFO(r);

	return r->dtm ? r->dtm->backend : NULL;
}

int riscv_dmi_select(struct target *target)
{
	RISCV_INFO(r);

	if (!r->dtm && riscv_dtm_assign_implicit(target) != ERROR_OK)
		return ERROR_FAIL;

	if (!r->dtm)
		return ERROR_FAIL;

	return riscv_dtm_init(r->dtm);
}

int riscv_dmi_get_info(struct target *target, struct riscv_dmi_info *info)
{
	assert(info);
	memset(info, 0, sizeof(*info));

	const struct riscv_dmi_backend_ops *backend = riscv_dmi_backend(target);
	if (!backend || !backend->get_info) {
		LOG_TARGET_ERROR(target, "RISC-V DMI backend is not selected.");
		return ERROR_FAIL;
	}

	int result = backend->get_info(target, info);
	if (result == ERROR_OK) {
		RISCV_INFO(r);
		riscv_dtm_update_info(r->dtm, info->dtm_version, info->abits,
				info->idle, info->has_dtmcs);
	}

	return result;
}

int riscv_dmi_reset(struct target *target)
{
	const struct riscv_dmi_backend_ops *backend = riscv_dmi_backend(target);
	if (!backend || !backend->reset)
		return ERROR_FAIL;

	return backend->reset(target);
}

int riscv_dmi_prepare_access(struct target *target)
{
	const struct riscv_dmi_backend_ops *backend = riscv_dmi_backend(target);
	if (!backend || !backend->prepare_access)
		return ERROR_FAIL;

	return backend->prepare_access(target);
}

void riscv_dmi_direct_register_ops(const struct riscv_dmi_direct_ops *ops)
{
	direct_ops = ops;
}

int riscv_dmi_direct_read(uint32_t address, uint32_t *value)
{
	if (!direct_ops || !direct_ops->read) {
		LOG_ERROR("No direct RISC-V DMI read operation is registered.");
		return ERROR_FAIL;
	}

	return direct_ops->read(address, value);
}

int riscv_dmi_direct_write(uint32_t address, uint32_t value)
{
	if (!direct_ops || !direct_ops->write) {
		LOG_ERROR("No direct RISC-V DMI write operation is registered.");
		return ERROR_FAIL;
	}

	return direct_ops->write(address, value);
}

int riscv_dmi_direct_reset(void)
{
	if (direct_ops && direct_ops->reset)
		return direct_ops->reset();

	return ERROR_OK;
}

void riscv_dmi_direct_batch_exec(void)
{
	if (direct_ops && direct_ops->batch_exec)
		direct_ops->batch_exec();
}
