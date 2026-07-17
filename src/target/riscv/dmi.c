// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "dmi.h"
#include "dtm.h"
#include "riscv.h"

#include "helper/log.h"
#include "helper/list.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

struct riscv_dmi_direct_provider_entry {
	struct list_head lh;
	struct riscv_dmi_direct_provider provider;
};

static OOCD_LIST_HEAD(direct_providers);

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
	riscv_dmi_direct_register_provider("legacy", ops);
}

int riscv_dmi_direct_register_provider(const char *name,
		const struct riscv_dmi_direct_ops *ops)
{
	if (!name || !ops)
		return ERROR_FAIL;

	struct riscv_dmi_direct_provider_entry *entry;
	list_for_each_entry(entry, &direct_providers, lh) {
		if (!strcmp(entry->provider.name, name)) {
			entry->provider.ops = ops;
			return ERROR_OK;
		}
	}

	entry = calloc(1, sizeof(*entry));
	if (!entry)
		return ERROR_FAIL;

	entry->provider.name = name;
	entry->provider.ops = ops;
	list_add_tail(&entry->lh, &direct_providers);
	return ERROR_OK;
}

const struct riscv_dmi_direct_provider *riscv_dmi_direct_provider_by_name(
		const char *name)
{
	struct riscv_dmi_direct_provider_entry *entry;
	list_for_each_entry(entry, &direct_providers, lh) {
		if (!strcmp(entry->provider.name, name))
			return &entry->provider;
	}

	return NULL;
}

const struct riscv_dmi_direct_provider *riscv_dmi_direct_provider_for_dtm(
		struct riscv_dtm *dtm)
{
	if (!dtm)
		return NULL;

	if (dtm->direct_provider_name)
		return riscv_dmi_direct_provider_by_name(dtm->direct_provider_name);

	const struct riscv_dmi_direct_provider *provider = NULL;
	unsigned int count = 0;
	struct riscv_dmi_direct_provider_entry *entry;
	list_for_each_entry(entry, &direct_providers, lh) {
		provider = &entry->provider;
		count++;
	}

	if (count == 1)
		return provider;

	return NULL;
}

static const struct riscv_dmi_direct_ops *riscv_dmi_direct_ops_for_target(
		struct target *target)
{
	RISCV_INFO(r);
	const struct riscv_dmi_direct_provider *provider =
		riscv_dmi_direct_provider_for_dtm(r->dtm);
	if (!provider || !provider->ops) {
		LOG_TARGET_ERROR(target, "No RISC-V Direct DMI provider is available.");
		return NULL;
	}

	return provider->ops;
}

int riscv_dmi_direct_read(struct target *target, uint32_t address, uint32_t *value)
{
	const struct riscv_dmi_direct_ops *ops = riscv_dmi_direct_ops_for_target(target);
	if (!ops || !ops->read) {
		LOG_ERROR("No direct RISC-V DMI read operation is registered.");
		return ERROR_FAIL;
	}

	return ops->read(address, value);
}

int riscv_dmi_direct_write(struct target *target, uint32_t address, uint32_t value)
{
	const struct riscv_dmi_direct_ops *ops = riscv_dmi_direct_ops_for_target(target);
	if (!ops || !ops->write) {
		LOG_ERROR("No direct RISC-V DMI write operation is registered.");
		return ERROR_FAIL;
	}

	return ops->write(address, value);
}

int riscv_dmi_direct_reset(struct target *target)
{
	const struct riscv_dmi_direct_ops *ops = riscv_dmi_direct_ops_for_target(target);
	if (ops && ops->reset)
		return ops->reset();

	return ERROR_OK;
}

int riscv_dmi_direct_batch_exec(struct target *target)
{
	const struct riscv_dmi_direct_ops *ops = riscv_dmi_direct_ops_for_target(target);
	if (ops && ops->batch_exec)
		return ops->batch_exec();

	return ERROR_OK;
}
