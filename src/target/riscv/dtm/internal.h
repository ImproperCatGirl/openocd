/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_DTM_INTERNAL_H
#define OPENOCD_TARGET_RISCV_DTM_INTERNAL_H

#include "target/riscv/batch.h"
#include "target/riscv/dmi.h"
#include "target/riscv/dtm.h"

enum riscv_dmi_direct_opcode {
	RV_OP_INVALID = 0,
	RV_OP_READ,
	RV_OP_WRITE,
};

struct riscv_dmi_direct_op {
	enum riscv_dmi_direct_opcode opcode;
	union {
		struct {
			unsigned int addr;
			uint32_t data_from_target;
		} read;
		struct {
			unsigned int addr;
			uint32_t data_to_target;
		} write;
	} params;
};

struct riscv_dmi_direct_batch {
	size_t allocated_ops;
	size_t used_ops;
	struct riscv_dmi_direct_op *ops;
	size_t *read_keys;
	size_t read_keys_used;
};

struct riscv_batch *riscv_dmi_batch_alloc_common(struct target *target,
		size_t scans, const struct riscv_dmi_backend_ops *backend);
void riscv_dmi_batch_free_common(struct riscv_batch *batch);

struct riscv_batch *riscv_dmi_direct_batch_alloc(struct target *target,
		size_t scans, const struct riscv_dmi_backend_ops *backend);
void riscv_dmi_direct_batch_free(struct riscv_batch *batch);
void riscv_dmi_direct_batch_add_write(struct riscv_batch *batch,
		uint32_t address, uint32_t data, bool read_back,
		enum riscv_scan_delay_class delay_class);
size_t riscv_dmi_direct_batch_add_read(struct riscv_batch *batch,
		uint32_t address, enum riscv_scan_delay_class delay_class);
uint32_t riscv_dmi_direct_batch_get_read_op(const struct riscv_batch *batch,
		size_t key);
uint32_t riscv_dmi_direct_batch_get_read_data(const struct riscv_batch *batch,
		size_t key);
size_t riscv_dmi_direct_batch_available_scans(struct riscv_batch *batch);
bool riscv_dmi_direct_batch_was_busy(const struct riscv_batch *batch);
size_t riscv_dmi_direct_batch_finished_scans(const struct riscv_batch *batch);

#endif /* OPENOCD_TARGET_RISCV_DTM_INTERNAL_H */
