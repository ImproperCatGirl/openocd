/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_DMI_H
#define OPENOCD_TARGET_RISCV_DMI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "batch.h"

struct target;
struct riscv_dtm;
struct jtag_tap;

struct riscv_dmi_info {
	unsigned int dtm_version;
	unsigned int abits;
	unsigned int idle;
	bool has_dtmcs;
};

struct riscv_dmi_direct_ops {
	int (*read)(uint32_t address, uint32_t *value);
	int (*write)(uint32_t address, uint32_t value);
	int (*reset)(void);
	void (*batch_exec)(void);
};

struct riscv_dmi_backend_ops {
	const char *name;
	int (*get_info)(struct target *target, struct riscv_dmi_info *info);
	int (*reset)(struct target *target);
	int (*prepare_access)(struct target *target);

	struct riscv_batch *(*batch_alloc)(struct target *target, size_t scans);
	void (*batch_free)(struct riscv_batch *batch);
	int (*batch_run_from)(struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after);
	void (*batch_add_dmi_write)(struct riscv_batch *batch, uint32_t address,
		uint32_t data, bool read_back, enum riscv_scan_delay_class delay_class);
	size_t (*batch_add_dmi_read)(struct riscv_batch *batch, uint32_t address,
		enum riscv_scan_delay_class delay_class);
	uint32_t (*batch_get_dmi_read_op)(const struct riscv_batch *batch, size_t key);
	uint32_t (*batch_get_dmi_read_data)(const struct riscv_batch *batch, size_t key);
	size_t (*batch_available_scans)(struct riscv_batch *batch);
	bool (*batch_was_busy)(const struct riscv_batch *batch);
	size_t (*batch_finished_scans)(const struct riscv_batch *batch);
};

extern const struct riscv_dmi_backend_ops riscv_dmi_jtag_backend;
extern const struct riscv_dmi_backend_ops riscv_dmi_direct_backend;
extern const struct riscv_dmi_backend_ops riscv_dmi_ap_backend;

int riscv_dmi_select(struct target *target);
int riscv_dmi_get_info(struct target *target, struct riscv_dmi_info *info);
int riscv_dmi_reset(struct target *target);
int riscv_dmi_prepare_access(struct target *target);
const struct riscv_dmi_backend_ops *riscv_dmi_backend(struct target *target);

void riscv_dmi_direct_register_ops(const struct riscv_dmi_direct_ops *ops);
int riscv_dmi_direct_read(uint32_t address, uint32_t *value);
int riscv_dmi_direct_write(uint32_t address, uint32_t value);
int riscv_dmi_direct_reset(void);
void riscv_dmi_direct_batch_exec(void);

int riscv_dmi_jtag_set_ir(const char *ir_name, uint32_t value);
int riscv_dmi_jtag_use_bscan_tunnel(uint8_t ir_width, int tunnel_type);
int riscv_dmi_jtag_set_bscan_tunnel_ir(int ir_id);
int riscv_dmi_jtag_init_tap(struct jtag_tap *tap);

#endif /* OPENOCD_TARGET_RISCV_DMI_H */
