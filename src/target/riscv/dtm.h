/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_DTM_H
#define OPENOCD_TARGET_RISCV_DTM_H

#include <stdint.h>
#include <stdbool.h>

#include "jtag/jtag.h"
#include "helper/command.h"

struct target;
struct riscv_dmi_backend_ops;
struct adiv5_ap;
struct adiv5_dap;

enum riscv_dtm_type {
	RISCV_DTM_TYPE_JTAG,
	RISCV_DTM_TYPE_DDMI,
	RISCV_DTM_TYPE_AP,
	RISCV_DTM_TYPE_UNKNOWN,
};

struct riscv_dmi_ap_config {
	struct adiv5_dap *dap;
	uint64_t ap_num;
	struct adiv5_ap *ap;
};

struct riscv_dtm {
	const char *name;
	enum riscv_dtm_type type;
	const struct riscv_dmi_backend_ops *backend;
	void *backend_priv;
	struct jtag_tap *tap;
	bool initialized;

	unsigned int dtm_version;
	unsigned int abits;
	unsigned int idle;
	bool has_dtmcs;
	int reset_delays_wait;
};

const char *riscv_dtm_name(const struct riscv_dtm *dtm);
struct riscv_dtm *riscv_dtm_by_name(const char *name);
struct riscv_dtm *riscv_dtm_by_jim_obj(Jim_Interp *interp, Jim_Obj *obj);
int riscv_dtm_init(struct riscv_dtm *dtm);
int riscv_dtm_init_all(void);
int riscv_dtm_assign_target(struct target *target, struct riscv_dtm *dtm);
int riscv_dtm_assign_ap_target(struct target *target, struct adiv5_dap *dap,
		uint64_t ap_num);
int riscv_dtm_assign_implicit(struct target *target);
void riscv_dtm_update_info(struct riscv_dtm *dtm,
		unsigned int dtm_version, unsigned int abits, unsigned int idle,
		bool has_dtmcs);

int riscv_dtm_register_commands(struct command_context *cmd_ctx);
void riscv_dtm_cleanup_all(void);

int riscv_dtm_set_ir(const char *ir_name, uint32_t value);
int riscv_dtm_use_bscan_tunnel(uint8_t ir_width, int tunnel_type);
int riscv_dtm_set_bscan_tunnel_ir(int ir_id);
void riscv_dtm_reset_delays(struct riscv_dtm *dtm, int wait);
int riscv_dtm_configure_ap(struct riscv_dtm *dtm, struct adiv5_dap *dap,
		uint64_t ap_num);

#endif /* OPENOCD_TARGET_RISCV_DTM_H */
