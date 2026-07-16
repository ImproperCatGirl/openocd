// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "dtm.h"
#include "dmi.h"
#include "riscv.h"

#include "helper/list.h"
#include "helper/log.h"
#include "target/arm_adi_v5.h"
#include "transport/transport.h"

#include <inttypes.h>
#include <string.h>

struct riscv_dtm_object {
	struct list_head lh;
	struct riscv_dtm dtm;
	char *name;
	bool implicit;
};

static OOCD_LIST_HEAD(all_dtm);
static unsigned int implicit_dtm_count;

static const struct command_registration dtm_instance_commands[];

static const struct {
	const char *name;
	enum riscv_dtm_type type;
} dtm_type_names[] = {
	{ "jtag", RISCV_DTM_TYPE_JTAG },
	{ "ddmi", RISCV_DTM_TYPE_DDMI },
	{ "ap", RISCV_DTM_TYPE_AP },
};

static const char *riscv_dtm_type_name(enum riscv_dtm_type type)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(dtm_type_names); i++) {
		if (dtm_type_names[i].type == type)
			return dtm_type_names[i].name;
	}
	return "unknown";
}

static enum riscv_dtm_type riscv_dtm_type_by_name(const char *name)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(dtm_type_names); i++) {
		if (!strcmp(dtm_type_names[i].name, name))
			return dtm_type_names[i].type;
	}
	return RISCV_DTM_TYPE_UNKNOWN;
}

const char *riscv_dtm_name(const struct riscv_dtm *dtm)
{
	return dtm && dtm->name ? dtm->name : "<unnamed-dtm>";
}

struct riscv_dtm *riscv_dtm_by_name(const char *name)
{
	struct riscv_dtm_object *obj;

	list_for_each_entry(obj, &all_dtm, lh) {
		if (!strcmp(name, obj->name))
			return &obj->dtm;
	}

	return NULL;
}

struct riscv_dtm *riscv_dtm_by_jim_obj(Jim_Interp *interp, Jim_Obj *obj)
{
	return riscv_dtm_by_name(Jim_GetString(obj, NULL));
}

static int riscv_dtm_set_backend(struct riscv_dtm *dtm)
{
	switch (dtm->type) {
	case RISCV_DTM_TYPE_JTAG:
		dtm->backend = &riscv_dmi_jtag_backend;
		return ERROR_OK;
	case RISCV_DTM_TYPE_DDMI:
		dtm->backend = &riscv_dmi_direct_backend;
		return ERROR_OK;
	case RISCV_DTM_TYPE_AP:
		dtm->backend = &riscv_dmi_ap_backend;
		return ERROR_OK;
	default:
		LOG_ERROR("Unsupported RISC-V DTM type '%s'.",
				riscv_dtm_type_name(dtm->type));
		return ERROR_FAIL;
	}
}

int riscv_dtm_init(struct riscv_dtm *dtm)
{
	if (!dtm)
		return ERROR_FAIL;
	if (dtm->initialized)
		return ERROR_OK;

	int result = riscv_dtm_set_backend(dtm);
	if (result != ERROR_OK)
		return result;

	if (dtm->type == RISCV_DTM_TYPE_JTAG && !dtm->tap) {
		LOG_ERROR("RISC-V DTM '%s' has no JTAG TAP.", riscv_dtm_name(dtm));
		return ERROR_FAIL;
	}
	if (dtm->type == RISCV_DTM_TYPE_JTAG) {
		result = riscv_dmi_jtag_init_tap(dtm->tap);
		if (result != ERROR_OK)
			return result;
	}
	if (dtm->type == RISCV_DTM_TYPE_AP) {
		struct riscv_dmi_ap_config *config = dtm->backend_priv;
		if (!config || !config->dap || config->ap_num == DP_APSEL_INVALID) {
			LOG_ERROR("RISC-V DTM '%s' needs -dap and -ap-num.",
					riscv_dtm_name(dtm));
			return ERROR_FAIL;
		}
		if (!config->ap) {
			config->ap = dap_get_ap(config->dap, config->ap_num);
			if (!config->ap) {
				LOG_ERROR("Cannot use DAP '%s' AP 0x%08" PRIx64
						" for RISC-V DMI access.",
						adiv5_dap_name(config->dap), config->ap_num);
				return ERROR_FAIL;
			}
		}
	}

	dtm->initialized = true;
	return ERROR_OK;
}

int riscv_dtm_init_all(void)
{
	struct riscv_dtm_object *obj;

	list_for_each_entry(obj, &all_dtm, lh) {
		int result = riscv_dtm_init(&obj->dtm);
		if (result != ERROR_OK)
			return result;
	}

	return ERROR_OK;
}

int riscv_dtm_assign_target(struct target *target, struct riscv_dtm *dtm)
{
	if (!target || !dtm)
		return ERROR_FAIL;

	struct riscv_private_config *config = target->private_config;
	if (!config) {
		config = calloc(1, sizeof(*config));
		if (!config)
			return ERROR_FAIL;
		target->private_config = config;
	}

	config->dtm = dtm;
	target->has_dtm = true;
	target->dtm_configured = true;
	if (target->arch_info) {
		RISCV_INFO(r);
		r->dtm = dtm;
	}

	if (dtm->type == RISCV_DTM_TYPE_JTAG && dtm->tap) {
		target->tap = dtm->tap;
		target->tap_configured = true;
	}

	return ERROR_OK;
}

static struct riscv_dtm *riscv_dtm_find_implicit_jtag(struct jtag_tap *tap)
{
	struct riscv_dtm_object *obj;

	list_for_each_entry(obj, &all_dtm, lh) {
		if (obj->implicit && obj->dtm.type == RISCV_DTM_TYPE_JTAG
				&& obj->dtm.tap == tap)
			return &obj->dtm;
	}

	return NULL;
}

static struct riscv_dtm *riscv_dtm_find_implicit_ap(struct adiv5_dap *dap,
		uint64_t ap_num)
{
	struct riscv_dtm_object *obj;

	list_for_each_entry(obj, &all_dtm, lh) {
		struct riscv_dmi_ap_config *config = obj->dtm.backend_priv;
		if (obj->implicit && obj->dtm.type == RISCV_DTM_TYPE_AP
				&& config && config->dap == dap && config->ap_num == ap_num)
			return &obj->dtm;
	}

	return NULL;
}

static struct riscv_dtm *riscv_dtm_create_object(const char *name,
		enum riscv_dtm_type type, struct jtag_tap *tap, bool implicit)
{
	struct riscv_dtm_object *obj = calloc(1, sizeof(*obj));
	if (!obj)
		return NULL;

	obj->name = strdup(name);
	if (!obj->name) {
		free(obj);
		return NULL;
	}

	obj->implicit = implicit;
	obj->dtm.name = obj->name;
	obj->dtm.type = type;
	obj->dtm.tap = tap;
	obj->dtm.reset_delays_wait = -1;
	if (riscv_dtm_set_backend(&obj->dtm) != ERROR_OK) {
		free(obj->name);
		free(obj);
		return NULL;
	}

	list_add_tail(&obj->lh, &all_dtm);
	return &obj->dtm;
}

static struct riscv_dtm_object *riscv_dtm_object_by_dtm(struct riscv_dtm *dtm)
{
	return container_of(dtm, struct riscv_dtm_object, dtm);
}

static void riscv_dtm_destroy_object(struct riscv_dtm_object *obj)
{
	struct riscv_dmi_ap_config *config = obj->dtm.backend_priv;
	if (config) {
		if (config->ap)
			dap_put_ap(config->ap);
		free(config);
	}
	list_del(&obj->lh);
	free(obj->name);
	free(obj);
}

int riscv_dtm_configure_ap(struct riscv_dtm *dtm, struct adiv5_dap *dap,
		uint64_t ap_num)
{
	if (!dtm || !dap || ap_num == DP_APSEL_INVALID)
		return ERROR_FAIL;

	struct riscv_dmi_ap_config *config = dtm->backend_priv;
	if (!config) {
		config = calloc(1, sizeof(*config));
		if (!config)
			return ERROR_FAIL;
		config->ap_num = DP_APSEL_INVALID;
		dtm->backend_priv = config;
	}

	if (config->ap && (config->dap != dap || config->ap_num != ap_num)) {
		dap_put_ap(config->ap);
		config->ap = NULL;
	}
	config->dap = dap;
	config->ap_num = ap_num;
	dtm->initialized = false;
	return ERROR_OK;
}

int riscv_dtm_assign_implicit(struct target *target)
{
	if (!target)
		return ERROR_FAIL;

	struct riscv_private_config *config = target->private_config;
	if (config && config->dtm)
		return riscv_dtm_assign_target(target, config->dtm);

	struct riscv_dtm *dtm = NULL;
	enum riscv_dtm_type type = RISCV_DTM_TYPE_UNKNOWN;
	if (transport_is_ddmi()) {
		type = RISCV_DTM_TYPE_DDMI;
	} else if (transport_is_jtag()) {
		type = RISCV_DTM_TYPE_JTAG;
		dtm = riscv_dtm_find_implicit_jtag(target->tap);
	} else {
		LOG_TARGET_ERROR(target, "Unsupported RISC-V DTM transport '%s'.",
				get_current_transport_name());
		return ERROR_FAIL;
	}

	if (!dtm) {
		char name[64];
		snprintf(name, sizeof(name), "__implicit_riscv_dtm_%u",
				implicit_dtm_count++);
		dtm = riscv_dtm_create_object(name, type, target->tap, true);
		if (!dtm)
			return ERROR_FAIL;
	}

	return riscv_dtm_assign_target(target, dtm);
}

int riscv_dtm_assign_ap_target(struct target *target, struct adiv5_dap *dap,
		uint64_t ap_num)
{
	if (!target || !dap || ap_num == DP_APSEL_INVALID)
		return ERROR_FAIL;

	struct riscv_dtm *dtm = riscv_dtm_find_implicit_ap(dap, ap_num);
	if (!dtm) {
		char name[64];
		snprintf(name, sizeof(name), "__implicit_riscv_ap_dtm_%u",
				implicit_dtm_count++);
		dtm = riscv_dtm_create_object(name, RISCV_DTM_TYPE_AP, NULL, true);
		if (!dtm)
			return ERROR_FAIL;
		if (riscv_dtm_configure_ap(dtm, dap, ap_num) != ERROR_OK) {
			riscv_dtm_destroy_object(riscv_dtm_object_by_dtm(dtm));
			return ERROR_FAIL;
		}
	}

	return riscv_dtm_assign_target(target, dtm);
}

void riscv_dtm_update_info(struct riscv_dtm *dtm,
		unsigned int dtm_version, unsigned int abits, unsigned int idle,
		bool has_dtmcs)
{
	if (!dtm)
		return;
	dtm->dtm_version = dtm_version;
	dtm->abits = abits;
	dtm->idle = idle;
	dtm->has_dtmcs = has_dtmcs;
}

void riscv_dtm_reset_delays(struct riscv_dtm *dtm, int wait)
{
	if (dtm)
		dtm->reset_delays_wait = wait;
}

void riscv_dtm_cleanup_all(void)
{
	struct riscv_dtm_object *obj, *tmp;

	list_for_each_entry_safe(obj, tmp, &all_dtm, lh) {
		riscv_dtm_destroy_object(obj);
	}
}

static int dtm_parse_options(struct command_invocation *cmd,
		struct riscv_dtm *dtm, unsigned int start_arg)
{
	for (unsigned int i = start_arg; i < CMD_ARGC; ) {
		if (!strcmp(CMD_ARGV[i], "-type")) {
			if (++i == CMD_ARGC)
				return ERROR_COMMAND_SYNTAX_ERROR;
			enum riscv_dtm_type type = riscv_dtm_type_by_name(CMD_ARGV[i++]);
			if (type == RISCV_DTM_TYPE_UNKNOWN) {
				command_print(cmd, "Unknown DTM type '%s'", CMD_ARGV[i - 1]);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			dtm->type = type;
		} else if (!strcmp(CMD_ARGV[i], "-chain-position")) {
			if (++i == CMD_ARGC)
				return ERROR_COMMAND_SYNTAX_ERROR;
			dtm->tap = jtag_tap_by_string(CMD_ARGV[i]);
			if (!dtm->tap) {
				command_print(cmd, "Tap '%s' could not be found", CMD_ARGV[i]);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			i++;
		} else if (!strcmp(CMD_ARGV[i], "-dap")) {
			if (++i == CMD_ARGC)
				return ERROR_COMMAND_SYNTAX_ERROR;
			struct adiv5_dap *dap = dap_instance_by_jim_obj(cmd->ctx->interp,
					CMD_JIMTCL_ARGV[i]);
			if (!dap) {
				command_print(cmd, "DAP '%s' could not be found", CMD_ARGV[i]);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			struct riscv_dmi_ap_config *config = dtm->backend_priv;
			if (!config) {
				config = calloc(1, sizeof(*config));
				if (!config)
					return ERROR_FAIL;
				config->ap_num = DP_APSEL_INVALID;
				dtm->backend_priv = config;
			}
			if (config->ap && config->dap != dap) {
				dap_put_ap(config->ap);
				config->ap = NULL;
			}
			config->dap = dap;
			i++;
		} else if (!strcmp(CMD_ARGV[i], "-ap-num")) {
			if (++i == CMD_ARGC)
				return ERROR_COMMAND_SYNTAX_ERROR;
			jim_wide ap_num;
			int e = Jim_GetWide(cmd->ctx->interp, CMD_JIMTCL_ARGV[i], &ap_num);
			if (e != JIM_OK)
				return ERROR_COMMAND_ARGUMENT_INVALID;
			if (ap_num < 0 || (ap_num > DP_APSEL_MAX && (ap_num & 0xfff))) {
				command_print(cmd, "Invalid AP number");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			struct riscv_dmi_ap_config *config = dtm->backend_priv;
			if (!config) {
				config = calloc(1, sizeof(*config));
				if (!config)
					return ERROR_FAIL;
				dtm->backend_priv = config;
			}
			if (config->ap && config->ap_num != (uint64_t)ap_num) {
				dap_put_ap(config->ap);
				config->ap = NULL;
			}
			config->ap_num = ap_num;
			i++;
		} else {
			command_print(cmd, "Unknown DTM option '%s'", CMD_ARGV[i]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	if (dtm->type == RISCV_DTM_TYPE_JTAG && !dtm->tap) {
		command_print(cmd, "-chain-position is required for JTAG DTM");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	if (dtm->type == RISCV_DTM_TYPE_AP) {
		struct riscv_dmi_ap_config *config = dtm->backend_priv;
		if (!config || !config->dap || config->ap_num == DP_APSEL_INVALID) {
			command_print(cmd, "-dap and -ap-num are required for AP DTM");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	return riscv_dtm_set_backend(dtm);
}

COMMAND_HANDLER(handle_dtm_create)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (riscv_dtm_by_name(CMD_ARGV[0])) {
		command_print(CMD, "DTM '%s' already exists", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (Jim_GetCommand(CMD_CTX->interp, CMD_JIMTCL_ARGV[0], JIM_NONE)) {
		command_print(CMD, "Command '%s' already exists", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct riscv_dtm *dtm = riscv_dtm_create_object(CMD_ARGV[0],
			RISCV_DTM_TYPE_JTAG, NULL, false);
	if (!dtm)
		return ERROR_FAIL;

	int result = dtm_parse_options(CMD, dtm, 1);
	if (result != ERROR_OK) {
		riscv_dtm_destroy_object(riscv_dtm_object_by_dtm(dtm));
		return result;
	}

	struct riscv_dtm_object *obj = riscv_dtm_object_by_dtm(dtm);
	const struct command_registration instance_commands[] = {
		{
			.name = CMD_ARGV[0],
			.mode = COMMAND_ANY,
			.usage = "",
			.help = "RISC-V DTM instance command group",
			.chain = dtm_instance_commands,
		},
		COMMAND_REGISTRATION_DONE
	};

	return register_commands_with_data(CMD_CTX, NULL, instance_commands, obj);
}

COMMAND_HANDLER(handle_dtm_names)
{
	struct riscv_dtm_object *obj;

	list_for_each_entry(obj, &all_dtm, lh)
		command_print(CMD, "%s", obj->name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_dtm_init)
{
	return riscv_dtm_init_all();
}

COMMAND_HANDLER(handle_dtm_info)
{
	struct riscv_dtm *dtm;
	if (CMD_DATA) {
		if (CMD_ARGC != 0)
			return ERROR_COMMAND_SYNTAX_ERROR;
		dtm = &((struct riscv_dtm_object *)CMD_DATA)->dtm;
	} else {
		if (CMD_ARGC != 1)
			return ERROR_COMMAND_SYNTAX_ERROR;

		dtm = riscv_dtm_by_name(CMD_ARGV[0]);
		if (!dtm) {
			command_print(CMD, "DTM '%s' not found", CMD_ARGV[0]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	command_print(CMD, "name %s", riscv_dtm_name(dtm));
	command_print(CMD, "type %s", riscv_dtm_type_name(dtm->type));
	if (dtm->type == RISCV_DTM_TYPE_AP) {
		struct riscv_dmi_ap_config *config = dtm->backend_priv;
		command_print(CMD, "dap %s", config && config->dap
				? adiv5_dap_name(config->dap) : "<none>");
		command_print(CMD, "ap-num 0x%08" PRIx64,
				config ? config->ap_num : (uint64_t)DP_APSEL_INVALID);
	}
	command_print(CMD, "initialized %u", dtm->initialized ? 1 : 0);
	command_print(CMD, "version %u", dtm->dtm_version);
	command_print(CMD, "abits %u", dtm->abits);
	command_print(CMD, "idle %u", dtm->idle);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_dtm_configure)
{
	if (!CMD_DATA)
		return ERROR_FAIL;

	struct riscv_dtm *dtm = &((struct riscv_dtm_object *)CMD_DATA)->dtm;
	const bool configure = !strcmp(CMD_NAME, "configure");
	if (!configure) {
		if (CMD_ARGC != 1)
			return ERROR_COMMAND_SYNTAX_ERROR;
		if (!strcmp(CMD_ARGV[0], "-type"))
			command_print(CMD, "%s", riscv_dtm_type_name(dtm->type));
		else if (!strcmp(CMD_ARGV[0], "-chain-position") && dtm->tap)
			command_print(CMD, "%s", dtm->tap->dotted_name);
		else if (!strcmp(CMD_ARGV[0], "-dap") && dtm->type == RISCV_DTM_TYPE_AP) {
			struct riscv_dmi_ap_config *config = dtm->backend_priv;
			if (config && config->dap)
				command_print(CMD, "%s", adiv5_dap_name(config->dap));
		} else if (!strcmp(CMD_ARGV[0], "-ap-num") && dtm->type == RISCV_DTM_TYPE_AP) {
			struct riscv_dmi_ap_config *config = dtm->backend_priv;
			if (config)
				command_print(CMD, "0x%08" PRIx64, config->ap_num);
		}
		else
			return ERROR_COMMAND_ARGUMENT_INVALID;
		return ERROR_OK;
	}

	int result = dtm_parse_options(CMD, dtm, 0);
	if (result == ERROR_OK)
		dtm->initialized = false;
	return result;
}

COMMAND_HANDLER(handle_dtm_reset_delays)
{
	if (!CMD_DATA)
		return ERROR_FAIL;

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int wait = 0;
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], wait);

	riscv_dtm_reset_delays(&((struct riscv_dtm_object *)CMD_DATA)->dtm, wait);
	return ERROR_OK;
}

static const struct command_registration dtm_instance_commands[] = {
	{
		.name = "configure",
		.handler = handle_dtm_configure,
		.mode = COMMAND_ANY,
		.usage = "[-type jtag|ddmi|ap] [-chain-position tap] [-dap dap] [-ap-num num]",
		.help = "Configure a RISC-V DTM instance.",
	},
	{
		.name = "cget",
		.handler = handle_dtm_configure,
		.mode = COMMAND_ANY,
		.usage = "-type|-chain-position|-dap|-ap-num",
		.help = "Read a RISC-V DTM instance option.",
	},
	{
		.name = "info",
		.handler = handle_dtm_info,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Show RISC-V DTM information.",
	},
	{
		.name = "reset_delays",
		.handler = handle_dtm_reset_delays,
		.mode = COMMAND_ANY,
		.usage = "[wait]",
		.help = "Reset learned RISC-V DMI delays for this DTM.",
	},
	COMMAND_REGISTRATION_DONE
};

int riscv_dtm_set_ir(const char *ir_name, uint32_t value)
{
	return riscv_dmi_jtag_set_ir(ir_name, value);
}

int riscv_dtm_use_bscan_tunnel(uint8_t ir_width, int tunnel_type)
{
	return riscv_dmi_jtag_use_bscan_tunnel(ir_width, tunnel_type);
}

int riscv_dtm_set_bscan_tunnel_ir(int ir_id)
{
	return riscv_dmi_jtag_set_bscan_tunnel_ir(ir_id);
}

static const struct command_registration dtm_subcommand_handlers[] = {
	{
		.name = "create",
		.handler = handle_dtm_create,
		.mode = COMMAND_CONFIG,
		.usage = "name [-type jtag|ddmi|ap] [-chain-position tap] [-dap dap] [-ap-num num]",
		.help = "Create a named RISC-V DTM instance.",
	},
	{
		.name = "names",
		.handler = handle_dtm_names,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "List RISC-V DTM instances.",
	},
	{
		.name = "init",
		.handler = handle_dtm_init,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Initialize RISC-V DTM instances.",
	},
	{
		.name = "info",
		.handler = handle_dtm_info,
		.mode = COMMAND_ANY,
		.usage = "name",
		.help = "Show RISC-V DTM information.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dtm_commands[] = {
	{
		.name = "dtm",
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "RISC-V DTM command group",
		.chain = dtm_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int riscv_dtm_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, dtm_commands);
}
