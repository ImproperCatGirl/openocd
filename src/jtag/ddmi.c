#include "interface.h"
#include "ddmi.h"
#include <transport/transport.h>

extern struct adapter_driver *adapter_driver;  // From interface.h

struct ddmi_driver *ddmi_driver;

// Bridge your driver functions to target API
int ddmi_read(struct target *target, uint32_t *value, uint32_t address)
{
    assert(ddmi_driver && ddmi_driver->dmi_read);
    return ddmi_driver->dmi_read(address, (uint32_t*) value);
}

int ddmi_write(struct target *target, unsigned int address, uint32_t value)
{
    assert(ddmi_driver && ddmi_driver->dmi_write);
    return ddmi_driver->dmi_write(address, value);
}

int ddmi_dmi_reset(void)
{
    assert(ddmi_driver && ddmi_driver->srst);
    return ddmi_driver->srst(0,0);
}

// Your dummy implementations (replace with real PIO/USB)
static int dummy_dmi_read(uint8_t address, uint32_t *value)
{
    LOG_WARNING("dummy DMI read 0x%08x", address);
    *value = 0xdeadbeef;
    return ERROR_OK;
}

static int dummy_dmi_write(uint8_t address, uint32_t value)
{
    LOG_WARNING("dummy DMI write 0x%08x=0x%08x", address, value);
    return ERROR_OK;
}

static int dummy_srst(int srst, int trst)
{
    ddmi_dmi_reset();
    LOG_INFO("ddmi system reset");
    return ERROR_OK;
}

static int dummy_init(void)
{
    LOG_INFO("ddmi initialized");
    return ERROR_OK;
}

static struct ddmi_driver dummy_ddmi_driver = {
    .init = dummy_init,
    .dmi_read = dummy_dmi_read,
    .dmi_write = dummy_dmi_write,
    .srst = dummy_srst,
};

COMMAND_HANDLER(handle_ddmi_newtap_command)
{
	struct jtag_tap *tap;

	/*
	 * only need "basename" and "tap_type", but for backward compatibility
	 * ignore extra parameters
	 */
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	tap = calloc(1, sizeof(*tap));
	if (!tap) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	tap->chip = strdup(CMD_ARGV[0]);
	tap->tapname = strdup(CMD_ARGV[1]);
	tap->dotted_name = alloc_printf("%s.%s", CMD_ARGV[0], CMD_ARGV[1]);
	if (!tap->chip || !tap->tapname || !tap->dotted_name) {
		LOG_ERROR("Out of memory");
		free(tap->dotted_name);
		free(tap->tapname);
		free(tap->chip);
		free(tap);
		return ERROR_FAIL;
	}

	LOG_DEBUG("Creating new ddmi \"tap\", Chip: %s, Tap: %s, Dotted: %s",
			  tap->chip, tap->tapname, tap->dotted_name);

	/* default is enabled-after-reset */
	tap->enabled = true;
    
	jtag_tap_init(tap);
	return ERROR_OK;
}
// Transport command handlers (SWIM-style)
static const struct command_registration ddmi_transport_subcommand_handlers[] = {
    {
		.name = "newtap",
		.handler = handle_ddmi_newtap_command,
		.mode = COMMAND_CONFIG,
		.help = "Create a new TAP instance named basename.tap_type, "
				"and appends it to the scan chain.",
		.usage = "basename tap_type",
	},
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration ddmi_transport_command_handlers[] = {
    {
        .name = "ddmi",
        .mode = COMMAND_ANY,
        .help = "ddmi transport commands",
        .usage = "",
        .chain = ddmi_transport_subcommand_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

static int ddmi_transport_select(struct command_context *cmd_ctx)
{
    LOG_INFO("ddmi transport selected");
    return register_commands(cmd_ctx, NULL, ddmi_transport_command_handlers);
}

static int ddmi_transport_init(struct command_context *cmd_ctx)
{
    return 0;
    LOG_DEBUG("ddmi transport init");
    ddmi_dmi_reset();
    // Validate DMI (read dmstatus)
    uint32_t dmstatus;
    if (ddmi_read(NULL, &dmstatus, 0x11) != ERROR_OK) {
        LOG_ERROR("ddmi: Failed to read dmstatus");
        return ERROR_FAIL;
    }
    LOG_INFO("ddmi: dmstatus=0x%08x", dmstatus);
    enum reset_types jtag_reset_config = jtag_get_reset_config();

	LOG_DEBUG(__func__);

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			adapter_assert_reset();
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	} else
		adapter_deassert_reset();
    
    return ERROR_OK;
}

static struct transport ddmi_transport = {
    .id = TRANSPORT_DDMI,
    .select = ddmi_transport_select,
    .init = ddmi_transport_init,
};

static void ddmi_constructor(void) __attribute__((constructor));
static void ddmi_constructor(void)
{
    transport_register(&ddmi_transport);
    ddmi_driver = &dummy_ddmi_driver;  // Replace with real driver later
}

bool transport_is_ddmi(void)
{
    return get_current_transport() == &ddmi_transport;
}