#include "helper/log.h"
#include "jtag/interface.h"
#include "jtag/ddmi.h"
#include "transport/transport.h"
#include <libusb.h>
#include <stdlib.h>
#include <sys/param.h>

struct ddmi_usb {
    libusb_context *ctx;
    libusb_device_handle *handle;
    uint16_t vid, pid;
};


#define VID 0xCAFE
#define PID 0x4008
#define EP_OUT 0x04 // Bulk OUT endpoint (host to device)
#define EP_IN 0x84  // Bulk IN endpoint (device to host)
#define TIMEOUT_MS 1000

static struct ddmi_usb ddmi_usb_priv;

// Host-side command buffer
struct ddmi_usb_op {
    uint8_t opcode;
    //uint8_t serial;
    uint8_t addr;
    uint32_t data_to_target;  // for writes only
    uint32_t *data_from_target;
} ddmi_usb_op_t;

#define MAX_BATCH 200
#define CMD_SIZE      9
#define RESP_SIZE     4

//static uint8_t next_serial = 1;
//static size_t total_bytes = 3;

typedef enum ddmi_opcode {
    ddmi_WRITE,
    ddmi_READ,
    ddmi_RESET,
} ddmi_opcode_t;


static int ddmi_usb_execute_queue(struct jtag_command *cmd_queue);

struct rv_usb_read_callback {
    uint32_t *dest;      // Pointer provided by the caller (e.g., dummy_dmi_read)
    size_t batch_index;  // Which operation in the batch this belongs to
};

struct rv_usb_batch {
    uint8_t buffer[MAX_BATCH * CMD_SIZE];
    struct rv_usb_read_callback reads[MAX_BATCH];
    size_t num_ops;
    size_t num_reads;
};

static struct rv_usb_batch r_w_buffer;


void batch_init(struct rv_usb_batch *batch) {
    batch->num_ops = 0;
}
// Add a read to the batch and store the destination pointer
void batch_add_read(struct rv_usb_batch *batch, uint32_t addr, uint32_t *dest) {
    if (batch->num_ops >= MAX_BATCH)
    {
        LOG_ERROR("BATCH TOO BIG!\n");
        return;
    } 

    // 1. Setup the command in the USB buffer
    uint8_t *ptr = &batch->buffer[batch->num_ops * CMD_SIZE];
    ptr[0] = 'r';
    memcpy(ptr + 1, &addr, 4);
    memset(ptr + 5, 0, 4); // data_in is 0 for reads

    // 2. Save the pointer to be filled later
    batch->reads[batch->num_reads].dest = dest;
    batch->reads[batch->num_reads].batch_index = batch->num_ops;
    
    batch->num_ops++;
    batch->num_reads++;
}

int batch_add_op(struct rv_usb_batch *batch, char type, uint32_t addr, uint32_t data) {
    if (batch->num_ops >= MAX_BATCH)
    {
        LOG_ERROR("BATCH TOO BIG!\n");
        return -1;
    } 
    uint8_t *ptr = &batch->buffer[batch->num_ops * CMD_SIZE];
    ptr[0] = (uint8_t)type;
    memcpy(ptr + 1, &addr, 4);
    memcpy(ptr + 5, &data, 4);
    
    batch->num_ops++;
    return 0;
}

// ------------------------- Read / Write Wrappers -------------------------
static int ddmi_usb_dmi_read(uint8_t addr, uint32_t *value)
{
    LOG_DEBUG("USB DMI read 0x%08x", addr);
    batch_add_read(&r_w_buffer, addr, value);
    //usb_dmi_op(ddmi_usb_priv.handle, 'r', addr, 0x0, value);
    return 0;
    
}

static int ddmi_usb_dmi_write(uint8_t addr, uint32_t value)
{
    batch_add_op(&r_w_buffer, 'w', addr, value);
    // YOUR REAL PIO IMPLEMENTATION HERE
    LOG_DEBUG("USB DMI write 0x%08x=0x%08x", addr, value);
    return 0;
}
static uint32_t results_scratch[MAX_BATCH];

#define PROBE_MAX_PACKETS 5
#define MAX_OPS_PER_PACKET 6
#define MAX_OPS_PER_CHAIN (PROBE_MAX_PACKETS * MAX_OPS_PER_PACKET) // 60 ops
#define USB_MPS 64

int batch_execute(libusb_device_handle *handle, struct rv_usb_batch *batch) {
    if (!batch || batch->num_ops == 0) return ERROR_OK;

    size_t global_ops_completed = 0;

    // Outer loop: Break the massive OpenOCD batch into "Probe-Sized" chains
    while (global_ops_completed < batch->num_ops) {
        size_t ops_remaining = batch->num_ops - global_ops_completed;
        size_t chain_size = (ops_remaining > MAX_OPS_PER_CHAIN) ? MAX_OPS_PER_CHAIN : ops_remaining;
        
        uint8_t total_packets_in_chain = (chain_size + MAX_OPS_PER_PACKET - 1) / MAX_OPS_PER_PACKET;
        size_t ops_in_this_chain = 0;
        // --- PHASE 1: Send the Chain ---
        for (uint8_t p = 0; p < total_packets_in_chain; p++) {
            size_t ops_in_packet = (chain_size - ops_in_this_chain);
            if (ops_in_packet > MAX_OPS_PER_PACKET) ops_in_packet = MAX_OPS_PER_PACKET;

            uint8_t tx_buf[USB_MPS] = {0};
            tx_buf[0] = (total_packets_in_chain - 1) - p; // Countdown
            tx_buf[1] = (uint8_t)ops_in_packet;           // Op count

            // Copy commands from the global batch buffer
            memcpy(tx_buf + 2, batch->buffer + ((global_ops_completed + ops_in_this_chain) * CMD_SIZE), 
                   ops_in_packet * CMD_SIZE);

            int transferred;
            //printf("this packet has %d commands, total %d bytes long, countdown %d\n", (int)ops_in_packet, USB_MPS, tx_buf[0]);
            libusb_bulk_transfer(handle, EP_OUT, tx_buf, USB_MPS, &transferred, 1000);
            
            ops_in_this_chain += ops_in_packet;
        }

        // --- PHASE 2: Collect Results for this Chain ---
        size_t bytes_to_receive = chain_size * RESP_SIZE;
        size_t received = 0;
        int timeout = 10;
        while (received < bytes_to_receive && timeout -- > 0) {
            int actual;
            // Place results into the correct global offset
            uint8_t *dest_ptr = ((uint8_t*)results_scratch) + (global_ops_completed * RESP_SIZE) + received;
            
            libusb_bulk_transfer(handle, EP_IN, dest_ptr, (int)(bytes_to_receive - received), &actual, 1000);
            //printf("RX actual = %d\n", actual);
            received += actual;
        }

        global_ops_completed += chain_size;
        //printf("global ops completed %d\n",(int) global_ops_completed);
    }

    // --- PHASE 3: Map results to OpenOCD pointers ---
    for (size_t i = 0; i < batch->num_reads; i++) {
        *batch->reads[i].dest = results_scratch[batch->reads[i].batch_index];

        //printf("result = %08X\n", results_scratch[batch->reads[i].batch_index]);

    }

    batch->num_ops = 0;
    batch->num_reads = 0;
    return ERROR_OK;
}


void ddmi_init_reset(libusb_device_handle *handle)
{
    uint8_t buf[3] = {0, 255, 'R'};
    int actual;
    libusb_bulk_transfer(handle, EP_OUT, buf, 3, &actual, 1000);
}

static int ddmi_reset(int srst, int trst)
{
    (void) srst;
    (void) trst;
    ddmi_init_reset(ddmi_usb_priv.handle);
    LOG_DEBUG("RESET is currently implemented");
    return ERROR_OK;
}

static int ddmi_usb_quit(void)
{
    libusb_release_interface(ddmi_usb_priv.handle, 0);
    libusb_close(ddmi_usb_priv.handle);
    libusb_exit(ddmi_usb_priv.ctx);
    // libusb cleanup
    return ERROR_OK;
}

// ------------------------- Execute Queue -------------------------
int ddmi_usb_execute_queue(struct jtag_command *cmd_queue)
{
    batch_execute(ddmi_usb_priv.handle, &r_w_buffer);
    LOG_DEBUG("execute queue is currently implemented");
    return 0;
}

void ddmi_exec_queue(void)
{
    batch_execute(ddmi_usb_priv.handle, &r_w_buffer);
}
static int ddmi_usb_init(void)
{
    ddmi_usb_priv.vid = VID;
    ddmi_usb_priv.pid = PID;
    ddmi_usb_priv.ctx = NULL;
    //return 0;
    int ret = libusb_init(&ddmi_usb_priv.ctx);
    if (ret < 0) {
        LOG_DEBUG("libusb_init failed: %s\n", libusb_error_name(ret));
        return 1;
    }

    ddmi_usb_priv.handle = libusb_open_device_with_vid_pid( ddmi_usb_priv.ctx, VID, PID);
    if (ddmi_usb_priv.handle == NULL) {
        LOG_DEBUG("Could not open device with VID:PID %04x:%04x\n", VID, PID);
        libusb_exit(ddmi_usb_priv.ctx);
        return 1;
    }

    if (libusb_kernel_driver_active(ddmi_usb_priv.handle, 0) == 1) {
        libusb_detach_kernel_driver(ddmi_usb_priv.handle, 0);
    }

    ret = libusb_claim_interface(ddmi_usb_priv.handle, 0);
    if (ret < 0) {
        LOG_DEBUG("Failed to claim interface: %s\n", libusb_error_name(ret));
        libusb_close(ddmi_usb_priv.handle);
        libusb_exit(ddmi_usb_priv.ctx);
        return 1;
    }
    libusb_set_debug(ddmi_usb_priv.ctx, LIBUSB_LOG_LEVEL_INFO);
    // libusb_init, open VID/PID, claim interface
    LOG_INFO("ddmi USB probe initialized");
    
    // Install driver into global ddmi_driver
    static struct ddmi_driver usb_driver = {
        .init = NULL,  // Already called
        .quit = ddmi_usb_quit,
        .dmi_read = ddmi_usb_dmi_read,
        .dmi_write = ddmi_usb_dmi_write,
        .srst = ddmi_reset,  // Optional
        .batch_exec = ddmi_exec_queue,
    };
    ddmi_driver = &usb_driver;
    ddmi_init_reset(ddmi_usb_priv.handle);
    return ERROR_OK;
}


static int ddmi_speed_set(int speed_index)
{
    LOG_DEBUG("set speed is not currently implemented");
    return ERROR_OK;
}



static const struct command_registration ddmi_command_handlers[] = {
	{
		.name = "ddmi_usb",
		.mode = COMMAND_ANY,
		.help = "perform ddmi_usb management",
		.chain = NULL,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};



struct jtag_interface ddmi_usb_interface = {
    .supported = 0,           // No JTAG/SWD capabilities needed
    .execute_queue = ddmi_usb_execute_queue,
};

struct adapter_driver ddmi_adapter_driver = {
	.name = "ddmi_usb",
	.transport_ids = TRANSPORT_DDMI,
	.transport_preferred_id = TRANSPORT_DDMI,
	.commands = ddmi_command_handlers,

	.init = ddmi_usb_init,
	.quit = ddmi_usb_quit,
	.reset = ddmi_reset,
	.speed = ddmi_speed_set,

	.jtag_ops = &ddmi_usb_interface,
};
