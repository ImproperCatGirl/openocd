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
#define PID 0x4002
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

#define MAX_BYTES 512
#define MAX_BATCH 200
#define CMD_SIZE      9
#define RESP_SIZE     4

static struct ddmi_usb_op pending_op_queue[10240];// TODO: switch to a linked list 
static int pending_op_len = 0;
//static uint8_t next_serial = 1;
//static size_t total_bytes = 3;

typedef enum ddmi_opcode {
    ddmi_WRITE,
    ddmi_READ,
    ddmi_RESET,
} ddmi_opcode_t;

/* -------------------------------------------------------------
 *  Helper: how many bytes does a *single* op consume?
 * ------------------------------------------------------------- */
 static inline size_t op_bytes(const struct ddmi_usb_op *op)
 {
     if (op->opcode == ddmi_READ)
         return 2;                     /* serial + addr */
     else if (op->opcode == ddmi_WRITE)
         return 6;                     /* serial + addr + data */
     else if (op->opcode == ddmi_RESET)
         return 1; //just the serial
     else
         return 0;                     /* unknown – will be filtered out */
 }

static int ddmi_usb_execute_queue(struct jtag_command *cmd_queue);

// ------------------------- Helper -------------------------
static void queue_op(uint8_t opcode, uint8_t addr, uint32_t data_to_target)
{
    /*if(total_bytes >= MAX_BYTES || pending_op_len >= MAX_BATCH) {
        LOG_DEBUG("Batch full, auto-flushing");
        ddmi_usb_execute_queue(NULL);
    }*/
    pending_op_queue[pending_op_len].opcode = opcode;
    pending_op_queue[pending_op_len].addr = addr;
    pending_op_queue[pending_op_len].data_to_target = data_to_target;
    pending_op_len++;
}
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


uint8_t tx_full[1024];

void batch_init(struct rv_usb_batch *batch) {
    batch->num_ops = 0;
}
// Add a read to the batch and store the destination pointer
void batch_add_read(struct rv_usb_batch *batch, uint32_t addr, uint32_t *dest) {
    if (batch->num_ops >= MAX_BATCH) return;

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
    if (batch->num_ops >= MAX_BATCH) return -1;

    uint8_t *ptr = &batch->buffer[batch->num_ops * CMD_SIZE];
    ptr[0] = (uint8_t)type;
    memcpy(ptr + 1, &addr, 4);
    memcpy(ptr + 5, &data, 4);
    
    batch->num_ops++;
    return 0;
}


/**
 * Executes a single DMI operation over USB
 * @param handle   The libusb device handle
 * @param op_type  'r' for read, 'w' for write
 * @param addr     32-bit RISC-V DMI address
 * @param data_in  The value to write (set to 0 for reads)
 * @param data_out Pointer to store the result from the debugger
 * @return 0 on success, negative on error
 */
 int usb_dmi_op(libusb_device_handle *handle, char op_type, 
    uint32_t addr, uint32_t data_in, uint32_t *data_out)
{
    int transferred;
    int rc;
    uint8_t tx_buf[9]; // 1 (cmd) + 4 (addr) + 4 (data)
    uint8_t rx_buf[4]; // 4 (data result)

    // Construct the binary blob
    tx_buf[0] = (uint8_t)op_type;
    // Using Little Endian - ensure this matches your debugger's architecture
    memcpy(&tx_buf[1], &addr, 4);
    memcpy(&tx_buf[5], &data_in, 4);

    // 1. Send Command Blob
    rc = libusb_bulk_transfer(handle, EP_OUT, tx_buf, sizeof(tx_buf), 
                    &transferred, TIMEOUT_MS);
    if (rc != 0 || transferred != sizeof(tx_buf)) {
        fprintf(stderr, "USB Write Failed: %s\n", libusb_strerror(rc));
        return -1;
    }

    // 2. Read Response (Debugger sends 4 bytes back)
    rc = libusb_bulk_transfer(handle, EP_IN, rx_buf, sizeof(rx_buf), 
                    &transferred, TIMEOUT_MS);
    if (rc != 0 || transferred != sizeof(rx_buf)) {
        fprintf(stderr, "USB Read Failed: %s\n", libusb_strerror(rc));
        return -1;
    }

    if (data_out) {
        memcpy(data_out, rx_buf, 4);
    }

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
// Define this in your driver's private struct or as a static global
static uint8_t usb_scratch_buf[4 + (MAX_BATCH * CMD_SIZE)];
static uint32_t results_scratch[MAX_BATCH];


#define MAX_OPS_PER_CHUNK 6
int batch_execute(libusb_device_handle *handle, struct rv_usb_batch *batch) {
    if (!batch || batch->num_ops == 0)
        return ERROR_OK;

    // 1. Safety Check
    if (batch->num_ops > MAX_BATCH) {
        LOG_ERROR("Batch size %zu exceeds limit!", batch->num_ops);
        return ERROR_FAIL;
    }

    size_t ops_completed = 0;
        //size_t reads_dispatched = 0;
    
    while (ops_completed < batch->num_ops) {
        size_t chunk_ops = (batch->num_ops - ops_completed);
        if (chunk_ops > MAX_OPS_PER_CHUNK)
            chunk_ops = MAX_OPS_PER_CHUNK;

        // 1. Pack Sub-Batch Header + Data
        uint32_t header = (uint32_t)chunk_ops;
        memcpy(usb_scratch_buf, &header, 4);
        memcpy(usb_scratch_buf + 4, batch->buffer + (ops_completed * CMD_SIZE), chunk_ops * CMD_SIZE);

        int transferred;
        size_t tx_len = 4 + (chunk_ops * CMD_SIZE);
        int rc = libusb_bulk_transfer(handle, EP_OUT, usb_scratch_buf, (int)tx_len, &transferred, 1000);
        if (rc != 0) return ERROR_FAIL;

        // 2. Receive Results for this chunk
        size_t rx_len = chunk_ops * RESP_SIZE;
        // Offset the result buffer so we don't overwrite previous chunk results
        uint8_t *rx_ptr = ((uint8_t*)results_scratch) + (ops_completed * RESP_SIZE);
        
        int received = 0;
        while (received < (int)rx_len) {
            int actual;
            rc = libusb_bulk_transfer(handle, EP_IN, rx_ptr + received, (int)rx_len - received, &actual, 1000);
            if (rc != 0) return ERROR_FAIL;
            received += actual;
        }

        // 3. Update progress
        ops_completed += chunk_ops;
    }

    // 4. Final Dispatch: Map all results to the original pointers
    for (size_t i = 0; i < batch->num_reads; i++) {
        uint32_t *dest = batch->reads[i].dest;
        size_t global_idx = batch->reads[i].batch_index;
        if (dest)
            *dest = results_scratch[global_idx];
    }
    
    batch->num_ops = 0;
    batch->num_reads = 0;
    return ERROR_OK;
    // 6. IMPORTANT: Reset batch immediately to prevent double-execution

}


static int ddmi_reset(int srst, int trst)
{
    (void) srst;
    (void) trst;
    queue_op(ddmi_RESET, 0x0, 0x0);
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
    };
    ddmi_driver = &usb_driver;
    
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


// ------------------------- Execute Queue -------------------------
int ddmi_usb_execute_queue(struct jtag_command *cmd_queue)
{
    batch_execute(ddmi_usb_priv.handle, &r_w_buffer);
    LOG_DEBUG("execute queue is currently implemented");
    return 0;
}

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
