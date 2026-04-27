// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CH32 SiP Flash Driver (No-Algorithm Page Programming)
 * Adapted for CH32V series with internal Auto-Erase Serial NOR
 */

 #include "flash/nor/core.h"
#include "helper/binarybuffer.h"
#include "target/algorithm.h"
#ifdef HAVE_CONFIG_H
 #include "config.h"
 #endif
 
 #include "imp.h"
 #include <helper/time_support.h>
 
 /* Register Definitions */
 #define FLASH_KEYR      0x40022004
 #define FLASH_MODEKEYR  0x40022024
 #define FLASH_STATR     0x4002200C
 #define FLASH_CTLR      0x40022010
 #define FLASH_ADDR 0x40022014 
 
 /* Bit Definitions (Adjust these to your specific POS_ names) */
 #define CTLR_LOCK       (1 << 7)
 #define CTLR_FTPG       (1 << 16)   /* Fast Page Programming Enable */
 #define CTLR_FTER          (1<<17)
 #define CTLR_PGSTRT     (1 << 21)   /* Fast Page Start */
 #define CTLR_STRT (1<<6)
 #define STATR_BSY       (1 << 0)
 #define STATR_WR_BSY    (1 << 1)    /* Word Buffer Busy */
 #define STATR_EOP       (1 << 5)
 unsigned char stub_bin[] = {
    0xb7, 0x07, 0x67, 0x45, 0x37, 0x27, 0x02, 0x40, 0x93, 0x87, 0x37, 0x12,
    0x37, 0x98, 0xef, 0xcd, 0x23, 0x22, 0xf7, 0x00, 0x13, 0x08, 0xb8, 0x9a,
    0x23, 0x22, 0x07, 0x01, 0x23, 0x22, 0xf7, 0x02, 0x23, 0x22, 0x07, 0x03,
    0x93, 0x06, 0x05, 0x00, 0x63, 0x02, 0x06, 0x0e, 0x13, 0x16, 0x86, 0x00,
    0x93, 0x88, 0x05, 0x00, 0x33, 0x83, 0xc5, 0x00, 0x37, 0x08, 0x02, 0x00,
    0x13, 0x05, 0x00, 0x02, 0x83, 0x27, 0xc7, 0x00, 0x93, 0xf7, 0x17, 0x00,
    0xe3, 0x9c, 0x07, 0xfe, 0x83, 0x27, 0x07, 0x01, 0xb3, 0xe7, 0x07, 0x01,
    0x23, 0x28, 0xf7, 0x00, 0x23, 0x2a, 0xb7, 0x00, 0x83, 0x27, 0x07, 0x01,
    0x93, 0xe7, 0x07, 0x04, 0x23, 0x28, 0xf7, 0x00, 0x83, 0x27, 0xc7, 0x00,
    0x93, 0xf7, 0x17, 0x00, 0xe3, 0x9c, 0x07, 0xfe, 0x23, 0x26, 0xa7, 0x00,
    0x93, 0x85, 0x05, 0x10, 0xe3, 0x92, 0x65, 0xfc, 0x13, 0x83, 0x06, 0x10,
    0x33, 0x88, 0xc6, 0x00, 0x93, 0x05, 0x03, 0x00, 0x37, 0x27, 0x02, 0x40,
    0x37, 0x0f, 0x01, 0x00, 0xb7, 0x0e, 0x20, 0x00, 0x13, 0x0e, 0x00, 0x02,
    0x83, 0x27, 0xc7, 0x00, 0x93, 0xf7, 0x17, 0x00, 0xe3, 0x9c, 0x07, 0xfe,
    0x83, 0x27, 0x07, 0x01, 0x33, 0x85, 0xd8, 0x40, 0xb3, 0xe7, 0xe7, 0x01,
    0x23, 0x28, 0xf7, 0x00, 0x03, 0xa6, 0x06, 0x00, 0xb3, 0x07, 0xd5, 0x00,
    0x23, 0xa0, 0xc7, 0x00, 0x83, 0x27, 0xc7, 0x00, 0x93, 0xf7, 0x27, 0x00,
    0xe3, 0x9c, 0x07, 0xfe, 0x93, 0x86, 0x46, 0x00, 0xe3, 0x92, 0xb6, 0xfe,
    0x83, 0x27, 0x07, 0x01, 0x93, 0x06, 0x03, 0x00, 0xb3, 0xe7, 0xd7, 0x01,
    0x23, 0x28, 0xf7, 0x00, 0x83, 0x27, 0xc7, 0x00, 0x93, 0xf7, 0x17, 0x00,
    0xe3, 0x9c, 0x07, 0xfe, 0x23, 0x26, 0xc7, 0x01, 0x93, 0x85, 0x05, 0x10,
    0x93, 0x88, 0x08, 0x10, 0x63, 0x06, 0x03, 0x01, 0x13, 0x03, 0x03, 0x10,
    0x6f, 0xf0, 0x5f, 0xf9, 0x37, 0x27, 0x02, 0x40, 0x83, 0x27, 0x07, 0x01,
    0x93, 0xe7, 0x07, 0x08, 0x23, 0x28, 0xf7, 0x00, 0x73, 0x00, 0x10, 0x00,
    0x67, 0x80, 0x00, 0x00
  };
  unsigned int stub_bin_len = 292;
  
  
  
 static int ch32_sip_unlock(struct flash_bank *bank)
 {
     struct target *target = bank->target;
     uint32_t ctrl;
 
     target_read_u32(target, FLASH_CTLR, &ctrl);
     if (!(ctrl & CTLR_LOCK))
         return ERROR_OK;
 
     /* Unlock FPEC */
     target_write_u32(target, FLASH_KEYR, 0x45670123);
     target_write_u32(target, FLASH_KEYR, 0xCDEF89AB);
 
     /* Unlock Fast Mode */
     target_write_u32(target, FLASH_MODEKEYR, 0x45670123);
     target_write_u32(target, FLASH_MODEKEYR, 0xCDEF89AB);
 
     target_read_u32(target, FLASH_CTLR, &ctrl);
     if (ctrl & CTLR_LOCK) {
         LOG_ERROR("Failed to unlock CH32 Flash");
         return ERROR_FLASH_OPERATION_FAILED;
     }
     uint32_t check_ctrl;
    target_read_u32(target, FLASH_CTLR, &check_ctrl);
    if (check_ctrl & 0x8000) {
        LOG_ERROR("CRITICAL: Flash re-locked (0x%08x) before algorithm start!", check_ctrl);
    }
    
     return ERROR_OK;
 }
 
 static int ch32_sip_wait_bsy(struct target *target, int timeout_ms)
 {
     uint32_t status;
     long long endtime = timeval_ms() + timeout_ms;
 
     while (1) {
         target_read_u32(target, FLASH_STATR, &status);
         if (!(status & STATR_BSY))
             break;
         if (timeval_ms() > endtime) {
             LOG_ERROR("Timeout waiting for Flash BSY");
             return ERROR_FLASH_OPERATION_FAILED;
         }
         keep_alive();
     }
     return ERROR_OK;
 }

 static int ch32_write_block_fast(struct flash_bank *bank, 
    struct working_area *stub_area,
    struct working_area *source_area,
    const uint8_t *buffer, 
    uint32_t offset, 
    uint32_t count)
{
    struct target *target = bank->target;
    struct reg_param reg_params[3];
    uint32_t address = (bank->base + offset) | 0x08000000;
    int retval;

    /* 1. Upload only the DATA chunk to the pre-allocated source area */
    retval = target_write_buffer(target, source_area->address, count, buffer);
    if (retval != ERROR_OK)
    return retval;

    /* 2. Setup registers (Using your PARAM_OUT mapping) */
    init_reg_param(&reg_params[0], "a0", 32, PARAM_OUT);
    buf_set_u32(reg_params[0].value, 0, 32, source_area->address);

    init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);
    buf_set_u32(reg_params[1].value, 0, 32, address);

    init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);
    /* Calculate pages: count is bytes, stub expects 256-byte page count */
    buf_set_u32(reg_params[2].value, 0, 32, (count + 255) / 256);

    /* 3. Run the algorithm */
    /* No stack used (requires stub compiled with -O2) */
    retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
        stub_area->address, 
        0, // OpenOCD handles ebreak exit
        10000, NULL);

    /* 4. Cleanup registers */
    destroy_reg_param(&reg_params[0]);
    destroy_reg_param(&reg_params[1]);
    destroy_reg_param(&reg_params[2]);

    return retval;
}


static int ch32_sip_write(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t offset, uint32_t count)
    {
    struct target *target = bank->target;
    int retval;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = ch32_sip_unlock(bank);
    if (retval != ERROR_OK)
        return retval;

    //struct target *target = bank->target;
    struct working_area *stub_area = NULL;
    struct working_area *source_area = NULL;
    //int retval;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = ch32_sip_unlock(bank);
    if (retval != ERROR_OK)
        return retval;

    if (count >= 256 && (offset % 256 == 0)) {

        uint32_t bytes_done = 0;
        printf("Engaging target ALGO!\n");

        /* 1. PRE-ALLOCATE: Allocate areas ONCE before the loop */
        /* Allocate space for the code stub */
        retval = target_alloc_working_area(target, sizeof(stub_bin), &stub_area);
        if (retval != ERROR_OK) goto fallback;

        /* Allocate a fixed 1KB buffer for data chunks */
        retval = target_alloc_working_area(target, 1024, &source_area);
        if (retval != ERROR_OK) goto fallback;

        /* 2. UPLOAD STUB: Upload the machine code ONCE */
        retval = target_write_buffer(target, stub_area->address, sizeof(stub_bin), stub_bin);
        if (retval != ERROR_OK) goto fallback;

        while (bytes_done < count) {
            uint32_t current_chunk = count - bytes_done;
            if (current_chunk > 1024) {
                current_chunk = 1024;
            }

            /* 3. EXECUTE: Call the execution block */
            retval = ch32_write_block_fast(bank, stub_area, source_area, 
                                          buffer + bytes_done, 
                                          offset + bytes_done, 
                                          current_chunk);
            
            if (retval != ERROR_OK) {
                LOG_ERROR("Fast block write failed at offset 0x%08" PRIx32, offset + bytes_done);
                break; 
            }
            bytes_done += current_chunk;
        }

    fallback:
        /* 4. CLEANUP: Free working areas only after the loop is done */
        if (source_area) target_free_working_area(target, source_area);
        if (stub_area) target_free_working_area(target, stub_area);

        if (bytes_done >= count) {
            return ERROR_OK;
        }

        /* Update pointers for RMW fallback if algorithm partially succeeded or failed */
        buffer += bytes_done;
        offset += bytes_done;
        count -= bytes_done;
    }

    /* --- ORIGINAL RMW LOGIC FOR SMALL/UNALIGNED WRITES --- */
    uint32_t current_addr = (bank->base + offset) | 0x08000000;
    uint32_t bytes_remaining = count;
    const uint8_t *src = buffer;
    uint8_t page_buffer[256];

    while (bytes_remaining > 0) {
        uint32_t page_start = current_addr & ~0xFF;
        uint32_t offset_in_page = current_addr & 0xFF;
        uint32_t chunk_size = 256 - offset_in_page;

        if (chunk_size > bytes_remaining)
            chunk_size = bytes_remaining;

        /* Read-Modify-Write if not a full page */
        if (chunk_size != 256) {
            retval = target_read_buffer(target, page_start, 256, page_buffer);
            if (retval != ERROR_OK) return retval;
        }

        memcpy(page_buffer + offset_in_page, src, chunk_size);

        /* 1. FAST PAGE ERASE (FTER) */
        target_write_u32(target, FLASH_CTLR, CTLR_FTER);
        target_write_u32(target, FLASH_ADDR, page_start);
        target_write_u32(target, FLASH_CTLR, CTLR_FTER | CTLR_STRT);
        retval = ch32_sip_wait_bsy(target, 100);
        if (retval != ERROR_OK) return retval;
        target_write_u32(target, FLASH_STATR, STATR_EOP);

        /* 2. FAST PAGE PROGRAM (FTPG) */
        target_write_u32(target, FLASH_CTLR, CTLR_FTPG);
        for (int i = 0; i < 256; i += 4) {
            uint32_t word;
            memcpy(&word, page_buffer + i, 4);
            target_write_u32(target, page_start + i, word);
            uint32_t status;
            do {
                target_read_u32(target, FLASH_STATR, &status);
            } while (status & STATR_WR_BSY);
        }

        /* Commit Programming */
        target_write_u32(target, FLASH_CTLR, CTLR_FTPG | CTLR_PGSTRT);
        retval = ch32_sip_wait_bsy(target, 100);
        if (retval != ERROR_OK) return retval;

        target_write_u32(target, FLASH_STATR, STATR_EOP);
        target_write_u32(target, FLASH_CTLR, 0);

        bytes_remaining -= chunk_size;
        src += chunk_size;
        current_addr += chunk_size;
        keep_alive();
    }

    return ERROR_OK;
}

/*
 static int ch32_sip_write_(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t offset, uint32_t count)
    {
    struct target *target = bank->target;
    int retval;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = ch32_sip_unlock(bank);
    if (retval != ERROR_OK)
        return retval;

    // * GDB might send data to 0x00000000. We must ensure the address used 
    // * for the FLASH_ADDR register starts with 0x08... 
    
    uint32_t current_addr = (bank->base + offset) | 0x08000000;
    printf("ADDR = %08X\n", current_addr);
    uint32_t bytes_remaining = count;
    const uint8_t *src = buffer;

    uint8_t page_buffer[256];

    while (bytes_remaining > 0) {
        uint32_t page_start = current_addr & ~0xFF;
        uint32_t offset_in_page = current_addr & 0xFF;
        uint32_t chunk_size = 256 - offset_in_page;

        if (chunk_size > bytes_remaining)
            chunk_size = bytes_remaining;

        // * READ-MODIFY-WRITE Logic:
        // If we aren't writing a perfect 256B aligned page, we must 
        // preserve the existing data (or the 0xE339 erase pattern).
        //
        if (chunk_size != 256) {
            / Read existing 256B page from target 
            retval = target_read_buffer(target, page_start, 256, page_buffer);
            if (retval != ERROR_OK)
                return retval;
        }

        // Patch the buffer with new data 
        memcpy(page_buffer + offset_in_page, src, chunk_size);

        // 1. FAST PAGE ERASE (FTER) 
        target_write_u32(target, FLASH_CTLR, CTLR_FTER);
        target_write_u32(target, FLASH_ADDR, page_start);
        target_write_u32(target, FLASH_CTLR, CTLR_FTER | CTLR_STRT);

        retval = ch32_sip_wait_bsy(target, 100);
        if (retval != ERROR_OK) return retval;

        target_write_u32(target, FLASH_STATR, STATR_EOP);

        // 2. FAST PAGE PROGRAM (FTPG) 
        target_write_u32(target, FLASH_CTLR, CTLR_FTPG);

        // Write exactly 64 words (256 bytes) from our RAM buffer 
        for (int i = 0; i < 256; i += 4) {
            uint32_t word;
            memcpy(&word, page_buffer + i, 4);

            target_write_u32(target, page_start + i, word);

            // Wait for Word Buffer (WR_BSY) 
            uint32_t status;
            do {
                target_read_u32(target, FLASH_STATR, &status);
            } while (status & STATR_WR_BSY);
        }

        // Commit Programming 
        target_write_u32(target, FLASH_CTLR, CTLR_FTPG | CTLR_PGSTRT);

        retval = ch32_sip_wait_bsy(target, 100);
        if (retval != ERROR_OK) return retval;

        // Clear Status and Disable Modes 
        target_write_u32(target, FLASH_STATR, STATR_EOP);
        target_write_u32(target, FLASH_CTLR, 0);

        // Advance pointers 
        bytes_remaining -= chunk_size;
        src += chunk_size;
        current_addr += chunk_size;
        
        keep_alive();
    }

    return ERROR_OK;
}*/
 
 static int ch32_sip_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
 {
     /* * Since your SiP handles auto-erase during write, we can 
      * technically return OK here or implement a standard 
      * sector erase if required by the chip's manual.
      */
     return ERROR_OK;
 }
 
 static int ch32_sip_probe(struct flash_bank *bank)
 {
     /* Setup bank info - Adjust size based on your specific chip */
     bank->size = 64 * 1024; 
     bank->num_sectors = bank->size / 1024;
     bank->sectors = alloc_block_array(0, 1024, bank->num_sectors);
     bank->erased_value = 0xFF;
     bank->default_padded_value = 0xFF;
 
     return ERROR_OK;
 }
 
 FLASH_BANK_COMMAND_HANDLER(ch32_sip_flash_bank_command)
 {
     return ERROR_OK;
 }


int get_ch32_flash_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    return 0;
}
 
 const struct flash_driver ch32_sip_flash = {
     .name = "ch32_sip",
     .flash_bank_command = ch32_sip_flash_bank_command,
     .erase = ch32_sip_erase,
     .write = ch32_sip_write,
     .read = default_flash_read,
     .probe = ch32_sip_probe,
     .auto_probe = ch32_sip_probe,
     .info =get_ch32_flash_info,
     .free_driver_priv = default_flash_free_driver_priv,
 };