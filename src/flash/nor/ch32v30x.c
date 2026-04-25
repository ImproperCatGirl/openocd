// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CH32 SiP Flash Driver (No-Algorithm Page Programming)
 * Adapted for CH32V series with internal Auto-Erase Serial NOR
 */

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

    /* * GDB might send data to 0x00000000. We must ensure the address used 
    * for the FLASH_ADDR register starts with 0x08... 
    */
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

        /* * READ-MODIFY-WRITE Logic:
        * If we aren't writing a perfect 256B aligned page, we must 
        * preserve the existing data (or the 0xE339 erase pattern).
        */
        if (chunk_size != 256) {
            /* Read existing 256B page from target */
            retval = target_read_buffer(target, page_start, 256, page_buffer);
            if (retval != ERROR_OK)
                return retval;
        }

        /* Patch the buffer with new data */
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

        /* Write exactly 64 words (256 bytes) from our RAM buffer */
        for (int i = 0; i < 256; i += 4) {
            uint32_t word;
            memcpy(&word, page_buffer + i, 4);

            target_write_u32(target, page_start + i, word);

            /* Wait for Word Buffer (WR_BSY) */
            uint32_t status;
            do {
                target_read_u32(target, FLASH_STATR, &status);
            } while (status & STATR_WR_BSY);
        }

        /* Commit Programming */
        target_write_u32(target, FLASH_CTLR, CTLR_FTPG | CTLR_PGSTRT);

        retval = ch32_sip_wait_bsy(target, 100);
        if (retval != ERROR_OK) return retval;

        /* Clear Status and Disable Modes */
        target_write_u32(target, FLASH_STATR, STATR_EOP);
        target_write_u32(target, FLASH_CTLR, 0);

        /* Advance pointers */
        bytes_remaining -= chunk_size;
        src += chunk_size;
        current_addr += chunk_size;
        
        keep_alive();
    }

    return ERROR_OK;
}
 
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