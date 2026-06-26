// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CH32X035 OpenOCD flash driver.
 *
 * The target-side stub only performs the timing-sensitive 256-byte fast page
 * program sequence. Host code owns unlock, erase, and partial-page RMW.
 */

 #ifdef HAVE_CONFIG_H
 #include "config.h"
 #endif
 
 #include "flash/nor/core.h"
 #include "helper/binarybuffer.h"
 #include "helper/log.h"
 #include "helper/time_support.h"
 #include "target/algorithm.h"
 #include "imp.h"
 
 #include <inttypes.h>
 #include <string.h>
 
 #include "ch32x035_flash_algo_blob.h"
 
 #define CH32X035_FLASH_BASE      0x08000000u
 #define CH32X035_FLASH_SIZE      0x0000F800u
 #define CH32X035_PAGE_SIZE       256u
 #define CH32X035_SOURCE_SIZE     1024u
 
 #define FLASH_REG_BASE           0x40022000u
 #define FLASH_KEYR               0x40022004u
 #define FLASH_MODEKEYR           0x40022024u
 #define FLASH_STATR              0x4002200Cu
 #define FLASH_CTLR               0x40022010u
 #define FLASH_ADDR               0x40022014u
 
 #define FLASH_KEY1               0x45670123u
 #define FLASH_KEY2               0xCDEF89ABu
 
 #define CTLR_PER                 (1u << 1)
 #define CTLR_MER                 (1u << 2)
 #define CTLR_STRT                (1u << 6)
 #define CTLR_LOCK                (1u << 7)
 #define CTLR_FLOCK               (1u << 15)
 #define CTLR_FTPG                (1u << 16)
 #define CTLR_FTER                (1u << 17)
 #define CTLR_BER32               (1u << 23)
 
 #define STATR_BSY                (1u << 0)
 #define STATR_WRPRTERR           (1u << 4)
 #define STATR_EOP                (1u << 5)
 
 static uint32_t ch32x035_bank_base(struct flash_bank *bank)
 {
	 return bank->base != 0 ? bank->base : CH32X035_FLASH_BASE;
 }
 
 static int ch32x035_wait_ready(struct target *target, int timeout_ms)
 {
	 uint32_t status;
	 long long endtime = timeval_ms() + timeout_ms;
 
	 do {
		 int retval = target_read_u32(target, FLASH_STATR, &status);
		 if (retval != ERROR_OK)
			 return retval;
 
		 if ((status & STATR_BSY) == 0)
			 return ERROR_OK;
 
		 keep_alive();
	 } while (timeval_ms() <= endtime);
 
	 LOG_ERROR("Timeout waiting for CH32X035 FLASH BSY clear");
	 return ERROR_FLASH_OPERATION_FAILED;
 }
 
 static int ch32x035_clear_status(struct target *target)
 {
	 return target_write_u32(target, FLASH_STATR, STATR_EOP | STATR_WRPRTERR);
 }
 
 static int ch32x035_unlock_fast(struct target *target)
 {
	 uint32_t ctrl;
	 int retval = target_read_u32(target, FLASH_CTLR, &ctrl);
	 if (retval != ERROR_OK)
		 return retval;
 
	 if ((ctrl & CTLR_LOCK) != 0) {
		 retval = target_write_u32(target, FLASH_KEYR, FLASH_KEY1);
		 if (retval != ERROR_OK)
			 return retval;
		 retval = target_write_u32(target, FLASH_KEYR, FLASH_KEY2);
		 if (retval != ERROR_OK)
			 return retval;
	 }
 
	 retval = target_read_u32(target, FLASH_CTLR, &ctrl);
	 if (retval != ERROR_OK)
		 return retval;
 
	 if ((ctrl & CTLR_FLOCK) != 0) {
		 retval = target_write_u32(target, FLASH_MODEKEYR, FLASH_KEY1);
		 if (retval != ERROR_OK)
			 return retval;
		 retval = target_write_u32(target, FLASH_MODEKEYR, FLASH_KEY2);
		 if (retval != ERROR_OK)
			 return retval;
	 }
 
	 retval = target_read_u32(target, FLASH_CTLR, &ctrl);
	 if (retval != ERROR_OK)
		 return retval;
 
	 if ((ctrl & (CTLR_LOCK | CTLR_FLOCK)) != 0) {
		 LOG_ERROR("Failed to unlock CH32X035 flash controller, CTLR=0x%08" PRIx32, ctrl);
		 return ERROR_FLASH_OPERATION_FAILED;
	 }
 
	 return ERROR_OK;
 }
 
 static int ch32x035_lock(struct target *target)
 {
	 uint32_t ctrl;
	 int retval = target_read_u32(target, FLASH_CTLR, &ctrl);
	 if (retval != ERROR_OK)
		 return retval;
 
	 return target_write_u32(target, FLASH_CTLR, ctrl | CTLR_FLOCK | CTLR_LOCK);
 }
 
 static int ch32x035_erase_page(struct target *target, uint32_t page_addr)
 {
	 uint32_t ctrl;
	 int retval = ch32x035_wait_ready(target, 100);
	 if (retval != ERROR_OK)
		 return retval;
 
	 retval = ch32x035_clear_status(target);
	 if (retval != ERROR_OK)
		 return retval;
 
	 retval = target_read_u32(target, FLASH_CTLR, &ctrl);
	 if (retval != ERROR_OK)
		 return retval;
 
	 ctrl &= ~(CTLR_PER | CTLR_MER | CTLR_BER32 | CTLR_FTER | CTLR_FTPG);
	 ctrl |= CTLR_FTER;
 
	 retval = target_write_u32(target, FLASH_CTLR, ctrl);
	 if (retval != ERROR_OK)
		 return retval;
	 retval = target_write_u32(target, FLASH_ADDR, page_addr);
	 if (retval != ERROR_OK)
		 return retval;
	 retval = target_write_u32(target, FLASH_CTLR, ctrl | CTLR_STRT);
	 if (retval != ERROR_OK)
		 return retval;
 
	 retval = ch32x035_wait_ready(target, 100);
	 if (retval != ERROR_OK)
		 return retval;
 
	 retval = ch32x035_clear_status(target);
	 if (retval != ERROR_OK)
		 return retval;
 
	 return target_write_u32(target, FLASH_CTLR, ctrl & ~CTLR_FTER);
 }
 
 static int ch32x035_load_stub(struct target *target, struct working_area **stub_area)
 {
	 int retval = target_alloc_working_area(target, sizeof(ch32x035_flash_algo_bin),
							stub_area);
	 if (retval != ERROR_OK)
		 return retval;
 
	 retval = target_write_buffer(target, (*stub_area)->address,
					  sizeof(ch32x035_flash_algo_bin),
					  ch32x035_flash_algo_bin);
	 if (retval != ERROR_OK) {
		 target_free_working_area(target, *stub_area);
		 *stub_area = NULL;
	 }
 
	 return retval;
 }
 
 static int ch32x035_run_program_stub(struct target *target,
					  struct working_area *stub_area,
					  struct working_area *source_area,
					  uint32_t page_offset,
					  uint32_t byte_count)
 {
	 struct reg_param reg_params[5];
	 int retval;
 
	 init_reg_param(&reg_params[0], "a0", 32, PARAM_OUT);
	 buf_set_u32(reg_params[0].value, 0, 32, FLASH_REG_BASE);
 
	 init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);
	 buf_set_u32(reg_params[1].value, 0, 32, page_offset);
 
	 init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);
	 buf_set_u32(reg_params[2].value, 0, 32, source_area->address);
 
	 init_reg_param(&reg_params[3], "a3", 32, PARAM_OUT);
	 buf_set_u32(reg_params[3].value, 0, 32, byte_count);
 
	 init_reg_param(&reg_params[4], "a4", 32, PARAM_OUT);
	 buf_set_u32(reg_params[4].value, 0, 32, 0);
 
	 retval = target_run_algorithm(target, 0, NULL, 5, reg_params,
					   stub_area->address, 0, 10000, NULL);
 
	 destroy_reg_param(&reg_params[0]);
	 destroy_reg_param(&reg_params[1]);
	 destroy_reg_param(&reg_params[2]);
	 destroy_reg_param(&reg_params[3]);
	 destroy_reg_param(&reg_params[4]);
 
	 return retval;
 }
 
 static int ch32x035_write(struct flash_bank *bank, const uint8_t *buffer,
			   uint32_t offset, uint32_t count)
 {
	 struct target *target = bank->target;
	 struct working_area *stub_area = NULL;
	 struct working_area *source_area = NULL;
	 uint8_t page_buffer[CH32X035_PAGE_SIZE];
	 uint32_t base = ch32x035_bank_base(bank);
	 int retval;
 
	 if (target->state != TARGET_HALTED) {
		 LOG_ERROR("Target not halted");
		 return ERROR_TARGET_NOT_HALTED;
	 }
 
	 if (count == 0)
		 return ERROR_OK;
 
	 retval = ch32x035_unlock_fast(target);
	 if (retval != ERROR_OK)
		 return retval;
 
	 retval = ch32x035_load_stub(target, &stub_area);
	 if (retval != ERROR_OK)
		 goto out;
 
	 retval = target_alloc_working_area(target, CH32X035_SOURCE_SIZE, &source_area);
	 if (retval != ERROR_OK)
		 goto out;
 
	 while (count > 0) {
		 uint32_t page_offset = offset & ~(CH32X035_PAGE_SIZE - 1u);
		 uint32_t in_page = offset - page_offset;
		 uint32_t chunk = CH32X035_PAGE_SIZE - in_page;
		 const uint8_t *page_src = buffer;
		 uint32_t page_addr = base + page_offset;
		 uint32_t program_bytes = CH32X035_PAGE_SIZE;
 
		 if (chunk > count)
			 chunk = count;
 
		 if (in_page == 0 && count >= CH32X035_PAGE_SIZE) {
			 program_bytes = count & ~(CH32X035_PAGE_SIZE - 1u);
			 if (program_bytes > CH32X035_SOURCE_SIZE)
				 program_bytes = CH32X035_SOURCE_SIZE;
			 chunk = program_bytes;
		 } else {
			 retval = target_read_buffer(target, page_addr,
							 CH32X035_PAGE_SIZE, page_buffer);
			 if (retval != ERROR_OK)
				 goto out;
 
			 memcpy(page_buffer + in_page, buffer, chunk);
			 page_src = page_buffer;
		 }
 
		 LOG_DEBUG("CH32X035 erase/program offset 0x%08" PRIx32
			   ", %" PRIu32 " bytes", page_offset, program_bytes);
 
		 for (uint32_t erase_off = 0; erase_off < program_bytes;
			  erase_off += CH32X035_PAGE_SIZE) {
			 retval = ch32x035_erase_page(target, page_addr + erase_off);
			 if (retval != ERROR_OK)
				 goto out;
		 }
 
		 retval = target_write_buffer(target, source_area->address,
						  program_bytes, page_src);
		 if (retval != ERROR_OK)
			 goto out;
 
		 retval = ch32x035_run_program_stub(target, stub_area, source_area,
							page_offset, program_bytes);
		 if (retval != ERROR_OK)
			 goto out;
 
		 offset += chunk;
		 buffer += chunk;
		 count -= chunk;
		 keep_alive();
	 }
 
 out:
	 if (source_area != NULL)
		 target_free_working_area(target, source_area);
	 if (stub_area != NULL)
		 target_free_working_area(target, stub_area);
 
	 ch32x035_lock(target);
	 return retval;
 }
 
 static int ch32x035_erase(struct flash_bank *bank, unsigned int first,
			   unsigned int last)
 {
	 struct target *target = bank->target;
	 uint32_t base = ch32x035_bank_base(bank);
	 int retval;
 
	 if (target->state != TARGET_HALTED) {
		 LOG_ERROR("Target not halted");
		 return ERROR_TARGET_NOT_HALTED;
	 }
 
	 retval = ch32x035_unlock_fast(target);
	 if (retval != ERROR_OK)
		 return retval;
 
	 for (unsigned int sector = first; sector <= last; sector++) {
		 uint32_t page_offset = sector * CH32X035_PAGE_SIZE;
		 retval = ch32x035_erase_page(target, base + page_offset);
		 if (retval != ERROR_OK)
			 break;
		 keep_alive();
	 }
 
	 ch32x035_lock(target);
	 return retval;
 }
 
 static int ch32x035_probe(struct flash_bank *bank)
 {
	 if (bank->sectors != NULL)
		 return ERROR_OK;
 
	 bank->size = CH32X035_FLASH_SIZE;
	 bank->num_sectors = bank->size / CH32X035_PAGE_SIZE;
	 bank->sectors = alloc_block_array(0, CH32X035_PAGE_SIZE, bank->num_sectors);
	 bank->erased_value = 0xFF;
	 bank->default_padded_value = 0xFF;
 
	 return ERROR_OK;
 }
 
 FLASH_BANK_COMMAND_HANDLER(ch32x035_flash_bank_command)
 {
	 (void)bank;
	 return ERROR_OK;
 }
 
 static int ch32x035_info(struct flash_bank *bank, struct command_invocation *cmd)
 {
	 (void)bank;
	 (void)cmd;
	 return ERROR_OK;
 }
 
 const struct flash_driver ch32x035_flash = {
	 .name = "ch32x035",
	 .flash_bank_command = ch32x035_flash_bank_command,
	 .erase = ch32x035_erase,
	 .write = ch32x035_write,
	 .read = default_flash_read,
	 .probe = ch32x035_probe,
	 .auto_probe = ch32x035_probe,
	 .info = ch32x035_info,
	 .free_driver_priv = default_flash_free_driver_priv,
 };
 