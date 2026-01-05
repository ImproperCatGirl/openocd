#ifndef OPENOCD_JTAG_ddmi_H
#define OPENOCD_JTAG_ddmi_H

#include <stdint.h>
#include "interface.h"

struct ddmi_driver {
    /**
     * Initialize ddmi connection to probe/target
     */
    int (*init)(void);

    //cleanup
    int (*quit)(void);
    
    /**
     * Read DMI register (your PIO read)
     */
    int (*dmi_read)(uint8_t address, uint32_t *value);
    
    /**
     * Write DMI register (your PIO write)
     */
    int (*dmi_write)(uint8_t address, uint32_t value);
    
    /**
     * System reset
     */
    int (*srst)(int srst, int trst);
};

extern struct ddmi_driver *ddmi_driver;

int ddmi_read(struct target *target, uint32_t *value, uint32_t address);
int ddmi_write(struct target *target, uint32_t address, uint32_t value);

#endif