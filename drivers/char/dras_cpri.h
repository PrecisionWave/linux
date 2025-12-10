#ifndef __CHAR_DRAS_CPRI_H
#define __CHAR_DRAS_CPRI_H

#include <linux/ioctl.h>

// mmap special page offsets
#define DRAS_CPRI_MMAP_PORTID_REG 0
#define DRAS_CPRI_MMAP_RECCLK_REG 0x10
#define DRAS_CPRI_MMAP_FREQCNTR_REG 0x20
#define DRAS_CPRI_MMAP_CLKMON_REG 0x30
#define DRAS_CPRI_MMAP_XLNX_PORT_REG 0x100
#define DRAS_CPRI_MMAP_PCW_PORT_REG 0x200

// IOCTL calls
#define DRAS_CPRI_IOCTL_GET_CPRI_PORT_COUNT _IOR(0, 1, uint32_t)

#endif // #define __CHAR_DRAS_CPRI_H
