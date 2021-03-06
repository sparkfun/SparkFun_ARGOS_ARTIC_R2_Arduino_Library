// These are the firmware parameter memory locations for the ARTIC004 firmware
// They are defined in the ARTIC Datasheet and Arribada Horizon
// The locations for ARTIC006 are very different!

#ifndef ARTIC_R2_MEM_LOC_H
#define ARTIC_R2_MEM_LOC_H

const uint8_t ARTIC_R2_FIRMWARE_VERSION = 4; // Make it clear that we are using ARTIC004

const uint32_t ARTIC_R2_PMEM_LEN = 10240; // PMEM Length in 32-Bit Words
const uint32_t ARTIC_R2_XMEM_LEN = 21845; // XMEM Length in 24-Bit Words
const uint32_t ARTIC_R2_YMEM_LEN = 6826; // YMEM Length in 24-Bit Words

const uint32_t ARTIC_R2_PMEM_CHECKSUM = 0x493982; // ARTIC004
const uint32_t ARTIC_R2_XMEM_CHECKSUM = 0x32DBBA; // ARTIC004
const uint32_t ARTIC_R2_YMEM_CHECKSUM = 0x65CC81; // ARTIC004

// P Memory Locations
const uint16_t MEM_LOC_FIRMWARE_VERSION = 0x0010; // 2 * 32-bit words = 8 bytes: 'ARTICnnn'

// X Memory locations
const uint16_t MEM_LOC_ARGOS_CONFIGURATION = 0x0384; // Size 1. Read only
const uint16_t MEM_LOC_RX_PAYLOAD = 0x0200; // Size 9. Read only
const uint16_t MEM_LOC_RX_FILTERING_CONFIGURATION = 0x0209; // Size 104. Read/Write (Was 0x120D in ARTIC006)
const uint16_t MEM_LOC_RX_FILTERING_ENABLE_CRC = 0x0209; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_TRANSPARENT_MODE = 0x020A; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_LUT_LENGTH = 0x020B; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_LUT_FIRST_ADDRESS = 0x020C; // Read/Write
const uint16_t MEM_LOC_RX_TIMEOUT = 0x0271; // Size 1. Read/Write
const uint16_t MEM_LOC_SATELLITE_DETECTION_TIMEOUT = 0x0272; // Size 1. Read/Write
const uint16_t MEM_LOC_TX_PAYLOAD = 0x0273; // Size 220. Write only
const uint16_t MEM_LOC_TX_FREQ_ARGOS_2_3 = 0x034F; // Size 1. Read/Write
const uint16_t MEM_LOC_TX_FREQ_ARGOS_4 = 0x035F; // Size 1. Read/Write
const uint16_t MEM_LOC_TCXO_WARMUP_TIME = 0x036F; // Sizde 1. Read/Write
const uint16_t MEM_LOC_TCXO_CONTROL = 0x0370; // Size 1. Read/Write
const uint16_t MEM_LOC_CRC_RESULTS = 0x0371; // Size 3. Read only
const uint16_t MEM_LOC_TX_CERTIFICATION_INTERVAL = 0x0379; // Size 1. Read/Write

// IO Memory locations
const uint16_t MEM_LOC_FIRMWARE_STATUS_REGISTER = 0x8018;

// Include the firmware for SPI upload - if desired
#ifdef ARTIC_R2_UPLOAD_FIRMWARE

// Include firmware binary data
// P 32-bit 10240 DSP Program memory
const uint32_t ARTIC_R2_PMEM[ARTIC_R2_PMEM_LEN] = {
#include "ARTIC_R2_Firmware_PMEM.h"
};

// X 24-bit 21845 DSP X memory
const uint32_t ARTIC_R2_XMEM[ARTIC_R2_XMEM_LEN] = {
#include "ARTIC_R2_Firmware_XMEM.h"
};

// Y 24-bit 6826 DSP Y memory
const uint32_t ARTIC_R2_YMEM[ARTIC_R2_YMEM_LEN] = {
#include "ARTIC_R2_Firmware_YMEM.h"
};

#endif

#endif
