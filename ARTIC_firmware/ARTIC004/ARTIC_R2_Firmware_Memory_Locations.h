// These are the firmware parameter memory locations for the ARTIC004 firmware
// They are defined in the ARTIC Datasheet and Arribada Horizon
// The locations for ARTIC006 are very different!

#ifndef ARTIC_R2_MEM_LOC_H
#define ARTIC_R2_MEM_LOC_H

#define ARTIC_R2_FIRMWARE_VERSION 4 // Make it clear that we are using ARTIC004

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

#endif
