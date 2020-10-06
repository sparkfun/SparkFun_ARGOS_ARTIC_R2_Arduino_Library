// These are the firmware parameter memory locations for the ARTIC006 firmware
// They are defined in the Release_README.txt for ARTIC006
// The locations for ARTIC004 are very different!

#ifndef ARTIC_R2_MEM_LOC_H
#define ARTIC_R2_MEM_LOC_H

const uint8_t ARTIC_R2_FIRMWARE_VERSION = 6; // Make it clear that we are using ARTIC006

// P Memory Locations
const uint16_t MEM_LOC_FIRMWARE_VERSION = 0x0010; // 2 * 32-bit words = 8 bytes: 'ARTICnnn'

// X Memory locations
const uint16_t MEM_LOC_ARGOS_CONFIGURATION = 0x137E; // Size 1. Read only (Was 0x0384 in ARTIC004)
const uint16_t MEM_LOC_RX_PAYLOAD = 0x1204; // Size 9. Read only (Was 0x0200 in ARTIC004)
const uint16_t MEM_LOC_RX_FILTERING_CONFIGURATION = 0x120D; // Size 104. Read/Write (Was 0x0209 in ARTIC004)
const uint16_t MEM_LOC_RX_FILTERING_ENABLE_CRC = 0x120D; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_TRANSPARENT_MODE = 0x120E; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_LUT_LENGTH = 0x120F; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_LUT_FIRST_ADDRESS = 0x1210; // Read/Write
const uint16_t MEM_LOC_RX_TIMEOUT = 0x1275; // Size 1. Read/Write (Was 0x0271 in ARTIC004)
const uint16_t MEM_LOC_SATELLITE_DETECTION_TIMEOUT = 0x1276; // Size 1. Read/Write (Was 0x0272 in ARTIC004)
const uint16_t MEM_LOC_TX_PAYLOAD = 0x1277; // Size 220. Write only.  (Was 0x0273 in ARTIC004)
const uint16_t MEM_LOC_TX_FREQ_ARGOS_2_3 = 0x1353; // Size 1. Read/Write (Was 0x034F in ARTIC004)
const uint16_t MEM_LOC_TX_FREQ_ARGOS_4 = 0x1363; // Size 1. Read/Write (Was 0x035F in ARTIC004)
const uint16_t MEM_LOC_TCXO_WARMUP_TIME = 0x1373; // Sizde 1. Read/Write (Was 0x036F in ARTIC004)
const uint16_t MEM_LOC_TCXO_CONTROL = 0x1374; // Size 1. Read/Write (Was 0x0370 in ARTIC004)
const uint16_t MEM_LOC_CRC_RESULTS = 0x1375; // Size 3. Read only.  (Was 0x0371 in ARTIC004)
const uint16_t MEM_LOC_TX_CERTIFICATION_INTERVAL = 0x137D; // Size 1. Read/Write (Was 0x0379 in ARTIC004)
const uint16_t MEM_LOC_FLASH_PROG_BUFFER = 0x137F; // Flash Programming Buffer (New in ARTIC006)

// IO Memory locations
const uint16_t MEM_LOC_FIRMWARE_STATUS_REGISTER = 0x8018;

#endif
