/*
  This is a library written for the ARGOS ARTIC R2 Breakout
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/

  Written by Paul Clark, September 29th 2020

  The ARGOS ARTIC R2 chipset allows you to send and receive short data messages via the
	ARGOS satellite system.

  This library handles SPI communication with the chipset.

  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	See the LICENSE.md for more details.

	Parts of this library were inspired by or based on the Arribada Horizon bio-logging platform:
	https://github.com/arribada/horizon
	Arribada's work is gratefully acknowledged.

*/

#ifndef ARTIC_R2_H
#define ARTIC_R2_H

#include <SPI.h> // Needed for SPI communication

// SCLK is normally low (CPOL=0). Data is valid/latched on the falling SCLK edge (CPHA=1). So we need to use SPI MODE1.
#define ARTIC_R2_SPI_MODE SPI_MODE1

#define ARTIC_R2_PWR_EN_ON HIGH // Arribada Horizon: pull PWR_EN high to enable power
#define ARTIC_R2_PWR_EN_OFF LOW // Arribada Horizon: pull PWR_EN low to disable power
//#define ARTIC_R2_PWR_EN_ON LOW // SparkFun ARTIC R2 Breakout: pull PWR_EN low to enable power
//#define ARTIC_R2_PWR_EN_OFF HIGH // SparkFun ARTIC R2 Breakout: pull PWR_EN high to disable power

#define ARTIC_R2_FLASH_BOOT_TIMEOUT 2500 // ARTIC should boot in 2.25 secs. Timeout after 2500ms.
#define ARTIC_R2_BOOT_TIMEOUT 500 // Datasheet says ARTIC should boot in 0.25 secs after firmware upload. Timeout after 500ms.
//#define ARTIC_R2_BOOT_TIMEOUT 10000 // Arribada Horizon waits for up to 10 seconds
#define ARTIC_R2_BOOT_DELAY_MS 1000 // Delay used when uploading firmware
#define ARTIC_R2_BOOT_MAX_RETRIES 5 // Attempt to read the status register this many times when uploading firmware
#define ARTIC_R2_BURST_INTER_WORD_DELAY_US 50 // Delay for this many microseconds between words during a write burst
#define ARTIC_R2_BURST_BLOCK_SIZE 60 // Break burst data up into blocks of this size
#define ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS 5 // Delay for this many milliseconds between blocks of words
#define ARTIC_R2_BURST_FINISH_DELAY_MS 13 // Delay for this many milliseconds after a burst
#define ARTIC_R2_INSTRUCTION_DELAY_MS 10 // Delay for this many milliseconds after sending an MCU instruction before checking the MCU status
#define ARTIC_R2_CONFIGURATION_DELAY_MS 10 // Delay for this many milliseconds after sending an MCU configuration command before checking the MCU status
#define ARTIC_R2_HOUSEKEEPING_DELAY_MS 1 // Delay for this many milliseconds after sending an MCU housekeeping command before checking the MCU status

#define ARTIC_R2_MAX_ADDRESS_LUT_LENGTH 50 // Maximum length of the address-filtering look-up-table

#define ARTIC_R2_UPLOAD_FIRMWARE // Comment this line to save memory once the flash memory on the ARTIC R2 Breakout has been programmed successfully

#ifdef ARTIC_R2_UPLOAD_FIRMWARE
// Include firmware binary data
// P 32-bit 10240 DSP Program memory
// X 24-bit 21845 DSP X memory
// Y 24-bit 6826 DSP Y memory
#include "Firmware_ARTIC006_flash_image__PMEM.h"
#include "Firmware_ARTIC006_flash_image__XMEM.h"
#include "Firmware_ARTIC006_flash_image__YMEM.h"
#endif

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Firmware status register (0x8018)
typedef struct {
	union {
		uint32_t STATUS_REGISTER;
		struct {
			// Current Firmware state
			uint32_t IDLE								: 1; // The firmware is idle and ready to accept commands
			uint32_t RX_IN_PROGRESS			: 1; // The firmware is receiving
			uint32_t TX_IN_PROGRESS			: 1; // The firmware is transmitting
			uint32_t BUSY								: 1; // The firmware is changing state
			// Interrupt 1 flags
			uint32_t RX_VALID_MESSAGE					: 1; // A message has been received
			uint32_t RX_SATELLITE_DETECTED		: 1; // A satellite has been detected
			uint32_t TX_FINISHED							: 1; // The transmission was completed
			uint32_t MCU_COMMAND_ACCEPTED			: 1; // The configuration command has been accepted
			uint32_t CRC_CALCULATED						: 1; // CRC calculation has finished
			uint32_t IDLE_STATE								: 1; // Firmware returned to the idle state
			uint32_t RX_CALIBRATION_FINISHED	: 1; // RX offset calibration has completed
			uint32_t 													: 2;
			// Interrupt 2 flags
			uint32_t RX_TIMEOUT						: 1; // The specified reception time has been exceeded
			uint32_t SATELLITE_TIMEOUT		: 1; // No satellite was detected within the specified time
			uint32_t RX_BUFFER_OVERFLOW		: 1; // A received message was lost. No buffer space left
			uint32_t TX_INVALID_MESSAGE		: 1; // Incorrect TX payload length specified
			uint32_t MCU_COMMAND_REJECTED	: 1; // Incorrect command send or Firmware is not in idle
			uint32_t MCU_COMMAND_OVERFLOW	: 1; // Previous command was not yet processed
			uint32_t 											: 2;
			uint32_t INTERNAL_ERROR				: 1; // An internal error has occurred
			// Interrupt pin status
			uint32_t DSP2MCU_INT1	: 1; // Interrupt 1 pin status
			uint32_t DSP2MCU_INT2	: 1; // Interrupt 2 pin status
		} STATUS_REGISTER_BITS;
	};
} ARTIC_R2_Firmware_Status;

// Burstmode register
typedef struct {
	union {
		uint32_t BURSTMODE_REGISTER;
		struct {
			uint32_t BURSTMODE_START_ADDR	  : 16; //Burstmode Start Address
			uint32_t BURST_R_RW_MODE			  : 1;
			uint32_t BURST_MEM_SEL				  : 2; // Burstmode memory select: 00: Program Memory; 01: X Memory; 10: Y Memory; 11: IO Memory
			uint32_t BURST_MODE_ON				  : 1;
			uint32_t                        : 4; // Reserved bits
			uint32_t BURSTMODE_REG_SPI_ADDR : 8; // Burstmode register is write only and occupies address 0x00. Shift left by 1 bit and OR with 0 for writing = 0x00.
		} BURSTMODE_REGISTER_BITS;
	};
} ARTIC_R2_Burstmode_Register;

enum ARTIC_R2_Memory {
	ARTIC_R2_P_MEMORY = 0,
	ARTIC_R2_X_MEMORY,
	ARTIC_R2_Y_MEMORY,
	ARTIC_R2_IO_MEMORY,
};

enum ARTIC_R2_Burst_R_RW_Mode {
	ARTIC_R2_WRITE_BURST = 0,
	ARTIC_R2_READ_BURST,
};

// MCU Configuration Commands
//const uint8_t CONFIG_CMD_SET_ARGOS_4_RX_MODE = 0x01; // Unsupported by ARTIC006! Use ARGOS 3 RX.
const uint8_t CONFIG_CMD_SET_ARGOS_3_RX_MODE = 0x02;
const uint8_t CONFIG_CMD_SET_ARGOS_3_RX_BACKUP_MODE = 0x03;
const uint8_t CONFIG_CMD_SET_PTT_A2_TX_MODE = 0x04;
const uint8_t CONFIG_CMD_SET_PTT_A3_TX_MODE = 0x05;
const uint8_t CONFIG_CMD_SET_PTT_ZE_TX_MODE = 0x06;
const uint8_t CONFIG_CMD_SET_ARGOS_3_PTT_HD_TX_MODE = 0x07;
const uint8_t CONFIG_CMD_SET_ARGOS_4_PTT_HD_TX_MODE = 0x08;
const uint8_t CONFIG_CMD_SET_ARGOS_4_PTT_MD_TX_MODE = 0x09;
const uint8_t CONFIG_CMD_SET_ARGOS_4_PTT_VLD_TX_MODE = 0x0A;

// MCU Command Results
typedef enum {
	ARTIC_R2_MCU_COMMAND_ACCEPTED = 0x00, // The configuration command / instruction has been accepted
	ARTIC_R2_MCU_COMMAND_REJECTED, // Incorrect command / instruction sent or firmware is not in idle
	ARTIC_R2_MCU_COMMAND_OVERFLOW, // Previous command / instruction was not yet processed
	ARTIC_R2_MCU_COMMAND_UNCERTAIN, // Command / instruction uncertain? (The MCU did not raise ACCEPTED, REJECTED or OVERFLOW)
	ARTIC_R2_MCU_COMMAND_INVALID, // The configuration command / instruction was invalid
	ARTIC_R2_MCU_INSTRUCTION_IN_PROGRESS, // sendMCUinstruction will return this if an instruction is already in progress
	ARTIC_R2_MCU_COMMAND_VALID, // sendHousekeepingCommand will return this if the command was valid
} ARTIC_R2_MCU_Command_Result;

// Configuration register (read only)
typedef struct {
	union {
		uint32_t CONFIGURATION_REGISTER;
		struct {
			uint32_t TX_CONFIGURATION : 4;
			uint32_t RX_CONFIGURATION : 4;
		} CONFIGURATION_REGISTER_BITS;
	};
} ARGOS_Configuration_Register;

// RX configuration
const uint8_t RX_CONFIG_ARGOS_3_RX_MODE = 0x00;
const uint8_t RX_CONFIG_ARGOS_3_RX_BACKUP_MODE = 0x01;
const uint8_t RX_CONFIG_ARGOS_4_RX_MODE = 0x02;

// TX configuration
const uint8_t TX_CONFIG_ARGOS_PTT_A2_MODE = 0x00;
const uint8_t TX_CONFIG_ARGOS_PTT_A3_MODE = 0x01;
const uint8_t TX_CONFIG_ARGOS_PTT_ZE_MODE = 0x02;
const uint8_t TX_CONFIG_ARGOS_PTT_HD_MODE = 0x03;
const uint8_t TX_CONFIG_ARGOS_PTT_A4_MD_MODE = 0x04;
const uint8_t TX_CONFIG_ARGOS_PTT_A4_HD_MODE = 0x05;
const uint8_t TX_CONFIG_ARGOS_PTT_A4_VLD_MODE = 0x06;

// MCU Instructions
const uint8_t INST_START_CONTINUOUS_RECEPTION = 0x41; // Start the ARTIC in receiving mode for an unlimited time and unlimited number of messages. The user has to use the ‘Go to idle’ command to stop the receiver.
const uint8_t INST_START_RECEIVING_1_MESSAGE = 0x42; // Start the ARTIC in receiving mode for an unlimited time until 1 message has been received. If the message is received the Artic will go to IDLE. The user can abort the reception using the ‘Go to idle’ command.
const uint8_t INST_START_RECEIVING_2_MESSAGES = 0x43; // Start the ARTIC in receiving mode for an unlimited time until 2 messages have been received. If the 2 messages are received the ARTIC will go to IDLE. The user can abort the reception using the ‘Go to idle’ command.
const uint8_t INST_START_RECEPTION_FOR_FIXED_TIME = 0x46; // Start the ARTIC in receiving mode for a programmable time and for an unlimited amount of messages. After the programmed time has finished the ARTIC will go to IDLE. The user can abort the reception using the ‘Go to idle’ command. After the receiver has been active for the programmed amount of time, INT 2 will be set with the ‘RX_TIMEOUT’ flag.
const uint8_t INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE = 0x48; // The ARTIC will transmit the payload message according to the configured ARGOS mode and will go to IDLE.
const uint8_t INST_TRANSMIT_ONE_PACKAGE_AND_START_RX = 0x49; // The ARTIC will transmit the payload message according to the configured ARGOS mode and will switch to reception mode as described in ‘Start reception for fixed time’.
const uint8_t INST_GO_TO_IDLE = 0x50; // The ARTIC will return to idle mode. This command will not have an effect whenever the ARTIC is in ‘BUSY’ state. Once the ARTIC has returned to the idle state, interrupt 1 will be raised and the IDLE_STATE flag will be set.
const uint8_t INST_SATELLITE_DETECTION = 0x55; // The ARTIC will start looking for a satellite for a specified amount of time. If no satellite is detected, INT 2 will be set with the ‘SATELLITE_TIMEOUT’ flag. If a satellite was detected, by receiving 5 consecutive 0x7E flags, INT 1 will be set with the ‘RX_SATELLITE_DETECTED’ flag.

// MCU Instruction Progress
typedef enum {
	ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS = 0x00,

	ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION,
	ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_SATELLITE_DETECTED,
	ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_VALID_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_BUFFER_OVERFLOW,

	ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_SATELLITE_DETECTED,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_VALID_MESSAGE,

	ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES_RX_SATELLITE_DETECTED,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES_RX_VALID_MESSAGE,

	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_SATELLITE_DETECTED,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_VALID_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_BUFFER_OVERFLOW,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT_WITH_VALID_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT_WITH_BUFFER_OVERFLOW,

	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_IDLE_STATE,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_TX_FINISHED,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_TX_INVALID_MESSAGE,

	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_TX_FINISHED,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_TX_INVALID_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_SATELLITE_DETECTED,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_VALID_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_BUFFER_OVERFLOW,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT_WITH_VALID_MESSAGE,
	ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT_WITH_BUFFER_OVERFLOW,

	ARTIC_R2_MCU_PROGRESS_GO_TO_IDLE,
	ARTIC_R2_MCU_PROGRESS_GO_TO_IDLE_IDLE_STATE,

	ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION,
	ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_RX_SATELLITE_DETECTED,
	ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT,
	// ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT_WITH_SATELLITE_DETECTED, // Note: this state may be impossible to reach. Status seems to go IDLE as soon as a satellite is detected.

	ARTIC_R2_MCU_PROGRESS_INTERNAL_ERROR,

	ARTIC_R2_MCU_PROGRESS_UNKNOWN_INSTRUCTION,
} ARTIC_R2_MCU_Instruction_Progress;

// MCU Housekeeping: command words
const uint8_t CMD_CLEAR_INT_1 = 0x80; // Clear interrupt line 1
const uint8_t CMD_CLEAR_INT_2 = 0xC0; // Clear interrupt line 2

// SPI Standard Mode Addresses
const uint8_t ARTIC_R2_BURSTMODE_REG_WRITE = 0x00; // Burstmode register is at address 0x00. Shift left by 1 bit and OR with 0 for writing = 0x00.
const uint8_t ARTIC_R2_DSP_CRTL_REG_WRITE = 0x02; // DSP control register is at address 0x01. Shift left by 1 bit and OR with 0 for writing = 0x02.
const uint8_t ARTIC_R2_DSP_CRTL_REG_READ = 0x03; // DSP control register is at address 0x01. Shift left by 1 bit and OR with 1 for reading = 0x03.

const uint8_t ARTIC_R2_DSP_CRTL_REG_MAGIC_NUMBER = 85; // The DSP control register will contain this when it is ready for firmware upload

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

// TCXO Control
typedef struct {
	union {
		uint32_t TCXO_CONTROL_REGISTER;
		struct {
			uint32_t AUTO_DISABLE 			: 1;
			uint32_t										: 15;
			uint32_t SELECT_1V3_TO_2V7	: 4; // Vout = 1.3 + 0.1 * reg
			uint32_t SELECT_1V8					: 1;
			uint32_t SELECT_3V3					: 1;
		} CONTROL_REGISTER_BITS;
	};
} TCXO_Control_Register;

// Downlink Message - as defined by ARGOS standard A4-SYS-IF-0086-CNES
typedef struct {
	uint32_t payloadLength;
	uint32_t addresseeIdentification;
	uint8_t ADCS;
	uint8_t service;
	uint8_t payload[17]; // Maxmimum payload length is 17 bytes (136 bits)
	uint16_t FCS;
} Downlink_Message;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

class ARTIC_R2
{
public:
	// The maximum SPI clock speed for ARTIC read operations from the X/Y/IO memory is 1.25MHz, so let's play safe and default to 1MHz
	boolean begin(uint8_t user_CSPin, uint8_t user_RSTPin, uint8_t user_BOOTPin, uint8_t user_PWRENPin, uint8_t user_INT1Pin, uint8_t user_INT2Pin, uint8_t user_GAIN8Pin = -1, uint8_t user_GAIN16Pin = -1, uint32_t spiPortSpeed = 1000000, SPIClass &spiPort = SPI);

	void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

	boolean setTXgain(int gain = 24); // Set the TX gain (valid values are 0,8,16,24)

	void enableARTICpower(); // Enable power for the ARTIC R2 by pulling the power enable pin low
	void disableARTICpower(); // Disable power for the ARTIC R2 by pulling the power enable pin high

	void readStatusRegister(ARTIC_R2_Firmware_Status *status); // Read the ARTIC R2 status register

	void printFirmwareStatus(ARTIC_R2_Firmware_Status status, Stream &port = Serial); // Pretty-print the firmware status

	ARTIC_R2_MCU_Command_Result sendConfigurationCommand(uint8_t command); // Send a single 8-bit configuration command
	ARTIC_R2_MCU_Command_Result sendMCUinstruction(uint8_t instruction); // Send a single 8-bit MCU instruction
	ARTIC_R2_MCU_Command_Result sendHousekeepingCommand(uint8_t command); // Send a single 8-bit MCU housekeeping command
	void printCommandResult(ARTIC_R2_MCU_Command_Result result, Stream &port = Serial); // Pretty-print the command result

	boolean checkMCUinstructionProgress(ARTIC_R2_MCU_Instruction_Progress *progress); // Check the MCU instruction progress. Returns true if the instruction is complete.
	void printInstructionProgress(ARTIC_R2_MCU_Instruction_Progress progress, Stream &port = Serial); // Pretty-print the MCU instruction progress

	boolean clearInterrupts(uint8_t interrupts = 3); // Clear one or both interrupts. Default to both.

	void readFirmwareVersion(char *buffer); // Read the firmware version from PMEM
	void readMemoryCRC(uint32_t *PMEM_CRC, uint32_t *XMEM_CRC, uint32_t *YMEM_CRC); // Read the memories CRCs (after firmware boot)

	boolean setRxTimeout(uint32_t timeout_secs = 0x0A); // Set the RX timeout (seconds). Default to 10.
	boolean setSatelliteDetectionTimeout(uint32_t timeout_secs = 0x02); // Set the satellite detection timeout (seconds). Default to 2.
	boolean setTCXOWarmupTime(uint32_t timeout_secs = 0x0A); // Set the TCXO warm up time (seconds). Default to 10.
	boolean setTxCertificationInterval(uint32_t timeout_secs = 0x02); // Set the TX certification interval

	void readARGOSconfiguration(ARGOS_Configuration_Register *configuration); // Read the ARGOS configuration register

	void printARGOSconfiguration(ARGOS_Configuration_Register configuration, Stream &port = Serial); // Pretty-print the ARGOS configuration
	const char *txConfigurationString(ARGOS_Configuration_Register configuration); // Returns a human-readable version of the ARGOS TX configuration
	const char *rxConfigurationString(ARGOS_Configuration_Register configuration); // Returns a human-readable version of the ARGOS RX configuration

	uint32_t readRxTimeout(); // Read the RX timeout
	uint32_t readSatelliteDetectionTimeout(); // Read the satellite detection timeout
	uint32_t readTCXOWarmupTime(); // Read the TCXO warm up time
	uint32_t readTxCertificationInterval(); // Read the TX certification interval

	boolean setTCXOControl(float voltage = 3.3, bool autoDisable = true); // Set the TCXO control voltage and auto-disable. Default to 3.3V and leave enabled.
	float readTCXOControlVoltage(); // Read the TCXO control voltage. Auto-disable is ignored.
	boolean readTCXOAutoDisable(); // Read the TCXO control auto-disable bit

	boolean setARGOS23TxFrequency(float freq_MHz); // Set the ARGOS 2/3 TX Frequency
	boolean setARGOS4TxFrequency(float freq_MHz); // Set the ARGOS 4 TX Frequency
	float getARGOS23TxFrequency(); // Get the ARGOS 2/3 TX Frequency
	float getARGOS4TxFrequency(); // Get the ARGOS 4 TX Frequency

	boolean enableRXCRC(); // Enable RX CRC check
	boolean disableRXCRC(); // Disable RX CRC check
	boolean isRXCRCenabled(); // Returns true if the RX CRC is enabled
	boolean enableRXTransparentMode(); // Enable RX transparent mode
	boolean disableRXTransparentMode(); // Disable RX transparent mode
	boolean isRXTransparentModeEnabled(); // Returns true if RX transparent mode is enabled
	boolean clearAddressLUT(); // Clear the address look-up-table by setting the length to zero
	boolean addAddressToLUT(uint32_t platformID); // Add the specified platform ID to the message filter Look Up Table
	int getAddressLUTlength(); // Returns the address look-up-table length
	boolean readAddressLUTentry(int entry, uint32_t *address); // Returns the LUT address for the chosen entry

	boolean readDownlinkMessage(Downlink_Message *downlinkMessage); // Read a downlink message from the RX payload buffer

	// Helper functions to assemble the different message payloads
	boolean setPayloadARGOS4VLD0(uint32_t platformID); // Set the Tx payload for a ARGOS 4 VLD message with 0 bits of user data
	boolean setPayloadARGOS4VLD28(uint32_t platformID, uint32_t userData); // Set the Tx payload for a ARGOS 4 VLD message with 28 bits of user data

	// Storage for message transmission
	// This storage is used by (e.g.) setPayloadARGOS4VLD0 and setPayloadARGOS4VLD28
	// It should probably be private, but it is public just in case the user wants
	// to assemble their own payload.
	uint32_t _txPayloadLengthBits = 0; // The encoded message length in bits
	uint8_t _txPayloadBytes[660]; // Storage for up to 220 24-bit words

	// This function copies _txPayloadLengthBits and _txPayloadBytes to the TX Payload in XMEM
	// It gets called by (e.g.) setPayloadARGOS4VLD0 and setPayloadARGOS4VLD28
	// This function should probably be private, but it is public just in case the user wants
	// to assemble their own payload.
	// It returns true if the payload was copied successfully.
	boolean setTxPayload();

private:
	//Variables
	Stream *_debugPort;			 //The stream to send debug messages to if enabled. Usually Serial.
	boolean _printDebug = false; //Flag to print debugging variables

	SPIClass *_spiPort; // The generic connection to user's chosen SPI hardware
	unsigned long _spiPortSpeed; // Optional user defined port speed
	uint8_t _cs; // ARTIC R2 SPI Chip Select
	uint8_t _rst; // ARTIC R2 Reset pin
	uint8_t _boot; // ARTIC R2 Boot pin
	uint8_t _pwr_en; // Pull this pin low to enable power for the ARTIC R2
	uint8_t _int1; // ARTIC R2 Interrupt 1 pin
	uint8_t _int2; // ARTIC R2 Interrupt 2 pin
	uint8_t _gain8 = -1; // Pull this pin high to _disable_ the x8 RF gain
	uint8_t _gain16 = -1; // Pull this pin high to _disable_ the x16 RF gain

	// The user has to wait for the duration of 24 SPI clock cycles after configuring the burst read mode, before starting the first read.
	// This allows some time for the internal memory access block to retrieve the first data sample.
	// Default value is 24 * 1/3000000 = 8 microseconds
	uint32_t _delay24cycles = 8; // Delay for this many microseconds before performing a read

	// Use instructionInProgress to keep track of which instruction is currently in progress.
	// instructionInProgress will be ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS when the ARTIC is idle.
	// It will be set to one of the INST_ states by sendMCUinstruction.
	uint8_t _instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS;

	//Functions
	void configureBurstmodeRegister(ARTIC_R2_Burstmode_Register burstmode); // Configure the burst mode register
	void readMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords); // Read multiple words using burst mode. configureBurstmodeRegister must have been called first.
	void write24BitWord(uint32_t word); // Write a single 24-bit word using burst mode. configureBurstmodeRegister must have been called first.
	void writeTwo24BitWords(uint32_t word1, uint32_t word2); // Write two 24-bit words using burst mode. configureBurstmodeRegister must have been called first.
	void writeMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords); // Write multiple words using burst mode. configureBurstmodeRegister must have been called first.
	boolean setARGOSTxFrequency(uint16_t mem_loc, float freq_MHz); // Set mem_loc to the desired TX frequency.
	float getARGOSTxFrequency(uint16_t mem_loc); // Return the TX frequency from mem_loc in MHz
};

#endif
