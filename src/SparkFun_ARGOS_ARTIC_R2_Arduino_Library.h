/*
  This is a library written for the ARGOS ARTIC R2 Breakout
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/

  Written by Paul Clark, August 17th 2020

  The ARGOS ARTIC R2 chipset allows you to send and receive short data messages via the
	ARGOS satellite system.

  This library handles SPI communication with the chipset.

  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	See the LICENSE.md for more details.

	Parts of this library have been remixed from the Arribada Horizon bio-logging platform:
	https://github.com/arribada/horizon
	Arribada's work is gratefully acknowledged.

*/

#ifndef ARTIC_R2_H
#define ARTIC_R2_H

#include <SPI.h> // Needed for SPI communication
// SCLK is normally low (CPOL=0). Data is valid/latched on the falling SCLK edge (CPHA=1). So we need to use SPI MODE1.
#define ARTIC_R2_SPI_MODE SPI_MODE1

#define ARTIC_R2_FLASH_BOOT_TIMEOUT 2500 // ARTIC should boot in 2.25 secs. Timeout after 2500ms.
#define ARTIC_R2_BOOT_TIMEOUT 500 // ARTIC should boot in 0.25 secs after firmware upload. Timeout after 500ms.

//#include "ARTIC.h" // Include Arribada's ARTIC header file

#define ARTIC_R2_UPLOAD_FIRMWARE // Comment this line to save memory once the flash memory on the ARTIC R2 Breakout has been programmed successfully

#ifdef ARTIC_R2_UPLOAD_FIRMWARE
// Include firmware binary data
// P 32-bit 10240 DSP Program memory
// X 24-bit 21845 DSP X memory
// Y 24-bit 6826 DSP Y memory
#include "Firmware_ARTIC004_flash_image__PMEM.h"
#include "Firmware_ARTIC004_flash_image__XMEM.h"
#include "Firmware_ARTIC004_flash_image__YMEM.h"
#endif

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Firmware status register (0x8018)
typedef struct {
	union {
		uint32_t STATUS_REGISTER;
		struct {
			// Current Firmware state
			uint32_t IDLE								: 1; // The firmware is idle and ready to accept commands
			uint32_t RX_IN_PROGRESS			: 1; // The firmeware is receiving
			uint32_t TX_IN_PROGRESS			: 1; // The firmware is transmitting
			uint32_t BUSY								: 1; // The firmware is changing state
			// Interrupt 1 flags
			uint32_t RX_VALID_MESSAGE					: 1; // A message has been received
			uint32_t RX_SATELLITE_DETECTED		: 1; // A saellite has been detected
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
			uint32_t BURSTMODE_START_ADDR	: 16; //Burstmode Start Address
			uint32_t BURST_R_RW_MODE			: 1;
			uint32_t BURST_MEM_SEL				: 2; // Burstmode memory select: 00: Program Memory; 01: X Memory; 10: Y Memory; 11: IO Memory
			uint32_t BURST_MODE_ON				: 1;
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
const uint8_t CONFIG_CMD_SET_ARGOS_4_RX_MODE = 0x01;
const uint8_t CONFIG_CMD_SET_ARGOS_3_RX_MODE = 0x02;
const uint8_t CONFIG_CMD_SET_ARGOS_3_RX_BACKUP_MODE = 0x03;
const uint8_t CONFIG_CMD_SET_PTT_A2_TX_MODE = 0x04;
const uint8_t CONFIG_CMD_SET_PTT_A3_TX_MODE = 0x05;
const uint8_t CONFIG_CMD_SET_PTT_ZE_TX_MODE = 0x06;
const uint8_t CONFIG_CMD_SET_ARGOS_3_PTT_HD_TX_MODE = 0x07;
const uint8_t CONFIG_CMD_SET_ARGOS_4_PTT_HD_TX_MODE = 0x08;
const uint8_t CONFIG_CMD_SET_ARGOS_4_PTT_MD_TX_MODE = 0x09;
const uint8_t CONFIG_CMD_SET_ARGOS_4_PTT_VLD_TX_MODE = 0x0A;

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

// MCU Housekeeping: command words
const uint8_t CMD_CLEAR_INT_1 = 0x80; // Clear interrupt line 1
const uint8_t CMD_CLEAR_INT_2 = 0xC0; // Clear interrupt line 2

// P Memory Locations
const uint16_t MEM_LOC_FIRMWARE_VERSION = 0x0010; // 2 * 32-bit words = 8 bytes: 'ARTICnnn'

// X Memory locations
const uint16_t MEM_LOC_ARGOS_CONFIGURATION = 0x0384; // Size 1. Read only
const uint16_t MEM_LOC_RX_PAYLOAD = 0x0200; // Size 9. Read only
const uint16_t MEM_LOC_RX_FILTERING_CONFIGURATION = 0x0209; // Size 104. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_ENABLE_CRC = 0x0209; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_TRANSPARENT_MODE = 0x020A; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_LUT_LENGTH = 0x020C; // Size 1. Read/Write
const uint16_t MEM_LOC_RX_FILTERING_LUT_FIRST_ADDRESS = 0x020D; // Read/Write
const uint16_t MEM_LOC_RX_TIMEOUT = 0x0271; // Size 1. Read/Write
const uint16_t MEM_LOC_SATELLITE_DETECTION_TIMEOUT = 0x272; // Size 1. Read/Write
const uint16_t MEM_LOC_TX_PAYLOAD = 0x273; // Size 220. Write only. == Arribada's "TX_PAYLOAD_ADDRESS"
const uint16_t MEM_LOC_TX_FREQ_ARGOS_2_3 = 0x034F; // Size 1. Read?Write
const uint16_t MEM_LOC_TX_FREQ_ARGOS_4 = 0x035F; // Size 1. Read/Write
const uint16_t MEM_LOC_TCXO_WARMUP_TIME = 0x036F; // Sizde 1. Read/Write
const uint16_t MEM_LOC_TCXO_CONTROL = 0x0370; // Size 1. Read/Write
const uint16_t MEM_LOC_CRC_RESULTS = 0x0371; // Size 3. Read only. == Arribada's "CRC_ADDRESS"
const uint16_t MEM_LOC_TX_CERTIFICATION_INTERVAL = 0x0379; // Size 1. Read/Write

// IO Memory locations
const uint16_t MEM_LOC_FIRMWARE_STATUS_REGISTER = 0x8018; // == Arribada's "INTERRUPT_ADDRESS"

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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

class ARTIC_R2
{
public:
	boolean begin(uint8_t user_CSPin, uint8_t user_RSTPin, uint8_t user_BOOTPin, uint8_t user_PWRENPin, uint8_t user_INT1Pin, uint8_t user_INT2Pin, uint8_t user_GAIN8Pin = -1, uint8_t user_GAIN16Pin = -1, uint32_t spiPortSpeed = 3000000, SPIClass &spiPort = SPI);

	void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

	boolean setTXgain(int gain = 24); // Set the TX gain (valid valyes are 0,8,16,24)

	void enableARTICpower(); // Enable power for the ARTIC R2 by pulling the power enable pin low
	void disableARTICpower(); // Disable power for the ARTIC R2 by pulling the power enable pin high

	void readStatusRegister(ARTIC_R2_Firmware_Status *status); // Read the ARTIC R2 status register
	void sendCommandByte(uint8_t command); // Send a single 8-bit command
	boolean clearInterrupts(uint8_t interrupts = 3); // Clear one or both interrupts. Default to both.

	void readFirmwareVersion(uint8_t *buffer); // Read the firmware version from PMEM
	void readMemoryCRC(uint32_t *PMEM_CRC, uint32_t *XMEM_CRC, uint32_t *YMEM_CRC); // Read the memories CRCs (after firmware boot)

	void setRxTimeout(uint32_t timeout_secs = 0x0A); // Set the RX timeout (seconds). Default to 10.
	void setSatelliteDetectionTimeout(uint32_t timeout_secs = 0x02); // Set the satellite detection timeout (seconds). Default to 2.
	void setTCXOWarmupTime(uint32_t timeout_secs = 0x0A); // Set the TCXO warm up time (seconds). Default to 10.
	void setTxCertificationInterval(uint32_t timeout_secs = 0x02); // Set the TX certification interval

	void readARGOSconfiguration(ARGOS_Configuration_Register *configuration); // Read the ARGOS configuration register
	uint32_t readRxTimeout(); // Read the RX timeout
	uint32_t readSatelliteDetectionTimeout(); // Read the satellite detection timeout
	uint32_t readTCXOWarmupTime(); // Read the TCXO warm up time
	uint32_t readTxCertificationInterval(); // Read the TX certification interval

	boolean setTCXOControl(float voltage = 3.3, bool autoDisable = true); // Set the TCXO control voltage and auto-disable. Default to 3.3V and leave enabled.
	float readTCXOControlVoltage(); // Read the TCXO control voltage. Auto-disable is ignored.
	boolean readTCXOAutoDisable(); // Read the TCXO control auto-disable bit

	boolean enableRXCRC(); // Enable RX CRC check
	boolean disableRXCRC(); // Disable RX CRC check
	boolean enableRXTransparentMode(); // Enable RX transparent mode
	boolean disableRXTransparentMode(); // Disable RX transparent mode
	boolean clearAddressLUT(); // Clear the address look-up-table by setting the length to zero
	boolean addAddressToLUT(uint32_t AddressLSBits, uint32_t AddressMSBits); // Add the specified address to the message filter Look Up Table

	boolean readDownlinkMessage(uint32_t *payloadLength, uint32_t *addresseeIdentification, uint8_t *ADCS, uint8_t *service, uint8_t *rxData, uint16_t *FCS); // Read a downlink message from the RX payload buffer

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

	//Functions
	void configureBurstmodeRegister(ARTIC_R2_Burstmode_Register burstmode);
	void readMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords);
	void write24BitWord(uint32_t word);
	void writeTwo24BitWords(uint32_t word1, uint32_t word2);
};

#endif
