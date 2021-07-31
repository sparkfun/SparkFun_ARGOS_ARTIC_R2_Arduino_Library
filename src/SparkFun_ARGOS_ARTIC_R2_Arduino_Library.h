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

#if (ARDUINO >= 100)
#include "Arduino.h" // ESP32 needs this - otherwise the code fails to compile
#else
#include "WProgram.h"
#endif

#include <time.h> // Needed for epoch calculation

#include <SPI.h> // Needed for SPI communication

#include <Wire.h> // Needed for I2C communication with the PCA9536 on the smôl ARTIC R2

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//#define ARTIC_R2_UPLOAD_FIRMWARE // Uncomment this line to configure the ARTIC R2 firmware via SPI, instead of booting from flash memory

#include "ARTIC_R2_Firmware.h" // This file defines the firmware parameter memory locations. The locations for ARTIC004 and ARTIC006 are very different!

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

typedef enum {
	ARTIC_R2_BOARD_SHIELD = 0,	// The original Shield / Breakout
	ARTIC_R2_BOARD_IOTA,		// IOTA
	ARTIC_R2_BOARD_SMOL			// smôl ARTIC R2
} ARTIC_R2_Board_Type_e;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// SCLK is normally low (CPOL=0). Data is valid/latched on the falling SCLK edge (CPHA=1). So we need to use SPI MODE1.
#define ARTIC_R2_SPI_MODE SPI_MODE1

#define ARTIC_R2_TX_POWER_ON_DELAY_MS 50 // Wait for this many milliseconds after setting the TX gain pins before enabling the power
#define ARTIC_R2_POWER_ON_DELAY_MS 250 // Wait for this many milliseconds after enabling the power before attempting to communicate with the ARTIC
#define ARTIC_R2_FLASH_BOOT_TIMEOUT_MS 2500 // ARTIC should boot in 2.25 secs. Timeout after 2500ms.
#define ARTIC_R2_BOOT_TIMEOUT_MS 500 // Datasheet says ARTIC should boot in 0.25 secs after firmware upload. Timeout after 500ms.
//#define ARTIC_R2_BOOT_TIMEOUT_MS 10000 // Arribada Horizon waits for up to 10 seconds
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

#define ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_BYTES 660 // Length of the TX Payload in XMEM (220 * 24-bit words)
#define ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_WORDS 220 // Length of the TX Payload in XMEM (220 * 24-bit words)

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

// ARGOS Platform ID Length Bits
const uint8_t ARTIC_R2_PLATFORM_ID_BITS = 28;

// The ARGOS PTT-A2 message format is defined in:
// Platform Transmitter Terminal (PTT-A2) - Platform Message Transceiver (PMT-A2)
// - Physical Layer System Requirements, AS3-SP-516-2098-CNES

// ARGOS PTT-A2 Message Length Bits
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_BITS = 4;

// ARGOS PTT-A2 Message Lengths
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_24 = 0x0; // 24 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_56 = 0x3; // 56 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_88 = 0x5; // 88 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_120 = 0x6; // 120 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_152 = 0x9; // 152 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_184 = 0xA; // 184 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_216 = 0xC; // 216 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_MESSAGE_LENGTH_248 = 0xF; // 248 user bits (with a 28-bit Platform ID)

// ARGOS PTT-A2 User Message Bits
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_1 = 24; // 24 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_2 = 56; // 56 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_3 = 88; // 88 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_4 = 120; // 120 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_5 = 152; // 152 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_6 = 184; // 184 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_7 = 216; // 216 user bits (with a 28-bit Platform ID)
const uint8_t ARTIC_R2_PTT_A2_USER_BITS_N_8 = 248; // 248 user bits (with a 28-bit Platform ID)

// The ARGOS PTT-A3 message format is defined in:
// Platform Transmitter Terminal (PTT-A3, including PTT-ZE) - Platform Message Transceiver (PMT-A3)
// - Physical Layer System Requirements, AS3-SP-516-274-CNES

// ARGOS 3 PTT-A3 Message Length Bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_BITS = 4;

// ARGOS 3 PTT-A3 Message Lengths
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_24 = 0x0; // 24 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_56 = 0x3; // 56 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_88 = 0x5; // 88 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_120 = 0x6; // 120 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_152 = 0x9; // 152 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_184 = 0xA; // 184 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_216 = 0xC; // 216 user bits
const uint8_t ARTIC_R2_PTT_A3_MESSAGE_LENGTH_248 = 0xF; // 248 user bits

// ARGOS 3 PTT-A3 User Message Bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_1 = 24; // 24 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_2 = 56; // 56 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_3 = 88; // 88 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_4 = 120; // 120 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_5 = 152; // 152 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_6 = 184; // 184 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_7 = 216; // 216 user bits
const uint8_t ARTIC_R2_PTT_A3_USER_BITS_N_8 = 248; // 248 user bits

// ARGOS 3 PTT-A3 Number Of Tail Bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_24 = 7; // 24 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_56 = 8; // 56 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_88 = 9; // 88 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_120 = 7; // 120 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_152 = 8; // 152 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_184 = 9; // 184 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_216 = 7; // 216 user bits
const uint8_t ARTIC_R2_PTT_A3_NUM_TAIL_BITS_248 = 8; // 248 user bits

// ARGOS 3 PTT-ZE Number Of Tail Bits
const uint8_t ARTIC_R2_PTT_ZE_NUM_TAIL_BITS = 8;

// The ARGOS PTT-HD-A3 message format is defined in:
// Platform Transmitter Terminal (PTT-HD-A3) Platform Message Transceiver (PMT-HD-A3) Physical Layer Requirements
// AS3-SP-516-273-CNES

// ARGOS PTT-HD-A3 Message Length Bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_BITS = 8;

// ARGOS PTT-HD-A3 Message Lengths
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_32 = 0x00; // 32 user bits (Message service bit = 0)
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_512 = 0x16; // 512 user bits (Message service bit = 0)
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_1024 = 0x2D; // 1024 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_1536 = 0x3B; // 1536 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_2048 = 0x4F; // 2048 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_2560 = 0x59; // 2560 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_3072 = 0x63; // 3072 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_3584 = 0x75; // 3584 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_4096 = 0x8B; // 4096 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_MESSAGE_LENGTH_4608 = 0x9D; // 4608 user bits

// ARGOS PTT-HD-A3 Number Of Tail Bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_32 = 8; // 32 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_512 = 8; // 512 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_1024 = 9; // 1024 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_1536 = 7; // 1536 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_2048 = 8; // 2048 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_2560 = 9; // 2560 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_3072 = 7; // 3072 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_3584 = 8; // 3584 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_4096 = 9; // 4096 user bits
const uint8_t ARTIC_R2_PTT_A3_HD_NUM_TAIL_BITS_4608 = 7; // 4608 user bits

// The ARGOS 4 MD and HD message format is defined in:
// High Data Rate Platform Transmitter Terminal For ARGOS 4 (PTT/PMT-HD-A4) Physical layer requirements
// A4-SS-TER-SP-0078-CNES

// TO DO: Add the definitions needed for MD and HD messages:
// W0 = 0xCC33 33C3 CC3C CC3C
// W1 = 0x33CC CC3C 33C3 33C3
// Message Length: MD 1 user packet = W0 W0 W0 W0
// Message Length: MD 2 user packets = W0 W0 W1 W1
// Message Length: HD 1 user packet = W0 W1 W0 W1
// Message Length: HD 2 user packets = W0 W1 W1 W0
// Message Length: HD 3 user packets = W1 W0 W0 W1
// Message Length: HD 4 user packets = W1 W0 W1 W0
// Message Length: HD 5 user packets = W1 W1 W0 W0
// Num Tail Bits = 3
// End Word = 0xE312 B65F

// The ARGOS PTT-VLD-A4 (Very Low Data Rate) message format is defined in:
// Platform Transmitter Terminal (PTT-VLD-A4) - Physical Layer Requirements
// A4-SS-TER-SP-0079-CNES

// From the ARTIC R2 datasheet:
// "The message length is coded inside the synchronization pattern which is added by the ARTIC."

// So, we do not need to define this:
// ARGOS 4 VLD Message Length Bits
//const uint8_t ARTIC_R2_PTT_A4_VLD_MESSAGE_LENGTH_BITS = 2;

// Or these:
// ARGOS 4 VLD Message Lengths (2-bit)
//const uint8_t ARTIC_R2_PTT_A4_VLD_MESSAGE_LENGTH_SHORT = 0x0; // 0 user bits
//const uint8_t ARTIC_R2_PTT_A4_VLD_MESSAGE_LENGTH_LONG = 0x3; // 56 user bits

// ARGOS 4 VLD Number Of Tail Bits
const uint8_t ARTIC_R2_PTT_A4_VLD_NUM_TAIL_BITS = 6;

// ARGOS 4 VLD Short messsage length in bits
// 28-bit Platform ID plus 6 tail bits
const uint8_t ARTIC_R2_PTT_A4_VLD_SHORT_NUM_MESSAGE_BITS = 34;

// ARGOS 4 VLD Long messsage length in bits
// 28-bit Platform ID plus 6 tail bits
// plus 28 data bits plus 6 tail bits plus 28 data bits plus 6 tail bits
const uint8_t ARTIC_R2_PTT_A4_VLD_LONG_NUM_MESSAGE_BITS = 102;

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
	uint32_t payloadLength; // 24 bits
	uint32_t addresseeIdentification; // 28 bits
	uint8_t ADCS; // 4 bits
	uint8_t service; // 8 bits
	uint8_t payload[17]; // Maxmimum payload length is 17 bytes (136 bits)
	uint16_t FCS; // 16 bits
} Downlink_Message;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Satellite Pass Prediction - taken from Arribada Horizon
// Thank you Arribada!
// Originally written by CLS
// Code is taken from Arribada's prepas.h

#define ARTIC_R2_MAX_NUM_SATS (10) // Create some headroom for the next two ANGELS satellites. Was 8.

#define ARTIC_R2_MAXLU 132
#define sortie_erreur -1

#ifndef ARTIC_R2_MINIMUM_PAS
#define ARTIC_R2_MINIMUM_PAS 1
#endif

typedef struct
{
    float pf_lon;            /* geodetic position of the beacon */
    float pf_lat;            /* geodetic position of the beacon */
    long  time_start;
    long  time_end;
    int   s_differe;
    float site_min_requis;   /* min site pour calcul de la duree (deg.) */
    float site_max_requis;   /* min site (deg.) */
    float marge_temporelle;  /* marge temporelle (min/6mois) */
    float marge_geog_lat;    /* marge geographique (deg) */
    float marge_geog_lon;    /* marge geographique (deg) */
    int   Npass_max;         /* nombre de passages max par satellite */
} configurationParameters;

typedef struct
{
    char  sat[2];
    long  time_bul;
    float dga;        /* semi-major axis (km) */
    float inc;        /* orbit inclination (deg) */
    float lon_asc;    /* longitude of ascending node (deg) */
    float d_noeud;    /* asc. node drift during one revolution (deg) */
    float ts;         /* orbit period (min) */
    float dgap;       /* drift of semi-major axis (m/day) */
} orbitParameters;

typedef struct
{
    char sat[2];
    long tpp;        /* date du prochain passage (sec90) */
    int  duree;      /* duree (sec) */
    int  site_max;   /* site max dans le passage (deg) */
} predictionParameters;

typedef struct //__attribute__((__packed__))
{
    char sat[2]; // The two-character satellite identifier
    uint32_t time_bulletin; // Time of the orbit bulletin in seconds since the epoch
		// The params are:
		// semi-major axis (km)
		// orbit inclination (deg)
		// longitude of ascending node (deg)
		// asc. node drift during one revolution (deg)
		// orbit period (min)
		// drift of semi-major axis (m/day)
    float params[6];
} bulletin_data_t;

#define ARTIC_R2_AOP_ENTRY_WIDTH 85 // Width of one row in the AOP table from ARGOS Web

/* +-------------------------------------------------------------------+*/
/* +                      C O N S T A N T E S                          +*/
/* +-------------------------------------------------------------------+*/

#define SECONDS_IN_DAY (24 * 60 * 60)

const   float   pi  = 3.1415926535;          /* Pi     value     */
const   float   demi_pi = 1.570796327;       /* Pi/2   value     */
const   float   two_pi  = 6.283185307;       /* 2*pi   value     */
const   float   deg_rad = 0.017453292;       /* pi/180 value     */
const   float   rad_deg = 57.29577951;       /* 180/pi value     */

const   int pas = ARTIC_R2_MINIMUM_PAS;      /* en (sec)     */

const   float   rt  = 6378.137;              /* Earth radius     */
const   float   rs  = 7200;                  /* Orbit radius     */
const   float   rs_a  = 6890;                /* Orbit radius for ANGELS (Added by Paul) */

const int TIME_CONVERTOR = 631152000; /* Change apoch from 1990 to 1970 */

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

class ARTIC_R2
{
public:
	// The maximum SPI clock speed for ARTIC read operations from the X/Y/IO memory is 1.25MHz, so let's play safe and default to 1MHz
	boolean begin(int user_CSPin, int user_RSTPin, int user_BOOTPin, int user_ARTICPWRENPin, int user_RFPWRENPin, int user_INT1Pin, int user_INT2Pin, int user_GAIN8Pin = -1, unsigned long spiPortSpeed = 1000000, SPIClass &spiPort = SPI);
	boolean beginIOTA(int user_CSPin, int user_RSTPin, int user_BOOTPin, int user_IOTAPWRENPin, int user_INT1Pin, int user_INT2Pin, int user_GAIN8Pin = -1, unsigned long spiPortSpeed = 1000000, SPIClass &spiPort = SPI);
	boolean beginSmol(int user_CSPin, int user_ARTICPWRENPin, unsigned long spiPortSpeed = 1000000, SPIClass &spiPort = SPI, TwoWire &wirePort = Wire);

	void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

	boolean attenuateTXgain(boolean attenuate = false); // If attenuate is true, the TX gain is attenuated by approx. 5dB. Default to full power.

	void enableARTICpower(); // Enable power for the ARTIC R2 by pulling the power enable pin high
	void disableARTICpower(); // Disable power for the ARTIC R2 by pulling the power enable pin low
	boolean enableRFpower(); // Enable power for the RF amplifier by pulling the power enable pin high
	boolean disableRFpower(); // Disable power for the RF amplifier by pulling the power enable pin low

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
	uint32_t readPlatformID(); // From v1.1.0: read the Platform ID from the final PMEM location
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

	boolean setTCXOControl(float voltage = 1.8, bool autoDisable = true); // Set the TCXO control voltage and auto-disable. Default to 1.8V and leave enabled.
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
	boolean setPayloadARGOS3ZE(); // Set the Tx payload for a ARGOS 3 ZE message
	boolean setPayloadARGOS3LatLon(float Lat, float Lon); // Set the Tx payload for a ARGOS 3 PTT-A3 message containing GPS lat & lon in a compact form which ARGOS Web understands
	boolean setPayloadARGOS2LatLon(float Lat, float Lon); // Set the Tx payload for a ARGOS PTT-A2 message containing GPS lat & lon in a compact form which ARGOS Web understands
	boolean setPayloadARGOS4VLDshort(); // Set the Tx payload for a ARGOS 4 VLD message with 0 bits of user data
	boolean setPayloadARGOS4VLDLatLon(float Lat, float Lon); // Set the Tx payload for a ARGOS 4 VLD message containing GPS lat & lon in a compact form which ARGOS Web understands
	boolean setPayloadARGOS4VLDLong(uint32_t payload1, uint32_t payload2); // Set the Tx payload for a ARGOS 4 VLD Long message containing two 28-bit payload words

	// Set the Tx payload for a ARGOS PTT-A2 message containing Nx32_bits * 32 bit words (1<=N<=8)
	// Special note:
	//   Because the platform ID is 28 bits (not 20), only the 24 least-significant bits of the first payload uint32_t are used.
	//   The 8 most-significant bits of the first payload uint32_t are ignored.
	//   All 32 bits of the second and subsequent payload uint32_t's are included in the message.
	boolean setPayloadARGOS2(uint8_t Nx32_bits, uint32_t *payload);

	// Set the Tx payload for a ARGOS PTT-A3 message containing Nx32_bits * 32 bit words (1<=N<=8)
	// Special note:
	//   Only the 24 least-significant bits of the first payload uint32_t are used.
	//   The 8 most-significant bits of the first payload uint32_t are ignored.
	//   All 32 bits of the second and subsequent payload uint32_t's are included in the message.
	boolean setPayloadARGOS3(uint8_t Nx32_bits, uint32_t *payload);

	// Storage for message transmission
	// This storage is used by the setPayloadARGOS functions
	// It should probably be private, but it is public just in case the user wants
	// to assemble their own payload.
	uint8_t _txPayloadBytes[ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_BYTES]; // Storage for up to 220 24-bit words

	// This function copies _txPayloadLengthBits and _txPayloadBytes to the TX Payload in XMEM
	// It gets called by the setPayloadARGOS functions
	// This function should probably be private, but it is public just in case the user wants
	// to assemble their own payload.
	// It returns true if the payload was copied successfully.
	boolean setTxPayload();

	// This function will read the payload from ARTIC XMEM into back _txPayloadBytes so the user can
	// check that the payload was written correctly.
	// NOTE: this of course overwrites the contents of _txPayloadBytes!
	// NOTE: The ARTIC datasheet indicates that TX Payload is write-only. So, strictly, this function
	//   should not work. But it seems to work just fine...
	void readTxPayload();
	void printTxPayload(Stream &port = Serial); // Pretty-print the Tx payload

	// Arribada / CLS Satellite Pass Predictor
	uint32_t predictNextSatellitePass(bulletin_data_t *bulletin, float min_elevation, const uint8_t number_sat, float lon, float lat, long current_time, int max_npass = 1);

	// Satellite pass prediciton helper tools
	boolean convertAOPtoParameters(const char *AOP, bulletin_data_t *satelliteParameters, const uint8_t number_sat); // Convert the AOP from text to bulletin_data_t
	char* const convertEpochToDateTime(uint32_t epoch); // Convert the epoch from the satellite predictor to a date & time string
	char* const convertEpochToDateTimeAOP(uint32_t epoch); // Convert the epoch from the satellite predictor to a date & time string in AOP format
	uint32_t convertGPSTimeToEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second); // Convert GPS date & time to epoch
	boolean printAOPbulletin(bulletin_data_t bulletin, Stream &port = Serial); // Pretty-print the AOP bulletin
	uint32_t convertAllcastDateTimeToEpoch(const char *DateTime); // Convert the Allcast JSON Date Time to Epoch

private:
	//Variables
	ARTIC_R2_Board_Type_e _board;				// Shield, IOTA or smôl

	Stream *_debugPort = NULL;		//The stream to send debug messages to if enabled. Usually Serial.
	boolean _printDebug = false;	//Flag to print debugging variables

	SPIClass *_spiPort = NULL;		// The generic connection to user's chosen SPI hardware
	unsigned long _spiPortSpeed = 1000000;	// Optional user defined port speed. Default to 1MHz

	int _cs;				// ARTIC R2 SPI Chip Select
	int _rst = -1;			// ARTIC R2 Reset pin
	int _boot = -1;			// ARTIC R2 Boot pin
	int _artic_pwr_en;		// Pull this pin high to enable power for the ARTIC R2
	int _rf_pwr_en = -1;	// Pull this pin high to enable power for the RF amplifier
	int _int1 = -1;			// ARTIC R2 Interrupt 1 pin
	int _int2 = -1;			// ARTIC R2 Interrupt 2 pin
	int _gain8 = -1;		// Pull this pin high to use the full transmit power

    TwoWire *_i2cPort = NULL;		//Connection to the PCA9536 on the smôl ARTIC R2

	// The user has to wait for the duration of 24 SPI clock cycles after configuring the burst read mode, before starting the first read.
	// This allows some time for the internal memory access block to retrieve the first data sample.
	// Default value is 24 * 1/1000000 = 24 microseconds
	uint32_t _delay24cycles = 24; // Delay for this many microseconds before performing a read

	// Use instructionInProgress to keep track of which instruction is currently in progress.
	// instructionInProgress will be ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS when the ARTIC is idle.
	// It will be set to one of the INST_ states by sendMCUinstruction.
	uint8_t _instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS;

	// From v1.1.0: the Platform ID is programmed by SparkFun into the last PMEM location during production testing
	// In .begin, the Platform ID is read from PMEM and stored here:
	uint32_t _platformID = 0; // Default to zero as that is what will be read from memory on earlier SparkFun boards

	//Functions
	boolean beginInternal();
	void configureBurstmodeRegister(ARTIC_R2_Burstmode_Register burstmode); // Configure the burst mode register
	void readMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords); // Read multiple words using burst mode. configureBurstmodeRegister must have been called first.
	void write24BitWord(uint32_t word); // Write a single 24-bit word using burst mode. configureBurstmodeRegister must have been called first.
	void writeTwo24BitWords(uint32_t word1, uint32_t word2); // Write two 24-bit words using burst mode. configureBurstmodeRegister must have been called first.
	void writeMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords); // Write multiple words using burst mode. configureBurstmodeRegister must have been called first.
	boolean setARGOSTxFrequency(uint16_t mem_loc, float freq_MHz); // Set mem_loc to the desired TX frequency.
	float getARGOSTxFrequency(uint16_t mem_loc); // Return the TX frequency from mem_loc in MHz

	// Arribada / CLS Satellite Pass Predictor
	void su_distance(long   t1,     /* input */

	                 float  x_pf,
	                 float  y_pf,
	                 float  z_pf,
	                 float  ws,
	                 float  sin_i,
	                 float  cos_i,
	                 float  asc_node,
	                 float  wt,

	                 float  *d2);       /* output */
	int select_closest(predictionParameters *pt_pp, int number_sat, uint32_t desired_time);
	void print_list(predictionParameters * p_pp,  int number_sat);
	void print_config(configurationParameters *p_pc);
	void print_sat(orbitParameters *p_po, int number_sat);
	int satellitePassPrediction(configurationParameters *p_pc, orbitParameters *p_po, predictionParameters *p_pp, int number_sat);
	float textToFloat(const char *ptr, uint8_t digitsBeforeDP, uint8_t digitsAfterDP);

	// smôl specifics
	const uint8_t SMOL_PCA9536_I2C_ADDRESS = 0x41;
	const uint8_t SMOL_PCA9536_INPUT_PORT = 0x00;
	const uint8_t SMOL_PCA9536_OUTPUT_PORT = 0x01;
	const uint8_t SMOL_PCA9536_CONFIGURATION_REGISTER = 0x03;
	boolean beginPCA9536();
	boolean setSmolG8(uint8_t highLow);     // Gain8  = PCA9536 GPIO3
	boolean setSmolBOOT(uint8_t highLow);   // BOOT   = PCA9536 GPIO2
	uint8_t getSmolINT1();               // INT1   = PCA9536 GPIO1
	boolean setSmolRESETB(uint8_t highLow); // RESETB = PCA9536 GPIO0
	boolean setPCA9536Output(uint8_t highLow, uint8_t GPIO);

	// GPIO helper functions
	boolean configureBootPin();
	boolean setRESETBPin(uint8_t highLow);
	uint8_t getINT1();
};

#endif
