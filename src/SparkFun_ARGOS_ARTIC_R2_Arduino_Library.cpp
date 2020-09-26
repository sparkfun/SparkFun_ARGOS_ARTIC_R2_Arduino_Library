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

	Parts of this library were inspired by the Arribada Horizon bio-logging platform:
	https://github.com/arribada/horizon
	Arribada's work is gratefully acknowledged.

*/

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h"

boolean ARTIC_R2::begin(uint8_t user_CSPin, uint8_t user_RSTPin, uint8_t user_BOOTPin, uint8_t user_PWRENPin, uint8_t user_INT1Pin, uint8_t user_INT2Pin, uint8_t user_GAIN8Pin, uint8_t user_GAIN16Pin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	//Get user settings
	_spiPort = &spiPort;
	_spiPortSpeed = spiPortSpeed;
	if (_spiPortSpeed > 25000000)
		_spiPortSpeed = 25000000; //Datasheet indicates max speed is 25MHz

	_delay24cycles = 24000000 / _spiPortSpeed; // Calculate the 24-cycle read delay based on the clock speed
	_delay24cycles++; // Round up by 1

	// Mandatory pins
	_cs = user_CSPin;
	_boot = user_BOOTPin;
	_rst = user_RSTPin;
	_pwr_en = user_PWRENPin;
	_int1 = user_INT1Pin;
	_int2 = user_INT2Pin;

	// Optional pins
	_gain8 = user_GAIN8Pin;
	_gain16 = user_GAIN16Pin;

	pinMode(_pwr_en, OUTPUT);
	digitalWrite(_pwr_en, HIGH); // Disable the power until we have configured the rest of the IO pins

	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH); //Deselect ARTIC

	pinMode(_boot, OUTPUT);
	// The ARTIC will boot from the [onboard] external flash memory when the boot pin is high and reset is released.
	// If the boot pin is held low at reset the ARTIC will wait for the MCU to upload the Firmware.
#ifdef ARTIC_R2_UPLOAD_FIRMWARE
	digitalWrite(_boot, LOW); // Get ready to upload the firmware
#else
	digitalWrite(_boot, HIGH); // Boot the ARTIC from flash memory
#endif

	pinMode(_rst, OUTPUT);
	digitalWrite(_rst, LOW); //Place the ARTIC into reset

	pinMode(_int1, INPUT_PULLUP);
	pinMode(_int2, INPUT_PULLUP);

	if ((_gain8 >= 0) && (_gain8 >= 16)) // Set the RF TX gain
	{
		pinMode(_gain8, OUTPUT);
		digitalWrite(_gain8, LOW); // Default to maximum gain
		pinMode(_gain16, OUTPUT);
		digitalWrite(_gain16, LOW); // Default to maximum gain
	}

	digitalWrite(_pwr_en, LOW); // Enable power for the ARTIC R2
	delay(100);
	digitalWrite(_rst, HIGH); //Bring the ARTIC out of reset

#ifdef ARTIC_R2_UPLOAD_FIRMWARE

	// Upload ARTIC firmware: PMEM
	delay(100); // Wait 100ms

	if (_printDebug == true)
		_debugPort->println(F("Uploading ARTIC firmware: PMEM..."));

	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start at address 0
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_P_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delay(1);

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (uint32_t addr = 0; addr < ARTIC_R2_PMEM_LEN; addr++)
	{
		uint32_t word = ARTIC_R2_PMEM[addr];
		_spiPort->transfer(word >> 24);
		_spiPort->transfer(word >> 16);
		_spiPort->transfer(word >> 8);
		_spiPort->transfer(word & 0xFF);
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	// Upload ARTIC firmware: XMEM
	delay(10); // Wait 10ms

	if (_printDebug == true)
		_debugPort->println(F("Uploading ARTIC firmware: XMEM..."));

	burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start at address 0
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delay(1);

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (uint32_t addr = 0; addr < ARTIC_R2_XMEM_LEN; addr++)
	{
		uint32_t word = ARTIC_R2_XMEM[addr];
		_spiPort->transfer(word >> 16);
		_spiPort->transfer(word >> 8);
		_spiPort->transfer(word & 0xFF);
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	// Upload ARTIC firmware: YMEM
	delay(10); // Wait 10ms

	if (_printDebug == true)
		_debugPort->println(F("Uploading ARTIC firmware: YMEM..."));

	burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start at address 0
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_Y_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delay(1);

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (uint32_t addr = 0; addr < ARTIC_R2_YMEM_LEN; addr++)
	{
		uint32_t word = ARTIC_R2_YMEM[addr];
		_spiPort->transfer(word >> 16);
		_spiPort->transfer(word >> 8);
		_spiPort->transfer(word & 0xFF);
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	// Activate the DSP
	delay(10); // Wait 10ms

	if (_printDebug == true)
		_debugPort->println(F("Uploading ARTIC firmware: activating the DSP..."));

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(0x02);
	_spiPort->transfer(0x00);
	_spiPort->transfer(0x00);
	_spiPort->transfer(0x00);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	// Wait for ARTIC to boot

	unsigned long bootStartTime = millis();

	while (((millis() - bootStartTime) < ARTIC_R2_BOOT_TIMEOUT) && (digitalRead(_int1) == LOW))
	{
		if (_printDebug == true)
			_debugPort->println(F("Uploading ARTIC firmware: waiting for the ARTIC to boot..."));
		delay(100);
	}

	if ((millis() - bootStartTime) >= ARTIC_R2_BOOT_TIMEOUT)
	{
		if (_printDebug == true)
			_debugPort->println(F("Uploading ARTIC firmware: boot timed out!"));
		return (false); // Boot timed out!
	}

	if(clearInterrupts(3) == false) // Clear both interrupts
	{
		if (_printDebug == true)
			_debugPort->println(F("Uploading ARTIC firmware: failed to clear interrupts!"));
		return (false);
	}

	// Read the checksum words
	uint32_t PMEM_CRC, XMEM_CRC, YMEM_CRC;
	readMemoryCRC(&PMEM_CRC, &XMEM_CRC, &YMEM_CRC);

	// Check that the checksums match
	if ((PMEM_CRC == ARTIC_R2_PMEM_CHECKSUM) && (XMEM_CRC == ARTIC_R2_XMEM_CHECKSUM) && (YMEM_CRC == ARTIC_R2_YMEM_CHECKSUM))
	{
		if (_printDebug == true)
			_debugPort->println(F("Uploading ARTIC firmware: checksums match"));
		return (true);
	}
	else
	{
		if (_printDebug == true)
			_debugPort->println(F("Uploading ARTIC firmware: checksums do not match!"));
		return (false);
	}

#else

	// ARTIC boots from flash memory
	delay(100); // Wait 100ms
	unsigned long bootStartTime = millis();

	while (((millis() - bootStartTime) < ARTIC_R2_FLASH_BOOT_TIMEOUT) && (digitalRead(_int1) == LOW))
	{
		if (_printDebug == true)
			_debugPort->println(F("ARTIC is booting from flash..."));
		delay(100);
	}

	if ((millis() - bootStartTime) >= ARTIC_R2_FLASH_BOOT_TIMEOUT)
		return (false); // Boot timed out!

	return (clearInterrupts(3)); // Clear both interrupts

#endif
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void ARTIC_R2::enableDebugging(Stream &debugPort)
{
	_debugPort = &debugPort;
	_printDebug = true;
}

//Set the RF TX gain
//Returns true if the gain pins have been defined and if the gain value is valid
boolean ARTIC_R2::setTXgain(int gain)
{
	if ((_gain8 >= 0) && (_gain16 >= 0))
	{
		if (gain == 0)
		{
			digitalWrite(_gain8, HIGH);
			digitalWrite(_gain16, HIGH);
			return (true);
		}
		else if (gain == 8)
		{
			digitalWrite(_gain8, LOW);
			digitalWrite(_gain16, HIGH);
			return (true);
		}
		else if (gain == 16)
		{
			digitalWrite(_gain8, HIGH);
			digitalWrite(_gain16, LOW);
			return (true);
		}
		else if (gain == 24)
		{
			digitalWrite(_gain8, LOW);
			digitalWrite(_gain16, LOW);
			return (true);
		}
		else
			return (false);
	}
	else
		return (false);
}

// Enable power for the ARTIC R2 by pulling the power enable pin low
void ARTIC_R2::enableARTICpower()
{
	digitalWrite(_pwr_en, LOW);
}

// Disable power for the ARTIC R2 by pulling the power enable pin high
void ARTIC_R2::disableARTICpower()
{
	digitalWrite(_pwr_en, HIGH);
}

// Read ARTIC R2 firmware status register
// The user can access the firmware status/interrupt flags by reading a single register.
// This 24-bit register is present in the IO memory at address 0x8018 and can be accessed using an SPI burst.
// This register can only be read.
// The complete 32-bit sequence is 0x000F8018. Next in a 24-bit SPI burst the register is read.
void ARTIC_R2::readStatusRegister(ARTIC_R2_Firmware_Status *status)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_FIRMWARE_STATUS_REGISTER;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_IO_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	status->STATUS_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status
}

// Pretty-print the firmware status to port
void ARTIC_R2::printFirmwareStatus(ARTIC_R2_Firmware_Status status, Stream &port)
{
	if (status.STATUS_REGISTER_BITS.IDLE) port.println(F("The firmware is idle and ready to accept commands."));
	if (status.STATUS_REGISTER_BITS.RX_IN_PROGRESS) port.println(F("The firmware is receiving."));
	if (status.STATUS_REGISTER_BITS.TX_IN_PROGRESS) port.println(F("The firmware is transmitting."));
	if (status.STATUS_REGISTER_BITS.BUSY) port.println(F("The firmware is busy changing state."));
	if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) port.println(F("A message has been received."));
	if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) port.println(F("A satellite has been detected."));
	if (status.STATUS_REGISTER_BITS.TX_FINISHED) port.println(F("The transmission was completed."));
	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED) port.println(F("The configuration command has been accepted."));
	if (status.STATUS_REGISTER_BITS.CRC_CALCULATED) port.println(F("The CRC calculation has finished."));
	if (status.STATUS_REGISTER_BITS.IDLE_STATE) port.println(F("The firmware has returned to the idle state."));
	if (status.STATUS_REGISTER_BITS.RX_CALIBRATION_FINISHED) port.println(F("The RX offset calibration has completed."));
	if (status.STATUS_REGISTER_BITS.RX_TIMEOUT) port.println(F("The specified reception time has been exceeded."));
	if (status.STATUS_REGISTER_BITS.SATELLITE_TIMEOUT) port.println(F("No satellite was detected within the specified time."));
	if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) port.println(F("A received message is lost. No buffer space left."));
	if (status.STATUS_REGISTER_BITS.TX_INVALID_MESSAGE) port.println(F("Incorrect TX payload length specified."));
	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED) port.println(F("Incorrect command sent or firmware is not in idle."));
	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW) port.println(F("Previous command was not yet processed."));
	if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR) port.println(F("An internal error has occurred."));
	if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) port.println(F("Interrupt pin 1 is high."));
	if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) port.println(F("Interrupt pin 2 is high."));
}

// Read ARTIC R2 ARGOS configuration register
void ARTIC_R2::readARGOSconfiguration(ARGOS_Configuration_Register *configuration)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_ARGOS_CONFIGURATION;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	configuration->CONFIGURATION_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status
}

// Pretty-print the ARGOS configuration
void ARTIC_R2::printARGOSconfiguration(ARGOS_Configuration_Register configuration, Stream &port)
{
		port.print(txConfigurationString(configuration)); // Print the TX configuration
		port.print(rxConfigurationString(configuration)); // Print the RX configuration
}

// Returns a human-readable version of the ARGOS TX configuration
const char *ARTIC_R2::txConfigurationString(ARGOS_Configuration_Register configuration)
{
	switch ((uint8_t)configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION)
	{
		case TX_CONFIG_ARGOS_PTT_A2_MODE:
			return "TX configuration: ARGOS PTT-A2.\r\n";
			break;
		case TX_CONFIG_ARGOS_PTT_A3_MODE:
			return "TX configuration: ARGOS PTT-A3.\r\n";
			break;
		case TX_CONFIG_ARGOS_PTT_ZE_MODE:
			return "TX configuration: ARGOS PTT-ZE.\r\n";
			break;
		case TX_CONFIG_ARGOS_PTT_HD_MODE:
			return "TX configuration: ARGOS PTT-HD.\r\n";
			break;
		case TX_CONFIG_ARGOS_PTT_A4_MD_MODE:
			return "TX configuration: ARGOS PTT-A4-MD.\r\n";
			break;
		case TX_CONFIG_ARGOS_PTT_A4_HD_MODE:
			return "TX configuration: ARGOS PTT-A4-HD.\r\n";
			break;
		case TX_CONFIG_ARGOS_PTT_A4_VLD_MODE:
			return "TX configuration: ARGOS PTT-A4-VLD.\r\n";
			break;
		default:
			return "TX configuration: UNDEFINED!\r\n";
			break;
		}
}

// Returns a human-readable version of the ARGOS RX configuration
const char *ARTIC_R2::rxConfigurationString(ARGOS_Configuration_Register configuration)
{
	switch ((uint8_t)configuration.CONFIGURATION_REGISTER_BITS.RX_CONFIGURATION)
	{
		case RX_CONFIG_ARGOS_3_RX_MODE:
			return "RX configuration: ARGOS 3.\r\n";
			break;
		case RX_CONFIG_ARGOS_3_RX_BACKUP_MODE:
			return "RX configuration: ARGOS 3 BACKUP.\r\n";
			break;
		case RX_CONFIG_ARGOS_4_RX_MODE:
			return "RX configuration: ARGOS 4.\r\n";
			break;
		default:
			return "RX configuration: UNDEFINED!\r\n";
			break;
		}
}

// Configure the burstmode Register
// The burstmode register occupies address 0x00 and is write only
void ARTIC_R2::configureBurstmodeRegister(ARTIC_R2_Burstmode_Register burstmode)
{
	// Write 32 bits to SPI:
	// Burstmode register address: 0b0000000
	// Write: 0b0
	// ARTIC_R2_Burstmode_Register: 24-bits
	uint32_t fourBytes = burstmode.BURSTMODE_REGISTER & 0xFFFFFF;

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(fourBytes >> 24);
	_spiPort->transfer(fourBytes >> 16);
	_spiPort->transfer(fourBytes >> 8);
	_spiPort->transfer(fourBytes & 0xFF);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();
}

// Send an MCU Configuration Command
// Write a single 8-bit configuration command
// Return the result
ARTIC_R2_MCU_Command_Result ARTIC_R2::sendConfigurationCommand(uint8_t command)
{
	// Check that the command is valid
	if((command != CONFIG_CMD_SET_ARGOS_3_RX_MODE) &&
		(command != CONFIG_CMD_SET_ARGOS_3_RX_BACKUP_MODE) &&
		//(command != CONFIG_CMD_SET_ARGOS_4_RX_MODE) &&  // Unsupported by ARTIC006 ?! TO DO: Check this!
		(command != CONFIG_CMD_SET_PTT_A2_TX_MODE) &&
		(command != CONFIG_CMD_SET_PTT_A3_TX_MODE) &&
		(command != CONFIG_CMD_SET_PTT_ZE_TX_MODE) &&
		(command != CONFIG_CMD_SET_ARGOS_3_PTT_HD_TX_MODE) &&
		(command != CONFIG_CMD_SET_ARGOS_4_PTT_HD_TX_MODE) &&
		(command != CONFIG_CMD_SET_ARGOS_4_PTT_MD_TX_MODE) &&
		(command != CONFIG_CMD_SET_ARGOS_4_PTT_VLD_TX_MODE))
		return ARTIC_R2_MCU_COMMAND_INVALID; // Command is invalid

	ARTIC_R2_Firmware_Status status; // Read the status register before attempting to send the command

	if (_printDebug == true)
	{
		readStatusRegister(&status);
		if (!status.STATUS_REGISTER_BITS.IDLE) // If the firmware is not idle
		{
			_debugPort->println(F("sendConfigurationCommand: ARTIC is not idle. This will probably fail."));
		}
	}

	// Write 8 bits to SPI:
	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(command);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles *** TO DO: Check if this needs to be extended! ***

	readStatusRegister(&status); // Read the status register again

	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED)
	{
		return ARTIC_R2_MCU_COMMAND_ACCEPTED;
	}
	else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED)
	{
		return ARTIC_R2_MCU_COMMAND_REJECTED;
	}
	else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW)
	{
		return ARTIC_R2_MCU_COMMAND_OVERFLOW;
	}
	else
	{
		return ARTIC_R2_MCU_COMMAND_UNCERTAIN; // Hopefully this is impossible?
	}
}

// Pretty-print the command result
void ARTIC_R2::printCommandResult(ARTIC_R2_MCU_Command_Result result, Stream &port)
{
	switch (result)
	{
		case ARTIC_R2_MCU_COMMAND_ACCEPTED:
			port.println(F("MCU command result: success. The configuration command has been accepted."));
			break;
		case ARTIC_R2_MCU_COMMAND_REJECTED:
			port.println(F("MCU command result: fail! Incorrect command sent or firmware is not in idle."));
			break;
		case ARTIC_R2_MCU_COMMAND_OVERFLOW:
			port.println(F("MCU command result: fail! Previous command was not yet processed."));
			break;
		case ARTIC_R2_MCU_COMMAND_UNCERTAIN:
			port.println(F("MCU command result: UNCERTAIN! (MCU did not raise ACCEPTED, REJECTED or OVERFLOW)"));
			break;
		case ARTIC_R2_MCU_COMMAND_INVALID:
			port.println(F("MCU command result: INVALID! (MCU command was invalid / not recognized)"));
			break;
		case ARTIC_R2_MCU_INSTRUCTION_IN_PROGRESS:
			port.print(F("MCU command result: fail! "));
			if (_instructionInProgress == INST_START_CONTINUOUS_RECEPTION)
				port.println(F("Continuous reception is in progress. Use INST_GO_TO_IDLE to cancel."));
			else if (_instructionInProgress == INST_START_RECEIVING_1_MESSAGE)
				port.println(F("MCU is receiving one message. Use INST_GO_TO_IDLE to cancel."));
			else if (_instructionInProgress == INST_START_RECEIVING_2_MESSAGES)
				port.println(F("MCU is receiving two messages. Use INST_GO_TO_IDLE to cancel."));
			else if (_instructionInProgress == INST_START_RECEPTION_FOR_FIXED_TIME)
				port.println(F("MCU is receiving for a fixed time. Use INST_GO_TO_IDLE to cancel."));
			else if (_instructionInProgress == INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE)
				port.println(F("MCU is transmitting one packet. It will go idle when transmission is complete."));
			else if (_instructionInProgress == INST_TRANSMIT_ONE_PACKAGE_AND_START_RX)
				port.println(F("MCU is transmitting one packet. It will start reception for a fixed time when transmission is complete."));
			else if (_instructionInProgress == INST_GO_TO_IDLE)
				port.println(F("MCU is attempting to go to idle."));
			else if (_instructionInProgress == INST_SATELLITE_DETECTION)
				// TO DO: check if it is possible to GO_TO_IDLE during satellite detection
				port.println(F("MCU is attempting to detect a satellite for the specified amount of time."));
			else
				port.println(F("I don't know what the MCU is doing. This should be impossible!"));
			break;
		default:
			port.println(F("MCU command result: UNKNOWN RESULT! This should be impossible!")); // This should be impossible?
			break;
		}
}

// Send an MCU Instruction
// Write a single 8-bit instruction
// Return the result
ARTIC_R2_MCU_Command_Result ARTIC_R2::sendMCUinstruction(uint8_t instruction)
{
	// Check that the instruction is valid
	if((instruction != INST_START_CONTINUOUS_RECEPTION) &&
		(instruction != INST_START_RECEIVING_1_MESSAGE) &&
		(instruction != INST_START_RECEIVING_2_MESSAGES) &&
		(instruction != INST_START_RECEPTION_FOR_FIXED_TIME) &&
		(instruction != INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) &&
		(instruction != INST_TRANSMIT_ONE_PACKAGE_AND_START_RX) &&
		(instruction != INST_GO_TO_IDLE) &&
		(instruction != INST_SATELLITE_DETECTION))
		return ARTIC_R2_MCU_COMMAND_INVALID; // Instruction is invalid

	// Check if this is not a INST_GO_TO_IDLE instruction
	if (instruction != INST_GO_TO_IDLE)
	{
		// Check that there is not already an instruction in progress
		if (_instructionInProgress > 0)
			return ARTIC_R2_MCU_INSTRUCTION_IN_PROGRESS; // Abort - an instruction is already in progress
	}

	ARTIC_R2_Firmware_Status status; // Read the status register before attempting to send the command

	if (_printDebug == true)
	{
		readStatusRegister(&status);
		if (!status.STATUS_REGISTER_BITS.IDLE) // If the firmware is not idle
		{
			_debugPort->println(F("sendMCUinstruction: ARTIC is not idle. This will probably fail."));
		}
	}

	// Write 8 bits to SPI:
	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(instruction);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles *** TO DO: Check if this needs to be extended! ***

	readStatusRegister(&status); // Read the status register again

	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED)
	{
		_instructionInProgress = instruction; // Update _instructionInProgress
		return ARTIC_R2_MCU_COMMAND_ACCEPTED;
	}
	else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED)
	{
		// Leave _instructionInProgress unmodified?
		return ARTIC_R2_MCU_COMMAND_REJECTED;
	}
	else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW)
	{
		// Leave _instructionInProgress unmodified?
		return ARTIC_R2_MCU_COMMAND_OVERFLOW;
	}
	else
	{
		// Leave _instructionInProgress unmodified?
		return ARTIC_R2_MCU_COMMAND_UNCERTAIN; // Hopefully this is impossible?
	}
}

// Check the MCU instruction progress. Return true if the instruction is complete.
boolean ARTIC_R2::checkMCUinstructionProgress(ARTIC_R2_MCU_Instruction_Progress *progress)
{
	ARTIC_R2_Firmware_Status status; // Read the status register before attempting to send the command
	readStatusRegister(&status);

	switch (_instructionInProgress)
	{
		case 0: // If there is no instruction in progress
			*progress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS;
			return true; // Return true as instruction is complete - even though none was in progress?
			break;
		case INST_START_CONTINUOUS_RECEPTION:
			if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) // RX_BUFFER_OVERFLOW takes priority over RX_VALID_MESSAGE
			{
				*progress = ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_BUFFER_OVERFLOW;
			}
			else if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE)
			{
				*progress = ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_VALID_MESSAGE;
			}
			else if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED)
			{
				*progress = ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_SATELLITE_DETECTED;
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION;
			}
			return false; // Continuous reception is never complete. It has to be stopped by GO_TO_IDLE.
			break;
		case INST_START_RECEIVING_1_MESSAGE:
			if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) // IDLE will also be set
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_VALID_MESSAGE;
				return true; // Rx is complete
			}
			else if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED)
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_SATELLITE_DETECTED;
				return false; // Rx is still in progress
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE;
				return false; // Rx is still in progress
			}
			break;
		case INST_START_RECEIVING_2_MESSAGES:
			if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) // If one or two messages have been received
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES_RX_VALID_MESSAGE;
				if (status.STATUS_REGISTER_BITS.IDLE) // If MCU is idle then two messages have been received
				{
					return true; // Rx is complete - both messages received
				}
				else
				{
					return false; // Rx is still in progress - only one message received
				}
			}
			else if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED)
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES_RX_SATELLITE_DETECTED;
				return false; // Rx is still in progress
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES;
				return false; // Rx is still in progress
			}
			break;
		case INST_START_RECEPTION_FOR_FIXED_TIME:
			if (status.STATUS_REGISTER_BITS.RX_TIMEOUT) // If Rx has timed out
			{
				if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) // If the buffer has overflowed
				{
					*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT_WITH_BUFFER_OVERFLOW;
					return true; // Rx is complete. It timed out.
				}
				else if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) // If a message was received
				{
					*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT_WITH_VALID_MESSAGE;
					return true; // Rx is complete. It timed out.
				}
				else // Rx timed out. IDLE_STATE should also be set.
				{
					*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT;
					return true; // Rx is complete. It timed out.
				}
			}
			// Rx has not yet timed out
			if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) // RX_BUFFER_OVERFLOW takes priority over RX_VALID_MESSAGE
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_BUFFER_OVERFLOW;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) // If a message was received
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_VALID_MESSAGE;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // If a satellite has been detected
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_SATELLITE_DETECTED;
				return false; // Rx is not yet complete.
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME;
				return false; // Rx is not yet complete.
			}
			break;
		case INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE:
			if (status.STATUS_REGISTER_BITS.TX_INVALID_MESSAGE) // If the message was invalid
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_TX_INVALID_MESSAGE;
				return true; // We're done. We can't send an invalid message...
			}
			else if (status.STATUS_REGISTER_BITS.TX_FINISHED) // If Tx is complete
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_TX_FINISHED;
				return true; // We're done.
			}
			else if (status.STATUS_REGISTER_BITS.IDLE_STATE) // If Tx has gone idle
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_IDLE_STATE;
				return true; // We're done.
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE;
				return false; // We're not done.
			}
			break;
		case INST_TRANSMIT_ONE_PACKAGE_AND_START_RX:
			if (status.STATUS_REGISTER_BITS.RX_TIMEOUT) // If Rx has timed out
			{
				if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) // If the buffer has overflowed
				{
					*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT_WITH_BUFFER_OVERFLOW;
					return true; // Rx is complete. It timed out.
				}
				else if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) // If a message was received
				{
					*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT_WITH_VALID_MESSAGE;
					return true; // Rx is complete. It timed out.
				}
				else // Rx timed out. IDLE_STATE should also be set.
				{
					*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT;
					return true; // Rx is complete. It timed out.
				}
			}
			// Rx has not yet timed out
			if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) // RX_BUFFER_OVERFLOW takes priority over RX_VALID_MESSAGE
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_BUFFER_OVERFLOW;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) // If a message was received
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_VALID_MESSAGE;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // If a satellite has been detected
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_SATELLITE_DETECTED;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // If a satellite has been detected
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_SATELLITE_DETECTED;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.TX_INVALID_MESSAGE) // If the message was invalid
			{
				// Let's assume that the ARTIC still goes into Rx if the message was invalid. TO DO: check this!
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_TX_INVALID_MESSAGE;
				return false; // Rx is not yet complete.
			}
			else if (status.STATUS_REGISTER_BITS.TX_FINISHED) // If the message was transmitted
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_TX_FINISHED;
				return false; // Rx is not yet complete.
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX;
				return false; // Not yet complete.
			}
			break;
		case INST_GO_TO_IDLE:
			if (status.STATUS_REGISTER_BITS.IDLE_STATE)
			{
				*progress = ARTIC_R2_MCU_PROGRESS_GO_TO_IDLE_IDLE_STATE;
				return true; // We're done
			}
			else
			{
				*progress = ARTIC_R2_MCU_PROGRESS_GO_TO_IDLE;
				return false; // We're not done
			}
			break;
		case INST_SATELLITE_DETECTION:
			if (status.STATUS_REGISTER_BITS.SATELLITE_TIMEOUT) // Has detection timed out?
			{
				if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // Did we detect a satellite
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT_WITH_SATELLITE_DETECTED;
					return true; // We're done. Rx timed out.
				}
				else
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT;
					return true; // We're done. Rx timed out.
				}
			}
			else
			{
				if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // Have we detected a satellite
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_RX_SATELLITE_DETECTED;
					return false; // We may not be done? Maybe Rx continues until it times out? TO DO: check this!
				}
				else
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION;
					return false; // We're not done.
				}
			}
			break;
		default:
			*progress = ARTIC_R2_MCU_PROGRESS_UNKNOWN_INSTRUCTION;
			return true; // Let's assume we're done?
			break;
	}
	return false; // Make the compile warning go away...
}

// Pretty-print the MCU instruction progress
void ARTIC_R2::printInstructionProgress(ARTIC_R2_MCU_Instruction_Progress progress, Stream &port)
{
	switch (progress)
	{
	case ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS:
		port.println(F("MCU Instruction Progress: No Instruction In Progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION:
		port.println(F("MCU Instruction Progress: Continuous Reception in progress."));
  	break;
	case ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_SATELLITE_DETECTED:
		port.println(F("MCU Instruction Progress: Continuous Reception in progress. A satellite has been detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_VALID_MESSAGE:
		port.println(F("MCU Instruction Progress: Continuous Reception in progress. A valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_BUFFER_OVERFLOW:
  	port.println(F("MCU Instruction Progress: Continuous Reception in progress. The RX buffer has overflowed!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE:
  	port.println(F("MCU Instruction Progress: Receive One Message in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_SATELLITE_DETECTED:
  	port.println(F("MCU Instruction Progress: Receive One Message in progress. A satellite has been detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_VALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Receive One Message is complete. A valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES:
  	port.println(F("MCU Instruction Progress: Receive Two Messages in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES_RX_SATELLITE_DETECTED:
  	port.println(F("MCU Instruction Progress: Receive Two Messages in progress. A satellite has been detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_TWO_MESSAGES_RX_VALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Receive Two Messages in progress. At least one valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_SATELLITE_DETECTED:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time in progress. A satellite has been detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_VALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time in progress. At least one valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_BUFFER_OVERFLOW:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time in progress. The RX buffer has overflowed!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time is complete. Reception timed out - no messages received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT_WITH_VALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time is complete. At least one valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_RECEIVE_FIXED_TIME_RX_TIMEOUT_WITH_BUFFER_OVERFLOW:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time is complete. The RX buffer has overflowed!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE:
  	port.println(F("MCU Instruction Progress: Reception For Fixed Time in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_IDLE_STATE:
  	port.println(F("MCU Instruction Progress: Transmit One Package And Go Idle in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_TX_FINISHED:
  	port.println(F("MCU Instruction Progress: Transmit One Package And Go Idle is complete. Message was transmitted."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_TX_INVALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Transmit One Package And Go Idle is complete. Message was invalid!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_TX_FINISHED:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time in progress. Message transmission is complete. Reception continues."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_TX_INVALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time in progress. Message was invalid! Reception continues."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_SATELLITE_DETECTED:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time in progress. Message transmission should be complete. A satellite has been detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_VALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time in progress. Message transmission is complete. At least one valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_BUFFER_OVERFLOW:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time in progress. Message transmission is complete. The RX buffer has overflowed!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time is complete. Message transmission is complete. Reception timed out - no messages received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT_WITH_VALID_MESSAGE:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time is complete. Message transmission is complete. At least one valid message has been received."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_FIXED_RX_RX_TIMEOUT_WITH_BUFFER_OVERFLOW:
  	port.println(F("MCU Instruction Progress: Transmit One Package With Reception For Fixed Time is complete. Message transmission is complete. The RX buffer has overflowed!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_GO_TO_IDLE:
  	port.println(F("MCU Instruction Progress: Go To Idle in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_GO_TO_IDLE_IDLE_STATE:
  	port.println(F("MCU Instruction Progress: Go To Idle is complete."));
		break;
	case ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION:
  	port.println(F("MCU Instruction Progress: Satellite Detection in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_RX_SATELLITE_DETECTED:
  	port.println(F("MCU Instruction Progress: Satellite Detection in progress. A satellite has been detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT:
  	port.println(F("MCU Instruction Progress: Satellite Detection is complete. Reception timed out."));
		break;
	case ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT_WITH_SATELLITE_DETECTED:
  	port.println(F("MCU Instruction Progress: Satellite Detection is complete. Reception timed out. A satellite was detected."));
		break;
	case ARTIC_R2_MCU_PROGRESS_UNKNOWN_INSTRUCTION:
  	port.println(F("MCU Instruction Progress: UNKNOWN INSTRUCTION!"));
		break;
	default:
		port.println(F("MCU Instruction Progress: UNKNOWN PROGRESS STATE! This should be impossible!"));
		break;
	}
}

// Read the firmware version from PMEM
void ARTIC_R2::readFirmwareVersion(uint8_t *buffer)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_FIRMWARE_VERSION;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_P_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(buffer, 32, 2); // Read 2 * 32-bit words

	buffer[8] = 0; // Null-terminate the string

	if (_printDebug == true)
	{
		_debugPort->print(F("Firmware version: "));
		_debugPort->println(*buffer);
	}
}

// Read the memories CRCs (after firmware boot)
void ARTIC_R2::readMemoryCRC(uint32_t *PMEM_CRC, uint32_t *XMEM_CRC, uint32_t *YMEM_CRC)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_CRC_RESULTS;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[12]; // Buffer for the CRC words
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 3); // Read 3 * 24-bit words

	// Extract the checksums
	*PMEM_CRC = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);
	*XMEM_CRC = (((uint32_t)buffer[3]) << 16) | (((uint32_t)buffer[4]) << 8) | ((uint32_t)buffer[5]);
	*YMEM_CRC = (((uint32_t)buffer[6]) << 16) | (((uint32_t)buffer[7]) << 8) | ((uint32_t)buffer[8]);

	if (_printDebug == true)
	{
		_debugPort->print(F("Firmware checksum PMEM: 0x"));
		_debugPort->print(*PMEM_CRC, HEX);
		_debugPort->print(F("  XMEM: 0x"));
		_debugPort->print(*XMEM_CRC, HEX);
		_debugPort->print(F("  YMEM: 0x"));
		_debugPort->println(*YMEM_CRC, HEX);
	}
}

// Set the RX timeout (seconds)
boolean ARTIC_R2::setRxTimeout(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readRxTimeout() == timeout_secs);
}

// Read the RX timeout (seconds)
uint32_t ARTIC_R2::readRxTimeout()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the rx timeout
}

// Set the satellite detection timeout (seconds)
boolean ARTIC_R2::setSatelliteDetectionTimeout(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_SATELLITE_DETECTION_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readSatelliteDetectionTimeout() == timeout_secs);
}

// Read the satellite detection timeout
uint32_t ARTIC_R2::readSatelliteDetectionTimeout()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_SATELLITE_DETECTION_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the satellite detection timeout
}

// Set the TCXO warm up time (seconds)
boolean ARTIC_R2::setTCXOWarmupTime(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_WARMUP_TIME;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readTCXOWarmupTime() == timeout_secs);
}

// Read the TCXO warm up time
uint32_t ARTIC_R2::readTCXOWarmupTime()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_WARMUP_TIME;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the TCXO warm up time
}

// Set the TX certification interval
boolean ARTIC_R2::setTxCertificationInterval(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_CERTIFICATION_INTERVAL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readTxCertificationInterval() == timeout_secs);
}

// Read the TX certification interval
uint32_t ARTIC_R2::readTxCertificationInterval()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_CERTIFICATION_INTERVAL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the TX certification interval
}

// Set the TCXO control voltage and auto-disable
boolean ARTIC_R2::setTCXOControl(float voltage, bool autoDisable)
{
	if ((voltage < 1.3) || ((voltage > 2.7) && (voltage < 3.3)) || (voltage > 3.3))
	{
		if (_printDebug == true)
		{
			_debugPort->print(F("setTCXOControl: Invalid control voltage: "));
			_debugPort->println(voltage);
		}
		return (false);
	}

	TCXO_Control_Register tcxo_control;
	tcxo_control.TCXO_CONTROL_REGISTER = 0x000000; // Clear the control register

	if (voltage == 3.3) // Check for 3.3V
	{
		tcxo_control.CONTROL_REGISTER_BITS.SELECT_3V3 = 1;
	}
	else if (voltage == 1.8) // Check for 1.8V
	{
		tcxo_control.CONTROL_REGISTER_BITS.SELECT_1V8 = 1;
	}
	else // Calculate the nibble for 1.3V to 2.7V
	{
		uint32_t voltage_nibble = (voltage - 1.3) / 0.1;
		tcxo_control.CONTROL_REGISTER_BITS.SELECT_1V3_TO_2V7 = voltage_nibble & 0x0F;
	}

	// The ARTIC can be programmed to keep the TCXO on after a TX or RX command.
	// Auto disable: Set if TCXO shall remain active after an instruction command
	if (autoDisable)
		tcxo_control.CONTROL_REGISTER_BITS.AUTO_DISABLE = 1;
	else
		tcxo_control.CONTROL_REGISTER_BITS.AUTO_DISABLE = 0;

	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_CONTROL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(tcxo_control.TCXO_CONTROL_REGISTER); // Set the TCXO control

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return ((readTCXOControlVoltage() == voltage) && (readTCXOAutoDisable() == autoDisable));
}

// Read the TCXO control voltage. Auto-disable is ignored.
float ARTIC_R2::readTCXOControlVoltage()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_CONTROL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	TCXO_Control_Register tcxo_control;
	tcxo_control.TCXO_CONTROL_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status

	if (tcxo_control.CONTROL_REGISTER_BITS.SELECT_3V3 == 1) // Check for 3.3V
		return (3.3);
	else if (tcxo_control.CONTROL_REGISTER_BITS.SELECT_1V8 = 1) // Check for 1.8V
		return (1.8);
	else
	{
		float voltage = tcxo_control.CONTROL_REGISTER_BITS.SELECT_1V3_TO_2V7;
		voltage = (voltage * 0.1) + 1.3;
		return (voltage);
	}
}

// Read the TCXO auto-disable bit
boolean ARTIC_R2::readTCXOAutoDisable()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_CONTROL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	TCXO_Control_Register tcxo_control;
	tcxo_control.TCXO_CONTROL_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status

	if (tcxo_control.CONTROL_REGISTER_BITS.AUTO_DISABLE == 1)
		return (true);
	else
		return (false);
}

// Enable the RX CRC check by writing 0x000001 to MEM_LOC_RX_FILTERING_ENABLE_CRC
boolean ARTIC_R2::enableRXCRC()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(0x000001); // Enable CRC

	// Check that the CRC is enabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	if (buffer[2] == 0x01)
		return (true);
	else
		return (false);
}

// Disable the RX CRC check by writing 0x000000 to MEM_LOC_RX_FILTERING_ENABLE_CRC
boolean ARTIC_R2::disableRXCRC()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(0x000000); // Disable CRC

	// Check that the CRC is disabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	if (buffer[2] == 0x00)
		return (true);
	else
		return (false);
}

// Enable the RX transparent mode by writing 0x000001 to MEM_LOC_RX_FILTERING_TRANSPARENT_MODE
// All messages are send to the MCU.
boolean ARTIC_R2::enableRXTransparentMode()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(0x000001); // Enable transparent mode

	// Check that transparent mode is enabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	if (buffer[2] == 0x01)
		return (true);
	else
		return (false);
}

// Disable the RX transparent mode by writing 0x000000 to MEM_LOC_RX_FILTERING_TRANSPARENT_MODE
// Only messages with an ID mentioned in the Address LUT are sent to the MCU.
boolean ARTIC_R2::disableRXTransparentMode()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(0x000000); // Disable transparent mode

	// Check that transparent mode is disabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	if (buffer[2] == 0x00)
		return (true);
	else
		return (false);
}

// Clear the address LUT by writing 0x000000 to MEM_LOC_RX_FILTERING_LUT_LENGTH
boolean ARTIC_R2::clearAddressLUT()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(0x000000); // Clear the LUT

	// Check that the LUT is clear by reading the length back again

	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	if (buffer[2] == 0x00)
		return (true);
	else
		return (false);
}

// Add a new address to the message filtering LUT
// AddressLSBits and AddressMSBits are 24-bit (not 32)
boolean ARTIC_R2::addAddressToLUT(uint32_t AddressLSBits, uint32_t AddressMSBits)
{
	if (AddressLSBits >= 0x01000000) // Check for an invalid address (> 24 bits)
	{
		if (_printDebug == true)
			_debugPort->println(F("addAddressToLUT: AddressLSBits is invalid!"));
		return (false);
	}
	if (AddressMSBits >= 0x01000000) // Check for an invalid address (> 24 bits)
	{
		if (_printDebug == true)
			_debugPort->println(F("addAddressToLUT: AddressMSBits is invalid!"));
		return (false);
	}

	// Read the LUT Length
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read the LUT length

	uint32_t tableLength = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	if (tableLength > 49)
	{
		if (_printDebug == true)
			_debugPort->println(F("addAddressToLUT: address table is full!"));
		return (false);
	}

	// Write the new address to the LUT
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_FIRST_ADDRESS + (tableLength << 1); // Calculate where to store the address
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	writeTwo24BitWords(AddressLSBits, AddressMSBits); // Write the address words

	// Now increment the table length

	tableLength++; // Increment the table length by one

	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(tableLength); // Write the incremented length

	// Finally, read the LUT Length and check it was incremented correctly
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read the LUT length

	uint32_t newTableLength = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	if (tableLength == newTableLength)
		return (true);
	else
	{
		if (_printDebug == true)
			_debugPort->println(F("addAddressToLUT: table length was not incremented correctly!"));
		return (false);
	}
}

// Read a downlink message from the RX payload buffer
//
// 3.6.2 Reception signalling and buffering
//
// The ARTIC can buffer 2 messages internally before the third message arrives, then this third message will be discarded.
// On this event INT 2 will be set with the RX_BUFFER_OVERFLOW flag.
//
// When the ARTIC receives a valid downlink message, it raises INT 1 with the RX_VALID_MESSAGE flag.
// The MCU can then use an SPI burst access to read the downlink message from the X data memory.
//
// After reading the downlink message, the ARTIC reception buffer must be cleared.
// This is done by sending the ‘Clear interrupt 1’ command.
//
// If a second message was received it will be moved to the ‘RX payload’ buffer upon the ‘Clear interrupt 1’ command.
// In this case INT 1 will be re-raised after 100usec.
// The MCU can read the RX payload again and use the ‘Clear interrupt 1’ command.
//
// 3.6.3 Read downlink messages
// A part of the X data memory is reserved for a downlink message.
// This part is referred to as RX payload.
// The RX payload buffer occupies 9 x 24-bit words and starts at the address location described in 0
//
// The RX payload buffer contains 2 items:
// o Downlink payload length:
//   o Defines the length of the downlink message in bits.
//   o The downlink message length occupies the first 24-bit word of the payload buffer.
// o Downlink message
//   o Downlink message as defined by ARGOS standard A4-SYS-IF-0086-CNES, see Table 17.
//   o Starts at the second 24-bit word of the payload buffer.
//   o The first bit of the Downlink message is aligned with the MSB of the second 24-bit word of the payload buffer.
//     Bit 25 of the Downlink message is aligned with the MSB of the third 24-bit word of the payload buffer.
//     In this way the total Downlink message occupies a number of consecutive 24-bit words in the payload buffer.
//   o In case the total length of the Downlink message is not a multiple of 24 bits,
//     the last used 24-bit word of the payload buffer is stuffed by the DSP with 0’s at the LSB locations.
boolean ARTIC_R2::readDownlinkMessage(Downlink_Message *downlinkMessage)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[9*3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 9); // Read 9 24-bit words

	// Pretty-print the received payload as 24-bit values
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: payload buffer contents:"));
		for (int i = 0; i < 27; i++)
		{
			if ((i % 3) == 0) _debugPort->print(" 0x");
			if (buffer[i] < 0x10) _debugPort->print("0");
			_debugPort->print(buffer[i], HEX);
		}
		_debugPort->println();
	}

	// Assemble the payload length
	downlinkMessage->payloadLength = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: payloadLength in bit is "));
		_debugPort->println(downlinkMessage->payloadLength);
	}

	// Trap zero payload length (hopefully redundant!?)
	if (downlinkMessage->payloadLength == 0)
	{
		if (_printDebug == true)
			_debugPort->println(F("readDownlinkMessage: zero payloadLength!"));
		return (false);
	}

	// Assemble the Addressee Identification (28 bits)
	downlinkMessage->addresseeIdentification = (((uint32_t)buffer[3]) << 20) | (((uint32_t)buffer[4]) << 12) | (((uint32_t)buffer[5]) << 4) | (((uint32_t)buffer[6]) >> 4);
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: Addressee Identification is 0x"));
		_debugPort->println(downlinkMessage->addresseeIdentification, HEX);
	}

	// Extract the ADCS
	downlinkMessage->ADCS = buffer[6] & 0x0F;
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: ADCS is 0x"));
		_debugPort->println(downlinkMessage->ADCS, HEX);
	}

	// Extract the Service
	downlinkMessage->service = buffer[7];
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: Service is 0x"));
		_debugPort->println(downlinkMessage->service, HEX);
	}

	// Calculate the number of whole bytes in the message
	// TO DO: Check if the payload length does include the 7 bytes for the Addressee ID, ADCS, Service and FCS
	int numBytes = ((downlinkMessage->payloadLength) >> 4) - 7; // Divide by 8 and subtract 7 (for the Addressee ID, ADCS, Service and FCS)

	// Copy the full bytes into rxData
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: left-justified payload data is 0x"));
	}
	for (int i = 0; i < numBytes; i++)
	{
		downlinkMessage->payload[i] = buffer[i + 8];
		if (_printDebug == true)
		{
			if (downlinkMessage->payload[i] < 0x10) _debugPort->print("0");
			_debugPort->print(downlinkMessage->payload[i], HEX);
		}
	}

	// Add a partial payload byte (if there is one)

	// Calculate the number of extra bits
	int numExtraBits = ((int)downlinkMessage->payloadLength) - ((((int)numBytes) + 7) * 8);
	// Extract the extra bits. Left-justify them.
	if (numExtraBits > 0)
	{
		// Left-justify the extra bits
		downlinkMessage->payload[numBytes] = (buffer[numBytes + 8] >> (8 - numExtraBits)) << (8 - numExtraBits);
		if (_printDebug == true)
		{
			if ((downlinkMessage->payload[numBytes] < 0x10) && (numExtraBits >= 5)) _debugPort->print("0");
			_debugPort->println(downlinkMessage->payload[numBytes], HEX);
		}
		// Extract the FCS bits
		downlinkMessage->FCS = ((uint16_t)(buffer[numBytes + 8] << numExtraBits)) << 8;
		downlinkMessage->FCS = downlinkMessage->FCS | (((uint16_t)buffer[numBytes + 9]) << numExtraBits);
		downlinkMessage->FCS = downlinkMessage->FCS | (((uint16_t)buffer[numBytes + 10]) >> (8 - numExtraBits));
	}
	else
	{
		// There are no extra bits so FCS is byte-aligned
		downlinkMessage->FCS = (((uint16_t)buffer[numBytes + 9]) << 8) | ((uint16_t)buffer[numBytes + 10]);
		if (_printDebug == true)
		{
			_debugPort->println(); // Tidy up debug printing
		}
	}
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: FCS is 0x"));
		_debugPort->println(downlinkMessage->FCS, HEX);
	}

	return (true);
}

// Write a single 24-bit word to the ARTIC R2
void ARTIC_R2::write24BitWord(uint32_t word)
{
	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(word >> 16);
	_spiPort->transfer(word >> 8);
	_spiPort->transfer(word & 0xFF);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();
}

// Write two 24-bit words to the ARTIC R2
void ARTIC_R2::writeTwo24BitWords(uint32_t word1, uint32_t word2)
{
	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(word1 >> 16);
	_spiPort->transfer(word1 >> 8);
	_spiPort->transfer(word1 & 0xFF);

	_spiPort->transfer(word2 >> 16);
	_spiPort->transfer(word2 >> 8);
	_spiPort->transfer(word2 & 0xFF);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();
}

// Read multiple words from the ARTIC R2
// Store them in buffer
// It is the user's responsibility to make sure buffer is large enough to receive all of the data
void ARTIC_R2::readMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords)
{
	int bufferPointer = 0;

	if ((wordSizeInBits != 24) && (wordSizeInBits != 32))
		return;
	//if ((numWords <= 0) || (numWords > ARTIC_R2_MAX_SPI_TRANSFER))
	if (numWords <= 0)
		return;

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (int word = 0; word < numWords; word++)
	{
		for (int byteSize = 0; byteSize < (wordSizeInBits >> 4); byteSize++)
		{
			buffer[bufferPointer++] = _spiPort->transfer(0xFF);
		}
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();
}

// Clear one or both interrupts
boolean ARTIC_R2::clearInterrupts(uint8_t interrupts)
{
	if ((interrupts < 1) || (interrupts > 3))
		return (false);

	if ((interrupts == 1) || (interrupts == 3)) // Clear interrupt 1
	{
		sendMCUinstruction(CMD_CLEAR_INT_1);
	}

	delay(10);

	if ((interrupts == 2) || (interrupts == 3)) // Clear interrupt 2
	{
		sendMCUinstruction(CMD_CLEAR_INT_2);
	}

	delay(10);

	// Check that the interrupt flags have been cleared (hopefully redundant?)
	ARTIC_R2_Firmware_Status status;
	readStatusRegister(&status);

	if ((interrupts == 1) || (interrupts == 3)) // Check if interrupt 1 is clear
	{
		if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // If the interrupt bit is still set
			return (false); // Flag is still set so clearing failed...
	}

	if ((interrupts == 2) || (interrupts == 3)) // Check if interrupt 2 is clear
	{
		if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // If the interrupt bit is still set
			return (false); // Flag is still set so clearing failed...
	}

	return (true); // Success!
}

/*

void ARTIC_R2::construct_PPT_A3_header(uint8_t *buffer, uint32_t *index, uint32_t total_len_bits, uint8_t len_bitmask)
{
    buffer[0] = total_len_bits >> 16;
    buffer[1] = total_len_bits >> 8;
    buffer[2] = total_len_bits;

    buffer[3] = len_bitmask | (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK) >> 24;
    buffer[4] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK) >> 16;
    buffer[5] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK) >> 8;
    buffer[6] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK);

    *index += 7;
}

void ARTIC_R2::construct_ZTE_header(uint8_t *buffer, uint32_t total_len_bits)
{
    buffer[0] = total_len_bits >> 16;
    buffer[1] = total_len_bits >> 8;
    buffer[2] = total_len_bits;

    buffer[3] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK) >> 24;
    buffer[4] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK) >> 16;
    buffer[5] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK) >> 8;
    buffer[6] = (config.artic->contents.device_identifier & ARTIC_MSG_ID_BITMASK);
}

void ARTIC_R2::construct_artic_message_buffer(const uint8_t *buffer, size_t buffer_size, uint8_t *send_buffer, uint32_t *bytes_to_send)
{
    uint32_t index = 0;
    *bytes_to_send = 0;

    memset(send_buffer, 0, ARTIC_MSG_MAX_SIZE);

    if (buffer_size <= ARTIC_ZTE_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_ZTE_BYTES_TO_SEND;
        construct_ZTE_header(send_buffer, ARTIC_ZTE_MSG_TOTAL_BITS);
    }
    else if (buffer_size <= ARTIC_PTT_A3_24_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_24_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_24_MSG_TOTAL_BITS, ARTIC_PTT_A3_24_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_56_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_56_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_56_MSG_TOTAL_BITS, ARTIC_PTT_A3_56_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_88_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_88_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_88_MSG_TOTAL_BITS, ARTIC_PTT_A3_88_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_120_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_120_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_120_MSG_TOTAL_BITS, ARTIC_PTT_A3_120_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_152_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_152_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_152_MSG_TOTAL_BITS, ARTIC_PTT_A3_152_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_184_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_184_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_184_MSG_TOTAL_BITS, ARTIC_PTT_A3_184_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_216_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_216_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_216_MSG_TOTAL_BITS, ARTIC_PTT_A3_216_MSG_LEN_FIELD);
    }
    else if (buffer_size <= ARTIC_PTT_A3_248_MAX_USER_BYTES)
    {
        *bytes_to_send = ARTIC_PTT_A3_248_BYTES_TO_SEND;
        construct_PPT_A3_header(send_buffer, &index, ARTIC_PTT_A3_248_MSG_TOTAL_BITS, ARTIC_PTT_A3_248_MSG_LEN_FIELD);
    }

    // Copy the data payload into the message
    memcpy(&send_buffer[index], buffer, buffer_size);
}

*/

/*! \brief Function to send a message using ARGOS A3 standard
 *
 * \param buffer[in] Array with the data to send.
 * \param  buffer_size[in] total bytes of data to send.
 *
 * \return \ref SYSHAL_SAT_NO_ERROR on success.
 * \return \ref SYSHAL_SAT_ERROR_NOT_PROGRAMMED ARTIC device has to be programmed everytime before send data.
 * \return \ref SYSHAL_SAT_ERROR_MAX_SIZE Maximum size of user data is 31 bytes.
 * \return ref SYSHAL_SAT...
 */
/*
int ARTIC_R2::syshal_sat_send_message(const uint8_t *buffer, size_t buffer_size)
{
    uint8_t send_buffer[ARTIC_MSG_MAX_SIZE] = {0};
    uint32_t bytes_to_send;
    uint8_t artic_packet_type;
    int ret;

    if (sat_state != STATE_PROGRAMMED)
        return SYSHAL_SAT_ERROR_NOT_PROGRAMMED;

    if (buffer_size > MAX_TX_SIZE_BYTES)
        return SYSHAL_SAT_ERROR_MAX_SIZE;

    if (buffer_size == 0)
        artic_packet_type = ARTIC_CMD_SET_PTT_ZE_TX_MODE;
    else
        artic_packet_type = ARTIC_CMD_SET_PTT_A3_TX_MODE;

    // Set ARGOS TX MODE in ARTIC device and wait for the status response
    ret = send_command_check_clean(artic_packet_type, 1, MCU_COMMAND_ACCEPTED, true, SYSHAL_SAT_ARTIC_DELAY_INTERRUPT);
    if (ret)
        return ret;

    construct_artic_message_buffer(buffer, buffer_size, send_buffer, &bytes_to_send);

    for (size_t i = 0; i < bytes_to_send; ++i)
    {
        DEBUG_PR_TRACE("Byte[%u]: %02X", (unsigned int) i, (unsigned int) send_buffer[i]);
    }

    // It could be a problem if we set less data than we are already sending, just be careful and in case change TOTAL
    // Burst transfer the tx payload
    ret = burst_access(XMEM, TX_PAYLOAD_ADDRESS, send_buffer, NULL, bytes_to_send, false);
    if (ret)
        return SYSHAL_SAT_ERROR_SPI;

#ifndef DEBUG_DISABLED
    print_status();
#endif

    // Send to ARTIC the command for sending only one packet and wait for the response TX_FINISHED
    ret = send_command_check_clean(ARTIC_CMD_START_TX_1M_SLEEP, 1, TX_FINISHED, true, SYSHAL_SAT_ARTIC_TIMEOUT_SEND_TX);

    if (ret)
    {
        // If there is a problem wait until interrupt 2 is launched and get the status response
        if (!wait_interrupt(SYSHAL_SAT_ARTIC_DELAY_INTERRUPT, INTERRUPT_2))
        {
#ifndef DEBUG_DISABLED
            print_status();
#endif
            clear_interrupt(INTERRUPT_2);
            return SYSHAL_SAT_ERROR_TX;
        }
        else
        {
            return ret;
        }
    }

    return SYSHAL_SAT_NO_ERROR;
}
*/
