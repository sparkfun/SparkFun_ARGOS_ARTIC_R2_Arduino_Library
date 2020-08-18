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

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h"

boolean ARTIC_R2::begin(uint8_t user_CSPin, uint8_t user_RSTPin, uint8_t user_BOOTPin, uint8_t user_PWRENPin, uint8_t user_INT1Pin, uint8_t user_INT2Pin, uint8_t user_GAIN8Pin, uint8_t user_GAIN16Pin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	//Get user settings
	_spiPort = &spiPort;
	_spiPortSpeed = spiPortSpeed;
	if (_spiPortSpeed > 25000000)
		_spiPortSpeed = 25000000; //Datasheet indicates max speed is 25MHz

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
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_PROGRAM_MEMORY;
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
			_debugPort->println(F("Waiting for the ARTIC to boot..."));
		delay(100);
	}

	if ((millis() - bootStartTime) >= ARTIC_R2_BOOT_TIMEOUT)
		return (false); // Boot timed out!

	return (clearInterrupts(3)); // Clear both interrupts

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

	readMultipleWords(&buffer[0], 24, 1); // Read 1 24-bit word

	status->STATUS_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status
}

// Read ARTIC R2 ARGOS configuration register
void ARTIC_R2::readConfigurationRegister(ARGOS_Configuration_Register *configuration)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_ARGOS_CONFIGURATION;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data

	readMultipleWords(&buffer[0], 24, 1); // Read 1 24-bit word

	configuration->CONFIGURATION_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status
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

// Command access
// Write a single 8-bit command
void ARTIC_R2::sendCommandByte(uint8_t command)
{
	// Write 8 bits to SPI:
	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(command);

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
		sendCommandByte(CMD_CLEAR_INT_1);
	}

	delay(10);

	if ((interrupts == 2) || (interrupts == 3)) // Clear interrupt 2
	{
		sendCommandByte(CMD_CLEAR_INT_2);
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
