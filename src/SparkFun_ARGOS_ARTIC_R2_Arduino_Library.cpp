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

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h"

boolean ARTIC_R2::begin(uint8_t user_CSPin, uint8_t user_RSTPin, uint8_t user_BOOTPin, uint8_t user_PWRENPin, uint8_t user_INT1Pin, uint8_t user_INT2Pin, uint8_t user_GAIN8Pin, uint8_t user_GAIN16Pin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	if (_printDebug == true)
		_debugPort->println(F("begin: ARTIC is starting..."));

	//Get user settings
	_spiPort = &spiPort;
	_spiPortSpeed = spiPortSpeed;
	if (_spiPortSpeed > 5000000)
		_spiPortSpeed = 5000000; //Datasheet indicates max speed is 5MHz for P memory writes

	_delay24cycles = 24000000 / _spiPortSpeed; // Calculate the 24-cycle read delay based on the clock speed
	_delay24cycles++; // Round up by 1

	_instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS; // Clear _instructionInProgress

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
	disableARTICpower(); // Disable the power until we have configured the rest of the IO pins

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

	if ((_gain8 >= 0) && (_gain16 >= 0)) // Set the RF TX gain if both pins are defined
	{
		pinMode(_gain8, OUTPUT);
		pinMode(_gain16, OUTPUT);
		setTXgain(24); // Default to maximum gain
	}

	enableARTICpower(); // Enable power for the ARTIC R2
	delay(100);
	digitalWrite(_rst, HIGH); //Bring the ARTIC out of reset

	if (_printDebug == true)
		_debugPort->println(F("begin: IO pins are configured..."));

#ifdef ARTIC_R2_UPLOAD_FIRMWARE

	// <arribada/horizon>

	// Wait until the device's status register contains 85
	// NOTE: This 85 value is undocumentated but can be seen in the supplied Artic_evalboard.py file
	int retries = ARTIC_R2_BOOT_MAX_RETRIES;
	do
	{
		delay(ARTIC_R2_BOOT_DELAY_MS);

		// Read the DSP control register - check for the magic number
		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
		digitalWrite(_cs, LOW);

		uint8_t buffer[3]; // Buffer for the SPI data

		_spiPort->transfer(ARTIC_R2_DSP_CRTL_REG_READ);
		buffer[0] = _spiPort->transfer(0x00);
		buffer[1] = _spiPort->transfer(0x00);
		buffer[2] = _spiPort->transfer(0x00);

		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
/*
		if (_printDebug == true)
		{
			_debugPort->print(F("begin: Reading DSP Control Register: 0x"));
			if (buffer[0] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(buffer[0], HEX);
			if (buffer[1] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(buffer[1], HEX);
			if (buffer[2] < 0x10) _debugPort->print(F("0"));
			_debugPort->println(buffer[2], HEX);
		}
*/
		uint32_t ctrl_reg = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

		if (ctrl_reg == ARTIC_R2_DSP_CRTL_REG_MAGIC_NUMBER) // Does the control register contain the magic number?
		{
			if (_printDebug == true)
				_debugPort->println(F("begin: DSP Control Register magic number seen!"));
			break;
		}

		retries--;
	}
	while(retries >= 0);

	if (retries < 0) // If we failed to see the magic number
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: DSP Control Register magic number not seen! Aborting..."));
		return (false);
	}

	// </arribada/horizon>

	// Datasheet suggests PMEM then XMEM then YMEM
	// Arribada Horizon uploads XMEM then YMEM then PMEM

	// Upload ARTIC firmware: PMEM
	delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait

	if (_printDebug == true)
		_debugPort->println(F("begin: uploading ARTIC firmware to PMEM..."));

	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start at address 0
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_P_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;
/*
	if (_printDebug == true)
	{
		_debugPort->print(F("begin: burstmode.BURSTMODE_REGISTER is 0x"));
		_debugPort->println(burstmode.BURSTMODE_REGISTER, HEX);
	}
*/
	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (uint32_t addr = 0; addr < ARTIC_R2_PMEM_LEN; addr++)
	{
		uint32_t word = ARTIC_R2_PMEM[addr];
		_spiPort->transfer(word >> 24);
		_spiPort->transfer(word >> 16);
		_spiPort->transfer(word >> 8);
		_spiPort->transfer(word & 0xFF);
/*
		if ((_printDebug == true) && (addr == 0))
		{
			_debugPort->print(F("begin: value being written to PMEM address 0 is 0x"));
			_debugPort->println(word, HEX);
		}

		if ((_printDebug == true) && (addr == 8000))
		{
			_debugPort->print(F("begin: value being written to PMEM address 8000 is 0x"));
			_debugPort->println(word, HEX);
		}
*/
		// Delay between words
		delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

		// Extra delay between blocks
		if (addr % ARTIC_R2_BURST_BLOCK_SIZE == 0)
			delay(ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS);
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait
/*
	// Sanity check: read back the 32-bit words from PMEM addresses 0 and 8000 to see if they were written correctly
	if (_printDebug == true)
	{
		burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start of PMEM
		burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_P_MEMORY;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

		configureBurstmodeRegister(burstmode); // Configure the burstmode register

		delayMicroseconds(_delay24cycles);

		uint8_t pmemBuffer[4]; // Buffer for the PMEM data
		uint8_t *ptr = pmemBuffer; // Pointer to the buffer

		readMultipleWords(ptr, 32, 1); // Read exactly 1 32-bit word

		_debugPort->print(F("begin: PMEM address 0 contains 0x"));
		if (pmemBuffer[0] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(pmemBuffer[0], HEX);
		if (pmemBuffer[1] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(pmemBuffer[1], HEX);
		if (pmemBuffer[2] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(pmemBuffer[2], HEX);
		if (pmemBuffer[3] < 0x10) _debugPort->print(F("0"));
		_debugPort->println(pmemBuffer[3], HEX);

		delay(ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS);

		burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 8000;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_P_MEMORY;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

		configureBurstmodeRegister(burstmode); // Configure the burstmode register

		delayMicroseconds(_delay24cycles);

		readMultipleWords(ptr, 32, 1); // Read exactly 1 32-bit word

		_debugPort->print(F("begin: PMEM address 8000 contains 0x"));
		if (pmemBuffer[0] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(pmemBuffer[0], HEX);
		if (pmemBuffer[1] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(pmemBuffer[1], HEX);
		if (pmemBuffer[2] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(pmemBuffer[2], HEX);
		if (pmemBuffer[3] < 0x10) _debugPort->print(F("0"));
		_debugPort->println(pmemBuffer[3], HEX);

		delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait
	}
*/
	// Upload ARTIC firmware: XMEM
	if (_printDebug == true)
		_debugPort->println(F("begin: uploading ARTIC firmware to XMEM..."));

	burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start at address 0
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;
/*
	if (_printDebug == true)
	{
		_debugPort->print(F("begin: burstmode.BURSTMODE_REGISTER is 0x"));
		_debugPort->println(burstmode.BURSTMODE_REGISTER, HEX);
	}
*/
	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (uint32_t addr = 0; addr < ARTIC_R2_XMEM_LEN; addr++)
	{
		uint32_t word = ARTIC_R2_XMEM[addr];
		_spiPort->transfer(word >> 16);
		_spiPort->transfer(word >> 8);
		_spiPort->transfer(word & 0xFF);
/*
		if ((_printDebug == true) && (addr == 7000))
		{
			_debugPort->print(F("begin: value being written to XMEM address 7000 is 0x"));
			_debugPort->println(word, HEX);
		}

		if ((_printDebug == true) && (addr == 10000))
		{
			_debugPort->print(F("begin: value being written to XMEM address 10000 is 0x"));
			_debugPort->println(word, HEX);
		}
*/
		// Delay between words
		delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

		// Extra delay between blocks
		if (addr % ARTIC_R2_BURST_BLOCK_SIZE == 0)
			delay(ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS);
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait
/*
	// Sanity check: read back the 32-bit words from XMEM addresses 7000 and 10000 to see if they were written correctly
	if (_printDebug == true)
	{
		burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 7000;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

		configureBurstmodeRegister(burstmode); // Configure the burstmode register

		delayMicroseconds(_delay24cycles);

		uint8_t xmemBuffer[3]; // Buffer for the XMEM data
		uint8_t *ptr = xmemBuffer; // Pointer to the buffer

		readMultipleWords(ptr, 24, 1); // Read exactly 1 24-bit word

		_debugPort->print(F("begin: XMEM address 7000 contains 0x"));
		if (xmemBuffer[0] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(xmemBuffer[0], HEX);
		if (xmemBuffer[1] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(xmemBuffer[1], HEX);
		if (xmemBuffer[2] < 0x10) _debugPort->print(F("0"));
		_debugPort->println(xmemBuffer[2], HEX);

		delay(ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS);

		burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 10000;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

		configureBurstmodeRegister(burstmode); // Configure the burstmode register

		delayMicroseconds(_delay24cycles);

		readMultipleWords(ptr, 24, 1); // Read exactly 1 24-bit word

		_debugPort->print(F("begin: XMEM address 10000 contains 0x"));
		if (xmemBuffer[0] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(xmemBuffer[0], HEX);
		if (xmemBuffer[1] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(xmemBuffer[1], HEX);
		if (xmemBuffer[2] < 0x10) _debugPort->print(F("0"));
		_debugPort->println(xmemBuffer[2], HEX);

		delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait
	}
*/
	// Upload ARTIC firmware: YMEM
	if (_printDebug == true)
		_debugPort->println(F("begin: uploading ARTIC firmware to YMEM..."));

	burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 0; // Start at address 0
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_Y_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;
/*
	if (_printDebug == true)
	{
		_debugPort->print(F("begin: burstmode.BURSTMODE_REGISTER is 0x"));
		_debugPort->println(burstmode.BURSTMODE_REGISTER, HEX);
	}
*/
	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	for (uint32_t addr = 0; addr < ARTIC_R2_YMEM_LEN; addr++)
	{
		uint32_t word = ARTIC_R2_YMEM[addr];
		_spiPort->transfer(word >> 16);
		_spiPort->transfer(word >> 8);
		_spiPort->transfer(word & 0xFF);
/*
		if ((_printDebug == true) && (addr == 600))
		{
			_debugPort->print(F("begin: value being written to YMEM address 600 is 0x"));
			_debugPort->println(word, HEX);
		}
*/
		// Delay between words
		delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

		// Extra delay between blocks
		if (addr % ARTIC_R2_BURST_BLOCK_SIZE == 0)
			delay(ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS);
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait
/*
	// Sanity check: read back the 32-bit word from YMEM address 600 to see if it was written correctly
	if (_printDebug == true)
	{
		burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
		burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = 600;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_Y_MEMORY;
		burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

		configureBurstmodeRegister(burstmode); // Configure the burstmode register

		delayMicroseconds(_delay24cycles);

		uint8_t ymemBuffer[3]; // Buffer for the YMEM data
		uint8_t *ptr = ymemBuffer; // Pointer to the buffer

		readMultipleWords(ptr, 24, 1); // Read exactly 1 24-bit word

		_debugPort->print(F("begin: YMEM address 600 contains 0x"));
		if (ymemBuffer[0] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(ymemBuffer[0], HEX);
		if (ymemBuffer[1] < 0x10) _debugPort->print(F("0"));
		_debugPort->print(ymemBuffer[1], HEX);
		if (ymemBuffer[2] < 0x10) _debugPort->print(F("0"));
		_debugPort->println(ymemBuffer[2], HEX);

		delay(ARTIC_R2_BURST_FINISH_DELAY_MS); // Wait
	}
*/
	// Activate the DSP
	if (_printDebug == true)
		_debugPort->println(F("begin: activating the DSP (writing 0 to the DSP control register)..."));

	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(ARTIC_R2_DSP_CRTL_REG_WRITE);
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
			_debugPort->println(F("begin: waiting for the ARTIC to boot (checking if INT1 has gone high)..."));
		delay(100);
	}

	// With the ARTIC006 firmware, it seems to take up to 5 minutes for INT1 to go high.
	// The datasheet suggests INT1 should go high after 250ms!
	// So, I'm not quite sure what is happening... Maybe it is a feature of ARTIC006?

	if ((millis() - bootStartTime) >= ARTIC_R2_BOOT_TIMEOUT)
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: boot timed out! INT1 did not go high! Carrying on regardless..."));
		//return (false); // Boot timed out!
	}
	else
	{
		// Print the firmware status - before we clear the interrupt pins
		if (_printDebug == true)
		{
			ARTIC_R2_Firmware_Status status;
		  readStatusRegister(&status); // Read the ARTIC R2 status register
		  _debugPort->println(F("begin: ARTIC R2 Firmware Status (before clearInterrupts):"));
		  printFirmwareStatus(status, *_debugPort); // Pretty-print the firmware status
		}

		if(clearInterrupts(3) == false) // Clear both interrupts
		{
			if (_printDebug == true)
				_debugPort->println(F("begin: failed to clear interrupts! Carrying on regardless..."));
			//return (false);
		}

		// Print the firmware status - after clearing the interrupt pins
		if (_printDebug == true)
		{
			ARTIC_R2_Firmware_Status status;
		  readStatusRegister(&status); // Read the ARTIC R2 status register
		  _debugPort->println(F("begin: ARTIC R2 Firmware Status (after clearInterrupts):"));
		  printFirmwareStatus(status, *_debugPort); // Pretty-print the firmware status
		}
	}

	// Read the checksum words
	uint32_t PMEM_CRC, XMEM_CRC, YMEM_CRC;
	readMemoryCRC(&PMEM_CRC, &XMEM_CRC, &YMEM_CRC);

	// Check that the checksums match
	if ((PMEM_CRC == ARTIC_R2_PMEM_CHECKSUM) && (XMEM_CRC == ARTIC_R2_XMEM_CHECKSUM) && (YMEM_CRC == ARTIC_R2_YMEM_CHECKSUM))
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: checksums match! Firmware upload was successful."));
		return (true);
	}
	else
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: checksums do not match! Firmware upload failed!"));
		return (false);
	}

#else

	// ARTIC boots from flash memory
	delay(ARTIC_R2_BOOT_DELAY_MS);
	unsigned long bootStartTime = millis();

	while (((millis() - bootStartTime) < ARTIC_R2_FLASH_BOOT_TIMEOUT) && (digitalRead(_int1) == LOW))
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: ARTIC is booting from flash (waiting for INT1 to go high)..."));
		delay(100);
	}

	if ((millis() - bootStartTime) >= ARTIC_R2_FLASH_BOOT_TIMEOUT)
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: boot timed out!  Carrying on regardless..."));
		//return (false); // Boot timed out!
	}

	// Print the firmware status - before we clear the interrupt pins
	if (_printDebug == true)
	{
		ARTIC_R2_Firmware_Status status;
		readStatusRegister(&status); // Read the ARTIC R2 status register
		_debugPort->println(F("begin: ARTIC R2 Firmware Status (before clearInterrupts):"));
		printFirmwareStatus(status, *_debugPort); // Pretty-print the firmware status
	}

	if(clearInterrupts(3) == false) // Clear both interrupts
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: failed to clear interrupts! Carrying on regardless..."));
		//return (false);
	}

	// Print the firmware status - after clearing the interrupt pins
	if (_printDebug == true)
	{
		ARTIC_R2_Firmware_Status status;
		readStatusRegister(&status); // Read the ARTIC R2 status register
		_debugPort->println(F("begin: ARTIC R2 Firmware Status (after clearInterrupts):"));
		printFirmwareStatus(status, *_debugPort); // Pretty-print the firmware status
	}

	return (true);

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
			if (_printDebug == true)
				_debugPort->println(F("setTXgain: gain set to 0"));
			return (true);
		}
		else if (gain == 8)
		{
			digitalWrite(_gain8, LOW);
			digitalWrite(_gain16, HIGH);
			if (_printDebug == true)
				_debugPort->println(F("setTXgain: gain set to 8"));
			return (true);
		}
		else if (gain == 16)
		{
			digitalWrite(_gain8, HIGH);
			digitalWrite(_gain16, LOW);
			if (_printDebug == true)
				_debugPort->println(F("setTXgain: gain set to 16"));
			return (true);
		}
		else if (gain == 24)
		{
			digitalWrite(_gain8, LOW);
			digitalWrite(_gain16, LOW);
			if (_printDebug == true)
				_debugPort->println(F("setTXgain: gain set to 24"));
			return (true);
		}
		else
			return (false);
	}
	else
	{
		if (_printDebug == true)
			_debugPort->println(F("setTXgain: _gain8 and _gain16 pins are not defined! Unable to set the TX gain!"));
		return (false);
	}
}

// Enable power for the ARTIC R2
void ARTIC_R2::enableARTICpower()
{
	digitalWrite(_pwr_en, ARTIC_R2_PWR_EN_ON);
}

// Disable power for the ARTIC R2
void ARTIC_R2::disableARTICpower()
{
	digitalWrite(_pwr_en, ARTIC_R2_PWR_EN_OFF);
}

// Read ARTIC R2 firmware status register
// The user can access the firmware status/interrupt flags by reading a single register.
// This 24-bit register is present in the IO memory at address 0x8018 and can be accessed using an SPI burst.
// This register can only be read.
// The complete 32-bit sequence is 0x000F8018. Next in a 24-bit SPI burst the register is read.
void ARTIC_R2::readStatusRegister(ARTIC_R2_Firmware_Status *status)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_FIRMWARE_STATUS_REGISTER;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_IO_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	status->STATUS_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status
}

// Pretty-print the firmware status to port
void ARTIC_R2::printFirmwareStatus(ARTIC_R2_Firmware_Status status, Stream &port)
{
	if (status.STATUS_REGISTER_BITS.IDLE) port.println(F("The firmware status IDLE flag is set. The firmware is idle and ready to accept commands."));
	if (status.STATUS_REGISTER_BITS.RX_IN_PROGRESS) port.println(F("The firmware status RX_IN_PROGRESS flag is set. The firmware is receiving."));
	if (status.STATUS_REGISTER_BITS.TX_IN_PROGRESS) port.println(F("The firmware status TX_IN_PROGRESS flag is set. The firmware is transmitting."));
	if (status.STATUS_REGISTER_BITS.BUSY) port.println(F("The firmware status BUSY flag is set. The firmware is busy changing state."));
	if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) port.println(F("The firmware status RX_VALID_MESSAGE flag is set. A message has been received."));
	if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) port.println(F("The firmware status RX_SATELLITE_DETECTED flag is set. A satellite has been detected."));
	if (status.STATUS_REGISTER_BITS.TX_FINISHED) port.println(F("The firmware status TX_FINISHED flag is set. The transmission was completed."));
	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED) port.println(F("The firmware status MCU_COMMAND_ACCEPTED flag is set. The configuration command has been accepted."));
	if (status.STATUS_REGISTER_BITS.CRC_CALCULATED) port.println(F("The firmware status CRC_CALCULATED flag is set. The CRC calculation has finished."));
	if (status.STATUS_REGISTER_BITS.IDLE_STATE) port.println(F("The firmware status IDLE_STATE flag is set. The firmware has returned to the idle state."));
	if (status.STATUS_REGISTER_BITS.RX_CALIBRATION_FINISHED) port.println(F("The firmware status RX_CALIBRATION_FINISHED flag is set. The RX offset calibration has completed."));
	if (status.STATUS_REGISTER_BITS.RX_TIMEOUT) port.println(F("The firmware status RX_TIMEOUT flag is set. The specified reception time has been exceeded."));
	if (status.STATUS_REGISTER_BITS.SATELLITE_TIMEOUT) port.println(F("The firmware status SATELLITE_TIMEOUT flag is set. No satellite was detected within the specified time."));
	if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) port.println(F("The firmware status RX_BUFFER_OVERFLOW flag is set. A received message is lost. No buffer space left."));
	if (status.STATUS_REGISTER_BITS.TX_INVALID_MESSAGE) port.println(F("The firmware status TX_INVALID_MESSAGE flag is set. Incorrect TX payload length specified."));
	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED) port.println(F("The firmware status MCU_COMMAND_REJECTED flag is set. Incorrect command sent or firmware is not in idle."));
	if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW) port.println(F("The firmware status MCU_COMMAND_OVERFLOW flag is set. Previous command was not yet processed."));
	if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR) port.println(F("The firmware status INTERNAL_ERROR flag is set. An internal error has occurred."));
	if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) port.println(F("The firmware status DSP2MCU_INT1 flag is set. Interrupt pin 1 is high."));
	if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) port.println(F("The firmware status DSP2MCU_INT2 flag is set. Interrupt pin 2 is high."));
}

// Read ARTIC R2 ARGOS configuration register
void ARTIC_R2::readARGOSconfiguration(ARGOS_Configuration_Register *configuration)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_ARGOS_CONFIGURATION;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

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
	uint32_t fourBytes = burstmode.BURSTMODE_REGISTER;

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
		//(command != CONFIG_CMD_SET_ARGOS_4_RX_MODE) &&  // Unsupported by ARTIC006! Use ARGOS 3 RX.
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

	delay(ARTIC_R2_CONFIGURATION_DELAY_MS);

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
		case ARTIC_R2_MCU_COMMAND_VALID:
			port.println(F("MCU command result: valid. A valid housekeeping command was attempted."));
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
//
// Notes: We know that configuration commands cause the MCU_COMMAND_ACCEPTED,
//        MCU_COMMAND_REJECTED and MCU_COMMAND_OVERFLOW flags to be set. But
//        it is unclear if MCU instructions use the same flags.
//        At present, it looks like the firmware status instead goes BUSY (changing state)
//        and then changes to (e.g.) RX_IN_PROGRESS (for satellite detection and message receive).
//        So, E.g.:
//          this function will return true if either BUSY or TX_IN_PROGRESS are raised
//          after starting reception or satellite detection
//
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

	// // Check if this is not a INST_GO_TO_IDLE instruction
	// if (instruction != INST_GO_TO_IDLE)
	// {
	// 	// Check that there is not already an instruction in progress
	// 	if (_instructionInProgress > 0)
	// 		return ARTIC_R2_MCU_INSTRUCTION_IN_PROGRESS; // Abort - an instruction is already in progress
	// }

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

	delay(ARTIC_R2_INSTRUCTION_DELAY_MS); // Delay before reading the status

	readStatusRegister(&status); // Read the status register again

	if ((instruction == INST_START_CONTINUOUS_RECEPTION) ||
		(instruction == INST_START_RECEIVING_1_MESSAGE) ||
		(instruction == INST_START_RECEIVING_2_MESSAGES) ||
		(instruction == INST_START_RECEPTION_FOR_FIXED_TIME) ||
		(instruction == INST_SATELLITE_DETECTION))
	{
		if ((status.STATUS_REGISTER_BITS.BUSY) || (status.STATUS_REGISTER_BITS.RX_IN_PROGRESS))
		{
			_instructionInProgress = instruction; // Update _instructionInProgress
			return ARTIC_R2_MCU_COMMAND_ACCEPTED;
		}
		else if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR)
		{
			_instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS; // Update _instructionInProgress (assume failure!)
			return ARTIC_R2_MCU_COMMAND_REJECTED;
		}
		else
		{
			_instructionInProgress = instruction; // Update _instructionInProgress (assume success!?)
			return ARTIC_R2_MCU_COMMAND_VALID;
		}
	}
	else if ((instruction == INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) ||
		(instruction == INST_TRANSMIT_ONE_PACKAGE_AND_START_RX))
	{
		if ((status.STATUS_REGISTER_BITS.BUSY) || (status.STATUS_REGISTER_BITS.TX_IN_PROGRESS))
		{
			_instructionInProgress = instruction; // Update _instructionInProgress
			return ARTIC_R2_MCU_COMMAND_ACCEPTED;
		}
		else if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR)
		{
			_instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS; // Update _instructionInProgress (assume failure!)
			return ARTIC_R2_MCU_COMMAND_REJECTED;
		}
		else
		{
			_instructionInProgress = instruction; // Update _instructionInProgress (assume success!?)
			return ARTIC_R2_MCU_COMMAND_VALID;
		}
	}
	else if (instruction == INST_GO_TO_IDLE)
	{
		if ((status.STATUS_REGISTER_BITS.BUSY) || (status.STATUS_REGISTER_BITS.IDLE_STATE))
		{
			_instructionInProgress = instruction; // Update _instructionInProgress
			return ARTIC_R2_MCU_COMMAND_ACCEPTED;
		}
		else if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR)
		{
			_instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS; // Update _instructionInProgress (assume failure!)
			return ARTIC_R2_MCU_COMMAND_REJECTED;
		}
		else
		{
			_instructionInProgress = instruction; // Update _instructionInProgress (assume success!?)
			return ARTIC_R2_MCU_COMMAND_VALID;
		}
	}
	else
	{
		_instructionInProgress = ARTIC_R2_MCU_PROGRESS_NONE_IN_PROGRESS; // Update _instructionInProgress (assume failure!)
		return ARTIC_R2_MCU_COMMAND_INVALID; // Hopefully this is impossible?
	}
}

// Send an MCU Housekeeping Command (i.e. Clear Interrupt)
// Write a single 8-bit command
ARTIC_R2_MCU_Command_Result ARTIC_R2::sendHousekeepingCommand(uint8_t command)
{
	// Check that the command is valid
	if((command != CMD_CLEAR_INT_1) &&
		(command != CMD_CLEAR_INT_2))
		return ARTIC_R2_MCU_COMMAND_INVALID; // Command is invalid

	ARTIC_R2_Firmware_Status status; // Read the status register before attempting to send the command

	// if (_printDebug == true)
	// {
	// 	readStatusRegister(&status);
	// 	if (!status.STATUS_REGISTER_BITS.IDLE) // If the firmware is not idle
	// 	{
	// 		_debugPort->println(F("sendHousekeepingCommand: ARTIC is not idle. This will probably fail."));
	// 	}
	// }

	// Write 8 bits to SPI:
	_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, ARTIC_R2_SPI_MODE));
	digitalWrite(_cs, LOW);

	_spiPort->transfer(command);

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();

	delay(ARTIC_R2_HOUSEKEEPING_DELAY_MS); // Delay before reading the status

	return (ARTIC_R2_MCU_COMMAND_VALID);
}

// Check the MCU instruction progress. Return true if the instruction is complete.
boolean ARTIC_R2::checkMCUinstructionProgress(ARTIC_R2_MCU_Instruction_Progress *progress)
{
	ARTIC_R2_Firmware_Status status; // Read the status register
	readStatusRegister(&status);

	// TO DO: Work out how to handle INTERNAL_ERROR. Maybe like this?
	if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR)
	{
		*progress = ARTIC_R2_MCU_PROGRESS_INTERNAL_ERROR;
		return true; // Return true to indicate the instruction is 'complete'?
	}

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
/*
	Note: here is what a satellite detection timeout looks like with ARTIC006 firmware:
	===================================================================================

	ARTIC R2 Firmware Status:
	The firmware status RX_IN_PROGRESS flag is set. The firmware is receiving.

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Satellite Detection in progress.

	ARTIC R2 Firmware Status:
	The firmware status IDLE flag is set. The firmware is idle and ready to accept commands.
	The firmware status IDLE_STATE flag is set. The firmware has returned to the idle state.
	The firmware status SATELLITE_TIMEOUT flag is set. No satellite was detected within the specified time.
	The firmware status DSP2MCU_INT1 flag is set. Interrupt pin 1 is high.
	The firmware status DSP2MCU_INT2 flag is set. Interrupt pin 2 is high.
	INT1 pin is high. Satellite detected!
	INT2 pin is high. Satellite detection has timed out!

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Satellite Detection is complete. Reception timed out.

*/
			if (status.STATUS_REGISTER_BITS.SATELLITE_TIMEOUT) // Has detection timed out?
			{
				// if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // Did we detect a satellite
				// {
				// 	*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT_WITH_SATELLITE_DETECTED;
				// 	return true; // We're done. Rx timed out. Note: this state may be impossible to reach. Status seems to go IDLE as soon as a satellite is detected.
				// }
				// else
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT;
					// Note: both INT2 and INT1 will be high. INT1 should be ignored...
					return true; // We're done. Rx timed out.
				}
			}
			else
			{
				if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // Have we detected a satellite?
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_RX_SATELLITE_DETECTED;
					return true; // We are done. Status seems to never time out once a satellite has been detected. It goes IDLE instead.
				}
				else
				{
					*progress = ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION;
					return false; // We are not done.
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
	// case ARTIC_R2_MCU_PROGRESS_SATELLITE_DETECTION_SATELLITE_TIMEOUT_WITH_SATELLITE_DETECTED:
  // 	port.println(F("MCU Instruction Progress: Satellite Detection is complete. Reception timed out. A satellite was detected."));
	// 	break;
	case ARTIC_R2_MCU_PROGRESS_INTERNAL_ERROR:
		port.println(F("MCU Instruction Progress: INTERNAL ERROR!"));
		break;
	case ARTIC_R2_MCU_PROGRESS_UNKNOWN_INSTRUCTION:
  	port.println(F("MCU Instruction Progress: UNKNOWN INSTRUCTION!"));
		break;
	default:
		port.println(F("MCU Instruction Progress: UNKNOWN PROGRESS STATE! This should be impossible!"));
		break;
	}
}

// Read and return the firmware version from PMEM
void ARTIC_R2::readFirmwareVersion(char *buffer)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_FIRMWARE_VERSION;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_P_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords((uint8_t *)buffer, 32, 2); // Read 2 * 32-bit words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	buffer[8] = '\0'; // Null-terminate the string

	if (_printDebug == true)
	{
		_debugPort->print(F("readFirmwareVersion: firmware version: "));
		_debugPort->println(buffer);
	}
}

// Read the memories CRCs (after firmware boot)
void ARTIC_R2::readMemoryCRC(uint32_t *PMEM_CRC, uint32_t *XMEM_CRC, uint32_t *YMEM_CRC)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_CRC_RESULTS;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[12]; // Buffer for the CRC words
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 3); // Read 3 * 24-bit words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	// Extract the checksums
	*PMEM_CRC = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);
	*XMEM_CRC = (((uint32_t)buffer[3]) << 16) | (((uint32_t)buffer[4]) << 8) | ((uint32_t)buffer[5]);
	*YMEM_CRC = (((uint32_t)buffer[6]) << 16) | (((uint32_t)buffer[7]) << 8) | ((uint32_t)buffer[8]);

	if (_printDebug == true)
	{
		_debugPort->print(F("readMemoryCRC: firmware checksum PMEM: 0x"));
		_debugPort->print(*PMEM_CRC, HEX);
		_debugPort->print(F("  XMEM: 0x"));
		_debugPort->print(*XMEM_CRC, HEX);
		_debugPort->print(F("  YMEM: 0x"));
		_debugPort->println(*YMEM_CRC, HEX);
	}
}

// Set the RX timeout (seconds)
// Returns true if the timeout was set successfully
boolean ARTIC_R2::setRxTimeout(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readRxTimeout() == timeout_secs);
}

// Read and return the RX timeout (seconds)
uint32_t ARTIC_R2::readRxTimeout()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the rx timeout
}

// Set the satellite detection timeout (seconds)
// Returns true if the timeout was set successfully
boolean ARTIC_R2::setSatelliteDetectionTimeout(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_SATELLITE_DETECTION_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readSatelliteDetectionTimeout() == timeout_secs);
}

// Read and return the satellite detection timeout
uint32_t ARTIC_R2::readSatelliteDetectionTimeout()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_SATELLITE_DETECTION_TIMEOUT;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the satellite detection timeout
}

// Set the TCXO warm up time (seconds)
// Returns true if the warm up time was set successfully
boolean ARTIC_R2::setTCXOWarmupTime(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_WARMUP_TIME;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readTCXOWarmupTime() == timeout_secs);
}

// Read and return the TCXO warm up time
uint32_t ARTIC_R2::readTCXOWarmupTime()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_WARMUP_TIME;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the TCXO warm up time
}

// Set the TX certification interval
// Returns true if the interval was set successfully
boolean ARTIC_R2::setTxCertificationInterval(uint32_t timeout_secs)
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_CERTIFICATION_INTERVAL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(timeout_secs); // Set the timeout

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	return (readTxCertificationInterval() == timeout_secs);
}

// Read and return the TX certification interval
uint32_t ARTIC_R2::readTxCertificationInterval()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_CERTIFICATION_INTERVAL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	return (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the TX certification interval
}

// Set the TCXO control voltage and auto-disable
// Returns true if the values were set successfully
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
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_CONTROL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	write24BitWord(tcxo_control.TCXO_CONTROL_REGISTER); // Set the TCXO control

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Read the new values
	float new_voltage = readTCXOControlVoltage();
	boolean new_disable = readTCXOAutoDisable();

	return ((new_voltage == voltage) && (new_disable == autoDisable));
}

// Read and return the TCXO control voltage. Auto-disable is ignored.
float ARTIC_R2::readTCXOControlVoltage()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_CONTROL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

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

// Read and return the TCXO auto-disable bit
boolean ARTIC_R2::readTCXOAutoDisable()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TCXO_CONTROL;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	TCXO_Control_Register tcxo_control;
	tcxo_control.TCXO_CONTROL_REGISTER = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]); // Return the firmware status

	if (tcxo_control.CONTROL_REGISTER_BITS.AUTO_DISABLE == 1)
		return (true);
	else
		return (false);
}

// Set the ARGOS 2/3 TX Frequency
// Returns true if the frequency was set correctly in XMEM
boolean ARTIC_R2::setARGOS23TxFrequency(float freq_MHz)
{
	return setARGOSTxFrequency(MEM_LOC_TX_FREQ_ARGOS_2_3, freq_MHz);
}

// Set the ARGOS 4 TX Frequency
// Returns true if the frequency was set correctly in XMEM
boolean ARTIC_R2::setARGOS4TxFrequency(float freq_MHz)
{
	return setARGOSTxFrequency(MEM_LOC_TX_FREQ_ARGOS_4, freq_MHz);
}

// Set the ARGOS TX Frequency using the equation defined in section 3.5.3 of the ARTIC R2 datasheet
// Returns true if the frequency was set correctly in XMEM
boolean ARTIC_R2::setARGOSTxFrequency(uint16_t mem_loc, float freq_MHz)
{
	double fractional_part = (double)freq_MHz; // Convert freq to double
	fractional_part *= 4; // Multiply by 4
	fractional_part /= 26; // Divide by 26 (MHz)
	fractional_part -= 61; // Subtract 61
	fractional_part *= 4194304; // Multiply by 2^22

	uint32_t round = (uint32_t)fractional_part;

	if (_printDebug == true)
	{
		_debugPort->print(F("setARGOSTxFrequency: setting XMEM location 0x"));
		_debugPort->print(mem_loc, HEX);
		_debugPort->print(F(" to 0x"));
		_debugPort->println(round, HEX);
	}

	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = mem_loc;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(round); // Update frequency (fractional PLL parameter)

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Check that the frequency is correct by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = mem_loc;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	uint32_t new_val = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	return (new_val == round);
}

// Get the ARGOS 2/3 TX Frequency
// Returns the TX Frequency in MHz
float ARTIC_R2::getARGOS23TxFrequency()
{
	return getARGOSTxFrequency(MEM_LOC_TX_FREQ_ARGOS_2_3);
}

// Set the ARGOS 4 TX Frequency
// Returns the TX Frequency in MHz
float ARTIC_R2::getARGOS4TxFrequency()
{
	return getARGOSTxFrequency(MEM_LOC_TX_FREQ_ARGOS_4);
}

// Get the ARGOS TX Frequency using the equation defined in section 3.5.3 of the ARTIC R2 datasheet
// Returns the frequency in MHz as float
float ARTIC_R2::getARGOSTxFrequency(uint16_t mem_loc)
{
	// Read the TX frequency from memory
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = mem_loc;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	uint32_t round = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	double fractional_part = (double)round; // Convert to double

	fractional_part /= 4194304; // Divide by 2^22
	fractional_part += 61; // Add 61
	fractional_part *= 26; // Multiply by 26 (MHz)
	fractional_part /= 4; // Divide by 4

	if (_printDebug == true)
	{
		_debugPort->print(F("getARGOSTxFrequency: XMEM location 0x"));
		_debugPort->print(mem_loc, HEX);
		_debugPort->print(F(" contains 0x"));
		_debugPort->print(round, HEX);
		_debugPort->print(F(" which equals "));
		_debugPort->print((float)fractional_part, 3);
		_debugPort->println(F(" MHz"));
	}

	return ((float)fractional_part);
}

// Enable the RX CRC check by writing 0x000001 to MEM_LOC_RX_FILTERING_ENABLE_CRC
// Returns true if the CRC check was enabled successfully
boolean ARTIC_R2::enableRXCRC()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(0x000001); // Enable CRC

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Check that the CRC is enabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x01)
		return (true);
	else
		return (false);
}

// Disable the RX CRC check by writing 0x000000 to MEM_LOC_RX_FILTERING_ENABLE_CRC
// Returns true if the CRC check was successfully disabled
boolean ARTIC_R2::disableRXCRC()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(0x000000); // Disable CRC

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Check that the CRC is disabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x00)
		return (true);
	else
		return (false);
}

// Check if the RX CRC is enabled by reading MEM_LOC_RX_FILTERING_ENABLE_CRC
// Returns true if the CRC check is enabled
boolean ARTIC_R2::isRXCRCenabled()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_ENABLE_CRC;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x01)
		return (true);
	else
		return (false);
}

// Enable the RX transparent mode by writing 0x000001 to MEM_LOC_RX_FILTERING_TRANSPARENT_MODE
// All messages are send to the MCU.
// Returns true if transparent mode was successfully enabled.
boolean ARTIC_R2::enableRXTransparentMode()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(0x000001); // Enable transparent mode

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Check that transparent mode is enabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x01)
		return (true);
	else
		return (false);
}

// Disable the RX transparent mode by writing 0x000000 to MEM_LOC_RX_FILTERING_TRANSPARENT_MODE
// Only messages with an ID mentioned in the Address LUT are sent to the MCU.
// Returns true if transparent mode was successfully disabled.
boolean ARTIC_R2::disableRXTransparentMode()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(0x000000); // Disable transparent mode

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Check that transparent mode is disabled by reading the value back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x00)
		return (true);
	else
		return (false);
}

// Read and return MEM_LOC_RX_FILTERING_TRANSPARENT_MODE
boolean ARTIC_R2::isRXTransparentModeEnabled()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_TRANSPARENT_MODE;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x01)
		return (true);
	else
		return (false);
}

// Clear the address LUT by writing 0x000000 to MEM_LOC_RX_FILTERING_LUT_LENGTH
// Returns true if the LUT length was successfully set to zero.
boolean ARTIC_R2::clearAddressLUT()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(0x000000); // Clear the LUT

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Check that the LUT is clear by reading the length back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	if (buffer[2] == 0x00)
		return (true);
	else
		return (false);
}

// Add a new address (platform ID) to the message filtering LUT
// Returns true if the address was added successfully.
// Will return false if the table is full.
boolean ARTIC_R2::addAddressToLUT(uint32_t platformID)
{
	uint32_t AddressLSBits = platformID & 0xFFFFFF; // Extract the 24 LS bits
	uint32_t AddressMSBits = (platformID >> 24) & 0x0F; // Extract the 4 MS bits (platform ID is 28 bits, not 32)

	// Read the LUT Length
	uint32_t tableLength = (uint32_t)getAddressLUTlength();

	if (tableLength >= ARTIC_R2_MAX_ADDRESS_LUT_LENGTH)
	{
		if (_printDebug == true)
			_debugPort->println(F("addAddressToLUT: address table is full!"));
		return (false);
	}

	// Write the new address to the LUT
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_FIRST_ADDRESS + (tableLength << 1); // Calculate where to store the address (each entry occupies two words)
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	writeTwo24BitWords(AddressLSBits, AddressMSBits); // Write the address words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Now increment the table length

	tableLength++; // Increment the table length by one

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(tableLength); // Write the incremented length

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Finally, read the LUT Length and check it was incremented correctly
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read the LUT length

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

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

// Reads and returns MEM_LOC_RX_FILTERING_LUT_LENGTH
int ARTIC_R2::getAddressLUTlength()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_LENGTH;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	uint32_t lut_len = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	if (_printDebug == true)
	{
		_debugPort->print(F("getAddressLUTlength: table length is "));
		_debugPort->println((int)lut_len);
	}

	return ((int)lut_len);
}

// Read and return the chosen LUT entry
// Returns true if the LUT entry was valid
// Will return false if the entry is invalid (i.e. < current table length)
// entry numbers starts at zero. If entry is zero, the first address in the LUT is returned (if valid).
boolean ARTIC_R2::readAddressLUTentry(int entry, uint32_t *address)
{
	int currentLUTlength = getAddressLUTlength(); // Read the LUT length

	if (entry >= currentLUTlength) // Check that the entry is valid
	{
		if (_printDebug == true)
			_debugPort->println(F("readAddressLUTentry: invalid entry!"));
		return (false);
	}

	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_FILTERING_LUT_FIRST_ADDRESS + (((uint32_t)entry) << 1); // Calculate the address (each entry occupies two words)
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[6]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 2); // Read 2 24-bit words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	// LUT entries are stored LS 24 bits first
	uint32_t lut_address = (((uint32_t)buffer[5]) << 24) | (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	if (_printDebug == true)
	{
		_debugPort->print(F("readAddressLUTentry: LUT entry "));
		_debugPort->print(entry);
		_debugPort->print(F(" is 0x"));
		_debugPort->println(lut_address, HEX);
	}

	*address = lut_address;

	return (true);
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
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_RX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[9*3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 9); // Read 9 24-bit words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

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
		_debugPort->print(F("readDownlinkMessage: payloadLength in bits is "));
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

	// Calculate the number of bytes in the message
	// Payload length does include the 7 bytes for the Addressee ID, ADCS, Service and FCS
	int numBytes = ((downlinkMessage->payloadLength) >> 3) - 7; // Divide by 8 and subtract 7 (for the Addressee ID, ADCS, Service and FCS)

	// Copy the data bytes into rxData
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: payload data is 0x"));
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
	if (_printDebug == true)
	{
		_debugPort->println(); // Tidy up debug printing
	}

	// FCS is always byte-aligned
	downlinkMessage->FCS = (((uint16_t)buffer[numBytes + 8]) << 8) | ((uint16_t)buffer[numBytes + 9]);
	if (_printDebug == true)
	{
		_debugPort->print(F("readDownlinkMessage: FCS is 0x"));
		_debugPort->println(downlinkMessage->FCS, HEX);
	}

	return (true);
}

// Set the Tx payload for a ARGOS 4 VLD message with 0 bits of user data
// The payload contains _only_ the 28-bit platform ID (left justified)
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS4VLD0(uint32_t platformID)
{
	_txPayloadLengthBits = 28;
	_txPayloadBytes[0] = (platformID >> 20) & 0xFF; // Left justify the 28-bit platform ID
	_txPayloadBytes[1] = (platformID >> 12) & 0xFF;
	_txPayloadBytes[2] = (platformID >> 4) & 0xFF;
	_txPayloadBytes[3] = (platformID << 4) & 0xFF;

	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS4VLD0: left-justified payload is 0x"));
		_debugPort->print(_txPayloadBytes[0], HEX);
		_debugPort->print(_txPayloadBytes[1], HEX);
		_debugPort->print(_txPayloadBytes[2], HEX);
		_debugPort->println(_txPayloadBytes[3], HEX);
	}

	return setTxPayload();
}

// Set the Tx payload for a ARGOS 4 VLD message with 28 bits of user data
// The payload contains the 28-bit platform ID and the 28 bits of user data (left justified)
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS4VLD28(uint32_t platformID, uint32_t userData)
{
	_txPayloadLengthBits = 56;
	_txPayloadBytes[0] = (platformID >> 20) & 0xFF; // Left justify the 28-bit platform ID
	_txPayloadBytes[1] = (platformID >> 12) & 0xFF;
	_txPayloadBytes[2] = (platformID >> 4) & 0xFF;
	_txPayloadBytes[3] = ((platformID << 4) | ((userData >> 24) & 0xF)) & 0xFF; // Left justify the 28 bits of user data
	_txPayloadBytes[4] = (userData >> 16) & 0xFF;
	_txPayloadBytes[5] = (userData >> 8) & 0xFF;
	_txPayloadBytes[6] = userData & 0xFF;

	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS4VLD28: left-justified payload is 0x"));
		_debugPort->print(_txPayloadBytes[0], HEX);
		_debugPort->print(_txPayloadBytes[1], HEX);
		_debugPort->print(_txPayloadBytes[2], HEX);
		_debugPort->print(_txPayloadBytes[3], HEX);
		_debugPort->print(_txPayloadBytes[4], HEX);
		_debugPort->print(_txPayloadBytes[5], HEX);
		_debugPort->println(_txPayloadBytes[6], HEX);
	}

	return setTxPayload();
}

// Set the Tx payload by copying txPayloadLengthBits and txPayloadBytes into X memory
// Returns true if the payload was copied successfully
boolean ARTIC_R2::setTxPayload()
{
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	write24BitWord(_txPayloadLengthBits); // Write the encoded message length

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	int totalWords = ((int)_txPayloadLengthBits) / 24; // Convert bits to words
	if ((((int)_txPayloadLengthBits) % 24) > 0) totalWords++; // Increment totalWords by 1 if there are any additional bits

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_PAYLOAD + 1;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	uint8_t *bufferPtr = _txPayloadBytes;

	writeMultipleWords(bufferPtr, 24, totalWords); // Write the words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Read the encoded message length back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	uint32_t newPayloadLength = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

 	return (newPayloadLength == _txPayloadLengthBits);
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

// Write multiple words to the ARTIC R2
void ARTIC_R2::writeMultipleWords(uint8_t *buffer, int wordSizeInBits, int numWords)
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
			_spiPort->transfer(buffer[bufferPointer++]);
		}
	}

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
		for (int byteSize = 0; byteSize < (wordSizeInBits >> 3); byteSize++)
		{
			buffer[bufferPointer++] = _spiPort->transfer(0xFF);
		}
	}

	digitalWrite(_cs, HIGH);
	_spiPort->endTransaction();
}

// Clear one or both interrupts
// Returns true if the interrupts were cleared successfully
boolean ARTIC_R2::clearInterrupts(uint8_t interrupts)
{
	if ((interrupts < 1) || (interrupts > 3))
		return (false);

	ARTIC_R2_MCU_Command_Result commandResult;

	if ((interrupts == 1) || (interrupts == 3)) // Clear interrupt 1
	{
		commandResult = sendHousekeepingCommand(CMD_CLEAR_INT_1);
		if (_printDebug == true)
		{
			_debugPort->print(F("clearInterrupts: clear INT1: commandResult is: "));
			printCommandResult(commandResult, *_debugPort);
		}
	}

	if ((interrupts == 2) || (interrupts == 3)) // Clear interrupt 2
	{
		commandResult = sendHousekeepingCommand(CMD_CLEAR_INT_2);
		if (_printDebug == true)
		{
			_debugPort->print(F("clearInterrupts: clear INT2: commandResult is: "));
			printCommandResult(commandResult, *_debugPort);
		}
	}

	// Check that the interrupt flags have been cleared (hopefully redundant?)
	ARTIC_R2_Firmware_Status status;
	readStatusRegister(&status);

	if ((interrupts == 1) || (interrupts == 3)) // Check if interrupt 1 is clear
	{
		if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // If the interrupt bit is still set
		{
			if (_printDebug == true)
			{
				_debugPort->println(F("clearInterrupts: failed to clear INT1!"));
			}
			return (false); // Flag is still set so clearing failed...
		}
	}

	if ((interrupts == 2) || (interrupts == 3)) // Check if interrupt 2 is clear
	{
		if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // If the interrupt bit is still set
		{
			if (_printDebug == true)
			{
				_debugPort->println(F("clearInterrupts: failed to clear INT2!"));
			}
			return (false); // Flag is still set so clearing failed...
		}
	}

	return (true); // Success!
}
