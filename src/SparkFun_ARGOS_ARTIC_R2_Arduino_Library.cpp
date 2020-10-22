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

boolean ARTIC_R2::begin(int user_CSPin, int user_RSTPin, int user_BOOTPin, int user_PWRENPin, int user_INT1Pin, int user_INT2Pin, int user_GAIN8Pin, int user_GAIN16Pin, unsigned long spiPortSpeed, SPIClass &spiPort)
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
		setTXgain(0); // Set the TX gain to minimum while we turn the power on, to reduce the current surge
		delay(ARTIC_R2_TX_POWER_ON_DELAY_MS);
	}

	delay(ARTIC_R2_POWER_ON_DELAY_MS); // Make sure the power has been turned off for at least ARTIC_R2_POWER_ON_DELAY_MS

	enableARTICpower(); // Enable power for the ARTIC R2

	delay(ARTIC_R2_POWER_ON_DELAY_MS); // Wait for ARTIC_R2_POWER_ON_DELAY_MS

	// Now ramp up the TX gain to 24, if the _gain8 and _gain16 pins are defined, to reduce the current surge
	if ((_gain8 >= 0) && (_gain16 >= 0))
	{
		setTXgain(16);
		delay(ARTIC_R2_TX_POWER_ON_DELAY_MS);
		setTXgain(24);
		delay(ARTIC_R2_TX_POWER_ON_DELAY_MS);
	}

	//Bring the ARTIC out of reset
	digitalWrite(_rst, HIGH);

	if (_printDebug == true)
		_debugPort->println(F("begin: IO pins are configured. ARTIC has been reset."));

#ifdef ARTIC_R2_UPLOAD_FIRMWARE

	// This do-while is based extensively on Arribada Horizon. Thank you Arribada!
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

	while (((millis() - bootStartTime) < ARTIC_R2_BOOT_TIMEOUT_MS) && (digitalRead(_int1) == LOW))
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: waiting for the ARTIC to boot (checking if INT1 has gone high)..."));
		delay(100);
	}

	if ((millis() - bootStartTime) >= ARTIC_R2_BOOT_TIMEOUT_MS)
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: boot timed out! INT1 did not go high!"));
		return (false); // Boot timed out!
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
				_debugPort->println(F("begin: failed to clear interrupts!"));
			return (false);
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

	while (((millis() - bootStartTime) < ARTIC_R2_FLASH_BOOT_TIMEOUT_MS) && (digitalRead(_int1) == LOW))
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: ARTIC is booting from flash (waiting for INT1 to go high)..."));
		delay(100);
	}

	if ((millis() - bootStartTime) >= ARTIC_R2_FLASH_BOOT_TIMEOUT_MS)
	{
		if (_printDebug == true)
			_debugPort->println(F("begin: boot timed out! INT1 did not go high!"));
		return (false); // Boot timed out!
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
			_debugPort->println(F("begin: failed to clear interrupts!"));
		return (false);
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
	if (_invertedPWREN == false) // Default: SparkFun ARTIC R2 Breakout
		digitalWrite(_pwr_en, LOW);
	else
		digitalWrite(_pwr_en, HIGH); // Arribada Horizon
}

// Disable power for the ARTIC R2
void ARTIC_R2::disableARTICpower()
{
	if (_invertedPWREN == false) // Default: SparkFun ARTIC R2 Breakout
		digitalWrite(_pwr_en, HIGH);
	else
		digitalWrite(_pwr_en, LOW); // Arribada Horizon
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
			_debugPort->println(F("sendConfigurationCommand: ARTIC is not idle."));
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
		if (_printDebug == true)
		{
			if (!status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // If the INT1 flag is _not_set
			{
				_debugPort->println(F("sendConfigurationCommand: command was accepted but INT1 did not go high!"));
			}
		}

		boolean clrIntResult = clearInterrupts(1); // Clear INT1

		if (_printDebug == true)
		{
			if (!clrIntResult) // If clearing the INT1 flag failed
			{
				_debugPort->println(F("sendConfigurationCommand: failed to clear INT1!"));
			}
		}

		return ARTIC_R2_MCU_COMMAND_ACCEPTED;
	}
	else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED)
	{
		if (_printDebug == true)
		{
			if (!status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // If the INT2 flag is _not_set
			{
				_debugPort->println(F("sendConfigurationCommand: command was REJECTED but INT2 did not go high!"));
			}
		}

		boolean clrIntResult = clearInterrupts(2); // Clear INT2

		if (_printDebug == true)
		{
			if (!clrIntResult) // If clearing the INT2 flag failed
			{
				_debugPort->println(F("sendConfigurationCommand: failed to clear INT2!"));
			}
		}

		return ARTIC_R2_MCU_COMMAND_REJECTED;
	}
	else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW)
	{
		if (_printDebug == true)
		{
			_debugPort->println(F("sendConfigurationCommand: COMMAND OVERFLOW!"));
		}

		boolean clrIntResult = clearInterrupts(3); // Clear both interrupts - because we probably should?

		if (_printDebug == true)
		{
			if (!clrIntResult) // If clearing the INT flags failed
			{
				_debugPort->println(F("sendConfigurationCommand: failed to clear interrupts!"));
			}
		}

		return ARTIC_R2_MCU_COMMAND_OVERFLOW;
	}
	else
	{
		clearInterrupts(3); // Clear both interrupts - because we probably should?

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
			_debugPort->println(F("sendMCUinstruction: ARTIC is not idle."));
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
	// 		_debugPort->println(F("sendHousekeepingCommand: ARTIC is not idle."));
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
/*
	Note: here is what transmit one package looks like with ARTIC006 firmware:
	==========================================================================

	ARTIC R2 Firmware Status:
	The firmware status BUSY flag is set. The firmware is busy changing state.

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Transmit One Package And Go Idle in progress.

	ARTIC R2 Firmware Status:
	The firmware status TX_IN_PROGRESS flag is set. The firmware is transmitting.

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Transmit One Package And Go Idle in progress.

	ARTIC R2 Firmware Status:
	The firmware status IDLE flag is set. The firmware is idle and ready to accept commands.
	The firmware status TX_FINISHED flag is set. The transmission was completed.
	The firmware status IDLE_STATE flag is set. The firmware has returned to the idle state.
	The firmware status DSP2MCU_INT1 flag is set. Interrupt pin 1 is high.
	INT1 pin is high. TX is finished (or MCU is in IDLE_STATE)!

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Transmit One Package And Go Idle is complete. Message was transmitted.

*/
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
	INT2 pin is high. Satellite detection has timed out!

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Satellite Detection is complete. Reception timed out.

*/
/*
	Note: here is what a successful satellite detection looks like with ARTIC006 firmware:
	======================================================================================

	ARTIC R2 Firmware Status:
	The firmware status RX_IN_PROGRESS flag is set. The firmware is receiving.

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Satellite Detection in progress.

	ARTIC R2 Firmware Status:
	The firmware status IDLE flag is set. The firmware is idle and ready to accept commands.
	The firmware status RX_SATELLITE_DETECTED flag is set. A satellite has been detected.
	The firmware status IDLE_STATE flag is set. The firmware has returned to the idle state.
	The firmware status DSP2MCU_INT1 flag is set. Interrupt pin 1 is high.
	INT1 pin is high. Satellite detected!

	ARTIC R2 instruction progress:
	MCU Instruction Progress: Satellite Detection in progress. A satellite has been detected.
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
  	port.println(F("MCU Instruction Progress: Transmit One Package And Go Idle in progress."));
		break;
	case ARTIC_R2_MCU_PROGRESS_TRANSMIT_ONE_GO_IDLE_IDLE_STATE:
  	port.println(F("MCU Instruction Progress: Transmit One Package And Go Idle is complete. MCU has returned to the idle state."));
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
  	port.println(F("MCU Instruction Progress: Go To Idle is complete. MCU has returned to the idle state."));
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

// Set the Tx payload for a ARGOS 3 ZE message
// The payload contains _only_ the 28-bit platform ID (left justified) plus 8 tail bits (0x00)
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS3ZE(uint32_t platformID)
{
	// Tx length in bits
	// For PTT-ZE messages, the total message length is the sum of:
	//   28 bits for the Platform ID
	//   8 tail bits
	_txPayloadBytes[0] = 0x00;
	_txPayloadBytes[1] = 0x00;
	_txPayloadBytes[2] = ARTIC_R2_PLATFORM_ID_BITS + ARTIC_R2_PTT_ZE_NUM_TAIL_BITS;

	// The payload
	_txPayloadBytes[3] = (platformID >> 20) & 0xFF; // Left justify the 28-bit platform ID
	_txPayloadBytes[4] = (platformID >> 12) & 0xFF;
	_txPayloadBytes[5] = (platformID >> 4) & 0xFF;
	_txPayloadBytes[6] = (platformID << 4) & 0xF0; // Last 4 bits of the platform ID plus 4 tail bits
	_txPayloadBytes[7] = 0x00; // Remaining 4 tail bits plus four stuff bits
	_txPayloadBytes[8] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)

	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS3ZE: left-justified payload is 0x"));
		for (uint16_t i = 0; i < 9; i++)
		{
			if (_txPayloadBytes[i] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(_txPayloadBytes[i], HEX);
		}
		_debugPort->println();
	}

	return setTxPayload();
}

// Set the Tx payload for a ARGOS 3 PTT-A3 message
// The message contains the GPS latitude and longitude in a compact form which ARGOS Web will understand.
// Please contact CLS / Woods Hole Group and ask them to apply the SPARKFUN_GPS template on ARGOS Web.
// The Latitude and Longitude will then be extracted, converted and displayed automatically when you view your data.
// The number of user bits is 56.
// Lat is encoded as 21 bits: the MSB is 0 for +ve latitude, 1 for -ve latitude (SOUTH); the unit is 0.0001 degrees. (Note: this is not two's complement!)
// Lon is encoded as 22 bits: the MSB is 0 for +ve longitude, 1 for -ve longitude (WEST); the unit is 0.0001 degrees. (Note: this is not two's complement!)
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS3LatLon(uint32_t platformID, float Lat, float Lon)
{
	// Tx length in bits
	// For PTT-A3 messages, the total message length is the sum of:
	//   4 message length bits
	//   28 bits for the Platform ID
	//   the number of user bits: 24, 56, 88, 120, 152, 184, 216 or 248
	//   7, 8 or 9 tail bits: see ARTIC_R2_PTT_A3_NUM_TAIL_BITS_
	_txPayloadBytes[0] = 0x00;
	_txPayloadBytes[1] = 0x00;
	_txPayloadBytes[2] = ARTIC_R2_PTT_A3_MESSAGE_LENGTH_BITS + ARTIC_R2_PLATFORM_ID_BITS + 56 + ARTIC_R2_PTT_A3_NUM_TAIL_BITS_56;

	// The payload itself
	_txPayloadBytes[3] = (ARTIC_R2_PTT_A3_MESSAGE_LENGTH_56 << 4) | ((platformID >> 24) & 0x0F); // Message length and the first 4 bits of the 28-bit platform ID
	_txPayloadBytes[4] = (platformID >> 16) & 0xFF;
	_txPayloadBytes[5] = (platformID >> 8) & 0xFF;
	_txPayloadBytes[6] = platformID & 0xFF;

	boolean negative = false;
	if (Lat < 0.0)
	{
		negative = true; // Is the Lat negative?
		Lat = 0.0 - Lat; // Make it +ve
	}
	Lat *= 10000.0; // Shift by 4 decimal places
	uint32_t Lat_32 = (uint32_t)Lat; // Convert to uint32_t
	if (negative) Lat_32 |= 0x100000; // Set the MS bit if Lat was negative (note: this is not two's complement)
	_txPayloadBytes[7] = (Lat_32 >> 13) & 0xFF; // Load 8 bits of Lat into the payload
	_txPayloadBytes[8] = (Lat_32 >> 5) & 0xFF; // Load 8 bits of Lat into the payload
	_txPayloadBytes[9] = (Lat_32 << 3) & 0xF8; // Load 5 bits of Lat into the payload

	negative = false;
	if (Lon < 0.0)
	{
		negative = true; // Is the Lon negative?
		Lon = 0.0 - Lon; // Make it +ve
	}
	Lon *= 10000.0; // Shift by 4 decimal places
	uint32_t Lon_32 = (uint32_t)Lon; // Convert to uint32_t
	if (negative) Lon_32 |= 0x200000; // Set the MS bit if Lon was negative (note: this is not two's complement)
	_txPayloadBytes[9] |= (Lon_32 >> 19) & 0x07; // Load 3 bits of Lon into the payload
	_txPayloadBytes[10] = (Lon_32 >> 11) & 0xFF; // Load 8 bits of Lon into the payload
	_txPayloadBytes[11] = (Lon_32 >> 3) & 0xFF; // Load 8 bits of Lon into the payload
	_txPayloadBytes[12] = (Lon_32 << 5) & 0xE0; // Load 3 bits of Lon into the payload, pad with five stuff bits
	_txPayloadBytes[13] = 0x00; // Eight stuff bits
	_txPayloadBytes[14] = 0x00; // Eight tail bits
	// (No need to stuff buffer with zeros to a multiple of 24 bits)
	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS3LatLon: left-justified payload is 0x"));
		for (uint16_t i = 0; i < 15; i++)
		{
			if (_txPayloadBytes[i] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(_txPayloadBytes[i], HEX);
		}
		_debugPort->println();
	}

	return setTxPayload();
}

// Set the Tx payload for a ARGOS PTT-A2 message
// The message contains the GPS latitude and longitude in a compact form which ARGOS Web will understand.
// Please contact CLS / Woods Hole Group and ask them to apply the SPARKFUN_GPS template on ARGOS Web.
// The Latitude and Longitude will then be extracted, converted and displayed automatically when you view your data.
// The number of user bits is 56.
// Lat is encoded as 21 bits: the MSB is 0 for +ve latitude, 1 for -ve latitude (SOUTH); the unit is 0.0001 degrees. (Note: this is not two's complement!)
// Lon is encoded as 22 bits: the MSB is 0 for +ve longitude, 1 for -ve longitude (WEST); the unit is 0.0001 degrees. (Note: this is not two's complement!)
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS2LatLon(uint32_t platformID, float Lat, float Lon)
{
	// Tx length in bits
	// For PTT-A2 messages, the total message length is the sum of:
	//   4 message length bits
	//   28 bits for the Platform ID
	//   the number of user bits: 24, 56, 88, 120, 152, 184, 216 or 248 (with a 28-bit Platform ID)
	// (PTT-A2 messages can also use a 20-bit Platform ID)

	_txPayloadBytes[0] = 0x00;
	_txPayloadBytes[1] = 0x00;
	_txPayloadBytes[2] = ARTIC_R2_PTT_A2_MESSAGE_LENGTH_BITS + ARTIC_R2_PLATFORM_ID_BITS + 56;

	// The payload itself
	_txPayloadBytes[3] = (ARTIC_R2_PTT_A2_MESSAGE_LENGTH_56 << 4) | ((platformID >> 24) & 0x0F); // Message length and the first 4 bits of the 28-bit platform ID
	_txPayloadBytes[4] = (platformID >> 16) & 0xFF;
	_txPayloadBytes[5] = (platformID >> 8) & 0xFF;
	_txPayloadBytes[6] = platformID & 0xFF;

	boolean negative = false;
	if (Lat < 0.0)
	{
		negative = true; // Is the Lat negative?
		Lat = 0.0 - Lat; // Make it +ve
	}
	Lat *= 10000.0; // Shift by 4 decimal places
	uint32_t Lat_32 = (uint32_t)Lat; // Convert to uint32_t
	if (negative) Lat_32 |= 0x100000; // Set the MS bit if Lat was negative (note: this is not two's complement)
	_txPayloadBytes[7] = (Lat_32 >> 13) & 0xFF; // Load 8 bits of Lat into the payload
	_txPayloadBytes[8] = (Lat_32 >> 5) & 0xFF; // Load 8 bits of Lat into the payload
	_txPayloadBytes[9] = (Lat_32 << 3) & 0xF8; // Load 5 bits of Lat into the payload

	negative = false;
	if (Lon < 0.0)
	{
		negative = true; // Is the Lon negative?
		Lon = 0.0 - Lon; // Make it +ve
	}
	Lon *= 10000.0; // Shift by 4 decimal places
	uint32_t Lon_32 = (uint32_t)Lon; // Convert to uint32_t
	if (negative) Lon_32 |= 0x200000; // Set the MS bit if Lon was negative (note: this is not two's complement)
	_txPayloadBytes[9] |= (Lon_32 >> 19) & 0x07; // Load 3 bits of Lon into the payload
	_txPayloadBytes[10] = (Lon_32 >> 11) & 0xFF; // Load 8 bits of Lon into the payload
	_txPayloadBytes[11] = (Lon_32 >> 3) & 0xFF; // Load 8 bits of Lon into the payload
	_txPayloadBytes[12] = (Lon_32 << 5) & 0xE0; // Load 3 bits of Lon into the payload, pad with five stuff bits
	_txPayloadBytes[13] = 0x00; // Eight stuff bits
	_txPayloadBytes[14] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)
	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS2LatLon: left-justified payload is 0x"));
		for (uint16_t i = 0; i < 15; i++)
		{
			if (_txPayloadBytes[i] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(_txPayloadBytes[i], HEX);
		}
		_debugPort->println();
	}

	return setTxPayload();
}

// Set the Tx payload for a ARGOS 4 VLD message with 0 bits of user data
// The payload contains the 2-bit message length plus the 28-bit platform ID (left justified) plus 6 tail bits
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS4VLDshort(uint32_t platformID)
{
	// Tx length in bits
	_txPayloadBytes[0] = 0x00;
	_txPayloadBytes[1] = 0x00;
	_txPayloadBytes[2] = ARTIC_R2_PTT_A4_VLD_SHORT_NUM_MESSAGE_BITS;

	// The payload itself
	// _txPayloadBytes[3] = (ARTIC_R2_PTT_A4_VLD_MESSAGE_LENGTH_SHORT << 6) | ((platformID >> 22) & 0x3F); // Left justify the 28-bit platform ID
	// _txPayloadBytes[4] = (platformID >> 14) & 0xFF;
	// _txPayloadBytes[5] = (platformID >> 6) & 0xFF;
	// _txPayloadBytes[6] = (platformID << 2) & 0xFC; // Last 6 bits of the platform ID plus two tail bits
	// _txPayloadBytes[7] = 0x00; // Last four tail bits plus four stuff bits
	// _txPayloadBytes[8] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)
	_txPayloadBytes[3] = ((platformID >> 20) & 0xFF); // Left justify the 28-bit platform ID
	_txPayloadBytes[4] = (platformID >> 12) & 0xFF;
	_txPayloadBytes[5] = (platformID >> 4) & 0xFF;
	_txPayloadBytes[6] = (platformID << 4) & 0xF0; // Last 4 bits of the platform ID plus four tail bits
	_txPayloadBytes[7] = 0x00; // Last two tail bits plus six stuff bits
	_txPayloadBytes[8] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)

	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS4VLDshort: left-justified payload is 0x"));
		for (uint16_t i = 0; i < 9; i++)
		{
			if (_txPayloadBytes[i] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(_txPayloadBytes[i], HEX);
		}
		_debugPort->println();
	}

	return setTxPayload();
}

// Set the Tx payload for a ARGOS 4 VLD long message
// The message contains the GPS latitude and longitude in a compact form which ARGOS Web will understand.
// Please contact CLS / Woods Hole Group and ask them to apply the SPARKFUN_GPS template on ARGOS Web.
// The Latitude and Longitude will then be extracted, converted and displayed automatically when you view your data.
// The number of user bits is 56.
// Lat is encoded as 21 bits: the MSB is 0 for +ve latitude, 1 for -ve latitude (SOUTH); the unit is 0.0001 degrees. (Note: this is not two's complement!)
// Lon is encoded as 22 bits: the MSB is 0 for +ve longitude, 1 for -ve longitude (WEST); the unit is 0.0001 degrees. (Note: this is not two's complement!)
// Returns true if the payload was set successfully
boolean ARTIC_R2::setPayloadARGOS4VLDLatLon(uint32_t platformID, float Lat, float Lon)
{
	// Tx length in bits
	_txPayloadBytes[0] = 0x00;
	_txPayloadBytes[1] = 0x00;
	_txPayloadBytes[2] = ARTIC_R2_PTT_A4_VLD_LONG_NUM_MESSAGE_BITS;

	// The payload itself
	// _txPayloadBytes[3] = (ARTIC_R2_PTT_A4_VLD_MESSAGE_LENGTH_LONG << 6) | ((platformID >> 22) & 0x3F); // Left justify the 28-bit platform ID
	// _txPayloadBytes[4] = (platformID >> 14) & 0xFF;
	// _txPayloadBytes[5] = (platformID >> 6) & 0xFF;
	// _txPayloadBytes[6] = (platformID << 2) & 0xFC; // Last 6 bits of the platform ID plus two tail bits
	_txPayloadBytes[3] = (platformID >> 20) & 0xFF; // Left justify the 28-bit platform ID
	_txPayloadBytes[4] = (platformID >> 12) & 0xFF;
	_txPayloadBytes[5] = (platformID >> 4) & 0xFF;
	_txPayloadBytes[6] = (platformID << 4) & 0xF0; // Last 4 bits of the platform ID plus four tail bits

	boolean negative = false;
	if (Lat < 0.0)
	{
		negative = true; // Is the Lat negative?
		Lat = 0.0 - Lat; // Make it +ve
	}
	Lat *= 10000.0; // Shift by 4 decimal places
	uint32_t Lat_32 = (uint32_t)Lat; // Convert to uint32_t
	if (negative) Lat_32 |= 0x100000; // Set the MS bit if Lat was negative (note: this is not two's complement)
	// _txPayloadBytes[7] = (Lat_32 >> 17) & 0x0F; // Four tail bits plus 4 bits of Lat
	// _txPayloadBytes[8] = (Lat_32 >> 9) & 0xFF; // Load 8 bits of Lat into the payload
	// _txPayloadBytes[9] = (Lat_32 >> 1) & 0xFF; // Load 8 bits of Lat into the payload
	// _txPayloadBytes[10] = (Lat_32 << 7) & 0x80; // Load 1 bit of Lat into the payload
	_txPayloadBytes[7] = (Lat_32 >> 15) & 0x3F; // Two tail bits plus 6 bits of Lat
	_txPayloadBytes[8] = (Lat_32 >> 7) & 0xFF; // Load 8 bits of Lat into the payload
	_txPayloadBytes[9] = (Lat_32 << 1) & 0xFE; // Load 7 bits of Lat into the payload

	negative = false;
	if (Lon < 0.0)
	{
		negative = true; // Is the Lon negative?
		Lon = 0.0 - Lon; // Make it +ve
	}
	Lon *= 10000.0; // Shift by 4 decimal places
	uint32_t Lon_32 = (uint32_t)Lon; // Convert to uint32_t
	//if (negative) Lon_32 |= 0x200000; // Set the MS bit if Lon was negative (note: this is not two's complement)
	if (negative) _txPayloadBytes[9] |= 0x01; // Set the MS bit if Lon was negative (note: this is not two's complement)
	// _txPayloadBytes[10] |= (Lon_32 >> 15) & 0x7F; // OR 7 bits of Lon into the payload
	// _txPayloadBytes[11] = (Lon_32 >> 13) & 0x03; // Six tail bits plus 2 bits of Lon
	// _txPayloadBytes[12] = (Lon_32 >> 5) & 0xFF; // Load 8 bits of Lon into the payload
	// _txPayloadBytes[13] = (Lon_32 << 3) & 0xF8; // Load 5 bits of Lon into the payload, pad with three stuff bits
	// _txPayloadBytes[14] = 0x00; // Eight stuff bits
	// _txPayloadBytes[15] = 0x00; // Two stuff bits plus six tail bits
	// _txPayloadBytes[16] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)
	// _txPayloadBytes[17] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)
	_txPayloadBytes[10] = (Lon_32 >> 15) & 0xFC; // Load 6 bits of Lon into the payload plus two tail bits
	_txPayloadBytes[11] = (Lon_32 >> 11) & 0x0F; // Four tail bits plus 4 bits of Lon
	_txPayloadBytes[12] = (Lon_32 >> 3) & 0xFF; // Load 8 bits of Lon into the payload
	_txPayloadBytes[13] = (Lon_32 << 5) & 0xE0; // Load 3 bits of Lon into the payload, pad with five stuff bits
	_txPayloadBytes[14] = 0x00; // Eight stuff bits
	_txPayloadBytes[15] = 0x00; // Six tail bits plus two stuff bits
	_txPayloadBytes[16] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)
	_txPayloadBytes[17] = 0x00; // Stuff buffer with zeros to a multiple of 24 bits (ARTIC requires this)
	if (_printDebug == true)
	{
		_debugPort->print(F("setPayloadARGOS4VLDLatLon: left-justified payload is 0x"));
		for (uint16_t i = 0; i < 18; i++)
		{
			if (_txPayloadBytes[i] < 0x10) _debugPort->print(F("0"));
			_debugPort->print(_txPayloadBytes[i], HEX);
		}
		_debugPort->println();
	}

	return setTxPayload();
}

// Set the Tx payload by copying _txPayloadBytes into X memory
// Returns true if the payload was copied successfully
// NOTE: The ARTIC datasheet indicates that TX Payload is write-only. So, strictly,
//   reading back the message length should not work. But it seems to work just fine...
boolean ARTIC_R2::setTxPayload()
{
	// Write _txPayloadBytes, starting at MEM_LOC_TX_PAYLOAD
	// To keep life simple, we write the entire ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_WORDS words
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_WRITE_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	uint8_t *bufferPtr = _txPayloadBytes;

	writeMultipleWords(bufferPtr, 24, ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_WORDS); // Write the words

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	// Calculate what the first memory location (message length in bits) should contain
	bufferPtr = _txPayloadBytes;
	uint32_t payloadLength = (((uint32_t)bufferPtr[0]) << 16) | (((uint32_t)bufferPtr[1]) << 8) | ((uint32_t)bufferPtr[2]);

	// Read the encoded message length back again

	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	uint8_t buffer[3]; // Buffer for the SPI data
	uint8_t *ptr = buffer; // Pointer to the buffer

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(ptr, 24, 1); // Read 1 24-bit word

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away

	uint32_t newPayloadLength = (((uint32_t)buffer[0]) << 16) | (((uint32_t)buffer[1]) << 8) | ((uint32_t)buffer[2]);

	// Check that the lengths match

	boolean result = (newPayloadLength == payloadLength);

	if (result == false)
	{
		if (_printDebug == true)
		{
			_debugPort->print(F("setTxPayload: failed! Expected payload length is "));
			_debugPort->print(payloadLength);
			_debugPort->print(F(". Value read back from MEM_LOC_TX_PAYLOAD is "));
			_debugPort->print(newPayloadLength);
			_debugPort->println(F("."));
		}
	}

 	return result;
}

// Read the Tx payload from XMEM back into _txPayloadBytes
// This overwrites _txPayloadBytes
void ARTIC_R2::readTxPayload()
{
	uint8_t *bufferPtr = _txPayloadBytes;

	// Read the Tx payload
	ARTIC_R2_Burstmode_Register burstmode; // Prepare the burstmode register configuration
	burstmode.BURSTMODE_REGISTER = 0x00000000; // Clear all unused bits
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_REG_SPI_ADDR = ARTIC_R2_BURSTMODE_REG_WRITE;
	burstmode.BURSTMODE_REGISTER_BITS.BURSTMODE_START_ADDR = MEM_LOC_TX_PAYLOAD;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_R_RW_MODE = ARTIC_R2_READ_BURST;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MEM_SEL = ARTIC_R2_X_MEMORY;
	burstmode.BURSTMODE_REGISTER_BITS.BURST_MODE_ON = 1;

	configureBurstmodeRegister(burstmode); // Configure the burstmode register

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles

	readMultipleWords(bufferPtr, 24, ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_WORDS); // Read the entire Tx payload

	delayMicroseconds(_delay24cycles); // Wait for 24 clock cycles - just in case we need to do another burst mode transfer straight away
}

// Pretty-print the Tx payload as 24-bit values
void ARTIC_R2::printTxPayload(Stream &port)
{
	uint8_t *bufferPtr = _txPayloadBytes;

	port.println(F("TX Payload:"));

	for (int i = 0; i < ARTIC_R2_TX_MAX_PAYLOAD_LENGTH_BYTES; i++)
	{
		if (i % 3 == 0)
			port.print(F("0x")); // Add the leading 0x
		if (bufferPtr[i] < 0x10)
			port.print(F("0")); // Add the leading 0
		port.print(bufferPtr[i], HEX); // Print the byte
		if (i % 3 == 2)
			port.print(F(" ")); // Add the trailing space
		if (i % 12 == 11)
			port.println(); // Add a linefeed every 8 words
	}
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
		for (int byteSize = 0; byteSize < (wordSizeInBits >> 3); byteSize++) // Divide wordSizeInBits by 8
		{
			_spiPort->transfer(buffer[bufferPointer++]);
		}

		// Delay between words
		delayMicroseconds(ARTIC_R2_BURST_INTER_WORD_DELAY_US);

		// Extra delay between blocks
		if (word % ARTIC_R2_BURST_BLOCK_SIZE == 0)
			delay(ARTIC_R2_BURST_INTER_BLOCK_DELAY_MS);
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
// *** INT1 will go high again after 100us if there is another message to be read (which could cause clearInterrupts to return false) ***
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

// Calling invertPWNENpin() or invertPWNENpin(true) will invert the PWR_EN pin for the Arribada Horizon
// Call invertPWNENpin(false) to change it back to the default for the SparkFun ARTIC R2 Breakout
void ARTIC_R2::invertPWNENpin(boolean invert)
{
	_invertedPWREN = invert;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Satellite Pass Prediction - taken from Arribada Horizon
// Thank you Arribada!

// File was originally called: prepas.c

/*    *******************************************************    */
/*    *******************************************************    */
/*               Copyright (c) 1995 CLS                          */
/*    All rights reserved including the right of reproduction    */
/*    in whole or in part in any form.                           */
/*    *******************************************************    */
/*    *******************************************************    */

/* +-------------------------------------------------------------------+*/
/* +                      variables externes                           +*/
/* +-------------------------------------------------------------------+*/


/*Tables des nombres de jours ecoules avant chaque mois de l'annee  */
/*    ek_quant (1...12,1) pour les annees bissextiles           */
/*    ek_quant (1...12,2) pour les annees non bissextiles       */
/*    ---------------------------------------------------------------   */

// WARN: number_sat must be greater than 0
// October 21st 2020: Paul added max_npass as a parameter (default is 1). This is to allow prediction of the next pass
// when calculating the prediction for a single satellite at a time which is close to the previous pass.
uint32_t ARTIC_R2::predictNextSatellitePass(bulletin_data_t *bulletin, float min_elevation, const uint8_t number_sat, float lon, float lat, long current_time, int max_npass)
{
	// if (_printDebug == true)
	// 	_debugPort->println("predictNextSatellitePass: start");

  configurationParameters tab_PC[1];              /* array of configuration parameter */
  orbitParameters tab_PO[ARTIC_R2_MAX_NUM_SATS];
  predictionParameters tab_PP[ARTIC_R2_MAX_NUM_SATS];         /* array of result */
  tab_PC[0].pf_lon = lon;
  tab_PC[0].pf_lat = lat;

  tab_PC[0].time_start = current_time;
  tab_PC[0].time_end = current_time + SECONDS_IN_DAY;
  tab_PC[0].s_differe = 0;

  tab_PC[0].site_min_requis = min_elevation;
  tab_PC[0].site_max_requis = 90.0f;

  tab_PC[0].marge_temporelle = 0;
  tab_PC[0].marge_geog_lat = 0;
  tab_PC[0].marge_geog_lon = 0;

  tab_PC[0].Npass_max = max_npass;

  orbitParameters  *pt_po;          /* pointer on tab_po            */
  configurationParameters  *pt_pc;  /* pointer on tab_pc            */
  predictionParameters  *pt_pp;     /* pointer on tab_pp            */

	// if (_printDebug == true)
	// 	_debugPort->println("predictNextSatellitePass: copying bulletin[] into tab_PO[]");

  for (int i = 0; i < number_sat; ++i)
  {
      tab_PO[i].sat[0] = bulletin[i].sat[0];
			tab_PO[i].sat[1] = bulletin[i].sat[1];
      tab_PO[i].time_bul = bulletin[i].time_bulletin;
      tab_PO[i].dga = bulletin[i].params[0];
      tab_PO[i].inc = bulletin[i].params[1];
      tab_PO[i].lon_asc = bulletin[i].params[2];
      tab_PO[i].d_noeud = bulletin[i].params[3];
      tab_PO[i].ts = bulletin[i].params[4];
      tab_PO[i].dgap = bulletin[i].params[5];
  }

  pt_pc  = &tab_PC[0];
  pt_po  = &tab_PO[0];
  pt_pp  = &tab_PP[0];

	// if (_printDebug == true)
	// 	_debugPort->println("predictNextSatellitePass: calling print_config and print_sat");

  print_config(pt_pc);
  print_sat(pt_po, number_sat);

	// if (_printDebug == true)
	// 	_debugPort->println("predictNextSatellitePass: calling satellitePassPrediction");

  satellitePassPrediction(pt_pc, pt_po, pt_pp, number_sat);

	// if (_printDebug == true)
	// 	_debugPort->println("predictNextSatellitePass: calling print_list");

  print_list(pt_pp, number_sat);

	// if (_printDebug == true)
	// 	_debugPort->println("predictNextSatellitePass: calling select_closest");

	// October 21st 2020: Paul changed select_closest so it will return -1 if there is no closest satellite.
	// This helps when calculating the prediction for a single satellite at a time which is close to the previous pass.
	// Previously the function would have returned a valid index of zero if the single prediction was in the past.
  int index = select_closest(pt_pp, number_sat, current_time);
	if (index < 0) return 0; // If there is no closest satellite, return a next pass time of zero.

  return (pt_pp[index].tpp + (pt_pp[index].duree / 2));
}

int ARTIC_R2::satellitePassPrediction(configurationParameters * p_pc, orbitParameters * p_po, predictionParameters * p_pp, int number_sat)
{
    /* -----------------------------------------------------------------
    C
    C   Satellite passes prediction
    C   Circular method taking into account drag coefficient (friction effect)
    C   and J2 term of earth potential
    C
    C       ephemeris calculation
    C
    C
    C   Author  : J.P. Malarde
    C   Company : C.L.S.
    C   Issue   : 1.0   25/09/97    first issue
    C             2.0   10/02/14    Sharc version
    C
    C -----------------------------------------------------------------*/

    long tpp;
    int duree;
    int duree_passage_MC; /* duree du passage Marges Comprises */

    long s_deb;      /* beginning of prediction (sec) */
    long s_fin;      /* end of prediction (sec) */
    long k;          /* number of revolution */
    long s_bul;      /* bulletin epoch (sec) */
    long t0;     /* date (sec) */
    long t1;     /* date (sec) */
    long t2;     /* date (sec) */
    long tmil[ARTIC_R2_MAX_NUM_SATS];    /* date milieu du prochain passage/satellite */

    int isat;
    int step = 0;

    int site;       /* min site */
    int passage;
    float d2;
    float d2_min;
    float d2_mem;
    float d2_mem_mem;
    float temp;
    float v;

    int duree_passage;
    long date_milieu_passage;
//long date_debut_passage;
    int site_max_passage;

    int site_max;   /* min site */
    int marge;
    int table_site_max[ARTIC_R2_MAX_NUM_SATS];                  /* DB */
    int table_duree[ARTIC_R2_MAX_NUM_SATS];                     /* DB */
    float   delta_lon;  /* asc.node drift during one revolution (deg) */
    float   wt;     /* earth rotation */

    float   visi_min;       /* visibility at site min*/
    float   visi_max;       /* visibility at site_max */

    float   ws0;        /* mean anomaly */
    float   d_ws;       /* friction effect */
    float   ws;     /* mean anomaly with friction effect */
    float   sin_i, cos_i;   /* sin, cos of orbit inclination */
    float   asc_node;   /* longitude of ascending node in terrestrial frame */
    float   x_pf;       /* beacon position */
    float   y_pf;
    float   z_pf;

    float   v_lon;
    float   v_lat;
    int     v_differe;
    float   v_site_max_requis;
    float   v_ts;
    float   v_marge_temporelle;
    float   v_marge_geog_lat;
    float   v_marge_geog_lon;
    float   v_dgap;
    int     Npass;      /* nombre de passages */
    int     Npass_max;  /* nombre de passages max par satellite */

    float   v_site_min_requis;

    memset(tmil,        0, sizeof(tmil));
    memset(table_site_max,  0, sizeof(table_site_max));
    memset(table_duree, 0, sizeof(table_duree));

    /* ...  EEPROM --> RAM transfert    */
    /*  ------------------------    */

    v_lon           = p_pc[0].pf_lon;
    v_lat           = p_pc[0].pf_lat;
    v_differe       = p_pc[0].s_differe;
    v_site_max_requis   = p_pc[0].site_max_requis;
    v_site_min_requis   = p_pc[0].site_min_requis;
    v_marge_temporelle  = p_pc[0].marge_temporelle;
    v_marge_geog_lat        = p_pc[0].marge_geog_lat;
    v_marge_geog_lon        = p_pc[0].marge_geog_lon;
    Npass_max = p_pc[0].Npass_max;

    /* ...  input parameter conversion  */
    /*  --------------------------- */

    v_lon = v_lon * deg_rad;
    v_lat = v_lat * deg_rad;

    x_pf = cos (v_lat) * cos (v_lon);
    y_pf = cos (v_lat) * sin (v_lon);
    z_pf = sin (v_lat);


    s_deb = p_pc[0].time_start - TIME_CONVERTOR;
    s_fin = p_pc[0].time_end - TIME_CONVERTOR;


    v_marge_temporelle = v_marge_temporelle + 5.;
    v_marge_temporelle = v_marge_temporelle / 259200.;

    v_marge_geog_lat = v_marge_geog_lat * 7.857142;
    v_marge_geog_lon = 0;

    /* ...  reading of OP */

    isat = 1;
//  t_sel = 999999999;

    //while (((isat-1) < 8) && (strncmp(p_po[isat-1].sat, "  ",2)!=0 )) {
    for (isat = 1; isat <= number_sat; isat ++)
    {
				//float radius = rs; // Orbit radius
				//if (p_po[isat - 1].sat[0] == 'A') radius = rs_a; // Added by Paul to improve max elevation calculations for ANGELS

				// Or, even better...
				float radius = p_po[isat - 1].dga;

				// Moved by Paul so the correct radius is used to calculate visi_min and _max
				visi_min = v_site_min_requis * deg_rad;
		    visi_min = demi_pi - visi_min - asin(rt / radius * cos(visi_min));
		    visi_min = 2. * sin(visi_min / 2.);
		    visi_min = visi_min * visi_min;

				visi_max = v_site_max_requis * deg_rad;
		    visi_max = demi_pi - visi_max - asin(rt / radius * cos(visi_max));
		    visi_max = 2. * sin(visi_max / 2.);
		    visi_max = visi_max * visi_max;


        p_pp[isat - 1].tpp = 0;
        sin_i    = sin(p_po[isat - 1].inc * deg_rad);
        cos_i    = cos(p_po[isat - 1].inc * deg_rad);

        v_dgap     = p_po[isat - 1].dgap / 1000;    /*  conversion m/jr --> km/jr */


        s_bul = p_po[isat - 1].time_bul - TIME_CONVERTOR;
        v_ts       = p_po[isat - 1].ts * 60.;
        ws0      = two_pi / v_ts;                   /* tour/sec */
        delta_lon = p_po[isat - 1].d_noeud * deg_rad;       /* distance between 2 asc nodes */
        wt       = delta_lon / v_ts;            /* tour/secondes */
        asc_node = p_po[isat - 1].lon_asc * deg_rad;

        /* ... recherche du prochain passage dont le site max > site max requis */

        Npass = 0;
        passage = 0;
        site = 0;
        duree = 0;
        site_max = 0;
        d2_min = 999.;
        t1 = s_deb - s_bul - (30 * 60);
        t2 = s_fin - s_bul;

        t0 = t1;
        k = (long)(t1 / v_ts);
        d_ws = -.75 * (v_dgap / p_po[isat - 1].dga / 86400.) * two_pi * k;
        ws = ws0 + d_ws;

        /* ... on saute le passage courrant */

        su_distance(    t1,
                        x_pf,
                        y_pf,
                        z_pf,
                        ws,
                        sin_i,
                        cos_i,
                        asc_node,
                        wt,
                        &d2);

        d2_mem     = d2;

        while ( (t1 < t2) && (Npass < Npass_max)  )
        {

            d2_mem_mem = d2_mem;
            d2_mem     = d2;

            su_distance(    t1,
                            x_pf,
                            y_pf,
                            z_pf,
                            ws,
                            sin_i,
                            cos_i,
                            asc_node,
                            wt,
                            &d2);

            if (d2 < visi_min)
            {

                passage = 1;

//           printf("********* passage en visibilite ***********\n");

                duree = duree + pas;
                v = 2.*asin(sqrt(d2) / 2.);
                temp =  (radius * sin(v)) / sqrt(rt * rt + radius * radius - 2 * rt * radius * cos(v));
                temp =  rad_deg * acos(temp);
                site = (int) (temp);
                if (site > site_max) site_max = site;
                if (d2 < d2_min)     d2_min = d2;

                step = pas;
            }
            else
            {

//           printf("********* passage hors visibilite ******");

                if (passage == 1)
                {

//              printf("****** on sort juste d'une visibilite ***********\n");

                    if (site_max < v_site_max_requis)
                    {

                        // on memorise lorsque l'on quitte la visibilit
                        duree_passage = duree;
                        date_milieu_passage = s_bul + t1 - duree / 2;
//                  date_debut_passage = date_milieu_passage - duree/2;
                        site_max_passage = site_max;

                        Npass += 1;


                        /* ...  Donnees en sortie :                 */
                        /*  - date de debut du prochain passage (sec)       */
                        /*  - duree du prochain passage, marge comprises (sec)  */

                        temp = (int)(v_marge_temporelle * t1 + v_marge_geog_lat + v_marge_geog_lon);
                        marge = (int)(temp);
                        tpp = date_milieu_passage - duree_passage / 2 - marge;
                        duree_passage_MC = duree_passage + 2 * marge;


                        strcpy(p_pp[isat - 1].sat, p_po[isat - 1].sat);
                        p_pp[isat - 1].tpp      = tpp + TIME_CONVERTOR;
                        p_pp[isat - 1].duree    = duree_passage_MC;
                        p_pp[isat - 1].site_max = site_max_passage;
                        passage = 2;

                    }
                    else
                    {

//                 printf("********* site max > site max requis ***********\n");

                        passage = 0;

                    }

                    site_max = 0;
                    duree = 0;
                    step = 30 * 60;        // 75 min
                    d2_min = 999999.;

                }
                else
                {

//              printf("********* on ne sort pas juste d'une visibilite ***********\n");

                    /* on ajuste le pas en fonction de la distance */

                    if ( d2 < ( 4 * visi_min))          step = pas;

                    if ((d2 >= ( 4 * visi_min)) &&
                        (d2 < (16 * visi_min)) )   step = 4 * pas;

                    if ((d2 >= (16 * visi_min))  &&
                         (d2 <= (32 * visi_min)) )    step = 8 * pas;

                     if ( d2 > (32 * visi_min))  step = 16 * pas;

                }

            }

            t1 = t1 + step;

        }       /* END WHILE sur le temps   */

    } /* lecture de chaque bulletin de satellite */

    return 0;
}


void ARTIC_R2::su_distance(   long    t1,     /* input */

                    float   x_pf,
                    float   y_pf,
                    float   z_pf,
                    float   ws,
                    float   sin_i,
                    float   cos_i,
                    float   asc_node,
                    float   wt,

                    float   *d2)        /* output */

{
    float   lat_sat;    /*  latitude of the satellite between the a.n  */
    float   long_sat_an;    /*  longitude of the satellite between the a.n */
    float   long_sat;   /*  longitude of the satellite          */
    float   x, y, z;        /*  satellite position              */

    /* ...  calculation of the satellite latitude */

    lat_sat =  asin( sin(ws * (float)(t1)) * sin_i );

    /* ...  calculation of the satellite longitude */

    long_sat_an = atan( tan(ws * (float)(t1)) * cos_i);


    /*
        printf(" pi %lf \n", pi);
    */

    if (cos(ws * t1) < 0.)
    {
        long_sat_an = long_sat_an + pi;
    }

    long_sat = asc_node + long_sat_an + wt * (float)(t1);
    long_sat = fmod(long_sat, two_pi);

    if (long_sat < 0.)
    {
        long_sat = long_sat + two_pi;
    }

    /* ...  spheric satellite positions calculation in TR */

    x = cos (lat_sat) * cos (long_sat);
    y = cos (lat_sat) * sin (long_sat);
    z = sin (lat_sat);

    *d2 = (x - x_pf) * (x - x_pf) +
          (y - y_pf) * (y - y_pf) +
          (z - z_pf) * (z - z_pf) ;

}

// October 21st 2020: Paul changed select_closest so it will return -1 if there is no closest satellite.
// This helps when calculating the prediction for a single satellite at a time which is close to the previous pass.
// Previously the function would have returned a valid index of zero if the single prediction was in the past.
int ARTIC_R2::select_closest(predictionParameters *pt_pp, int number_sat, uint32_t desired_time)
{
    uint32_t min = 0xFFFFFFFF;
    int index = -1; // Return -1 if no satellites are closest
    uint32_t current = 0;

    for (int i = 0; i < number_sat; ++i)
    {
        current = pt_pp[i].tpp + (pt_pp[i].duree / 2);

        if ((current > desired_time) && ((current - desired_time) < min) )
        {
            min = (current - desired_time);
            index = i;
        }
    }

		if (_printDebug == true)
		{
			if (index >= 0)
			{
	    	_debugPort->print("SAT SELECTED: ");
				_debugPort->write(pt_pp[index].sat[0]);
				_debugPort->write(pt_pp[index].sat[1]);
				_debugPort->print("\tINDEX: ");
				_debugPort->print(index);
				_debugPort->print("\tDESIRED TIME: ");
				_debugPort->print(desired_time);
				_debugPort->print("\tNEXT PASS: ");
				_debugPort->println((pt_pp[index].tpp + (pt_pp[index].duree / 2)));
			}
			else
			{
				_debugPort->println("SAT SELECTED: *** NONE ***");
			}
		}

    return index;
}

void ARTIC_R2::print_list(predictionParameters * p_pp,  int number_sat)
{
	if (_printDebug == true)
	{
		_debugPort->print("\r\n------------PREDICTION PARAMETERS-----------\r\n\r\n");
    for (int i = 0; i < number_sat; ++i)
    {
        _debugPort->print("sat ");
				_debugPort->write(p_pp[i].sat[0]);
				_debugPort->write(p_pp[i].sat[1]);
				_debugPort->print(" duration (min) ");
				_debugPort->print((p_pp[i].duree) / 60);
				_debugPort->print(":");
				_debugPort->print((p_pp[i].duree) % 60);
				_debugPort->print("\tsite max elevation (deg) ");
				_debugPort->print(p_pp[i].site_max);
				_debugPort->print(" time of next pass ");
				_debugPort->print(p_pp[i].tpp);
				_debugPort->print(" middle ");
				_debugPort->print((p_pp[i].tpp + (p_pp[i].duree / 2))); // from 1990 to 1970
				_debugPort->print(" ");
				time_t pt_of_day = (p_pp[i].tpp + (p_pp[i].duree / 2)); // Convert to YY/MM/DD HH:MM:SS
			  tm* pt = gmtime(&pt_of_day);
			  _debugPort->print(pt->tm_mday);
			  _debugPort->print(F("/"));
			  _debugPort->print(pt->tm_mon + 1); // Jan = 1
			  _debugPort->print(F("/"));
			  _debugPort->print(pt->tm_year + 1900);
			  _debugPort->print(F(" "));
			  _debugPort->print(pt->tm_hour);
			  _debugPort->print(F(":"));
			  _debugPort->print(pt->tm_min);
			  _debugPort->print(F(":"));
			  _debugPort->println(pt->tm_sec);
    }
	}
}

void ARTIC_R2::print_config(configurationParameters *p_pc)
{
	if (_printDebug == true)
	{
		_debugPort->print("\r\n----------CONFIGURATION PARAMETERS----------\r\n\r\n");
		_debugPort->print("Latitude ");
		_debugPort->print(p_pc[0].pf_lat, 4);
    _debugPort->print("\tLongitude ");
		_debugPort->println(p_pc[0].pf_lon, 4);
    _debugPort->print("START: time ");
		_debugPort->print(p_pc[0].time_start);
    _debugPort->print(" END: time ");
		_debugPort->print(p_pc[0].time_end);
    _debugPort->print(" time difference ");
		_debugPort->print(p_pc[0].s_differe);
    _debugPort->print(" site min elevation ");
		_debugPort->print(p_pc[0].site_min_requis);
    _debugPort->print(" site max elevation ");
		_debugPort->print(p_pc[0].site_max_requis);
    _debugPort->print(" temporal margin ");
		_debugPort->print(p_pc[0].marge_temporelle);
    _debugPort->print(" geog lat margin ");
		_debugPort->print(p_pc[0].marge_geog_lat);
    _debugPort->print(" geog lon margin ");
		_debugPort->print(p_pc[0].marge_geog_lon);
    _debugPort->print(" maximum number of passes ");
		_debugPort->println(p_pc[0].Npass_max);
	}
}

void ARTIC_R2::print_sat(orbitParameters *p_po, int number_sat)
{
	if (_printDebug == true)
	{
		_debugPort->print("\r\n--------------ORBIT PARAMETERS--------------\r\n\r\n");
    for (int i = 0; i < number_sat; ++i)
    {
        _debugPort->print("NAME: ");
				_debugPort->write(p_po[i].sat[0]);
				_debugPort->write(p_po[i].sat[1]);
        _debugPort->print("\tSTART: time ");
				_debugPort->println(p_po[i].time_bul);
        _debugPort->print("VALUES: Semi-major axis ");
				_debugPort->print(p_po[i].dga, 3);
				_debugPort->print(" Orbit inclination ");
				_debugPort->print(p_po[i].inc, 4);
				_debugPort->print(" Ascending node longitude ");
				_debugPort->print(p_po[i].lon_asc, 3);
				_debugPort->print(" Ascending node longitudinal drift ");
				_debugPort->print(p_po[i].d_noeud, 3);
				_debugPort->print(" Orbital period ");
				_debugPort->print(p_po[i].ts, 4);
				_debugPort->print(" Semi-major axis drift ");
				_debugPort->println(p_po[i].dgap, 2);
    }
	}
}

// Convert the AOP from text to bulletin_data_t
boolean ARTIC_R2::convertAOPtoParameters(const char *AOP, bulletin_data_t *satelliteParameters, const uint8_t number_sat)
{
	boolean result = true;
	struct tm t;
  time_t t_of_day;

	uint32_t offsetAOP; // Offset into the AOP
	for (uint8_t i = 0; i < number_sat; i++)
	{
		offsetAOP = i * ARTIC_R2_AOP_ENTRY_WIDTH;
		// Extract the satellite identifier
		satelliteParameters[i].sat[0] = AOP[offsetAOP + 1];
		satelliteParameters[i].sat[1] = AOP[offsetAOP + 2];
		// Extract the date (epoch) of the AOP
		t.tm_year = ((AOP[offsetAOP + 12] - 0x30) * 1000) + ((AOP[offsetAOP + 13] - 0x30) * 100) + ((AOP[offsetAOP + 14] - 0x30) * 10) + ((AOP[offsetAOP + 15] - 0x30)) - 1900;  // Year - 1900
	  t.tm_mon = ((AOP[offsetAOP + 17] == 0x20 ? 0 : (AOP[offsetAOP + 17] - 0x30)) * 10) + (AOP[offsetAOP + 18] - 0x30) - 1; // Month, *** where 0 = jan ***
	  t.tm_mday = ((AOP[offsetAOP + 20] == 0x20 ? 0 : (AOP[offsetAOP + 20] - 0x30)) * 10) + (AOP[offsetAOP + 21] - 0x30); // Day of the month
	  t.tm_hour = ((AOP[offsetAOP + 23] == 0x20 ? 0 : (AOP[offsetAOP + 23] - 0x30)) * 10) + (AOP[offsetAOP + 24] - 0x30);
	  t.tm_min = ((AOP[offsetAOP + 26] == 0x20 ? 0 : (AOP[offsetAOP + 26] - 0x30)) * 10) + (AOP[offsetAOP + 27] - 0x30);
	  t.tm_sec = ((AOP[offsetAOP + 29] == 0x20 ? 0 : (AOP[offsetAOP + 29] - 0x30)) * 10) + (AOP[offsetAOP + 30] - 0x30);
	  t.tm_isdst = 0;         // Is DST on? 1 = yes, 0 = no, -1 = unknown
	  t_of_day = mktime(&t);
		satelliteParameters[i].time_bulletin = (long)t_of_day;
		// Extract the six parameters
		satelliteParameters[i].params[0] = textToFloat(&AOP[offsetAOP + 32], 5, 3); // Semi-major axis (km)
		satelliteParameters[i].params[1] = textToFloat(&AOP[offsetAOP + 42], 3, 4); // Orbit inclination (deg)
		satelliteParameters[i].params[2] = textToFloat(&AOP[offsetAOP + 51], 4, 3); // Ascending node longitude (deg)
		satelliteParameters[i].params[3] = textToFloat(&AOP[offsetAOP + 60], 4, 3); // Ascending node longitudinal drift (deg)
		satelliteParameters[i].params[4] = textToFloat(&AOP[offsetAOP + 69], 4, 4); // Orbital period (min)
		satelliteParameters[i].params[5] = textToFloat(&AOP[offsetAOP + 79], 3, 2); // Semi-major axis drift (m/day)
		// Basic syntax checking - just to make sure nothing bad has happened and that the AOP appears to be correctly formatted
		// Check that the first digit of the satellite identifier is M or 1 or S or A
		if ((satelliteParameters[i].sat[0] != 'M') && (satelliteParameters[i].sat[0] != '1') && (satelliteParameters[i].sat[0] != 'S') && (satelliteParameters[i].sat[0] != 'A'))
		{
			if (_printDebug == true)
				_debugPort->print("convertAOPtoParameters: invalid satellite identifier!");
			result = false;
		}
		// Check that the date (epoch) of the AOP is >= 2020/1/1 00:00:00 and <= 2040/1/1 00:00:00
		if ((satelliteParameters[i].time_bulletin < 1577836800) || (satelliteParameters[i].time_bulletin > 2208988800))
		{
			if (_printDebug == true)
				_debugPort->print("convertAOPtoParameters: AOP date (epoch) is invalid!");
			result = false;
		}
		// Check that the Semi-major axis is >= 6500 and <= 7500
		// (ANGELS is typically 6892km)
		if ((satelliteParameters[i].params[0] < 6500) || (satelliteParameters[i].params[0] > 7500))
		{
			if (_printDebug == true)
				_debugPort->print("convertAOPtoParameters: semi-major axis is invalid!");
			result = false;
		}
	}

	return (result);
}

// Convert AOP text to float
float ARTIC_R2::textToFloat(const char *ptr, uint8_t digitsBeforeDP, uint8_t digitsAfterDP)
{
	float result = 0; // The result
	boolean negative = false; // Is the number negative?
	float multiplier = 1; // Multiplier (power of ten)
	for (uint8_t i = digitsBeforeDP; i > 1; i--) multiplier *= 10; // Calculate the multiplier for the first digit
	for (uint8_t offset = 0; offset < (digitsBeforeDP + digitsAfterDP + 1); offset++)
	{
		if (offset == digitsBeforeDP) continue; // Skip the DP. Don't decrease the multiplier.
		else if (ptr[offset] == 0x20); // Skip spaces
		else if (ptr[offset] == 0x2D) negative = true; // Check for minus sign
		else result += (ptr[offset] - 0x30) * multiplier; // Add the digit
		multiplier /= 10;
	}
	if (negative)
		return (0 - result);
	else
		return (result);
}

// Convert epoch to date & time string
char* const ARTIC_R2::convertEpochToDateTime(uint32_t epoch)
{
	static char dateTime[20]; // Storage for the date & time
	time_t pt_of_day = epoch; // Convert to YY/MM/DD HH:MM:SS
	tm* pt = gmtime(&pt_of_day);
	sprintf(dateTime, "%02d/%02d/%04d %02d:%02d:%02d", pt->tm_mday, pt->tm_mon + 1, pt->tm_year + 1900, pt->tm_hour, pt->tm_min, pt->tm_sec);
	return dateTime;
}

// Convert epoch to AOP format date & time string
char* const ARTIC_R2::convertEpochToDateTimeAOP(uint32_t epoch)
{
	static char dateTime[20]; // Storage for the date & time
	time_t pt_of_day = epoch; // Convert to YY/MM/DD HH:MM:SS
	tm* pt = gmtime(&pt_of_day);
	sprintf(dateTime, "%4d %2d %2d %2d %2d %2d", pt->tm_year + 1900, pt->tm_mon + 1, pt->tm_mday, pt->tm_hour, pt->tm_min, pt->tm_sec);
	return dateTime;
}

// Convert GPS date & time to epoch
uint32_t ARTIC_R2::convertGPSTimeToEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
	struct tm t;
  time_t t_of_day;

  t.tm_year = year - 1900;  // Year - 1900
  t.tm_mon = month - 1;     // Month, where 0 = jan
  t.tm_mday = day;          // Day of the month
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  t.tm_isdst = 0;           // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);

  return ((uint32_t)t_of_day);
}

// Pretty-print the AOP bulletin
// Also prints using the same text format as the ARGOS Web AOP tool
// so the data can be cut and pasted into other code and processed using convertAOPtoParameters
// Returns true if the satellite ID is A1, MA, MB, MC, 15, 18, 19 or SR (i.e. will be included in the ARGOS Web AOP and has a known two character ID)
boolean ARTIC_R2::printAOPbulletin(bulletin_data_t bulletin, Stream &port)
{
	port.println(F("AOP Bulletin:"));
	port.println(F("============="));
	port.print(F("Satellite ID: "));
	port.write(bulletin.sat[0]);
	port.write(bulletin.sat[1]);
	port.println();
	port.print(F("Time of the bulletin: "));
	port.println(convertEpochToDateTime(bulletin.time_bulletin));
	port.print(F("Semi-major axis: "));
	port.print(bulletin.params[0], 3);
	port.println(F(" km"));
	port.print(F("Orbit inclination: "));
	port.print(bulletin.params[1], 4);
	port.println(F(" deg"));
	port.print(F("Ascending node longitude: "));
	port.print(bulletin.params[2], 3);
	port.println(F(" deg"));
	port.print(F("Ascending node longitudinal drift: "));
	port.print(bulletin.params[3], 3);
	port.println(F(" deg"));
	port.print(F("Orbital period: "));
	port.print(bulletin.params[4], 4);
	port.println(F(" min"));
	port.print(F("Semi-major axis drift: "));
	port.print(bulletin.params[5], 2);
	port.println(F(" m/day"));
	// Print again in AOP text format
	port.print(F("\" "));
	port.write(bulletin.sat[0]);
	port.write(bulletin.sat[1]);
	port.print(F(" # # # # "));
	port.print(convertEpochToDateTimeAOP(bulletin.time_bulletin));
	port.print(F("  "));
	port.print(bulletin.params[0], 3);
	port.print(F(" "));
	if (bulletin.params[1] < 100.0) port.print(F(" "));
	port.print(bulletin.params[1], 4);
	port.print(F("  "));
	if (bulletin.params[2] < 100.0) port.print(F(" "));
	if (bulletin.params[2] < 10.0) port.print(F(" "));
	port.print(bulletin.params[2], 3);
	port.print(F("  "));
	port.print(bulletin.params[3], 3);
	port.print(F("  "));
	if (bulletin.params[4] < 100.0) port.print(F(" "));
	port.print(bulletin.params[4], 4);
	port.print(F(" "));
	if (bulletin.params[5] > -10.0) port.print(F(" "));
	if (bulletin.params[5] >= 0.0) port.print(F(" "));
	port.print(bulletin.params[5], 2);
	port.println(F("\""));

	if (((bulletin.sat[0] == 'M') && (bulletin.sat[1] == 'A')) ||
		((bulletin.sat[0] == 'M') && (bulletin.sat[1] == 'B')) ||
		((bulletin.sat[0] == 'M') && (bulletin.sat[1] == 'C')) ||
		((bulletin.sat[0] == '1') && (bulletin.sat[1] == '5')) ||
		((bulletin.sat[0] == '1') && (bulletin.sat[1] == '8')) ||
		((bulletin.sat[0] == '1') && (bulletin.sat[1] == '9')) ||
		((bulletin.sat[0] == 'A') && (bulletin.sat[1] == '1')) ||
		((bulletin.sat[0] == 'S') && (bulletin.sat[1] == 'R')))
		return true;
	else
		return false;
}
