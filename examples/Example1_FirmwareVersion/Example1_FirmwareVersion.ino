/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: September 29th 2020

  This example begins (initializes) the ARTIC breakout. The pin numbers are passed to begin.
  Begin takes take of setting the PWR_EN pin to enable power for the ARTIC.
  Begin also controls the BOOT pin and downloads the firmware to the ARTIC.
  Begin returns true if the firmware checksum is valid.

  The ARTIC firmware version is read and printed to Serial.

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  The ARTIC firmware takes up 127KB of program memory! Please choose a processor with memory to spare.

  Hardware Connections:
  This example assumes the ARTIC Breakout has been mounted on a SparkFun Thing Plus - Artemis:
  https://www.sparkfun.com/products/15574
  CS_Pin = A5 (D24)
  GAIN8_Pin = D3
  GAIN16_Pin = D4
  BOOT_Pin = D5
  INT1_Pin = D6
  INT2_Pin = D7
  RESET_Pin = D8
  PWR_EN_Pin = D9
  (SPI COPI = D11)
  (SPI CIPO = D12)
  (SPI SCK = D13)
*/

#include <SPI.h>

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

// Pin assignments for the SparkFun Thing Plus - Artemis
// (Change these if required)
uint8_t CS_Pin = 24;
uint8_t GAIN8_Pin = 3;
uint8_t GAIN16_Pin = 4;
uint8_t BOOT_Pin = 5;
uint8_t INT1_Pin = 6;
uint8_t INT2_Pin = 7;
uint8_t RESET_Pin = 8;
uint8_t PWR_EN_Pin = 9;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));

  SPI.begin();

  myARTIC.enableDebugging(); // Enable debug messages to Serial

  //myARTIC.enableDebugging(Serial1); // E.g. enable debug messages to Serial1 instead

  // Begin (initialize) the ARTIC
  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin, GAIN16_Pin) == false)
  {
    Serial.println(F("ARTIC R2 not detected. Freezing..."));
    while (1);
  }

  Serial.println(F("ARTIC R2 boot was successful."));

  char buffer[9]; // Buffer to store the firmware version (8 bytes + NULL)

  myARTIC.readFirmwareVersion(&buffer[0]); // Read the firmware version from PMEM

  Serial.print(F("ARTIC R2 firmware version is: "));
  Serial.println(buffer);
}

void loop()
{
  // Nothing to do here...
}
