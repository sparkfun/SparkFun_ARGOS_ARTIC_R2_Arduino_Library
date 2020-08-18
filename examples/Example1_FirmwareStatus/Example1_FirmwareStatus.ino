/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: August 18th 2020

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  The ARTIC firmware takes up 127KB of program memory!

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

  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin, GAIN16_Pin) == false)
  {
    Serial.println("ARTIC R2 not detected. Freezing...");
    while (1);
  }

  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register

  Serial.println(F("ARTIC R2 Firmware Status:"));
  if (status.STATUS_REGISTER_BITS.IDLE) Serial.println(F("IDLE"));
  if (status.STATUS_REGISTER_BITS.RX_IN_PROGRESS) Serial.println(F("RX_IN_PROGRESS"));
  if (status.STATUS_REGISTER_BITS.TX_IN_PROGRESS) Serial.println(F("TX_IN_PROGRESS"));
  if (status.STATUS_REGISTER_BITS.BUSY) Serial.println(F("BUSY"));
  if (status.STATUS_REGISTER_BITS.RX_VALID_MESSAGE) Serial.println(F("RX_VALID_MESSAGE"));
  if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) Serial.println(F("RX_SATELLITE_DETECTED"));
  if (status.STATUS_REGISTER_BITS.TX_FINISHED) Serial.println(F("TX_FINISHED"));
  if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED) Serial.println(F("MCU_COMMAND_ACCEPTED"));
  if (status.STATUS_REGISTER_BITS.CRC_CALCULATED) Serial.println(F("CRC_CALCULATED"));
  if (status.STATUS_REGISTER_BITS.IDLE_STATE) Serial.println(F("IDLE_STATE"));
  if (status.STATUS_REGISTER_BITS.RX_CALIBRATION_FINISHED) Serial.println(F("RX_CALIBRATION_FINISHED"));
  if (status.STATUS_REGISTER_BITS.RX_TIMEOUT) Serial.println(F("RX_TIMEOUT"));
  if (status.STATUS_REGISTER_BITS.SATELLITE_TIMEOUT) Serial.println(F("SATELLITE_TIMEOUT"));
  if (status.STATUS_REGISTER_BITS.RX_BUFFER_OVERFLOW) Serial.println(F("RX_BUFFER_OVERFLOW"));
  if (status.STATUS_REGISTER_BITS.TX_INVALID_MESSAGE) Serial.println(F("TX_INVALID_MESSAGE"));
  if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED) Serial.println(F("MCU_COMMAND_REJECTED"));
  if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW) Serial.println(F("MCU_COMMAND_OVERFLOW"));
  if (status.STATUS_REGISTER_BITS.INTERNAL_ERROR) Serial.println(F("INTERNAL_ERROR"));
  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) Serial.println(F("DSP2MCU_INT1"));
  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) Serial.println(F("DSP2MCU_INT2"));
}

void loop()
{
  // Nothing to do here...
}
