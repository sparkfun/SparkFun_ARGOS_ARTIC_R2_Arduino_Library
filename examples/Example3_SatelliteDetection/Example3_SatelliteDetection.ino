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

  //myARTIC.enableDebugging(); // Enable debug messages to Serial

  // Begin the ARTIC: enable power and upload firmware or boot from flash
  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin, GAIN16_Pin) == false)
  {
    Serial.println("ARTIC R2 not detected. Freezing...");
    while (1)
      ; // Do nothing more
  }

  // Read and print the ARGOS configuration
  ARGOS_Configuration_Register configuration;
  myARTIC.readARGOSconfiguration(&configuration);
  if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_A2_MODE) Serial.println(F("TX mode is: ARGOS PTT A2"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_A3_MODE) Serial.println(F("TX mode is: ARGOS PTT A3"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_ZE_MODE) Serial.println(F("TX mode is: ARGOS PTT ZE"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_HD_MODE) Serial.println(F("TX mode is: ARGOS PTT HD"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_A4_MD_MODE) Serial.println(F("TX mode is: ARGOS PTT A4 MD"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_A4_HD_MODE) Serial.println(F("TX mode is: ARGOS PTT A4 HD"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.TX_CONFIGURATION == TX_CONFIG_ARGOS_PTT_A4_VLD_MODE) Serial.println(F("TX mode is: ARGOS PTT A4 VLD"));
  else Serial.println(F("TX mode is not defined!"));
  if (configuration.CONFIGURATION_REGISTER_BITS.RX_CONFIGURATION == RX_CONFIG_ARGOS_3_RX_MODE) Serial.println(F("RX mode is: ARGOS 3"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.RX_CONFIGURATION == RX_CONFIG_ARGOS_3_RX_BACKUP_MODE) Serial.println(F("RX mode is: ARGOS 3 BACKUP"));
  else if (configuration.CONFIGURATION_REGISTER_BITS.RX_CONFIGURATION == RX_CONFIG_ARGOS_4_RX_MODE) Serial.println(F("RX mode is: ARGOS 4"));
  else Serial.println(F("RX mode is not defined!"));

  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register

  if (!status.STATUS_REGISTER_BITS.IDLE) Serial.println(F("Warning! ARTIC is not idle! This is likely to fail..."));

  // Set the satellite detection timeout to 60 seconds
  myARTIC.setSatelliteDetectionTimeout(60);

  // Delay for 10ms
  delay(10);

  // Check that the MCU command was accepted
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED) Serial.println(F("MCU command was accepted"));
  else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED)
  {
    Serial.println(F("MCU command was rejected! Freezing..."));
    while (1)
      ; // Do nothing more
  }
  else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW)
  {
    Serial.println(F("MCU command overflow! Freezing..."));
    while (1)
      ; // Do nothing more
  }
  
  // Start satellite detection
  // The ARTIC will start looking for a satellite for a specified amount of time.
  // If no satellite is detected, INT 2 will be set with the ‘SATELLITE_TIMEOUT’ flag.
  // If a satellite was detected, by receiving 5 consecutive 0x7E flags, INT 1 will be set with the ‘RX_SATELLITE_DETECTED’ flag.
  myARTIC.sendCommandByte(INST_SATELLITE_DETECTION);

  // Delay for 10ms
  delay(10);

  // Check that the MCU command was accepted
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  if (status.STATUS_REGISTER_BITS.MCU_COMMAND_ACCEPTED) Serial.println(F("MCU command was accepted"));
  else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_REJECTED)
  {
    Serial.println(F("MCU command was rejected! Freezing..."));
    while (1)
      ; // Do nothing more
  }
  else if (status.STATUS_REGISTER_BITS.MCU_COMMAND_OVERFLOW)
  {
    Serial.println(F("MCU command overflow! Freezing..."));
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  delay(1000); // Query the ARTIC firmware status once per second
  
  Serial.println();

  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register

  if (status.STATUS_REGISTER_BITS.IDLE) // Check the IDLE flag
    Serial.println(F("ARTIC is IDLE"));
  else
    Serial.println(F("ARTIC is not IDLE"));

  if (status.STATUS_REGISTER_BITS.BUSY) // Check the BUSY flag
    Serial.println(F("ARTIC is BUSY"));
  else
    Serial.println(F("ARTIC is not BUSY"));

  if (status.STATUS_REGISTER_BITS.IDLE_STATE) // Check the IDLE_STATE flag
    Serial.println(F("ARTIC firmware has returned to the idle state"));
  else
    Serial.println(F("ARTIC firmware has not yet returned to the idle state"));

  if (status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED) // Check the RX_SATELLITE_DETECTED flag
  {
    Serial.println(F("A satellite was detected! Satellite detection complete..."));
    //while (1)
    //  ; // Do nothing more
  }

  if (status.STATUS_REGISTER_BITS.SATELLITE_TIMEOUT) // Check the SATELLITE_TIMEOUT flag
  {
    Serial.println(F("Satellite detection timed out! Satellite detection complete..."));
    //while (1)
    //  ; // Do nothing more
  }

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when a satellite is detected
    Serial.println(F("INT1 pin is high. Satellite detected!"));
  else
    Serial.println(F("INT1 pin is low."));

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // Check the interrupt 2 flag. This will go high if satellite detection times out
    Serial.println(F("INT2 pin is high. Satellite detection has timed out!"));
  else
    Serial.println(F("INT2 pin is low."));
}
