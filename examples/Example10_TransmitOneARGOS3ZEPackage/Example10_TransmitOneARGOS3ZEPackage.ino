/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: September 25th 2020

  This example:
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the satellite detection timeout to 60 seconds;
    sets the TX mode to ARGOS PTT-ZE;
    sets the TX frequency;
    instructs the ARTIC to Transmit One Package And Go Idle;
    keeps checking the MCU status until transmit is complete.

  The ARGOS 3 PTT-ZE message contains only the 28-bit platform ID.

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  This example requires a receiver address. Copy and paste your 48-bit receiver address into ADDRESS_LS_BITS and ADDRESS_MS_BITS

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

// CLS will have provided you with a Platform ID for your ARGOS R2. Copy and paste it into PLATFORM_ID below.
// E.g.: if your Platform ID is 01:23:AB:CD then set PLATFORM_ID to 0x0123ABCD
const uint32_t PLATFORM_ID = 0x00000000; // Update this with your Platform ID

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

  // Uncomment the next line to invert the PWR_EN pin if you are using the Arribada Horizon instead of the SparkFun ARTIC R2 Breakout
  // (Make sure you call .invertPWNENpin _before_ you call .begin !)
  myARTIC.invertPWNENpin();

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
  myARTIC.printARGOSconfiguration(configuration); // Pretty-print the TX and RX configuration to Serial
  
  //myARTIC.printARGOSconfiguration(configuration, Serial1); // E.g.: pretty-print the TX and RX configuration to Serial1 instead

  // Read and print the firmware status
  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
  
  //myARTIC.printFirmwareStatus(status, Serial1); // E.g.: pretty-print the firmware status to Serial1 instead

  // Set the satellite detection timeout to 60 seconds
  if (myARTIC.setSatelliteDetectionTimeout(60) == false)
  {
    Serial.println("setSatelliteDetectionTimeout failed. Freezing...");
    while (1)
      ; // Do nothing more
  }

  // Set the TX mode to ARGOS 3 PTT-ZE
  ARTIC_R2_MCU_Command_Result result = myARTIC.sendConfigurationCommand(CONFIG_CMD_SET_PTT_ZE_TX_MODE);
  myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
  if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
  {
    Serial.println("sendConfigurationCommand failed. Freezing...");
    while (1)
      ; // Do nothing more
  }

  // Read and print the ARGOS configuration
  myARTIC.readARGOSconfiguration(&configuration);
  myARTIC.printARGOSconfiguration(configuration);

  // Set the ARGOS 3 TX frequency to 401.68 MHz
  if (myARTIC.setARGOS23TxFrequency(401.68) == false)
  {
    Serial.println("setARGOS23TxFrequency failed. Freezing...");
    while (1)
      ; // Do nothing more
  }
  
  // Configure the Tx payload for ARGOS 3 PTT-ZE using our platform ID and 0 bits of user data
  if (myARTIC.setPayloadARGOS3ZE(PLATFORM_ID) == false)
  {
    Serial.println(F("setPayloadARGOS3ZE failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Start the ARTIC in Transmit One Package And Go Idle mode
  result = myARTIC.sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE);
  if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
  {
    Serial.println();
    Serial.println("<sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) failed>");
    Serial.println();
    myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
    Serial.println(F("ARTIC R2 Firmware Status:"));
    myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
    Serial.println();
    Serial.println(F("ARTIC_R2_MCU_Command_Result:"));
    myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
    Serial.println();
    Serial.println("</sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) failed> Freezing...");
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  delay(1000);

  Serial.println();

  // Read and print the ARTIC R2 status register
  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when TX is finished
  {
    Serial.println(F("INT1 pin is high. TX is finished (or MCU is in IDLE_STATE)!"));
  }

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // Check the interrupt 2 flag. This will go high when if the message was invalid
  {
    Serial.println(F("INT2 pin is high. TX message was invalid! (Something really bad must have happened... ARGOS 4 VLD is as simple as it gets!)"));
  }

  Serial.println();

  // Read and print the instruction progress
  ARTIC_R2_MCU_Instruction_Progress progress;
  // checkMCUinstructionProgress will return true if the instruction is complete
  boolean instructionComplete = myARTIC.checkMCUinstructionProgress(&progress); // Check the instruction progress
  Serial.println(F("ARTIC R2 instruction progress:"));
  myARTIC.printInstructionProgress(progress); // Pretty-print the progress to Serial

  if (instructionComplete)
  {
    Serial.println();
    Serial.println(F("Transmission is complete!"));
    Serial.println();
    Serial.println(F("We are done. Freezing..."));
    while (1)
      ; // Do nothing more
  }
}
