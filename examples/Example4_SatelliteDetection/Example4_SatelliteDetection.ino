/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: September 29th 2020

  This example:
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the satellite detection timeout to 600 seconds;
    starts satellite detection;
    keeps checking the MCU status until a satellite is detected or the detection times out.

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
  Serial.println();

  Serial.println(F("The ARTIC is booting. This will take approx. 12 seconds."));
  Serial.println();

  SPI.begin();

  myARTIC.enableDebugging(); // Enable debug messages to Serial

  // Begin the ARTIC: enable power and upload firmware or boot from flash
  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin, GAIN16_Pin) == false)
  {
    Serial.println("ARTIC R2 not detected. Freezing...");
    while (1)
      ; // Do nothing more
  }

  Serial.println(F("ARTIC R2 boot was successful."));
  Serial.println();

  unsigned long beginFinishedAt = millis(); // Keep a record of millis() when .begin finished

  Serial.println(F("Waiting for INT1 to go high... (This could take up to 5 minutes with ARTIC006 firmware!)"));
  Serial.println();

  ARTIC_R2_Firmware_Status status;

  do
  {
    myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  
    Serial.println(F("ARTIC R2 Firmware Status:"));
    myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
    Serial.println();
  
    Serial.print(F("It has been "));
    Serial.print((millis() - beginFinishedAt) / 1000);
    Serial.println(F(" seconds since the ARTIC was booted."));
    Serial.println();
    
    delay(5000);
  }
  while (status.STATUS_REGISTER_BITS.DSP2MCU_INT1 == false); // Check the interrupt 1 flag. This will go high when the RX offset calibration has completed.
  
  Serial.println(F("INT1 pin is high. ARTIC is ready!"));
  Serial.println();

  Serial.println(F("Clearing INT1."));
  Serial.println();

  // Clear INT1
  if (myARTIC.clearInterrupts(1) == false)
  {
    Serial.println("clearInterrupts failed. Freezing...");
    //while (1)
    //  ; // Do nothing more
  }  
  
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register  
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
  Serial.println();
  
  // Read and print the ARGOS configuration
  ARGOS_Configuration_Register configuration;
  myARTIC.readARGOSconfiguration(&configuration);
  myARTIC.printARGOSconfiguration(configuration); // Pretty-print the TX and RX configuration to Serial
  
  // Set the satellite detection timeout to 600 seconds
  if (myARTIC.setSatelliteDetectionTimeout(600) == false)
  {
    Serial.println("setSatelliteDetectionTimeout failed. Freezing...");
    while (1)
      ; // Do nothing more
  }

  // Get the satellite detection timeout
  uint32_t detectionTimeout = myARTIC.readSatelliteDetectionTimeout();
  Serial.print(F("The Satellite Detection Timeout is "));
  Serial.print(detectionTimeout);
  Serial.println(F(" seconds."));

  Serial.println();
  Serial.println(F("Starting satellite detection..."));
  Serial.println();

  // Start satellite detection
  // The ARTIC will start looking for a satellite for the specified amount of time.
  // If no satellite is detected, INT 2 will be set with the ‘SATELLITE_TIMEOUT’ flag.
  // If a satellite was detected, by receiving 5 consecutive 0x7E flags, INT 1 will be set with the ‘RX_SATELLITE_DETECTED’ flag.
  // sendMCUinstruction returns an ARTIC_R2_MCU_Command_Result
  // sendMCUinstruction will return ARTIC_R2_MCU_COMMAND_ACCEPTED if the instruction was accepted
  ARTIC_R2_MCU_Command_Result result = myARTIC.sendMCUinstruction(INST_SATELLITE_DETECTION);

  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
  Serial.println();
  Serial.println(F("ARTIC R2 MCU instruction result:"));
  myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
  Serial.println();
  
  if ((result == ARTIC_R2_MCU_COMMAND_REJECTED) || (result == ARTIC_R2_MCU_COMMAND_OVERFLOW))
  {
    Serial.println("MCU Command failed! Freezing...");
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

  // Note: satellite detection does not time out once a satellite has been detected. The firmware goes IDLE instead.

  // Note: With ARTIC006 firmware: when satellite detection times out and INT2 goes high, INT1 goes high too! Confusing...
  // We should only believe INT1 if INT2 is low.

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // Check the interrupt 2 flag. This will go high if satellite detection times out
  {
    Serial.println(F("INT2 pin is high. Satellite detection has timed out!"));
  }
  else if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when a satellite is detected
  {
    Serial.println(F("INT1 pin is high. Satellite detected!"));
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
    Serial.println(F("Satellite detection is complete! Freezing..."));
    while (1)
      ; // Do nothing more
  }
}
