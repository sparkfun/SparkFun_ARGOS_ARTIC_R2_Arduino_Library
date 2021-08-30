/*
  Using the SparkFun smôl ARGOS ARTIC R2 Board
  By: Paul Clark
  SparkFun Electronics
  Date: August 30th 2021

  This example shows how to use the smôl ARTIC R2 for ARGOS satellite communication.
  The smôl ESP32 is the board which runs this example.
  Power can be provided by:
    The ESP32 board USB-C connection
    The smôl Power Board AAA
    The smôl Power Board LiPo

  Feel like supporting our work? Buy a board from SparkFun!

  The smôl stack-up for this example is:
  smôl ARTIC R2:         https://www.sparkfun.com/products/18363
  smôl ESP32:            https://www.sparkfun.com/products/18362
  
  smôl Power Board LiPo: https://www.sparkfun.com/products/18359 (Optional)
  smôl Power Board AAA:  https://www.sparkfun.com/products/18360 (Optional)

  The way the boards are stacked is important:

  OUT ---smôl ARTIC R2--- IN
                           |
   ________________________/
  /
  |
  OUT ---  smôl ESP32 --- IN

  Arranged like this:
  The ESP32 GPIO0 (Digital Pin 27) controls the power for the ARTIC R2
  ARTIC R2 uses SPI Chip Select 0 (ESP32 Digital Pin 5)
  
  This example:
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the TCXO voltage;
    sets the satellite detection timeout to 600 seconds;
    starts satellite detection;
    keeps checking the MCU status until a satellite is detected or the detection times out.

*/

#include <Wire.h> //Needed for I2C to ARTIC R2 GPIO and GNSS

#include <SPI.h>

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

// Pin assignments for the smôl stack-up described above
int CS_Pin = 5;            // smôl CS0 = ESP32 Pin 5
int ARTIC_PWR_EN_Pin = 27; // smôl GPIO0 = ESP32 Pin 27

// The ARTIC RESETB, INT1, BOOT and G8 signals are accessed through a PCA9536 I2C-GPIO chip on the smôl ARTIC R2

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS smôl ARTIC R2 Example"));
  Serial.println();

  Serial.println(F("ARTIC R2 is booting..."));
  Serial.println();

  Wire.begin(); // Needed to communicate with the I2C GPIO chip on the smôl ARTIC R2
  SPI.begin();

  //myARTIC.enableDebugging(); // Enable debug messages to Serial

  // Begin the ARTIC: enable power and upload firmware or boot from flash
  if (myARTIC.beginSmol(CS_Pin, ARTIC_PWR_EN_Pin) == false) // Default to using Wire to communicate with the PCA9536 I2C-GPIO chip on the smôl ARTIC R2
  {
    Serial.println("ARTIC R2 not detected. Please check the smôl stack-up and flexible circuits. Freezing...");
    while (1)
      ; // Do nothing more
  }

  Serial.println(F("ARTIC R2 boot was successful."));
  Serial.println();

  // Read and print the ARTIC R2 firmware status
  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
  Serial.println();

  // Read and print the ARGOS configuration
  ARGOS_Configuration_Register configuration;
  myARTIC.readARGOSconfiguration(&configuration);
  myARTIC.printARGOSconfiguration(configuration); // Pretty-print the TX and RX configuration to Serial

  // Set the TCXO voltage to 1.8V and autoDisable to 1
  if (myARTIC.setTCXOControl(1.8, true) == false)
  {
    Serial.println("setTCXOControl failed. Freezing...");
    while (1)
      ; // Do nothing more
  }

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
