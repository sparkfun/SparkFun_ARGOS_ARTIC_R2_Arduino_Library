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
    sets the RX mode to ARGOS 3;
    enables RX transparent mode (so we will receive the first valid message even if it is not addressed to us);
    instructs the ARTIC to Receive One Message (for an unlimited time);
    keeps checking the MCU status until a message is received.

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

  // Set the RX mode to ARGOS 3
  ARTIC_R2_MCU_Command_Result result = myARTIC.sendConfigurationCommand(CONFIG_CMD_SET_ARGOS_3_RX_MODE);
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

  // Enable RX transparent mode so we will receive the first valid message even if it is not addressed to us
  if (myARTIC.enableRXTransparentMode() == false)
  {
    Serial.println(F("enableRXTransparentMode failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Start the ARTIC in receiving mode for an unlimited time until 1 message has been received.
  // If the message is received the Artic will go to IDLE. The user can abort the reception using the ‘Go to idle’ command.
  result = myARTIC.sendMCUinstruction(INST_START_RECEIVING_1_MESSAGE);
  if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
  {
    Serial.println();
    Serial.println("<sendMCUinstruction(INST_START_RECEIVING_1_MESSAGE) failed>");
    Serial.println();
    myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
    Serial.println(F("ARTIC R2 Firmware Status:"));
    myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
    Serial.println();
    Serial.println(F("ARTIC_R2_MCU_Command_Result:"));
    myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
    Serial.println();
    Serial.println("</sendMCUinstruction(INST_START_RECEIVING_1_MESSAGE) failed> Freezing...");
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

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when a satellite is detected
  {
    Serial.println(F("INT1 pin is high. Valid message received!"));
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
    Serial.println(F("Instruction is complete!"));
    Serial.println();
    
    if (progress == ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_VALID_MESSAGE) // If a message was received, read it.
    {
      // Read a downlink message from the RX payload buffer
      Downlink_Message downlinkMessage;
      if (myARTIC.readDownlinkMessage(&downlinkMessage))
      {
        Serial.println(F("Message received:"));
        Serial.printf("Payload length:  %d\n", downlinkMessage.payloadLength);
        Serial.printf("Addressee ID:    0x%04X\n", downlinkMessage.addresseeIdentification);
        Serial.printf("ADCS:            0x%02X\n", downlinkMessage.ADCS);
        Serial.printf("Service:         0x%02X\n", downlinkMessage.service);
        Serial.printf("FCS:             0x%04X\n", downlinkMessage.FCS);
        Serial.print(F("Payload buffer:  0x"));
        for (int i = 0; i < 17; i++)
        {
          Serial.printf("%02X", downlinkMessage.payload[i]);
        }
        Serial.println();
        //while (1)
        //  ; // Do nothing more
      }
      else
      {
        Serial.println(F("readDownlinkMessage failed!"));
      }
    }
    
    Serial.println();
    Serial.println(F("We are done. Freezing..."));
    while (1)
      ; // Do nothing more
  }
}
