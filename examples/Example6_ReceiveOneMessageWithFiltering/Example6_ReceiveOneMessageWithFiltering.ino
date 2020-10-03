/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: September 29th 2020

  This example:
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the RX mode to ARGOS 3;
    adds the platform ID to the address Look Up Table (LUT);
    disables RX transparent mode;
    enables the RX CRC check (even though this is enabled by default);
    instructs the ARTIC to Receive One Message (for an unlimited time);
    keeps checking the MCU status until a message is received.

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  This example requires a Platform ID. Copy and paste your 28-bit Platform ID into ADDRESS_LS_BITS and ADDRESS_MS_BITS. (See below)

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

// CLS will have provided you with a Platform ID for your ARGOS R2. Copy and paste it into PLATFORM_ID below to filter out all other messages.
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
  Serial.println();

  Serial.println(F("The ARTIC is booting. This will take approx. 12 seconds."));
  Serial.println();

  SPI.begin();

  myARTIC.enableDebugging(); // Enable debug messages to Serial

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
    Serial.println("clearInterrupts failed!");
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

  // Add our address (platform ID) to the LUT.
  //   You can add multiple addresses if required, up to a maximum of 50.
  //   Add an additional address by calling addAddressToLUT again.
  //   addAddressToLUT will return false if the LUT is full.
  //   You can clear the LUT by calling clearAddressLUT.
  if (myARTIC.addAddressToLUT(PLATFORM_ID) == false)
  {
    Serial.println(F("addAddressToLUT failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Check that our platform ID was added correctly
  uint32_t tableEntry;
  if (myARTIC.readAddressLUTentry(0, &tableEntry) == false)
  {
    Serial.println(F("readAddressLUTentry failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }
  Serial.print(F("Address Look Up Table entry 0 is 0x"));
  Serial.println(tableEntry, HEX);
  if (tableEntry != PLATFORM_ID)
  {
    Serial.println(F("Platform ID mismatch! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Disable RX transparent mode so we will only receive messages addressed to us
  // (transparent mode is disabled by default)
  if (myARTIC.disableRXTransparentMode() == false)
  {
    Serial.println(F("disableRXTransparentMode failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Enable the RX CRC check (even though this is enabled by default)
  if (myARTIC.enableRXCRC() == false)
  {
    Serial.println(F("enableRXCRC failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  Serial.println();
  Serial.println(F("Starting message reception..."));
  Serial.println();

  // Start the ARTIC in receiving mode for an unlimited time until 1 message has been received.
  // If the message is received the Artic will go to IDLE. The user can abort the reception using the ‘Go to idle’ command.
  result = myARTIC.sendMCUinstruction(INST_START_RECEIVING_1_MESSAGE);
  
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
    Serial.println(F("Message reception is complete!"));
    Serial.println();
    
    if (progress == ARTIC_R2_MCU_PROGRESS_RECEIVE_ONE_MESSAGE_RX_VALID_MESSAGE) // If a message was received, read it.
    {
      // Read a downlink message from the RX payload buffer
      Downlink_Message downlinkMessage;
      if (myARTIC.readDownlinkMessage(&downlinkMessage))
      {
        Serial.println(F("Message received:"));
        Serial.printf("Payload length:  %d\n", downlinkMessage.payloadLength);
        Serial.printf("Addressee ID:    0x%07X\n", downlinkMessage.addresseeIdentification);
        Serial.printf("ADCS:            0x%X\n", downlinkMessage.ADCS);
        Serial.printf("Service:         0x%02X\n", downlinkMessage.service);
        Serial.printf("FCS:             0x%04X\n", downlinkMessage.FCS);
        Serial.print(F("Payload buffer:  0x"));
        for (int i = 0; i < ((downlinkMessage.payloadLength / 8) - 7); i++)
        {
          Serial.printf("%02X", downlinkMessage.payload[i]);
        }
        Serial.println();
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
