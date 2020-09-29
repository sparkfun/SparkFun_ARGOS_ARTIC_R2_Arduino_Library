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
    enables RX transparent mode (so we will receive all valid messages even if they are not addressed to us);
    instructs the ARTIC to Start Continuous Reception;
    keeps checking the MCU status;
    downloads messages as they are received;
    clears INT1 each time a message is downloaded.
  LED_BUILTIN will illuminate when a satellite is detected.

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
  pinMode(LED_BUILTIN, OUTPUT); // Turn the LED off
  digitalWrite(LED_BUILTIN, LOW);
  
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

  // Enable RX transparent mode so we will receive all valid messages even if they are not addressed to us
  if (myARTIC.enableRXTransparentMode() == false)
  {
    Serial.println(F("enableRXTransparentMode failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  Serial.println();
  Serial.println(F("Starting message reception..."));
  Serial.println();

  // Start the ARTIC in receiving mode for an unlimited time and unlimited number of messages.
  // The user has to use the 'Go To Idle' command to stop the receiver.
  result = myARTIC.sendMCUinstruction(INST_START_CONTINUOUS_RECEPTION);

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

  // Read the ARTIC R2 status register
  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when a satellite is detected
  {
    Serial.println();
    Serial.println(F("Firmware status: INT1 flag is high. Valid message received!"));
  }

  // Illuminate the LED if a satellite is detected
  digitalWrite(LED_BUILTIN, status.STATUS_REGISTER_BITS.RX_SATELLITE_DETECTED);

  // Read the instruction progress
  ARTIC_R2_MCU_Instruction_Progress progress;
  myARTIC.checkMCUinstructionProgress(&progress); // Check the instruction progress

  // Check if there is a valid message waiting to be downloaded
  if (progress == ARTIC_R2_MCU_PROGRESS_CONTINUOUS_RECEPTION_RX_VALID_MESSAGE)
  {
    Serial.println();
    Serial.println(F("Valid message received! Downloading..."));
    Serial.println();
    
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

    // Manually clear INT1 now that the message has been downloaded. This will clear the RX_VALID_MESSAGE flag too.
    // *** INT1 will go high again after 100us if there is another message to be read ***
    Serial.println();
    Serial.println(F("Clearing INT1."));
    if (myARTIC.clearInterrupts(1) == false)
    {
      Serial.println("clearInterrupts may have failed!");
    }  
  }
}
