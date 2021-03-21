/*
  Using the SparkFun ARGOS ARTIC R2 Breakout & IOTA
  By: Paul Clark
  SparkFun Electronics
  Date: March 21st 2021

  This example:
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the RX mode to ARGOS 3;
    sets the TCXO voltage;
    enables RX transparent mode (so we will receive all valid messages even if they are not addressed to us);
    instructs the ARTIC to Start Continuous Reception;
    keeps checking the MCU status;
    downloads messages as they are received;
    clears INT1 each time a message is downloaded;
    after GO_IDLE_AFTER milliseconds, the ARTIC is instructed to Go To Idle (which aborts continuous reception).

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/17236

  Hardware Connections:
  This example assumes the ARTIC Breakout has been mounted on a SparkFun Thing Plus - Artemis:
  https://www.sparkfun.com/products/15574
  CS_Pin = A5 (D24)
  GAIN8_Pin = D3
  BOOT_Pin = D4
  INT1_Pin = D5
  INT2_Pin = D6
  RESET_Pin = D7
  ARTIC_PWR_EN_Pin = IOTA_PWR_EN_Pin = D8
  RF_PWR_EN_Pin = D9
  (SPI COPI = D11)
  (SPI CIPO = D12)
  (SPI SCK = D13)

  If you are using IOTA, uncomment the #define IOTA below.
  IOTA only has one power enable pin. Uncommenting the #define IOTA will let the code run correctly on IOTA.
  
*/

//#define IOTA // Uncomment this line if you are using IOTA (not the ARTIC R2 Breakout)

#define GO_IDLE_AFTER 120000 // Abort receive (go idle) after this many milliseconds (120000 = 2 mins)

boolean goToIdleSent = false; // Use this flag to make sure we only send Go To Idle once

#include <SPI.h>

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

// Pin assignments for the SparkFun Thing Plus - Artemis
// (Change these if required)
int CS_Pin = 24;
int GAIN8_Pin = 3; // Optional. Set to -1 if you don't want to control the gain. The library defaults to maximum power.
int BOOT_Pin = 4;
int INT1_Pin = 5;
int INT2_Pin = 6;
int RESET_Pin = 7;
#ifdef IOTA
int IOTA_PWR_EN_Pin = 8; // IOTA has a single power enable pin
#else
int ARTIC_PWR_EN_Pin = 8; // The ARTIC R2 Breakout has separate enables for the ARTIC and the RF Amplifier
int RF_PWR_EN_Pin = 9;
#endif

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));
  Serial.println();

  Serial.println(F("ARTIC R2 is booting..."));
  Serial.println();

  SPI.begin();

  //myARTIC.enableDebugging(); // Enable debug messages to Serial

  // Begin the ARTIC: enable power and upload firmware or boot from flash
#ifdef IOTA
  if (myARTIC.beginIOTA(CS_Pin, RESET_Pin, BOOT_Pin, IOTA_PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin) == false)
#else
  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, ARTIC_PWR_EN_Pin, RF_PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin) == false)
#endif
  {
    Serial.println("ARTIC R2 not detected. Freezing...");
    while (1)
      ; // Do nothing more
  }

  Serial.println(F("ARTIC R2 boot was successful."));
  Serial.println();

  // Read and print the ARTIC R2 status register
  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
  Serial.println();

  // Read and print the ARGOS configuration
  ARGOS_Configuration_Register configuration;
  myARTIC.readARGOSconfiguration(&configuration);
  myARTIC.printARGOSconfiguration(configuration); // Pretty-print the TX and RX configuration to Serial

  Serial.println(F("Setting the RX mode to ARGOS 3..."));

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

  // Set the TCXO voltage to 1.8V and autoDisable to 1
  if (myARTIC.setTCXOControl(1.8, true) == false)
  {
    Serial.println("setTCXOControl failed. Freezing...");
    while (1)
      ; // Do nothing more
  }

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

  goToIdleSent = false; // Make sure goToIdleSent is false (redundant!)
}

void loop()
{
  delay(5000);

  Serial.println();

  // Read and print the ARTIC R2 status register
  ARTIC_R2_Firmware_Status status;
  myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
  Serial.println(F("ARTIC R2 Firmware Status:"));
  myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial

  if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when a message is received - or Go To Idle is complete.
  {
    Serial.println(F("INT1 pin is high."));
  }

  Serial.println();

  // Read and print the instruction progress
  ARTIC_R2_MCU_Instruction_Progress progress;
  // checkMCUinstructionProgress will return true if the instruction is complete
  boolean instructionComplete = myARTIC.checkMCUinstructionProgress(&progress); // Check the instruction progress
  Serial.println(F("ARTIC R2 instruction progress:"));
  myARTIC.printInstructionProgress(progress); // Pretty-print the progress to Serial

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
    // *** INT1 will go high again after 100us if there is another message to be read (which could cause clearInterrupts to return false) ***
    Serial.println(F("Clearing INT1."));
    Serial.println();
    if (myARTIC.clearInterrupts(1) == false)
    {
      Serial.println("clearInterrupts may have failed!");
    }
  }

  // Check if it is time to abort reception and Go To Idle
  if ((goToIdleSent == false) and (millis() > GO_IDLE_AFTER))
  {
    Serial.println();
    Serial.println(F("Time to stop receiving! Sending Go To Idle..."));
    Serial.println();

    // Tell the ARTIC to return to idle mode.
    ARTIC_R2_MCU_Command_Result result = myARTIC.sendMCUinstruction(INST_GO_TO_IDLE);

    if ((result == ARTIC_R2_MCU_COMMAND_REJECTED) || (result == ARTIC_R2_MCU_COMMAND_OVERFLOW))
    {
      Serial.println("MCU Command failed!");
      Serial.println();
      myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
      Serial.println(F("ARTIC R2 Firmware Status:"));
      myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
      Serial.println();
      Serial.println(F("ARTIC R2 MCU Command Result:"));
      myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
      Serial.println();
      Serial.println("Freezing...");
      while (1)
        ; // Do nothing more
    }

    Serial.println(F("Go To Idle sent."));
    Serial.println();

    goToIdleSent = true; // Set goToIdleSent to true so we only Go To Idle once
  }

  // Check if Go To Idle is complete
  // (Continuous Reception never goes idle, so Go To Idle must have caused this)
  if (instructionComplete)
  {
    Serial.println();
    Serial.println(F("Instruction (Go To Idle) is complete! Freezing..."));
    Serial.println();

    while (1)
      ; // Do nothing more
  }
}
