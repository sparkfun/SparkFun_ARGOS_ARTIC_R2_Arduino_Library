/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: January 26th 2021

  This example:
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the TCXO voltage;
    sets the TCXO warmup time;
    sets the satellite detection timeout to 60 seconds;
    sets the TX mode to ARGOS PTT A2;
    sets the TX frequency;
    instructs the ARTIC to Transmit One Package And Go Idle;
    keeps checking the MCU status until transmit is complete;
    repeats.

  The transmit power can be reduced by 8dB by uncommenting the line: myARTIC.attenuateTXgain(true);

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
  ARTIC_PWR_EN_Pin = D8
  RF_PWR_EN_Pin = D9
  (SPI COPI = D11)
  (SPI CIPO = D12)
  (SPI SCK = D13)
*/

const uint32_t PLATFORM_ID = 0x01234567;

const uint8_t Nx32_bits = 8; // In this example, transmit the maximum amount of data

// The user data. Note: only the least-significant 24 bits of userData[0] are transmitted.
const uint32_t userData[Nx32_bits] = {0x00111213, 0x21222324, 0x31323334, 0x41424344, 0x51525354, 0x61626364, 0x71727374, 0x81828384};

// The complete over-air data pattern (after the sync pattern) will be 0xF123456711121321222324313233344142434451525354616263647172737481828384

const unsigned long repetitionPeriod = 50000; // Define the repetition period in milliseconds

const uint32_t tcxoWarmupTime = 10; // Define the TCXO warmup time

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
int ARTIC_PWR_EN_Pin = 8;
int RF_PWR_EN_Pin = 9;

// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to jump between any of the steps
enum {
  configure_ARTIC,     // Configure the ARTIC (set the satellite detection timeout and TX mode)
  ARTIC_TX,            // Start the ARTIC TX
  wait_for_ARTIC_TX,   // Wait for the ARTIC to transmit
} loop_steps;
int loop_step = configure_ARTIC; // Make sure loop_step is set to configure_ARTIC

unsigned long lastTransmit = 0; // Keep a record of the last transmit

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));
  Serial.println();

  SPI.begin();

  // Uncomment the next line to enable the helpful debug messages
  //myARTIC.enableDebugging(); // Enable debug messages to Serial

  Serial.println(F("Starting the ARTIC R2..."));
  Serial.println();

  // Begin the ARTIC: enable power and upload firmware or boot from flash
  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, ARTIC_PWR_EN_Pin, RF_PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin) == false)
  {
    Serial.println("ARTIC R2 not detected. Freezing...");
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  // loop is one large switch/case that controls the sequencing of the code
  switch (loop_step) {

    // ************************************************************************************************
    // Configure the ARTIC
    case configure_ARTIC:
    {
      // Set the TCXO voltage to 1.8V and autoDisable to 1
      if (myARTIC.setTCXOControl(1.8, true) == false)
      {
        Serial.println("setTCXOControl failed. Freezing...");
        while (1)
          ; // Do nothing more
      }

      // Set the TCXO warm-up time
      if (myARTIC.setTCXOWarmupTime(tcxoWarmupTime) == false)
      {
        Serial.println("setTCXOWarmupTime failed. Freezing...");
        while (1)
          ; // Do nothing more
      }

      // Set the satellite detection timeout to 60 seconds
      if (myARTIC.setSatelliteDetectionTimeout(60) == false)
      {
        Serial.println("setSatelliteDetectionTimeout failed. Freezing...");
        while (1)
          ; // Do nothing more
      }

      // Set the TX mode to ARGOS PTT A2
      ARTIC_R2_MCU_Command_Result result = myARTIC.sendConfigurationCommand(CONFIG_CMD_SET_PTT_A2_TX_MODE);
      myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
      if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
      {
        Serial.println("sendConfigurationCommand failed. Freezing...");
        while (1)
          ; // Do nothing more
      }

      // Read and print the ARGOS configuration
      ARGOS_Configuration_Register configuration;
      myARTIC.readARGOSconfiguration(&configuration);
      myARTIC.printARGOSconfiguration(configuration);

      // Set the ARGOS PTT A2 frequency to 401.630 MHz
      // From AS3-SP-516-2098-CNES:
      // The transmission frequency for PTT/PMT-A2 platforms shall be set between 399.91 MHz to 401.68 MHz.
      // Due to frequency regulations, the frequency ranges [400.05 MHz to 401.0 MHz] and [401.2 MHz to 401.3 MHz] are forbidden for A2 transmissions.
      if (myARTIC.setARGOS23TxFrequency(401.630) == false)
      {
        Serial.println("setARGOS23TxFrequency failed. Freezing...");
        while (1)
          ; // Do nothing more
      }

      // Print the TX frequency
      float tx2freq = myARTIC.getARGOS23TxFrequency();
      Serial.print(F("The ARGOS PTT A2 TX Frequency is "));
      Serial.print(tx2freq, 3);
      Serial.println(F(" MHz."));

      // Uncomment the next line if you want to attenuate the transmit power by 8dB
      //myARTIC.attenuateTXgain(true);

      loop_step = ARTIC_TX; // Move on
    }
    break;

    // ************************************************************************************************
    // Start the ARTIC in Transmit One Package And Go Idle mode
    case ARTIC_TX:
    {
      // Configure the Tx payload for ARGOS PTT A2 using the platform ID and the defined user data
      if (myARTIC.setPayloadARGOS2(PLATFORM_ID, Nx32_bits, (uint32_t *)&userData) == false)
      {
        Serial.println(F("setPayloadARGOS2 failed!"));
        Serial.println();
        // Read the payload back again and print it
        myARTIC.readTxPayload();
        myARTIC.printTxPayload();
        Serial.println();
        Serial.println(F("Freezing..."));
        while (1)
          ; // Do nothing more
      }

/*
        // Read the payload back again and print it
        myARTIC.readTxPayload();
        myARTIC.printTxPayload();
        Serial.println();
*/

      // Wait for the next repetition period
      while ((lastTransmit + repetitionPeriod) > millis())
      {
        if ((millis() % 1000) < 50) // Print how long it is until the next transmit
        {
          Serial.print(F("The next transmit will take place in "));
          unsigned long remaining = ((lastTransmit + repetitionPeriod) - millis()) / 1000;
          Serial.print(remaining);
          Serial.println(F(" seconds"));
        }
        delay(50);
      }
      lastTransmit = millis(); // Update lastTransmit

      // Tell the ARTIC to do its thing!
      ARTIC_R2_MCU_Command_Result result = myARTIC.sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE);
      if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
      {
        Serial.println("sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) failed!");
        Serial.println();
        ARTIC_R2_Firmware_Status status;
        myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
        Serial.println(F("ARTIC R2 Firmware Status:"));
        myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
        Serial.println();
        Serial.println(F("ARTIC_R2_MCU_Command_Result:"));
        myARTIC.printCommandResult(result); // Pretty-print the command result to Serial
        Serial.println();
        Serial.println("Freezing...");
        while (1)
          ; // Do nothing more
      }

      loop_step = wait_for_ARTIC_TX; // Move on
    }
    break;

    // ************************************************************************************************
    // Start the ARTIC in Transmit One Package And Go Idle mode
    case wait_for_ARTIC_TX:
    {
      delay(1000); // Check the status every second

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
        Serial.println(F("INT2 pin is high. TX message was invalid! (Something really bad must have happened...)"));
      }

      Serial.println();

      // Read and print the instruction progress
      ARTIC_R2_MCU_Instruction_Progress progress;
      // checkMCUinstructionProgress will return true if the instruction is complete
      boolean instructionComplete = myARTIC.checkMCUinstructionProgress(&progress); // Check the instruction progress
      Serial.println(F("ARTIC R2 instruction progress:"));
      myARTIC.printInstructionProgress(progress); // Pretty-print the progress to Serial

      Serial.println();

      if (instructionComplete)
      {
        Serial.println(F("Transmission is complete!"));
        Serial.println();

        Serial.println(F("Clearing INT1."));
        Serial.println();

        // Clear INT1
        if (myARTIC.clearInterrupts(1) == false)
        {
          Serial.println("clearInterrupts failed! Freezing...");
          while (1)
            ; // Do nothing more
        }

        loop_step = ARTIC_TX; // Do over...
      }
    }
    break;
  }
}
