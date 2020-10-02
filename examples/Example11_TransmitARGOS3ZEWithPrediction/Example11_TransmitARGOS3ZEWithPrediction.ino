/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: September 25th 2020

  This example requires a u-blox GPS/GNSS module (for the time, latitude and longitude)
  and assumes it is connected via Qwiic (I2C):
  https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

  This example:
    begins the u-blox module;
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the satellite detection timeout to 60 seconds;
    sets the TX mode to ARGOS PTT-ZE;
    sets the TX frequency;
    reads the GPS time, latitude and longitude;
    calculates the next satellite pass;
    waits until the next satellite pass;
    instructs the ARTIC to Transmit One Package And Go Idle;
    keeps checking the MCU status until transmit is complete;
    repeats for the next satellite pass.

  This example only transmits one ZE package per pass. A future version will transmit multiple packages on each pass.

  The ARGOS 3 PTT-ZE message contains only the 28-bit platform ID. Please change PLATFORM_ID below to your ID.

  Please log in to ARGOS Web https://argos-system.cls.fr/argos-cwi2/login.html
  and copy and paste the latest Satellite AOP (Adapted Orbit Parameters)
  into AOP below.

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

const uint8_t numARGOSsatellites = 7; // Change this if required to match the number of satellites in the AOP

// Copy and paste the latest AOP from ARGOS Web between the quotes and then carefully delete the line feeds
// Check the alignment afterwards - make sure that the satellite identifiers still ine up correctly (or convertAOPtoParameters will go horribly wrong!)
// Check the alignment: " MA A 5 3 0 2020 10  1 22  7 29  7195.569  98.5114  336.036  -25.341  101.3592   0.00 MB 9 3 0 0 2020 10  1 23 21 58  7195.654  98.7194  331.991  -25.340  101.3604   0.00 MC B 7 3 0 2020 10  1 22 34 23  7195.569  98.6883  344.217  -25.340  101.3587   0.00 15 5 0 0 0 2020 10  1 22 44 11  7180.495  98.7089  308.255  -25.259  101.0408   0.00 18 8 0 0 0 2020 10  1 21 50 32  7225.981  99.0331  354.556  -25.498  102.0000  -0.79 19 C 6 0 0 2020 10  1 22  7  6  7226.365  99.1946  301.174  -25.499  102.0077  -0.54 SR D 4 3 0 2020 10  1 22 33 38  7160.233  98.5416  110.362  -25.154  100.6146  -0.12";
const char AOP[] =      " MA A 5 3 0 2020 10  1 22  7 29  7195.569  98.5114  336.036  -25.341  101.3592   0.00 MB 9 3 0 0 2020 10  1 23 21 58  7195.654  98.7194  331.991  -25.340  101.3604   0.00 MC B 7 3 0 2020 10  1 22 34 23  7195.569  98.6883  344.217  -25.340  101.3587   0.00 15 5 0 0 0 2020 10  1 22 44 11  7180.495  98.7089  308.255  -25.259  101.0408   0.00 18 8 0 0 0 2020 10  1 21 50 32  7225.981  99.0331  354.556  -25.498  102.0000  -0.79 19 C 6 0 0 2020 10  1 22  7  6  7226.365  99.1946  301.174  -25.499  102.0077  -0.54 SR D 4 3 0 2020 10  1 22 33 38  7160.233  98.5416  110.362  -25.154  100.6146  -0.12";

#include <SPI.h>

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

// Pin assignments for the SparkFun Thing Plus - Artemis
uint8_t CS_Pin = 24;
uint8_t GAIN8_Pin = 3;
uint8_t GAIN16_Pin = 4;
uint8_t BOOT_Pin = 5;
uint8_t INT1_Pin = 6;
uint8_t INT2_Pin = 7;
uint8_t RESET_Pin = 8;
uint8_t PWR_EN_Pin = 9;

// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to jump between any of the steps
#define start_ARTIC         0 // Wait for the ARTIC to finish booting
#define configure_ARTIC     1 // Configure the ARTIC (set the satellite detection timeout and TX mode)
#define wait_for_GPS        2 // Wait for the GPS time and position to be valid
#define calculate_next_pass 3 // Read the GPS time, lat and lon. Calculate the next satellite pass
#define wait_for_next_pass  4 // Wait for the next satellite pass
#define ARTIC_TX            5 // Start the ARTIC TX
int loop_step = start_ARTIC; // Make sure loop_step is set to start_ARTIC

uint32_t nextSatellitePass; // Time of the next satellite pass

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ; // Do nothing more
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the current ioPortsettings to flash and BBR

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
}

void loop()
{
  // loop is one large switch/case that controls the sequencing of the code
  switch (loop_step) {

    // ************************************************************************************************
    // Wait for the ARTIC to finish booting
    case start_ARTIC:
    
      ARTIC_R2_Firmware_Status status;
      myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
    
      if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1 == false); // Check the interrupt 1 flag. This will go high when the RX offset calibration has completed.
      {
        Serial.println(F("Waiting for INT1 to go high... (This could take up to 5 minutes with ARTIC006 firmware!)"));
        Serial.println();
    
        Serial.println(F("ARTIC R2 Firmware Status:"));
        myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial
        Serial.println();
      
        delay(5000);
      }
      else
      {
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
  
        loop_step = configure_ARTIC; // Move on
      }
      
      break;
  
    // ************************************************************************************************
    // Configure the ARTIC
    case configure_ARTIC:
    
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

      // Print the TX frequency
      float tx23freq = myARTIC.getARGOS23TxFrequency();
      Serial.print(F("The ARGOS 2/3 TX Frequency is "));
      Serial.print(tx23freq, 3);
      Serial.println(F(" MHz."));
      
      // Configure the Tx payload for ARGOS 3 PTT-ZE using our platform ID and 0 bits of user data
      if (myARTIC.setPayloadARGOS3ZE(PLATFORM_ID) == false)
      {
        Serial.println(F("setPayloadARGOS3ZE failed! Freezing..."));
        while (1)
          ; // Do nothing more
      }

      loop_step = wait_for_GPS; // Move on

      break;
    
    // ************************************************************************************************
    // Wait for the GPS time to be valid and for the position fix to be 3D
    case wait_for_GPS:

      // Read the GPS. Check that the time is valid. It should be by now as we have waited for the ARTIC to start!
      boolean timeValid = myGPS.getTimeValid();
      timeValid = myGPS.getTimeValid(); // Call getTimeValid twice to ensure we have fresh data
      Serial.print(F("GPS time is "));
      if (timeValid == false) Serial.print(F("not "));
      Serial.println(F("valid"));

      // Read the GPS. Check that the position fix is 3D. It should be by now as we have just waited for the ARTIC to start!
      uint8_t fixType = myGPS.getFixType();
      Serial.print(F("GPS position fix type is "));
      Serial.println(fixType);
      Serial.println();

      if ((timeValid == true) && (fixType >= 3)) // Check if both time and fix are valid
      {
        loop_step = calculate_next_pass; // Move on
      }
      else
      {
        Serial.println(F("Waiting for GPS time to be valid and the fix type to be 3D..."));
        Serial.println();
      }

      break;
      
    // ************************************************************************************************
    // Read the AOP
    // Read the time, latitude and longitude from GPS
    // Calculate the time of the next satellite pass
    case calculate_next_pass:

      // Read the AOP, convert into bulletin_data_t
      bulletin_data_t satelliteParameters[numARGOSsatellites]; // Create an array of bulletin_data_t to hold the parameters for all satellites
      if (myARTIC.convertAOPtoParameters(AOP, satelliteParameters, numARGOSsatellites) == false)
      {
        Serial.println("convertAOPtoParameters failed! Freezing...");
        while (1)
          ; // Do nothing more
      }

      // Read the GPS time, latitude and longitude. Convert to epoch.
      uint32_t epochNow = myARTIC.convertGPSTimeToEpoch(myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()); // Convert GPS date & time to epoch
      Serial.print(F("GPS time is: "));
      Serial.println(myARTIC.convertEpochToDateTime(epochNow));
      Serial.print(F("The number of seconds since the epoch is: "));
      Serial.println(epochNow);

      // Read the GPS lat and lon. Convert to float.
      float lat = ((float)myGPS.getLatitude()) / 10000000; // Convert from degrees^-7
      float lon = ((float)myGPS.getLongitude()) / 10000000; // Convert from degrees^-7

      float min_elevation = 45.0; // Minimum satellite elevation (above the horizon) - REDUCE THIS IF YOU HAVE A CLEAR VIEW TO THE HORIZON

      // Predict the next satellite pass
      nextSatellitePass = myARTIC.predictNextSatellitePass(satelliteParameters, min_elevation, numARGOSsatellites, lon, lat, epochNow);

      // Print the prediction
      Serial.print(F("The next satellite pass will take place at: "));
      Serial.println(myARTIC.convertEpochToDateTime(nextSatellitePass));
      Serial.print(F("The number of seconds since the epoch will be: "));
      Serial.println(nextSatellitePass);
      Serial.println();

      loop_step = wait_for_next_pass; // Move on

      break;
      
    // ************************************************************************************************
    // Wait until the next satellite pass
    case wait_for_next_pass:
    
      // Read the GPS time
      uint32_t epochNow = myARTIC.convertGPSTimeToEpoch(myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()); // Convert GPS date & time to epoch

      // Calculate how many seconds remain until the satellite is in view
      int32_t secsRemaining = (int32_t)nextSatellitePass - (int32_t)epochNow;
      Serial.print(F("The satellite will be in view in "));
      Serial.print(secsRemaining);
      Serial.println(F(" seconds"));

      // Check if we should start the TX
      if (secsRemaining <= 0)
      {
        Serial.println();
        Serial.println(F("*** STARTING TX ***"));
        Serial.println();
        loop_step = ARTIC_TX; // Move on
      }

      break;
      
    // ************************************************************************************************
    // Start the ARTIC in Transmit One Package And Go Idle mode
    case ARTIC_TX:
    
      // Tell the ARTIC to do its thing!
      ARTIC_R2_MCU_Command_Result result = myARTIC.sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE);
      if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
      {
        Serial.println("sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) failed!");
        Serial.println();
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

      break;
    
    // ************************************************************************************************
    // Start the ARTIC in Transmit One Package And Go Idle mode
    case wait_for_ARTIC_TX:
    
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
        Serial.println(F("INT2 pin is high. TX message was invalid! (Something really bad must have happened... ARGOS PTT-ZE is as simple as it gets!)"));
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
        
        loop_step = wait_for_GPS; // Do over...
      }

      break;
  }
}
