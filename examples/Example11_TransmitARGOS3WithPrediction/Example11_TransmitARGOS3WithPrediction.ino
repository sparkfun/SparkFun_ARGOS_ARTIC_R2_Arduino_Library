/*
  Using the SparkFun ARGOS ARTIC R2 Breakout & IOTA
  By: Paul Clark
  SparkFun Electronics
  Date: June 8th 2021

  This example requires a u-blox GPS/GNSS module (for the time, latitude and longitude)
  and assumes it is connected via Qwiic (I2C):
  https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/17236
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/15210

  This example:
    begins the u-blox module;
    begins (initializes) the ARTIC;
    reads and prints the ARTIC TX and RX configuration;
    reads and prints the firmware status;
    sets the TCXO voltage;
    sets the TCXO warmup time;
    sets the satellite detection timeout to 60 seconds;
    sets the TX mode to ARGOS PTT-A3;
    sets the TX frequency;
    reads the GPS time, latitude and longitude;
    calculates the next satellite pass;
    waits until the next satellite pass;
    instructs the ARTIC to Transmit One Package And Go Idle;
    keeps checking the MCU status until transmit is complete;
    repeats the message transmit numberTransmits times, repetitionPeriod seconds apart;
    repeats for the next satellite pass.

  The message contains the GPS latitude and longitude in a compact form which ARGOS Web will understand.
  Please contact CLS / Woods Hole Group and ask them to apply the SPARKFUN_GPS template on ARGOS Web.
  The Latitude and Longitude will then be extracted, converted and displayed automatically when you view your data.
  The number of user bits is 56.
  Lat is encoded as 21 bits: the MSB is 0 for +ve latitude, 1 for -ve latitude (SOUTH); the unit is 0.0001 degrees. (Note: this is not two's complement!)
  Lon is encoded as 22 bits: the MSB is 0 for +ve longitude, 1 for -ve longitude (WEST); the unit is 0.0001 degrees. (Note: this is not two's complement!)

  Please log in to ARGOS Web https://argos-system.cls.fr/argos-cwi2/login.html
  and copy and paste the latest Satellite AOP (Adapted Orbit Parameters)
  into AOP below.

  From KINEIS-MU-2019-0094:
  Even though most of the satellites are maintained on their orbit thanks to maneuver capability (propulsion), they still drift with time
  because of the solar activity. The linear time margin parameter compensates for the drift by adding extra time to the computed satellite
  passes, allowing to use the same AOP data for up to 6 months, but the resulting passes have a much greater duration. Therefore, it is
  recommended to update the AOP at maximum every 2 or 3 months for the computation to be as accurate as possible and limit the time margin
  contribution in the satellite passes output calculations.

  Please see the ContinuousReceiveWithAOPParsing example to see how to download the AOP from the satellites themselves.

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

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

// From v1.1.0 of the library, the platform ID is stored in PMEM

const uint32_t repetitionPeriod = 90; // The delay in seconds between transmits a.k.a. the repetition period (CLS will have told you what your repetition period should be)
const uint8_t numberTransmits = 5; // The number of transmit attempts for each pass (** Make sure this is >= 1 **)
const uint32_t tcxoWarmupTime = 10; // Start the transmit this many seconds early to compensate for the TCXO warmup time

const uint8_t numARGOSsatellites = 8; // Change this if required to match the number of satellites in the AOP

// Copy and paste the latest AOP from ARGOS Web between the quotes and then carefully delete the line feeds
// Check the alignment afterwards - make sure that the satellite identifiers still line up correctly (or convertAOPtoParameters will go horribly wrong!)
// Check the alignment: " A1 6 0 0 1 2020 10 17 23 45 54  6891.715  97.4600   89.939  -23.755   95.0198  -2.04 MA A 5 3 0 2020 10 17 23 17 28  7195.659  98.5078  318.195  -25.342  101.3611   0.00 MB 9 3 0 0 2020 10 17 22 50 39  7195.586  98.7164  339.849  -25.339  101.3590   0.00 MC B 7 3 0 2020 10 17 22  3  0  7195.670  98.7232  352.079  -25.340  101.3608   0.00 15 5 0 0 0 2020 10 17 22 41 11  7180.481  98.7069  309.136  -25.259  101.0405  -0.11 18 8 0 0 0 2020 10 17 22  2 34  7226.005  99.0303  351.904  -25.498  102.0006  -0.80 19 C 6 0 0 2020 10 17 22 20 53  7226.397  99.1943  298.377  -25.499  102.0084  -0.51 SR D 4 3 0 2020 10 17 22 34 12  7160.232  98.5409  110.208  -25.154  100.6145  -0.12";
const char AOP[] =      " A1 6 0 0 1 2021 10 22 22  4 22  6890.361  97.4669  114.927  -23.748   94.9919  -4.55 MA A 5 3 0 2021 10 22 22 19 59  7195.503  98.4587  323.467  -25.341  101.3580   0.00 MB 9 3 0 0 2021 10 22 21 55 40  7195.600  98.7004  353.821  -25.340  101.3593   0.00 MC B 7 3 0 2021 10 22 22 48 12  7195.606  98.7207  340.689  -25.340  101.3594   0.00 15 5 0 0 0 2021 10 22 22  9 58  7180.144  98.6715  319.198  -25.258  101.0335  -0.65 18 8 0 0 0 2021 10 22 20 49 30  7225.632  98.9820   17.278  -25.497  101.9928  -0.94 19 C 6 0 0 2021 10 22 21 56 16  7226.119  99.1755  319.119  -25.498  102.0026  -0.66 SR D 4 3 0 2021 10 22 23 19 35  7160.106  98.5431   98.627  -25.153  100.6119  -0.28";

// Minimum satellite elevation (above the horizon):
//  Set this to 5 to 20 degrees if you have a clear view to the horizon.
//  45 degrees is really only suitable for urban environments and will severely limit the number of transmit windows...
float min_elevation = 15.0;

#include <SPI.h>

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

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

// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to jump between any of the steps
enum {
  configure_ARTIC,     // Configure the ARTIC (set the satellite detection timeout and TX mode)
  wait_for_GPS,        // Wait for the GPS time and position to be valid
  calculate_next_pass, // Read the GPS time, lat and lon. Calculate the next satellite pass
  wait_for_next_pass,  // Wait for the next satellite pass
  ARTIC_TX,            // Start the ARTIC TX
  wait_for_ARTIC_TX,   // Wait for the ARTIC to transmit
} loop_steps;
int loop_step = configure_ARTIC; // Make sure loop_step is set to configure_ARTIC

// AS3-SP-516-274-CNES specifies a ±10% 'jitter' on the repetition period to reduce the risk of transmission collisions
uint32_t nextTransmitTime; // Time of the next satellite transmission (before jitter is added)
uint32_t nextTransmitTimeActual; // Actual time of the next satellite transmission (including jitter)
uint8_t remainingTransmits; // Remaining number of satellite transmits
boolean firstTransmit; // Flag to indicate if this is the first transmission on this satellite pass
float lat_tx; // The latitude included in the transmitted message
float lon_tx; // The longitude included in the transmitted message

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));
  Serial.println();

  Wire.begin();

  Serial.println(F("Starting the u-blox GPS module..."));
  Serial.println();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ; // Do nothing more
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGPS.factoryDefault(); delay(5000);

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the current ioPortsettings to flash and BBR

  SPI.begin();

  // Uncomment the next line to enable the helpful debug messages
  //myARTIC.enableDebugging(); // Enable debug messages to Serial

  Serial.println(F("Starting the ARTIC R2..."));
  Serial.println();

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

  // From v1.1.0: we were instructed by Kineis to ensure the Platform ID was written into each module
  // and not stored in a configuration file accessible to standard users. To comply with this, SparkFun
  // ARTIC R2 boards are now shipped with the Platform ID programmed into PMEM. Customers who have
  // earlier versions of the board will need to use version 1.0.9 of the library.
  uint32_t platformID = myARTIC.readPlatformID();
  if (platformID == 0)
  {
    Serial.println(F("You appear to have an early version of the SparkFun board."));
    Serial.println(F("Please use the Library Manager to select version 1.0.9 of this library."));
    Serial.println(F("Freezing..."));
    while (1)
      ; // Do nothing more
  }
  else
  {
    Serial.print(F("Your Platform ID is: 0x"));
    Serial.println(platformID, HEX);
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

      // Set the TX mode to ARGOS 3 PTT-A3
      ARTIC_R2_MCU_Command_Result result = myARTIC.sendConfigurationCommand(CONFIG_CMD_SET_PTT_A3_TX_MODE);
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

      // Set the ARGOS 3 TX frequency to 401.630 MHz
      // From AS3-SP-516-274-CNES:
      // The transmission frequency for PTT/PMT-A3 platforms shall be set between 399.91 MHz to 401.68 MHz.
      // Due to frequency regulations, the frequency ranges [400.05 MHz to 401.0 MHz] and [401.2 MHz to 401.3 MHz] are forbidden for VLD-A4 transmissions.
      if (myARTIC.setARGOS23TxFrequency(401.630) == false)
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

      loop_step = wait_for_GPS; // Move on
    }
    break;

    // ************************************************************************************************
    // Wait for the GPS time to be valid and for the position fix to be 3D
    case wait_for_GPS:
    {
      delay(250); // Let's not pound the u-blox too hard...

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
    }
    break;

    // ************************************************************************************************
    // Read the AOP
    // Read the time, latitude and longitude from GPS
    // Calculate the time of the next satellite pass
    case calculate_next_pass:
    {
      // Read the AOP, convert into bulletin_data_t
      bulletin_data_t satelliteParameters[numARGOSsatellites]; // Create an array of bulletin_data_t to hold the parameters for all satellites
      if (myARTIC.convertAOPtoParameters(AOP, satelliteParameters, numARGOSsatellites) == false)
      {
        Serial.println("convertAOPtoParameters failed! Freezing...");
        while (1)
          ; // Do nothing more
      }

/*
      // Pretty-print the AOP for all satellites
      for (uint8_t i = 0; i < numARGOSsatellites; i++)
      {
        myARTIC.printAOPbulletin(satelliteParameters[i]);
        Serial.println();
      }
*/

      // Read the GPS time, latitude and longitude. Convert to epoch.
      uint32_t epochNow = myARTIC.convertGPSTimeToEpoch(myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()); // Convert GPS date & time to epoch
      Serial.print(F("GPS time is: "));
      Serial.print(myARTIC.convertEpochToDateTime(epochNow));
      Serial.println(F(" UTC"));
      Serial.print(F("The number of seconds since the epoch is: "));
      Serial.println(epochNow);

      // Read the GPS lat and lon. Convert to float.
      float lat = ((float)myGPS.getLatitude()) / 10000000; // Convert from degrees^-7
      float lon = ((float)myGPS.getLongitude()) / 10000000; // Convert from degrees^-7

      // Print the lat and lon
      Serial.print(F("GPS Latitude is: "));
      Serial.println(lat, 4);
      Serial.print(F("GPS Longitude is: "));
      Serial.println(lon, 4);

      // Predict the next satellite pass
      uint32_t nextSatellitePass = myARTIC.predictNextSatellitePass(satelliteParameters, min_elevation, numARGOSsatellites, lon, lat, epochNow);

      // Print the prediction
      Serial.print(F("The middle of the next satellite pass will be at: "));
      Serial.print(myARTIC.convertEpochToDateTime(nextSatellitePass));
      Serial.println(F(" UTC"));
      Serial.print(F("The number of seconds since the epoch will be: "));
      Serial.println(nextSatellitePass);

      if (numberTransmits >= 1)
      {
        nextTransmitTime = nextSatellitePass - (((numberTransmits - 1) / 2) * repetitionPeriod);
        nextTransmitTimeActual = nextTransmitTime + random((-0.1 * repetitionPeriod), (0.1 * repetitionPeriod));
        nextTransmitTimeActual -= tcxoWarmupTime; // Start the transmit early to compensate for the TCXO warmup time
        remainingTransmits = numberTransmits; // Remaining number of satellite transmits

        //nextTransmitTimeActual = epochNow + 10; // Uncomment this line if you want to test Tx as soon as possible
      }
      else
      {
        remainingTransmits = 0; // Remaining number of satellite transmits
      }

      // If transmits should have already started (i.e. nextTransmitTime < epochNow)
      // then add repetitionPeriod to nextTransmitTime and decrement remainingTransmits
      // to avoid violating the repetitionPeriod on the next transmit
      while ((remainingTransmits > 0) && (nextTransmitTime < epochNow))
      {
        nextTransmitTime += repetitionPeriod;
        nextTransmitTimeActual = nextTransmitTime; // Do not subtract the jitter or tcxoWarmup as we do not want the next transmit to be in the past
        remainingTransmits--;
      }

      if (remainingTransmits >= 1)
      {
        Serial.print(F("Transmit attempt 1 of "));
        Serial.print(remainingTransmits);
        Serial.print(F(" will take place at: "));
        Serial.print(myARTIC.convertEpochToDateTime(nextTransmitTimeActual));
        Serial.println(F(" UTC"));

        firstTransmit = true; // Set the firstTransmit flag

        loop_step = wait_for_next_pass; // Move on
      }
      else
      {
        Serial.println(F("The transmission window was missed. Recalculating..."));
        Serial.println();
        // Leave loop_step unchanged so the next pass is recalculated
      }
    }
    break;

    // ************************************************************************************************
    // Wait until the next satellite pass
    case wait_for_next_pass:
    {
      delay(250); // Let's not pound the u-blox too hard...

      // Read the GPS time
      uint32_t epochNow = myARTIC.convertGPSTimeToEpoch(myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()); // Convert GPS date & time to epoch

      // Calculate how many seconds remain until the next transmit
      int32_t secsRemaining = (int32_t)nextTransmitTimeActual - (int32_t)epochNow;

      // Count down in intervals of 100, then 10, then 1 second
      if (((secsRemaining >= 100) && (secsRemaining % 100 == 0)) ||
        ((secsRemaining < 100) && (secsRemaining % 10 == 0)) ||
        (secsRemaining < 10))
      {
        Serial.print(F("Transmit will take place in "));
        Serial.print(secsRemaining);
        Serial.print(F(" second"));
        if (secsRemaining != 1) // Attention to detail is everything... :-)
          Serial.println(F("s"));
        else
          Serial.println();
      }

      // Check for a GPS time glitch
      // Stay in wait_for_next_pass if secsRemaining < -15
      if (secsRemaining < -15)
      {
        Serial.println(F("GPS time glitch? Ignoring..."));
      }
      // Check if we should start the TX
      else if (secsRemaining <= 0)
      {
        Serial.println();
        Serial.println(F("*** STARTING TX ***"));
        Serial.println();
        loop_step = ARTIC_TX; // Move on
      }
    }
    break;

    // ************************************************************************************************
    // Start the ARTIC in Transmit One Package And Go Idle mode
    case ARTIC_TX:
    {
      // Update the GPS lat and lon - in case we have moved since we calculated the next pass
      // But only if firstTransmit is true as we want to send the exact same data on each satellite pass
      if (firstTransmit == true)
      {
        lat_tx = ((float)myGPS.getLatitude()) / 10000000; // Convert from degrees^-7
        lat_tx = ((float)myGPS.getLatitude()) / 10000000; // Read the lat twice to ensure we have fresh data
        lon_tx = ((float)myGPS.getLongitude()) / 10000000; // Convert from degrees^-7

        // Print the lat and lon
        Serial.print(F("GPS Latitude is: "));
        Serial.println(lat_tx, 4);
        Serial.print(F("GPS Longitude is: "));
        Serial.println(lon_tx, 4);
        Serial.println();

        // Configure the Tx payload for ARGOS 3 PTT-A3 using our platform ID and the latest lat/lon
        // From v1.1.0 the Platform ID is stored in PMEM
        if (myARTIC.setPayloadARGOS3LatLon(lat_tx, lon_tx) == false)
        {
          Serial.println(F("setPayloadARGOS3LatLon failed!"));
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

        firstTransmit = false; // Clear the firstTransmit flag
      }

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

        remainingTransmits--; // Decrement the remaining number of satellite transmits
        if (remainingTransmits > 0) // Are we done?
        {
          nextTransmitTime += repetitionPeriod;
          nextTransmitTimeActual = nextTransmitTime + random((-0.1 * repetitionPeriod), (0.1 * repetitionPeriod));
          nextTransmitTimeActual -= tcxoWarmupTime; // Start the transmit early to compensate for the TCXO warmup time
          loop_step = wait_for_next_pass; // Wait for next transmit
        }
        else
        {
          Serial.println();
          Serial.println("All transmission attempts are complete!");
          Serial.println();
          Serial.println("Calculating next TX window...");
          Serial.println();
          loop_step = wait_for_GPS; // Do over...
        }
      }
    }
    break;
  }
}
