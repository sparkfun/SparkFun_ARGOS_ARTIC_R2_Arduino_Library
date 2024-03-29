/*
  Using the SparkFun ARGOS ARTIC R2 Breakout & IOTA
  By: Paul Clark
  SparkFun Electronics
  Date: June 8th 2021

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
    clears INT1 each time a message is downloaded.
  LED_BUILTIN will illuminate when a satellite is detected.
  Messages are parsed for UTC time and AOP data.

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
  pinMode(LED_BUILTIN, OUTPUT); // Turn the LED off
  digitalWrite(LED_BUILTIN, LOW);

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

  // From v1.1.0: we were instructed by Kineis to ensure the Platform ID was written into each module
  // and not stored in a configuration file accessible to standard users. To comply with this, SparkFun
  // ARTIC R2 boards are now shipped with the Platform ID programmed into PMEM. Customers who have
  // earlier versions of the board will need to use version 1.0.9 of the library.
  uint32_t platformID = myARTIC.readPlatformID();
  if (platformID == 0)
  {
    Serial.println(F("You appear to have an early version of the SparkFun board."));
    Serial.println(F("For the transmit examples, you will need to use the Library Manager to select version 1.0.9 of this library."));
  }
  else
  {
    Serial.print(F("Your Platform ID is: 0x"));
    Serial.println(platformID, HEX);
  }

  // Read the ARTIC R2 status register
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

      // Parse for UTC allcast
      // Taken from KINEIS-MU-2019-0094 and AS3-SP-516-2095-CNES
      if ((downlinkMessage.addresseeIdentification == 0x00000E1) && (downlinkMessage.service == 0x08))
      {
        Serial.print(F("UTC allcast received: Year="));
        Serial.print(downlinkMessage.payload[0] >> 4); Serial.print(downlinkMessage.payload[0] & 0x0F);
        Serial.print(downlinkMessage.payload[1] >> 4); Serial.print(downlinkMessage.payload[1] & 0x0F);
        Serial.print(F(" Day of year="));
        Serial.print(downlinkMessage.payload[2] >> 4); Serial.print(downlinkMessage.payload[2] & 0x0F);
        Serial.print(downlinkMessage.payload[3] >> 4);
        Serial.print(F(" Time="));
        Serial.print(downlinkMessage.payload[3] & 0x0F); Serial.print(downlinkMessage.payload[4] >> 4);
        Serial.print(F(":"));
        Serial.print(downlinkMessage.payload[4] & 0x0F); Serial.print(downlinkMessage.payload[5] >> 4);
        Serial.print(F(":"));
        Serial.print(downlinkMessage.payload[5] & 0x0F); Serial.print(downlinkMessage.payload[6] >> 4);
        Serial.print(F("."));
        Serial.print(downlinkMessage.payload[6] & 0x0F); Serial.print(downlinkMessage.payload[7] >> 4);
        Serial.println(downlinkMessage.payload[7] & 0x0F);
      }

      // Parse for AOP data
      // Taken from KINEIS-MU-2019-0094 and AS3-SP-516-2095-CNES
      if ((downlinkMessage.addresseeIdentification == 0x00000BE) && (downlinkMessage.service == 0x00))
      {
        bulletin_data_t bulletin; // Assemble the data as a bulletin_data_t which could then be used for satellite pass prediction (see the later examples)

        Serial.print(F("Satellite orbit parameters received: Satellite ID="));
        switch (downlinkMessage.payload[0] >> 4)
        {
          // The full list from KINEIS-MU-2019-0094 is as follows:
          // 0x1 CDARS
          // 0x2 OCEANSAT-3
          // 0x3 METOP-SG1B
          // 0x4 METOP-SG2B
          // 0x5 NOAA K
          // 0x6 ANGELS
          // 0x8 NOAA N
          // 0x9 METOP B
          // 0xA METOP A
          // 0xB METOP C
          // 0xC NOAA N' ( = NP)
          // 0xD SARAL

          // Process the 'known' satellites
          // The two character satellite identifiers are:
          // A1 : ANGELS
          // MA : METOP-A
          // MB : METOP-B
          // MC : METOP-C
          // NK : NOAA-15 (appears as 15 in the AOP)
          // NN : NOAA-18 (appears as 18 in the AOP)
          // NP : NOAA-19 (appears as 19 in the AOP)
          // SR : SARAL

          case 0x6:
            Serial.println(F("ANGELS"));
            bulletin.sat[0] = 'A'; bulletin.sat[1] = '1';
            break;
          case 0xA:
            Serial.println(F("METOP-A"));
            bulletin.sat[0] = 'M'; bulletin.sat[1] = 'A';
            break;
          case 0x9:
            Serial.println(F("METOP-B"));
            bulletin.sat[0] = 'M'; bulletin.sat[1] = 'B';
            break;
          case 0xB:
            Serial.println(F("METOP-C"));
            bulletin.sat[0] = 'M'; bulletin.sat[1] = 'C';
            break;
          case 0x5:
            Serial.println(F("NOAA K"));
            bulletin.sat[0] = '1'; bulletin.sat[1] = '5';
            break;
          case 0x8:
            Serial.println(F("NOAA N"));
            bulletin.sat[0] = '1'; bulletin.sat[1] = '8';
            break;
          case 0xC:
            Serial.println(F("NOAA N\'"));
            bulletin.sat[0] = '1'; bulletin.sat[1] = '9';
            break;
          case 0xD:
            Serial.println(F("SARAL"));
            bulletin.sat[0] = 'S'; bulletin.sat[1] = 'R';
            break;
          default: // We don't know the two character ID for the other satellites!
            Serial.print(F("0x"));
            Serial.println((downlinkMessage.payload[0] >> 4), HEX);
            bulletin.sat[0] = '?'; bulletin.sat[1] = '?';
            break;
        }

        // Extract the date and time of the bulletin
        uint16_t year = 2000;
        year += (((downlinkMessage.payload[0] & 0x03) << 2) | (downlinkMessage.payload[1] >> 6)) * 10;
        year += ((downlinkMessage.payload[1] & 0x3C) >> 2);
        uint16_t day_of_year = (((downlinkMessage.payload[1] & 0x03) << 2) | (downlinkMessage.payload[2] >> 6)) * 100;
        day_of_year += ((downlinkMessage.payload[2] & 0x3C) >> 2) * 10;
        day_of_year += (((downlinkMessage.payload[2] & 0x03) << 2) | (downlinkMessage.payload[3] >> 6));
        uint8_t hour = ((downlinkMessage.payload[3] & 0x3C) >> 2) * 10;
        hour += (((downlinkMessage.payload[3] & 0x03) << 2) | (downlinkMessage.payload[4] >> 6));
        uint8_t minute = ((downlinkMessage.payload[4] & 0x3C) >> 2) * 10;
        minute += (((downlinkMessage.payload[4] & 0x03) << 2) | (downlinkMessage.payload[5] >> 6));
        uint8_t second = ((downlinkMessage.payload[5] & 0x3C) >> 2) * 10;
        second += (((downlinkMessage.payload[5] & 0x03) << 2) | (downlinkMessage.payload[6] >> 6));
        // Convert to seconds from the epoch - using Jan 1st
        uint32_t time_bulletin = myARTIC.convertGPSTimeToEpoch(year, 1, 1, hour, minute, second);
        time_bulletin += ((uint32_t)day_of_year - 1) * 24 * 60 * 60; // Now add the day of year (remembering that Jan 1st is day 1, not day 0)
        bulletin.time_bulletin = time_bulletin; // Store it

        // Extract the 19-bit ascending node longitude (deg)
        // Longitude of the ascending node : the ascending node (terrestrial ascending node) is the
        // point where the satellite ground track intersects the equatorial plane on the northbound crossing.
        uint32_t anl = (((uint32_t)(downlinkMessage.payload[6] & 0x3F)) << 13);
        anl |= ((uint32_t)downlinkMessage.payload[7]) << 5;
        anl |= ((uint32_t)downlinkMessage.payload[8]) >> 3;
        float anl_f = ((float)anl) / 1000; // Defined in AS3-SP-516-2095-CNES
        bulletin.params[2] = anl_f; // Store it

        // Extract the 10-bit ascending node longitude drift (deg)
        // Angular separation between two successive ascending nodes
        uint32_t anld = (((uint32_t)(downlinkMessage.payload[8] & 0x07)) << 7);
        anld |= ((uint32_t)downlinkMessage.payload[9]) >> 1;
        float anld_f = (((float)anld) / 1000) - 26; // Defined in AS3-SP-516-2095-CNES
        bulletin.params[3] = anld_f; // Store it

        // Extract the 14-bit orbital period (min)
        // Nodal period : elapsed time between two successive ascending node passes
        uint32_t op = (((uint32_t)(downlinkMessage.payload[9] & 0x01)) << 13);
        op |= ((uint32_t)downlinkMessage.payload[10]) << 5;
        op |= ((uint32_t)downlinkMessage.payload[11]) >> 3;
        float op_f = (((float)op) / 1000) + 95; // Defined in AS3-SP-516-2095-CNES
        bulletin.params[4] = op_f; // Store it

        // Extract the 19-bit semi-major axis (km)
        // Semi-major axis : distance from the apogee (point farthest from the earth) to the center of
        // the earth.
        // (ANGELS A1's orbit is less than 7000km! I guess ANGELS will need a new definition?)
        uint32_t sma = (((uint32_t)(downlinkMessage.payload[11] & 0x07)) << 16);
        sma |= ((uint32_t)downlinkMessage.payload[12]) << 8;
        sma |= ((uint32_t)downlinkMessage.payload[13]);
        float sma_f = (((float)sma) / 1000) + 7000; // Defined in AS3-SP-516-2095-CNES
        bulletin.params[0] = sma_f; // Store it

        // Extract the 8-bit semi-major axis drift (m/day)
        // Semi-major axis decay : semi-major axis first derivative
        uint32_t smad = ((uint32_t)(downlinkMessage.payload[14]));
        float smad_f = 0 - (((float)smad) / 10); // Defined in AS3-SP-516-2095-CNES
        bulletin.params[5] = smad_f; // Store it

        // Extract the 16-bit orbit inclination (deg)
        // Inclination : angle between the plane of the satellite orbit and the earth's equatorial plane
        uint32_t oi = (((uint32_t)(downlinkMessage.payload[15])) << 8);
        oi |= ((uint32_t)downlinkMessage.payload[16]);
        float oi_f = (((float)oi) / 10000) + 97; // Defined in AS3-SP-516-2095-CNES
        bulletin.params[1] = oi_f; // Store it

        // Pretty-print the bulletin to Serial
        myARTIC.printAOPbulletin(bulletin);
      }
    }
    else
    {
      Serial.println(F("readDownlinkMessage failed!"));
    }

    // Manually clear INT1 now that the message has been downloaded. This will clear the RX_VALID_MESSAGE flag too.
    // *** INT1 will go high again after 100us if there is another message to be read (which could cause clearInterrupts to return false) ***
    Serial.println();
    Serial.println(F("Clearing INT1."));
    if (myARTIC.clearInterrupts(1) == false)
    {
      Serial.println("clearInterrupts may have failed!");
    }
  }
}
