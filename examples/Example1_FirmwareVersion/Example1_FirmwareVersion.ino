/*
  Using the SparkFun ARGOS ARTIC R2 Breakout & IOTA
  By: Paul Clark
  SparkFun Electronics
  Date: June 8th 2021

  This example begins (initializes) the ARTIC breakout or IOTA. The pin numbers are passed to begin.
  Begin takes care of setting the PWR_EN pins to enable power for the ARTIC and the RF Amp.
  Begin also controls the BOOT pin and downloads the firmware to the ARTIC.
  Begin returns true if the firmware checksum is valid.

  The ARTIC firmware version is read and printed to Serial.

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

  If you are using the SparkFun ESP32 Thing Plus, the pins are different:
  https://www.sparkfun.com/products/15663
  CS_Pin = A5 (D4)
  GAIN8_Pin = D14
  BOOT_Pin = D32
  INT1_Pin = D15
  INT2_Pin = D33
  RESET_Pin = D27
  ARTIC_PWR_EN_Pin = IOTA_PWR_EN_Pin = D12
  RF_PWR_EN_Pin = D13     ** Please note: D13 on the ESP32 Thing Plus is LED_BUILTIN **
  (SPI COPI = D18)
  (SPI CIPO = D19)
  (SPI SCK = D5)


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


/*
// Pin assignments for the SparkFun ESP32 Thing Plus
// (Change these if required)
int CS_Pin = 4;
int GAIN8_Pin = 14; // Optional. Set to -1 if you don't want to control the gain. The library defaults to maximum power.
int BOOT_Pin = 32;
int INT1_Pin = 15;
int INT2_Pin = 33;
int RESET_Pin = 27;
#ifdef IOTA
int IOTA_PWR_EN_Pin = 12; // IOTA has a single power enable pin
#else
int ARTIC_PWR_EN_Pin = 12; // The ARTIC R2 Breakout has separate enables for the ARTIC and the RF Amplifier
int RF_PWR_EN_Pin = 13; // ** Please note: D13 on the ESP32 Thing Plus is LED_BUILTIN **
#endif
*/

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));
  Serial.println();

  SPI.begin();

  Serial.println(F("ARTIC R2 is booting..."));
  Serial.println();

  //myARTIC.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  //myARTIC.enableDebugging(Serial1); // E.g. enable debug messages on Serial1 instead

  // Begin (initialize) the ARTIC
#ifdef IOTA
  if (myARTIC.beginIOTA(CS_Pin, RESET_Pin, BOOT_Pin, IOTA_PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin) == false)
#else
  if (myARTIC.begin(CS_Pin, RESET_Pin, BOOT_Pin, ARTIC_PWR_EN_Pin, RF_PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin) == false)
#endif
  {
    Serial.println(F("ARTIC R2 not detected. Freezing..."));
    while (1)
      ; // Do nothing more
  }

  Serial.println(F("ARTIC R2 boot was successful."));

  char buffer[9]; // Buffer to store the firmware version (8 bytes + NULL)

  myARTIC.readFirmwareVersion(&buffer[0]); // Read the firmware version from PMEM

  Serial.print(F("ARTIC R2 firmware version is: "));
  Serial.println(buffer);

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
}

void loop()
{
  // Nothing to do here...
}
