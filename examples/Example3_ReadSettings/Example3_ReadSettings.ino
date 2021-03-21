/*
  Using the SparkFun ARGOS ARTIC R2 Breakout & IOTA
  By: Paul Clark
  SparkFun Electronics
  Date: March 21st 2021

  This example begins (initializes) the ARTIC breakout or IOTA and then reads and prints a bunch of settings.

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
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));
  Serial.println();

  Serial.println(F("ARTIC R2 is booting..."));
  Serial.println();

  SPI.begin();

  //myARTIC.enableDebugging(); // Enable debug messages to Serial
  //myARTIC.enableDebugging(Serial1); // E.g. enable debug messages to Serial1 instead

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
  Serial.println();

  // Read and print the ARGOS configuration
  ARGOS_Configuration_Register configuration;
  myARTIC.readARGOSconfiguration(&configuration);
  myARTIC.printARGOSconfiguration(configuration); // Pretty-print the TX and RX configuration to Serial

  uint32_t rxTimeout = myARTIC.readRxTimeout();
  Serial.print(F("The RX Timeout is "));
  Serial.print(rxTimeout);
  Serial.println(F(" seconds."));

  uint32_t detectionTimeout = myARTIC.readSatelliteDetectionTimeout();
  Serial.print(F("The Satellite Detection Timeout is "));
  Serial.print(detectionTimeout);
  Serial.println(F(" seconds."));

  uint32_t warmupTime = myARTIC.readTCXOWarmupTime();
  Serial.print(F("The TCXO Warmup Time is "));
  Serial.print(warmupTime);
  Serial.println(F(" seconds."));

  uint32_t certificationInterval = myARTIC.readTxCertificationInterval();
  Serial.print(F("The TX Certification Interval is "));
  Serial.print(certificationInterval);
  Serial.println(F(" seconds."));

  float controlVoltage = myARTIC.readTCXOControlVoltage();
  Serial.print(F("The TCXO Control Voltage is "));
  Serial.print(controlVoltage, 2);
  Serial.println(F(" Volts."));

  boolean autoDisable = myARTIC.readTCXOAutoDisable();
  Serial.print(F("TCXO Auto Disable is "));
  if (autoDisable == false) Serial.print(F("not "));
  Serial.println(F("enabled."));

  float tx23freq = myARTIC.getARGOS23TxFrequency();
  Serial.print(F("The ARGOS 2/3 TX Frequency is "));
  Serial.print(tx23freq, 3);
  Serial.println(F(" MHz."));

  float tx4freq = myARTIC.getARGOS4TxFrequency();
  Serial.print(F("The ARGOS 4 TX Frequency is "));
  Serial.print(tx4freq, 3);
  Serial.println(F(" MHz."));

  boolean rxCRCenabled = myARTIC.isRXCRCenabled();
  Serial.print(F("The RX CRC is "));
  if (rxCRCenabled == false) Serial.print(F("not "));
  Serial.println(F("enabled."));

  int lutLength = myARTIC.getAddressLUTlength();
  Serial.print(F("The Address-Filtering Look-Up-Table length is "));
  Serial.print(lutLength);
  Serial.println(F("."));

  boolean rxTransparentMode = myARTIC.isRXTransparentModeEnabled();
  Serial.print(F("RX Transparent Mode is "));
  if (rxTransparentMode == false) Serial.print(F("not "));
  Serial.println(F("enabled."));
}

void loop()
{
  // Nothing to do here...
}
