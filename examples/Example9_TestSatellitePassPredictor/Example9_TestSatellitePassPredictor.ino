/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: September 29th 2020

  This example tests the satellite pass predictor.
  The predictor was originally written by CLS.
  This code is gratefully based on the version in Arribada Horizon.

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/


  You can download the Satellite AOP (Adapted Orbit Parameters) from https://argos-system.cls.fr/argos-cwi2/login.html
  
  The AOP file contains the following fields separated with spaces :
    Satellite identifier (number for NOAA spacecraft)
    Satellite identifier into the downlink allcast data
    First digit of downlink identifier of the first instrument associated to satellite. (0 if no downlink)
    Downlink operating status
      0=OFF
      1=ON
      3=ON (ARGOS-3)
      4=ON (ARGOS-4)
    Argos Instrument Operating status
      0= OPERATIONAL ARGOS-2/3
      1= OPERATIONAL ARGOS-NEO
      2= OPERATIONAL ARGOS-4
      3= OUTOFSERVICE or OTHER
    Date : YYYY MM DD HH mm ss (Y = year, M = month, d = day, H = hours, m = minutes, s = seconds)
    Semi-major axis (km)
    Orbit inclination (deg)
    Ascending node longitude (deg)
    Ascending node longitude drift (deg)
    Orbital period (min)
    Semi-major axis drift (m/day)

  Example data for METOP-A for October 2nd 2020 is:
  
  MA A 5 3 0 2020 10  1 22  7 29  7195.569  98.5114  336.036  -25.341  101.3592   0.00

  The two character satellite identifiers are:
  
  MA : METOP-A
  MB : METOP-B
  MC : METOP-C
  NK : NOAA-15 (appears as 15 in the AOP)
  NN : NOAA-18 (appears as 18 in the AOP)
  NP : NOAA-19 (appears as 19 in the AOP)
  SR : SARAL
  
 */

const uint8_t numARGOSsatellites = 7; // Change this if required to match the number of satellites in the AOP

// Copy and paste the latest AOP from ARGOS Web between the quotes and then carefully delete the line feeds
// Check the alignment afterwards - make sure that the satellite identifiers still ine up correctly (or convertAOPtoParameters will go horribly wrong!)
// Check the alignment: " MA A 5 3 0 2020 10  1 22  7 29  7195.569  98.5114  336.036  -25.341  101.3592   0.00 MB 9 3 0 0 2020 10  1 23 21 58  7195.654  98.7194  331.991  -25.340  101.3604   0.00 MC B 7 3 0 2020 10  1 22 34 23  7195.569  98.6883  344.217  -25.340  101.3587   0.00 15 5 0 0 0 2020 10  1 22 44 11  7180.495  98.7089  308.255  -25.259  101.0408   0.00 18 8 0 0 0 2020 10  1 21 50 32  7225.981  99.0331  354.556  -25.498  102.0000  -0.79 19 C 6 0 0 2020 10  1 22  7  6  7226.365  99.1946  301.174  -25.499  102.0077  -0.54 SR D 4 3 0 2020 10  1 22 33 38  7160.233  98.5416  110.362  -25.154  100.6146  -0.12";
const char AOP[] =      " MA A 5 3 0 2020 10  1 22  7 29  7195.569  98.5114  336.036  -25.341  101.3592   0.00 MB 9 3 0 0 2020 10  1 23 21 58  7195.654  98.7194  331.991  -25.340  101.3604   0.00 MC B 7 3 0 2020 10  1 22 34 23  7195.569  98.6883  344.217  -25.340  101.3587   0.00 15 5 0 0 0 2020 10  1 22 44 11  7180.495  98.7089  308.255  -25.259  101.0408   0.00 18 8 0 0 0 2020 10  1 21 50 32  7225.981  99.0331  354.556  -25.498  102.0000  -0.79 19 C 6 0 0 2020 10  1 22  7  6  7226.365  99.1946  301.174  -25.499  102.0077  -0.54 SR D 4 3 0 2020 10  1 22 33 38  7160.233  98.5416  110.362  -25.154  100.6146  -0.12";

#include <time.h> // Needed to calculate the epoch

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));

  myARTIC.enableDebugging(); // Enable debug messages to Serial

  // Read the AOP, convert into bulletin_data_t
  bulletin_data_t satelliteParameters[numARGOSsatellites]; // Create an array of bulletin_data_t to hold the parameters for all satellites
  myARTIC.convertAOPtoParameters(AOP, satelliteParameters, numARGOSsatellites);

  // Epoch timestamp for the prediction
  // E.g. 2020/10/2 08:00:00 UTC/GMT should be 1601625600
  // https://www.epochconverter.com/
  // https://www.epochconverter.com/programming/c
  
  uint32_t current_time = myARTIC.convertGPSTimeToEpoch(2020, 10, 2, 8, 0, 0);

  Serial.print(F("The time of the prediction in seconds since the epoch is "));
  Serial.println(current_time);
  Serial.print(F("The date and time of the prediction is "));
  Serial.println(myARTIC.convertEpochToDateTime(current_time));

  // Prediction parameters
  float min_elevation = 45.0; // Minimum satellite elevation (above the horizon)
  float lat = 48.8548; // Site latitude (C'est La Tour Eiffel naturellement!)
  float lon = 2.2945;  // Site longitude

  // Calculate the prediction
  // For this example, the answer should be:
  // Satellite | Start date/time | Middle date/time | End date/time | Duration | Middle elevation | Start azimuth | Middle azimuth | End azimuth
  // MA | 02/10/2020 08:48:04 | 02/10/2020 08:52:28 | 02/10/2020 08:56:51 | 00:08:47 | 76.69 | 18.28 | 104.36 | 189.26
  uint32_t predicted_time = myARTIC.predictNextSatellitePass(satelliteParameters, min_elevation, numARGOSsatellites, lon, lat, current_time);
  Serial.print(F("Predicted next pass will take place at "));
  Serial.println(predicted_time);
  Serial.print(F(" = "));
  Serial.print(myARTIC.convertEpochToDateTime(predicted_time));
  Serial.println();
}

void loop()
{
  // Nothing to do here...
}
