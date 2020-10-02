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

*/

#include <time.h> // Needed to calculate the epoch

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));

  myARTIC.enableDebugging(); // Enable debug messages to Serial

/*  
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

  // Run the pass predictor for METOP-A
  // Satellite bulletin is a struct which contains the:
  // satellite identifier; time of the orbit bulletin; and the six orbit parameters
  // (Create an array of these if number_sat is > 1)
  bulletin_data_t sat_bulletin; 
  
  sat_bulletin.sat[0] = 'M'; // satellite (char[2])
  sat_bulletin.sat[1] = 'A';

  // Time of the orbit bulletin (uint32_t)
  // This is defined in seconds-since-the-epoch
  // So, for the above example, we need to convert 2020/10/1 22:07:29 into seconds
  // The original time library can do this for us:
  struct tm t;
  time_t t_of_day;

  t.tm_year = 2020-1900;  // Year - 1900
  t.tm_mon = 9;           // Month, *** where 0 = jan ***
  t.tm_mday = 1;          // Day of the month
  t.tm_hour = 22;
  t.tm_min = 7;
  t.tm_sec = 29;
  t.tm_isdst = 0;         // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);

  sat_bulletin.time_bulletin = (long)t_of_day;

  // For the above example (2020/10/1 22:07:29), the answer should be 1601590049
  // https://www.epochconverter.com/
  Serial.print(F("The time of the orbit bulletin in seconds since the epoch is "));
  Serial.println(sat_bulletin.time_bulletin);

  // The six orbit parameters (for the above example)
  sat_bulletin.params[0] = 7195.569; // Semi-major axis                (code notation: dga)     (km)    (float)
  sat_bulletin.params[1] = 98.5114;  // Orbit inclination              (code notation: inc)     (deg)   (float)
  sat_bulletin.params[2] = 336.036;  // Ascending node longitude       (code notation: lon_asc) (deg)   (float)
  sat_bulletin.params[3] = -25.341;  // Ascending node longitude drift (code notation: d_noeud) (deg)   (float)
  sat_bulletin.params[4] = 101.3592; // Orbital period                 (code notation: ts)      (min)   (float)
  sat_bulletin.params[5] = 0.00;     // Semi-major axis drift          (code notation: gap)     (m/day) (float)
  
  // Epoch timestamp for the prediction
  // E.g. 2020/10/2 08:00:00 UTC/GMT should be 1601625600
  // https://www.epochconverter.com/
  // https://www.epochconverter.com/programming/c

  t.tm_year = 2020-1900;  // Year - 1900
  t.tm_mon = 9;           // Month, where 0 = jan
  t.tm_mday = 2;          // Day of the month
  t.tm_hour = 8;
  t.tm_min = 0;
  t.tm_sec = 0;
  t.tm_isdst = 0;         // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);
  
  long current_time = (long)t_of_day;

  Serial.print(F("The time of the prediction in seconds since the epoch is "));
  Serial.println(current_time);

  // Prediction parameters
  float min_elevation = 15.0; // Minimum satellite elevation (above the horizon)
  uint8_t number_sat = 1; // Number of satellite
  float lat = 48.8548; // Site latitude (C'est La Tour Eiffel naturellement!)
  float lon = 2.2945;  // Site longitude

  // Calculate the prediction
  // For this example, the answer should be:
  // Satellite | Start date/time | Middle date/time | End date/time | Duration | Middle elevation | Start azimuth | Middle azimuth | End azimuth
  // MA | 02/10/2020 08:48:04 | 02/10/2020 08:52:28 | 02/10/2020 08:56:51 | 00:08:47 | 76.69 | 18.28 | 104.36 | 189.26
  uint32_t predicted_time = myARTIC.predictNextSatellitePass(&sat_bulletin, min_elevation, number_sat, lon, lat, current_time);
  time_t pt_of_day = predicted_time; // Convert to YY/MM/DD HH:MM:SS

  Serial.print(F("Predicted next pass will take place at "));
  Serial.println((long)pt_of_day);

  tm* pt = gmtime(&pt_of_day);

  Serial.print(F("Predicted next pass will take place at "));
  Serial.print(pt->tm_mday);
  Serial.print(F("/"));
  Serial.print(pt->tm_mon + 1); // Jan = 1
  Serial.print(F("/"));
  Serial.print(pt->tm_year + 1900);
  Serial.print(F(" "));
  Serial.print(pt->tm_hour);
  Serial.print(F(":"));
  Serial.print(pt->tm_min);
  Serial.print(F(":"));
  Serial.print(pt->tm_sec);
  if (pt->tm_isdst == 1)
    Serial.print(F(" DST"));
  Serial.println();
}

void loop()
{
  // Nothing to do here...
}
