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

  //myARTIC.enableDebugging(Serial1); // E.g. enable debug messages to Serial1 instead

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
    Semi-major axis
    Inclinaison
    Ascending node longitude
    Ascending node longitude drift
    Orbital period
    Semi-major axis drift

  Example data for METOP-A is:
  
  MA A 5 3 0 2020  9 30 22 28 27  7195.584  98.5120  330.816  -25.341  101.3595   0.00

  The two character satellite identifiers are:
  
  MA : METOP-A
  MB : METOP-B
  MC : METOP-C
  NK : NOAA-15 (appears as 15 in the AOP)
  NN : NOAA-18 (appears as 18 in the AOP)
  NP : NOAA-19 (appears as 19 in the AOP)
  SR : SARAL
  
 */

  // Run the pass predictor for:
  bulletin_data_t sat_bulletin; // Satellite bulletin (create an array of these if number_sat is > 1)
  
  sat_bulletin.sat[0] = 'M'; // satellite (char[2])
  sat_bulletin.sat[1] = 'A';

  // Time of the bulletin (uint32_t)
  sat_bulletin.time_bulletin = 0;
  
  sat_bulletin.params[0] = 7195.584; // dga (float)
  sat_bulletin.params[1] = 98.5120; // inc (float)
  sat_bulletin.params[2] = 330.816; // lon_asc (float)
  sat_bulletin.params[3] = -25.341; // d_noeud (float)
  sat_bulletin.params[4] = 101.3595; // ts (float)
  sat_bulletin.params[5] = 0.00; // gap (float)
  
  float min_elevation = 15.0; // Minimum satellite elevation (above the horizon)
  uint8_t number_sat = 1; // Number of satellite
  float lon = -1.428; // Site longitude
  float lat = 54.866; // Site latitude

  // Calculate the epoch timestamp
  // (https://www.epochconverter.com/)
  // https://www.epochconverter.com/programming/c

  struct tm t;
  time_t t_of_day;

  t.tm_year = 2020-1900;  // Year - 1900
  t.tm_mon = 8;           // Month, where 0 = jan
  t.tm_mday = 30;         // Day of the month
  t.tm_hour = 22;
  t.tm_min = 28;
  t.tm_sec = 27;
  t.tm_isdst = 0;         // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);
  
  long current_time = (long)t_of_day;

  Serial.print(F("AOP: seconds since the epoch is: "));
  Serial.println(current_time);

  t.tm_year = 2020-1900;  // Year - 1900
  t.tm_mon = 9;           // Month, where 0 = jan
  t.tm_mday = 1;          // Day of the month
  t.tm_hour = 16;
  t.tm_min = 00;
  t.tm_sec = 00;
  t.tm_isdst = 1;         // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);
  
  current_time = (long)t_of_day;

  Serial.print(F("Seconds since the epoch is: "));
  Serial.println(current_time);

  uint32_t predicted_time = myARTIC.next_predict(&sat_bulletin, min_elevation, number_sat, lon, lat, current_time);

  Serial.print(F("Predicted time is: "));
  Serial.println(predicted_time);

  time_t pt_of_day = predicted_time;

  Serial.print(F("Predicted time is: "));
  Serial.println((long)pt_of_day);

  tm* pt = gmtime(&pt_of_day);

  Serial.print(F("Predicted time is: "));
  Serial.print(pt->tm_year + 1900);
  Serial.print(F("  "));
  Serial.print(pt->tm_mon + 1); // Jan = 1
  Serial.print(F("  "));
  Serial.print(pt->tm_mday);
  Serial.print(F("  "));
  Serial.print(pt->tm_hour);
  Serial.print(F("  "));
  Serial.print(pt->tm_min);
  Serial.print(F("  "));
  Serial.print(pt->tm_sec);
  Serial.print(F("   pt->tm_isdst = "));
  Serial.print(pt->tm_isdst);
}

void loop()
{
  // Nothing to do here...
}
