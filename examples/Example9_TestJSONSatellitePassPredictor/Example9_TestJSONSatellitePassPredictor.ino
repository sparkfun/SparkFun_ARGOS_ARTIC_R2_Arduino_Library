/*
  Using the ARGOS ARTIC R2 Breakout
  By: Paul Clark
  SparkFun Electronics
  Date: March 6th 2021

  This example tests the satellite pass predictor using JSON-format Satellite Allcast Info.
  The predictor was originally written by CLS.
  This code is gratefully based on the version in Arribada Horizon.

  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/17236

  You can download the Satellite Allcast Info from https://argos-system.cls.fr/argos-cwi2/login.html
  Select System \ Satellite Allcast Info and then click "Download Allcast" to download the orbit parameters etc. in JSON format.
  Use ARTIC_R2_Allcast_Converter.py to convert the format of the .txt file into a .h file which Arduino can compile:
  
  python ARTIC_R2_Allcast_Converter.py allcast_2021_03_06_13_27_618.txt allcast_2021_03_06_13_27_618.h
  
  Update the #include below with the name of your .h file.

  The JSON file is parsed (deserialized) by Benoit Blanchon's ArduinoJson library. Thank you Benoit.

 */

// Include the Allcast JSON Info. Update the filename as required:
char allcast_JSON[] = {
#include "allcast_2021_03_06_13_27_618.h"
};

#include <ArduinoJson.h> // Click here to get the library: http://librarymanager/All#ArduinoJson_deserialization

StaticJsonDocument<3100> allcast; // Increase this if you see DeserializationError::NoMemory errors

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("ARGOS ARTIC R2 Example"));

  myARTIC.enableDebugging(); // Enable debug messages to Serial

  // Deserialize the JSON data
  DeserializationError error = deserializeJson(allcast, allcast_JSON);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed! Error: "));
    Serial.print(error.f_str());
    Serial.println(F(". Freezing..."));
    while (1)
      ; // Do nothing more
  }

  int numARGOSsatellites = extractNumSatellitesFromAllcast(); // Extract the number of satellites from the Allcast Info
  if (numARGOSsatellites == 0) // Check that the number of satellites is > 0
  {
    Serial.println("numARGOSsatellites is zero! Freezing...");
    while (1)
      ; // Do nothing more
  }
  Serial.print(F("allcast_JSON contains data for "));
  Serial.print(numARGOSsatellites);
  Serial.println(F(" satellites"));

  // Extract the satellite orbit parameters from the Allcast Info
  bulletin_data_t satelliteParameters[numARGOSsatellites]; // Create an array of bulletin_data_t to hold the parameters for all satellites
  extractOrbitParametersFromAllcast(numARGOSsatellites, satelliteParameters); // Extract the orbit parameters from the Allcast Info

  // Epoch timestamp for the prediction
  // E.g. 2021/03/06 14:00:00 UTC/GMT should be 1615039200
  // https://www.epochconverter.com/
  // https://www.epochconverter.com/programming/c

  uint32_t current_time = myARTIC.convertGPSTimeToEpoch(2021, 3, 6, 14, 0, 0);

  Serial.print(F("The time of the prediction in seconds since the epoch is "));
  Serial.println(current_time);
  Serial.print(F("The date and time of the prediction is "));
  Serial.println(myARTIC.convertEpochToDateTime(current_time));

  // Prediction parameters
  float min_elevation = 45.0; // Minimum satellite elevation (above the horizon)
  float lat = 48.8548; // Site latitude (C'est La Tour Eiffel naturellement!)
  float lon = 2.2945;  // Site longitude

  // Calculate the prediction
  // For this example, using allcast_2021_03_06_13_27_618.h, the answer should be:
  // Satellite | Start date/time | Middle date/time | End date/time | Duration | Middle elevation | Start azimuth | Middle azimuth | End azimuth
  // A1 | 06/03/2021 18:19:52 | 06/03/2021 18:20:49 | 06/03/2021 18:21:45 | 00:01:53 | 67.37 | 350.46 | 284.87 | 221.39
  uint32_t predicted_time = myARTIC.predictNextSatellitePass(satelliteParameters, min_elevation, numARGOSsatellites, lon, lat, current_time);
  Serial.print(F("Predicted next pass will take place at "));
  Serial.print(predicted_time);
  Serial.print(F(" = "));
  Serial.print(myARTIC.convertEpochToDateTime(predicted_time));
  Serial.println();
}

void loop()
{
  // Nothing to do here...
}

// Extract the number of satellites from the Allcast Info
int extractNumSatellitesFromAllcast()
{
  return(allcast["satellitesInformation"].size()); // Get the size of "satellitesInformation"
}

// Extract the satellite orbit parameters from the Allcast Info
void extractOrbitParametersFromAllcast(int numARGOSsatellites, bulletin_data_t* satelliteParameters)
{
  // For each satellite, extract the orbit parameters
  for (int i = 0; i < numARGOSsatellites; i++)
  {
    satelliteParameters[i].sat[0] = *((const char *)(allcast["satellitesInformation"][i]["satellite"])); // Two-character satellite ID
    satelliteParameters[i].sat[1] = *((const char *)(allcast["satellitesInformation"][i]["satellite"]) + 1);
    satelliteParameters[i].time_bulletin = myARTIC.convertAllcastDateTimeToEpoch((const char *)allcast["satellitesInformation"][i]["aop"]["date"]); // DateTime of the parameters
    satelliteParameters[i].params[0] = (float)allcast["satellitesInformation"][i]["aop"]["semiMajorAxis"]; // Orbit parameters
    satelliteParameters[i].params[1] = (float)allcast["satellitesInformation"][i]["aop"]["inclination"];
    satelliteParameters[i].params[2] = (float)allcast["satellitesInformation"][i]["aop"]["ascendantNodeLongitude"];
    satelliteParameters[i].params[3] = (float)allcast["satellitesInformation"][i]["aop"]["ascendantNodeDrift"];
    satelliteParameters[i].params[4] = (float)allcast["satellitesInformation"][i]["aop"]["orbitalPeriod"];
    satelliteParameters[i].params[5] = (float)allcast["satellitesInformation"][i]["aop"]["semiMajorAxisDrift"];
  }
}
