# This code converts the Satellite Allcast Info .txt file
# into the correct format for ArduinoJson

# All CR and LF characters are removed
# All spaces are removed unless between quotes
# All quotes (") are replaced with \"
# A quote is added at the beginning and end of the file

# Written by Paul Clark, March 6th 2021

# License: please see the license file at:
# https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md

import sys

with open(sys.argv[1], "r") as reader:
    with open(sys.argv[2], "w") as writer:
        char = '"'
        writer.write(char) # Add " at the start of the file
        in_quotes = False
        while (char != ''):
            char = reader.read(1) # Read a single character
            if (char == ''): # Check for NULL (End Of File)
                break
            if (char == '"'): # Check for a quote
                in_quotes = not in_quotes # Toggle in_quotes
                writer.write('\\') # Always convert " into \"
            skip = False
            if ((char == ' ') and not in_quotes): # Skip spaces but not if in_quotes
                skip = True
            if ((char == '\r') or (char == '\n')): # Skip CR and LF
                skip = True
            if (not skip):
                writer.write(char) # Write the character if not skipping
        writer.write('"') # Add " at the end of the file
