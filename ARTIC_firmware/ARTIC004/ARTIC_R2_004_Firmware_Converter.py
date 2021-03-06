# This code converts the ARTIC R2 ARTIC004 firmware from "flash_image.bin" format
# into header files containing arrays of const uint32_t.
# The arrays can then be downloaded to the ARTIC R2.
# The array LENgths come from the ARTIC R2 datasheet.
# The array CHECKSUMs come from the ARTIC itself as I don't have the Release_README.txt for ARTIC004.

# Written by Paul Clark, October 1st 2020

# License: please see the license file at:
# https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md


with open("Firmware_ARTIC004_flash_image.bin", "r") as reader:
    line = reader.readline()
    #print (line)
    if (line[0:4] != '0xd1'):
        raise ValueError("First byte was not 0xd1!")
    line = reader.readline()
    line = reader.readline()
    #print (line)
    if (line[0:4] != '0x20'):
        raise ValueError("Second byte was not 0x20!")
    with open("ARTIC_R2_Firmware_PMEM.h", "w") as PMEM:
        PMEM.write("// ARTIC R2 ARTIC004 Firmware PMEM\n")
        PMEM.write("// Converted from Firmware_ARTIC004_flash_image.bin\n")
        for x in range(10240):
            PMEM.write("\t0x")
            line = reader.readline()
            line = reader.readline()
            PMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            PMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            PMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            PMEM.write(line[2:4])
            if (x < 10239):
                PMEM.write(",")
            PMEM.write("\n")
    with open("ARTIC_R2_Firmware_XMEM.h", "w") as XMEM:
        XMEM.write("// ARTIC R2 ARTIC004 Firmware XMEM\n")
        XMEM.write("// Converted from Firmware_ARTIC004_flash_image.bin\n")
        for x in range(21845):
            XMEM.write("\t0x")
            line = reader.readline()
            line = reader.readline()
            XMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            XMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            XMEM.write(line[2:4])
            if (x < 21844):
                XMEM.write(",")
            XMEM.write("\n")
    with open("ARTIC_R2_Firmware_YMEM.h", "w") as YMEM:
        YMEM.write("// ARTIC R2 ARTIC004 Firmware YMEM\n")
        YMEM.write("// Converted from Firmware_ARTIC004_flash_image.bin\n")
        for x in range(6826):
            YMEM.write("\t0x")
            line = reader.readline()
            line = reader.readline()
            YMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            YMEM.write(line[2:4])
            line = reader.readline()
            line = reader.readline()
            YMEM.write(line[2:4])
            if (x < 6825):
                YMEM.write(",")
            YMEM.write("\n")
