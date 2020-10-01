# This code converts the ARTIC R2 ARTIC006 firmware from "flash" format
# into header files containing arrays of const uint32_t.
# The arrays can then be downloaded to the ARTIC R2.
# The array LENgths come from the ARTIC R2 datasheet.
# The array CHECKSUMs come from the ARTIC006 Release_README.txt.

# Written by Paul Clark, October 1st 2020

# License: please see the license file at:
# https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md


with open("ARTIC.flash", "r") as reader:
    line = reader.readline()
    #print (line)
    if (line[0:2] != 'D1'):
        raise ValueError("First byte was not 0xD1!")
    line = reader.readline()
    #print (line)
    if (line[0:2] != '20'):
        raise ValueError("Second byte was not 0x20!")
    with open("ARTIC_R2_Firmware_PMEM.h", "w") as PMEM:
        PMEM.write("// ARTIC R2 ARTIC006 Firmware PMEM\n")
        PMEM.write("// Converted from ARTIC.flash\n\n")
        PMEM.write("#ifndef ARTIC_R2_PMEM_H\n")
        PMEM.write("#define ARTIC_R2_PMEM_H\n\n")
        PMEM.write("const uint32_t ARTIC_R2_PMEM_LEN = 10240;\n")
        PMEM.write("const uint32_t ARTIC_R2_PMEM_CHECKSUM = 0x4DC69E;\n")
        PMEM.write("const uint32_t ARTIC_R2_PMEM[10240] = {\n")
        for x in range(10240):
            PMEM.write("\t0x")
            line = reader.readline()
            PMEM.write(line[0:2])
            line = reader.readline()
            PMEM.write(line[0:2])
            line = reader.readline()
            PMEM.write(line[0:2])
            line = reader.readline()
            PMEM.write(line[0:2])
            if (x < 10239):
                PMEM.write(",")
            PMEM.write("\n")
        PMEM.write("};\n")
        PMEM.write("\n#endif\n")
    with open("ARTIC_R2_Firmware_XMEM.h", "w") as XMEM:
        XMEM.write("// ARTIC R2 ARTIC006 Firmware XMEM\n")
        XMEM.write("// Converted from ARTIC.flash\n\n")
        XMEM.write("#ifndef ARTIC_R2_XMEM_H\n")
        XMEM.write("#define ARTIC_R2_XMEM_H\n\n")
        XMEM.write("const uint32_t ARTIC_R2_XMEM_LEN = 21845;\n")
        XMEM.write("const uint32_t ARTIC_R2_XMEM_CHECKSUM = 0x4A51F6;\n")
        XMEM.write("const uint32_t ARTIC_R2_XMEM[21845] = {\n")
        for x in range(21845):
            XMEM.write("\t0x")
            line = reader.readline()
            XMEM.write(line[0:2])
            line = reader.readline()
            XMEM.write(line[0:2])
            line = reader.readline()
            XMEM.write(line[0:2])
            if (x < 21844):
                XMEM.write(",")
            XMEM.write("\n")
        XMEM.write("};\n")
        XMEM.write("\n#endif\n")
    with open("ARTIC_R2_Firmware_YMEM.h", "w") as YMEM:
        YMEM.write("// ARTIC R2 ARTIC006 Firmware YMEM\n")
        YMEM.write("// Converted from ARTIC.flash\n\n")
        YMEM.write("#ifndef ARTIC_R2_YMEM_H\n")
        YMEM.write("#define ARTIC_R2_YMEM_H\n\n")
        YMEM.write("const uint32_t ARTIC_R2_YMEM_LEN = 6826;\n")
        YMEM.write("const uint32_t ARTIC_R2_YMEM_CHECKSUM = 0x65CC81;\n")
        YMEM.write("const uint32_t ARTIC_R2_YMEM[6826] = {\n")
        for x in range(6826):
            YMEM.write("\t0x")
            line = reader.readline()
            YMEM.write(line[0:2])
            line = reader.readline()
            YMEM.write(line[0:2])
            line = reader.readline()
            YMEM.write(line[0:2])
            if (x < 6825):
                YMEM.write(",")
            YMEM.write("\n")
        YMEM.write("};\n")
        YMEM.write("\n#endif\n")
