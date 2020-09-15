with open("ARTIC.flash", "r") as reader:
    line = reader.readline()
    print (line)
    if (line[0:2] != 'D1'):
        raise ValueError("First byte was not 0xD1!")
    line = reader.readline()
    print (line)
    if (line[0:2] != '20'):
        raise ValueError("Second byte was not 0x20!")
    with open("Firmware_ARTIC006_flash_image__PMEM.h", "w") as PMEM:
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
    with open("Firmware_ARTIC006_flash_image__XMEM.h", "w") as XMEM:
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
            line = reader.readline()
            XMEM.write(line[0:2])
            if (x < 21844):
                XMEM.write(",")
            XMEM.write("\n")
        XMEM.write("};\n")
    with open("Firmware_ARTIC006_flash_image__YMEM.h", "w") as YMEM:
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
            line = reader.readline()
            YMEM.write(line[0:2])
            if (x < 6825):
                YMEM.write(",")
            YMEM.write("\n")
        YMEM.write("};\n")
