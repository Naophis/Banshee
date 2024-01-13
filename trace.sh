#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x4200c354:0x3fcb8b90 0x42078fed:0x3fcb8bd0