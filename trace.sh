#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x4200f213:0x3fcab330 0x42072369:0x3fcab3a0 0x4037e895:0x3fcab3c0