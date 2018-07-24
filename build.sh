#!/bin/bash
set -e

cargo build --release

arm-none-eabi-objcopy -O binary target/thumbv6m-none-eabi/release/coffee-compass target/thumbv6m-none-eabi/release/coffee-compass.bin
/mnt/c/Users/David/Documents/ArduinoData/packages/arduino/tools/bossac/1.7.0/bossac.exe -i -d -U true -i -e -w -v target/thumbv6m-none-eabi/release/coffee-compass.bin -R
