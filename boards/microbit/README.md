Platform-Specific Instructions: BBC micro:bit
=============================================

The [BBC micro:bit](https://en.wikipedia.org/wiki/Microbit) is a platform
based around the nRF51822, an SoC with an ARM Cortex-M0 and a BLE
radio.

## Getting Started

First, follow the [Tock Getting Started guide](../../doc/Getting_Started.md)

### Programming the kernel

Once you have all software installed, you should run `make` in this
directory.

Support for flashing is currently rudimentary. You should [follow the
instructions to set up
pyOCD](https://docs.mbed.com/docs/mbed-os-handbook/en/5.3/debugging/debugging_microbit/)
and can then flash the generated elf file from gdb.

### Programming user-level applications

To flash a user application the same way, you have to first conert the
generated bin-file it into an intel hex file using e.g. srec_cat,
which is part of the srecord package:

cd apps/blink
make
srec_cat build/cortex-m0/cortex-m0.bin -binary -output -intel > blink.hex

Then you can load it into the proper location in the flash from gdb with

load apps/blink/blink.hex 0x20000