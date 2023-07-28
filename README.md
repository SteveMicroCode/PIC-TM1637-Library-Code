# PIC-TM1637-Library-Code
A library for TM1637 displays that should work with many 8 bit pics.

Developed using Microchip's XC8 compiler and tested on the PIC12F1840
and 18F6410.

I developed the PIC demo code for the TM1637 in my other repositories
into a generic TM1637 PIC library. The library functions are in file TM1637PIC.c
and there is also a .h file to include in the application code. 

See the libdemo application for how to use the library. Note the need to call the
initialisation function before attempting output using outputInteger(). Initialisation
allows configuration of both the port and pins for TM1637 connection to the PIC.

The library is intended to be resource sparing for use on 8 bit PICs. It does not
attempt floating point handling or offer full printf() style output capability.
My demo shows how it can be used to output floating point numbers. Basic rounding and
zero blanking is possible however.

Steve, July 2023
