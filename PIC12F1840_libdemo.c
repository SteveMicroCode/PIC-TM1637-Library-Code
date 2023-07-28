// ---------------------------------------------------------------------
// The PIC12F1840/TM1637 display code was adapted by Steve Williams
// for Microchip's MPLAB XC8 compiler. The TM1637 routines were originally
// written by electro-dan for the BoostC compiler as part of project: 
// https://github.com/electro-dan/PIC12F_TM1637_Thermometer. That project used
// a PIC12F675 and I have ported the original code for the PIC12F1840 here. 
// 
// This code demonstrates use of my TM1637PIC.c library code
// The library code should work with many 8 bit pics, also tested with 18F6410
// Supports multiple display types, 4 and 6 digit, see the #defines to configure
// Add the TM1637PIC.c and TM1637PIC.h files to the project and include the .h in the code
// Ensure that tm1637initialise() is called before attempting output
// Use tm1637output() to send uint32_t data to the display
//
// No warranty is implied and the code is for test use at users own risk. 
// 
// Hardware configuration for the PIC 12F1840:
// RA0 = TM1637 CLK
// RA1 = n/c
// RA2 = LED via 560R resistor
// RA3 = INPUT ONLY n/c
// RA4 = TM1637 DIO
// RA5 = n/c
// -----------------------------------------------------------------------


#include <xc.h>
#include "TM1637PIC.h"

// PIC12F1840 Configuration Bit Settings:

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)
// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config DEBUG = OFF      // In-Circuit Debugger Mode (In-Circuit Debugger disabled, ICSPCLK and ICSPDAT are general purpose I/O pins)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// Single line config setup can be used: #pragma config FOSC=INTOSC,etc,....

#define _XTAL_FREQ 32000000      // Define clock frequency used by xc8 __delay(time) functions


//TM1637 definitions/variables, including port/pin setup:
#define DISPLAY4DIG1TO4 1                 // TM1637 display types, this is standard 4 digit, 0..3 from left
#define DISPLAY6DIG1TO6 6                 // 6 digit, digits 0..5 from left
#define DISPLAY6DIG321654 7               // 6 digit, Chinese made board with 2..0 5..3 pattern
const uint8_t trisTM1637 = 0b00010001;    // TM1637 pins are inputs, note TM1637 module pullups, LED ouput RA2 is enabled
uint8_t *portLatch = (uint8_t*)&LATA;     // Set up a pointer for writes to the TM1637 port latch, cast avoids LAT volatile warning
uint8_t *portPins = (uint8_t*)&PORTA;     // For port pin reads need to use the actual port not latch address
uint8_t *portTris = (uint8_t*)&TRISA;     // Also a pointer for TRIS address
uint8_t dioBit = 4;                       // This is the bit *SHIFT* (not mask) to set or clear data TRIS or PORT bits
uint8_t clkBit = 0;                       // Clock TRIS bit shift, not mask
const uint8_t dispType = DISPLAY4DIG1TO4;
uint8_t decimalPos = 99;            // Flag for decimal point (digits counted from left 0..n),if > MaxDigits dp off
uint8_t round = 0;                  // Number of digits to round, from right, 0 = no rounding, zeros rounded digits
uint8_t ldgZeroB = 0;               // If set true blanks leading zeros
uint8_t rightShift = 0;             // Right shifts displayed digits, discarding values on right, use after rounding
const uint8_t brightness = 2;             // Brightness, max 7, 0 is off


// MAX31865 definitions:
#define HIGH_THRESH_MSB 0x87                            // High fault threshold is 0x87B6, low fault 0x196C, configuration values are ADC RTD output in 16 bit format
#define HIGH_THRESH_LSB 0xB6                            // Fault will be flagged if RTD resistance > 212.05R, equivalent to approx 300C or if < 39.72R, -150C
#define LOW_THRESH_MSB  0x19
#define LOW_THRESH_LSB 0x6C                                
#define CONFIG_WRITE_ADDR 0x80                          // Note reg address has b7 set for writes
#define CONFIG_READ_ADDR 0x00    
#define RTDLSB 0x02                                     // RTD read data address LSB
#define RTDMSB 0x01                                     // RTD read data address MSB
#define FAULT_ADDR 0x07                                 // Fault register
#define RTDFAULTBIT 0x01                                // Fault bit is bit 0 of rtd LSB 
#define FAULTCLEAR 0b00000010                           // Bit D1 of config clears faults when set, autoclear
#define BIAS_ON 0b10000000                              // Mask to set D7 of config register, turns on RTD bias current     
#define CONVERSION_ON 0b00100000                        // Mask to set config D5, starts a one shot conversion
#define CONFIG3WIRE 0b00110001       // Config register for MAX31865 set up with bias current off(D7) initially, 3 wire(D4),1 shot conversion
#define CONFIG24WIRE 0b00100001      // Config setup for 2/4 wire sensors.Base configs do *not* clear faults, random initial data on read fault reg.
#define TEMPMULTIPLIER 100           // Temperature is returned as an integer, for n decimal places multiply float result by 10^n
#define MAXDRDY 0x08                 // MAX31865 DRDY o/p connected to RA3, active low

// Global variables:
const uint8_t portDefault = 0;       // PORT A will be set to outputs, all low

// Functions:
void initialise12F1840(void);


void main(void)
{                                                // Demo of TM1637 library code
  uint32_t outputInteger = 0;                    // Library code expects a 32 bit integer, max 4 digits here though
  __delay_ms(100);
  initialise12F1840();
  tm1637initialise(portTris,portPins,portLatch,dioBit,clkBit,dispType,brightness);
  outputInteger = 1234;                          // Output a 4 digit integer
  tm1637output(outputInteger, decimalPos , round, ldgZeroB, rightShift);
  __delay_ms(1000);
  outputInteger = 1;                             // Display 1 with leading zeros                      
  tm1637output(outputInteger, decimalPos , round, ldgZeroB, rightShift);
  __delay_ms(1000);
  outputInteger = 1;                             // Then without leading zeros
  ldgZeroB = 1;
  tm1637output(outputInteger, decimalPos , round, ldgZeroB, rightShift);
  __delay_ms(1000);
  float number = 99.99;                          // Next display a float with 2 decimal places
  number *= 100;
  outputInteger = (uint32_t)number;
  decimalPos = 1;                                // Decimal point is at digit 1, options 0..3
  tm1637output(outputInteger, decimalPos , round, ldgZeroB, rightShift);
  __delay_ms(1000);
  number = 10.46;                  // Next show rounding, round this to 10.5, note creates a trailing 0
  number *= 100;        
  outputInteger = (uint32_t)number;
  round = 1;                       // Can round > 1 digit, rounded digits all replaced by zeros
  tm1637output(outputInteger, decimalPos , round, ldgZeroB, rightShift);
  __delay_ms(2000);
  rightShift = 1;         // Shift right and get rid of one trailing zero, leading zeros also blanked
  decimalPos = 2;         // Position the dp and display 10.5 right justified
  tm1637output(outputInteger, decimalPos , round, ldgZeroB, rightShift);
  __delay_ms(2000);
  while(1);
}

void initialise12F1840()
{   
    OSCCON = 0b11110000;     // SPLLEN (b7) set = 4x PLL enable (nb config setting will override)
    PORTA = portDefault;     // OSCCON IRCF=1110 (b6..3), gives 8MHz clock, x4 with PLL = 32 MHz. SCS=00(1..0)
    TRISA = trisTM1637;      // TM1637 TRIS (default state): All pins set as digital outputs other than TM1637 clk/data
    CM1CON0 = 7;                   // Comparator off
    OPTION_REG = 0b10001000;       // Set bit 7, disable pullups, plus bit 3, prescaler not assigned Timer0
    ANSELA = 0;                    // Disable analogue inputs, all are digital
    ADCON0 = 0;                    // ADC disabled/unused
}