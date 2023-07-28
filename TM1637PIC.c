/*
 * PIC TM1637 Library functions
 * 
 * A basic TM1637 library that should work with most 8 bit PICS
 * See PIC12F1840 for demo use of this library with a 4 digit display
 * Has been tested on a PIC18F6410 additionally
 * Also tested with 210543 and 0..5 pattern 6 digit TM1637 displays
 * See demo for configuration details
 * 
 * Add the TM1637PIC.c and TM1637PIC.h files to the project and include the .h in the code
 * Ensure that tm1637initialise() is called before attempting output
 * Use tm1637output() to send uint32_t data to the display
 * 
 * The PIC12F1840/TM1637 display code was adapted by Steve Williams
 * for Microchip's MPLAB XC8 compiler. The TM1637 routines were originally
 * written by electro-dan for the BoostC compiler as part of project: 
 * https://github.com/electro-dan/PIC12F_TM1637_Thermometer. That project used
 * a PIC12F675 and I have ported the original code for the PIC12F1840 here. 
 * 
 * Demo code only with no warranty
 * 
 * Steve Williams, July 2023
 */


#include <xc.h>
#include "TM1637PIC.h"

// Bit time for serial comms set here, may be reduced from default 100us if display dio/clk lines have no capacitor:
#define BIT_TIME 5               // Bit time in us, note TM1637 max data rate of 500kHz assuming short signal path
#define _XTAL_FREQ 32000000      // Define clock frequency used by xc8 __delay(time) functions

// Define library scope constants:
const uint8_t tm1637ByteSetData = 0x40;        // 0x40 [01000000] = Indicate command to display data
const uint8_t tm1637ByteSetAddr = 0xC0;        // 0xC0 [11000000] = Start address write out all display bytes 
const uint8_t tm1637ByteSetOn = 0x88;          // 0x88 [10001000] = Display ON, plus brightness
const uint8_t tm1637ByteSetOff = 0x80;         // 0x80 [10000000] = Display OFF 
// Used to output the segment data for numbers 0..9 :
const uint8_t tm1637DisplayNumtoSeg[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};


// Define the library scope variables: 
// Library scope only hence display control variables and TM1637 tris/port bits must
// be set using functions such as initialise() from the application code

uint8_t *portTris;                   // Pointer to TRIS for the port used for TM1637
uint8_t *portLatch;                  // Pointer to latch for TM1637 port
uint8_t *portPins;                   // Pointer to port address for reads
uint8_t tm1637dioTrisBit = 0;        // This is the bit shift to set or clear data TRIS or PORT bit (set by initialise())
uint8_t tm1637clkTrisBit = 0;        // Bit shift to set or clear clk TRIS or PORT bit
uint8_t tm1637MaxDigits = 0;         // calling TM1637initialise() with parameters will reset these defaults 
uint8_t tm1637RightDigit = 0;        // Digits number 0 .. n from left
uint8_t tm1637Brightness = 0;        // Range 0 to 7
uint8_t tm1637Data[] = {1, 2, 3, 4, 5, 6}; // Digit numeric data to display,array elements are for digits 0..n
uint8_t decimalPointPos = 0;         // Flag for decimal point (digits counted from left),if > MaxDigits dp off
uint8_t ldgZeroBlanking = 0;         // If set true blanks leading zeros
uint8_t reorderDigits = 0;           // Flags requirement to output digit data non-sequentially
 
/*********************************************************************************************
  Function sets up display management variables and the port for TM1637 display
*********************************************************************************************/
void tm1637initialise(uint8_t *tris,uint8_t *portPins,uint8_t *portLatch, uint8_t dataBit, uint8_t clkBit, uint8_t displayType, uint8_t brightness)
{   // Update the global TM1637 variables with initialisation parameters:
    portLatch = portLatch;               // Pointer to latch for TM1637 port
    portPins = portPins;
    portTris = tris;
    switch (displayType)            // Set up for the TM1637 supported display type:
    { 
        case 1:                     // DISPLAY4DIG1TO4 = 1, Standard 4 digit, 0..3 from left                    
        {
            tm1637MaxDigits = 4;
            break;
        }
        case 6:                     // DISPLAY6DIG1TO6 = 6, 6 digit, digits 0..5 from left
        {
            tm1637MaxDigits = 6;
            break;
        }
        case 7:                     // DISPLAY6DIG321654 = 7, 6 digit, chinese made board with 210543 pattern
        {
            tm1637MaxDigits = 6;  
            reorderDigits = 1;      // Set up digit reordering for this type of display
            break;
        }
        
    }
    tm1637RightDigit = tm1637MaxDigits - 1;
    tm1637dioTrisBit = dataBit;
    tm1637clkTrisBit = clkBit;
    tm1637Brightness = brightness;       // Range 0 to 7  
    *portLatch &= ~(1<<dataBit);         // Clear the TM1637 port clk/dio bits before setting TRIS
    *portLatch &= ~(1<<clkBit);   
    *portTris |= (1<<dataBit);           // Set TRIS bits for the TM1637 dio and clk lines,
    *portTris |= (1<<clkBit);            //  lines go high as display has pullups
}


/*********************************************************************************************
 tm1637output()
 Outputs an integer on the display
*******************************************************************************************/
void tm1637output(uint32_t outputInteger,uint8_t decimalPos, uint8_t round, uint8_t ldgZeroBlank, uint8_t rightShift)
{
    ldgZeroBlanking = ldgZeroBlank;        // If set true blanks leading zeros
    decimalPointPos = decimalPos;          // Flag for decimal point (digits counted from left),if > MaxDigits dp off
    getDigits(outputInteger);
    if (round)
        roundDigits(round);
    if (rightShift)
        rightShiftDigits(rightShift);
    tm1637UpdateDisplay();
    return;
}

/*********************************************************************************************
 tm1637UpdateDisplay()
 Publish the tm1637Data array to the display
*******************************************************************************************/
void tm1637UpdateDisplay()
{   
    uint8_t tm1637Output[] = {0, 0, 0, 0, 0, 0};
    uint8_t tm1637DigitSegs = 0;
    uint8_t ctr;
    uint8_t stopBlanking = !ldgZeroBlanking;            // Allow blanking of leading zeros if flag set
            
    for (ctr = 0; ctr < tm1637MaxDigits; ctr ++)
    {
        tm1637DigitSegs = tm1637DisplayNumtoSeg[tm1637Data[ctr]];
        if (!stopBlanking && (tm1637Data[ctr]==0))  // Blank leading zeros if stop blanking flag not set
            {
               if (ctr < tm1637RightDigit)          // Never blank the rightmost digit
                  tm1637DigitSegs = 0;              // Segments set 0x00 gives blanked display numeral
            }
        else
        {
           stopBlanking = 1;                    // Stop blanking if have reached a non-zero digit
           if (ctr==decimalPointPos)            // Flag for presence of decimal point, digits 0..3
           {                                    // No dp display if decimalPointPos is set > Maxdigits
               tm1637DigitSegs |= 0b10000000;   // High bit of segment data is decimal point, set to display
           }
        }
        tm1637Output[ctr] = tm1637DigitSegs;    // Store all the digit output sequentially 0..n in buffer
    }
    
    // Write 0x40 [01000000] to indicate command to display data - [Write data to display register]:
    tm1637StartCondition();
    tm1637ByteWrite(tm1637ByteSetData);
    tm1637StopCondition();
    
    tm1637StartCondition();
    tm1637ByteWrite(tm1637ByteSetAddr);         // Specify the first digit display address 0xC0
    
    if(reorderDigits)
    {
       uint8_t tm1637OutputPattern[] = {2, 1, 0, 5, 4, 3};   // Specify the (bizarre) digit order of non-sequential 6 digit display
       for (ctr = 0; ctr < tm1637MaxDigits; ctr ++)
       {
           tm1637ByteWrite(tm1637Output[tm1637OutputPattern[ctr]]);    // Write out the segment data for out of sequence digits
       }
    }
    else
    {
        for (ctr = 0; ctr < tm1637MaxDigits; ctr ++)
        {
           tm1637ByteWrite(tm1637Output[ctr]);       // Write out the sequential segment data for each digit 0..n
        }
    } 
    
    tm1637StopCondition();
    tm1637StartCondition();                          // Write 0x80 [10001000] - Display ON, plus brightness
    tm1637ByteWrite((tm1637ByteSetOn + tm1637Brightness));
    tm1637StopCondition();
}


/*********************************************************************************************
 tm1637DisplayOn()
 Send display on command
*********************************************************************************************/
void tm1637DisplayOn(void)
{
    tm1637StartCondition();
    tm1637ByteWrite((tm1637ByteSetOn + tm1637Brightness));
    tm1637StopCondition();
}


/*********************************************************************************************
 tm1637DisplayOff()
 Send display off command
*********************************************************************************************/
void tm1637DisplayOff(void)
{
    tm1637StartCondition();
    tm1637ByteWrite(tm1637ByteSetOff);
    tm1637StopCondition();
}

/*********************************************************************************************
 tm1637StartCondition()
 Send the start condition
*********************************************************************************************/
void tm1637StartCondition(void) 
{
    *portTris &= ~(1<<tm1637dioTrisBit);   //Clear data tris bit
    *portLatch &= ~(1<<tm1637dioTrisBit);                     //Data output set low
    __delay_us(BIT_TIME);
}


/*********************************************************************************************
 tm1637StopCondition()
 Send the stop condition
*********************************************************************************************/
void tm1637StopCondition() 
{
    *portTris &= ~(1<<tm1637dioTrisBit);    // Clear data tris bit
    *portLatch &= ~(1<<tm1637dioTrisBit);    // Data low
    __delay_us(BIT_TIME);
    *portTris |= 1<<tm1637clkTrisBit;       // Set tris to release clk
    __delay_us(BIT_TIME);
    // Release data
    *portTris |= 1<<tm1637dioTrisBit;       // Set tris to release data
    __delay_us(BIT_TIME);
}


/*********************************************************************************************
 tm1637ByteWrite(char bWrite)
 Write one byte
*********************************************************************************************/
uint8_t tm1637ByteWrite(uint8_t bWrite) {
    for (uint8_t i = 0; i < 8; i++) {
        // Clock low
        *portTris &= ~(1<<tm1637clkTrisBit);       // Clear clk tris bit
        *portLatch &= ~(1<<tm1637clkTrisBit);       // Clock low;
        __delay_us(BIT_TIME);
        
        // Test bit of byte, data high or low:
        if ((bWrite & 0x01) > 0) 
          {
            *portTris |= 1<<tm1637dioTrisBit;       // Set data tris 
          } 
        else
          {
            *portTris &= ~(1<<tm1637dioTrisBit);    // Clear data tris bit
            *portLatch &= ~(1<<tm1637dioTrisBit);    // Data low;
          }
        __delay_us(BIT_TIME);

        // Shift bits to the left:
        bWrite = (bWrite >> 1);
        *portTris |= 1<<tm1637clkTrisBit;           // Set tris so clk goes high
        __delay_us(BIT_TIME);
    }

    // Wait for ack, send clock low:
    *portTris &= ~(1<<tm1637clkTrisBit);       // Clear clk tris bit
    *portLatch &= ~(1<<tm1637clkTrisBit);       // Clock low;
    *portTris |= 1<<tm1637dioTrisBit;          // Set data tris, makes input
    *portLatch &= ~(1<<tm1637dioTrisBit);       // Data low;
    __delay_us(BIT_TIME);
    
    *portTris |= 1<<tm1637clkTrisBit;          // Set tris so clk goes high
    __delay_us(BIT_TIME);
    uint8_t tm1637ack = *portPins & (1<<tm1637dioTrisBit);   // Read port data line, don't read latch
    if (!tm1637ack)
    {
        *portTris &= ~(1<<tm1637dioTrisBit);   // Clear data tris bit
        *portLatch &= ~(1<<tm1637dioTrisBit);   // Data low
    }
    __delay_us(BIT_TIME);
    *portTris &= ~(1<<tm1637clkTrisBit);       // Clear clk tris bit, set clock low
    *portLatch &= ~(1<<tm1637clkTrisBit);       // Clock low;
    __delay_us(BIT_TIME);

    return 1;
}


/*************************************************************************************************
 getDigits extracts decimal digit numbers from an integer for the display, note max displayed value is 
 9999 for 4 digit display, truncation of larger numbers. Larger displays: note maximum 65K as coded with 
 16 bit parameter - probable need to declare number as uint32_t if coding for a 6 digit display  
 ************************************************************************************************/

uint8_t getDigits(uint32_t number)
{
    int8_t ctr = (int8_t)tm1637RightDigit;            // Start processing for the rightmost displayed digit
    for (uint8_t ctr2 = 0; ctr2 < tm1637MaxDigits; ctr2++)
    {
        tm1637Data[ctr2]=0;      //Initialise the display data array with 0s
    }
    while(number > 0)            //Do if number greater than 0, ie. until all number's digits processed
    {
        if (ctr >= 0)
        {
           uint32_t modulus = number % 10;      // Split last digit from number
           tm1637Data[ctr] = (uint8_t)modulus;  // Update display character array
           number = number / 10;                // Divide number by 10
           ctr --;                              // Decrement digit counter to process number from right to left
        }
        else
        {
           number = 0;                          // Stop processing if have exceeded display's no of digits
        }
    }
    return 1;
}

/*************************************************************************************************
* roundDigits() zeros n digits starting right side of the decimal digit data array tm1637Data[],
* bankers rounding is applied. Processed digits become trailing zeros.
************************************************************************************************/
void roundDigits(uint8_t numberToRemove)
{
    int8_t startDigit = (int8_t)tm1637RightDigit;            // First digit to process is rightmost
    int8_t digit = 0;                                        // for loop 2 current digit counter
    for (uint8_t removeCount = 0; removeCount < numberToRemove; removeCount ++)  // One iteration for loop 1 per digit removal 
    {
        uint8_t carry = 0;                                   // Carry is set if need to add 1 to previous digit
        for (digit = startDigit; digit >= 0; digit -- )   // nested for loop 2, iterations = numberToRemove
        {                                                    // for loop 2 processes digits 0..n L->R
           if (digit == startDigit)
             {
                if (tm1637Data[digit]>5)                  // Round up if rightmost digit >5 by setting carry
                carry = 1;
                tm1637Data[digit] = 0;                    // Processed digit set to zero
             }
        
           if (digit < startDigit)
           {
              tm1637Data[digit] += carry;                 // Add any carry back from previous rounding
              if (tm1637Data[digit]>9)
                 {
                   tm1637Data[digit]=0;
                   carry = 1;                             // Set a carry back to previous digit
                 }
              else
                 {
                   carry = 0;
                 }
           }
        } // nested for loop 2, digit = ....
        startDigit --;     // If further iteration loop 1 should start processing from the previous digit
    } // for loop 1, removeCount = ....
}

/*************************************************************************************************
* Shifts displayed digits by n to right, discarding n digits, apply shift after rounding
************************************************************************************************/
void rightShiftDigits (uint8_t rightShift)
{
    if (rightShift)
        for (int8_t digit = (int8_t)tm1637RightDigit; digit >= 0; digit --)
        {
            int8_t shiftSource = digit-(int8_t)rightShift;
            if (shiftSource < 0 )                          // Don't try to shift from beyond left digit == tm1637Data[0]
                tm1637Data[digit] = 0;                     // Pad with zero for digits to left of shifted data
            else
                tm1637Data[digit] = tm1637Data[shiftSource];
        }
    return;
}