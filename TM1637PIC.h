/* 
 * File:   TM1637PIC.h
 * Author: Steve Williams
 *
 * Created on 21 June 2023, 10:21
 */

#ifndef TM1637PIC_H
#define	TM1637PIC_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
    
//Function prototypes:
void tm1637initialise(uint8_t *tris,uint8_t *portPins,uint8_t *portLatch, uint8_t dataBit, uint8_t clkBit,uint8_t displayType,uint8_t brightness);
void tm1637StartCondition(void);
void tm1637StopCondition(void);
uint8_t tm1637ByteWrite(uint8_t bWrite);
void tm1637UpdateDisplay(void);
void tm1637output(uint32_t outputInteger, uint8_t decimalPos, uint8_t round, uint8_t ldgZeroBlank, uint8_t rightShift);
void tm1637DisplayOn(void);
void tm1637DisplayOff(void);
uint8_t getDigits(uint32_t number);   // Extracts decimal digits from integer, populates tm1637Data array
void roundDigits(uint8_t numberToRemove);     // Removes n digits from rightmost with bankers rounding carried back
void rightShiftDigits (uint8_t rightShift);   // Shifts to right by n digits, padding on left with zeros

#ifdef	__cplusplus
}
#endif

#endif	/* TM1637PIC_H */

