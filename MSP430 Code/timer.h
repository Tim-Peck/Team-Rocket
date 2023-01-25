#ifndef TIMER_H
#define TIMER_H

#include <inttypes.h>

// initialise timerA0 to have period of 1 second using TAIFG
void timerB0_init();

// begin timerB0 and functions in TBIFG
void beginRecord();

// initialise timerB0 for RGB LED
void timerB1_init();

// set RGB LED colour
// input: 8 bit (0-255) RGB values for each colour
void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal);

#endif
