#ifndef TIMER_H
#define TIMER_H

#include <inttypes.h>

// initialise timerA0 to have period of 1 second using TAIFG
void timerB0_init();

// begin timerB0 and functions in TBIFG
// this will execute the main functions of reading the sensors and writing to the SD card every second
void begin1HzTimer();

// initialise timerB0 for RGB LED
void timerB1_init();

// set RGB LED colour
// input: 8 bit (0-255) RGB values for each colour
void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal);

// combine 4 individual bytes to one uint32
// big-endian
uint32_t convert_uint8_array_to_uint32(uint8_t *arr);

// split uint32 to 4 individual bytes
// big-endian
void convert_uint32_to_uint8_array(uint32_t num, uint8_t* arr);

#endif
