#ifndef TIMER_H
#define TIMER_H

#include <inttypes.h>

static volatile uint8_t rainbowOn, rainbowPhase;
static volatile uint8_t RGB[3];

// initialise timerB0 to have period of 1 second using CCR0IFG
void timerB0_init();

// begin timerB0 and functions in TBIFG
// this will execute the main functions of reading the sensors and writing to the SD card every second
void begin1HzTimer();

// initialise timerB2 for buzzer
void timerB2_init();

// turn on buzzer at desired frequency
// frequency range from AROUND 16.088Hz to 527165Hz. Note at high frequencies, resolution is very low
// input: desired frequency within range
void buzzerOn(int frequency);

// initialise timerB0 for RGB LED
void timerB3_init();

// set RGB LED colour
// input: 8 bit (0-255) RGB values for each colour
void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal);

// set a rainbow effect for the RGB LED
// starts from red to yellow, green, cyan, blue, magenta and then back to red
void rainbowEffect();

uint32_t convert_uint8_array_to_uint32(uint8_t *arr);

void convert_uint32_to_uint8_array(uint32_t num, uint8_t* arr);

#endif
