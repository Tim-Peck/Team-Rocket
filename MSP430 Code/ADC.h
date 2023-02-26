#ifndef ADC_H
#define ADC_H

#include <inttypes.h>

// User functions

// Initalise the ADC registers and the battery voltage pin.
void adc_init();

// Get the battery voltage as a double
double getBatVoltage();


// Helper Functions

// Covert the uint16_t value from the ADC into a ratio of the reference voltages
double convertADCToVoltage(uint16_t adcVal);

// Read and return the currently set ADC pin
uint16_t getADCRawVal();



#endif
