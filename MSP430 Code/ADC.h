#ifndef ADC_H
#define ADC_H

#include <inttypes.h>

const double ADC_VRMAX 3.3;
const double ADC_VRMIN 0.0;

// User functions

// Initalise the ADC registers and the battery voltage pin.
void initADC();

// Get the battery voltage as a double
double getBatVoltage();


// Helper Functions

// Covert the uint16_t value from the ADC into a ratio of the reference voltages
double convertADCToVoltage(uint16_t adcVal);

// Read and return the currently set ADC pin
uint16_t getADCRawVal();



#endif
