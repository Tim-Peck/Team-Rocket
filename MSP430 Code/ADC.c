#include <msp430.h>

#include "ADC.h"

void adc_init() {
  // Key settings
  // Set the sample time in 4x + 4 ADC clock pulses
  // ADCCTL0 register, ADCSHTx 4 bits, default = 1 (8 ADCCK)
  // Should be good

  // Set the ADC trigger to the ADCSC register
  // ADCCTL1 register, ADCSHSx 2 bits, default = 0 (ADCSC)
  // Should be good

  // Set to pulse sample mode so the sample time is defined by the timer
  // ADCCTL1 register, ADCSHP bit, default = 0 (sample-input mode)
  ADCCTL1 |= ADCSHP; // (sampling timer mode)

  // Set to single-channel single-conversion mode
  // ADCCTL1 register, ADCCONSEQx 2 bits, default = 0 (single-channel single-conversion mode)
  // should be good

  // Set the ADC resolution (8, 10 or 12 bits)
  // ADCCTL2 register, ADCRES 2 bits, default = 1 (10 bit)
  ADCCTL2 |= ADCRES1; // (12 bit)

  // Reduce the ADC current consumption and max read rate
  // ADCCTL2 register, ADCSR bit, default = 0 (fast mode)
  ADCCTL2 |= ADCSR; // (slow mode)

  // Configure the pins
  P5SEL0 |= BIT5;
  P5SEL1 |= BIT5;

  // Turn the ADC on
  ADCCTL0 |= ADCON;

}

uint8_t getRawRef() {
  // Disable conversion to change source
  ADCCTL0 &= ~ADCON;
  ADCCTL0 &= ~ADCENC;
  // Select the read analogue read channel
  // ADCMCTL0 register, ADCINCHx 0-3 bits
  ADCMCTL0 |= ADCINCH0 | ADCINCH2 | ADCINCH3; // (A13)
  ADCCTL0 |= ADCON;
  return getADCRawVal();
}

uint8_t getRawBat() {
  // Disable conversion to change source
  ADCCTL0 &= ~ADCON;
  ADCCTL0 &= ~ADCENC;
  // Select the read analogue read channel
  // ADCMCTL0 register, ADCINCHx 0-3 bits
  ADCMCTL0 |= ADCINCH0 | ADCINCH3; // (A9)
  ADCCTL0 |= ADCON;
  return getADCRawVal();
}

double getBatVoltage() {
  return convertADCToVoltage(getRawRef(),getRawBat());
}

double convertADCToVoltage(uint16_t refVal,uint16_t batVal) {
  // 1024 for 10 bit
  return (1.5/(refVal/1024.0)*(batVal/1024.0)) * 2;
}

uint16_t getADCRawVal() {

// Enable Conversion
  ADCCTL0 |= ADCENC;

  // start taking sample
  ADCCTL0 |= ADCSC;

  // Poll until sample and conversion is complete
  while (!(ADCIFG & ADCIFG0)) {}

  // Reset the ADCSC value if not already low
//  ADCCTL0 &= ~ADCSC;

  // Read and return the conversion register
  uint16_t rawVal = ADCMEM0;

  return rawVal;
}
