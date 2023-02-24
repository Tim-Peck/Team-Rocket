
#include <msp430.h>
#include <inttypes.h>
#include "ADC.h"

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    initADC();

    double batVal = getBatVoltage();
    
    // keep MSP running
    while (1)
    {
    }

    return 0;
}
