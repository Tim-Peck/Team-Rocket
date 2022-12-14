// 13/11/2022
// Last change: Kenny Yu
// Current function: displays received byte/s on serial monitor with UART from I2C on test microcontroller with test sensors

#include <msp430.h>
#include <inttypes.h>
#include "I2C.h"
#include "UART.h"

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    uint8_t receiveData[10];

    // set direction of pin 1.0 for led
    P1DIR |= BIT0;

    uart_init();
    i2c_init();

    accelInit();
    barInit();

//    getAccel(receiveDataData);
//    getBar(receiveData);
//    getBytes(0x28, 0x3C, receiveData, 2);
////
//    uart_send_byte(receiveData[0]);
//    uart_send_byte(receiveData[1]);

    // keep MSP running
    while (1)
    {
        __delay_cycles(100000);
        getAccel(receiveData);
        uart_send_byte(receiveData[0]);
        uart_send_byte(receiveData[1]);
    }

    return 0;
}
