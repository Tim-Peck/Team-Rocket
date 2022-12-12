// 21/11/2022
// Last change: Kenny Yu
// Current function: displays received byte/s on serial monitor with UART from SPI on test microcontroller with test sensors

#include <msp430.h>
#include <inttypes.h>
#include "I2C.h"
#include "UART.h"
#include "SPI.h"

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    uint8_t receiveData[100];

    // set direction of pin 1.0 for led
    P1DIR |= BIT0;

    uart_init();
//    i2c_init();

    spi_init();

//    accelInit();
//    barInit();

//    getAccel(receiveData);
//    getBar(receiveData);

//    getBytes(0x28, 0x3C, receiveData, 2);

//    barInitSPI();
//    getBarSPI(receiveData);

    getBytesSPI(0xD0, receiveData, 1);
    uart_send_byte(receiveData[0]);

    __delay_cycles(1000000);
    getBytesSPI(0xD0, receiveData, 1);
    uart_send_byte(receiveData[0]);

    __delay_cycles(1000000);
    getBytesSPI(0xD0, receiveData, 1);
    uart_send_byte(receiveData[0]);

//    int i;
//    for (i = 0; i < 1; i++) {
//        uart_send_byte(receiveData[i]);
//    }

    // keep MSP running
    while (1)
    {
//        __delay_cycles(1000000);
//        getBytesSPI(0xD0, receiveData, 1);
//        uart_send_byte(receiveData[0]);
    }

    return 0;
}
