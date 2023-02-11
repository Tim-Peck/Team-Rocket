// 13/12/2022
// Last change: Kenny Yu
// Current function: displays received byte/s on serial monitor with UART from SPI on test microcontroller with test sensors

#include <msp430.h>
#include <inttypes.h>

#include "I2C.h"
#include "UART.h"
#include "SPI.h"
#include "Camera.h"

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    // SD card testing

    uint8_t R1, buf[512], token;

    uart_init();
    //    i2c_init();
    spi_init(); // initialise microcontroller SPI for SD card

    // initialize SD card
    if (SD_init())
    {
        uart_send_bytes("SD Initialization Success\r",
                        sizeof("SD Initialization Success\r"));
    }
    else
    {
        uart_send_bytes("SD Initialization Failure\r",
                        sizeof("SD Initialization Failure\r"));
    }

    uart_send_bytes("------------------\r", sizeof("------------------\r"));

    // read a block from SD card
    R1 = SD_readSingleBlock(0, buf, &token);

    // print read SD block
    print_SDBlock(R1, buf, &token);

    // read a block from SD card
    SD_readSingleBlock(0xFFFFFFFF, buf, &token);

    // keep MSP running
    while (1)
    {
//        __delay_cycles(100000);

    }

    /*
     // ------------------------------ //
     // ACTUAL MICROCONTROLLER TESTING //

     // set direction of pins for led and buzzer
     P2DIR |= (redLEDPin | greenLEDPin | blueLEDPin);
     P5DIR |= BIT0;

     // set colour
     digital_write(redLEDPin, HIGH);
     digital_write(greenLEDPin, HIGH);
     digital_write(blueLEDPin, HIGH);

     */

    return 0;
}
