// 13/12/2022
// Last change: Kenny Yu
// Current function: displays received byte/s on serial monitor with UART from SPI on test microcontroller with test sensors

#include <msp430.h>
#include <inttypes.h>
#include "I2C.h"
#include "UART.h"
#include "SPI.h"
#include "timer.h"
#include "GNSS.h"

const uint8_t redLEDPin = BIT1;
const uint8_t greenLEDPin = BIT2;
const uint8_t blueLEDPin = BIT3;
const int HIGH = 1;
const int LOW = 0;

// output a port 2 pin as high or low
// input: defined pins, defined values (HIGH/LOW)
void digital_write(uint8_t pin, int value)
{
    if (value)
    {
        P2OUT |= pin; // output high
    }
    else
    {
        P2OUT &= ~pin; // output low
    }
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    uart_init();
    uart_GNSS_init();

    timerB0_init();
//    timerB1_init();

    P1DIR |= BIT0 | BIT1;

//    beginRecord();

//    rgbLED(1, 255, 0);


    // GNSS testing

//    GNSS_receive();

//    uart_send_byte('X');

    uint8_t testArray[] = "155.582,";
    float f;
    f = ASCII_to_float(testArray);
    uint8_t floatValsPtr[4];
    float_to_uint8_t(f, floatValsPtr);

//     print the float
    uart_send_bytes(floatValsPtr, 4);

//    uart_send_byte('X');


/*
     // SD card testing

     uint8_t R1, buf[512], token;
     uint16_t i;

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

     // fill buffer
     for (i = 0; i < 512; i++)
     {
     buf[i] = 0xAA;
     }

     //    // write a block to SD card to address 0x100 (256)
     //    SD_writeSingleBlock(0, buf, &token);

     // read a block from SD card
     R1 = SD_readSingleBlock(0x00000101, buf, &token);

     // print read SD block
     print_SDBlock(R1, buf, &token);

     // read a block from SD card
     R1 = SD_readSingleBlock(0x00000100, buf, &token);

     // print read SD block
     print_SDBlock(R1, buf, &token);
*/


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

    // keep MSP running
    while (1)
    {
    }

    return 0;
}
