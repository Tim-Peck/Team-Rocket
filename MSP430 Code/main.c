// Note: Maximum amount of time available for writing data is 24 hours

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

void blip()
{
    // toggle on and off for a second
    P1OUT &= ~BIT0;
    int i;
    for (i = 0; i < 5; i++)
    {
        P1OUT ^= BIT0;
        __delay_cycles(100000);
        P1OUT ^= BIT0;
        __delay_cycles(100000);
    }
    __delay_cycles(150000);
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    P1DIR |= BIT0 | BIT1; // <- MOVE INSIDE APPROPRIATE FUNCTION


    // SET MODE HERE
    uint8_t mode = 0; // probably better way than elif statements


    // Initialising MSP430 pins and timers

//    uart_init(); // Serial Monitor UART // FOR FR2433 SERIAL UART HAS TO OCCUPY SAME PINS SO COMMENT THIS OUT WHEN IN FLIGHT MODE MODE
    uart_GNSS_init(); // GNSS module UART // FOR FR2433 GNSS UART HAS TO OCCUPY SAME PINS SO COMMENT THIS OUT WHEN IN DATA ANALYSIS MODE
    i2c_init(); // IMU I2C
    spi_init(); // SD Card SPI

    timerB0_init(); // 1Hz timer
    timerB1_init(); // RGB LED PWM timer

    // ----- FLIGHT LOGIC CODE ----- //

//    // Initialise SD card
//    if (!SD_init())
//    {
//        mode = 3;
//    }
//
//    // Mode list
//    // 0 - FLIGHT MODE (Flight logic - see flow chart)
//    // 1 - RESET MODE (Erase all content by setting all bytes to 0)
//    // 2 - DATA ANALYSIS MODE (View complete flight data)
//
//    if (mode == 0)
//    { // FLIGHT MODE
//
//        while (1)
//        {
//            // ------- INITIALISATION STAGE ------- //
//
//            // Check if flight/recording is complete
//            if (checkFinishStatus())
//            {
//                // set LED to green
//                rgbLED(0, 255, 0);
//                break; // exit
//            }
//
//            // Verify IMU connection
////            if (!checkIMUConnection()){
////                // set LED to pink
//////                rgbLED(255,0,255);
////                rgbLED(255,0,0); //temporary for testing
////                break; // exit
////            }
//
//            // set LED to orange
////            rgbLED(255,165,0);
//            rgbLED(255, 0, 0); //temporary for testing
//
//            // begin receiving GNSS signals
//            GNSS_receive();
//
//            // wait for fix acquired
//            while (!fixAcquired())
//                ;
//
//            // set LED to yellow
//            rgbLED(255, 255, 0);
//
//            // ------- FLIGHT READY STAGE ------- //
//
//            // begin 1Hz timer
//            begin1HzTimer();
//            break;
//            // check timer.c ISR for continued code
//        }
//
//    }
//    else if (mode == 1)
//    { // RESET MODE
//      // note: write speed limited by microcontroller and SD process
//      // at 1MHz, this is quite slow
//      // EVEN assuming writing a single block takes 512 cycles (more processes are involved)
//      // 1048576Hz / 512cycles per block = 2048 blocks per second
//      // 24 hours of data is required = 86400 blocks (at 1 block per second)
//      // Therefore, 86400/2048 = 42.1875 seconds BEST CASE to reset 24 hours worth of data
//
//      // MAX ADDRESS: 86399 (takes at least 90 minutes)
//        const uint32_t blocks = 1;
//
//        uint8_t R1, buf[512], token;
//        uint32_t i;
//
//        // set LED to off
//        rgbLED(0, 0, 0);
//
//        // fill buffer with 0x00
//        for (i = 0; i < 512; i++)
//        {
//            buf[i] = 0x00;
//        }
//
//        // reset first x blocks in SD card
//        // MAKE RESET MODE CHECK FOR FINAL ADDRESS WRITTEN TO SO DON'T HAVE TO RESET ALL 86400 BLOCKS
//        // actual time required for 1000 blocks is 1 minute and 2 seconds.
//        // time required for 5000 blocks is 5 minute and 13 seconds
//        // Therefore, about 1 minute and 2.75 seconds per 1000 blocks
//        for (i = 0; i < blocks; i++)
//        {
//            SD_writeSingleBlock(i, buf, &token);
//        }
//
//        // set LED to white
//        rgbLED(255, 255, 255);
//
//    }
//    else if (mode == 2)
//    { // DATA ANALYSIS MODE
//
//    }
//    else
//    { // error initialising SD card, set LED to red
//        rgbLED(255, 255, 0);
//    }

    // ----- FLIGHT LOGIC CODE ----- //

    begin1HzTimer();

//    rgbLED(1, 255, 0);

    // GNSS testing

//    GNSS_receive();

//    uart_send_byte('X');

    /*
     // SD card testing

     uint8_t R1, buf[512], token;
     uint16_t i;

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
