// READING CODE ONLY, NOT FLIGHT-READY
// Requries 2433 Launchpad connected to the SD breakout at the required pins
// with SD card in raw data mode and data recording started

#include <msp430.h>
#include <inttypes.h>
#include "I2C.h"
#include "UART.h"
#include "SPI.h"
#include "timer.h"

// data variables
static uint8_t accelXYZ[6];
static float altitude_f;
static uint8_t altitude[4];
static uint8_t UTC[3];
static float GCS_f[2];
static uint8_t latitude[4], longitude[4];
static uint8_t dataArray[25];

// SD variables
static uint8_t R1, metaData[512], buf[512], token;
static uint32_t i;
static uint32_t blockAddress;
static uint8_t blockAddressArr[4];
const static uint8_t dataByteLengths[5] = { 6, 4, 3, 4, 4 }; // accel, alt, UTC, lat, long
const static uint8_t dataByteLocations[5] = { 0, 6, 10, 14, 18 };

// operation variables
static uint8_t currentStage; // stage currently in each time timer ISR is entered: 0 = flight ready, 1 = recording stage A, 2 = recording stage B, 3 = in flight stage, 4 = landing stage

// FOR TESTING ONLY //
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
// FOR TESTING ONLY //

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    P1DIR |= BIT0 | BIT1; // <- MOVE INSIDE APPROPRIATE FUNCTION

    // Initialising MSP430 pins and timers

    uart_init(); // Serial Monitor UART
    i2c_init(); // IMU I2C
    spi_init(); // SD Card SPI

    timerB0_init(); // 1Hz timer
    timerB1_init(); // RGB LED PWM timer

    // Initialise variables
    // set first stage as flight ready
    currentStage = 0;

    blockAddress = 1; // data begins in second SD block

    // fill SD arrays with 0
    for (i = 0; i < 512; i++)
    {
        metaData[i] = 0;
        buf[i] = 0;
    }

    // ----- READ SD ----- //
    // Initialise SD card
    if (!SD_init())
    {
        // error initialising SD card, set LED to red
        rgbLED(255, 0, 0);

        uart_send_bytes("SD Initialization Failure\r",
                        sizeof("SD Initialization Failure\r"));
    }
    else
    {
// DATA ANALYSIS MODE
        rgbLED(0, 255, 0);
        uart_send_bytes("SD Initialization Success\r",
                        sizeof("SD Initialization Success\r"));
#define TESTING // uncomment to read metadata and first data block
        // also uncomment in UART and SPI
#ifdef TESTING
        // read metadata block
        R1 = SD_readSingleBlock(0, buf, &token);
        // print metadata block
        print_SDBlock(R1, buf, &token);

        // read 1st data block
        R1 = SD_readSingleBlock(1, buf, &token);
        // print 1st data block
        print_SDBlock(R1, buf, &token);
#else
        // read metadata information
        // determine if data has been written
        while (1)
        {
            SD_readSingleBlock(0, buf, &token);
            if (!buf[0])
            {
                break;
            }
            // then determine number of blocks written to
            for (i = 0; i < 4; i++)
            {
                blockAddressArr[i] = buf[i + 1]; // block number is big endian
            }
            blockAddress = convert_uint8_array_to_uint32(blockAddressArr);

            // OPTIONAL TO-DO (requires converting uint32 to ASCII)
            // print block address

            uint8_t j;
            uint8_t ninetyNinesCount = 0;
            uint32_t blockCount = 0;
            while (blockCount < blockAddress)
            {
                // print out data by batch of ninety nine blocks (RealTerm 100 row limit)
                for (i = 1 + ninetyNinesCount * 99;
                        i <= 99 + ninetyNinesCount * 99; i++)
                {
                    SD_readSingleBlock(i, buf, &token);
                    // only print out block if less than last block address
                    if (blockCount < blockAddress)
                    {
                        blockCount++;
                        // print out data in current block, 21 bytes wide
                        for (j = 0; j < 21; j++)
                        {
                            uart_send_hex8(buf[j]);
                        }
                    }
                }
                // print Xs to delineate
                for (j = 0; j < 42; j++)
                {
                    uart_send_byte('X');
                }
                ninetyNinesCount++;
                __delay_cycles(30000000); // wait 100 seconds before sending out next batch
            }
            break;
        }
#endif
    }

    // ----- READ SD ----- //

//    begin1HzTimer();

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
