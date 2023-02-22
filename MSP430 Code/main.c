// Note: Maximum amount of time available for writing data is 24 hours

#include <msp430.h>
#include <inttypes.h>
#include "I2C.h"
#include "UART.h"
#include "SPI.h"
#include "timer.h"
#include "GNSS.h"

// -- USER ENTRY -- //
// set function mode
// mode 0 = flight mode, mode 1 = reset mode, mode 2 = data analysis mode
static uint8_t mode = 0;

// set time to begin recording in NZDT
static uint8_t recordHour = 19; // 0 to 23
static uint8_t recordMinute = 59 ; // 0 to 60
// -- USER ENTRY -- //

// data variables
static uint8_t accelXYZ[6];
static float altitude_f;
static uint8_t altitude[4];
static uint8_t UTC[3];
static float GCS_f[2];
static uint8_t latitude[4], longitude[4];

// SD variables
static uint8_t R1, metaData[512], buf[512], token;
static uint32_t i;
static uint32_t blockAddress;
static uint8_t blockAddressArr[4];

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
// FOR TESTING ONLY //

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    // Initialising MSP430 pins and timers

    uart_init(); // Serial Monitor UART
    uart_GNSS_init(); // GNSS module UART
    i2c_init(); // IMU I2C
    spi_init(); // SD Card SPI

    timerB0_init(); // 1Hz timer
    timerB3_init(); // RGB LED PWM timer

    // Initialise variables
    // set first stage as flight ready
    currentStage = 0;

    blockAddress = 1; // data begins in second SD block
    /*
    // fill SD arrays with 0
    for (i = 0; i < 512; i++)
    {
        metaData[i] = 0;
        buf[i] = 0;
    }

    // ----- FLIGHT LOGIC CODE ----- //

    // Initialise SD card
    if (!SD_init())
    {
        mode = 3;
        // error initialising SD card, set LED to red
        rgbLED(0, 0, 0);
    }

    // Mode list
    // 0 - FLIGHT MODE (Flight logic - see flow chart)
    // 1 - RESET MODE (Erase all content by setting all bytes to 0)
    // 2 - DATA ANALYSIS MODE (View complete flight data)

    if (mode == 0)
    { // FLIGHT MODE

        while (1)
        {
            // ------- INITIALISATION STAGE ------- //
            uint8_t byte = checkFinishStatus();

            // Check if recording has begun to not override existing data
            if (byte)
            {
                // set LED to green
                rgbLED(0, 255, 0);
                break; // exit
            }

            // Verify IMU connection // UNCOMMENT FOR IMU
            if (!checkIMUConnection()){
                // set LED to pink
//                rgbLED(255,0,255);
                rgbLED(255,0,0); //temporary for testing
                break; // exit
            }

            // initialise IMU // UNCOMMENT FOR IMU
            IMUInit();

            // set LED to orange
//            rgbLED(255,165,0);
            rgbLED(255, 0, 0); //temporary for testing

            // begin receiving GNSS signals
            GNSS_receive();

            // wait for fix acquired
            while (!fixAcquired())
                ;

            // set LED to yellow
            rgbLED(0, 0, 0);

            // ------- FLIGHT READY STAGE ------- //

            // begin 1Hz timer
            begin1HzTimer();

            while (1)
            {
                if (TB0CCTL0 & CCIFG)
                {
                    // ------- FLIGHT READY STAGE ------- //
                    // check if time is T=0 minute to launch time to begin recording
                    if (currentStage == 0)
                    {
                        // get current UTC time
                        parse_GGA_UTC(UTC);

                        // convert current UTC time to NZDT (NZDT is 13 hours ahead of UTC)
                        UTC[0] = (UTC[0] + 13) % 24;

                        // check if T=0 reached
                        if ((UTC[0] == recordHour) & (UTC[1] == recordMinute))
                        {
                            // set current stage to recording stage
                            currentStage = 1;
                        }
                    }

                    // ------- SET CONDITIONS ------- //
                    if (currentStage == 1)
                    {
                        // TO DO: set LED to rainbow
                        rgbLED(255, 255, 0);

                        // set SD recording status as recording
                        metaData[0] = 1;
                        SD_writeSingleBlock(0, metaData, &token);

                        // TO DO: set buzzer


                        currentStage = 2;
                    }

                    // ------- RECORD DATA ------- //
                    if (currentStage == 2)
                    {
                        // retrieve 2 accelerometer bytes in each direction
                        // raw register bytes - 2 bytes of acceleration in X, Y, Z each
                        getAccel(accelXYZ);

                        // retrieve MSL altitude time
                        // float: 4 raw bytes
                        altitude_f = parse_GGA_alt();
                        float_to_uint8(altitude_f, altitude);

                        // retrieve UTC
                        // fixed point: 3 bytes of uint8_t of hh,mm,ss
                        parse_GGA_UTC(UTC);

                        // retrieve latitude and longitude
                        // float: 4 raw bytes of latitude NORTH and longitude EAST each
                        parse_GGA_GCS(GCS_f);
                        float_to_uint8(GCS_f[0], latitude);
                        float_to_uint8(GCS_f[1], longitude);

                        // combine data together
                        for (i = 0; i < 6; i++)
                        {
                            buf[i] = accelXYZ[i];
                        }
                        for (i = 6; i < 10; i++)
                        {
                            buf[i] = altitude[i - 6];
                        }
                        for (i = 10; i < 13; i++)
                        {
                            buf[i] = UTC[i - 10];
                        }
                        for (i = 13; i < 17; i++)
                        {
                            buf[i] = latitude[i - 13];
                        }
                        for (i = 17; i < 21; i++)
                        {
                            buf[i] = longitude[i - 17];
                        }

                        // write data to block
                        SD_writeSingleBlock(blockAddress++, buf, &token);

                        // record last written block address in metadata block as big-endian
                        convert_uint32_to_uint8_array(blockAddress - 1,
                                                      blockAddressArr);
                        for (i = 0; i < 4; i++)
                        {
                            metaData[i + 1] = blockAddressArr[3-i];
                        }
                        SD_writeSingleBlock(0, metaData, &token);

                    }
                }
            }
        }

    }
    else if (mode == 1)
    { // RESET MODE

        // set LED to off
        rgbLED(0, 0, 0);

        // fill buffer with 0x00
        for (i = 0; i < 512; i++)
        {
            buf[i] = 0xAA;
        }

        // reset metadata block
        SD_writeSingleBlock(0, buf, &token);

        // set LED to white
        rgbLED(255, 255, 255);

    }
    else if (mode == 2)
    { // DATA ANALYSIS MODE

        // // print IMU acceleration
        // uint8_t rawAccels[6];
        // float linearAccels[3];
        // getAccel(rawAccels);
        // parseAccelBytes(rawAccels,linearAccels);
        // uint8_t accelx[4], accely[4], accelz[4];
        // float_to_uint8(linearAccels[0], accelx);
        // float_to_uint8(linearAccels[1], accely);
        // float_to_uint8(linearAccels[1], accelz);
        // uart_send_bytes(accelx, 4);
        // uart_send_bytes(accely, 4);
        // uart_send_bytes(accelz, 4);
    }
    else
    {
    }

    // ----- FLIGHT LOGIC CODE ----- //
*/

    // ---------- TESTING ---------- //
    // Timer testing // Launchpad VERIFIED

//    begin1HzTimer();

//    rgbLED(255, 255, 0);

    // Serial UART testing // Launchpad not verified fully working

//    // MCLK/SMCLK output
//    P3SEL0 |= BIT0 | BIT4;
//    P3DIR |= BIT0 | BIT4;

//    uart_send_bytes("hello world", 11); // could not verify fully working, only works on flash

    // GNSS testing // Launchpad VERIFIED

//    rgbLED(255, 255, 0);
//
//    GNSS_receive();
//
//    while (!fixAcquired())
//        ;
//
//// set LED to yellow
//    rgbLED(0, 0, 0);


     // SD card testing // Launchpad VERIFIED

     // initialize SD card
     if (SD_init())
     {
        uart_send_bytes("SD Initialization Success\r",
                        sizeof("SD Initialization Success\r"));
        uart_send_bytes("------------------\r", sizeof("------------------\r"));
        // fill buffer
        for (i = 0; i < 512; i++)
        {
            buf[i] = 0x00;
        }

        // write a block to SD card to address 0x100 (256)
        SD_writeSingleBlock(0, buf, &token);

        // read block 0 from SD card
        R1 = SD_readSingleBlock(0, buf, &token);

        // print read SD block
        print_SDBlock(R1, buf, &token);

        // check if contents are correct
        if (buf[0] == 0x00) {
            rgbLED(0, 255, 0); // received correct: temporary for testing
        } else {
            rgbLED(255, 0, 0);
        }
     }
     else
     {
        uart_send_bytes("SD Initialization Failure\r",
                        sizeof("SD Initialization Failure\r"));
     }

    // I2C testing // Launchpad VERIFIED
//     if (checkIMUConnection()){
//         rgbLED(0,255,0); // received correct: temporary for testing
//     } else {
//         rgbLED(255,0,0);
//     }


    // ---------- TESTING ---------- //

    // keep MSP running
    while (1)
    {
    }

    return 0;
}
