// Unit tests
//#define TEST_LED
//#define TEST_TIMER // Must uncomment LED testing in TIMER B3 ISR
//#define TEST_UART // Serial Comm has not been tested to work on PCB or 2355 Launchpad
//#define TEST_GNSS // Function not made to test GNSS UART communication only
//#define TEST_GNSS_FIX
//#define TEST_SD
//#define TEST_IMU // DOES NOT WORK WITH CURRENT I2C CODE
//#define TEST_ADC
//#define TEST_BUZZER
//#define TEST_RAINBOW

// Operation type list
// FLIGHT MODE (Flight logic - see flow chart)
#define FLIGHT_MODE

// RESET MODE (Erase all content by setting all bytes to 0)
// #define RESET_MODE

// DATA ANALYSIS MODE (View complete flight data)
// #define DATA_ANALYSIS_MODE

// USER ENTRY //
// SET START TIME IN NZDT (CANNOT SET TIME IN THE NEXT DAY)
#define FLIGHT_START_HOUR 16
#define FLIGHT_START_MIN 26
// USER ENTRY //

#include <msp430.h>
#include <inttypes.h>

#include "Defines.h"
#include "I2C.h"
#include "UART.h"
#include "SPI.h"
#include "timer.h"
#include "GNSS.h"
// #include "ADC.h"
// #include "FS.c"

const uint8_t blueLEDPin = BIT1;
const uint8_t greenLEDPin = BIT2;
const uint8_t redLEDPin = BIT3;
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

void test_start_sequence()
{
    rgbLED(0, 0, 255);
    __delay_cycles(1000000);
    rgbLED(0, 0, 0);
    __delay_cycles(1000000);
    rgbLED(0, 0, 255);
    __delay_cycles(1000000);
    rgbLED(0, 0, 0);
    __delay_cycles(1000000);
}

#ifdef FLIGHT_MODE
#define FLIGHT_LOGIC
#endif
#ifdef RESET_MODE
#define FLIGHT_LOGIC
#endif
#ifdef DATA_ANALYSIS_MODE
#define FLIGHT_LOGIC
#endif

#ifdef FLIGHT_MODE
// data variables
static uint8_t accelXYZ[6];
static float altitude_f;
static uint8_t altitude[4];
static uint8_t UTC[3];
static float GCS_f[2];
static uint8_t latitude[4], longitude[4];
#endif

#ifdef FLIGHT_LOGIC
// SD variables
static uint8_t R1, metaData[512], buf[512], token;
static uint32_t i;
static uint32_t blockAddress;
static uint8_t blockAddressArr[4];

// operation variables
static uint8_t currentStage; // stage currently in each time timer ISR is entered: 0 = flight ready, 1 = recording stage A, 2 = recording stage B, 3 = in flight stage, 4 = landing stage
#endif

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

// Initialising MSP430 pins and timers

#ifdef TEST_LED
    timerB3_init(); // RGB LED PWM timer
#endif

#ifdef TEST_TIMER
  timerB0_init(); // 1Hz timer
#endif

#ifdef TEST_UART
  uart_init(); // Serial Monitor UART
#endif

#ifdef TEST_GNSS
  #ifndef TEST_UART
  uart_init();
  #endif
  uart_GNSS_init(); // GNSS module UART
#endif

#ifdef TEST_GNSS_FIX
  #ifndef TEST_GNSS
    #ifndef TEST_UART
    uart_init();
    #endif
    uart_GNSS_init(); // GNSS module UART
  #endif
#endif

#ifdef TEST_SD
  #ifndef TEST_UART
  uart_init();
  #endif
  spi_init(); // SD Card SPI
#endif

#ifdef TEST_IMU
    i2c_init(); // IMU I2C
    timerB0_init();
    IMUInit();
#endif

#ifdef TEST_ADC
  adc_init();
#endif

#ifdef TEST_BUZZER
  timerB2_init();
#endif

#ifdef TEST_RAINBOW
    timerB3_init();
#endif

// ---------- TESTING ---------- //

#ifdef TEST_LED
    // LED testing
    digital_write(redLEDPin, HIGH);
    digital_write(greenLEDPin, HIGH);
#ifndef USE_DEV_BOARD
    digital_write(blueLEDPin, HIGH);
#endif
    rgbLED(255, 0, 0);

    __delay_cycles(1000000);

    rgbLED(0, 255, 0);

    __delay_cycles(1000000);

    rgbLED(255, 0, 0);

    __delay_cycles(1000000);

    rgbLED(0, 0, 0);

#endif

// Test 1Hz Timer
#ifdef TEST_TIMER
  test_start_sequence();

  //Main 1Hz timer
  begin1HzTimer();
#endif

// Test serial comm UART
#ifdef TEST_UART
  test_start_sequence();

  uart_send_byte('X');
  uart_send_bytes("hello world", 11); // could not verify fully working, only works on flash

  __delay_cycles(1000000);
#endif

// #ifdef TEST_GNSS
//
//   test_start_sequence();
//   // GNSS testing // Launchpad VERIFIED, PCB VERIFIED
//
//   GNSS_receive();
//
//   if (GNSSIsResponding()) {
//     rgbLED(0, 255, 0);
//   } else {
//     rgbLED(255, 0, 0);
//   }
//
//   __delay_cycles(1000000);
//
// #endif

// Test if communication is good and fix received from GNSS on UART line
#ifdef TEST_GNSS_FIX
  test_start_sequence();

  rgbLED(255, 0, 0);

  GNSS_receive();

  while (!fixAcquired());

  rgbLED(0, 255, 0);

  __delay_cycles(1000000);

#endif

// Test if SD communication, writing and reading is good is SPI line
#ifdef TEST_SD
  test_start_sequence();

  int i;
  uint8_t buf[512];
  uint8_t R1;
  uint8_t token;


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
    buf[4] = 0x02;

    // write a block to SD card to address 0
    SD_writeSingleBlock(0, buf, &token);

    // read block 0 from SD card
    R1 = SD_readSingleBlock(0, buf, &token);

    // print read SD block
    print_SDBlock(R1, buf, &token);

    // check if contents are correct
    if (buf[4] == 0x02) {
    rgbLED(0, 255, 0); // received correct: temporary for testing
    } else {
    rgbLED(255, 255, 0);
    }
  }
  else
  {
    uart_send_bytes("SD Initialization Failure\r",
    sizeof("SD Initialization Failure\r"));
    rgbLED(255, 0, 0);
  }

  __delay_cycles(1000000);
#endif

// Test if IMU communication is good on I2C line
#ifdef TEST_IMU
    test_start_sequence();

    if (checkIMUConnection())
    {
        uint8_t accelCalibLevel;
        // wait for IMU to be calibrated
        // begin 1Hz timer
        begin1HzTimer();
        while (1)
        {
            if (TB0CCTL0 & CCIFG)
            {
                accelCalibLevel = readAccelCalib();

                // display different colours for each stage of calibration
                if (accelCalibLevel == 0)
                {
                    rgbLED(255, 0, 255); // calibration 0: pink
                }
                else if (accelCalibLevel == 1)
                {
                    rgbLED(127, 0, 255); // calibration 1: purple
                }
                else if (accelCalibLevel == 2)
                {
                    rgbLED(0, 0, 255); // calibration 2: blue
                }
                else if (accelCalibLevel == 3)
                {
                    rgbLED(0, 255, 255); // error: cyan
                }
                else
                {
                    rgbLED(0, 255, 0);
                }
            }
        }
    }
    else
    {
        rgbLED(255, 0, 0);
    }

    __delay_cycles(1000000);
#endif

// Test if ADC working
#ifdef TEST_ADC
  test_start_sequence();

  double batVal = getBatVoltage();
  if (batVal < 3) {
    rgbLED(255, 255, 255);
  } else if (batVal < 3.5) {
    rgbLED(255, 0, 0);
  } else if (batVal < 3.7) {
    rgbLED(255, 255, 0);
  } else if (batVal < 3.8){
    rgbLED(0, 255, 0);
  } else if (batVal < 3.9){
    rgbLED(0, 255, 255);
  } else {
    rgbLED(0, 0, 255);
  }


#endif

// Test if buzzer working
#ifdef TEST_BUZZER
  test_start_sequence();

  rgbLED(0, 255, 0);

  buzzerOn(262);
  __delay_cycles(1000000);

  buzzerOn(330);
  __delay_cycles(1000000);

  buzzerOn(392);
  __delay_cycles(1000000);

  buzzerOn(523);
  __delay_cycles(1000000);

  buzzerOn(392);
  __delay_cycles(1000000);

  buzzerOn(330);
  __delay_cycles(1000000);

  buzzerOn(262);
  __delay_cycles(1000000);

#endif

// Test if rainbow effect for LED working
#ifdef TEST_RAINBOW
    test_start_sequence();

    rainbowEffect();
#endif

#ifdef FLIGHT_LOGIC

// ----- FLIGHT LOGIC CODE ----- //

    uart_init(); // Serial Monitor UART
    uart_GNSS_init(); // GNSS module UART
    i2c_init(); // IMU I2C
    spi_init(); // SD Card SPI
// adc_init();

    timerB0_init(); // 1Hz timer
    timerB3_init(); // RGB LED PWM timer

    rgbLED(255, 255, 255); // set a first light to be white

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

// Loop for initialising SD card
    while (1)
    {
        if (!SD_init())
        {
            // error initialising SD card, set LED to red
            rgbLED(255, 0, 0);
            __delay_cycles(100000);
        }
        else
        {
            break;
        }
    }
#endif

#ifdef FLIGHT_MODE
    while (1)
    {
        // ------- INITIALISATION STAGE ------- //
        uint8_t byte = checkFinishStatus();
        // Check if recording has begun to not override existing data
        if (byte)
        {
            // SD card has data, set LED to green
            rgbLED(0, 255, 0);
            break; // exit
        }

        // Verify IMU connection
        if (!checkIMUConnection())
        {
            // IMU initialisation failure, set LED to pink
            rgbLED(255, 0, 255);
            break; // exit
        }
        // initialise IMU
        IMUInit();
        // IMU initialisation complete, set LED to cyan
        rgbLED(0, 255, 255);

        // begin receiving GNSS signals
        GNSS_receive();
        // wait for fix acquired
        while (!fixAcquired())
            ;
        // fix acquired, set LED to yellow
        rgbLED(255, 255, 0);

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
                    // note: cannot set a time in the next day
                    if (((UTC[0] == FLIGHT_START_HOUR)
                            && (UTC[1] >= FLIGHT_START_MIN))
                            || (UTC[0] > FLIGHT_START_HOUR))
                    {
                        // set current stage to recording stage when time reached
                        currentStage = 1;
                    }
                }
                // ------- SET CONDITIONS ------- //
                if (currentStage == 1)
                {
                    // set SD recording status as recording
                    metaData[0] = 1;
                    SD_writeSingleBlock(0, metaData, &token);

                    // TO DO: recording time reached, set LED to rainbow
                    rainbowEffect();
                    // TO DO: set buzzer
                    buzzerOn(500);

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
                        metaData[i + 1] = blockAddressArr[3 - i];
                    }
                    SD_writeSingleBlock(0, metaData, &token);
                }
            }
        }
    }
#endif

#ifdef RESET_MODE
// RESET MODE
  // set LED to off
  rgbLED(0, 0, 0);
  // fill buffer with 0x00
  for (i = 0; i < 512; i++)
  {
      buf[i] = 0;
  }
  // reset metadata block
  SD_writeSingleBlock(0, buf, &token);
  // set LED to white
  rgbLED(255, 255, 255);
#endif

#ifdef DATA_ANALYSIS_MODE
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
#endif

    // keep MSP running
    while (1)
    {
    }

    return 0;
}
