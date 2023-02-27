
// Unit tests
#define TEST_LED
//#define TEST_TIMER
//#define TEST_UART
//#define TEST_GNSS
//#define TEST_GNSS_FIX
//#define TEST_SD
// #define TEST_IMU
//#define TEST_ADC
//#define TEST_BUZZER

// Flight type list
// FLIGHT MODE (Flight logic - see flow chart)
// #define FLIGHT_MODE

// RESET MODE (Erase all content by setting all bytes to 0)
// #define RESET_MODE

// DATA ANALYSIS MODE (View complete flight data)
// #define DATA_ANALYSIS_MODE

#define FLIGHT_START_MIN 13
#define FLIGHT_START_HOUR 17

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
  IMUInit();
#endif

#ifdef TEST_ADC
  adc_init();
#endif

#ifdef TEST_BUZZER
  timerB2_init();
#endif


// ---------- TESTING ---------- //

#ifdef TEST_LED
  // LED testing
  digital_write(redLEDPin, HIGH);
  digital_write(greenLEDPin, HIGH);
  #ifndef USE_DEV_BOARD
  digital_write(blueLEDPin, HIGH);
  #endif
  rgbLED(255, 0, 255);

  __delay_cycles(1000000);

  rgbLED(0, 255, 255);

   __delay_cycles(1000000);

  rgbLED(255, 0, 255);

  __delay_cycles(1000000);

#endif

#ifdef TEST_TIMER
  rgbLED(0, 0, 255);
  __delay_cycles(1000000);
  // Timer testing // Launchpad VERIFIED, PCB VERIFIED

  //Main 1Hz timer
  begin1HzTimer();

  //Tone - buzzer
  // TO-DO

__delay_cycles(1000000);

#endif

#ifdef TEST_UART
  rgbLED(0, 0, 255);
  __delay_cycles(1000000);
  // Serial UART testing // Launchpad not verified fully working, PCB NOT WORKING
  // MCLK/SMCLK output
  P3SEL0 |= BIT0 | BIT4;
  P3DIR |= BIT0 | BIT4;

  uart_send_byte('X');
  uart_send_bytes("hello world", 11); // could not verify fully working, only works on flash

  __delay_cycles(1000000);
#endif

// #ifdef TEST_GNSS
//
//   rgbLED(0, 0, 255);
//   __delay_cycles(1000000);
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

#ifdef TEST_GNSS_FIX

  rgbLED(0, 0, 255);
  __delay_cycles(1000000);
  // GNSS testing // Launchpad VERIFIED, PCB VERIFIED

  rgbLED(255, 0, 0);

  GNSS_receive();

  while (!fixAcquired());

  rgbLED(0, 255, 0);

  __delay_cycles(1000000);

#endif

#ifdef TEST_SD
  rgbLED(0, 0, 255);
  __delay_cycles(1000000);
  // SD card testing // Launchpad VERIFIED, PCB VERIFIED
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

    // write a block to SD card to address 0x100 (256)
    SD_writeSingleBlock(0x100, buf, &token);

    // read block 0 from SD card
    R1 = SD_readSingleBlock(0x100, buf, &token);

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

#ifdef TEST_IMU
  rgbLED(0, 0, 255);
  __delay_cycles(1000000);
  // I2C testing // Launchpad VERIFIED, PCB VERIFIED

  if (checkIMUConnection()){
    uint8_t accelCalibLevel;
    // wait for IMU to be calibrated
    accelCalibLevel = readAccelCalib();
    while(accelCalibLevel != 3) {
      // set brightness of LED to reflect calibration level
      rgbLED(255*(accelCalibLevel+1)/4.0, 255*(accelCalibLevel+1)/4.0, 0);
      accelCalibLevel = readAccelCalib();
    }
    // set LED to green
    rgbLED(0, 255, 0);
  } else {
    rgbLED(255, 0, 0);
  }

  __delay_cycles(1000000);
#endif

#ifdef TEST_ADC
  rgbLED(0, 0, 255);
  __delay_cycles(1000000);

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

#ifdef TEST_BUZZER
  rgbLED(0, 0, 255);
  __delay_cycles(1000000);

  // rgbLED(0, 255, 0);
  buzzerOn(500);
  // __delay_cycles(1000000);
  // buzzerOn(262);
  // __delay_cycles(1000000);
  //
  // buzzerOn(330);
  // __delay_cycles(1000000);
  //
  // buzzerOn(392);
  // __delay_cycles(1000000);
  //
  // buzzerOn(523);
  // __delay_cycles(1000000);
  //
  // buzzerOn(392);
  // __delay_cycles(1000000);
  //
  // buzzerOn(330);
  // __delay_cycles(1000000);
  //
  // buzzerOn(262);
  // __delay_cycles(1000000);

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

rgbLED(255, 255, 255);


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

// Initialise SD card
while (1) {
  if (!SD_init())
  {
    // error initialising SD card, set LED to red
    rgbLED(255, 0, 0);
    __delay_cycles(100000);
  } else {
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
          // set LED to blue
          rgbLED(0, 0, 255);
          break; // exit
      }
      // Verify IMU connection // UNCOMMENT FOR IMU
      if (checkIMUConnection()){
        uint8_t accelCalibLevel;
        // wait for IMU to be calibrated
        while((accelCalibLevel = readAccelCalib()) != 3) {
          // set brightness of LED to reflect calibration level
          rgbLED(255*(accelCalibLevel+1)/4.0, 255*(accelCalibLevel+1)/4.0, 0);
        }
        // set LED to green
        rgbLED(255, 255, 255);
      } else {
        rgbLED(255, 0, 255);
      }
      // initialise IMU // UNCOMMENT FOR IMU
      IMUInit();
      // Set LED to turqourice
      rgbLED(0, 255, 255);
      // begin receiving GNSS signals
      GNSS_receive();
      // wait for fix acquired
      while (!fixAcquired());
      // set LED to yellow
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
                  if (((UTC[0] == FLIGHT_START_HOUR) && (UTC[1] >= FLIGHT_START_MIN)) || (UTC[0] > FLIGHT_START_HOUR))
                  {
                      // set current stage to recording stage
                      currentStage = 1;
                  }
              }
              // ------- SET CONDITIONS ------- //
              if (currentStage == 1)
              {
                  // TO DO: set LED to green
                  rgbLED(0, 255, 0);
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
#endif

#ifdef RESET_MODE
// RESET MODE
  // set LED to off
  rgbLED(0, 0, 0);
  // fill buffer with 0x00
  for (i = 0; i < 512; i++)
  {
      buf[i] = 0x00;
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
