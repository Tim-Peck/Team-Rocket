// Note: Maximum amount of time available for writing data is 24 hours

 #define TEST_LED
// #define TEST_TIMER
// #define TEST_UART
// #define TEST_GNSS
// #define TEST_SD
 #define TEST_IMU
// #define TEST_ADC

#include <msp430.h>
#include <inttypes.h>

#include "Defines.h"
#include "I2C.h"
#include "UART.h"
#include "SPI.h"
#include "timer.h"
#include "GNSS.h"
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

#ifdef TEST_SD
  #ifndef TEST_UART
  uart_init();
  #endif
  spi_init(); // SD Card SPI
#endif

#ifdef TEST_IMU
  i2c_init(); // IMU I2C
#endif

#ifdef TEST_ADC
  // adc_init();
#endif


// ---------- TESTING ---------- //

#ifdef TEST_LED
  // LED testing
  digital_write(redLEDPin, HIGH);
  digital_write(greenLEDPin, HIGH);
  #ifndef USE_DEV_BOARD
  digital_write(blueLEDPin, HIGH);
  #endif
  rgbLED(255, 255, 0);

  __delay_cycles(1000000);

  rgbLED(0, 255, 255);

   __delay_cycles(1000000);

  rgbLED(255, 255, 0);

#endif

#ifdef TEST_TIMER
// Timer testing // Launchpad VERIFIED, PCB VERIFIED

//Main 1Hz timer
begin1HzTimer();

//Tone - buzzer
// TO-DO

#endif

#ifdef TEST_UART
  // Serial UART testing // Launchpad not verified fully working, PCB NOT WORKING
  // MCLK/SMCLK output
  P3SEL0 |= BIT0 | BIT4;
  P3DIR |= BIT0 | BIT4;

  uart_send_byte('X');
  uart_send_bytes("hello world", 11); // could not verify fully working, only works on flash
#endif

#ifdef TEST_GNSS
  // GNSS testing // Launchpad VERIFIED, PCB VERIFIED

  rgbLED(255, 0, 0);

  GNSS_receive();

  while (!fixAcquired());

  rgbLED(0, 255, 0);

#endif

#ifdef TEST_SD
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
    if (buf[4] = 0x02) {
    rgbLED(0, 255, 0); // received correct: temporary for testing
    } else {
    rgbLED(0, 0, 255);
    }
  }
  else
  {
    uart_send_bytes("SD Initialization Failure\r",
    sizeof("SD Initialization Failure\r"));
    rgbLED(255, 0, 0);
  }
#endif

#ifdef TEST_IMU
  // I2C testing // Launchpad VERIFIED, PCB VERIFIED

  if (checkIMUConnection()){
    digital_write(greenLEDPin, LOW); // received correct: temporary for testing
  } else {
    digital_write(redLEDPin, LOW);
  }
#endif

#ifdef TEST_ADC
#endif



  // keep MSP running
  while (1)
  {
  }

  return 0;
}
