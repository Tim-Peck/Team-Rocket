// Note: Maximum amount of time available for writing data is 24 hours

#include <msp430.h>
#include <inttypes.h>
#include "SPI.h"
#include "UART.h"

#define rgbLED(X,Y,Z) blip()


static uint8_t dataArray[25];

// SD variables
static uint8_t R1, metaData[512], buf[512], token;
static uint32_t i;
static uint32_t blockAddress;
static uint8_t blockAddressArr[4];

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

    spi_init(); // SD Card SPI

    // SD card testing // Launchpad VERIFIED, PCB VERIFIED

     // initialize SD card
    if (SD_init())
    {
       // fill buffer
       for (i = 0; i < 512; i++)
       {
           buf[i] = 0x00;
       }

       // write a block to SD card to address 0x100 (256)
//        SD_writeSingleBlock(0, buf, &token);

       // read block 0 from SD card
       R1 = SD_readSingleBlock(0, buf, &token);

       // print read SD block
       print_SDBlock(R1, buf, &token);

       // check if contents are correct
       if (buf[0] == 0xAA) {
           rgbLED(0, 255, 0); // received correct: temporary for testing
       } else {
           rgbLED(0, 0, 255);
       }

      uint8_t res[17];
      SD_sendCSD_Command(&res);

      int block_len = ( (res[1] & (BIT0 | BIT1)) | (res[0] & (BIT6 | BIT7)));
      if (block_len == 9) {
        rgbLED(200, 200, block_len); // received correct
      }
    }
    else
    {
       uart_send_bytes("SD Initialization Failure\r",
                       sizeof("SD Initialization Failure\r"));
       rgbLED(255, 0, 0);
    }


    // keep MSP running
    while (1)
    {
    }

    return 0;
}
