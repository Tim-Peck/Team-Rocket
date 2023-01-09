// 13/12/2022
// Last change: Kenny Yu
// Current function: displays received byte/s on serial monitor with UART from SPI on test microcontroller with test sensors

#include <msp430.h>
#include <inttypes.h>
#include "I2C.h"
#include "UART.h"
#include "SPI.h"

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

/* TO BE DELETED AS DELAY_CYCLES CANNOT TAKE IN VARIABLES
 // note: temporary until timer
 // one period (~1kHz) for rgb led using PWM by changing duty cycle of each colour
 // input: RGB value from 0 to 255
 void led_write(int redValue, int greenValue, int blueValue)
 {
 //    int periodCycles = (double)1048576*1/1000; // cycles per second * time required
 //    int values[3] = { redValue, greenValue, blueValue };
 //    uint8_t pins[3] = { redLEDPin, greenLEDPin, blueLEDPin };
 //
 //    // insertion sort the three values from smallest to largest
 //    int tempShortest;
 //    int tempPos;
 //
 //    uint8_t tempPin;
 //
 //    int i;
 //    int j;
 //    for (i = 0; i < 2; i++)
 //    {
 //        // default values
 //        tempShortest = values[i];
 //        tempPos = i;
 //
 //        tempPin = pins[i];
 //
 //        // linear search for lowest number
 //        for (j = i; j < 3; j++)
 //        {
 //            if (values[j] < tempShortest)
 //            {
 //                tempPos = j;
 //                tempShortest = values[j];
 //
 //                // record index/pin change
 //                tempPin = pins[j];
 //
 //            }
 //        }
 //
 //        // swap
 //        values[tempPos] = values[i];
 //        values[i] = tempShortest;
 //
 //        pins[tempPos] = pins[i];
 //        pins[i] = tempPin;
 //    }
 //
 //
 //    // LED cycle
 //
 //    // turn all LED on
 //    digital_write(redLEDPin, HIGH);
 //    digital_write(greenLEDPin, HIGH);
 //    digital_write(blueLEDPin, HIGH);
 //
 //    // wait shortest time
 //    __delay_cycles((int)((double)values[0]/255 * periodCycles));
 //    // turn respective LED off
 //    digital_write(pins[0], LOW);
 //
 //    // wait second shortest time
 //    __delay_cycles((int)(((double)values[1]/255 - values[0]/255) * periodCycles));
 //    // turn respective LED off
 //    digital_write(pins[1], LOW);
 //
 //    // wait third time
 //    __delay_cycles((int)(periodCycles - (double)values[2]/255));
 //    // turn respective LED off
 //    digital_write(pins[2], LOW);
 }

 // note: temporary until timer
 // sound the piezo buzzer for one second at a specific frequency
 // input: frequency in Hz (range of 31 Hz to 65535 Hz), time buzzer is on in seconds
 void tone(int freq)
 {
 int periodCycles = (double)1048576*1/freq; // MCU cycles for one piezo period (cycles per second * time required)
 double period = (double)1/periodCycles;

 int numCycles = 1/period; // number of cycles for piezo to sound for one second
 int cycleCount = 0;

 // cycle for 1 second
 while (cycleCount < numCycles) {
 // 50% duty cycle for piezo
 P5OUT |= BIT0;
 //        __delay_cycles(periodCycles/2);
 __delay_cycles(53);
 P5OUT &= ~BIT0;
 //        __delay_cycles(periodCycles/2);
 __delay_cycles(53);

 cycleCount++;
 }
 }
 */

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // disable the GPIO power-on default high-impedance mode
                          // to activate previously configured port settings

    __enable_interrupt();  // enable global interrupts
    __delay_cycles(90000); // GIE (or watchdog?) needs time before set

    // ------------------------- DO NOT CHANGE ABOVE ---------------------------- //

    // SD card testing

    uart_init();
    //    i2c_init();
    spi_init(); // initialise microcontroller SPI for SD card

    if (SD_init()) {
        uart_send_bytes("SD Initialization Success\r", sizeof("SD Initialization Success\r"));
    } else {
        uart_send_bytes("SD Initialization Failure\r", sizeof("SD Initialization Failure\r"));
    }

    // keep MSP running
    while (1)
    {
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
