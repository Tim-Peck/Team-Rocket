#include <msp430.h>
#include <inttypes.h>
#include "timer.h"
#include "GNSS.h" // need to include .h in .c file if you want to use

#define CCR0VALUE 20000

void timerB0_init()
{
    // TIMER TESTING
//    P2DIR |= BIT3; // red

// stop timer
    TB0CTL &= ~(MC0 | MC1);

    // note: TBSSEL1 is control bit for second TBSSEL bit (so select SMCLK)
    // note: TAR has max 2^16 - 1 = 65535 so to count to 1 second with 1MHz clock, we can divide by 32 (ID divide by 8, TBIDEX divide by 4) to get 32768Hz which we set as our CCR0 for 1Hz
    TB0CCR0 = 33040; // modified from 32768 to get 1Hz
    TB0CTL |= TBSSEL1 | ID1 | ID0;
    TB0EX0 |= TBIDEX0 | TBIDEX1;
    // enable CCR0 interrupt
    TB0CCTL0 |= CCIE;

    // clear timer
    TB0CTL |= TBCLR;
    // clear flag
    TB0CCTL0 &= ~CCIFG;
}

void begin1HzTimer()
{
    // start timer in up mode
    TB0CTL |= MC0;
}

// IFG for CCR0: 1Hz timer for flight logic
#pragma vector=TIMER0_B0_VECTOR // CHANGE FOR FR2355
__interrupt void TIMER0_B0_ISR(void)
{
    // TIMER TESTING
//    P2OUT ^= BIT3;
}

void timerB2_init()
{
    // set pin 5.0 to timer output mode
    P5SEL0 |= BIT0;
    // set pin direction
    P5DIR |= BIT0;

    // stop timer
    TB2CTL &= ~(MC0 | MC1);

    // timer B1 clock at default SMCLK clock speed of 1MHz (1054430Hz)
    TB2CTL |= TBSSEL1;

    // set output mode for CCR1 to Reset/Set mode
    // (automatic timer output on pin)
    TB2CCTL1 |= OUTMOD0 | OUTMOD1 | OUTMOD2;

    // clear timer
    TB2CTL |= TBCLR;
}

void buzzerOn(int frequency)
{
    // stop timer
    TB2CTL &= ~(MC0 | MC1);

    // calculate CCR0 value from frequency
    // note: counts per second/counts per cycle = frequency
    uint16_t CCR0 = 1054430.0 / frequency;

    // set CCR0 and CCR1 values
    TB2CCR0 = CCR0;
    TB2CCR1 = CCR0 / 2; // duty cycle of 50%

    // clear timer
    TB2CTL |= TBCLR;

    // start timer in up mode
    TB2CTL |= MC0;
}

void timerB3_init()
{
    // stop timer
    TB3CTL &= ~(MC0 | MC1);

    // timer B3 clock at default SMCLK clock speed of 1MHz
    TB3CTL |= TBSSEL1;

    // enable CC interrupts
    TB3CCTL0 |= CCIE; // counts to CCR0

    // set CCR0 compare value for cycle period
    TB3CCR0 = CCR0VALUE;

    // clear timer
    TB3CTL |= TBCLR;
    // clear flags
    TB3CCTL0 &= ~CCIFG;
    TB3CCTL1 &= ~CCIFG;
    TB3CCTL2 &= ~CCIFG;
    TB3CCTL3 &= ~CCIFG;
}

void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal)
{
    // stop timer
    TB3CTL &= ~(MC0 | MC1);

    // set pins to be out (on)
    // PCB 2355
    P2DIR |= BIT3; // red
    P2DIR |= BIT2; // green
    P2DIR |= BIT1; // blue

    // check values for edge cases 0 or 255
    // if 0, set pin to inactive (direction in)
    // if 255, disable interrupt to reset LED
    // if other value, enable CC registers for duty cycle/width of pulse
    if (redVal == 255)
    {
        TB3CCTL1 &= ~CCIE;
    }
    else if (!redVal)
    {
        P2DIR &= ~BIT3;
    }
    else
    {
        TB3CCTL1 |= CCIE;
    }
    if (greenVal == 255)
    {
        TB3CCTL2 &= ~CCIE;
    }
    else if (!greenVal)
    {
        P2DIR &= ~BIT2;
    }
    else
    {
        TB3CCTL2 |= CCIE;
    }
    if (blueVal == 255)
    {
        TB3CCTL3 &= ~CCIE;
    }
    else if (!blueVal)
    {
        P2DIR &= ~BIT1;
    }
    else
    {
        TB3CCTL3 |= CCIE;
    }

    // set duty cycle of each LED
    TB3CCR1 = (uint16_t) (CCR0VALUE * (float) redVal / 255);
    TB3CCR2 = (uint16_t) (CCR0VALUE * (float) greenVal / 255);
    TB3CCR3 = (uint16_t) (CCR0VALUE * (float) blueVal / 255);

    // clear timer
    TB3CTL |= TBCLR;

    // start timer in up mode
    TB3CTL |= MC0;
}

void rainbowEffect()
{
    rainbowOn = 1;
    rainbowPhase = 1;
    RGB[0] = 255;
    RGB[1] = 0;
    RGB[2] = 0;

    rgbLED(RGB[0], RGB[1], RGB[2]); // begin from red to yellow and then the other colours
}

// CCR0: pull all LED low to turn on
// if rainbow effect on, transition through colours by setting next CCR values
#pragma vector=TIMER3_B0_VECTOR
__interrupt void TIMER3_B0_ISR(void)
{
    P2OUT &= ~(BIT1 | BIT2 | BIT3); // actual RGB pins

    // transition through 6 colours
    if (rainbowOn)
    {
        switch (rainbowPhase)
        {
        case 1: // phase 1 (red -> yellow)
            if (RGB[1] != 255)
            {
                rgbLED(255, ++RGB[1], 0);
            }
            else
            {
                rainbowPhase = 2;
                rgbLED(255, 255, 0);
            }
            break;
        case 2: // phase 2 (yellow -> green)
            if (RGB[0] != 0)
            {
                rgbLED(--RGB[0], 255, 0);
            }
            else
            {
                rainbowPhase = 3;
                rgbLED(0, 255, 0);
            }
            break;
        case 3: // phase 3 (green -> cyan)
            if (RGB[2] != 255)
            {
                rgbLED(0, 255, ++RGB[2]);
            }
            else
            {
                rainbowPhase = 4;
                rgbLED(0, 255, 255);
            }
            break;
        case 4: // phase 4 (cyan -> blue)
            if (RGB[1] != 0)
            {
                rgbLED(0, --RGB[1], 255);
            }
            else
            {
                rainbowPhase = 5;
                rgbLED(0, 0, 255);
            }
            break;
        case 5: // phase 5 (blue -> magenta)
            if (RGB[0] != 255)
            {
                rgbLED(++RGB[0], 0, 255);
            }
            else
            {
                rainbowPhase = 6;
                rgbLED(255, 0, 255);
            }
            break;
        case 6: // phase 6 (magenta -> red)
            if (RGB[2] != 0)
            {
                rgbLED(255, 0, --RGB[2]);
            }
            else
            {
                rainbowPhase = 1;
                rgbLED(255, 0, 0);
            }
            break;
        }
    }
}

// CCR1/2/3 IFG: set respective LED to turn off (sets duty cycle)
#pragma vector=TIMER3_B1_VECTOR
__interrupt void TIMER3_B1_ISR(void)
{
    switch (TB3IV)
    {
    case TBIV__TBCCR1: // red
        P2OUT |= BIT3;
        break;
    case TBIV__TBCCR2: // green
        P2OUT |= BIT2;
        break;
    case TBIV__TBCCR3: // blue
        P2OUT |= BIT1;
        break;
    default:
        break;
    }
}

uint32_t convert_uint8_array_to_uint32(uint8_t *arr)
{
    uint32_t num = 0;

    // combine the four uint8_t bytes into a single uint32_t variable using bitwise operations
    num |= ((uint32_t) arr[0]) << 24;
    num |= ((uint32_t) arr[1]) << 16;
    num |= ((uint32_t) arr[2]) << 8;
    num |= (uint32_t) arr[3];

    return num;
}

void convert_uint32_to_uint8_array(uint32_t num, uint8_t *arr)
{
    // note, this is little endian
    memcpy(arr, &num, sizeof(uint32_t)); // note num is a copy but it doesn't matter as it still contains the same values
}
