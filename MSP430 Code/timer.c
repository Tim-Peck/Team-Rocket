#include <msp430.h>
#include <inttypes.h>
#include "Defines.h"
#include "timer.h"
#include "GNSS.h" // need to include .h in .c file if you want to use

#define CCR0VALUE 20000

void timerB0_init()
{
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

void timerB2_init()
{
    // set pin 5.0 to timer output mode
    P5SEL0 |= BIT0;

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
    uint16_t CCR0 = 1054430.0/frequency;

    // set CCR0 and CCR1 values
    TB2CCR0 = CCR0;
    TB2CCR1 = CCR0/2; // duty cycle of 50%

    // clear timer
    TB2CTL |= TBCLR;

    // start timer in up mode
    TB2CTL |= MC0;
}

void timerB3_init()
{
    #ifdef USE_DEV_BOARD
    // set LED pins (LAUNCHPAD 2355)
    P1DIR |= BIT0;
    P6DIR |= BIT6;
    #else
    // PCB 2355
    P2DIR |= BIT3; // red
    P2DIR |= BIT2; // green
    P2DIR |= BIT1; // blue
    #endif

    // stop timer
    TB3CTL &= ~(MC0 | MC1);

    // timer B1 clock at default SMCLK clock speed of 1MHz
    TB3CTL |= TBSSEL1;

    // enable CC interrupts
    TB3CCTL0 |= CCIE; // counts to CCR0
    // enable compare registers for duty cycle/width of pulse
    TB3CCTL1 |= CCIE;
    TB3CCTL2 |= CCIE;
    // TB3CCTL3 |= CCIE; // uncomment for actual

    // set CCR0 compare value for cycle period
    TB3CCR0 = CCR0VALUE;

    // clear timer
    TB3CTL |= TBCLR;
    // clear flags
    TB3CCTL0 &= ~CCIFG;
    TB3CCTL1 &= ~CCIFG;
    TB3CCTL2 &= ~CCIFG;
    #ifndef USE_DEV_BOARD
    TB3CCTL3 &= ~CCIFG;
    #endif
}

void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal)
{
    // stop timer
    TB3CTL &= ~(MC0 | MC1);

    // check values for edge cases 0 or 255 to disable respective interrupts
    if (redVal == 255)
    {
        TB3CCTL1 &= ~CCIE;
    }
    else
    {
        TB3CCTL1 |= CCIE;
    }
    if (greenVal == 255)
    {
        TB3CCTL2 &= ~CCIE;
    }
    else
    {
        TB3CCTL2 |= CCIE;
    }
    #ifndef USE_DEV_BOARD
    if (blueVal == 255)
    {
        TB3CCTL3 &= ~CCIE;
    }
    else
    {
        TB3CCTL3 |= CCIE;
    }
    #endif

    // set duty cycle of each LED
    TB3CCR1 = (uint16_t) (CCR0VALUE * (float) redVal / 255);
    TB3CCR2 = (uint16_t) (CCR0VALUE * (float) greenVal / 255);
    #ifndef USE_DEV_BOARD
    TB3CCR3 = (uint16_t) (CCR0VALUE * (float) blueVal / 255);
    #endif

    // clear timer
    TB3CTL |= TBCLR;

    // start timer in up mode
    TB3CTL |= MC0;
}

// IFG for CCR0: 1Hz timer for flight logic
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
  #ifdef USE_DEV_BOARD
    P1OUT ^= BIT0; // TESTING
  #else
    P2OUT ^= BIT3; // TESTING
  #endif
}

// CCR0: pull all LED low to turn on
#pragma vector=TIMER3_B0_VECTOR
__interrupt void TIMER3_B0_ISR(void)
{
    #ifdef USE_DEV_BOARD
    // Launchpad 2355
    P1OUT |= BIT0;
    P6OUT |= BIT6;

    #else

    P2OUT &= ~(BIT1 | BIT2 | BIT3); // actual RGB pins

    #endif
}

// CCR1/2/3 IFG: set respective LED to turn off (sets duty cycle)
#pragma vector=TIMER3_B1_VECTOR
__interrupt void TIMER3_B1_ISR(void)
{
  #ifdef USE_DEV_BOARD
    switch (TB3IV)
    {
    case TBIV__TBCCR1: // red
       P1OUT &= ~BIT0;
        break;
    case TBIV__TBCCR2: // green
       P6OUT &= ~BIT6;
        break;
    default:
        break;
    }
  #else
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

  #endif

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
