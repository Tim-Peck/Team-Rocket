#include <msp430.h>
#include <inttypes.h>
#include "timer.h"
#include "GNSS.h" // need to include .h in .c file if you want to use

#define CCR0VALUE 20000

void timerB0_init() // CHANGE FOR FR2355
{
    // stop timer
    TA0CTL &= ~(MC0 | MC1);

    // note: TASSEL1 is control bit for second TASSEL bit (so select SMCLK)
    // note: TAR has max 2^16 - 1 = 65535 so to count to 1 second with 1MHz clock, we can divide by 16 (ID divide by 8, TAIDEX divide by 2) to get 65536Hz so ID and TAIDEX bits used
    TA0CTL |= TASSEL1 | ID1 | ID0 | TAIE;
    TA0EX0 |= TAIDEX0;

    // clear timer
    TA0CTL |= TACLR;
    // clear flag
    TA0CTL &= ~TAIFG;
}

void beginRecord() // CHANGE FOR FR2355
{
    // start timer in continuous mode
    TA0CTL |= MC1;
}

#pragma vector=TIMER0_A1_VECTOR // CHANGE FOR FR2355
__interrupt void TIMER0_A1_ISR(void)
{
    switch (TA0IV)
    {
    case TA0IV_TAIFG:
        P1OUT ^= BIT0;

        __bis_SR_register(GIE);
        if (fixAcquired())
        {
            uart_send_bytes("FIX ACQUIRED\r", 13);
        }
        else
        {
            uart_send_bytes("NO FIX\r", 7);
        }
        break;
    default:
        break;
    }
}

void timerB1_init() // CHANGE FOR FR2355
{
    // stop timer
    TA1CTL &= ~(MC0 | MC1);

    // timer B1 clock at default clock speed of 1MHz
    TA1CTL |= TASSEL1;

    // enable CC interrupts
    TA1CCTL0 |= CCIE;
    TA1CCTL1 |= CCIE;
    TA1CCTL2 |= CCIE;
    // TA1CCTL3 |= CCIE; // uncomment for actual

    // set CCR0 compare value for cycle period
    TA1CCR0 = CCR0VALUE;

    // clear timer
    TA1CTL |= TACLR;
    // clear flags
    TA1CCTL0 &= ~CCIFG;
    TA1CCTL1 &= ~CCIFG;
    TA1CCTL2 &= ~CCIFG;
    //TB1CCTL3 &= ~CCIFG;
}

void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal)
{
    // stop timer
    TA1CTL &= ~(MC0 | MC1);

    // check values for edge cases 0 or 255 to disable respective interrupts
    if (redVal == 255)
    {
        TA1CCTL1 &= ~CCIE;
    }
    else
    {
        TA1CCTL1 |= CCIE;
    }
    if (greenVal == 255)
    {
        TA1CCTL2 &= ~CCIE;
    }
    else
    {
        TA1CCTL2 |= CCIE;
    }
//    if (blueVal == 255)
//    {
//        TA1CCTL3 &= ~CCIE;
//    }
//    else
//    {
//        TA1CCTL3 |= CCIE;
//    }

    // set duty cycle of each LED
    TA1CCR1 = (uint16_t) (CCR0VALUE * (float) redVal / 255);
    TA1CCR2 = (uint16_t) (CCR0VALUE * (float) greenVal / 255);
    // TB1CCR3 = (uint16_t)(1000 * (float)blueVal/255);

    // clear timer
    TA1CTL |= TACLR;

    // start timer in up mode
    TA1CTL |= MC0;
}

// CC0IFG: set all LED
#pragma vector=TIMER1_A0_VECTOR // CHANGE FOR FR2355
__interrupt void TIMER1_A0_ISR(void)
{
    P1OUT |= BIT0 | BIT1;
}

// CCR1/2/3 IFG: reset respective LED
#pragma vector=TIMER1_A1_VECTOR // CHANGE FOR FR2355
__interrupt void TIMER1_A1_ISR(void)
{
    switch (TA1IV)
    {
    case TA1IV_TACCR1:
        P1OUT &= ~BIT0;
        break;
    case TA1IV_TACCR2:
        P1OUT &= ~BIT1;
        break;
        //case TB1IV_TBCCR3:
        //P2OUT &= ~BIT3
        //break;
    default:
        break;
    }
}
