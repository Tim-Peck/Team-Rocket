#include <msp430.h>
#include <inttypes.h>
#include "timer.h"
#include "GNSS.h" // need to include .h in .c file if you want to use

#define CCR0VALUE 20000

// -- USER ENTRY -- //
// enter valid launch time in NZDT
static uint8_t launchHour = 20; // 0 to 23
static uint8_t launchMinute = 25; // 0 to 60
// -- USER ENTRY -- //

void timerB0_init() // CHANGE FOR FR2355
{
    // set first stage as flight ready
    currentStage = 0;

    // change time to 1 minute before launch for comparing
    if (launchMinute == 0)
    {
        if (launchHour == 0)
        {
            launchHour = 23;
            launchMinute = 59;
        }
        else
        {
            launchHour -= 1;
            launchMinute = 59;
        }
    }
    else
    {
        launchMinute -= 1;
    }

    // stop timer
    TA0CTL &= ~(MC0 | MC1);

    // note: TASSEL1 is control bit for second TASSEL bit (so select SMCLK)
    // note: TAR has max 2^16 - 1 = 65535 so to count to 1 second with 1MHz clock, we can divide by 32 (ID divide by 8, TAIDEX divide by 4) to get 32768Hz which we set as our CCR0 for 1Hz
    TA0CCR0 = 33040;
    TA0CTL |= TASSEL1 | ID1 | ID0;
    TA0EX0 |= TAIDEX0 | TAIDEX1;
    // enable CCR0 interrupt
    TA0CCTL0 |= CCIE;

    // clear timer
    TA0CTL |= TACLR;
    // clear flag
    TA0CCTL0 &= ~CCIFG;
}

void begin1HzTimer() // CHANGE FOR FR2355
{
    // start timer in up mode
    TA0CTL |= MC0;
}

// TA0CC0IFG: 1Hz timer for flight logic
#pragma vector=TIMER0_A0_VECTOR // CHANGE FOR FR2355
__interrupt void TIMER0_A0_ISR(void)
{
    // ------- FLIGHT READY STAGE ------- //

    // check if time is T-1 minute to launch time to go to recording stage
    if (currentStage == 0)
    {
        // get current UTC time
        parse_GGA_UTC(UTC);

        // convert current UTC time to NZDT (NZDT is 13 hours ahead of UTC)
        UTC[0] = (UTC[0] + 13) % 24;

        // check if T-1 minute reached
        if ((UTC[0] == launchHour) & (UTC[1] == launchMinute))
        {
            // set current stage to recording stage
            currentStage = 1;
        }
    }

    if (currentStage == 1)
    {
        // set LED to rainbow
        rgbLED(255, 255, 0);
        currentStage = 2;
    }
    //        // check fix acquired before parsing NMEA_sentence
    //        __bis_SR_register(GIE);
    //        if (fixAcquired())
    //        {
    //            // print altitude
    //            float f = parse_GGA_alt();
    //            uint8_t array[4];
    //            float_to_uint8(f, array);
    //            uart_send_bytes(array, 4);
    //
    //            // print UTC time
    //            uint8_t UTC[3];
    //            parse_GGA_UTC(UTC);
    //            uart_send_bytes(UTC, 3);
    //            uart_send_byte(100); // FOR TESTING - REMOVE
    //
    //            // print latitude/longitude
    //            float GCS[2];
    //            parse_GGA_GCS(GCS);
    //            uint8_t latitude[4], longitude[4];
    //            float_to_uint8(GCS[0], latitude);
    //            float_to_uint8(GCS[1], longitude);
    //            uart_send_bytes(latitude, 4);
    //            uart_send_bytes(longitude, 4);
    //        }
    //        else
    //        {
    ////            uart_send_bytes("NO FIX\r", 7);
    //        }
}

void timerB1_init() // CHANGE FOR FR2355
{
    // stop timer
    TA1CTL &= ~(MC0 | MC1);

    // timer B1 clock at default clock speed of 1MHz
    TA1CTL |= TASSEL1;

    // enable CC interrupts
    TA1CCTL0 |= CCIE; // counts to CCR0
    // enable compare registers for duty cycle/width of pulse
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

// TA1CC0IFG: set all LED
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
