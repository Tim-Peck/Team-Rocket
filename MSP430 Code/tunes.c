#include <msp430.h>
#include <inttypes.h>
#include "tunes.h"

const uint32_t tunes_SongLength1 = 58;
const uint32_t tunes_Tune1[58] = {554, 493, 554, 369, 587, 554, 587, 554, 493, 587, 554, 587, 369, 493, 440, 493, 440, 415, 493, 440, 554, 493, 554, 369, 587, 554, 587, 554, 493, 587, 554, 587, 369, 493, 440, 493, 440, 415, 493, 440, 415, 440, 493, 440, 493, 554, 493, 440, 415, 369, 587, 554, 587, 554, 493, 554, 554, 0};
const uint32_t tunes_Rhythm1[58] = {1, 1, 4, 10, 1, 1, 2, 2, 10, 1, 1, 2, 10, 1, 1, 2, 2, 2, 2, 6, 1, 1, 2, 10, 1, 1, 2, 2, 10, 1, 1, 2, 10, 1, 1, 2, 2, 2, 2, 6, 1, 1, 6, 1, 1, 2, 2, 2, 2, 4, 4, 12, 1, 1, 1, 1, 16, 1};

const uint32_t tunes_SongLength2 = 128;
const uint32_t tunes_Tune2[128] = {329, 493, 415, 415, 369, 329, 329, 440, 415, 415, 369, 369, 329, 329, 493, 415, 415, 369, 329, 329, 277, 246, 0, 329, 329, 329, 493, 415, 415, 369, 369, 329, 329, 440, 415, 415, 369, 369, 329, 329, 493, 440, 440, 415, 369, 369, 415, 277, 0, 329, 277, 329, 329, 329, 329, 277, 246, 329, 329, 329, 329, 329, 329, 329, 329, 329, 329, 329, 329, 329, 415, 329, 415, 415, 493, 440, 415, 329, 369, 369, 369, 369, 415, 329, 329, 277, 246, 329, 329, 329, 0, 415, 493, 415, 554, 415, 493, 415, 554, 415, 493, 440, 415, 369, 329, 329, 349, 329, 0, 329, 329, 329, 329, 329, 329, 329, 329, 0, 329, 329, 415, 329, 329, 329, 277, 277, 277, 0};
const uint32_t tunes_Rhythm2[128] = {4, 2, 2, 4, 2, 2, 2, 4, 2, 2, 2, 2, 4, 2, 2, 2, 4, 2, 2, 2, 4, 6, 4, 2, 1, 1, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 4, 2, 2, 4, 2, 2, 4, 6, 4, 3, 1, 2, 2, 1, 1, 1, 1, 2, 2, 1, 3, 2, 1, 1, 2, 1, 1, 1, 1, 2, 1, 3, 1, 1, 2, 2, 1, 1, 2, 2, 3, 1, 2, 2, 2, 1, 1, 2, 2, 2, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 2, 2, 2, 2, 1, 2, 2, 1, 1, 1, 1, 2, 1, 1, 2, 4, 2, 1, 1, 1, 1, 2, 1, 1, 2, 4, 4};

const uint32_t tunes_noteGap = 1;


volatile uint32_t tunes_currNote = 0;
volatile uint32_t tunes_currSemis = 0;
volatile uint32_t tunes_noteGapCounter = 0;
volatile uint8_t tunes_songEnd = 0;


void tunes_PlayCMajAppegio() {
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
}

void tunes_PlaySong() {
  buzzerOff();
  tunes_currSemis = 0;
  tunes_currNote = 0;
  tunes_noteGapCounter = 0;
  tunes_songEnd = 0;
}

void timerB1_init() {
    // stop timer
    TB1CTL &= ~(MC0 | MC1);

    // note: TBSSEL1 is control bit for second TBSSEL bit (so select SMCLK)
    // note: TAR has max 2^16 - 1 = 65535 so to count to 1 second with 1MHz clock, we can divide by 32 (ID divide by 8, TBIDEX divide by 4) to get 32768Hz which we set as our CCR0 for 1Hz
    TB1CCR0 = 3000; // modified from 32768 to get 1Hz
    TB1CTL |= TBSSEL1 | ID1 | ID0;
    TB1EX0 |= TBIDEX0 | TBIDEX1;

    // enable CCR0 interrupt
    TB1CCTL0 |= CCIE;

    // clear timer
    TB1CTL |= TBCLR;
    // clear flag
    TB1CCTL0 &= ~CCIFG;

    // Start the timer in up mode
    TB1CTL |= MC0;
}

void songStep1() {
  if (tunes_songEnd == 1) {return;}
  if (tunes_currSemis >= tunes_Rhythm1[tunes_currNote]) {
    tunes_currSemis = 0;
    tunes_currNote ++;
    buzzerOn(tunes_Tune1[tunes_currNote]);
    if (tunes_currNote == tunes_SongLength1) {
      tunes_currNote = 0;
    }
    return;
  }
  tunes_currSemis ++;
}

void songStep2() {
  if (tunes_songEnd == 1) {return;}
  if (tunes_currSemis >= tunes_Rhythm2[tunes_currNote]) {
    tunes_currSemis = 0;
    tunes_currNote ++;
    buzzerOn(tunes_Tune2[tunes_currNote]);
    if (tunes_currNote == tunes_SongLength2) {
      tunes_currNote = 0;
    }
    return;
  }
  tunes_currSemis ++;
}

void songStepSelect() {
  if (tunes_songSelect == 1) {
    songStep1();
  } else if (tunes_songSelect == 2) {
    songStep2();
  }
}

#pragma vector=TIMER1_B0_VECTOR
__interrupt void TIMER1_B0_ISR(void) {
  __enable_interrupt();  // enable global interrupts
  songStepSelect();
}
