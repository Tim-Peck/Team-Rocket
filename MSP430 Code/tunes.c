#include <msp430.h>
#include <inttypes.h>
#include "tunes.h"

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
  tunes_currSemis = 0;
  tunes_currNote = 0;
  buzzerOn(tunes_Tune[tunes_currNote]);
  timerB2_init();
}

void tunes_SongStep() {
  if (tunes_currSemis >= tunes_Rhythm[tunes_currNote]*tunes_tempox1Hz) {
    if (tunes_currNote < tunes_SongLength-1) {
      tunes_currSemis = 0;
      tunes_currNote ++;
      buzzerOn(tunes_Tune[tunes_currNote]);
    } else {
      buzzerOff();
    }
  } else {
    tunes_currSemis ++;
  }
}
