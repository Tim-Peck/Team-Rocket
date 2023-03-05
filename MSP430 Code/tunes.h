#ifndef TUNES_H
#define TUNES_H

#include <inttypes.h>

volatile uint8_t tunes_songSelect;

// Plays a cmaj appegio using delay cycles
void tunes_PlayCMajAppegio();

// initalises the tunes timer and starts playing
void tunes_PlaySong();

// function for tunes timer
void timerB1_init();

#endif
