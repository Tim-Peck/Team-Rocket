#ifndef TUNES
#define TUNES

// SmashMouth
// const int tunes_SongLength = 23;
// const float tunes_Tune[23] = {329, 493, 392, 392, 349, 329, 329, 440, 392, 392, 349, 349, 329, 329, 493, 392, 392, 349, 329, 329, 261, 246, 1};
// const int tunes_Rhythm[23] = {4, 2, 2, 4, 2, 2, 2, 4, 2, 2, 2, 2, 4, 2, 2, 2, 4, 2, 2, 2, 4, 6, 4};

// The Final Countdown
const int tunes_SongLength = 41;
const float tunes_Tune[41] = {554, 493, 554, 369, 587, 554, 587, 554, 493, 587, 554, 587, 369, 493, 440, 493, 440, 415, 493, 440, 554, 493, 554, 369, 587, 554, 587, 554, 493, 587, 554, 587, 369, 493, 440, 493, 440, 415, 493, 440, 0};
const int tunes_Rhythm[41] = {1, 1, 4, 10, 1, 1, 2, 2, 10, 1, 1, 2, 10, 1, 1, 2, 2, 2, 2, 6, 1, 1, 2, 10, 1, 1, 2, 2, 10, 1, 1, 2, 10, 1, 1, 2, 2, 2, 2, 6, 1};

const int tunes_bmp = 120;
const int tunes_tempox1Hz = 15000/tunes_bmp;

volatile int tunes_currNote = 0;
volatile int tunes_currSemis = 0;

void tunes_PlayCMajAppegio();
void tunes_PlaySong();
void tunes_SongStep();

#endif
