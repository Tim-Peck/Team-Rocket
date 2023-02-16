#ifndef CAM_H
#define CAM_H

#include <inttypes.h>

const uint8_t cam_I2CAddress = 0x24;
static volatile uint8_t imageLineBuffer[512];
static volatile uint16_t imageCounter = 0;

// initalises the camera pins and changes required registers via I2C.
void camera_init();

// Sends a request to the camera
// Records the image response to the buffer.
// Saves the image using the saveImage function
void takeImage();

// Save the image to the SD card
void saveImageLine(uint16_t lineNum);

// Sends an I2C message to the camera to take a photo
void requestImage();

// Polls until the PXClock changes to the given level
// if already at level, waits for it to cycle to level
void waitForPXClockChangeTo(uint8_t level);

void waitForHSYNCClock(uint8_t level);

#endif
