#ifndef CAM_H
#define CAM_H

#include <inttypes.h>
#include "Camera.c"

// #define CAM_ROWLENGTH 320
// #define CAM_COLLENGTH 320
// #define CAM_PIXBITS 8

// #define CAM_I2CADDRESS 0x24
// #define CAM_WAITTIMEOUT 1000000
// #define CAM_IMAGEFOLDER "imageFolder"

const uint8_t cam_I2CAddress = 0x24;
const int cam_waitTimeout = 1000000;
const char imageFolder[] = "imageFolder";

const int cam_rowLength = 320;
const int cam_colLength = 320;
const int cam_pixBits = 8;

static volatile uint8_t imageBuffer[cam_rowLength*cam_colLength];
static volatile int imageCount = 0;

// initalises the camera pins and changes required registers via I2C.
void initCamera();

// Sends a request to the camera, then waits for and
// records the image response to the buffer.
// Returns success if the image was successfully recorded
void takeImage();

// Save the image to the SD card
void saveImage();

// Sends an I2C message to the camera to take a photo
void requestImage();

// Polls until the PXClock changes to the given level
// if already at level, waits for it to cycle to level
void waitForPXClockChangeTo(int level);

#endif
