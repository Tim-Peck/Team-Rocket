#ifndef I2C_H
#define I2C_H

#define CAM_ROWLENGTH 320
#define CAM_COLLENGTH 320
#define CAM_PIXBITS 8

#define CAM_I2CADDRESS 0x24
#define CAM_WAITTIMEOUT 1000000
#define CAM_IMAGEFOLDER "imagefolder"

// Pin number in register
#define CAM_VSYNC 0x01
#define CAM_HSYNC 0x02
#define CAM_PIXCLOCK 0x03
#define CAM_DATA0 0x04

// ERROR DEFINES
#define CAM_SUCCESS 0
#define CAM_ERROR_INVALIDI2CRESPONSE 1
#define CAM_ERROR_NORESPONSE 2

static volatile uint8 imageBuffer[CAM_ROWLENGTH*CAM_COLLENGTH];
static volatile int imageCount = 0;

// initalises the camera pins and changes required registers via I2C.
int cameraInit();

// Sends a request to the camera, then waits for and
// records the image response to the buffer.
// Returns success if the image was successfully recorded
int takeImage();

// Save the image to the SD card
void saveImage();

// Sends an I2C message to the camera to take a photo
void requestImage();

// Polls until the PXClock changes to the given level
// if already at level, waits for it to cycle to level
int waitForPXClockChangeTo(bool level);

#endif
