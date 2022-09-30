
#define CAM_ROWLENGTH 320
#define CAM_COLLENGTH 320
#define CAM_PIXBITS 8
// TODO: Check 0x24 syntax
#define CAM_I2CADDRESS 0x24
#define CAM_WAITTIMEOUT 1000000
#define CAM_IMAGEFOLDER "imageFolder"

// TODO: Not sure if these are defined like this?
#define CAM_VSYNC 0
#define CAM_HSYNC 0
#define CAM_PIXCLOCK 0
#define CAM_DATA0 0

// ERROR DEFINES
#define CAM_SUCCESS 0
#define CAM_ERROR_INVALIDI2CRESPONSE 1
#define CAM_ERROR_BUFFERTOOSMALL 2
#define CAM_ERROR_NORESPONSE 3

uint8 imageBuffer[CAM_ROWLENGTH*CAM_COLLENGTH];
int imageCount = 0;

// initalises the camera pins and changes required registers via I2C.
int initCamera(){
  // Set up the HSYNC, HSYNC, PIXCLOCK and DATA0 Pins
  // TODO: Figure pin setup procedure

  // USING I2C:
  // check enabled by reading the sensor part number
    // read 0x0000[7:0] should be 0x01 in MODEL_ID_H
    // read 0x0001[7:0] should be 0xB0 in MODEL_ID_L
    // if failed, return CAM_ERROR_INVALIDI2CRESPONSE

  // change setting registers \
  // set the data mode to serial
    // 0x3059[6] = 0  4bit_en
    // 0x3059[5] = 1  serial_en
    // 0x3060[5] = 0  gated_en
    // 0x3060[4] = 1  msb_en

  return 0
}

// Sends a request to the camera, then waits for and
// records the image response to the buffer.
// Returns success if the image was successfully recorded
int takeImage(){
  // initalise pixel position counters
  int rowNum = 0;
  int pixNum = 0;

  // Send request over I2C
  requestImage();

  // Poll until the VSYNC pin goes high
  int timeOut = 0;
  while (!CAM_VSYNC) {
    timeOut ++;
    if (CAM_WAITTIMEOUT < timeOut){
      return CAM_ERROR_NORESPONSE;
    }
  }

  // while frame is being transfered (VSYNC high)
  while (CAM_VSYNC) {
  // while transfering a line of pixels (HSYNC high)
    pixNum = 0;
    while (CAM_HSYNC) {
      // gather a single pixel, 8 cycles
      uint8 byte = 0;
      for (size_t i = 0; i < CAM_PIXBITS; i++) {
        // wait for the centre of the clock cycle
        if (waitForPXClockChangeTo(0) != CAM_SUCCESS){
          return CAM_ERROR_NORESPONSE;
        }
        // Read data0 pin and
        // or the value with the correct position in byte
        // TODO: Figure BIT[i]
        byte |=  (uint8)CAM_DATA0 << i;
      }
      imageBuffer[rowNum*CAM_ROWLENGTH + pixNum] = byte;
      pixNum ++;
    }
    rowNum ++;
  }
  imageCount ++;
  return CAM_SUCCESS;
}

// Save the image to the
void saveImage(){
  // gather an image line into a binary string???

  // write that binary string into the SD card
  // CAM_IMAGEFOLDER
}

// Sends an I2C message to the camera to take a photo
void requestImage(){
  // Address = 0x24
  // Trigger Message
    // set PMU_PROGRAMMAB LE_FRAMECNT to 1
    // 0x3020[7:0] = 1
    // Set to I2C trigger in mode select register
    // 0x0100[2:0] = 011
}

// Polls until the PXClock changes to the given level
// if already at level, waits for it to cycle to level
int waitForPXClockChangeTo(bool level){
  // Check that we are not already at level
  int timeOut = 0;
  if (CAM_PXCLOCK == level) {
    // If we are, wait until the pin is different to level
    while (CAM_PXCLOCK == level){
      // Wait until timeout cycles
      timeOut ++;
      if (CAM_WAITTIMEOUT < timeOut){
        return CAM_ERROR_NORESPONSE;
      }
    }
  }
  timeOut = 0;
  // Wait until the pin is the same as level
  while (CAM_PXCLOCK != level){
    timeOut ++;
    if (CAM_WAITTIMEOUT < timeOut){
      return CAM_ERROR_NORESPONSE;
    }
  }
  return CAM_SUCCESS;
}
