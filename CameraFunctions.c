
#define CAM_ROWLENGTH 320
#define CAM_COLLENGTH 320
#define CAM_PIXBITS 8
// TODO: Check 0x24 syntax
#define CAM_I2CADDRESS 0x24
#define CAM_WAITTIMEOUT 1000000
#define CAM_IMAGEFOLDER "imageFolder"

// Pin number in register
#define CAM_VSYNC 0x01
#define CAM_HSYNC 0x02
#define CAM_PIXCLOCK 0x03
#define CAM_DATA0 0x04

// ERROR DEFINES
#define CAM_SUCCESS 0
#define CAM_ERROR_INVALIDI2CRESPONSE 1
#define CAM_ERROR_NORESPONSE 2

uint8 imageBuffer[CAM_ROWLENGTH*CAM_COLLENGTH];
int imageCount = 0;

// i2C_write(slave address data);
// i2C_recieve(slave address);

// initalises the camera pins and changes required registers via I2C.
int initCamera(){
  // Set up the HSYNC, HSYNC, PIXCLOCK and DATA0 Pins
  // Clear pintype to 0 = GPIO read
  // Clear pin direction to 0 = input
  P1SEL &= ~CAM_VSYNC;
  P1SEL2 &= ~CAM_VSYNC;
  P1DIR &= ~CAM_VSYNC;

  P1SEL &= ~CAM_HSYNC;
  P1SEL2 &= ~CAM_HSYNC;
  P1DIR &= ~CAM_HSYNC;

  P1SEL &= ~CAM_PIXCLOCK;
  P1SEL2 &= ~CAM_PIXCLOCK;
  P1DIR &= ~CAM_PIXCLOCK;

  P1SEL &= ~CAM_DATA0;
  P1SEL2 &= ~CAM_DATA0;
  P1DIR &= ~CAM_DATA0;

  // check enabled by reading the sensor part number
  // read 0x0000[7:0] should be 0x01 in MODEL_ID_H
  // read 0x0001[7:0] should be 0xB0 in MODEL_ID_L
  // if failed, return CAM_ERROR_INVALIDI2CRESPONSE
  if (i2C_recieve(CAM_I2CADDRESS, 0x0000) != 0x01){
    return CAM_ERROR_INVALIDI2CRESPONSE;
  } else if (i2C_recieve(CAM_I2CADDRESS, 0x0001) != 0xB0) {
    return CAM_ERROR_INVALIDI2CRESPONSE;
  }

  // change setting registers \
  // set the data mode to serial
  // default 0x02
  // 0x3059[6] = 0  4bit_en
  // 0x3059[5] = 1  serial_en
  i2C_write(CAM_I2CADDRESS, 0x3059, 0x22);
  // default 0x0A
  // 0x3060[5] = 0  gated_en
  // 0x3060[4] = 1  msb_en
  i2C_write(CAM_I2CADDRESS, 0x3059, 0x1A);

  return CAM_SUCCESS;
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
  while (~(P1IN & CAM_VSYNC)) {
    timeOut ++;
    if (CAM_WAITTIMEOUT < timeOut){
      return CAM_ERROR_NORESPONSE;
    }
  }

  // while frame is being transfered (VSYNC high)
  while (P1IN & CAM_VSYNC) {
  // while transfering a line of pixels (HSYNC high)
    pixNum = 0;
    while (P1IN & CAM_HSYNC) {
      // gather a single pixel, 8 cycles
      uint8 byte = 0;
      for (size_t i = 0; i < CAM_PIXBITS; i++) {
        // wait for the centre of the clock cycle
        if (waitForPXClockChangeTo(0) != CAM_SUCCESS){
          return CAM_ERROR_NORESPONSE;
        }
        // Read data0 pin and
        // OR the value with the correct position in byte
        byte |=  (uint8)(P1IN & CAM_DATA0) << i;
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
    i2C_write(CAM_I2CADDRESS, 0x3020, 0x01);
    // Set to I2C trigger in mode select register
    // 0x0100[2:0] = 011
    i2C_write(CAM_I2CADDRESS, 0x0100, 0x03);
}

// Polls until the PXClock changes to the given level
// if already at level, waits for it to cycle to level
int waitForPXClockChangeTo(bool level){
  // Check that we are not already at level
  int timeOut = 0;
  if ((P1IN & CAM_PXCLOCK) == level) {
    // If we are, wait until the pin is different to level
    while ((P1IN & CAM_PXCLOCK) == level){
      // Wait until timeout cycles
      timeOut ++;
      if (CAM_WAITTIMEOUT < timeOut){
        return CAM_ERROR_NORESPONSE;
      }
    }
  }
  timeOut = 0;
  // Wait until the pin is the same as level
  while ((P1IN & CAM_PXCLOCK) != level){
    timeOut ++;
    if (CAM_WAITTIMEOUT < timeOut){
      return CAM_ERROR_NORESPONSE;
    }
  }
  return CAM_SUCCESS;
}
