#include <msp430.h>

#include "Camera.h"

void initCamera(){
  // Set upcam_rowLengthSYNC, PIXCLOCK and DATA0 Pins
  // Clear pintype to 0 = GPIO read
  // Clear pin direction to 0 = input

  // TODO: Check this is correct
  // TODO: Figure which pin corresponds to which camera pin
  // Pin 4 (Assuming VSYNC)
  P4DIR = 0; // Set direction to input
  P4REN = 0; // disable resistors
  P4SEL0 = 0; // ensure GPIO use
  P4SEL1 = 0; // ensure GPIO use
  // Pin 5 (Assuming HSYNC)
  P5DIR = 0;
  P5REN = 0;
  P5SEL0 = 0;
  P5SEL1 = 0;
  // Pin 6 (Assuming PXCLK)
  P6DIR = 0;
  P6REN = 0;
  P6SEL0 = 0;
  P6SEL1 = 0;
  // Pin 7 (Assuming D0)
  P7DIR = 0;
  P7REN = 0;
  P7SEL0 = 0;
  P7SEL1 = 0;

  // check enabled by reading the sensor part number
  // read 0x0000[7:0] should be 0x01 in MODEL_ID_H
  // read 0x0001[7:0] should be 0xB0 in MODEL_ID_L
  i2C_recieve(cam_I2CAddress, 0x0000, 2);
  // if (received_bytes[0] != 0x01){
  //   return;
  // } else if (received_bytes[1] != 0xB0) {
  //   return;
  // }

  // change setting registers \
  // set the data mode to serial
  // default 0x02
  // 0x3059[5] = 1  serial_en
  i2C_write(cam_I2CAddress, 0x3059, 0x22);

}

void takeImage(){
  // initalise pixel position counters
  int rowNum = 0;
  int pixNum = 0;

  // Send request over I2C
  requestImage();

  // Poll until the VSYNC pin goes high
  int timeOut = 0;
  while (~(P4IN)) {
    timeOut ++;
    if (cam_waitTimeout < timeOut){
      return;
    }
  }

  // while frame is being transfered (VSYNC high)
  while (P4IN) {
  // while transfering a line of pixels (HSYNC high)
    pixNum = 0;
    while (P5IN) {
      // gather a single pixel, 8 cycles
      volatile uint8_t byte = 0;
      int i = 0;
      for (i = 0; i < cam_pixBits; i++) {
        // wait for the centre of the clock cycle
        waitForPXClockChangeTo(0);
        // Read data0 pin and
        // OR the value with the correct position in byte
        byte |=  (P7IN) << i;
      }
      imageBuffer[rowNum*cam_rowLength + pixNum] = byte;
      pixNum ++;
    }
    rowNum ++;
  }
  imageCount ++;
}

void saveImage(){
  // gather an image line into a binary string???

  // write that binary string into the SD card
  // CAM_IMAGEFOLDER
}

void requestImage(){
  // Address = 0x24
  // Trigger Message
    // set PMU_PROGRAMMAB LE_FRAMECNT to 1
    // 0x3020[7:0] = 1
    i2C_write(cam_I2CAddress, 0x3020, 0x01);
    // Set to I2C trigger in mode select register
    // 0x0100[2:0] = 011
    i2C_write(cam_I2CAddress, 0x0100, 0x03);
}

void waitForPXClockChangeTo(int level){
  // Check that we are not already at level
  volatile int timeOut = 0;
  if (P6IN == level) {
    // If we are, wait until the pin is different to level
    while (P6IN & cam_pixBits == level){
      // Wait until timeout cycles
      timeOut ++;
      if (500000 < timeOut){
        return;
      }
    }
  }
  timeOut = 0;
  // Wait until the pin is the same as level
  while (P6IN != level){
    timeOut ++;
    if (500000 < timeOut){
      return;
    }
  }
  return;
}
