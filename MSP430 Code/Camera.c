#include <msp430.h>

#include "Camera.h"

// Initalises the camera data pins and sets the camera settings using I2C
void camera_init(){
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
  // uint8_t checkByte[2];
  // getBytes(cam_I2CAddress, 0x0000, checkByte, 2);
  // if (checkByte[0] != 0x01){
  //   return;
  // } else if (checkByte[1] != 0xB0) {
  //   return;
  // }

  // change setting registers
  // set the data mode to serial
  // default 0x02
  // 0x3059[5] = 1  serial_en
  i2C_write(cam_I2CAddress, 0x3059, 0x22);

}

// Requests, records and saves an image
void takeImage(){
  // reset buffer
  uint8_t i;
  for (size_t i = 0; i < 512; i++) {
    imageLineBuffer[i] = 0;
  }

  // initalise pixel position counters
  uint16_t lineNum = 0;
  uint16_t rowPix = 0;

  // Send request over I2C
  requestImage();

  // Poll until the VSYNC pin goes high
  uint32_t timeOut = 0;
  while (~(P4IN)) {
    timeOut ++;
    if (cam_waitTimeout < timeOut){
      return;
    }
  }

  // while frame is being transfered (VSYNC high)
  while (P4IN) {
    // while transfering a line of pixels (HSYNC high)
    rowPix = 0;
    waitForHSYNCClock(1);
    while (P5IN) {
      // gather a single pixel, 8 cycles
      volatile uint8_t byte = 0x00;
      for (i = 0; i < 8; i++) {
        // wait for the centre of the clock cycle
        waitForPXClock(0);
        // Read data0 pin and
        // OR the value with the correct position in byte
        byte |=  (P7IN) << i;
        // wait till next clock cycle
        waitForPXClock(1);
      }
      imageLineBuffer[rowPix] = byte;
      rowPix ++;
    }
    saveImageLine(lineNum);
    lineNum ++;
  }
  imageCounter ++;
}

// Saves the image line from the buffer
void saveImageLine(uint16_t lineNum){
    uint8_t token;
    // Calc address, skip first 24 blocks, 324 blocks per image, one block per row
    uint32_t addr = (uint32_t)imageCounter*324 + lineNum + 24;
    SD_writeSingleBlock(addr, imageLineBuffer, &token);
}

// Requests and image over I2C
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

// Helper function for waiting for the PXCLK pin to change
void waitForPXClock(uint8_t level){
  timeOut = 0;
  // Wait until the pin is the same as level
  while (P6IN != level){
    timeOut ++;
    if (timeOut > 500000){
      return;
    }
  }
  return;
}

// Helper function for waiting for the HSYNC pin to change
void waitForHSYNCClock(uint8_t level){
  timeOut = 0;
  // Wait until the pin is the same as level
  while (P5IN != level){
    timeOut ++;
    if (timeOut > 500000){
      return;
    }
  }
  return;
}
