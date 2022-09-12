
int initCamera(){
  // check enabled by reading the sensor part number
    // read 0x0000[7:0] should be 0x01 in MODEL_ID_H
    // read 0x0001[7:0] should be 0xB0 in MODEL_ID_L
    // if failed, return 1

  // change setting registers \
  // set the data mode to serial
    // 0x3059[6] = 0  4bit_en
    // 0x3059[5] = 1  serial_en
    // 0x3060[5] = 0  gated_en
    // 0x3060[4] = 1  msb_en

  return 0
}

// TODO: check how long it takes to save a single bit vs save line vs save image
void takeImage(uint8 *bufferPointer, int bufferLength){
  // initalise pixNum couter
  int pixNum = 0;

  // Send request over I2C
  requestImage();

  // Poll until the VSYNC pin goes high
  while (!VSYNC) {}

  // while frame is being transfered (VSYNC high)
  while (VSYNC) {
  // while transfering a line of pix (HSYNC high)
    while (HSYNC) {
      // gather a single pixel, 8 cycles
      int byte = 0; // TODO: figure Datatype
      for (size_t i = 0; i < 8; i++) {
        // wait for the centre of the clock cycle
        // waitForPXClockCycle(0);
        // add serial bit to byte
        // byte | something
      }
      // save byte to SD, or add byte to line buffer
    }
    // write new line, or add line to frame buffer
  }
  // write frame if need be
}

void saveImage(folderName, bufferPointer, bufferLength){
  // gather an image line into a binary string

  // write that binary string into the SD card
}

void requestImage(){
  // Address = 0x24
  // Trigger Message
    // set PMU_PROGRAMMAB LE_FRAMECNT to 1
    // 0x3020[7:0] = 1
    // Set to I2C trigger in mode select register
    // 0x0100[2:0] = 011
}

void waitForPXClockCycle(bool level){
  // while the PXpin == level
    // do nothing
}
