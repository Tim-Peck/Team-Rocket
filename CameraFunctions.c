
int initCamera(){
  // check enabled
    // if not enabled return 1

  // change setting registers \
  // set the data mode to serial
  // set the trigger to I2C

  return 0
}

void takeImage(uint8 *bufferPointer, int bufferLength){
  // initalise pixNum couter
  int pixNum = 0;

  // Send request over I2C
  requestImage();

  // Poll until the VSYNC pin goes high
  // while VSYNC pin is high

    // while HSYNC pin is high
      // wait for the centre of the clock cycle
      // waitForPXClockCycle()


      // if buffer length is not exceeded
      if (pixNum < bufferLength){
        // set d0 pin in buffer
        // bufferPointer[pixNum] = data0Value;
        // increment pixNum counter
        pixNum ++;
      }

    // add a newpixrow value to the buffer, None?
}

void saveImage(folderName, bufferPointer, bufferLength){
  // gather an image line into a binary string

  // write that binary string into the SD card
}

void requestImage(){
  // Address = ...
  // Trigger Message = ...
  // use sendI2CMessage()
}

void waitForPXClockCycle(){
  // while the PXpin != 1
    // do nothing
  // while the PXpin != 0
    // do nothing
}
