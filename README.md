# Team-Rocket

Branch to work on developing the camera functions
Currently worked on by Oliver


# Camera Module
Currently needs the SPI functions for saving the image buffer to the sd card

## Main Camera Functions:

### initCamera()
Initialises the camera pins, sets setting registers over I2C and return status.

returns 0 for successful start
returns ~0 if error

<!-- Unsure about buffers, what extent? -->
### takeImage(bufferPointer, bufferLength)
sends a image trigger over I2C, waits for response, stores image pixel by pixel to the image buffer


### saveImage(folderName, bufferPointer, bufferLength) Work In Progress
Writes buffer to a new file on the SD card in a given folder


## Camera Helper Functions:

### requestImage()
Sends the I2C trigger command to the module

### waitForPXClockCycle(bool)
polls until the given level on the pixel clock


## Dependancies
Uses these undefined global functions

### i2C_write(uint8_t slave, uint8_t address, uint8_t data)
Sends 1 byte to slave at address.

### i2C_recieve(uint8_t slave, uint8_t address)
Returns 1 byte from slave at address
