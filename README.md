# Team-Rocket

Branch to work on developing the camera functions
Currently worked on by Oliver


# Camera Module
TODO: Make all of the main functions and helper functions


## Main Camera Functions:

### initCamera()
Initialises the camera, sets setting registers over I2C, return status.

returns 0 for successful start
returns 1 if error

<!-- Unsure about buffers, what extent? -->
### takeImage(bufferPointer, bufferLength)
sends a image trigger over I2C, waits for response, stores image pixel by pixel to a buffer

### saveImage(folderName, bufferPointer, bufferLength)
writes buffer to a new file on the SD card in a given folder


## Camera Helper Functions:

### requestImage()
sends the I2C trigger command to the module

### waitForPXClockCycle(boolean)
polls until the given level on the pixel clock


## External Communication Functions:

<!-- Unsure how this will work, need OBC advisement -->
### writeToSDFile(folderName, fileName, bufferPointer, bufferLength)
writes the given buffer to the given file in the given folder
??? Might need something to do with creating or overwriting existing files
??? Not sure how the buffer thing might work, might just include this process in the save image function
