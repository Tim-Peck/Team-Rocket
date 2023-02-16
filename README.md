# Team-Rocket

Thought I would put all the useful functions together

SD hardware ???

Barometer hardware ???

IMU hardware BNO055 chip???  
(dev with Adafruit BNO055)  
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

GNSS hardware is Adafruit Ultimate GPS featherwing  
https://learn.adafruit.com/adafruit-ultimate-gps-featherwing

Camera hardware is HMO1BO  
https://www.sparkfun.com/products/15570


## Key Functions:

### UART
`uart_init()`  
`void uart_send_bytes(uint8_t *bytes, uint8_t number_of_bytes)`

### I2C
`void i2c_init()`  
`void i2c_write(uint8_t slave_byte, uint8_t address_byte, uint8_t data_byte)`  
`uint8_t i2c_receive(uint8_t slave_byte, uint8_t address_byte, int length)`


### SPI
`void spi_init()`  
`uint8_t SD_init()`  
`uint8_t SD_readSingleBlock(uint32_t addr, uint8_t *buf, uint8_t *token)`  
`uint8_t SD_writeSingleBlock(uint32_t addr, uint8_t *writeBuf, uint8_t *token)`


### Accelerometer
`uint8_t checkIMUConnection()`  
`void accelInit()`  
`void getAccel(uint8_t *data_array)`  
`void parseAccelBytes(uint8_t *data_array, float *accelerations)`

### Barometer
`void barInit()`  
`void getBar(uint8_t *data_array)`

### GNSS
`void uart_GNSS_init()`  
`uint8_t fixAcquired()`  
`float parse_GGA_alt()`  
`void parse_GGA_UTC(uint8_t *UTC)`  
`void parse_GGA_GCS(float *GCS)`   
`void float_to_uint8(float floatVal, uint8_t *rawFloatPtr)`  

### Camera
`void camera_init()`  
`void takeImage()`

### Timers
`void timerB0_init();`   
`void begin1HzTimer();`  
`void timerB1_init();`  
`void rgbLED(uint8_t redVal, uint8_t greenVal, uint8_t blueVal);`
