#ifndef I2C_H
#define I2C_H

#include <inttypes.h>

#define BUFFER_SIZE 256

static volatile uint8_t address_buffer[BUFFER_SIZE]; 
static volatile uint16_t current_length;
static volatile uint8_t next_idx_to_send;
static uint8_t next_idx_to_store;

static volatile int received; // bool for received state
static volatile int receive_idx; // index to increment
static volatile int receiveLength; // length of message to be received (default 1)
static volatile uint8_t received_bytes[BUFFER_SIZE];

static volatile int rwStatus; // bool value for ISR: 0 means read, 1 means write

// initialising I2C once
// Using P1.2/1.3 for I2C with launchpad (1.3 is SCL, 1.2 is SDA)
// eUSCI_B0
void i2c_init();

// write data to an address on I2C 
// input: slave address, target register address, data (each in 1 byte)
void i2c_write(uint8_t slave_byte, uint8_t address_byte, uint8_t data_byte);

// receive byte/s from address on I2C
// input: slave address, target register address of first byte to be received, number of bytes to be received from data
// note: register address is incremented for subsequent bytes to be received
// output: received bytes stored in "received_bytes" array
void i2c_receive(uint8_t slave_byte, uint8_t address_byte, int length);


// initialise accelerometer to switch to IMU operating mode
void accelInit();

// capture accelerometer values in all 3 directions
// input: array to store 6 acceleration bytes
void getAccel(uint8_t *data_array);

// initialise barometer
void barInit();

// capture pressure data
// input: array to store 2 pressure bytes
void getBar(uint8_t *data_array);

// get 1 byte from a register with I2C
// input: 8 byte slave address, 8 byte register address to read, target pointer to store data
void getByte(uint8_t slaveAddress, uint8_t registerAddress, uint8_t *storeByte) ;

// blips led for testing
void blip();

#endif