#ifndef SPI_H
#define SPI_H

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

// initialising SPI once
// Using P2.5/2.6/3.1/2.4 for SPI with launchpad (2.5 is SOMI, 2.6 is SIMO, 3.1 is CS, 2.4 is CLK)
// eUSCI_A1
void spi_init();


// -------- private functions -------- //

// write data to an address on SPI
// input: slave address, target register address, data (each in 1 byte)
void spi_write(uint8_t slave_byte, uint8_t address_byte, uint8_t data_byte);

// receive byte/s from address on SPI
// input: slave address, target register address of first byte to be received, number of bytes to be received from data
// note: register address is incremented for subsequent bytes to be received
// output: received bytes stored in "received_bytes" array
void spi_receive(uint8_t slave_byte, uint8_t address_byte, int length);

// -------- private functions -------- //


// initialise barometer
void barInit();

// capture pressure data
// input: array to store 2 pressure bytes
void getBar(uint8_t *data_array);

// get any number of bytes from a register with SPI
// note: register address is incremented for subsequent bytes to be received
// input: 8 byte slave address, 8 byte register address to read, target pointer to store data, number of bytes to read
void getBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t *storeByte, int numBytes) ;

// blips led for testing
void blip();

#endif
