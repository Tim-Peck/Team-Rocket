#ifndef SPI_H
#define SPI_H

#include <inttypes.h>

#define BUFFER_SIZE 256

static volatile uint8_t address_bufferSPI[BUFFER_SIZE];
static volatile int current_lengthSPI;
static volatile uint8_t next_idx_to_sendSPI;
static uint8_t next_idx_to_storeSPI;

static volatile int receivedStatus; // bool for received state
static volatile int receive_idxSPI; // index to increment
static volatile int receiveLengthSPI; // length of message to be received (default 1)
static volatile uint8_t received_bytesSPI[BUFFER_SIZE];

static volatile int rwStatus; // bool value for ISR: 0 means read, 1 means write

// initialising SPI once
// Using P2.5/2.6/3.1/2.4 for SPI with launchpad (2.5 is SOMI, 2.6 is SIMO, 3.1 is CS, 2.4 is CLK)
// eUSCI_A1
void spi_init();


// -------- private functions -------- //

// write data to an address on SPI
// input: target register address, data (each in 1 byte)
void spi_write(uint8_t address_byte, uint8_t data_byte);

// receive byte/s from address on SPI
// input: target register address of first byte to be received, number of bytes to be received from data
// note: register address is incremented for subsequent bytes to be received
// output: received bytes stored in "received_bytesSPI" array
void spi_receive(uint8_t address_byte, int length);

// -------- private functions -------- //


// initialise SD card
void SDInit();

// get any number of bytes from a register with SPI
// note: register address is incremented for subsequent bytes to be received
// input: 8 byte slave address, 8 byte register address to read, target pointer to store data, number of bytes to read
void getBytesSPI(uint8_t registerAddress, uint8_t *storeByte, int numBytes) ;

void blipSPI();

#endif
