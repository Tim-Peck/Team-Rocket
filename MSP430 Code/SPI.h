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

// initialise SPI for SD card
// Using P2.5/2.6/3.1/2.4 for SPI with launchpad (2.5 is SOMI, 2.6 is SIMO, 3.1 is CS, 2.4 is CLK)
// eUSCI_A1
void spi_init();

// -------- private functions -------- //

// transfer one byte from microcontroller
// input: target register address, data (each in 1 byte)
// output: previous byte in receive register
uint8_t spi_transfer(uint8_t byte);

// send a command to SD card
// format: 6 bytes
// MSB: start bit (0), transmission bit (1), command index
// bytes 2-5: argument
// LSB: CRC, end bit (1)
void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc);

// receive a response byte from address on SPI
// output: response byte if card responds (less than 8 bytes), otherwise 0xFF
uint8_t SD_readResponse1();

// -------- private functions -------- //

// required power up sequence for SD card includes 1ms delay and sending 74 clock cycles
void SD_powerUpSeq();

// switching to SPI mode and going to idle state
// output: response R1 from CMD0
uint8_t SD_goIdleState();

// initialise SD card
void SDInit();

// get any number of bytes from a register with SPI
// note: register address is incremented for subsequent bytes to be received
// input: 8 byte slave address, 8 byte register address to read, target pointer to store data, number of bytes to read
void getBytesSPI(uint8_t registerAddress, uint8_t *storeByte, int numBytes);

void blipSPI();

#endif
