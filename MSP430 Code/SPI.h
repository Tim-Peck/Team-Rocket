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

// transfer one byte from microcontroller and receive previous byte
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
uint8_t SD_readRes1();

// receive response R7
// input: 5 byte uin8_t array to store response R7
void SD_readRes7(uint8_t *res);

// -------- print functions -------- //

// print R1 after any CMD to serial monitor in ASCII
// prints error conditions and idle state
void SD_printR1(uint8_t res);

// print R7 after CMD8 to serial monitor in ASCII
// checks compatability with card; whether supplied voltage is appropriate and SD version
void SD_printR7(uint8_t *res);

// print R3 after CMD58 to serial monitor in ASCII
// checks whether card is powered up, whether it's high capacity and its operating voltages
void SD_printR3(uint8_t *res);

// print read token error
void SD_printTokenError(uint8_t token);

// print contents of read block
void print_SDBlock(uint8_t R1, uint8_t *buf, uint8_t *token);

// -------- startup functions -------- //

// required power up sequence for SD card includes 1ms delay and sending 74 clock cycles
void SD_powerUpSeq();

// switching to SPI mode by holding CS low and going to idle state
// output: response R1 from CMD0
uint8_t SD_goIdleState();

// send interface condition to check generation of card
// input: 5 byte uint8_t array to store response R7
void SD_sendInterfaceCond(uint8_t *res);

// read operating conditions register
// input: 5 byte uint8_t array to store response R3
void SD_readOCR(uint8_t *res);

// send CMD55 application command (required before any ACMD)
// output: R1
uint8_t SD_sendAppCommand();

// send operating conditions register to activate card initialization
// output: R1
uint8_t SD_sendOCR();

// -------- complete initialization function -------- //

// complete initialization function for SD card
// output: 0 for initialization failure, 1 for success
uint8_t SD_init();

// -------- read/write functions -------- //

// read a single data block from a specified address
// each block consists of 512 bytes (unless standard capacity which can be set)
// input: 32 bit block address, 512 byte uint8_t data array to read, 1 byte variable to store start token
// output: SD card status
uint8_t SD_readSingleBlock(uint32_t addr, uint8_t *buf, uint8_t *token);

#endif
