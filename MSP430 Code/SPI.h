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
// Using P1.1/1.2/1.3/2.1 for SPI with launchpad (1.1 is CLK, 1.2 is MOSI, 1.3 is MISO, 2.1 is CS)
// eUSCI_B0
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

/* TO REMOVE
// receive response R2
// input: 17 byte uin8_t array to store response R2
void SD_readRes2(uint8_t *res);
*/

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

void SD_printReadTokenError(uint8_t token);

// print contents of read block
void print_SDBlock(uint8_t R1, uint8_t *buf, uint8_t *token);

void SD_printReadTokenError(uint8_t token);

// -------- startup functions -------- //

// required power up sequence for SD card includes 1ms delay and sending 74 clock cycles
void SD_powerUpSeq();

// switching to SPI mode by holding CS low and going to idle state
// CMD0
// output: response R1
uint8_t SD_goIdleState();

// send interface condition to check generation of card
// CMD8
// input: 5 byte uint8_t array to store response R7
void SD_sendInterfaceCond(uint8_t *res);

// read operating conditions register
// CMD58
// input: 5 byte uint8_t array to store response R3
void SD_readOCR(uint8_t *res);

// send CMD55 application command (required before any ACMD)
// output: R1
uint8_t SD_sendAppCommand();

// send operating conditions register to activate card initialization
// ACMD41
// output: R1
uint8_t SD_sendOCR();

/* TO REMOVE
// Sends the command to return the CSD register
// input: 17 byte array for the Response
void SD_sendCSD_Command(uint8_t *res);
*/

// -------- complete initialization function -------- //

// complete initialization function for SD card
// output: 0 for initialization failure, 1 for success
uint8_t SD_init();

// -------- block read/write functions -------- //
// NOTE: R1 is returned and token is written to for user to actually check if read/write was successful in SD card (e.g. address out of range)
// although, checking for R1 here is kind of useless

// read a single data block from a specified sector address
// each block consists of 512 bytes (unless standard capacity which can be set)
// input: 32 bit block address, 512 byte uint8_t data array to read to, 1 byte variable to store start token
// output: SD card status
uint8_t SD_readSingleBlock(uint32_t addr, uint8_t *buf, uint8_t *token);

// write a single data block to a specific sector address
// each block consists of 512 bytes (unless standard capacity which can be set)
// input: 32 bit block address, 512 byte uint8_t data array to write from, 1 byte variable to store data-response token
// output: SD card status
uint8_t SD_writeSingleBlock(uint32_t addr, uint8_t *writeBuf, uint8_t *token);

// -------- user operation functions -------- //

// check if flight is finished to stop flight mode
// used if power is disconnected and reconnected after flight is finished to stop flight mode process from restarting and prevent overriding data
uint8_t checkFinishStatus();

#endif
