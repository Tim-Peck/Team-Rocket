#ifndef GNSS_H
#define GNSS_H



#include <inttypes.h>

#define BUFFER_SIZE 256
static volatile uint8_t cmd_buffer[BUFFER_SIZE]; // Note: uint8_t will overflow allowing a circular buffer (1111 1111 + 1 = 0). So combine 8 bit type with 256 size array for automatic circular buffer
static volatile uint16_t current_length_GNSS;
static volatile uint8_t next_idx_to_send_GNSS;
static uint8_t next_idx_to_store_GNSS;
static volatile uint8_t byte_sent_GNSS;

static volatile uint8_t received_byte; // bool for received state
static volatile uint8_t receive_idx2; // index to increment
static volatile uint8_t NMEA_sentence[BUFFER_SIZE];

const static uint8_t NMEA_ID[] = "GGA"; // NMEA ID to store
static volatile uint8_t check_idx;

void uart_GNSS_init(); // (1.6 is RX, 1.7 is TX)

// send one byte
// NOTE: Does nothing if the buffer is full
void GNSS_send_byte(uint8_t byte);

// NOTE: Only sends as many bytes as there is free space in the buffer
// must pass in size of array as cannot use sizeof for pointer passed in
void GNSS_send_bytes(uint8_t *bytes, uint8_t number_of_bytes);

// send an NMEA command to the GNSS module
// input: 3 digit ID
void GNSS_cmd(int cmd_ID);

// begin storing GNSS data to NMEA_sentence array(currently GGA sentence)
void GNSS_receive();

// check if fix acquired by GNSS module from latest GGA sentence
// input: ASCII array for GGA sentence
// output: 1 for fix acquired, 0 for fix not acquired
uint8_t fixAcquired();

// store altitude from latest GGA sentence
// input: ASCII array for GGA sentence
// output: float of altitude
float parse_GGA_alt();

// store UTC from latest GGA sentence
// input: ASCII array for GGA sentence, 3 byte array to store UTC values (hh, mm, ss)
void parse_GGA_UTC(uint8_t *UTC);

// store latitude and longitude from latest GGA sentence
// input: ASCII array for GGA sentence, 2 float array to store geographic coordinates in decimal degrees
// note: GCS array will store in index 0: signed latitude with positive NORTH, and in index 1: longitude EAST
void parse_GGA_GCS(float *GCS);

// convert a sequence of ASCII characters representing a decimal number (with noninteger part) to a float
// input: pointer to first digit of ASCII sequence
// output: float conversion of ASCII sequence
float ASCII_to_float(uint8_t *firstDigitPtr);

// copy float's raw value to array of 4 individual bytes
// input: float value to convert, 4 byte array to store with endianness dependent on system
void float_to_uint8(float floatVal, uint8_t *rawFloatPtr);

// convert a two digit ASCII number to uint8_t
// input: ASCII value of tens digit, ASCII value of ones digit
// output: binary value of digit in uint8_t
uint8_t ASCII_to_uint8(uint8_t MSD, uint8_t LSD);

#endif
