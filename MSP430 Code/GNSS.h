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

void uart_GNSS_init();

// send one byte
// NOTE: Does nothing if the buffer is full
void GNSS_send_byte(byte);

// NOTE: Only sends as many bytes as there is free space in the buffer
// must pass in size of array as cannot use sizeof for pointer passed in
void GNSS_send_bytes(uint8_t *bytes, uint8_t number_of_bytes);

// send an NMEA command to the GNSS module
// input: 3 digit ID
void GNSS_cmd(int ID);

// begin storing GNSS data to NMEA_sentence array(currently GGA sentence)
void GNSS_receive();

// check if fix acquired by GNSS module from GGA sentence
// input: ASCII array for GGA sentence
// output: 1 for fix acquired, 0 for fix not acquired
uint8_t checkFix();

// store altitude from GGA sentence
// input: ASCII array for GGA sentence, float to store altitude
void parse_GGA_alt(float *altitude);

// store UTC from GGA sentence
// input: ASCII array for GGA sentence, 3 byte array to store UTC values (hh, mm, ss)
void parse_GGA_UTC(uint8_t *UTC);

// store latitude and longitude from GGA sentence
// input: ASCII array for GGA sentence, 2 float array to store geographic coordinates in decimal format
void parse_GGA_GCS(float *GCS);

#endif
