#ifndef UART_H
#define UART_H

#include <inttypes.h>

#define BUFFER_SIZE 256
static volatile uint8_t msg_buffer[BUFFER_SIZE]; // Note: uint8_t will overflow allowing a circular buffer (1111 1111 + 1 = 0). So combine 8 bit type with 256 size array for automatic circular buffer
static volatile uint16_t current_length_UART;
static volatile uint8_t next_idx_to_send_UART;
static uint8_t next_idx_to_store_UART;
static volatile uint8_t byte_sent;

void uart_init();

// send one byte
// NOTE: Does nothing if the buffer is full
void uart_send_byte(byte);

// NOTE: Only sends as many bytes as there is free space in the buffer
// must pass in size of array as cannot use sizeof for pointer passed in
void uart_send_bytes(uint8_t *bytes, uint8_t number_of_bytes);

// send an 8 bit value to appear as hex when printing with ASCII
// input: single byte
void uart_send_hex8(uint8_t byte);

#endif
