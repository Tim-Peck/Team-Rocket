#include <msp430.h>
#include <inttypes.h>
#include "UART.h"

//#define METADATA // uncomment to read metadata and first data block

void uart_init() {
    next_idx_to_send_UART = 0;
    next_idx_to_store_UART = 0;
    current_length_UART = 0;
	
	// Configure pins
    P1SEL0 |= BIT5 | BIT4; // P1.5=RXD, P1.4=TXD

    // Configure UART
    UCA0CTLW0 |= UCSWRST;       // Put UART state machine in reset
    UCA0CTLW0 |= UCSSEL__SMCLK; // Choose SMCLK as clock source

    // Baud Rate calculation - we want 9600 baud
    // N = clock / baud = 1,048,576 Hz / 9600 = 109.227
    // N > 16 so UCOS16 = 1
    // UCA0BR0 = int(N / 16) = int(109.227 / 16) = int(6.827) = 6
    // UCBRSx  = 0x22 as per users guide table "UCBRSx settings"
    // UCBRFx  = int([(N / 16) - int(N / 16)] * 16) = int(13.227) = 8

    UCA0BRW = 6;
    UCA0MCTLW = (0x22 << 8) | UCBRF_13 | UCOS16;


    UCA0CTLW0 &= ~UCSWRST;      // Initialize UART state machine
}

void uart_send_byte(byte) {
    byte_sent = 0;

    if (current_length_UART < BUFFER_SIZE) {
        msg_buffer[next_idx_to_store_UART++] = byte; // next_idx is always incremented and not reset
        current_length_UART++; // so current length alternates between 0 and 1 for each byte
    }

    // enable transmit interrupt to start transmitting
    // if we are already transmitting this does nothing
    UCA0IE |= UCTXIE;

    // poll until byte is sent
    while (!byte_sent);
}

void uart_send_bytes(uint8_t *bytes, uint8_t number_of_bytes) {
#ifdef METADATA
    uint8_t i;
    for (i = 0; i < number_of_bytes; i++) {
        uart_send_byte(bytes[i]);
    }
#endif
}

void uart_send_hex8(uint8_t byte)
{
    // print most significant hex digit
    uint8_t MSD = ((byte >> 4) & 0b00001111);
    if (MSD > 0x09) { // if hex digit is a letter
        uart_send_byte(MSD + 0x37);
    } else { // if hex digit is a number
        uart_send_byte(MSD + 0x30);
    }

    // print least significant hex digit
    uint8_t LSD = (byte & 0b00001111);
    if (LSD > 0x09) { // if hex digit is a letter
            uart_send_byte(LSD + 0x37);
        } else { // if hex digit is a number
            uart_send_byte(LSD + 0x30);
        }
}


// ISR TO USCI_A0
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
    switch(UCA0IV) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG: break;

        case USCI_UART_UCTXIFG:
            if (current_length_UART > 0) {
                UCA0TXBUF = msg_buffer[next_idx_to_send_UART++];
                current_length_UART--;
            }
            else {
                // Entering the interrupt service routine (ISR), will clear
                // interrupt flags. i.e. we have just cleared UCTXIFG
                // UCTXIFG is usually set when UCA0TXBUF is emptied. However we
                // didn't write anything to the buffer this time (when ISR reentered), so it's empty
                // and cannot be "emptied". i.e. UCTXIFG will not get set again
                // We can however set UCTXIFG manually to re-enter the ISR!
                // Doing this outside of the ISR is dangerous because we may be
                // in the middle of a UART transmission. Instead, we can set it
                // here and disable TX interrupts. When interrupts are enabled
                // again, UCTXIFG will still be set so we will immediately come
                // back here.
                UCA0IE &= ~UCTXIE;
                UCA0IFG |= UCTXIFG;

                byte_sent = 1;
            }
            break;

        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
