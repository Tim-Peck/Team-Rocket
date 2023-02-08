#include <msp430.h>
#include <inttypes.h>
#include "GNSS.h"
#include "UART.h"

void uart_GNSS_init() {
    next_idx_to_send_GNSS = 0;
    next_idx_to_store_GNSS = 0;
    current_length_GNSS = 0;

    NMEA_sentence[0] = 0;
    receive_idx2 = 0;
    check_idx = 0;
	
	// Configure pins
    P2SEL0 |= BIT5 | BIT6; // P2.5=RXD, P2.6=TXD (is default high)

    // Configure UART
    UCA1CTLW0 |= UCSWRST;       // Put UART state machine in reset
    UCA1CTLW0 |= UCSSEL__SMCLK; // Choose SMCLK as clock source

    // Baud Rate calculation - we want 9600 baud (below are fields of registers)
    // N = clock / baud = 1,048,576 Hz / 9600 = 109.227
    // N > 16 so UCOS16 = 1
    // UCA1BR0 = int(N / 16) = int(109.227 / 16) = int(6.827) = 6
    // UCBRSx  = 0x22 as per users guide table "UCBRSx settings"
    // UCBRFx  = int([(N / 16) - int(N / 16)] * 16) = int(13.227) = 8
    UCA1BRW = 6;
    UCA1MCTLW = (0x22 << 8) | UCBRF_13 | UCOS16;


    UCA1CTLW0 &= ~UCSWRST;      // Initialize GNSS state machine
}

void GNSS_send_byte(byte) {
    byte_sent_GNSS = 0;

    if (current_length_GNSS < BUFFER_SIZE) {
        cmd_buffer[next_idx_to_store_GNSS++] = byte; // next_idx is always incremented and not reset
        current_length_GNSS++; // so current length alternates between 0 and 1 for each byte
    }

    // enable transmit interrupt to start transmitting
    // if we are already transmitting this does nothing
    UCA1IE |= UCTXIE;

    // poll until byte is sent
    while (!byte_sent_GNSS);
}

void GNSS_send_bytes(uint8_t *bytes, uint8_t number_of_bytes) {
    uint8_t i;
    for (i = 0; i < number_of_bytes; i++) {
        GNSS_send_byte(bytes[i]);
    }
}

void GNSS_cmd(int ID) {
    uint8_t cmd[8] = {'$','P','M','T','K'};

    // convert each digit of ID to ASCII characters
    uint8_t i;
    for (i = 0; i < 3; i++) {
        cmd[5+i] = (uint8_t)((ID % 10) + 48); // note: must convert int to ASCII number
        ID = ID / 10;
    }

    // send command
    GNSS_send_bytes(cmd, 8);
}

void GNSS_receive() {
    // enable receive interrupt to begin receiving ASCII characters
    UCA1IE |= UCRXIE;
}

uint8_t checkFix(){
    uint8_t i = 0;
    uint8_t commaCount = 0;

    // check if NMEA sentence received first
    if (!NMEA_sentence[0]) {
        return 0;
    }

    // walk the sentence till in fix field
    while (commaCount != 6) {
        if (NMEA_sentence[i++] == ',') {
            commaCount++; // increment commaCount when comma delimiter encountered
        }
    }

    // check if fix acquired
    if (NMEA_sentence[i] == '1') {
        return 1;
    } else {
        return 0;
    }
}

void parse_GGA_alt(float *altitude){
    // check if fix acquired first
    if (checkFix()) {

    }
}

void parse_GGA_UTC(uint8_t *UTC){
    // check if fix acquired first
    if (checkFix()) {

        }
}

void parse_GGA_GCS(float *GCS){
    // check if fix acquired first
    if (checkFix()) {

        }
}

// ISR TO USCI_A1
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
//    __bis_SR_register(GIE); // enable interrupt nesting

    switch(UCA1IV) {
        case USCI_NONE: break;

        case USCI_UART_UCRXIFG:
            received_byte = UCA1RXBUF;

            // check receive sequence for GGA
            if (received_byte == NMEA_ID[check_idx]) {
                check_idx++;
            } else if (check_idx == 3) {
                // GGA sequence found
                // store new NMEA sentence
                if (received_byte != '\r') {
                    NMEA_sentence[receive_idx2++] = received_byte;
                } else {
                    // reset indexes when whole NMEA sentence read
                   check_idx = 0;
                   receive_idx2 = 0;
                }
            } else {
                check_idx = 0;
            }
            break;

        case USCI_UART_UCTXIFG:
            if (current_length_GNSS > 0) {
                UCA1TXBUF = cmd_buffer[next_idx_to_send_GNSS++];
                current_length_GNSS--;
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
                UCA1IE &= ~UCTXIE;
                UCA1IFG |= UCTXIFG;

                byte_sent_GNSS = 1;
            }
            break;

        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
