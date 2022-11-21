#include <msp430.h>
#include "SPI.h"

void spi_init()
{
    next_idx_to_send = 0;
    next_idx_to_store = 0;
    current_length = 0;
    receive_idx = 0;

    // setting pins for SPI
    P2SEL0 |= (BIT4 | BIT5 | BIT6); // primary module function
    P3SEL0 |= BIT1;

//    P1REN |= BIT2 | BIT3; // Enable internal resistors
//    P1OUT |= BIT2 | BIT3; // Switch resistors to pull-up

    // configure SPI
    // see 23.4 for SPI registers
    // eUSCI_A1 so control register UCA"1"CTLW0
    // 16 bit "word" register
    UCA1CTLW0 |= UCSWRST; // control register set SPI to reset condition (must be in reset for configuration)

    // set USCI_A as SPI master mode
    UCA1CTLW0 |= (UCMODE_2 | UCMST | UCSYNC | UCSTEM); // UCMODE_2 sets eUSCI_A to SPI mode with active low CS, UCMST sets master mode, and UCSYNC sets synchronous mode, UCSTEM sets STE pin to be CS

    // set clock source of USCI_A
    UCA1CTLW0 |= UCSSEL__SMCLK; // control register UCSSELx field, set clock source to SMCLK (which is same as MCLK at ~1MHz)
    // configuring baud rate registers for 100kHz when sourcing from SMCLK where SMCLK = 1,048,576 Hz
    // acts as divisor for BRCLK
    UCA1BRW |= 10;

    UCA1CTLW0 &= ~UCSWRST; // bring SPI out of reset
}

void spi_write(uint8_t slave_byte, uint8_t address_byte, uint8_t data_byte)
{
    rwStatus = 1; // set bool for ISR to write

    // store address byte in buffer
    if (current_length < BUFFER_SIZE)
    {
        address_buffer[next_idx_to_store++] = address_byte; // next_idx is always incremented and not reset
        current_length++;
    }

    // store data byte in buffer
    if (current_length < BUFFER_SIZE)
    {
        // store byte in buffer
        address_buffer[next_idx_to_store++] = data_byte; // next_idx is always incremented and not reset
        current_length++;
    }

    // generate start by doing the following (See 24.3.5.2.1)

    // set slave address bit size (7 or 10)
    UCA1CTLW0 &= ~UCSLA10; // 7 bit
    // set desired slave address (transmits on START)
    UCA1SPISA = slave_byte;
    // set SPI for transmitter mode (the r/w bit in slave address byte)
    UCA1CTLW0 |= UCTR;
    // generate start condition in master mode
    UCA1CTLW0 |= UCTXSTT; // THIS SETS TXIFG0 when START generated AND TXBUF is clear

    UCA1IE |= UCTXIE0; // enable transmit interrupt
}

void spi_receive(uint8_t slave_byte, uint8_t address_byte, int length)
{
    rwStatus = 0; // set bool for ISR to read
	receiveLength = length;
    received = 0; // reset received bool

    if (current_length < BUFFER_SIZE)
    {
        // store byte in buffer
        address_buffer[next_idx_to_store++] = address_byte; // next_idx is always incremented and not reset
        current_length++; // so current length alternates between 0 and 1 for each byte
    }

    // generate start by doing the following (see 24.3.5.2.1)

    // set slave address bit size (7 or 10)
    UCA1CTLW0 &= ~UCSLA10; // 7 bit
    // set desired slave address (transmits on START)
    UCA1SPISA = slave_byte;
    // set SPI for transmitter mode (the r/w bit in slave address byte)
    UCA1CTLW0 |= UCTR;
    // generate start condition in master mode
    UCA1CTLW0 |= UCTXSTT; // THIS SETS TXIFG0 when START generated AND TXBUF is clear

    UCA1IE |= UCTXIE0;

    while (!received); // poll for data received before exiting function

    __delay_cycles(2200); // SPI receive requires delay for some reason
}


// ISR for USCI_A1 for SPI
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (UCA1IV)
    {
    case USCI_SPI_UCTXIFG: // TRANSMIT FIRST
        if (current_length > 0)
        {
            UCA1TXBUF = address_buffer[next_idx_to_send++];
            current_length--;
        }
        else
        { // transmission end
            if (rwStatus)
            { // if writing, generate stop once data is sent
                UCA1CTLW0 |= UCTXSTP;

                UCA1IE &= ~UCTXIE0;
                UCA1IFG |= UCTXIFG0;
            }
            else
            {
                // if writing, generate repeated start once address is sent
                // ENABLE REPEATED START FOR RECEIVING
                // set SPI for receiver mode
                UCA1CTLW0 &= ~UCTR;
                // generate restart condition in receiver mode
                UCA1CTLW0 |= UCTXSTT;
                UCA1IE |= UCRXIE0;

                UCA1IE &= ~UCTXIE0;
                UCA1IFG |= UCTXIFG0;
            }
        }
        break;

    case USCI_SPI_UCRXIFG: // RECEIVE SECOND
        // receive the data in global variable
        received_bytes[receive_idx] = UCA1RXBUF;
        receive_idx++;

        // send stop bit if last bit to be read
        if (receive_idx == receiveLength)
        {
            received = 1;
            receive_idx = 0; // reset idx

            // generate stop condition to stop SPI transmission
            UCA1CTLW0 |= UCTXSTP;

            UCA1IE &= ~UCRXIE0;
            UCA1IFG |= UCRXIFG0;
        }
        break;
    default:
        break;
    }
}


void barInit()
{
    const uint8_t barAddress = 0x76; // BMP280

    const uint8_t meas = 0xF4;
    const uint8_t measSettings = 0b00000111; // osrs_p & power mode

    spi_write(barAddress, meas, measSettings);
    __delay_cycles(400);
}

void getBar(uint8_t *data_array)
{
    // define addresses of barometer
    const uint8_t barAddress = 0x76; // BMP280
    const uint8_t pressReg = 0xF7;

    // store pressure data (2 bytes)
    spi_receive(barAddress, pressReg, 2);
    data_array[0] = received_bytes[0];
    data_array[1] = received_bytes[1];

}

void getBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t *storeByte, int numBytes)
{
	spi_receive(slaveAddress, registerAddress, numBytes);

	// store each byte read
	int i;
	for (i = 0; i < numBytes; i++) {
	    storeByte[i] = received_bytes[i];
	}
}

void blip()
{
    // toggle on and off for a second
    P1OUT &= ~BIT0;
    int i;
    for (i = 0; i < 5; i++)
    {
        P1OUT ^= BIT0;
        __delay_cycles(100000);
        P1OUT ^= BIT0;
        __delay_cycles(100000);
    }
    __delay_cycles(150000);
}
