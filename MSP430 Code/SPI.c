#include <msp430.h>
#include "SPI.h"

void spi_init()
{
    next_idx_to_sendSPI = 0;
    next_idx_to_storeSPI = 0;
    current_lengthSPI = 0;
    receive_idxSPI = 0;

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
    UCA1CTLW0 |= (UCMODE_2 | UCMST | UCSYNC | UCSTEM | UCMSB); // UCMODE_2 sets eUSCI_A to SPI mode with active low CS, UCMST sets master mode, and UCSYNC sets synchronous mode, UCSTEM sets STE pin to be CS

    // set clock source of USCI_A
    UCA1CTLW0 |= UCSSEL__SMCLK; // control register UCSSELx field, set clock source to SMCLK (which is same as MCLK at ~1MHz)
    // configuring baud rate registers for 100kHz when sourcing from SMCLK where SMCLK = 1,048,576 Hz
    // acts as divisor for BRCLK
    UCA1BRW |= 10;

    UCA1CTLW0 &= ~UCSWRST; // bring SPI out of reset (Testing note: This brings SIMO and CS low for some reason)
}

void spi_write(uint8_t address_byte, uint8_t data_byte)
{
    rwStatus = 0; // set bool for ISR to write

    // add write bit to MSB (RW = '0')
    address_byte &= ~0b10000000;

    // store address byte in buffer
    address_bufferSPI[next_idx_to_storeSPI++] = address_byte; // next_idx is always incremented and not reset
    current_lengthSPI++;

    // store data byte in buffer
    address_bufferSPI[next_idx_to_storeSPI++] = data_byte;
    current_lengthSPI++;

    UCA1IE |= UCTXIE; // enable transmit interrupt which generates interrupt request (23.3.8.1)
}

void spi_receive(uint8_t address_byte, int length)
{
    rwStatus = 1; // set bool for ISR to read
    receiveLengthSPI = length;
    receivedStatus = 0; // reset received bool

    // add read bit to MSB (RW = '1')
    address_byte |= 0b10000000;

    // store byte in buffer
    address_bufferSPI[next_idx_to_storeSPI++] = address_byte; // next_idx is always incremented and not reset
    current_lengthSPI++; // so current length alternates between 0 and 1 for each byte

    UCA1IE |= UCTXIE;

    while (!receivedStatus)
    {
//        blipSPI();
    } // poll for data received before exiting function

//    __delay_cycles(2200); // SPI receive requires delay for some reason
}

// ISR for USCI_A1 for SPI
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (UCA1IV)
    {
    case USCI_SPI_UCTXIFG: // transmit flag
//        while (UCA1STATW & UCBUSY); // poll until byte sent

        if (current_lengthSPI > 0)
        {
            UCA1TXBUF = address_bufferSPI[next_idx_to_sendSPI++]; // possibly requires 16 bits before actually shift register sends byte?? (8 for TX->shift, 8 for shift-> sensor)

            current_lengthSPI--;
        }
        else if (rwStatus && (receive_idxSPI != receiveLengthSPI + 5)) // send if not at the end
        {
            while (!(UCA1IFG & UCRXIFG)); // poll until byte sent

            UCA1TXBUF = 0xFF;
            received_bytesSPI[receive_idxSPI] = UCA1RXBUF;
            receive_idxSPI++;
        }
        else
        { // transmission end
            UCA1IE &= ~UCTXIE;
            UCA1IFG |= UCTXIFG;

            receivedStatus = 1;
            receive_idxSPI = 0; // reset idx
        }
        break;

    case USCI_SPI_UCRXIFG: // receive flag
        // receive the data in global variable
//        received_bytesSPI[receive_idxSPI] = UCA1RXBUF; // reading RXBUF resets RXIFG!
//        receive_idxSPI++;

        break;
    default:
        break;
    }
}

void barInitSPI()
{
    const uint8_t meas = 0xF4;
    const uint8_t measSettings = 0b00000111; // osrs_p & power mode

    spi_write(meas, measSettings);
    __delay_cycles(400);
}

void getBarSPI(uint8_t *data_array)
{
    // register address of barometer
    const uint8_t pressReg = 0xF7;

    // store pressure data (2 bytes)
    spi_receive(pressReg, 2);
    data_array[0] = received_bytesSPI[0];
    data_array[1] = received_bytesSPI[1];
}

void getBytesSPI(uint8_t registerAddress, uint8_t *storeByte, int numBytes)
{
    spi_receive(registerAddress, numBytes);

    // store each byte read
    int i;
    for (i = 0; i < numBytes + 5; i++)
    {
        storeByte[i] = received_bytesSPI[i];
    }

}

void blipSPI()
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
