#include <msp430.h>
#include "I2C.h"

void i2c_init()
{
    next_idx_to_send = 0;
    next_idx_to_store = 0;
    current_length = 0;
    receive_idx = 0;

    // setting P1.2 and P1.3 for I2C
    P1SEL0 |= (BIT2 | BIT3); // primary module function

    P1REN |= BIT2 | BIT3; // Enable internal resistors
    P1OUT |= BIT2 | BIT3; // Switch resistors to pull-up

    // configure I2C
    // see 24.3.1 for initialization and reset
    // see 24.4 for I2c registers
    // eUSCI_B0 so control register UCB"0"CTLW0
    // 16 bit "word" register
    UCB0CTLW0 |= UCSWRST; // control register set I2C to reset condition (must be in reset for configuration)

    // set USCI_B as I2C master mode
    UCB0CTLW0 |= (UCMODE_3 | UCMST | UCSYNC); // UCMODE_3 sets eUSCI_B to I2C mode, UCMST sets master mode, and UCSYNC sets synchronous mode

    // set clock source of USCI_B
    UCB0CTLW0 |= UCSSEL__SMCLK; // control register set clock source to SMCLK (which is same as MCLK at ~1MHz)
    // configuring baud rate registers for 100kHz when sourcing from SMCLK where SMCLK = 1,048,576 Hz
    // acts as divisor for BRCLK
    UCB0BR0 |= 10;
    UCB0BR1 |= 0;

    UCB0IE |= UCNACKIE + UCSTPIE + UCSTTIE; // UNVERIFIED - bunch of interrupt enables
    UCB0CTLW0 &= ~UCSWRST; // bring I2C out of reset
}

void i2c_write(uint8_t slave_byte, uint8_t address_byte, uint8_t data_byte)
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
    UCB0CTLW0 &= ~UCSLA10; // 7 bit
    // set desired slave address (transmits on START)
    UCB0I2CSA = slave_byte;
    // set I2C for transmitter mode (the r/w bit in slave address byte)
    UCB0CTLW0 |= UCTR;
    // generate start condition in master mode
    UCB0CTLW0 |= UCTXSTT; // THIS SETS TXIFG0 when START generated AND TXBUF is clear

    UCB0IE |= UCTXIE0; // enable transmit interrupt
}

void i2c_receive(uint8_t slave_byte, uint8_t address_byte, int length)
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
    UCB0CTLW0 &= ~UCSLA10; // 7 bit
    // set desired slave address (transmits on START)
    UCB0I2CSA = slave_byte;
    // set I2C for transmitter mode (the r/w bit in slave address byte)
    UCB0CTLW0 |= UCTR;
    // generate start condition in master mode
    UCB0CTLW0 |= UCTXSTT; // THIS SETS TXIFG0 when START generated AND TXBUF is clear

    UCB0IE |= UCTXIE0;

    while (!received); // poll for data received before exiting function

    __delay_cycles(2200); // I2C receive requires delay for some reason
}


// ISR for USCI_B0
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    switch (UCB0IV)
    {
    case USCI_I2C_UCNACKIFG: // IF NACK IS RECEIVED STOP TRANSMISSION
        UCB0CTLW0 |= UCTXSTP;

        UCB0IE &= ~UCTXIE0 & ~UCRXIE0;
        UCB0IFG |= UCTXIFG0 & ~UCRXIFG0;
        break;

    case USCI_I2C_UCTXIFG0: // TRANSMIT FIRST
        if (current_length > 0)
        {
            UCB0TXBUF = address_buffer[next_idx_to_send++];
            current_length--;
        }
        else
        { // transmission end
            if (rwStatus)
            { // if writing, generate stop once data is sent
                UCB0CTLW0 |= UCTXSTP;

                UCB0IE &= ~UCTXIE0;
                UCB0IFG |= UCTXIFG0;
            }
            else
            {
                // if writing, generate repeated start once address is sent
                // ENABLE REPEATED START FOR RECEIVING
                // set I2C for receiver mode
                UCB0CTLW0 &= ~UCTR;
                // generate restart condition in receiver mode
                UCB0CTLW0 |= UCTXSTT;
                UCB0IE |= UCRXIE0;

                UCB0IE &= ~UCTXIE0;
                UCB0IFG |= UCTXIFG0;
            }
        }
        break;

    case USCI_I2C_UCRXIFG0: // RECEIVE SECOND
        // receive the data in global variable
        received_bytes[receive_idx] = UCB0RXBUF;
        receive_idx++;

        // send stop bit if last bit to be read
        if (receive_idx == receiveLength)
        {
            received = 1;
            receive_idx = 0; // reset idx

            // generate stop condition to stop I2C transmission
            UCB0CTLW0 |= UCTXSTP;

            UCB0IE &= ~UCRXIE0;
            UCB0IFG |= UCRXIFG0;
        }
        break;

    case USCI_I2C_UCSTTIFG:
        break; // start flag used in slave mode only. SEE (24.3.11.4 I2C STATE CHANGE INTERRUPT OPERATION)
    case USCI_I2C_UCSTPIFG:
        break; // stop flag (not too sure what it is supposed to do)
    default:
        break;
    }
}


void accelInit()
{
    const uint8_t accelAddress = 0x28; // BNO055
    const uint8_t oprMode = 0x3D; // operating mode register to write to
    const uint8_t mode = 0b00001000; // IMU operating mode

    i2c_write(accelAddress, oprMode, mode);
    __delay_cycles(400); // OPR_MODE register takes 7ms to change mode
}

void getAccel(uint8_t *data_array)
{
    // define addresses of accelerometer
    const uint8_t accelAddress = 0x28; // BNO055
    const uint8_t xAddress = 0x28;
    const uint8_t yAddress = 0x2A;
    const uint8_t zAddress = 0x2C;

    // store acceleration values (2 bytes in each direction)
    i2c_receive(accelAddress, xAddress, 2);
    data_array[0] = received_bytes[0];
    data_array[1] = received_bytes[1];

    i2c_receive(accelAddress, yAddress, 2);
    data_array[2] = received_bytes[0];
	data_array[3] = received_bytes[1];

    i2c_receive(accelAddress, zAddress, 2);
    data_array[4] = received_bytes[0];
 	data_array[5] = received_bytes[1];
}

void barInit()
{
    const uint8_t barAddress = 0x76; // BMP280

    const uint8_t meas = 0xF4;
    const uint8_t measSettings = 0b00000111; // osrs_p & power mode

    i2c_write(barAddress, meas, measSettings);
    __delay_cycles(400);
}

void getBar(uint8_t *data_array)
{
    // define addresses of barometer
    const uint8_t barAddress = 0x76; // BMP280
    const uint8_t pressReg = 0xF7;

    // store pressure data (2 bytes)
    i2c_receive(barAddress, pressReg, 2);
    data_array[0] = received_bytes[0];
    data_array[1] = received_bytes[1];

}

void getBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t *storeByte, int numBytes)
{
	i2c_receive(slaveAddress, registerAddress, numBytes);

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
