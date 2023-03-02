#include <msp430.h>
#include "I2C.h"
#include "Defines.h"

void i2c_init()
{
    next_idx_to_send = 0;
    next_idx_to_store = 0;
    current_length = 0;
    receive_idx = 0;

    // setting P4.6 and P4.7 for I2C (2355)
    P4SEL0 |= (BIT6 | BIT7); // primary module function


#ifdef USE_DEV_BOARD
    // I2C line is pulled high (LAUNCHPAD ONLY)
    P4REN |= BIT6 | BIT7; // Enable internal resistors
    P4OUT |= BIT6 | BIT7; // Switch resistors to pull-up
#endif

    // configure I2C
    // see 24.3.1 for initialization and reset
    // see 24.4 for I2C registers
    // eUSCI_B1 so control register UCB"1"CTLW0
    // 16 bit "word" register
    UCB1CTLW0 |= UCSWRST; // control register set I2C to reset condition (must be in reset for configuration)

    // set USCI_B as I2C master mode
    UCB1CTLW0 |= (UCMODE_3 | UCMST | UCSYNC); // UCMODE_3 sets eUSCI_B to I2C mode, UCMST sets master mode, and UCSYNC sets synchronous mode

    // set clock source of USCI_B
    UCB1CTLW0 |= UCSSEL__SMCLK; // control register set clock source to SMCLK (which is same as MCLK at ~1MHz)
    // configuring baud rate registers for 100kHz when sourcing from SMCLK where SMCLK = 1,048,576 Hz
    // acts as divisor for BRCLK
    UCB1BR0 |= 10;
    UCB1BR1 |= 0;

    UCB1IE |= UCNACKIE + UCSTPIE + UCSTTIE; // UNVERIFIED - bunch of interrupt enables
    UCB1CTLW0 &= ~UCSWRST; // bring I2C out of reset
}

void i2c_write(uint8_t slave_byte, uint8_t address_byte, uint8_t data_byte)
{
    rwStatus = 1; // set bool for ISR to write

    // store address byte in buffer
    if (current_length < BUFFER_SIZE)
    {
        write_buffer[next_idx_to_store++] = address_byte; // next_idx is always incremented and not reset
        current_length++;
    }

    // store data byte in buffer
    if (current_length < BUFFER_SIZE)
    {
        // store byte in buffer
        write_buffer[next_idx_to_store++] = data_byte; // next_idx is always incremented and not reset
        current_length++;
    }

    // generate start by doing the following (See 24.3.5.2.1)

    // set slave address bit size (7 or 10)
    UCB1CTLW0 &= ~UCSLA10; // 7 bit
    // set desired slave address (transmits on START)
    UCB1I2CSA = slave_byte;
    // set I2C for transmitter mode (the r/w bit in slave address byte)
    UCB1CTLW0 |= UCTR;
    // generate start condition in master mode
    UCB1CTLW0 |= UCTXSTT; // THIS SETS TXIFG0 when START generated AND TXBUF is clear

    UCB1IE |= UCTXIE0; // enable transmit interrupt
}

uint8_t i2c_receive(uint8_t slave_byte, uint8_t address_byte, int length)
{
    rwStatus = 0; // set bool for ISR to read
	receiveLength = length;
    received = 0; // reset received bool

    if (current_length < BUFFER_SIZE)
    {
        // store byte in buffer
        write_buffer[next_idx_to_store++] = address_byte; // next_idx is always incremented and not reset
        current_length++; // so current length alternates between 0 and 1 for each byte
    }

    // generate start by doing the following (see 24.3.5.2.1)

    // set slave address bit size (7 or 10)
    UCB1CTLW0 &= ~UCSLA10; // 7 bit
    // set desired slave address (transmits on START)
    UCB1I2CSA = slave_byte;
    // set I2C for transmitter mode (the r/w bit in slave address byte)
    UCB1CTLW0 |= UCTR;
    // generate start condition in master mode
    UCB1CTLW0 |= UCTXSTT; // THIS SETS TXIFG0 when START generated AND TXBUF is clear

    UCB1IE |= UCTXIE0;

    // poll for data received before exiting function
    while (!received) {
        // check if NACK
        if (UCNACKIFG & UCB1IFG) {
            return 0;
        }
    }

    __delay_cycles(2200); // I2C receive requires delay for some reason
    return 1;
}


// ISR for USCI_B1
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
    switch (UCB1IV)
    {
    case USCI_I2C_UCNACKIFG: // IF NACK IS RECEIVED STOP TRANSMISSION
        UCB1CTLW0 |= UCTXSTP;

        UCB1IE &= ~UCTXIE0 & ~UCRXIE0;
        UCB1IFG |= UCTXIFG0 & ~UCRXIFG0;
        break;

    case USCI_I2C_UCTXIFG0: // TRANSMIT FIRST
        if (current_length > 0)
        {
            UCB1TXBUF = write_buffer[next_idx_to_send++];
            current_length--;
        }
        else
        { // transmission end
            if (rwStatus)
            { // if writing, generate stop once data is sent
                UCB1CTLW0 |= UCTXSTP;

                UCB1IE &= ~UCTXIE0;
                UCB1IFG |= UCTXIFG0;
            }
            else
            {
                // if reading, generate repeated start once address is sent
                // ENABLE REPEATED START FOR RECEIVING
                // set I2C for receiver mode
                UCB1CTLW0 &= ~UCTR;
                // generate restart condition in receiver mode
                UCB1CTLW0 |= UCTXSTT;
                UCB1IE |= UCRXIE0;

                UCB1IE &= ~UCTXIE0;
                UCB1IFG |= UCTXIFG0; // note: flag is enabled manually so receive vector is entered
            }
        }
        break;

    case USCI_I2C_UCRXIFG0: // RECEIVE SECOND
        // receive the data in global variable
        received_bytes[receive_idx] = UCB1RXBUF;
        receive_idx++;

        // send stop bit if last bit to be read
        if (receive_idx == receiveLength)
        {
            received = 1;
            receive_idx = 0; // reset idx

            // generate stop condition to stop I2C transmission
            UCB1CTLW0 |= UCTXSTP;

            UCB1IE &= ~UCRXIE0;
            UCB1IFG |= UCRXIFG0;
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

uint8_t checkIMUConnection() {
    const uint8_t chipIDRegister = 0; // BNO055

    if (!i2c_receive(IMUAddress, chipIDRegister, 1)) {
        return 0; // if NACK
    }

    // BNO055 chip ID is 0xA0 as seen in register table
    if (received_bytes[0] == 0xA0){
        return 1;
    } else {
        return 0;
    }
}

uint8_t readAccelCalib()
{
    const uint8_t calibStatRegister = 0x35; // BNO055

    // read IMU calibration register
    if (!i2c_receive(IMUAddress, calibStatRegister, 1))
    {
        return 0; // if NACK
    }

    // retrieve accelerometer calibration value
    uint8_t accelCalibLevel = (received_bytes[0] >> 2) & 0b00000011;

    return accelCalibLevel;
}

void IMUInit()
{
    const uint8_t oprMode = 0x3D; // operating mode register to write to
    const uint8_t mode = 0b00001000; // IMU operating mode

    i2c_write(IMUAddress, oprMode, mode);
    __delay_cycles(400); // OPR_MODE register takes 7ms to change mode
}

void getAccel(uint8_t *data_array)
{
    // define addresses of accelerometer
    const uint8_t xAddress = 0x28;
    const uint8_t yAddress = 0x2A;
    const uint8_t zAddress = 0x2C;

    // store acceleration values (2 bytes in each direction)
    i2c_receive(IMUAddress, xAddress, 2);
    data_array[0] = received_bytes[0];
    data_array[1] = received_bytes[1];

    i2c_receive(IMUAddress, yAddress, 2);
    data_array[2] = received_bytes[0];
    data_array[3] = received_bytes[1];

    i2c_receive(IMUAddress, zAddress, 2);
    data_array[4] = received_bytes[0];
    data_array[5] = received_bytes[1];
}

void parseAccelBytes(uint8_t *data_array, float *accelerations)
{
  uint16_t val16b = data_array[1] << 8 | data_array[0] ;
  accelerations[0] = val16b/100.0;

  val16b = data_array[3] << 8 | data_array[2] ;
  accelerations[1] = val16b/100.0;

  val16b = data_array[5] << 8 | data_array[4] ;
  accelerations[2] = val16b/100.0;
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
