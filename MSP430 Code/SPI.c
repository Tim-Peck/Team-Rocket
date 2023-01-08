#include <msp430.h>
#include "SPI.h"

// R1 values
#define PARAM_ERROR(X)      X & 0b01000000
#define ADDR_ERROR(X)       X & 0b00100000
#define ERASE_SEQ_ERROR(X)  X & 0b00010000
#define CRC_ERROR(X)        X & 0b00001000
#define ILLEGAL_CMD(X)      X & 0b00000100
#define ERASE_RESET(X)      X & 0b00000010
#define IN_IDLE(X)          X & 0b00000001

// R7 values
#define CMD_VER(X)          ((X >> 4) & 0xF0) // input: byte 31:24 (command verison for bits 31:28)
#define VOL_ACC(X)          (X & 0x1F) // input: byte 15:8 (voltage accept for bits 11:8)

#define VOLTAGE_ACC_27_33   0b00000001
#define VOLTAGE_ACC_LOW     0b00000010
#define VOLTAGE_ACC_RES1    0b00000100
#define VOLTAGE_ACC_RES2    0b00001000

// chip select macros
#define CS_ENABLE() P3OUT &= ~BIT1;
#define CS_DISABLE() P3OUT |= BIT1;

void spi_init()
{
    next_idx_to_sendSPI = 0;
    next_idx_to_storeSPI = 0;
    current_lengthSPI = 0;
    receive_idxSPI = 0;

    // setting pins for SPI
    P2SEL0 |= (BIT4 | BIT5 | BIT6); // primary module function
    P3DIR |= BIT1; // Use GPIO for CS

    // configure SPI
    // see 23.4 for SPI registers
    // eUSCI_A1 so control register UCA"1"CTLW0
    // 16 bit "word" register
    UCA1CTLW0 |= UCSWRST; // control register set SPI to reset condition (must be in reset for configuration)

    // set USCI_A as SPI master mode
    UCA1CTLW0 |= (UCCKPH | UCMODE_2 | UCMST | UCSYNC | UCSTEM | UCMSB);
    // for SD card
    // UCCKPL = 0, clock polarity idle low
    // UCCKPH = 1, data is captured on first edge before changing
    // UCMODE_2 sets eUSCI_A to SPI mode with active low CS
    // UCMST sets master mode
    // UCSYNC sets synchronous mode (SPI mode)
    // UCSTEM sets STE pin to be CS,
    // UCMSB sets Most Significant Bit first

    // set clock source of USCI_A
    UCA1CTLW0 |= UCSSEL__SMCLK; // control register UCSSELx field, set clock source to SMCLK (which is same as MCLK at ~1MHz)
    // configuring baud rate registers for 100kHz when sourcing from SMCLK where SMCLK = 1048576 Hz
    // acts as divisor for BRCLK
    UCA1BRW |= 10;

    UCA1CTLW0 &= ~UCSWRST; // bring SPI out of reset (Testing note: This brings SIMO and CS low for some reason)
}

uint8_t spi_transfer(uint8_t byte)
{
    // load data into register
    UCA1TXBUF = byte;

    // poll until byte received
    while (!(UCA1IFG & UCRXIFG))
        ;

    // return PREVIOUS byte received in receive register
    return UCA1RXBUF;
}

void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    // see command format

    // transmit command to SD card
    spi_transfer(cmd | 0x40); // bit 6 high (transmission bit) but bit 7 always low from cmd argument

    // transmit argument
    spi_transfer((uint8_t) (arg >> 24)); // shift down one byte at a time
    spi_transfer((uint8_t) (arg >> 16));
    spi_transfer((uint8_t) (arg >> 8));
    spi_transfer((uint8_t) (arg));

    // transmit crc
    spi_transfer(crc | 0x01);
}

uint8_t SD_readRes1()
{
    uint8_t count = 0, res;

    // poll until response received or 8 bytes passed
    // note: expect to see R1 in second 0xFF (transmit line must be active for clock and receive to be active)
    while ((count < 8) & ((res = spi_transfer(0xFF)) == 0xFF))
    {
        count++;
    }

    // return response byte
    return res;
}

SD_readRes7(uint8_t *res)
{
    // read response 1 in R7
    res[0] = SD_readRes1(); // note: reusing code

    // if error reading R1 (version 1 SD card), return
    if(res[0] > 1) {
        return;
    }

    // store next 4 bytes
    uint8_t i;
    for (i = 1; i < 5; i++) {
        res[i] = spi_transfer(0xFF);
    }
}

void SD_printR1(uint8_t res)
{
    if(res & 0b10000000)
        { uart_send_bytes("\tError: MSB = 1\r", sizeof("\tError: MSB = 1\r") - 1); return; }
    if(res == 0)
        { uart_send_bytes("\tCard Ready\r", sizeof("\tCard Ready\r") - 1); return; }
    if(PARAM_ERROR(res))
        uart_send_bytes("\tParameter Error\r", sizeof("\tParameter Error\r") - 1);
    if(ADDR_ERROR(res))
        uart_send_bytes("\tAddress Error\r", sizeof("\tAddress Error\r") - 1);
    if(ERASE_SEQ_ERROR(res))
        uart_send_bytes("\tErase Sequence Error\r", sizeof("\tErase Sequence Error\r") - 1);
    if(CRC_ERROR(res))
        uart_send_bytes("\tCRC Error\r", sizeof("\tCRC Error\r") - 1);
    if(ILLEGAL_CMD(res))
        uart_send_bytes("\tIllegal Command\r", sizeof("\tIllegal Command\r") - 1);
    if(ERASE_RESET(res))
        uart_send_bytes("\tErase Reset Error\r", sizeof("\tErase Reset Error\r") - 1);
    if(IN_IDLE(res))
        uart_send_bytes("\tIn Idle State\r", sizeof("\tIn Idle State\r") - 1);
}

void SD_printR7(uint8_t *res)
{
    SD_printR1(res[0]);

    if(res[0] > 1) return;

    uart_send_bytes("\tCommand Version: ", sizeof("\tCommand Version: ") - 1);
    uart_send_hex8(CMD_VER(res[1]));
    uart_send_bytes("\r", sizeof("\r") - 1);

    uart_send_bytes("\tVoltage Accepted: ", sizeof("\tVoltage Accepted: ") - 1);
    if(VOL_ACC(res[3]) == VOLTAGE_ACC_27_33)
        uart_send_bytes("2.7-3.6V\r", sizeof("2.7-3.6V\r") - 1);
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_LOW)
        uart_send_bytes("LOW VOLTAGE\r", sizeof("LOW VOLTAGE\r") - 1);
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES1)
        uart_send_bytes("RESERVED\r", sizeof("RESERVED\r") - 1);
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES2)
        uart_send_bytes("RESERVED\r", sizeof("RESERVED\r") - 1);
    else
        uart_send_bytes("NOT DEFINED\r", sizeof("NOT DEFINED\r") - 1);

    uart_send_bytes("\tEcho: ", sizeof("\tEcho: ") - 1);
    uart_send_hex8(res[4]);
    uart_send_bytes("\r", sizeof("\r") - 1);
}

void SD_powerUpSeq()
{
    // SD card reader must be held high during power up
    CS_DISABLE();

    // give SD card at least 1ms to power up
    __delay_cycles(104858);

    // send 80 clock cycles to synchronize where 1 byte is 8 clock cycles
    uint8_t i;
    for (i = 0; i < 10; i++)
    {
        spi_transfer(0xFF);
    }

    // deselect SD card
    CS_DISABLE();
    spi_transfer(0xFF);
}

uint8_t SD_goIdleState()
{
    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send CMD0
    // index: 0
    // argument: stuff bits
    // CRC: 10010100b (page 43 of physical spec)
    SD_command(0, 0x00000000, 0x94);

    // read response
    uint8_t res1 = SD_readRes1();

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);

    return res1;
}

void SD_sendInterfaceCond(uint8_t *res)
{
    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send CM8
    // index: 8
    // VHS: 001b (for 3.3V)
    // check pattern: 10101010b
    // CRC: 1000011b
    SD_command(8, 0x0000001AA, 0x86); // "1AA" for arg's last 12 bits which includes VHS and check pattern. 0x86 is CRC7 in bits 7:1 (1000011 << 1)

    // read response
    SD_readRes7(res);

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);
}

// ISR for USCI_A1 for SPI
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (UCA1IV)
    {
    case USCI_SPI_UCTXIFG: // transmit flag

        if (current_lengthSPI > 0)
        {
            UCA1TXBUF = address_bufferSPI[next_idx_to_sendSPI++];

            current_lengthSPI--;
        }
        else if (rwStatus && (receive_idxSPI != receiveLengthSPI + 1)) // send if not at the end, note: +1 needed as second to last receive byte is read
        {
            while (!(UCA1IFG & UCRXIFG))
                ; // poll until byte received

            UCA1TXBUF = 0xFF;
            received_bytesSPI[receive_idxSPI] = UCA1RXBUF;
            receive_idxSPI++;
        }
        else
        { // transmission end
            while (!(UCA1IFG & UCRXIFG))
                ; // poll until byte received

            UCA1IE &= ~UCTXIE;
            UCA1IFG |= UCTXIFG;
            UCA1IFG &= ~UCRXIFG; // note: flags need to be reset for same functionality after

            receivedStatus = 1;
            receive_idxSPI = 0; // reset idx
        }
        break;

    case USCI_SPI_UCRXIFG: // receive flag
        break;
    default:
        break;
    }
}

void SDInit()
{
    const uint8_t meas = 0xF4;
    const uint8_t measSettings = 0b00000111; // osrs_p & power mode

    spi_write(meas, measSettings);
}

void getBytesSPI(uint8_t registerAddress, uint8_t *storeByte, int numBytes)
{
    spi_receive(registerAddress, numBytes);

    // store each byte read
    int i;
    for (i = 1; i < numBytes + 1; i++)
    {
        storeByte[i - 1] = received_bytesSPI[i];
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
