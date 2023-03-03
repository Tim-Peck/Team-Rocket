#include <msp430.h>
#include "SPI.h"

//#define METADATA // uncomment to read metadata and first data block

// note for uint8_t: NON-ZERO NUMBERS ARE TRUE WHILE ZERO IS FALSE (so e.g. 0b10 & 0b11 is TRUE while 0b10 & 0b01 is FALSE)
// these check if a bit is positive
// R1 macros
#define PARAM_ERROR(X)      X & 0b01000000
#define ADDR_ERROR(X)       X & 0b00100000
#define ERASE_SEQ_ERROR(X)  X & 0b00010000
#define CRC_ERROR(X)        X & 0b00001000
#define ILLEGAL_CMD(X)      X & 0b00000100
#define ERASE_RESET(X)      X & 0b00000010
#define IN_IDLE(X)          X & 0b00000001

// R7 macros
#define CMD_VER(X)          ((X >> 4) & 0xF0) // input: byte 31:24 (command version for bits 31:28)
#define VOL_ACC(X)          (X & 0x1F) // input: byte 15:8 (voltage accept for bits 11:8)

#define VOLTAGE_ACC_27_33   0b00000001
#define VOLTAGE_ACC_LOW     0b00000010
#define VOLTAGE_ACC_RES1    0b00000100
#define VOLTAGE_ACC_RES2    0b00001000

// R3 macros
#define POWER_UP_STATUS(X)  X & 0x80 // input: byte 31:24 (card power up status for bit 31)
#define CCS_VAL(X)          X & 0x40 // input: byte 31:24 (card capacity status for bit 30)
#define VDD_2728(X)         X & 0b10000000 // voltages supported by card
#define VDD_2829(X)         X & 0b00000001
#define VDD_2930(X)         X & 0b00000010
#define VDD_3031(X)         X & 0b00000100
#define VDD_3132(X)         X & 0b00001000
#define VDD_3233(X)         X & 0b00010000
#define VDD_3334(X)         X & 0b00100000
#define VDD_3435(X)         X & 0b01000000
#define VDD_3536(X)         X & 0b10000000

// Read error token macros
#define TOKEN_ERROR(X)      X & 0b00000001
#define OUT_OF_RANGE(X)     X & 0b00001000
#define ECC_FAIL(X)         X & 0b00000100
#define CC_ERROR(X)         X & 0b00000010
// Write error token macros
#define WRITE_ERROR(X)         X & 0b00000100
#define WRITE_CRC_ERROR(X)     X & 0b00000010

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
    P2SEL0 |= (BIT4 | BIT5 | BIT6); // primary module function (SPI)
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
    // UCCKPH sets data to be captured on first edge before changing
    // UCMODE_2 sets eUSCI_A to SPI mode with active low CS
    // UCMST sets master mode
    // UCSYNC sets synchronous mode (SPI mode)
    // UCSTEM = 1, sets STE pin to be CS mode (needs to be set even though pin is in GPIO function)
    // UCMSB sets Most Significant Bit first

    // set clock source of USCI_A
    UCA1CTLW0 |= UCSSEL__SMCLK; // control register UCSSELx field, set clock source to SMCLK (which is same as MCLK at ~1MHz)
    // configuring baud rate registers for 100kHz when sourcing from SMCLK where SMCLK = 1048576 Hz
    // acts as divisor for BRCLK
    UCA1BRW |= 10;

    UCA1CTLW0 &= ~UCSWRST; // bring SPI out of reset (Testing note: This brings SIMO and CS low for some reason)
}

// note: in order to receive for SPI, transmit line must be active so this transmit acts as receiving as well
uint8_t spi_transfer(uint8_t byte)
{
    // load data into register
    UCA1TXBUF = byte;

    // poll until byte received
    while (!(UCA1IFG & UCRXIFG))
        ;

    // return PREVIOUS byte received in receive register (i.e. not from this transferred byte)
    // T: |Y|
    // R: |X|Z|
    // (so byte X received with Y transferred with this function)
    return UCA1RXBUF;
}

// see command format in notes
void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
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

void SD_readRes7(uint8_t *res)
{
    // read response 1 in R7
    res[0] = SD_readRes1(); // note: reusing code

    // if error reading R1 (version 1 SD card), return
    if (res[0] > 1)
    {
        return;
    }

    // store next 4 bytes
    uint8_t i;
    for (i = 1; i < 5; i++)
    {
        res[i] = spi_transfer(0xFF);
    }

    // send extra 0xFF after response for safety
    spi_transfer(0xFF);
}

void SD_printR1(uint8_t res)
{
    if (res & 0b10000000)
    {
        uart_send_bytes("\tError: MSB = 1\r", sizeof("\tError: MSB = 1\r") - 1);
        return;
    }
    if (res == 0)
    {
        uart_send_bytes("\tCard Ready\r", sizeof("\tCard Ready\r") - 1);
        return;
    }
    if (PARAM_ERROR(res))
        uart_send_bytes("\tParameter Error\r",
                        sizeof("\tParameter Error\r") - 1);
    if (ADDR_ERROR(res))
        uart_send_bytes("\tAddress Error\r", sizeof("\tAddress Error\r") - 1);
    if (ERASE_SEQ_ERROR(res))
        uart_send_bytes("\tErase Sequence Error\r",
                        sizeof("\tErase Sequence Error\r") - 1);
    if (CRC_ERROR(res))
        uart_send_bytes("\tCRC Error\r", sizeof("\tCRC Error\r") - 1);
    if (ILLEGAL_CMD(res))
        uart_send_bytes("\tIllegal Command\r",
                        sizeof("\tIllegal Command\r") - 1);
    if (ERASE_RESET(res))
        uart_send_bytes("\tErase Reset Error\r",
                        sizeof("\tErase Reset Error\r") - 1);
    if (IN_IDLE(res))
        uart_send_bytes("\tIn Idle State\r", sizeof("\tIn Idle State\r") - 1);
}

void SD_printR7(uint8_t *res)
{
    SD_printR1(res[0]);

    if (res[0] > 1)
        return;

    uart_send_bytes("\tCommand Version: ", sizeof("\tCommand Version: ") - 1);
#ifdef METADATA
    uart_send_hex8(CMD_VER(res[1]));
#endif
    uart_send_bytes("\r", sizeof("\r") - 1);

    uart_send_bytes("\tVoltage Accepted: ", sizeof("\tVoltage Accepted: ") - 1);
    if (VOL_ACC(res[3]) == VOLTAGE_ACC_27_33)
        uart_send_bytes("2.7-3.6V\r", sizeof("2.7-3.6V\r") - 1);
    else if (VOL_ACC(res[3]) == VOLTAGE_ACC_LOW)
        uart_send_bytes("LOW VOLTAGE\r", sizeof("LOW VOLTAGE\r") - 1);
    else if (VOL_ACC(res[3]) == VOLTAGE_ACC_RES1)
        uart_send_bytes("RESERVED\r", sizeof("RESERVED\r") - 1);
    else if (VOL_ACC(res[3]) == VOLTAGE_ACC_RES2)
        uart_send_bytes("RESERVED\r", sizeof("RESERVED\r") - 1);
    else
        uart_send_bytes("NOT DEFINED\r", sizeof("NOT DEFINED\r") - 1);

    uart_send_bytes("\tEcho: ", sizeof("\tEcho: ") - 1);
#ifdef METADATA
    uart_send_hex8(res[4]);
#endif
    uart_send_bytes("\r", sizeof("\r") - 1);
}

void SD_printR3(uint8_t *res)
{
    SD_printR1(res[0]);

    if (res[0] > 1)
        return;

    uart_send_bytes("\tCard Power Up Status: ",
                    sizeof("\tCard Power Up Status: ") - 1);
    // print card power up status
    if (POWER_UP_STATUS(res[1]))
    {
        uart_send_bytes("READY\r", sizeof("READY\r") - 1);
        // print card capacity status if card ready
        uart_send_bytes("\tCard Capacity Status: ",
                        sizeof("\tCard Capacity Status: ") - 1);
        if (CCS_VAL(res[1]))
        {
            uart_send_bytes("1\r", sizeof("1\r") - 1);
        }
        else
            uart_send_bytes("0\r", sizeof("0\r") - 1);
    }
    else
    {
        uart_send_bytes("BUSY\r", sizeof("BUSY\r") - 1);
    }

    // print voltage ranges of SD card
    uart_send_bytes("\tVDD Window: ", sizeof("\tVDD Window: ") - 1);
    if (VDD_2728(res[3]))
        uart_send_bytes("2.7-2.8, ", sizeof("2.7-2.8, ") - 1);
    if (VDD_2829(res[2]))
        uart_send_bytes("2.8-2.9, ", sizeof("2.8-2.9, ") - 1);
    if (VDD_2930(res[2]))
        uart_send_bytes("2.9-3.0, ", sizeof("2.9-3.0, ") - 1);
    if (VDD_3031(res[2]))
        uart_send_bytes("3.0-3.1, ", sizeof("3.0-3.1, ") - 1);
    if (VDD_3132(res[2]))
        uart_send_bytes("3.1-3.2, ", sizeof("3.1-3.2, ") - 1);
    if (VDD_3233(res[2]))
        uart_send_bytes("3.2-3.3, ", sizeof("3.2-3.3, ") - 1);
    if (VDD_3334(res[2]))
        uart_send_bytes("3.3-3.4, ", sizeof("3.3-3.4, ") - 1);
    if (VDD_3435(res[2]))
        uart_send_bytes("3.4-3.5, ", sizeof("3.4-3.5, ") - 1);
    if (VDD_3536(res[2]))
        uart_send_bytes("3.5-3.6", sizeof("3.5-3.6") - 1);
    uart_send_bytes("\r", sizeof("\r") - 1);

}

void SD_printReadTokenError(uint8_t token)
{
    uart_send_bytes("\tSD read error: ", sizeof("\tSD read error: ") - 1);
    if (token == 0xFF)
    {
        uart_send_bytes("Timeout", sizeof("Timeout") - 1);
    }
    if (OUT_OF_RANGE(token))
    {
        uart_send_bytes("Out of range", sizeof("Out of range") - 1);
    }
    if (ECC_FAIL(token))
    {
        uart_send_bytes("ECC fail", sizeof("ECC fail") - 1);
    }
    if (CC_ERROR(token))
    {
        uart_send_bytes("CC error", sizeof("CC error") - 1);
    }
    uart_send_bytes("\r", sizeof("\r") - 1);
}

void print_SDBlock(uint8_t R1, uint8_t *buf, uint8_t *token)
{
    // check if R1 and token valid
    if ((R1 > 1) | (*token != 0xFE))
    {
        return;
    }

    uint16_t i;

#ifdef METADATA
    for (i = 0; i < 512; i++)
    {
        uart_send_hex8(buf[i]);
    }
#endif

    uart_send_bytes("\r------------------\r", sizeof("\r------------------\r"));
}

void SD_printWriteTokenError(uint8_t token)
{
    uart_send_bytes("\tSD write error: ", sizeof("\tSD write error: ") - 1);
    if (token == 0)
    {
        uart_send_bytes("Timeout", sizeof("Timeout") - 1);
    }
    if (WRITE_CRC_ERROR(token))
    {
        uart_send_bytes("CRC error", sizeof("CRC error") - 1);
    }
    if (WRITE_ERROR(token))
    {
        uart_send_bytes("write error", sizeof("write error") - 1);
    }
    uart_send_bytes("\r", sizeof("\r") - 1);
}

void SD_powerUpSeq()
{
    // SD card reader must be held high during power up
    CS_DISABLE();

    // give SD card at least 1ms to power up
    __delay_cycles(1050);

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

    // read response R1
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
    // VHS (argument): 001b (for 3.3V)
    // check pattern (argument): 10101010b
    // CRC: 1000011b
    SD_command(8, 0x0000001AA, 0x86); // "1AA" for arg's last 12 bits which includes VHS and check pattern. 0x86 is CRC7 in bits 7:1 (1000011 << 1)

    // read response
    SD_readRes7(res);

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);
}

void SD_readOCR(uint8_t *res)
{
    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send CMD58
    // index: 58
    // argument: stuff bits
    SD_command(58, 0, 0);

    // read response R3 (same number of bytes as R7)
    SD_readRes7(res);

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);
}

uint8_t SD_sendAppCommand()
{
    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send CMD55
    // index: 55
    // argument: stuff bits
    SD_command(55, 0, 0);

    // read response R1
    uint8_t res1 = SD_readRes1();

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);

    return res1;
}

uint8_t SD_sendOCR()
{
    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send ACMD41
    // index: 41
    // bit 30 to indicate high capacity cards supported (0b01000000 << 24 = 0x40000000)
    // NOTE: high capacity cards appear to not initialize if bit 30 not set
    SD_command(41, 0x40000000, 0);

    // read response R1
    uint8_t res1 = SD_readRes1();

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);

    return res1;
}

uint8_t SD_init()
{
    uint8_t res[5], cmdAttempts = 0;

    // start power up sequence
    SD_powerUpSeq();

    // command card to idle and switch to SPI mode (CMD0)
    // give 10 attempts
    do
    {
        uart_send_bytes("Sending CMD0...\r", sizeof("Sending CMD0...\r"));
        res[0] = SD_goIdleState();
        uart_send_bytes("Response:\r", sizeof("Response:\r"));
        SD_printR1(res[0]);
        uart_send_bytes("------------------\r", sizeof("------------------\r"));

        cmdAttempts++;
        if (cmdAttempts > 9)
        {
            return 0;
        }
    }
    while (res[0] != 0x01); // R1's first bit should be 1 for idle state

    // check interface conditions (CMD8)
    uart_send_bytes("Sending CMD8...\r", sizeof("Sending CMD8...\r"));
    SD_sendInterfaceCond(res);
    uart_send_bytes("Response:\r", sizeof("Response:\r"));
    SD_printR7(res);
    uart_send_bytes("------------------\r", sizeof("------------------\r"));
    // check if first gen card
    if (res[0] != 0x01)
    {
        uart_send_bytes("First gen card\r", sizeof("First gen card\r"));
        return 0;
    }
    // check echo pattern
    if (res[4] != 0xAA)
    {
        uart_send_bytes("Wrong echo pattern\r", sizeof("Wrong echo pattern\r"));
        return 0;
    }

    // attempt SD initialization (ACMD41)
    cmdAttempts = 0;
    do
    {
        if (cmdAttempts > 100)
        { // return if not initialized after 100 attempts
            return 0;
        }

        // send CMD55
        uart_send_bytes("Sending CMD55...\r", sizeof("Sending CMD55...\r"));
        res[0] = SD_sendAppCommand();
        uart_send_bytes("Response:\r", sizeof("Response:\r"));
        SD_printR1(res[0]);
        uart_send_bytes("------------------\r", sizeof("------------------\r"));

        // send ACMD41
        uart_send_bytes("Sending ACMD41...\r", sizeof("Sending ACMD41...\r"));
        res[0] = SD_sendOCR();
        uart_send_bytes("Response:\r", sizeof("Response:\r"));
        SD_printR1(res[0]);
        uart_send_bytes("------------------\r", sizeof("------------------\r"));
        __delay_cycles(11000);

        cmdAttempts++;
    }
    while (res[0] != 0); // send initialization sequence 100 times max

    // check operating conditions (CMD58)
    uart_send_bytes("Sending CMD58...\r", sizeof("Sending CMD58...\r"));
    SD_readOCR(res);
    uart_send_bytes("Response:\r", sizeof("Response:\r"));
    SD_printR3(res);
    uart_send_bytes("------------------\r", sizeof("------------------\r"));
    // return if card not ready
    if (!(POWER_UP_STATUS(res[1])))
    {
        return 0;
    }

    return 1;
}

uint8_t SD_readSingleBlock(uint32_t addr, uint8_t *buf, uint8_t *token)
{
    uart_send_bytes("Reading SD card block\r",
                    sizeof("Reading SD card block\r"));
    uint8_t cycleCount = 0;

    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send CMD17
    // index: 17
    // argument: address
    // CRC: N/A
    SD_command(17, addr, 0);

    // read response R1
    uint8_t res1 = SD_readRes1();

    // if SD card ready (R1 value of 0), read data block
    if (!res1)
    {
        // transfer 0xFF and poll for token
        do
        {
            *token = spi_transfer(0xFF);
            cycleCount++;
        }
        while ((*token == 0xFF) & (cycleCount < 10486)); // stop if token received or past timeout (100ms)

        // check token before reading data
        if (TOKEN_ERROR(*token))
        {
            SD_printReadTokenError(*token);
        }
        else
        {
            uint16_t i;

            // read 512 bytes
            for (i = 0; i < 512; i++)
            {
                buf[i] = spi_transfer(0xFF);
            }

            // continue by reading 16 bit CRC
            spi_transfer(0xFF);
            spi_transfer(0xFF);

            uart_send_bytes("\tSD read success\r",
                            sizeof("\tSD read success\r"));
        }
    }

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);

    uart_send_bytes("------------------\r", sizeof("------------------\r"));

    return res1;
}

uint8_t SD_writeSingleBlock(uint32_t addr, uint8_t *writeBuf, uint8_t *token)
{
    uart_send_bytes("Writing SD card block\r",
                    sizeof("Writing SD card block\r"));
    uint8_t cycleCount = 0;

    // assert chip select
    spi_transfer(0xFF); // 0xFF send before and after CS for safety (see notes 29/12)
    CS_ENABLE();
    spi_transfer(0xFF);

    // send CMD24
    // index: 24
    // argument: address
    // CRC: N/A
    SD_command(24, addr, 0);

    // read response R1
    uint8_t res1 = SD_readRes1();

    // if SD card ready (R1 value of 0), write data block
    if (!res1)
    {
        uint16_t i;

        // send start block token
        spi_transfer(0xFE);

        // send data block
        for (i = 0; i < 512; i++)
        {
            spi_transfer(writeBuf[i]);
        }

        // wait for data response token
        // transfer 0xFF and poll for token
        do
        {
            *token = spi_transfer(0xFF);
            cycleCount++;
        }
        while ((*token == 0xFF) & (cycleCount < 26215)); // stop if token received or past timeout (250ms)

        // check token, if error send stop command (CMD 12)
        // note: must use compare to check for pattern and not just & like macros for one bit
        if (!((*token & 0b00001111) == 0b00000101))
        {
            // send CMD12
            // index: 12
            // argument: N/A
            // CRC: N/A
            SD_command(12, 0, 0);

            res1 = SD_readRes1();

            SD_printWriteTokenError(*token);
        }
        else
        {
            cycleCount = 0;

            // wait until card finishes writing by reading till no busy tokens
            while ((spi_transfer(0xFF) == 0) & (cycleCount++ < 26215))
                ;
            if (cycleCount == 26215)
            {
                uart_send_bytes("\tBusy signal timeout\r",
                                sizeof("\tBusy signal timeout\r") - 1);
            }
            else
            {
                uart_send_bytes("\tSD write success\r",
                                sizeof("\tSD write success\r") - 1);
            }
        }

    }

    // deassert chip select
    spi_transfer(0xFF);
    CS_DISABLE();
    spi_transfer(0xFF);

    uart_send_bytes("------------------\r", sizeof("------------------\r"));

    return res1;
}

uint8_t checkFinishStatus() {
    uint8_t buf[512], token;

    // read block 0
    SD_readSingleBlock(0, buf, &token);

    // return status byte
    return buf[0];
}
