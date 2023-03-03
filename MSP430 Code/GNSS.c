#include <msp430.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "Defines.h"
#include "GNSS.h"
#include "UART.h"

void uart_GNSS_init()
{
    next_idx_to_send_GNSS = 0;
    next_idx_to_store_GNSS = 0;
    current_length_GNSS = 0;

    NMEA_sentence[0] = 0; //set default value for when GGA sentence not received yet
    receive_idx2 = 0;
    check_idx = 0;

    //TESTING - REMOVE THIS
//    uint8_t NMEA_sentence1[] =
//            ",112037.000,3654.4991,S,17444.8546,E,1,04,1.47,110.7,";
//    memcpy(NMEA_sentence, NMEA_sentence1, 53);

#ifdef USE_DEV_BOARD
    // Configure pins
    P1SEL0 |= BIT6 | BIT7; // P1.6=RXD, P1.7=TXD (is default high)
    // Configure UART
    UCA0CTLW0 |= UCSWRST;       // Put UART state machine in reset

    // Baud Rate calculation - we want 9600 baud (below are fields of registers)
    // N = clock / baud = 1,048,576 Hz / 9600 = 109.227
    // N > 16 so UCOS16 = 1
    // UCA0BR0 = int(N / 16) = int(109.227 / 16) = int(6.827) = 6
    // UCBRSx  = 0x22 as per users guide table "UCBRSx settings"
    // UCBRFx  = int([(N / 16) - int(N / 16)] * 16) = int(13.227) = 8
    UCA0BRW = 6;
    UCA0MCTLW = (0x22 << 8) | UCBRF_13 | UCOS16;

    UCA0CTLW0 &= ~UCSWRST;      // Initialize GNSS state machine
#else
    // Configure pins
    P4SEL0 |= BIT2 | BIT3; // P4.2=RXD, P4.3=TXD (is default high)
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
#endif

}

void GNSS_send_byte(uint8_t byte)
{
    byte_sent_GNSS = 0;

    if (current_length_GNSS < BUFFER_SIZE)
    {
        cmd_buffer[next_idx_to_store_GNSS++] = byte; // next_idx is always incremented and not reset
        current_length_GNSS++; // so current length alternates between 0 and 1 for each byte
    }

    // enable transmit interrupt to start transmitting
    // if we are already transmitting this does nothing

    #ifdef USE_DEV_BOARD
      UCA0IE |= UCTXIE;
    #else
      UCA1IE |= UCTXIE;
    #endif

    // poll until byte is sent
    while (!byte_sent_GNSS)
        ;
}

void GNSS_send_bytes(uint8_t *bytes, uint8_t number_of_bytes)
{
    uint8_t i;
    for (i = 0; i < number_of_bytes; i++)
    {
        GNSS_send_byte(bytes[i]);
    }
}

void GNSS_cmd(int cmd_ID)
{
    uint8_t cmd[8] = { '$', 'P', 'M', 'T', 'K' };

    // convert each digit of cmd_ID to ASCII characters
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        cmd[5 + i] = (uint8_t) ((cmd_ID % 10) + 48); // note: must convert int to ASCII number
        cmd_ID = cmd_ID / 10;
    }

    // send command
    GNSS_send_bytes(cmd, 8);
}

void GNSS_receive()
{
  // enable receive interrupt to begin receiving ASCII characters

  #ifdef USE_DEV_BOARD
    UCA0IE |= UCRXIE;
  #else
    UCA1IE |= UCRXIE;
  #endif

}

uint8_t fixAcquired()
{
    uint8_t idx = 0;
    uint8_t commaCount = 0;

//    uart_send_bytes(NMEA_sentence, 40); // print NMEA_sentence for testing

    // check if NMEA sentence received first
    // GGA sentence may have not been reached yet
    if (!NMEA_sentence[0])
    {
        return 0;
    }

    // walk the sentence till fix field
    while (commaCount != 6)
    {
        // note: idx will be incremented to after the target comma
        if (NMEA_sentence[idx++] == ',')
        {
            commaCount++; // increment commaCount when comma delimiter encountered
        }
    }

    // check if fix acquired
    if (NMEA_sentence[idx] == '1')
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


float parse_GGA_alt()
{
    uint8_t idx = 0;
    uint8_t commaCount = 0;

    // check if fix acquired first
    if (!fixAcquired())
    {
        return 0;
    }

    // walk the GGA sentence till altitude field reached
    while (commaCount != 9)
    {
        if (NMEA_sentence[idx++] == ',')
        {
            commaCount++; // increment commaCount when comma delimiter encountered
        }
    }

    // return float value of altitude
    return ASCII_to_float(NMEA_sentence + idx);

}

void parse_GGA_UTC(uint8_t *UTC)
{

    if (!fixAcquired())
    {
        return;
    }

    // extract ASCII values for UTC
    uint8_t h1 = NMEA_sentence[1];
    uint8_t h0 = NMEA_sentence[2];
    uint8_t m1 = NMEA_sentence[3];
    uint8_t m0 = NMEA_sentence[4];
    uint8_t s1 = NMEA_sentence[5];
    uint8_t s0 = NMEA_sentence[6];

    // store hour, minute, and second values
    UTC[0] = ASCII_to_uint8(h1, h0);
    UTC[1] = ASCII_to_uint8(m1, m0);
    UTC[2] = ASCII_to_uint8(s1, s0);
}

void parse_GGA_GCS(float *GCS)
{
    uint8_t idx = 0;
    uint8_t commaCount = 0;
    float latitude = 0;
    float longitude = 0;

    // check if fix acquired first
    if (!fixAcquired())
    {
        return;
    }

    // retrieving latitude
    // walk the GGA sentence till latitude field reached
    while (commaCount != 2)
    {
        if (NMEA_sentence[idx++] == ',')
        {
            commaCount++; // increment commaCount when comma delimiter encountered
        }
    }
    // note: latitude format is ddmm.mmmm,N (or S)
    // calculate integer part from degrees
    latitude += (float) (NMEA_sentence[idx] - 48) * 10; // tens
    latitude += (float) (NMEA_sentence[idx + 1] - 48); // ones
    // calculate fractional part from arcminutes
    latitude += (float) 1 / 60 * ASCII_to_float(NMEA_sentence + idx + 2);
    // check direction if south for negative angle
    // walk the GGA sentence till N/S field reached
    while (commaCount != 3)
    {
        if (NMEA_sentence[idx++] == ',')
        {
            commaCount++; // increment commaCount when comma delimiter encountered
        }
    }
    if (NMEA_sentence[idx] == 'S')
    {
        latitude *= -1;
    }

    // retrieving longitude
    idx += 2;
    // note: longitude format is dddmm.mmmm,W (or E)
    // calculate integer part from degrees
    longitude += (float) (NMEA_sentence[idx] - 48) * 100; // hundreds, note: casts have higher precedence than *
    longitude += (float) (NMEA_sentence[idx + 1] - 48) * 10; // tens
    longitude += (float) (NMEA_sentence[idx + 2] - 48); // ones
    // calculate fractional part from arcminutes
    longitude += (float) 1 / 60 * ASCII_to_float(NMEA_sentence + idx + 3);
    // check direction if west to convert to east
    // walk the GGA sentence till W/E field reached
    while (commaCount != 4)
    {
        if (NMEA_sentence[idx++] == ',')
        {
            commaCount++; // increment commaCount when comma delimiter encountered
        }
    }
    if (NMEA_sentence[idx] == 'W') // should never happen because we'd have to take a flight LOL
    {
        longitude = 360 - longitude;
    }

    // store values
    GCS[0] = latitude;
    GCS[1] = longitude;
}

float ASCII_to_float(uint8_t *firstDigitPtr)
{
    uint8_t intCount = 0;
    uint8_t nonIntCount = 0;
    uint8_t i;
    float value = 0;

    // count number of integers
    while (firstDigitPtr[intCount] != '.')
    {
        intCount++;
    }

    // count number of nonintegers
    while (*(firstDigitPtr + intCount + 1 + nonIntCount) != ',')
    {
        nonIntCount++;
    }

    // calculate integer part of float
    for (i = 0; i < intCount; i++)
    {
        value += (float) (firstDigitPtr[i] - 48)
                * powf(10, (float) (intCount - 1 - i));
    }

    // calculate noninteger part of float
    for (i = 0; i < nonIntCount; i++)
    {
        value += (float) (*(firstDigitPtr + intCount + 1 + i) - 48)
                * powf(10, (float) (-1 - i));
    }

    return value;
}

void float_to_uint8(float floatVal, uint8_t *rawFloatPtr)
{
    memcpy(rawFloatPtr, &floatVal, sizeof(floatVal));
}

uint8_t ASCII_to_uint8(uint8_t MSD, uint8_t LSD)
{
    // convert from ASCII to binary value of digit
    MSD -= 48;
    LSD -= 48;

    return (MSD * 10) + LSD;
}

  #ifdef USE_DEV_BOARD
    // ISR TO USCI_A0
    // stores GGA sentence to NMEA_sentence every second (independent of micro timer)
    #pragma vector=USCI_A0_VECTOR
    __interrupt void USCI_A0_ISR(void)
    {
        __bis_SR_register(GIE); // enable interrupt nesting

        switch (UCA0IV)
        {
        case USCI_NONE:
            break;

        case USCI_UART_UCRXIFG:
            received_byte = UCA0RXBUF;

    //            uart_send_byte(received_byte); // print all GNSS bytes

            // check receive sequence for GGA
            if (received_byte == NMEA_ID[check_idx])
            {
                check_idx++;
            }
            else if (check_idx == 3)
            {
                // GGA sequence found
                // store new NMEA sentence
                if (received_byte != '\n')
                {
                    NMEA_sentence[receive_idx2++] = received_byte;
                }
                else
                {
                    // reset indexes when whole NMEA sentence read
                    check_idx = 0;
                    receive_idx2 = 0;
                }
            }
            else
            {
                check_idx = 0;
            }
            break;

        case USCI_UART_UCTXIFG:
            if (current_length_GNSS > 0)
            {
                UCA0TXBUF = cmd_buffer[next_idx_to_send_GNSS++];
                current_length_GNSS--;
            }
            else
            {
                UCA0IE &= ~UCTXIE;
                UCA0IFG |= UCTXIFG;

                byte_sent_GNSS = 1;
            }
            break;

        case USCI_UART_UCSTTIFG:
            break;
        case USCI_UART_UCTXCPTIFG:
            break;
        default:
            break;
        }
    }
  #else

    // ISR TO USCI_A1
    // stores GGA sentence to NMEA_sentence every second (independent of micro timer)
    #pragma vector=USCI_A1_VECTOR
    __interrupt void USCI_A1_ISR(void)
    {
        __bis_SR_register(GIE); // enable interrupt nesting

        switch (UCA1IV)
        {
        case USCI_NONE:
            break;

        case USCI_UART_UCRXIFG:
            received_byte = UCA1RXBUF;

    //            uart_send_byte(received_byte); // print all GNSS bytes

            // check receive sequence for GGA
            if (received_byte == NMEA_ID[check_idx])
            {
                check_idx++;
            }
            else if (check_idx == 3)
            {
                // GGA sequence found
                // store new NMEA sentence
                if (received_byte != '\n')
                {
                    NMEA_sentence[receive_idx2++] = received_byte;
                }
                else
                {
                    // reset indexes when whole NMEA sentence read
                    check_idx = 0;
                    receive_idx2 = 0;
                }
            }
            else
            {
                check_idx = 0;
            }
            break;

        case USCI_UART_UCTXIFG:
            if (current_length_GNSS > 0)
            {
                UCA1TXBUF = cmd_buffer[next_idx_to_send_GNSS++];
                current_length_GNSS--;
            }
            else
            {
                UCA1IE &= ~UCTXIE;
                UCA1IFG |= UCTXIFG;

                byte_sent_GNSS = 1;
            }
            break;

        case USCI_UART_UCSTTIFG:
            break;
        case USCI_UART_UCTXCPTIFG:
            break;
        default:
            break;
        }
    }


  #endif
