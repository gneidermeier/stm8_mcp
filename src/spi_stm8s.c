/**
  ******************************************************************************
  * @file spi.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2021
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <ctype.h> // isprint
#include <string.h> // memset

// unfortunately this has to be included merely for SPI ENABLED define
// todo consider -DSPI_ENABLED ? in project/makefile
#include "system.h"

#if defined (SPI_ENABLED)

// app headers
#include "mcu_stm8s.h"


/* Private defines -----------------------------------------------------------*/

/* Chip select */
#define CS_PIN      5

#define TIME_OUT_0 0x1000 // timer1 free running, time between periodic task is 0x2080
#define TIME_OUT_1 0x0002 //
#define RX_BUF_SZ 16


/* Private variables ---------------------------------------------------------*/

uint8_t spi_rx_buf[RX_BUF_SZ];


/* Private functions ---------------------------------------------------------*/

int SPI_read_write_b(uint8_t * chbuf, uint8_t data, uint16_t time_out)
{
    uint8_t index = 0;
    uint8_t hup = FALSE;
    uint8_t spi_rx;
    uint8_t spi_tx;
    uint16_t start = TIM1_GetCounter();
    (void)data; // unused atm

// shouldn't need this if it worked correctly ;
    memset(spi_rx_buf, 0, SPI_RX_BUF_SZ);


    while( 1 )
    {
        if (SPI->SR & SPI_SR_RXNE)
        {
            // Clearing the RXNE bit is performed by reading the SPI_DR register
            spi_rx = SPI->DR;
            hup = FALSE; // as long as they keep sending ... keep listening
            index = (uint8_t)((index < (RX_BUF_SZ-1) ) ? index + 1 : 0);

            if ( spi_rx == 0xF2 )
            {
                index = 0;
                spi_rx = 'X'; // trust rx_cntr get reset!
            }
            spi_tx = chbuf[ index ]; // send something back

            while (! (SPI->SR & SPI_SR_TXE) );

            SPI->DR = spi_tx;

            spi_rx_buf[ index ] =  spi_rx; // trust rx_cntr get reset!
        }
        else
        {
            // if timed out then return error
            if ( (TIM1_GetCounter() - start) > time_out )
            {
//                index = -1; // timed out, return error
//								goto exit;
                return -1 ;
            }
            if (FALSE == hup )
            {
                // one more time in the loop
                hup = TRUE;
            }
            else // if hup == true
            {
                if (index > 0)
                {
//                    goto exit;
                    return index;
                }
            }
        }
    } // while

exit:
    return index;
}


/*
 * example codes for SPI functions from
 * https://lujji.github.io/blog/bare-metal-programming-stm8/#SPI
 */

void chip_select(void)
{
#ifdef SPI_CTRLR_USE_CS
    GPIOE->ODR &= (uint8_t)~(1 << CS_PIN);
#endif
}
void chip_deselect(void)
{
#ifdef SPI_CTRLR_USE_CS
    GPIOE->ODR |= (1 << CS_PIN);
#endif
}

void SPI_write(uint8_t data)
{
    SPI->DR = data;
    while (! (SPI->SR & SPI_SR_TXE) );
}

uint8_t SPI_read( void )
{
    SPI_write( 0xA5 );
    while ( ! (SPI->SR & SPI_SR_RXNE) );
    return SPI->DR;
}

uint8_t SPI_read_write(uint8_t data)
{
    while (! (SPI->SR & SPI_SR_TXE) );

    SPI->DR = data;

    while ( ! (SPI->SR & SPI_SR_RXNE) );

    return SPI->DR;
}


void SPI_controld(void)
{
    static uint8_t n;
    char sbuf[16]; // am i big enuff?
    uint8_t rxbuf[8] = { 0, 0, 0, 0 } ;

    n = (uint8_t)((n >= 0x30 && n < 126) ? n + 1 : 0x30);
    
// pretty sure it shouldnot be necessary to DI/EI ... all byte read/writes and
// SPI is not being run in interrupt mode. Not shared variables. EI/DI
// will tick the motor if the SPI rate is real low!
//     disableInterrupts();
    chip_select();

    rxbuf[0] = SPI_read_write(0xa5); // start of sequence
    rxbuf[1] = SPI_read_write( n );
    rxbuf[2] = SPI_read_write('1');
    rxbuf[3] = SPI_read_write('2');

    chip_deselect();
//    enableInterrupts();
// tmp dump test SPI test data to UART
    sbuf[0] = '>';
    sbuf[1] = n;
    sbuf[2] = isprint( (int)rxbuf[0] ) ? rxbuf[0] : '.' ;
    sbuf[3] = isprint( (int)rxbuf[1] ) ? rxbuf[1] : '.' ;
    sbuf[4] = isprint( (int)rxbuf[2] ) ? rxbuf[2] : '.' ;
    sbuf[5] = isprint( (int)rxbuf[3] ) ? rxbuf[3] : '.' ;
    sbuf[6] = 0;
    printf("%s\r\n", sbuf);
}



#if 0
static void SPI_periphd(void)
{
    static int i;
    char sbuf[16]; // am i big enuff?

    i = (i >= 0x30 && i < 126) ? i+1 : 0x30;
// tmp dump test SPI test data to UART
    disableInterrupts();
    sbuf[0] = '>';
    sbuf[1] = i;
    sbuf[2] = isprint( (int)SPI_rxbuf[0] ) ? SPI_rxbuf[0] : '.' ;
    sbuf[3] = isprint( (int)SPI_rxbuf[1] ) ? SPI_rxbuf[1] : '.' ;
    sbuf[4] = isprint( (int)SPI_rxbuf[2] ) ? SPI_rxbuf[2] : '.' ;
    sbuf[5] = isprint( (int)SPI_rxbuf[3] ) ? SPI_rxbuf[3] : '.' ;
    sbuf[6] = isprint( (int)SPI_rxbuf[4] ) ? SPI_rxbuf[4] : '.' ;
    sbuf[7] = '$';
    sbuf[8] = 0;
    is_SPI_rx = FALSE;
    memset(SPI_rxbuf, 0, SPI_RX_BUF_SZ); // tmp
    enableInterrupts();

    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
}
#endif

#endif // defined (SPI_ENABLED)

