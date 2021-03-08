/**
  ******************************************************************************
  * @file main.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <ctype.h> // isprint

// app headers
#include "mcu_stm8s.h"
#include "bldc_sm.h"
#include "per_task.h"


#ifdef _SDCC_
// Interrupt vectors must be implemented in the same file that implements main()
/*
If you have multiple source files in your project, interrupt service routines can be present in any of them, but a prototype of the isr MUST be present or included in the file that contains the function main.
*/
//  #include "stm8s_it.h" // not sure if this works ... enable stm8s_it.c to be built in the build config
#include "stm8s_it.c"
#endif


/* Private defines -----------------------------------------------------------*/

/* Chip select */
#define CS_PIN      5


/* Private functions ---------------------------------------------------------*/

/*
 * example codes for SPI functions from
 * https://lujji.github.io/blog/bare-metal-programming-stm8/#SPI
 */

void chip_select(void)
{
    GPIOE->ODR &= (uint8_t)~(1 << CS_PIN);
}
void chip_deselect(void)
{
    GPIOE->ODR |= (1 << CS_PIN);
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

#define TIME_OUT_0 0x1000 // timer1 free running, time between periodic task is 0x2080
#define TIME_OUT_1 0x0002 //
#define RX_BUF_SZ 16
uint8_t dbg_rx_buf[RX_BUF_SZ];

int SPI_read_write_b(uint8_t * chbuf, uint8_t data, uint16_t time_out)
{
    uint8_t index = 0;
    uint8_t hup = FALSE;
    uint8_t spi_rx;
    uint8_t spi_tx;
    uint16_t start = TIM1_GetCounter();
    (void)data; // unused atm

    while( 1 )
    {
        if (SPI->SR & SPI_SR_RXNE)
        {
            // Clearing the RXNE bit is performed by reading the SPI_DR register
            spi_rx = SPI->DR;
            hup = FALSE; // as long as they keep sending ... keep listening
            index = (index < (RX_BUF_SZ-1) ) ? index + 1 : 0;

            if ( spi_rx == 0xF2 )
            {
                index = 0;
                spi_rx = 'X'; // trust rx_cntr get reset!
            }
            spi_tx = chbuf[ index ]; // send something back

            while (! (SPI->SR & SPI_SR_TXE) );

            SPI->DR = spi_tx;

            dbg_rx_buf[ index ] =  spi_rx; // trust rx_cntr get reset!
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


static void SPI_controld(void)
{
    static int i;

    char sbuf[16]; // am i big enuff?
    uint8_t spi_rx_bfr[8] = { 0 } ;

    disableInterrupts();
    chip_select();

    spi_rx_bfr[0] = SPI_read_write(0xa5); // start of sequence

    i = (i + 1) & ~0x80; // clear test  bit
    spi_rx_bfr[1] = SPI_read_write(i);

    i+= 1;// test data
    spi_rx_bfr[2] = SPI_read_write(i);

    i = (i + 1) | 0x80; // test data mask in a test bit so alignment errors
    spi_rx_bfr[3] = SPI_read_write(i);

    chip_deselect();
    enableInterrupts();
// tmp dump test SPI test data to UART
    sbuf[0] = '>';
    sbuf[1] = spi_rx_bfr[0];
    sbuf[2] = spi_rx_bfr[1];
    sbuf[3] = spi_rx_bfr[2];
    sbuf[4] = spi_rx_bfr[3];
    sbuf[5] = 0;
    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
}

/**
 * @brief mainly looping
 */
void main(int argc, char **argv)
{
    static const uint8_t tx_buf[RX_BUF_SZ] = "0123456789ABCDEF";
    int linec = 0;
    uint8_t framecount = 0;
    uint8_t i = 0;
    (void) argc;
    (void) argv;

    MCU_Init();

    BL_reset();

    enableInterrupts(); // interrupts are globally disabled by default

//    UARTputs("\033c"); // sends ANSI code to  clear the serial terminal
    UARTputs("Program Startup....... \r\n");

    while(1)
    {
//  button input either button would transition from OFF->RAMP
        if (! (( GPIOA->IDR)&(1<<4)))
        {
//            while( ! (( GPIOA->IDR)&(1<<4)) ); // no concern for debounce for a stop switch
            disableInterrupts();
            UI_Stop();
            enableInterrupts();
        }

#if 0 //  TEST DEV ONLY: manual adjustment of commutation cycle time
        if (! (( GPIOA->IDR)&(1<<6)))
        {
            while( ! (( GPIOA->IDR)&(1<<6)) ); // wait for debounce (sorta works)
            disableInterrupts();
            BLDC_Spd_inc();
            enableInterrupts();
        }
#endif

#if 0 //  TEST DEV ONLY: manual adjustment of commutation cycle time
        if ( ! (( GPIOE->IDR)&(1<<5)))
        {
            while( ! (( GPIOE->IDR)&(1<<5)) ) {;} // wait for debounce (sorta works)
            disableInterrupts();
            BLDC_Spd_dec();
            enableInterrupts();
        }
#endif

        if ( TRUE == Task_Ready() )
        {
// periodic task is enabled at ~60 Hz ... the modulus provides a time reference of
// approximately 2 Hz at which time the master attempts to read a few bytes from SPI
            if ( ! ((framecount++) % 0x20) )
            {
#ifdef SPI_CONTROLLER
                SPI_controld();
#endif
            }
        }

#ifndef SPI_CONTROLLER
        if (-1 != SPI_read_write_b(tx_buf, 0xA5, TIME_OUT_0) )
        {
            char sbuf[16]; // am i big enuff?
            // tmp dump test SPI test data to UART
            linec = linec < 126 ? linec++ : 0x30;
            sbuf[0] = '>';
            sbuf[1] = linec++;
            sbuf[2] = isprint( (int)dbg_rx_buf[0] ) ? dbg_rx_buf[0] : '.' ;
            sbuf[3] = isprint( (int)dbg_rx_buf[1] ) ? dbg_rx_buf[1] : '.' ;
            sbuf[4] = isprint( (int)dbg_rx_buf[2] ) ? dbg_rx_buf[2] : '.' ;
            sbuf[5] = isprint( (int)dbg_rx_buf[3] ) ? dbg_rx_buf[3] : '.' ;
            sbuf[6] = isprint( (int)dbg_rx_buf[4] ) ? dbg_rx_buf[4] : '.' ;
            sbuf[7] = 0;
            strcat(sbuf, "\r\n");
            UARTputs(sbuf);
            memset(dbg_rx_buf, 0, SPI_RX_BUF_SZ);
        }
#endif // SPI CONT
    } // while 1
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
