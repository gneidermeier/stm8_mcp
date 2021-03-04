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

// app headers
#include "mcu_stm8s.h"
#include "bldc_sm.h"
#include "per_task.h"





extern uint8_t  SPI_rxbuf[]; // tmp
extern uint8_t  is_SPI_rx;  // tmp

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

void chip_select(void) {
    GPIOE->ODR &= (uint8_t)~(1 << CS_PIN);
}
void chip_deselect(void) {
    GPIOE->ODR |= (1 << CS_PIN);
}

void SPI_write(uint8_t data) {
    SPI->DR = data;
    while (! (SPI->SR & SPI_SR_TXE) );
}

uint8_t SPI_read( void ) {
    SPI_write( 0xA5 );
    while ( ! (SPI->SR & SPI_SR_RXNE) );
    return SPI->DR;
}

uint8_t SPI_read_write(uint8_t data) {

    while (! (SPI->SR & SPI_SR_TXE) );

    SPI->DR = data;

    while ( ! (SPI->SR & SPI_SR_RXNE) );

    return SPI->DR;
}


/**
 * @brief mainly looping
 */
void main(int argc, char **argv)
{
    uint8_t framecount = 0;
    uint8_t i = 0;

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
                char sbuf[16]; // am i big enuff?
#ifdef SPI_CONTROLLER
// periodic task is enabled at ~60 Hz ... the modulus provides a time reference of
// approximately 2 Hz at which time the master attempts to read a few bytes from SPI
            if ( ! ((framecount++) % 0x20) )
            {
                uint8_t spi_rx_bfr[8] = { 0 } ;

                uint8_t data;

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
// test uart
            }
#else
            if ( is_SPI_rx )
            {
                i = (i >= 0x30 && i < 126) ? i+1 : 0x30;
// tmp dump test SPI test data to UART
                disableInterrupts();
                sbuf[0] = '>';
                sbuf[1] = i;
                sbuf[2] = '.'; // first on is F2 or something like that, mask off the msb
                sbuf[3] = SPI_rxbuf[1];
                sbuf[4] = SPI_rxbuf[2];
                sbuf[5] = SPI_rxbuf[3];
                sbuf[6] = '.';
                sbuf[7] = 0;
                is_SPI_rx = 0;
                enableInterrupts();
                strcat(sbuf, "\r\n");
                UARTputs(sbuf);
             }
#endif // MASTER
        }
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
