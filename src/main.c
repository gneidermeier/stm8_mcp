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
#include <string.h> // strcat

// app headers
#include "mcu_stm8s.h"
#include "bldc_sm.h"
#include "per_task.h"

#ifndef SPI_CONTROLLER
  #include "spi_stm8s.h"
#endif


#ifdef _SDCC_
// Interrupt vectors must be implemented in the same file that implements main()
/*
If you have multiple source files in your project, interrupt service routines can be present in any of them, but a prototype of the isr MUST be present or included in the file that contains the function main.
*/
//  #include "stm8s_it.h" // not sure if this works ... enable stm8s_it.c to be built in the build config
#include "stm8s_it.c"
#endif


/* Private defines -----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/


/**
 * @brief mainly looping
 */
void main(int argc, char **argv)
{
#ifndef SPI_CONTROLLER
    static const uint8_t tx_buf[RX_BUF_SZ] = "0123456789ABCDEF";
#endif
    uint8_t linec = 0;
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
// this modulus provides a time reference of approximately 2 Hz (debug/test/dev/usage)
            if ( ! ((framecount++) % 0x20) )
            {
            }
        }

#ifndef SPI_CONTROLLER
        if (-1 != SPI_read_write_b(tx_buf, 0xA5, TIME_OUT_0) )
        {
            char sbuf[16]; // am i big enuff?
            // tmp dump test SPI test data to UART
            linec = (uint8_t)(linec < 126 ? linec++ : 0x30);
            sbuf[0] = '>';
            sbuf[1] = linec++;
            sbuf[2] = isprint( (int)spi_rx_buf[0] ) ? spi_rx_buf[0] : '.' ;
            sbuf[3] = isprint( (int)spi_rx_buf[1] ) ? spi_rx_buf[1] : '.' ;
            sbuf[4] = isprint( (int)spi_rx_buf[2] ) ? spi_rx_buf[2] : '.' ;
            sbuf[5] = isprint( (int)spi_rx_buf[3] ) ? spi_rx_buf[3] : '.' ;
            sbuf[6] = isprint( (int)spi_rx_buf[4] ) ? spi_rx_buf[4] : '.' ;
            sbuf[7] = 0;
            strcat(sbuf, "\r\n");
            UARTputs(sbuf);
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
