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

/* Private defines -----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

void spi_write_data_8t( uint8_t *pBuffer, uint8_t WriterAddr);
void spi_write_data_1t( uint8_t bdata);
//uint8_t spi_tx_bfr[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0 } ;

/**
 * @brief mainly looping
 */
void main(int argc, char **argv)
{
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
          static uint8_t bdata = 0;
#ifdef SPI_MASTER
//        spi_write_data_8t(spi_tx_bfr, 0x40);//Random Address 0x40 for now.
        spi_write_data_1t( bdata++ );
#else
//        spi_write_data_8t("SPISLAVE", 0x40);//Random Address 0x40 for now.
//        spi_write_data_1t( bdata-- );
#endif
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
