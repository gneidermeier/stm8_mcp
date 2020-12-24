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

/*
 * mainly looping
 */
main()
{
    MCU_Init();

    BLDC_Stop();

    enableInterrupts(); // interrupts are globally disabled by default

    UARTputs("\033c"); // sends ANSI code to  clear the serial terminal
    UARTputs("Program Startup....... \r\n");

    while(1)
    {
//  button input either button would transition from OFF->RAMP
        if (! (( GPIOA->IDR)&(1<<4)))
        {
//            while( ! (( GPIOA->IDR)&(1<<4)) ); // no concern for debounce for a stop switch
            disableInterrupts();
            BLDC_Stop();
            enableInterrupts();
            // reset line couunter of the serial-port logger
        }

        if (! (( GPIOA->IDR)&(1<<6)))
        {
            while( ! (( GPIOA->IDR)&(1<<6)) ); // wait for debounce (sorta works)
#if 1 //  TEST DEV ONLY: manual adjustment of commutation cycle time
            disableInterrupts();
            BLDC_Spd_inc();
            enableInterrupts();
#endif
        }

        if ( ! (( GPIOE->IDR)&(1<<5)))
        {
            while( ! (( GPIOE->IDR)&(1<<5)) ) {;} // wait for debounce (sorta works)
#if 1 //  TEST DEV ONLY: manual adjustment of commutation cycle time
            disableInterrupts();
            BLDC_Spd_dec();
            enableInterrupts();
#endif
        }

// while( FALSE == TaskRdy )
        if ( FALSE == Task_Ready() )  // idk .. don't block here in case there were actually some background tasks to do
        {
            nop();
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
