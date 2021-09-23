/**
  ******************************************************************************
  * @file    main.c
  * @brief   Application entry point
  * @author  Neidermeier
  * @version V2.0.4
  * @date     26-April-2021
  ******************************************************************************
  */
/**
 * \defgroup per_task Periodic Task
 * @brief Background task / periodic task
 * @{
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <ctype.h> // isprint

// app headers
#include "mcu_stm8s.h"
#include "bldc_sm.h"
#include "per_task.h"


#ifdef _SDCC_
// Interrupt vectors must be implemented in the same file that implements main()
/*
If you have multiple source files in your project, interrupt service routines
can be present in any of them, but a prototype of the isr MUST be present or
included in the file that contains the function main.
*/
#include "stm8s_it.c"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Mainly looping.
  */
void main(int argc, char **argv)
{
  uint8_t linec = 0;
  uint8_t framecount = 0;
  uint8_t i = 0;
  (void) argc;
  (void) argv;

  MCU_Init();

  BL_reset();
  BL_set_opstate( BL_ARMING );  // set the initial control-state 

  printf("\n\rProgram Startup (%d)\n\r", (uint16_t)BL_get_opstate() );

  enableInterrupts(); // interrupts are globally disabled by default

  while(1)
  {
#if 0 //  TEST DEV ONLY: manual adjustment of commutation cycle time
//  button input either button would transition from OFF->RAMP
    if (! (( GPIOA->IDR)&(1<<4)))
    {
//            while( ! (( GPIOA->IDR)&(1<<4)) ); // no concern for debounce for a stop switch
      disableInterrupts();
      UI_Stop();
      enableInterrupts();
    }
#endif
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
  } // while 1
}

#ifdef USE_FULL_ASSERT
/** @cond */
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
/** @endcond */
#endif

/**
  * @}
  */
