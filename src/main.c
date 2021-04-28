/**
  ******************************************************************************
  * @file    main.c
  * @brief   Application entry point
  * @author  Neidermeier
  * @version V2.0.4
  * @date     26-April-2021
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include "stm8s.h"
#include "stdio.h"

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

/**
  * @addtogroup UART1_Printf
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

/**
  * @brief  Mainly looping.
  * @param  None
  * @retval None
  */
void main(int argc, char **argv)
{
  char ans;

  MCU_Init();

  printf("\n\rProgram Startup.......\n\r");

  enableInterrupts(); // interrupts are globally disabled by default

  while (1)
  {
#if 0
    ans = getchar();
    printf("%c", ans);
#else
    if ( TRUE == Task_Ready() )
    {
        GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS);			
    }
//    Delay(0xFFFF);
#endif
  }
}

/**
  * @brief Retargets the C library printf function to the UART.
  * @param c Character to send
  * @retval char Character sent
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

  return (c);
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
  c = UART1_ReceiveData8();
  return (c);
}

#ifdef USE_FULL_ASSERT

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
#endif

/**
  * @}
  */
