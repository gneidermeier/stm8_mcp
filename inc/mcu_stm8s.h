/**
  ******************************************************************************
  * @file mcu_stm8s.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */
#ifndef MCU_STM8S
#define MCU_STM8S

/* Includes ------------------------------------------------------------------*/

// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
//#include <stm8s.h>

// app headers
#include "system.h" // platform specific delarations


/* Private defines -----------------------------------------------------------*/

// reference: SPL UART example project

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

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* function prototypes -----------------------------------------------*/

uint8_t SerialKeyPressed(char *key);

void UARTputs(char *message);

void MCU_Init(void);

void MCU_comm_time_cfg(uint16_t);


#endif // MCU_STM8S
