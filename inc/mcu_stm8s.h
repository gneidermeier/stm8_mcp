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



/* Public variables  ---------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/



/* function prototypes -----------------------------------------------*/

uint8_t SerialKeyPressed(char *key);

void UARTputs(char *message);

void MCU_Init(void);


#endif // MCU_STM8S