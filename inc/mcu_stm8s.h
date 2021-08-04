/**
  ******************************************************************************
  * @file mcu_stm8s.h
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */
#ifndef MCU_STM8S
#define MCU_STM8S

/* Includes ------------------------------------------------------------------*/

// app headers
#include "system.h" // platform specific delarations

/* function prototypes -------------------------------------------------------*/

uint8_t SerialKeyPressed(char *key);

void MCU_Init(void);

void MCU_set_comm_timer(uint16_t);


#endif // MCU_STM8S
