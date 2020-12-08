/**
  ******************************************************************************
  * @file periodic_task.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date Dec-2020
  ******************************************************************************
  */
#ifndef PER_TASK_H
#define PER_TASK_H

/* Includes ------------------------------------------------------------------*/
#include <string.h>

// app headers
#include "system.h" // platform specific delarations


/* Private defines -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/

extern uint8_t Log_Level;         // global log-level


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

int Task_Ready(void);


#endif // PER_TASK_H
