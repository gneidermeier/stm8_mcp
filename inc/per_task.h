/**
  ******************************************************************************
  * @file per_task.h
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date Dec-2020
  ******************************************************************************
  */
#ifndef PER_TASK_H
#define PER_TASK_H

/* Includes ------------------------------------------------------------------*/

// app headers
#include "system.h" // platform specific delarations


/* Public function prototypes -----------------------------------------------*/

void Periodic_Task_Wake(void);

uint8_t Task_Ready(void);

void UI_Stop(void);

#endif // PER_TASK_H
