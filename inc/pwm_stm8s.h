/**
  ******************************************************************************
  * @file pwm_stm8s.h
  * @brief BLDC motor control - PWM, STM8s specific (TIM2, TIM1 alternate w/ bd.mods.)
  * @author Neidermeier
  * @version
  * @date Sept-2020
  ******************************************************************************
  */
#ifndef PWM_STM_S_H
#define PWM_STM_S_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Public defines -----------------------------------------------------------*/


/* Public types -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/
void PWM_PhA_Disable(void);
void PWM_PhB_Disable(void);
void PWM_PhC_Disable(void);

void PWM_PhA_Enable(uint16_t dc);
void PWM_PhB_Enable(uint16_t dc);
void PWM_PhC_Enable(uint16_t dc);

#endif // PWM_STM_S_H
