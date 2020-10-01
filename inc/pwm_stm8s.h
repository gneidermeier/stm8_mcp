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

#if 1
/*
 * bleh yuk erg
 */
// PD4 set LO 
#define PWM_PhA_OUTP_LO( _ARG_ ) \
    GPIOD->ODR &=  ~(1<<4);      \
    GPIOD->DDR |=  (1<<4);      \
    GPIOD->CR1 |=  (1<<4);

// PD3 set LO
#define PWM_PhB_OUTP_LO( _ARG_ ) \
    GPIOD->ODR &=  ~(1<<3);      \
    GPIOD->DDR |=  (1<<3);      \
    GPIOD->CR1 |=  (1<<3);

// PA3 set LO
#define PWM_PhC_OUTP_LO( _ARG_ ) \
    GPIOA->ODR &=  ~(1<<3);      \
    GPIOA->DDR |=  (1<<3);      \
    GPIOA->CR1 |=  (1<<3);

#else
// TIM1 PWM Chnnels
//        GPIOC->ODR &=  ~(1<<2);  // PC2 set LO

//        GPIOC->ODR &=  ~(1<<3);  // PC3 set LO

//        GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
#endif


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
