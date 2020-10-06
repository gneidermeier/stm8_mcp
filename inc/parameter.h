/**
  ******************************************************************************
  * @file parameter.h
  * @brief  
  * @author
  * @version 
  * @date    
  ******************************************************************************
  *
  * BLAH BLAH BLAH
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef PARAMETER_H
#define PARAMETER_H


// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>

//#define CLOCK_16

#define PWM_8K




// 1/8000  = 0.000125 = 12.5 * 10^(-5)
// 1/12000 = 0.000083 = 8.3 * 10^(-5) 

// With TIM2 prescale value of 1, period TIM2 == period fMaster
// @8Mhz, fMASTER period == 0.000000125 S
// fMASTER * TIM1_PS = 0.125us * 4 = 0.5us

// @8k:
//  0.000125 / 0.5 us = 250 counts

// @12k:
//  0.000083 / 0.5 us  = 166.67 counts


#ifdef PWM_8K
  #define TIM2_PWM_PD    250   // 125uS 
#else // 12kHz
  #define TIM2_PWM_PD    166   //  83uS 
#endif


#define LED  0

extern uint8_t TaskRdy;     // flag for background task to sync w/ timer refrence





#endif // PARAMETER_H
