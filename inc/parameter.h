/**
  ******************************************************************************
  * @file parameter.h
  * @brief This file contains the parameters for ADC.
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

// Setting up TIM2, presently to use 4kHz PWM frequency (or maybe even 6 or 7 kHz ?)

// Using TIM2 prescale value of 1, so period TIM2 == period fMaster
// @2Mhz, fMASTER period ==  0.5uS
//  1 / 2Mhz = 0.5uS

// 1/4kHz = 250uS
// 250uS / 0.5uS = 500 counts

//  0.5uS * 500 counts = 250uS
//  1 / 250uS = 4000Hz

#define TIM2_PWM_PD (500 - 1)

// Using TIM2 to count up to 20mS time base 
// 20mS / 250uS = 80
#define TIM2_T20_MS 80

#define N_PHASES  3 


#define LED  0
