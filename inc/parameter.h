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

//set prescaler with "(period - 1)" (see datasheet)
//#define TIM2_PWM_PD (125 - 0)    // 16k
#define TIM2_PWM_PD (250 - 0)  //  8k
//#define TIM2_PWM_PD (500 - 0)  //  4k
//#define TIM2_PWM_PD (1000 - 0) //  2k
//#define TIM2_PWM_PD (2000 - 0) //  1kHz ... test 290k*470pf -> Tau==136uSec

// Using TIM2 to count up to 20mS time base 
// 20mS / 250uS = 80

// 1/50Hz = 0.020 S
// 1/250uS = 4000Hz
// 20mS / 250uS = 20mS * 1/250uS = (1/50) * 4000 = 80

#define TIM2_T20_MS  (4000 / 50) // 80

// nbr of steps required to commutate 3 phase 
#define N_CSTEPS  3  // 6 ... 


#define LED  0
