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


/*
 * defines
 */

// Setting up TIM2, presently to use 4kHz PWM frequency (or maybe even 6 or 7 kHz ?)

// Using TIM2 prescale value of 1, so period TIM2 == period fMaster
// @2Mhz, fMASTER period ==  0.5uS
//  1 / 2Mhz = 0.5uS

// 1/8kHz = 125uS
// 125uS / 0.5uS = 250 counts

//  0.5uS * 250 counts = 125uS
//  1 / 125uS = 8kHz

//set prescaler with "(period - 1)" (see datasheet)
//#define TIM2_PWM_PD         (125 - 0)  // 16k
#define TIM2_PWM_PD           (126 - 0)  //  .... might as well make the math divisible by 2 see periodic task
//#define TIM2_PWM_PD         (250 - 0)  //  8k


// Using the TIM2 counter as time base for open-loop commutation time  
// arbitrary period (n * 125uS ^H^H^H 62.5uS) to PWM the user LED0
#define TIM2_COUNT_LED0_PD   80



#define LED  0


/*
 * variables
 */
extern u8 Duty_cycle_pcnt_LED0; // for test output on LED0 (builtin STM8-discover)
extern u8 TaskRdy;     // flag for background task to sync w/ timer refrence
extern u8 PWM_Is_Active;

/*
 * prototypes
 */
void PWM_Config(uint16_t uDC);
void set_outputs(void);


#endif // PARAMETER_H
