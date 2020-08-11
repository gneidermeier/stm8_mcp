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
#else
  #define TIM2_PWM_PD    166   //  83uS 
#endif


// Using the TIM2 counter as time base for open-loop commutation time  
// arbitrary period (n * 125uS ^H^H^H 62.5uS) to PWM the user LED0
#define TIM2_COUNT_LED0_PD   80



#define LED  0


/*
 * Types
 */
typedef  enum {
  BLDC_OFF,
  BLDC_RAMPUP,
  BLDC_ON
} BLDC_STATE_T;


/*
 * variables
 */
extern u8 TaskRdy;     // flag for background task to sync w/ timer refrence


extern  uint16_t global_uDC;   // tmp
extern  uint16_t Manual_uDC;

extern uint16_t AnalogInputs[]; // at least ADC NR CHANNELS


/*
 * prototypes
 */
void BLDC_Spd_inc(void);
void BLDC_Spd_dec(void);
void BLDC_Stop(void);
void BLDC_Step(void);
void BLDC_Update(void);
void PWM_Set_DC(uint16_t uDC);


#endif // PARAMETER_H
