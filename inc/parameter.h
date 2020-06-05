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
#define BLDC_TIM1_TEST

// Using TIM2 prescale value of 1, so period TIM2 == period fMaster
// @2Mhz, fMASTER period ==  0.5uS
//  1 / 2Mhz = 0.5uS

// 1/8kHz = 125uS
// 125uS / 0.5uS = 250 counts

//  0.5uS * 250 counts = 125uS
//  1 / 125uS = 8kHz

//set prescaler with "(period - 1)" (see datasheet)
#ifdef CLOCK_16
 #define TIM2_PWM_PD           (250 - 0)  // @16k -> 125uS

#else
//#define TIM2_PWM_PD         (125 - 0)   // @8k -> 125uS
 #define TIM2_PWM_PD           (126 - 0)  //  .... might as well make the math divisible by 2 see periodic task#endif
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
