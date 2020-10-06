/**
  ******************************************************************************
  * @file driver.h
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
#ifndef DRIVER_H
#define DRIVER_H


// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>


/*
 * defines
 */




typedef  uint16_t *  Global_ADC_Phase_t ;


/*
 * variables
 */


extern uint8_t Log_Level;


// how many ADCs can be saved in a single (60deg) commutation sector (how many PWM ISR per sector? depends on motor speed and PWM freq.)
#define ADC_PHASE_BUF_SZ  8  // make this power of 2 so modulus can be used 


extern Global_ADC_Phase_t Global_ADC_PhaseABC_ptr[];  // pointers to ADC phase buffers

extern uint8_t BackEMF_Sample_Index;


/*
 * prototypes
 */

void BLDC_Spd_inc(void); // should go away
void BLDC_Spd_dec(void);// should go away

uint16_t BLDC_PWMDC_Plus(void);
uint16_t BLDC_PWMDC_Minus(void);

void BLDC_Stop(void);
void BLDC_Step(void);
void BLDC_Update(void);


#endif // DRIVER_H
