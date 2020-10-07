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


/*
 * types
 */


/*
 * variables
 */


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
