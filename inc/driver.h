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

#include "system.h"


/*
 * defines
 */


/*
 * types
 */
// motor running-cycle state machine
typedef enum
{
    BLDC_OFF,
    BLDC_RAMPUP,
    BLDC_ON
} BLDC_STATE_T;


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

void set_commutation_period(uint16_t);
uint16_t get_commutation_period(void);

BLDC_STATE_T get_bldc_state(void);
BLDC_STATE_T set_bldc_state( BLDC_STATE_T );

int get_op_mode(void);
void set_op_mode(int mode);


#endif // DRIVER_H
