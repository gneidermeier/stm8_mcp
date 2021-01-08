/**
  ******************************************************************************
  * @file BLDC.h
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
#ifndef BLDC_H
#define BLDC_H

/* Includes ------------------------------------------------------------------*/
#include "system.h"

#ifdef UNIT_TEST
#include <stdint.h> // was supposed to go thru sytsem.h :(
#endif

/*
 * types
 */

/*
 * type of state machine
 */
typedef enum
{
    BLDC_FAULT,
    BLDC_OFF,
    BLDC_RAMPUP,
    BLDC_ON
} BLDC_STATE_T;


/*
 * prototypes
 */

BLDC_STATE_T get_bldc_state(void);
BLDC_STATE_T set_bldc_state( BLDC_STATE_T );

void set_commutation_period(uint16_t);
uint16_t get_commutation_period(void);

void BLDC_Spd_inc(void); // should go away
void BLDC_Spd_dec(void);// should go away

void BLDC_PWMDC_Plus(void);
void BLDC_PWMDC_Minus(void);

void BLDC_PWMDC_Set(uint16_t dc);

void BLDC_Stop(void);

void BLDC_Update(void);

#endif // BLDC_H
