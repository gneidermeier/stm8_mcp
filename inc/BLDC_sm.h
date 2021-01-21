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
    BLDC_RESET = 0,
    BLDC_READY,
    BLDC_RAMPUP,
    BLDC_ON,
    BLDC_FAULT = 255 // numerical value irrelevant other than for display purpose
} BLDC_STATE_T;


/*
 * prototypes
 */

BLDC_STATE_T get_bldc_state(void);

uint16_t get_commutation_period(void);

void BLDC_Spd_inc(void);
void BLDC_Spd_dec(void);

void BLDC_PWMDC_Set(uint8_t dc);
uint16_t BLDC_PWMDC_Get(void);

void BLDC_Stop(void);

void BLDC_Update(void);

#endif // BLDC_H
