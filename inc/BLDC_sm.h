/**
  ******************************************************************************
  * @file BLDC_sm.h
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

/* types ------------------------------------------------------------------*/

/**
 * @brief Typedef for state machine variable.
 */
typedef enum
{
    BLDC_RESET = 0,
    BLDC_READY,
    BLDC_RAMPUP,
    BLDC_RUNNING,
    BLDC_FAULT = 255 // numerical value irrelevant other than for display purpose
} BLDC_STATE_T;

/** @deprecated */
//#define BLDC_ON BLDC_RUNNING // tmp


/* prototypes ------------------------------------------------------------------*/

/**
 * @brief Accessor for state variable.
 *
 * @return state value
 */
BLDC_STATE_T get_bldc_state(void);

/**
  * @brief Accessor for commutation period.
  *
  * @return commutation period
  */
uint16_t get_commutation_period(void);

void BLDC_Spd_inc(void);
void BLDC_Spd_dec(void);

void BLDC_PWMDC_Set(uint8_t dc);
uint16_t BLDC_PWMDC_Get(void);

void BLDC_Stop(void);

/**
 * @brief Periodic state machine update.
 *
 * Called from ISR. Evaluate state transition conditions and determine new
 * state.
 *
 */
void BLDC_Update(void);

#endif // BLDC_H
