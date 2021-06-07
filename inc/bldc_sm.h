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

/* macros --------------------------------------------------------------------*/
#define PWM_BL_STOP  U8_MAX


/* types ---------------------------------------------------------------------*/


/* prototypes ----------------------------------------------------------------*/

/**
 * @brief Accessor for state variable.
 *
 * @return state value
 */
typedef enum
{
    BL_NOT_RUNNING,
    BL_IS_RUNNING
} BL_RUNSTATE_t;

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

void BL_reset(void);

BL_RUNSTATE_t BL_get_state(void);
uint8_t BL_get_ct_mode(void);

/**
 * @brief Periodic state machine update.
 *
 * Called from ISR. Evaluate state transition conditions and determine new
 * state.
 *
 */
void BLDC_Update(void);

#endif // BLDC_H
