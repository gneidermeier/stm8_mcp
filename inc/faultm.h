/**
  ******************************************************************************
  * @file BLDC.h
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
#ifndef FAULTM_H
#define FAULTM_H

/* Includes ------------------------------------------------------------------*/
#include "system.h"

#ifdef UNIT_TEST
#include <stdint.h> // was supposed to go thru sytsem.h :(
#endif


/*
 * types
 */

// condition asserted to the fault status logic ... essentially boolean
typedef char faultm_assert_t; 

typedef enum
{
// set explicit integer values to use bit-positions in system-error word w/o 
// requiring any bit-shift/division to index bits in the array
    FAULT_0 = 1,
    FAULT_1 = 2,
    VOLTAGE_NG = 4,
    THROTTLE_HI = 8
} faultm_ID_t;

//#define NR_DEFINED_FAULTS  ( FAULT_ID_NR - 1 )

typedef uint8_t fault_status_reg_t; // fault status bitmap


/*
 * prototypes
 */

void Faultm_init(void);

void Faultm_upd(faultm_ID_t faultm_ID, faultm_assert_t tcondition);
void Faultm_set(faultm_ID_t faultm_ID);
void Faultm_clr(faultm_ID_t faultm_ID);

fault_status_reg_t Faultm_get_status(void);


#endif // FAULTM_H
