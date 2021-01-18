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
    VOLTAGE_NG = 0,
    THROTTLE_HI = 1,
    FAULT_ID_NR
} faultm_ID_t;

//#define NR_DEFINED_FAULTS  ( FAULT_ID_NR - 1 )

typedef uint8_t fault_status_reg_t; // fault status bitmap


/*
 * prototypes
 */

void Faultm_init(void);
void Faultm_setf(faultm_ID_t faultm_ID, faultm_assert_t tcondition);
fault_status_reg_t Faultm_get_status(void);


#endif // FAULTM_H
