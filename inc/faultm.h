/**
  ******************************************************************************
  * @file faultm.h
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

/**
 * @brief condition asserted to the fault status logic ... essentially boolean
 */
typedef char faultm_assert_t;

/**
 * @brief integer enumeration of all defined system faults
 *
 * Relies on assigning explicit integer values to use bit-positions in the
 * system-error word - the system word is expected to fit in 8-bits.
 */
typedef enum
{
    FAULT_0 = 1,
    FAULT_1 = 2,
    VOLTAGE_NG = 4,
    THROTTLE_HI = 8
} faultm_ID_t;

/**
 * @brief Fault status system word
 */
typedef uint8_t fault_status_reg_t; // fault status bitmap


/*
 * prototypes
 */

void Faultm_init(void);

void Faultm_upd(faultm_ID_t faultm_ID, faultm_assert_t tcondition);
void Faultm_set(faultm_ID_t faultm_ID);
//void Faultm_clr(faultm_ID_t faultm_ID); // unimplemented

fault_status_reg_t Faultm_get_status(void);


#endif // FAULTM_H
