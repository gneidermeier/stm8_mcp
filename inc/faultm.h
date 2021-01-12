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
 *
 */
#define FAULT_SET    1
#define FAULT_CLEAR  0

/*
 * types
 */
typedef int FAULT_STATUS_t;

/*
 * prototypes
 */

void Faultm_init(void);

FAULT_STATUS_t Faultm_update(void);


#endif // BLDC_H
