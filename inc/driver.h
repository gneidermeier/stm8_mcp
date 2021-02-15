/**
  ******************************************************************************
  * @file driver.h
  * @brief Support functions for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2020
  ******************************************************************************
  *
  * BLAH BLAH BLAH
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef DRIVER_H
#define DRIVER_H

/* Includes ------------------------------------------------------------------*/

#include "system.h"


/* Private defines -----------------------------------------------------------*/

// divider: 33k/18k
//  18/(18+33)=0.35
// 0.35 * 14.1v = 4.98
// 4.98 / 2 = 2.48v ........... 1/2 Vdc in proportion to the resister divider
//  2.48v/5v =  x counts / 1024 ocunts so 1/2 Vdc is equivalent to x counts ...
//   x = 1024 * 2.48/5 = 509   (0x01FD)
#define DC_HALF_REF         0x01FD



/* Private types -----------------------------------------------------------*/

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

void Driver_Step(void);
void Driver_Update(void);

uint16_t Driver_Get_ADC(void);
uint16_t Driver_Get_Back_EMF_Avg(void);

void Driver_on_PWM_edge(void);
void Driver_on_ADC_conv(void);

void Driver_on_capture_rise(void);
void Driver_on_capture_fall(void);

uint16_t Driver_get_pulse_perd(void);
uint16_t Driver_get_pulse_dur(void);


#endif // DRIVER_H
