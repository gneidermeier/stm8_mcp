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


/* defines -----------------------------------------------------------*/





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

uint16_t Driver_get_motor_spd_pcnt(void);
uint16_t Driver_get_pulse_dur(void);
uint16_t Driver_get_pulse_perd(void);
uint16_t Driver_get_servo_position_counts(void);

void Driver_Get_Rx_It(void);

#endif // DRIVER_H
