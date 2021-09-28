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

#include "pwm_stm8s.h"
#include "mdata.h"

/* defines -------------------------------------------------------------------*/
/*
 * 100% / 250 counts == 0.4% per count
 * 250 * .004 == 1
 */
#define PWM_PERCENT_PER_COUNT_250  0.004 //  ( 100.0 / MDATA_TBL_SIZE ) // .4% i.e. .004

// table size originated from 250 step PWM confiugration
#define MSPEED_PCNT_INCREM_STEP   ( PWM_PERIOD_COUNTS * PWM_PERCENT_PER_COUNT_250 )

#define RX_BUFFER_SIZE  16  //how big should this be?


/* types --------------------------------------------------------------------*/



/* prototypes ---------------------------------------------------------------*/

void Driver_Step(void);
void Driver_Update(void);

uint16_t Driver_Get_ADC(void);

void Driver_on_PWM_edge(void);
void Driver_on_ADC_conv(void);

void Driver_on_capture_rise(void);
void Driver_on_capture_fall(void);

void Driver_set_pulse_dur(uint16_t);
uint16_t Driver_get_pulse_dur(void);

uint16_t Driver_get_motor_spd_pcnt(void);
uint16_t Driver_get_pulse_dur(void);
uint16_t Driver_get_pulse_perd(void);
uint16_t Driver_get_servo_position_counts(void);

void Driver_Get_Rx_It(void);
uint8_t Driver_Return_Rx_Buffer(void);
void Driver_Clear_Rx_Buffer_Element(uint8_t);

#endif // DRIVER_H
