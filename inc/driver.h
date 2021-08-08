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


/*
 * Throttle control servo signal timings are measured with th Spektrum DX8 
 * transmitter and AR620 6-channel air receiver. With the defailt frame rate
 * setting, the rate is observed to be:
 *
 * The timer must be a 16-bit timer with the period set to maximum 0xFFFF and 
 * remains as a free running timer with a maximum time of:
 *
 *   1/16Mhz * prescaler * 0xFFFF = 4.1ms * 8 ==  == 0.0327675  (32 ms)
 *   1/16Mhz * prescaler = 1/16Mhz * 8 = 0.0000005 = 0.5uS/tick
 *
 *  The pulse period is measured on the scope at about 22 ms. Software observes 
 *  a count of  $AC80 (44160d), converting to time:
 * 
 *    44160 * 0.5uS/tick = 22.08 ms  
 *    1/22.08 ms = ~45Hz
 */
#define TCC_TICKS_PSEC       (0.5)
#define TCC_LOW_STIK         (1104.0 * (1.0 / TCC_TICKS_PSEC)) // $08A0
//#define TCC_M_ARMED        (1152.0 * (1.0 / TCC_TICKS_PSEC)) // $0900
//#define TCC_M_STOP         (1192.0 * (1.0 / TCC_TICKS_PSEC)) // $0950
//#define TCC_M_START        (1200.0 * (1.0 / TCC_TICKS_PSEC)) // $0960
#define TCC_THRTTLE_100PCNT  (1890.0 * (1.0 / TCC_TICKS_PSEC)) // $0EC4
#define TCC_FULL_STIK        (1904.0 * (1.0 / TCC_TICKS_PSEC)) // $0EE0
/*
 * With throttle proportional to pulse width, and the motor speed range (0%:100%)
 * (PWM-DC) corresponsds to {0:100%) throttle (MAX_THRUST 
 * servo pulse. M_START is at 10% of motor speed range, i.e. range of 
 *
 * (M_START:MAX_THRUST) would be 90% i.e (1890 - 1200) = 690
 *  10% = 690 ms / 9 = 76.7 ms 
 *
 * i.e. 0 RPM correspond to ~1124 ms.
 * 100% motor speed is signal range (1124:1890) = 767 mS
 * 1% motor speed range = 7.67 ms.
 * Speed % = (pulse width - TCC_0RPM) / (
*/
#define TCC_THRTTLE_10_PCNT  ( 690.0/9.0 + 0.5 ) // 76.7
#define TCC_THRTTLE_0PCNT    \
                   ( (1200.0 - TCC_THRTTLE_10_PCNT) * (1.0 / TCC_TICKS_PSEC) ) // 8C5

#define TCC_M_ARMED  TCC_THRTTLE_0PCNT

#define SPEED_PCNT_SCALE  16.0 


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

#endif // DRIVER_H
