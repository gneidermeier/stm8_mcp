/**
  ******************************************************************************
  * @file pwm_stm8s.h
  * @brief BLDC motor control - PWM, STM8s specific (TIM2, TIM1 alternate w/ bd.mods.)
  * @author Neidermeier
  * @version
  * @date Sept-2020
  ******************************************************************************
  */
#ifndef PWM_STM_S_H
#define PWM_STM_S_H

/* Includes ------------------------------------------------------------------*/

// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>
#include "system.h" // system/build configuration

/* Public defines -----------------------------------------------------------*/

/*
 * (un)comment macro to set PWM 8 Khz or ?
 */
//#define PWM_8K


// 1/8000  = 0.000125 = 12.5 * 10^(-5)
// 1/12000 = 0.000083 = 8.3 * 10^(-5)

// With TIM2 prescale value of 1, period TIM2 == period fMaster
// @8Mhz, fMASTER period == 0.000000125 S
// fMASTER * TIM1_PS = 0.125us * 4 = 0.5us

// @8k:
//  0.000125 / 0.5 us = 250 counts

// @12k:
//  0.000083 / 0.5 us  = 166.67 counts

#ifdef PWM_8K
  #define PWM_PERIOD_COUNTS   250 // 1/16 Mhz * PS * 2500 = 0.000125 sec
#else // 
  #define PWM_PERIOD_COUNTS  1024
#endif


/**
 * @brief Compute PWM timer counts from percent duty-cycle
 * @param Percent duty-cycle, range (0:100.0)
 * @return PWM timer counts 
 */
#define PWM_GET_PULSE_COUNTS( _PCNT_ ) \
    (uint16_t)( ( _PCNT_ * (PWM_PERIOD_COUNTS / 4.0) ) / (100.0 / 4.0) )


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
 *
 * Reference:
 *  https://www.kdedirect.com/blogs/news/understanding-throttle-calibration-esc-deadbands-and-pwm
 *
 *  Motor Arming pulse: 1100탎
 *  Motor Spinning pulse: 1125탎
 *  Motor Stopping pulse: 1110탎
 *  Max thrust output: 1915탎
 *  Full Stick: 1940탎
 */

#define TCC_TICK_TIME_MSEC   (0.5)
#define TCC_TIME_ARMING      (uint16_t)(1100.0 * (1.0 / TCC_TICK_TIME_MSEC))
#define TCC_TIME_MAX_THRUST  (uint16_t)(1900.0 * (1.0 / TCC_TICK_TIME_MSEC))

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
#define TCC_LOW_STIK TCC_TIME_ARMING
#define TCC_FULL_STIK TCC_TIME_MAX_THRUST
#define TCC_THRTTLE_RANGE  ( TCC_FULL_STIK - TCC_LOW_STIK )


/**
 * @brief integer scale factor for pwm percent
 * @details speed percent is not used for setting PWM but rather for 
 *  calculations involving percent motor speed (0.5%/bit precision)
 * e.g. 
 *  %DC = (PWM_DC_SCALE * PWM_PULSE_CNT) / PWM_PULSE_100_PCNT
 *
 * As u16, and using 1600 for 100% pulse time (period)
 *   65535 / 1600 = 40 
 *
 * Therefore use p-o-2 scale of 2^5
 */
#define PWM_MSPEED_PCNT_SCALE  2.0

/**
 * @brief convert raw servo position counts to integer percent
 *
 * @details 
 *   100% * SERVO_POSN / SERVO_RANGE
 *
 *  factor of 2 is factored out of num. and denomnator terms to avoid overflow of 16-bits
 *
 *
 * @param  SERVO_POSITION_COUNTS, range (0:3600)
 * 
 */
#define PWM_MSPEED_PERCENT( _SERVO_POSITION_COUNTS_ )  \
  ( (100.0 / 2 ) * ( _SERVO_POSITION_COUNTS_ ) / ( TCC_THRTTLE_RANGE / 2 ) )

/**
 * The MCU drives 3 GPIO as output to IR2104 /SD pins. There is no significance 
 * to their pin assignment or pin configuration other than setting the internal
 * pullup (not open-collector) 
 */
#if defined ( S105_DEV )
// leave D3 and D4 available for servo pulse input capture (TIM2 CH1 and CH2)
  #define SDa_SD_PIN  GPIO_PIN_0  // D0
  #define SDb_SD_PIN  GPIO_PIN_2  // D2
  #define SDc_SD_PIN  GPIO_PIN_1  // A1

  #define SDa_SD_PORT  GPIOD  // D0
  #define SDb_SD_PORT  GPIOD  // D2
  #define SDc_SD_PORT  GPIOA  // A1

#elif defined ( S105_DISCOVERY )

  #define SDa_SD_PIN  GPIO_PIN_2  // D2
  #define SDb_SD_PIN  GPIO_PIN_0  // E0
  #define SDc_SD_PIN  GPIO_PIN_5  // A5

  #define SDa_SD_PORT  GPIOD
  #define SDb_SD_PORT  GPIOE
  #define SDc_SD_PORT  GPIOA

#elif defined ( S003_DEV ) // SPI not enabled, three drive pins useable for /SD
  #define SDa_SD_PIN  GPIO_PIN_7 // C7
  #define SDb_SD_PIN  GPIO_PIN_6 // C6
  #define SDc_SD_PIN  GPIO_PIN_5 // C5

  #define SDa_SD_PORT  GPIOC
  #define SDb_SD_PORT  GPIOC
  #define SDc_SD_PORT  GPIOC
#endif


#if defined( S105_DISCOVERY ) || defined( S003_DEV )
/**
 * Using TIM2 as PWM ... pins are same for 2 platforms
 */
  #define SDa_PWM_PIN  GPIO_PIN_4 // D4
  #define SDb_PWM_PIN  GPIO_PIN_3 // D3
  #define SDc_PWM_PIN  GPIO_PIN_3 // A3

  #define SDa_PWM_PORT  GPIOD
  #define SDb_PWM_PORT  GPIOD
  #define SDc_PWM_PORT  GPIOA

#elif defined( S105_DEV )
/**
 * TIM2 not available, uses TIM1
 */
  #define SDa_PWM_PIN  GPIO_PIN_2 // C2
  #define SDb_PWM_PIN  GPIO_PIN_3 // C3
  #define SDc_PWM_PIN  GPIO_PIN_4 // C4

  #define SDa_PWM_PORT  GPIOC
  #define SDb_PWM_PORT  GPIOC
  #define SDc_PWM_PORT  GPIOC
#endif

// PD4 set LO
#define PWM_PhA_OUTP_LO( )                              \
    SDc_PWM_PORT->ODR &= (uint8_t) ( ~SDa_PWM_PIN );    \
    SDc_PWM_PORT->DDR |=  SDa_PWM_PIN;                   \
    SDc_PWM_PORT->CR1 |=  SDa_PWM_PIN;

// PD3 set LO
#define PWM_PhB_OUTP_LO( )                              \
    SDc_PWM_PORT->ODR &= (uint8_t) ( ~SDb_PWM_PIN );    \
    SDc_PWM_PORT->DDR |=  SDb_PWM_PIN;                   \
    SDc_PWM_PORT->CR1 |=  SDb_PWM_PIN;

// PA3 set LO
#define PWM_PhC_OUTP_LO( )                              \
    SDc_PWM_PORT->ODR &= (uint8_t) ( ~SDc_PWM_PIN );    \
    SDc_PWM_PORT->DDR |=  SDc_PWM_PIN;                   \
    SDc_PWM_PORT->CR1 |=  SDc_PWM_PIN;


/**
 * Phase enable (/SD input pin on IR2104)
 */
#define PWM_PhA_HB_ENABLE( ) \
    SDa_SD_PORT->ODR |=   SDa_SD_PIN;

#define PWM_PhB_HB_ENABLE( ) \
    SDb_SD_PORT->ODR |=   SDb_SD_PIN;

#define PWM_PhC_HB_ENABLE( ) \
    SDc_SD_PORT->ODR |=   SDc_SD_PIN;

/**
 * Phase disable (/SD input pin on IR2104)
 */
// casts applied in order to quash warnings (bit inversion causes sign extension to to 16-bit)
#define PWM_PhA_HB_DISABLE( ) \
    SDa_SD_PORT->ODR &=  (uint8_t) ( ~SDa_SD_PIN );

#define PWM_PhB_HB_DISABLE( ) \
    SDb_SD_PORT->ODR &=  (uint8_t) ( ~SDb_SD_PIN );

#define PWM_PhC_HB_DISABLE( ) \
    SDc_SD_PORT->ODR &=  (uint8_t) ( ~SDc_SD_PIN );


/* Public types -------------------------------------------------------------*/
/**
 * @brief  Generic PWM channel type.
 */
typedef  TIM2_Channel_TypeDef PWM_Channel_Typedef ;


/* Public variables ---------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/

void All_phase_stop(void);

void PWM_PhA_Disable(void);
void PWM_PhB_Disable(void);
void PWM_PhC_Disable(void);

void PWM_PhA_Enable(void);
void PWM_PhB_Enable(void);
void PWM_PhC_Enable(void);

void PWM_set_dutycycle(uint16_t);

void PWM_setup(void);

uint16_t PWM_get_motor_spd_pcnt(uint16_t, uint16_t);
uint16_t PWM_get_servo_position_counts( uint16_t );

#endif // PWM_STM_S_H
