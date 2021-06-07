/**
  ******************************************************************************
  * @file system.h
  * @brief
  * @author
  * @version
  * @date
  ******************************************************************************
  *
  * Platform specific system definitions and includes.
  * Implement this header file for individual build environment.

  ******************************************************************************
  */
#ifndef SYSTEM_H
#define SYSTEM_H

// horrors of unit testing
#ifndef UNIT_TEST
// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>
#endif // UNIT_TEST


// advise enabling this, so long as it is working (not triggering fault positives)
#define UNDERVOLTAGE_FAULT_ENABLED

// STM8 timers for control/pwm task
#define MCP_CTRL_TIMER_1  (1)
#define MCP_CTRL_TIMER_2  (1)
// STM8 timers for commutation step
#define MCP_COMM_TIMER_1  (1)
#define MCP_COMM_TIMER_3  (3)
/**
 * the STM8 variant is defined in the project file, along with the appropriate 
 * compiler settings for the particular MCU (memory model etc.)
 * Here is defined the particular platform, where the particular MCU and pin
 * routing determines how the peripherals and GPIO pins are allocated.
 */
#if defined ( S105_DEV )
/*
 * S105 Dev Board only has TIM2 Ch3 on Alt pin mapping, therefore, TIM1 drives  
 * PWM/control, while either TIM2 or TIM3 can driver commutation step as they 
 * are both 16-bit timers. TIM2 remains available for other purpose (servo input?).
 */
  #define CONTRLLR_TIMER  MCP_CTRL_TIMER_1
  #define COMMSTEP_TIMER  MCP_COMM_TIMER_3

  #define LED_GPIO_PORT    GPIOE
  #define LED_GPIO_PIN     GPIO_PIN_5

  #define SERVO_GPIO_PORT  GPIOD
  #define SERVO_GPIO_PIN   GPIO_PIN_4

  #define HAS_SERVO_INPUT
  #define SPI_ENABLED

#elif defined ( S105_DISCOVERY )
/*
 * S105 Discovery board can't use TIM1 for PWM (unless solder bridges connecting the
 * touch sensor are removed). Therefore, TIM2 drives PWM/controller, and TIM3 to
 * drive the commutation step, leaving TIM1 unnused (TIM1 input capture/compare 
 * might be used capture servo signal from R/C radio or flight controller).
 */
  #define CONTRLLR_TIMER  MCP_CTRL_TIMER_2
  #define COMMSTEP_TIMER  MCP_COMM_TIMER_3

  #define LED_GPIO_PORT    GPIOD
  #define LED_GPIO_PIN     GPIO_PIN_0

  #define SERVO_GPIO_PORT  GPIOC
  #define SERVO_GPIO_PIN   GPIO_PIN_4

  #define SPI_ENABLED
//  #define HAS_SERVO_INPUT // 6/21 ... GN: haven't tested this one for some time ...

#else // S003_DEV ?
/*
 * s003 does not have TIM3. TIM2 drives PWM/control, TIM1 drives commutation step.
 * Not likely to use the 8k flash part, code barely or does not fit.
 */
  #define CONTRLLR_TIMER  MCP_CTRL_TIMER_2
  #define COMMSTEP_TIMER  MCP_COMM_TIMER_1

  #define LED_GPIO_PORT    GPIOB
  #define LED_GPIO_PIN     GPIO_PIN_5

// invalid .. no timer avilable ?
  #define SERVO_GPIO_PORT  (GPIO_TypeDef *)-1
  #define SERVO_GPIO_PIN   (uint8_t)-1

//  #define HAS_SERVO_INPUT // no timer available?
//  #define SPI_ENABLED     // can't fit SPI in 8k
#endif

#if defined( SPI_ENABLED )
  #define SPI_CONTROLLER
#endif

#define SPI_RX_BUF_SZ  16 // 256 // tmp


/*
 * (un)comment macro to set stm8 clock from 8Mhz or 16Mhz
 */
#define CLOCK_16

/*
 * The base system rate multiplier
 * Starting value was 2 which means nothing other than startingrelative to
 * factoring out a power-of-two factor from certain time related structures
 * aligned with this rate such as fault manager and control ramp timing
 */
#define SYS_RATE_MULT  4  // periodic task multipler constant term (@ 1.024 ms)


// bl_sm and bg_task both derived from TIM4 and invoked thru the Driver ISR
// handlers ... but could end up being invoked at sep. subrates?
//#define PERT_RATEM  SYS_RATE_MULT  // periodic task rate multiplier

#define CTRL_RATEM  SYS_RATE_MULT  // control task rate multiplier


// the timer prescalar is to show that fixed timing data must somehow factor in
// the timer rate - halving the prescalar to make timer 2x faster means timing
// periods are 2x duration relative to the previous scalar of 1

#define CTIME_SCALAR 2


/*
 * (un)comment macro to set PWM 8 Khz or ?
 */
#define PWM_8K


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
  #define TIM2_PWM_PD    250   // 125uS
#else // 12kHz
  #define TIM2_PWM_PD    166   //  83uS
#endif

#define PWM_100PCNT  TIM2_PWM_PD


#endif // SYSTEM_H
