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


/*
 * (un)comment macro to set stm8 clock from 8Mhz or 16Mhz
 */
//#define CLOCK_16

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


#define LED  0  // STM8-Discovery board built-in LED on PIN D0


#endif // SYSTEM_H
