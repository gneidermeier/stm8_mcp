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
#include "stm8s.h"

/* Public defines -----------------------------------------------------------*/

#if 1
/*
 * bleh yuk erg
 */
// PD4 set LO 
#define PWM_PhA_OUTP_LO( _ARG_ ) \
    GPIOD->ODR &=  ~(1<<4);      \
    GPIOD->DDR |=  (1<<4);      \
    GPIOD->CR1 |=  (1<<4);

// PD3 set LO
#define PWM_PhB_OUTP_LO( _ARG_ ) \
    GPIOD->ODR &=  ~(1<<3);      \
    GPIOD->DDR |=  (1<<3);      \
    GPIOD->CR1 |=  (1<<3);

// PA3 set LO
#define PWM_PhC_OUTP_LO( _ARG_ ) \
    GPIOA->ODR &=  ~(1<<3);      \
    GPIOA->DDR |=  (1<<3);      \
    GPIOA->CR1 |=  (1<<3);

/*
 * Half-bridge enables ... specific to certain pin 
 * as the device specific ... IR2104 is the /SD pin
 *
 * Combine enable/disable using _ARG_ ?
 */
 // D2 D1 A5

#define PWM_PhA_HB_ENABLE( _ARG_ ) \
    GPIOD->ODR |=   (1<<2);  // set /SD A

#define PWM_PhB_HB_ENABLE( _ARG_ ) \
    GPIOE->ODR |=   (1<<0); // set  /SD B         E0
//    GPIOD->ODR |=   (1<<1); // set  /SD B        D1

#define PWM_PhC_HB_ENABLE( _ARG_ ) \
    GPIOA->ODR |=   (1<<5); // set  /SD C

/*
 * Half-bridge dis-ables ... specific to certain pin 
 * as the device specific ... IR2104 is the /SD pin
 */
#define PWM_PhA_HB_DISABLE( _ARG_ ) \
    GPIOD->ODR &=   ~(1<<2);  // clr /SD A

#define PWM_PhB_HB_DISABLE( _ARG_ ) \
    GPIOE->ODR &=   ~(1<<0); // clr  /SD B    E0
//    GPIOD->ODR &=   ~(1<<1); // clr  /SD B    D1

#define PWM_PhC_HB_DISABLE( _ARG_ ) \
    GPIOA->ODR &=   ~(1<<5); // clr  /SD C


#else

// TIM1 PWM Chnnels ...legacy ... not maintained

//        GPIOC->ODR &=  ~(1<<2);  // PC2 set LO

//        GPIOC->ODR &=  ~(1<<3);  // PC3 set LO

//        GPIOC->ODR &=  ~(1<<4);  // PC4 set LO

/*
 * Half-bridge enables ... specific to certain pin 
 * as the device specific ... IR2104 is the /SD pin
 */
 // C5 C7 G1

#define PWM_PhA_HB_ENABLE( _ARG_ ) \
    GPIOC->ODR |=   (1<<5);  // set /SD A

#define PWM_PhB_HB_ENABLE( _ARG_ ) \
    GPIOC->ODR |=   (1<<7); // set  /SD B

#define PWM_PhC_HB_ENABLE( _ARG_ ) \
    GPIOG->ODR |=   (1<<1); // set  /SD C

/*
 * Half-bridge dis-ables ... specific to certain pin 
 * as the device specific ... IR2104 is the /SD pin
 */
#define PWM_PhA_HB_DISABLE( _ARG_ ) \
    GPIOC->ODR &=   ~(1<<5);  // clr /SD A

#define PWM_PhB_HB_DISABLE( _ARG_ ) \
    GPIOC->ODR &=   ~(1<<7); // clr  /SD B

#define PWM_PhC_HB_DISABLE( _ARG_ ) \
    GPIOG->ODR &=   ~(1<<1); // clr  /SD C

#endif                                           ////// TIM 1


/* Public types -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/
void PWM_PhA_Disable(void);
void PWM_PhB_Disable(void);
void PWM_PhC_Disable(void);

void PWM_PhA_Enable(uint16_t dc);
void PWM_PhB_Enable(uint16_t dc);
void PWM_PhC_Enable(uint16_t dc);

#endif // PWM_STM_S_H
