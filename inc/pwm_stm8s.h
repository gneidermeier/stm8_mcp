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

/* Public defines -----------------------------------------------------------*/

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



/* Public types -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/
void PWM_PhA_Disable(void);
void PWM_PhB_Disable(void);
void PWM_PhC_Disable(void);

void PWM_PhA_Enable(void);
void PWM_PhB_Enable(void);
void PWM_PhC_Enable(void);

void set_dutycycle(uint16_t);
uint16_t get_dutycycle(void);

void inc_dutycycle(void);
void dec_dutycycle(void);


#endif // PWM_STM_S_H
