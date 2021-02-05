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
 * re-purpose the TIM2 channel enumeration but disguise it
 */
#define  BL_PHASE_A  TIM2_CHANNEL_1
#define  BL_PHASE_B  TIM2_CHANNEL_2
#define  BL_PHASE_C  TIM2_CHANNEL_3


/*
 * bleh yuk erg
 */
// PD4 set LO
#define PWM_PhA_OUTP_LO( )                   \
    GPIOD->ODR &= (uint8_t) ( ~(1<<4) );     \
    GPIOD->DDR |=  (1<<4);                   \
    GPIOD->CR1 |=  (1<<4);

// PD3 set LO
#define PWM_PhB_OUTP_LO( ) \
    GPIOD->ODR &= (uint8_t) ( ~(1<<3) );     \
    GPIOD->DDR |=  (1<<3);                   \
    GPIOD->CR1 |=  (1<<3);

// PA3 set LO
#define PWM_PhC_OUTP_LO( )                   \
    GPIOA->ODR &= (uint8_t) ( ~(1<<3) );     \
    GPIOA->DDR |=  (1<<3);                   \
    GPIOA->CR1 |=  (1<<3);

/*
 * Half-bridge enables ... specific to certain pin
 * as the device specific ... IR2104 is the /SD pin
 *
 * Combine enable/disable using _ARG_ ?
 */
// D2 D1 A5

#define PWM_PhA_HB_ENABLE( ) \
    GPIOD->ODR |=   (1<<2);   // set /SD A

#define PWM_PhB_HB_ENABLE( ) \
    GPIOE->ODR |=   (1<<0);   // set  /SD B        E0

#define PWM_PhC_HB_ENABLE( ) \
    GPIOA->ODR |=   (1<<5);   // set  /SD C

/*
 * Half-bridge dis-ables ... specific to certain pin
 * as the device specific ... IR2104 is the /SD pin.
 * uint8 casts quash compiler value out of range warning presumably bitwise
 * inverse expression sign extends but the register type is 8 bits
 */
#define PWM_PhA_HB_DISABLE( ) \
    GPIOD->ODR &=  (uint8_t) ( ~(1<<2) ); // clr  /SD A

#define PWM_PhB_HB_DISABLE( ) \
    GPIOE->ODR &=  (uint8_t) ( ~(1<<0) ); // clr  /SD B    E0

#define PWM_PhC_HB_DISABLE( ) \
    GPIOA->ODR &=  (uint8_t) ( ~(1<<5) ); // clr  /SD C



/* Public types -----------------------------------------------------------*/


/**
 * @brief  Generic PWM channel type.
 */
typedef  TIM2_Channel_TypeDef PWM_Channel_Typedef ;


/* Public variables  ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/

void All_phase_stop(void);

void PWM_PhA_Disable(void);
void PWM_PhB_Disable(void);
void PWM_PhC_Disable(void);

void PWM_PhA_Enable(void);
void PWM_PhB_Enable(void);
void PWM_PhC_Enable(void);

void set_dutycycle(uint16_t);


#endif // PWM_STM_S_H
