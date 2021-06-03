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

#elif defined ( S003_DEV )
  #define SDa_SD_PIN  GPIO_PIN_1 // A1
  #define SDb_SD_PIN  GPIO_PIN_2 // A2
  #define SDc_SD_PIN  GPIO_PIN_3 // C3

  #define SDa_SD_PORT  GPIOA
  #define SDb_SD_PORT  GPIOA
  #define SDc_SD_PORT  GPIOC
#endif

/**
 * The PWM pins used depend on the Timer instance, which may likely be either TIM1 or TIM2
 */
#if (CONTRLLR_TIMER == 1)
// s105 DEV board TIM1 CH1 shares pin with UART2_CK on pin PC1 (not used, but maybe it would be)
  #define SDa_PWM_PIN  GPIO_PIN_2 // C1
  #define SDb_PWM_PIN  GPIO_PIN_3 // C2
  #define SDc_PWM_PIN  GPIO_PIN_4 // C3

  #define SDa_PWM_PORT  GPIOC
  #define SDb_PWM_PORT  GPIOC
  #define SDc_PWM_PORT  GPIOC

#elif (CONTRLLR_TIMER == 2)
  #define SDa_PWM_PIN  GPIO_PIN_4 // D4
  #define SDb_PWM_PIN  GPIO_PIN_3 // D3
  #define SDc_PWM_PIN  GPIO_PIN_3 // A3

  #define SDa_PWM_PORT  GPIOD
  #define SDb_PWM_PORT  GPIOD
  #define SDc_PWM_PORT  GPIOA

#else 
// #pragma controller/pwm timer not defined
asdg
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

void set_dutycycle(uint16_t);

void PWM_setup(void);

#endif // PWM_STM_S_H
