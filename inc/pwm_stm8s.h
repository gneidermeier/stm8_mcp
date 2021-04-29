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
//#define  BL_PHASE_A  TIM2_CHANNEL_1
//#define  BL_PHASE_B  TIM2_CHANNEL_2
//#define  BL_PHASE_C  TIM2_CHANNEL_3


/*
 * Half-bridge enable/disable signals (/SD input pin on IR2104 HB driver IC)
 */
#ifdef DISCOVERY
  #define SDa_GPIO_PIN  GPIO_PIN_2 // D2
  #define SDb_GPIO_PIN  GPIO_PIN_0 // E0
  #define SDc_GPIO_PIN  GPIO_PIN_5 // A5

  #define SDa_GPIO_PORT  GPIOD
  #define SDb_GPIO_PORT  GPIOE
  #define SDc_GPIO_PORT  GPIOA

#else // stm8s003
  #define SDa_GPIO_PIN  GPIO_PIN_1 // A1
  #define SDb_GPIO_PIN  GPIO_PIN_2 // A2
  #define SDc_GPIO_PIN  GPIO_PIN_3 // C3

  #define SDa_GPIO_PORT  GPIOA
  #define SDb_GPIO_PORT  GPIOA
  #define SDc_GPIO_PORT  GPIOC
#endif


// Enable
#define PWM_PhA_HB_ENABLE( ) \
    SDa_GPIO_PORT->ODR |=   SDa_GPIO_PIN; // /SD A

#define PWM_PhB_HB_ENABLE( ) \
    SDb_GPIO_PORT->ODR |=   SDb_GPIO_PIN; // /SD B

#define PWM_PhC_HB_ENABLE( ) \
    SDc_GPIO_PORT->ODR |=   SDc_GPIO_PIN; // /SD C

// Disable (casts applied in order to quash warnings (bit inversion causes sign extension to to 16-bit)
#define PWM_PhA_HB_DISABLE( ) \
    SDa_GPIO_PORT->ODR &=  (uint8_t) ( ~SDa_GPIO_PIN ); // /SD A

#define PWM_PhB_HB_DISABLE( ) \
    SDb_GPIO_PORT->ODR &=  (uint8_t) ( ~SDb_GPIO_PIN ); // /SD B

#define PWM_PhC_HB_DISABLE( ) \
    SDc_GPIO_PORT->ODR &=  (uint8_t) ( ~SDC_GPIO_PIN ); // /SD C


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
