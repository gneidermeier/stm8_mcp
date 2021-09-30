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


// List of supported SPI configurations
#define SPI_NONE                0
#define SPI_STM8_MASTER         1
#define SPI_STM8_SLAVE          2


#if defined ( __SDCC )
//  #define  S105_DEV 1 // TODO: needs improved and coordiated w/ makefile
#define S105_DISCOVERY   1
#endif


//#define ENABLE_MAN_TIMING

/**
 * the STM8 variant is defined in the project file, along with the appropriate 
 * compiler settings for the particular MCU (memory model etc.)
 * Here is defined the particular platform, where the particular MCU and pin
 * routing determines how the peripherals and GPIO pins are allocated.
 */
#if defined ( S105_DEV )

// AIN0, B0
  #define PH0_BEMF_IN_PORT   GPIOB
  #define PH0_BEMF_IN_PIN    GPIO_PIN_0
  #define PH0_BEMF_IN_CH     ADC1_CHANNEL_0

  #define LED_GPIO_PORT      GPIOE
  #define LED_GPIO_PIN       GPIO_PIN_5

  #define SERVO_GPIO_PORT    GPIOD
  #define SERVO_GPIO_PIN     GPIO_PIN_4

  #define HAS_SERVO_INPUT
  #define SPI_ENABLED        SPI_STM8_MASTER

  #define UNDERVOLTAGE_FAULT_ENABLED

#elif defined ( S105_DISCOVERY )
/*
 * S105 Discovery board can't use TIM1 for PWM (unless solder bridges connecting the
 * touch sensor are removed). Therefore, TIM2 drives PWM/controller, and TIM3 to
 * drive the commutation step, leaving TIM1 unnused (TIM1 input capture/compare 
 * might be used capture servo signal from R/C radio or flight controller).
 */
// AIN0, B0
  #define PH0_BEMF_IN_PORT   GPIOB
  #define PH0_BEMF_IN_PIN    GPIO_PIN_0
  #define PH0_BEMF_IN_CH     ADC1_CHANNEL_0

  #define LED_GPIO_PORT      GPIOD
  #define LED_GPIO_PIN       GPIO_PIN_0

  #define SERVO_GPIO_PORT    GPIOC
  #define SERVO_GPIO_PIN     GPIO_PIN_4

  #define SPI_ENABLED        SPI_STM8_MASTER
  #define HAS_SERVO_INPUT

  #define UNDERVOLTAGE_FAULT_ENABLED

#elif defined ( S003_DEV )
/*
 * s003 does not have TIM3. TIM2 drives PWM/control, TIM1 drives commutation step.
 * Not likely to use the 8k flash part, code barely or does not fit.
 */
// AIN3, D2
  #define PH0_BEMF_IN_PORT   GPIOD
  #define PH0_BEMF_IN_PIN    GPIO_PIN_2
  #define PH0_BEMF_IN_CH     ADC1_CHANNEL_3

  #define LED_GPIO_PORT      GPIOB
  #define LED_GPIO_PIN       GPIO_PIN_5

// TODO TIM1 CH3/4
  #define SERVO_GPIO_PORT    (GPIO_TypeDef *)(-1)
  #define SERVO_GPIO_PIN     (uint8_t)(-1)

//  #define HAS_SERVO_INPUT // TODO TIM1 CH3/4

//  #define SPI_ENABLED     // can't fit SPI in 8k

//  #define UNDERVOLTAGE_FAULT_ENABLED
#endif

#ifndef SPI_ENABLED
#define SPI_ENABLED SPI_NONE
#endif

#define SPI_RX_BUF_SZ  16 // 256 // tmp


/*
 * (un)comment macro to set stm8 clock from 8Mhz or 16Mhz
 */
#define CLOCK_16  // @deprecated


/**
 * @brief Scale factor in commutation-timing related constants
 */
#define CTIME_SCALAR 1

#endif // SYSTEM_H
