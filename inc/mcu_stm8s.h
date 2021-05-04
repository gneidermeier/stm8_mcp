/**
  ******************************************************************************
  * @file mcu_stm8s.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */
#ifndef MCU_STM8S
#define MCU_STM8S

/* Includes ------------------------------------------------------------------*/

// app headers
#include "system.h" // platform specific delarations

/* defines -------------------------------------------------------------------*/

#if defined ( S105_DEV )
  #define SDA_PORT  (GPIOC)
  #define SDA_PIN   (GPIO_PIN_5)
  #define SDB_PORT  (GPIOC)
  #define SDB_PIN   (GPIO_PIN_6)
  #define SDC_PORT  (GPIOC)
  #define SDC_PIN   (GPIO_PIN_7)

#elif defined ( S003_DEV )
  #define SDA_PORT  (GPIOA)
  #define SDA_PIN   (GPIO_PIN_1)
  #define SDB_PORT  (GPIOA)
  #define SDB_PIN   (GPIO_PIN_2)
  #define SDC_PORT  (GPIOC)
  #define SDC_PIN   (GPIO_PIN_3)
#else //S105_DISCOVERY 
  #define SDA_PORT  (GPIOD)
  #define SDA_PIN   (GPIO_PIN_2)
  #define SDB_PORT  (GPIOE)
  #define SDB_PIN   (GPIO_PIN_0)
  #define SDC_PORT  (GPIOA)
  #define SDC_PIN   (GPIO_PIN_5)
#endif

/**
 * @brief reference: SPL UART example project
 */
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

uint8_t SerialKeyPressed(char *key);

void UARTputs(char *message);

void MCU_Init(void);

void MCU_set_comm_timer(uint16_t);


#endif // MCU_STM8S
