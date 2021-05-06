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
#include "pwm_stm8s.h"

/* defines -------------------------------------------------------------------*/

#include "stm8s_gpio.h"

/**
 * GPIO definitions are neeeded to drive the SD signals.
 * (PWM Timer channel pins need no such explicit GPIO initialization)
 */
// Phase A
#define SDA_PORT  SDa_SD_PORT
#define SDA_PIN   SDa_SD_PIN
// Phase B
#define SDB_PORT  SDb_SD_PORT
#define SDB_PIN   SDb_SD_PIN
// Phase C
#define SDC_PORT  SDc_SD_PORT
#define SDC_PIN   SDc_SD_PIN

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
