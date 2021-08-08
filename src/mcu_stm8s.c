/**
  ******************************************************************************
  * @file mcu_stm8s.c
  * @brief STM8S platform and peripheral configuration.
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */
/**
 * @defgroup MCU Platform STM8S
 * @brief STM8S platform and peripheral configuration.
 * @{
 */
/* Includes ------------------------------------------------------------------*/
#include "stm8s_gpio.h"

// app headers
#include "pwm_stm8s.h" // pwm timer channels

/* Private defines -----------------------------------------------------------*/
/**
 * @brief GPIO macros for the /SD (enable) input pin to the half-bridge driver
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
 * @brief Forward declarations of low-level term IO functions 
 * Low-level access to support terminal IO on an available stm8s UART. Based on
 * example code from SPL ... Project/STM8S_StdPeriph_Examples/UART/UART1_Printf/
 * Macros for the prototypes were done evidently to support Raisonance compiler.
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

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/** @cond */

#ifdef STM8S105 // S105 Dev board or DISCOVERY

/**
  * @brief Low-level character IO on the serial terminal
  * @details Based on SPL example from STM8S_StdPeriph_Examples/UART/UART1_Printf/
  * @param c Character to send
  * @retval char Character sent
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART2_SendData8(c);
  /* Loop until the end of transmission */
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);

  return (c);
}

/**
  * @brief Low-level character IO on the serial terminal
  * @details Based on SPL example from STM8S_StdPeriph_Examples/UART/UART1_Printf/
  * @param None
  * @retval char Character to read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET);
    c = UART2_ReceiveData8();
  return (c);
}

/**
* @brief  Test to see if a key has been pressed on the terminal.
* @details Read a character non-blocking from serial terminal (c-n-p from STM8s 
*   application note AN3259). 
* @param [out] key Pointer to byte for receiving character code
* @retval 1 a character has been read
* @retval 0 no character has been read
*/
uint8_t SerialKeyPressed(char *key)
{
  FlagStatus flag  ;
  /* First clear Rx buffer */
  flag = UART2_GetFlagStatus(UART2_FLAG_RXNE);
  if ( flag == SET)
  {
    *key = (char)UART2->DR;
    return 1;
  }

  return 0;
}
#else // stm8s003
/**
  * @brief Low-level character IO on the serial terminal
  * @details Based on SPL example from STM8S_StdPeriph_Examples/UART/UART1_Printf/
  * @param c Character to send
  * @retval char Character sent
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

  return (c);
}

/**
  * @brief Low-level character IO on the serial terminal
  * @details Based on SPL example from STM8S_StdPeriph_Examples/UART/UART1_Printf/
  * @param one
  * @retval char Character to read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
  return (c);
}

/**
* @brief  Test to see if a key has been pressed on the terminal.
*
* @details Read a character non-blocking from serial terminal (c-n-p from STM8s 
*   application note AN3259). 
* @param [out]  key  Pointer to byte for receiving character code.
* @retval  1  a character has been read
* @retval  0  no character has been read
*/
uint8_t SerialKeyPressed(char *key)
{
  FlagStatus flag  ;
  /* First clear Rx buffer */
  flag = UART1_GetFlagStatus(UART1_FLAG_RXNE);
  if ( flag == SET)
  {
    *key = (char)UART1->DR;
    return 1;
  }

  return 0;
}
#endif
/** @endcond */

/*
 * @brief Configure GPIO.
 *
 * Specific peripheral module initialization (e.g. A/D, TIMx) will handle
 * setting up suitable IO pin behavior.
 * some of this bulk could be reduced by converting registers to STM8 Peripheral Lib.
 *
 * 11.4 Reset configuration
 *  All I/O pins are generally input floating under reset (i.e. during the reset phase) and at reset
 *  state (i.e. after reset release). However, a few pins may have a different behavior. Refer to
 *  the datasheet pinout description for all details.
 * 11.5 Unused I/O pins
 *  Unused I/O pins must not be left floating to avoid extra current consumption. They must be
 *  put into one of the following configurations:
 *  connected to VDD or VSS by external pull-up or pull-down resistor and kept as input
 *  floating (reset state), configured as input with internal pull-up/down resistor,
 *  configured as output push-pull low.
*/
static void GPIO_Config(void)
{
// LED output pin
  GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

// /SD: A1, A2, C3
  GPIO_Init(SDA_PORT, (GPIO_Pin_TypeDef)SDA_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(SDB_PORT, (GPIO_Pin_TypeDef)SDB_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(SDC_PORT, (GPIO_Pin_TypeDef)SDC_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

// AIN0 (back-EMF sensor): Input floating, no external interrupt
  GPIO_Init(PH0_BEMF_IN_PORT, (GPIO_Pin_TypeDef)PH0_BEMF_IN_PIN, GPIO_MODE_IN_FL_NO_IT);

#if defined( HAS_SERVO_INPUT )
// Input pull-up, no external interrupt
  GPIO_Init(SERVO_GPIO_PORT, (GPIO_Pin_TypeDef)SERVO_GPIO_PIN, GPIO_MODE_IN_PU_NO_IT);
#endif // SERVO

// Input pull-up, no external interrupt
  GPIO_Init(PH0_BEMF_IN_PORT, (GPIO_Pin_TypeDef)PH0_BEMF_IN_PIN, GPIO_MODE_IN_PU_NO_IT);

#if defined ( S105_DEV )

#elif defined( S105_DISCOVERY )

#if 0
// PA4 as button input (Stop)
  GPIOA->DDR &= ~GPIO_PIN_4;
  GPIOA->CR1 |= GPIO_PIN_4;  // pull up w/o interrupts
#endif

#endif
}

/**
 *  @brief Configure UART
 *  @details
 *      - BaudRate = 115200 baud
 *      - Word Length = 8 Bits
 *      - One Stop Bit
 *      - No parity
 *      - Receive and transmit enabled
 *      - UART1 Clock disabled
 *  @param none
 */
static void UART_setup(void)
{
#if defined( S105_DEV ) || defined( S105_DISCOVERY )

  UART2_DeInit();

  UART2_Init(115200,
             UART2_WORDLENGTH_8D,
             UART2_STOPBITS_1,
             UART2_PARITY_NO,
             UART2_SYNCMODE_CLOCK_DISABLE,
             UART2_MODE_TXRX_ENABLE);

  UART2_Cmd(ENABLE);

#elif defined( S003_DEV )

  UART1_DeInit();

  UART1_Init(
    (uint32_t)115200,
    UART1_WORDLENGTH_8D,
    UART1_STOPBITS_1,
    UART1_PARITY_NO,
    UART1_SYNCMODE_CLOCK_DISABLE,
    UART1_MODE_TXRX_ENABLE);

    UART1_Cmd(ENABLE);
#endif
}

/*
 * set ADC clock to 4Mhz  - sample time from the data sheet @ 4Mhz
 * min sample time .75 useconds @ fADC = 4Mhz
 * conversion time = 14 * 1/2000000 = 0.0000035 seconds (3.5 us)
 */
#ifdef CLOCK_16
#define ADC_DIVIDER ADC1_PRESSEL_FCPU_D4  // 8 -> 16/4 = 4
#else
#define ADC_DIVIDER ADC1_PRESSEL_FCPU_D2  // 4 ->  8/2 = 4
#endif
/*
 * https://community.st.com/s/question/0D50X00009XkbA1SAJ/multichannel-adc
 */
static void ADC1_setup(void)
{
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);

  ADC1_DeInit();

  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, // don't care, see ConversionConfig below ..
            ADC1_CHANNEL_3,        // i.e. Ch 0, 1, 2, and 3 are enabled
            ADC_DIVIDER,
            ADC1_EXTTRIG_TIM,      //  ADC1_EXTTRIG_GPIO ... not presently using any ex triggern
            DISABLE,               // ExtTriggerState
            ADC1_ALIGN_RIGHT,
            ADC1_SCHMITTTRIG_ALL,
            DISABLE);              // SchmittTriggerState

  ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE); // grab the sample in the ISR

//ADC1_DataBufferCmd(ENABLE);
  ADC1_ScanModeCmd(ENABLE); // Scan mode from channel 0 to n (as defined in ADC1_Init)

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
  ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion of all channels in sequence
  ADC1_StartConversion(); // i.e. for scanning mode only has to start once ...
}

/**
 * S003 did not have available timer for servo input ... 105 boards should have
 * a spare timer available, but not necessarily the same peripheral instance .
 */
#if defined( HAS_SERVO_INPUT )

#if defined( S105_DEV )
/*
 * Available pins for PWM channels require use of TIM1 ... TIM2 CH1 is a spare
 * pin on the STM8S105 Black (D4) so the input capture is assigned there.
 */
/**
 * @brief Setup timer capture for servo signal pulse input.
 *
 * @details
 * The clock prescaler is used to scale the input capture timer to approximately
 * the range of the radio pulse frame (about 25 ms).
 * The timer period is set to maximum (free running timer) and would have a 
 * maximum time of 1/16Mhz * 0xFFFF == 4.1ms
 * With timer prescale 8 i.e.  4.1 ms * 8 == 0.0327675  (32 ms) - max frame time
 * with 50 Hz frames should be within this range.
 * 
 * Reference information from
 *  https://www.kdedirect.com/blogs/news/understanding-throttle-calibration-esc-deadbands-and-pwm
 *
 *   The throttle range is based on the normal operating range of a Futaba system
 *  (1100탎-1940탎 with 1520 typically being center)
 *  The default Range calibration (1100-1940) has the following deadbands:
 *
 *    Motor Arming pulse: 1100탎
 *    Motor Spinning pulse: 1125탎
 *    Motor Stopping pulse: 1110탎
 *    Max thrust output: 1915탎
 *    Full Stick: 1940탎
 *
 * PWM resolution should be at least 0.1% (some are even claiming 2048 steps)
 *
 * With timers as above:  range observed ($08A0:%EE0)
 * Range is   %EE0 - $08A0 == $0640 == 1600d so 1600 steps of resolution over
 * the throttle signal range.
 * Motor operating range would be about half of throttle range ie. 800 steps 
 * resolution into the PWM driver.
 *
 * Timer operation:
 * The timer period is set to maximum and left free running and the capture-compare
 * channels 3 & 4 used to get leading and trailing edges of radio signal pulse.
 * Refer to the timer peripheral description in the STM8s10x Reference Manual (RM0016) 
 * for specific details of the CC functionality.
*/
static void Servo_CC_setup(void)
{
 #ifdef CLOCK_16
  const TIM2_Prescaler_TypeDef prescaler = TIM2_PRESCALER_8;
#else
  const TIM2_Prescaler_TypeDef prescaler = TIM2_PRESCALER_4;
#endif
  const uint16_t period = 0xFFFF;
  const uint8_t ICFilter = 1;

  TIM2_DeInit();

// The counter clock frequency fCK_CNT is equal to fCK_PSC / (2^(PSC[3:0]))
  TIM2_TimeBaseInit( prescaler, period);

  TIM2_ICInit(TIM2_CHANNEL_1,
              TIM2_ICPOLARITY_RISING,
              TIM2_ICSELECTION_DIRECTTI,
              TIM2_ICPSC_DIV1,
              ICFilter
             );

  TIM2_ICInit(TIM2_CHANNEL_2,
              TIM2_ICPOLARITY_FALLING,
              TIM2_ICSELECTION_INDIRECTTI,
              TIM2_ICPSC_DIV1, // TIM1_ICPrescaler
              ICFilter
             );

// timer update/ovrflow ISR not strictly needed but is handy to confirm timer rate
//  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);

// enable capture channels
  TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
  TIM2_ITConfig(TIM2_IT_CC2, ENABLE);

  TIM2_Cmd(ENABLE);
}

#elif defined( S105_DISCOVERY )
/*
 * STM8s105 Discovery TIM1 not available for PWM (unless touch pad disabled by
 * removing solder bridges, i.e. PWM must be on TIM2 but TIM1 is available for input capture.
 */
static void Servo_CC_setup(void)
{
/*
 * counter clock frequency fCK_CNT is equal to fCK_PSC / (PSCR[15:0]+1)
 */
 #ifdef CLOCK_16
  const uint16_t prescaler = 8;
#else
  const uint16_t prescaler = 4;
#endif
  const uint16_t period = 0xFFFF;
  const uint8_t repetitionCounter = 1;
  const uint8_t ICFilter = 1;
  TIM1_DeInit();

  TIM1_TimeBaseInit( prescaler - 1 , TIM1_COUNTERMODE_UP, period, repetitionCounter );

  TIM1_ICInit(TIM1_CHANNEL_4,
              TIM1_ICPOLARITY_RISING,
              TIM1_ICSELECTION_DIRECTTI,
              TIM1_ICPSC_DIV1,
              ICFilter
             );

  TIM1_ICInit(TIM1_CHANNEL_3,
              TIM1_ICPOLARITY_FALLING,
              TIM1_ICSELECTION_INDIRECTTI,
              TIM1_ICPSC_DIV1,
              ICFilter
             );

// timer update/ovrflow ISR not strictly needed but is handy to confirm timer rate
//  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE); // be sure flag is cleared in ISR!

// enable capture channels 3 & 4
  TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
  TIM1_ITConfig(TIM1_IT_CC3, ENABLE);

  TIM1_Cmd(ENABLE);
}
#endif // S105 DISCOVERY
#endif // HAS_SERVO_INP

/*
 * commutation timer on TIM1 or TIM3 depending on the specific stm8s part
 */
#if defined ( S105_DEV ) || defined ( S105_DISCOVERY )
/*
 * Timers 2 3 & 5 are 16-bit general purpose timers
 *  Sets the commutation switching period.
 *
 *  @8Mhz, fMASTER period ==  0.000000125 S
 *   Timer Step:
 *     step = 1 / 8Mhz * prescaler = 0.000000125 * (2^1) = 0.000000250 S
 */
#ifdef CLOCK_16
#define TIM3_PSCR  0x01  // 2^1 == 2
#else
#define TIM3_PSCR  0x00  // 2^0 == 1
#endif

/**
 * @brief  Set timer that establishes commutation timing period.
 * @param  period  Value written to timer reload register
 */
void MCU_set_comm_timer(uint16_t period)
{
  TIM3->PSCR = TIM3_PSCR;

  TIM3->ARRH = (uint8_t)(period >> 8); // be sure to set byte ARRH first, see data sheet
  TIM3->ARRL = (uint8_t)(period & 0xff);

  TIM3->IER |= TIM3_IER_UIE; // Enable Update Interrupt
  TIM3->CR1 = TIM3_CR1_ARPE; // auto (re)loading the count
  TIM3->CR1 |= TIM3_CR1_CEN; // Enable TIM3
}

#elif defined( S003_DEV ) // uses TIM1 which is not preferred

/**
 * @brief Sets period of the commutation timer.
 * @param  period  Value written to auto-reload register
 */
#ifdef CLOCK_16
#define TIM1_PSCR  0x02
#else
#define TIM1_PSCR  0x01
#endif

void MCU_set_comm_timer(uint16_t period)
{
  const uint16_t TIM1_Prescaler = TIM1_PSCR - 1;

  /* Set the Prescaler value */
  TIM1->PSCRH = (uint8_t)(TIM1_Prescaler >> 8);
  TIM1->PSCRL = (uint8_t)(TIM1_Prescaler);

  TIM1->ARRH = (uint8_t)(period >> 8); // be sure to set byte ARRH first, see data sheet
  TIM1->ARRL = (uint8_t)(period & 0xff);

  TIM1->IER |= TIM1_IER_UIE; // Enable Update Interrupt
  TIM1->CR1 = TIM1_CR1_ARPE; // auto (re)loading the count
  TIM1->CR1 |= TIM1_CR1_CEN; // Enable timer
}
#endif

/*
 * http://embedded-lab.com/blog/starting-stm8-microcontrollers/13/
 * GN:  by default  microcontroller uses   internal 16MHz RC oscillator
 * ("HSI", or high-speed internal) divided by eight  as a clock source. This results in a base timer frequency of 2MHz.
 * Using this function just to show the library way to explicit clock setup.
 */
static void Clock_setup(void)
{
  CLK_DeInit();

#if defined( S105_DEV ) || defined( S003_DEV )
  /*High speed internal clock prescaler: 1*/
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

#elif defined( S105_DISCOVERY )
  // Configure Quartz Clock
  CLK_HSECmd(ENABLE);
#endif

#ifdef CLOCK_16
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); // 16Mhz
#else
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2); // 8Mhz
#endif // CLK

// enable timer peripheral clocks ... otherwise the clocks enables left to the
// individual peripheral initialiations.
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);
}

#if SPI_ENABLED
/**
 * @brief  Configure SPI bus
 *
 * References:
 *   https://www.programmersought.com/article/34101773427/
 */
void SPI_setup(void)
{
  // Enable SPI Clock.
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);

  SPI_DeInit();

#if SPI_ENABLED == SPI_STM8_MASTER

  // Set GPIO pins to output push-pull high level.

// S105_BLACK ... LED on E5
// CS not required for single master/slave pair
//   GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_SLOW); // CS

  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_SLOW); // SCLK
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_SLOW); // MOSI

  // This setting is critical, as the master must be set to input MISO pin
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);

  SPI_Init(SPI_FIRSTBIT_MSB,
#ifdef CLOCK_16
           SPI_BAUDRATEPRESCALER_256, // tmp test //      SPI_BAUDRATEPRESCALER_16, // how fast
#else
           SPI_BAUDRATEPRESCALER_128, // tmp test //      SPI_BAUDRATEPRESCALER_16, // how fast
#endif
           SPI_MODE_MASTER,
           SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE,
           SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, (uint8_t)0x07);

#else
  // configure input pins with pullup
  GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);  // CS
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);  // SCLK
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);  // MOSI

  // MISO is output push-pull high level
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_PP_HIGH_SLOW);

  SPI_Init(SPI_FIRSTBIT_MSB,
           SPI_BAUDRATEPRESCALER_16, // don't care
           SPI_MODE_SLAVE,
           SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE,
           SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_HARD, (uint8_t)0x07);

#if 0 //  #ifdef SPI_PERPIPH_INT
  SPI_ITConfig(SPI_IT_RXNE, ENABLE); // Interrupt when the Rx buffer is not empty.
#endif

#endif // SPI_ENABLED == SPI_STM8_MASTER

  //Enable SPI.
  SPI_Cmd(ENABLE);
}
#endif // SPI_ENABLE


/**
 * @brief  Initialize MCU and peripheral modules
 * @details  Configures clocks, GPIO, UART, ADC, timers, PWM.
 */
void MCU_Init(void)
{
  Clock_setup();
  GPIO_Config();
  UART_setup();
  PWM_setup();

#if !defined (S003_DEV)
  ADC1_setup();
#endif

#if defined( HAS_SERVO_INPUT )
  Servo_CC_setup();
#endif

#if SPI_ENABLED
  SPI_setup();
#endif
}

/**@}*/ // defgroup
