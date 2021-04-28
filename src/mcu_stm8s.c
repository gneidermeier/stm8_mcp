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
 * @defgroup mcu Platform STM8S
 * @brief STM8S platform and peripheral configuration.
 * @{
 */

/* Includes ------------------------------------------------------------------*/

// app headers
#include "system.h" // platform specific delarations


/* Private defines -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/


/*
 * @brief Configures a few pins needed as GPIO.
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
#ifndef DISCOVERY

  /* Initialize I/Os in Output Mode */
  GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);

#else

// OUTPUTS
// built-in LED
  GPIOD->ODR |= (1 << LED); //LED initial state is OFF (cathode driven to Vcc)
  GPIOD->DDR |= (1 << LED); //PD.n as output
  GPIOD->CR1 |= (1 << LED); //push pull output

// D1 is reserved for SWIM
//    GPIOD->ODR &=  ~(1<<1);
//    GPIOD->DDR |=  (1<<1);
//    GPIOD->CR1 |=  (1<<1);

// SD/A=PD2
  GPIOD->ODR &=  ~(1<<2);
  GPIOD->DDR |=  (1<<2);
  GPIOD->CR1 |=  (1<<2);

// SD/B=PE0
  GPIOE->ODR &=  ~(1<<0); //E0 data
  GPIOE->DDR |=  (1<<0); //E0 dir
  GPIOE->CR1 |=  (1<<0); //E0	cfg

// SD/C=PA5
  GPIOA->ODR &= ~(1 << 5); // set pin off
  GPIOA->DDR |= (1 << 5);  // PA.5 as OUTPUT
  GPIOA->CR1 |= (1 << 5);  // push pull output

#ifdef DISCOVERY // GN: STM8s-105 Discovery lets this pin
// use PG0 (CN2-11) as test pins
  GPIOG->ODR &= ~(1<<0);
  GPIOG->DDR |=  (1<<0);
  GPIOG->CR1 |=  (1<<0);
#endif

// INPUTS
// C4 set as input (used with TIM1 CC4 to measure servo pulse)
  GPIOC->DDR &= ~(1 << 4); // PC.4 as input
  GPIOC->CR1 |= (1 << 4);  // pull up w/o interrupts

// PA4 as button input (Stop)
  GPIOA->DDR &= ~(1 << 4); // PA.4 as input
  GPIOA->CR1 |= (1 << 4);  // pull up w/o interrupts
// uses CN2.7 as GND

// PA6 as button input (B+)
  GPIOA->DDR &= ~(1 << 6); // PA.6 as input
  GPIOA->CR1 |= (1 << 6);  // pull up w/o interrupts

#if 0
// PE5 as button input (B-) ^H^H^H^H^H^H^H    SPI CSS  (Input)
  GPIOE->DDR &= ~(1 << 5); // PE.5 as input
  GPIOE->CR1 |= (1 << 5);  // pull up w/o interrupts
#endif

// PE.6 AIN9
  GPIOE->DDR &= ~(1 << 6);  // PE.6 as input
  GPIOE->CR1 &= ~(1 << 6);  // floating input
  GPIOE->CR2 &= ~(1 << 6);  // 0: External interrupt disabled   ???

// PE.7 AIN8
  GPIOE->DDR &= ~(1 << 7);  // PE.7 as input
  GPIOE->CR1 &= ~(1 << 7);  // floating input
  GPIOE->CR2 &= ~(1 << 7);  // 0: External interrupt disabled   ???

// AIN7
  GPIOB->DDR &= ~(1 << 7);  // PB.7 as input
  GPIOB->CR1 &= ~(1 << 7);  // floating input
  GPIOB->CR2 &= ~(1 << 7);  // 0: External interrupt disabled   ???

// AIN6
  GPIOB->DDR &= ~(1 << 6);  // PB.6 as input
  GPIOB->CR1 &= ~(1 << 6);  // floating input
  GPIOB->CR2 &= ~(1 << 6);  // 0: External interrupt disabled   ???

// AIN5
  GPIOB->DDR &= ~(1 << 5);  // PB.5 as input
  GPIOB->CR1 &= ~(1 << 5);  // floating input
  GPIOB->CR2 &= ~(1 << 5);  // 0: External interrupt disabled   ???

// AIN4
  GPIOB->DDR &= ~(1 << 4);  // PB.4 as input
  GPIOB->CR1 &= ~(1 << 4);  // floating input
  GPIOB->CR2 &= ~(1 << 4);  // 0: External interrupt disabled   ???

// AIN3
  GPIOB->DDR &= ~(1 << 3);  // PB.3 as input
  GPIOB->CR1 &= ~(1 << 3);  // floating input
  GPIOB->CR2 &= ~(1 << 3);  // 0: External interrupt disabled   ???

// AIN2
  GPIOB->DDR &= ~(1 << 2);  // PB.2 as input
  GPIOB->CR1 &= ~(1 << 2);  // floating input
  GPIOB->CR2 &= ~(1 << 2);  // 0: External interrupt disabled   ???

// AIN1
  GPIOB->DDR &= ~(1 << 1);  // PB.1 as input
  GPIOB->CR1 &= ~(1 << 1);  // floating input
  GPIOB->CR2 &= ~(1 << 1);  // 0: External interrupt disabled   ???

// AIN0
  GPIOB->DDR &= ~(1 << 0);  // PB.0 as input
  GPIOB->CR1 &= ~(1 << 0);  // floating input
  GPIOB->CR2 &= ~(1 << 0);  // 0: External interrupt disabled   ???
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
#ifdef DISCOVERY
  UART2_DeInit();

  UART2_Init(115200,
             UART2_WORDLENGTH_8D,
             UART2_STOPBITS_1,
             UART2_PARITY_NO,
             UART2_SYNCMODE_CLOCK_DISABLE,
             UART2_MODE_TXRX_ENABLE);

  UART2_Cmd(ENABLE);
#else
  UART1_DeInit();

  UART1_Init(
    (uint32_t)115200,
    UART1_WORDLENGTH_8D,
    UART1_STOPBITS_1,
    UART1_PARITY_NO,
    UART1_SYNCMODE_CLOCK_DISABLE,
    UART1_MODE_TXRX_ENABLE);

//    UART1_Cmd(ENABLE); // GN: must enable by default
//    UART1->CR1 &= (uint8_t)(~UART1_CR1_UARTD);
#endif
}

// TODO: deprecated

#ifdef DISCOVERY

#else
/**
 *  @brief Send a message to the debug port (UART2).
 *  @details
 *  Sends a variable length, null-terminated string to the UART.
 *    (https://blog.mark-stevens.co.uk/2012/08/using-the-uart-on-the-stm8s-2/)
 *  @param message Pointer to char array, i.e. a null-terminated ASCII string.
 */
void UARTputs(char *message)
{
  char *ch = message;
  while (*ch)
  {
    UART1->DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
    while ( 0 == (UART1->SR & UART1_SR_TXE) ); //  Wait for transmission to complete.
    ch++;                                      //  Grab the next character.
  }
}

/**
* @brief  Test to see if a key has been pressed on the terminal.
*
* @details Allows reading a character but non-blocking.
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
  else
  {
    return 0;
  }
}
#endif

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
// Port B[0..7]=floating input no interr
// STM8 Discovery, all PortB pins are on CN3
//    GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_IN_FL_NO_IT); // all AIN pins setup explicityly with the GPIO init

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

#ifdef DISCOVERY
// copied from STM8 peripheral library (it is static in there)
static void _TI3_Config(uint8_t TIM1_ICPolarity,
                        uint8_t TIM1_ICSelection,
                        uint8_t TIM1_ICFilter)
{
  /* Disable the Channel 3: Reset the CCE Bit */
  TIM1->CCER2 &=  (uint8_t)(~TIM1_CCER2_CC3E);

  /* Select the Input and set the filter */
  TIM1->CCMR3 =
    (uint8_t)((uint8_t)(TIM1->CCMR3 & (uint8_t)(~(uint8_t)( TIM1_CCMR_CCxS | TIM1_CCMR_ICxF)))
              | (uint8_t)(( (TIM1_ICSelection)) | ((uint8_t)( TIM1_ICFilter << 4))));

  /* Select the Polarity */
  if (TIM1_ICPolarity != TIM1_ICPOLARITY_RISING)
  {
    TIM1->CCER2 |= TIM1_CCER2_CC3P;
  }
  else
  {
    TIM1->CCER2 &= (uint8_t)(~TIM1_CCER2_CC3P);
  }
  /* Set the CCE Bit */
  TIM1->CCER2 |=  TIM1_CCER2_CC3E;
}

/*
 * Timer 1 prescaler
*/
#ifdef CLOCK_16
#define TIM1_PRESCALER 16 //
#else
#define TIM1_PRESCALER 8  // 1/8Mhz * 16 * 65536 = 0.131072 (about 131ms)
#endif

/**
 * @brief Timer 1 Setup
 *
 * @details
 * The clock prescaler is used to scale 1ms radio signal pulse measurement to a
 * range of just of 0x0300 (exceeds the PWM resolution commanded to the motor).
 * The timer period is set to maximum and left free running and the capture-compare
 * channels 3 & 4 used to get leading and trailing edges of radio signal pulse.
 * THis is explained in STM8 Reference Manual RM0016.
*/
static void TIM1_setup(void)
{
  const uint16_t T1_Period = 0xFFFF;

  TIM1_DeInit();

  TIM1_TimeBaseInit(( TIM1_PRESCALER - 1 ), TIM1_COUNTERMODE_UP, T1_Period, 1);

  TIM1_ICInit(TIM1_CHANNEL_4,
              TIM1_ICPOLARITY_RISING,
              TIM1_ICSELECTION_DIRECTTI,
              TIM1_ICPSC_DIV1, // TIM1_ICPrescaler
              0x0F // 1  // TIM1_ICFilter maxxd out ... motor noise
             );

  /* TI3 Configuration set input to TIM1ch4 .. copied from TIM1_ICInit() */
  _TI3_Config(TIM1_ICPOLARITY_FALLING,
              TIM1_ICSELECTION_INDIRECTTI, // TIM1 Input 3 is selected to be connected to IC4
              1);
  /* Set the Input Capture Prescaler value */
  TIM1_SetIC3Prescaler(TIM1_ICPSC_DIV1);

// timer update/ovrflow ISR not strictly needed but is handy to confirm timer rate
  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
// enable capture channels 3 & 4
  TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
  TIM1_ITConfig(TIM1_IT_CC3, ENABLE);

  TIM1_Cmd(ENABLE);
}
#else

/**
 * @brief Configure brushless motor commutation timing
 * @details
 * Sets period of commutation timer peripheral assigned to commutation timing.
 * Prescaler value is set depending whether system is configured for 8 or 16 Mhz CPU clock.
 * @param  period  Value written to the timer peripheral auto-reload register
 */
#ifdef CLOCK_16
#define TIM1_PSCR  0x02
#else
#define TIM1_PSCR  0x01
#endif

void MCU_comm_time_cfg(uint16_t period)
{
  const uint16_t TIM1_Prescaler = TIM1_PSCR - 1;

  /* Set the Prescaler value */
  TIM1->PSCRH = (uint8_t)(TIM1_Prescaler >> 8);
  TIM1->PSCRL = (uint8_t)(TIM1_Prescaler);

  TIM1->ARRH = (uint8_t)(period >> 8);   // be sure to set byte ARRH first, see data sheet
  TIM1->ARRL = (uint8_t)(period & 0xff);

  TIM1->IER |= TIM1_IER_UIE; // Enable Update Interrupt
  TIM1->CR1 = TIM1_CR1_ARPE; // auto (re)loading the count
  TIM1->CR1 |= TIM1_CR1_CEN; // Enable timer
}
#endif


/*
 * Setup TIM2 PWM
 * Reference: AN3332
 */
#ifdef CLOCK_16
#define TIM2_PRESCALER TIM2_PRESCALER_8  //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
#define TIM2_PRESCALER TIM2_PRESCALER_4  //    (1/8Mhz)  * 4 * 250 -> 0.000125 S
#endif

static void TIM2_setup(void)
{
  /* TIM2 Peripheral Configuration */
  TIM2_DeInit();

  /* Set TIM2 Frequency to 2Mhz */
  TIM2_TimeBaseInit(TIM2_PRESCALER, TIM2_PWM_PD);
  /* Channel 1 PWM configuration */
  TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
  TIM2_OC1PreloadConfig(ENABLE);

  /* Channel 2 PWM configuration */
  TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
  TIM2_OC2PreloadConfig(ENABLE);

  /* Channel 3 PWM configuration */
  TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
  TIM2_OC3PreloadConfig(ENABLE);

  /* Enables TIM2 peripheral Preload register on ARR */
  TIM2_ARRPreloadConfig(ENABLE);


  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE); // GN: for eventual A/D triggering

  /* Enable TIM2 */
  TIM2_Cmd(ENABLE);
}


/*
 * Configure Timer 4 as general purpose fixed time-base reference
 * Timer 4 & 6 are 8-bit basic timers
 *
 *   https://lujji.github.io/blog/bare-metal-programming-stm8/
 *
 * Setting periodic task for fast-ish rate of A/D acquisition.
 * ISR must set 'TaskRdy' flag and not block on the task since A/D does a blocking wait.
 */
static void TIM4_setup(void)
{
  const uint8_t Period = (16 * SYS_RATE_MULT);

#ifdef CLOCK_16
  TIM4->PSCR = 0x07; // PS = 128  -> 0.0000000625 * 128 * p
#else
  TIM4->PSCR = 0x06; // PS =  64  -> 0.000000125  *  64 * p
#endif

  TIM4->ARR = Period;

  TIM4->IER |= TIM4_IER_UIE; // Enable Update Interrupt
  TIM4->CR1 |= TIM4_CR1_CEN; // Enable TIM4
}

/*
 * Timers 2 3 & 5 are 16-bit general purpose timers
 *  Sets the commutation switching period.
 *
 *  @8Mhz, fMASTER period ==  0.000000125 S
 *   Timer Step:
 *     step = 1 / 8Mhz * prescaler = 0.000000125 * (2^1) = 0.000000250 S
 */

#ifdef DISCOVERY // 003 doesn't have this peripheral instance
#ifdef CLOCK_16
#define TIM3_PSCR  0x01  // 2^1 == 2
#else
#define TIM3_PSCR  0x00  // 2^0 == 1
#endif
/**
 * @brief Sets TIM3 period.
 * Sets the TIM3 prescaler, auto-reload register (ARR), enables interrupt.
 * Prescaler value is set depending whether system is configured for 8 or 16 Mhz CPU clock.
 * @param  period  Value written to TIM3 ARR
 */
void TIM3_setup(uint16_t period)
{
  TIM3->PSCR = TIM3_PSCR;

  TIM3->ARRH = (uint8_t)(period >> 8);   // be sure to set byte ARRH first, see data sheet
  TIM3->ARRL = (uint8_t)(period & 0xff);

  TIM3->IER |= TIM3_IER_UIE; // Enable Update Interrupt
  TIM3->CR1 = TIM3_CR1_ARPE; // auto (re)loading the count
  TIM3->CR1 |= TIM3_CR1_CEN; // Enable TIM3
}
#endif // DISCOVER

/*
 * http://embedded-lab.com/blog/starting-stm8-microcontrollers/13/
 * GN:  by default  microcontroller uses   internal 16MHz RC oscillator
 * ("HSI", or high-speed internal) divided by eight  as a clock source. This results in a base timer frequency of 2MHz.
 * Using this function just to show the library way to explicit clock setup.
 */
static void Clock_setup(void)
{
  CLK_DeInit();

#if !defined(DISCOVERY) // && defined(INTCLOCK)

  /*High speed internal clock prescaler: 1*/
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

#else
  // Configure Quartz Clock
  CLK_DeInit();
  CLK_HSECmd(ENABLE);
#ifdef CLOCK_16
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); // 16Mhz
#else
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV2); // 8Mhz
#endif // CLK
#endif
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE); // uart2 on the stm8s Discovery
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
}

#ifdef DISCOVERY
/**
* References:
*   https://www.programmersought.com/article/34101773427/
*/
void SPI_setup(void)
{
  // Enable SPI Clock.
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);

  SPI_DeInit();

#ifdef SPI_CONTROLLER

  // Set GPIO pins to output push-pull high level.
  GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_SLOW); // CS
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_SLOW); // SCLK
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_HIGH_SLOW); // MOSI

  // This setting is critical, as the master must be set to input MISO pin
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);

  SPI_Init(SPI_FIRSTBIT_MSB,
           SPI_BAUDRATEPRESCALER_256, // tmp test //      SPI_BAUDRATEPRESCALER_16, // how fast
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
#endif

  //Enable SPI.
  SPI_Cmd(ENABLE);
}
#endif // DISCOVER

/**
 * @brief  Initialize MCU and peripheral modules
 * Configures clocks, GPIO, UART, ADC, timers, PWM.
 */
void MCU_Init(void)
{
  Clock_setup();
  GPIO_Config();
  UART_setup();
#ifdef DISCOVERY
  ADC1_setup();
//  TIM1_setup(); // commutation timing setup is adjusted continuously by CL controller
  TIM2_setup();
  TIM4_setup();
  SPI_setup();
#endif
}

/**@}*/ // defgroup
