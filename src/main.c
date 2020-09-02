/**
  ******************************************************************************
  * @file main.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm8s.h"
#include "parameter.h" // app defines


/* Private defines -----------------------------------------------------------*/

#define UARTputs  uart_print // tmp this is sometimes a coding std no-no


#define ADC_N_CHANNELS 10       // [A0:A9]


/* Public variables  ---------------------------------------------------------*/

uint8_t TaskRdy;           // flag for timer interrupt for BG task timing

uint8_t Log_Level;


/* Private variables ---------------------------------------------------------*/

//static


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * home-made itoa function (16-bits only, hex-only)
 * seems Cosmic only provide atoi in stdlib and not itoa
 */
char * itoa(uint16_t u16in, char *sbuf, int base)
{
    int x;
    int shift = 16 - 4;	/* 4-bits to 1 nibble */
    uint16_t n16 = u16in;

    if (16 != base)
    {
        return NULL;
    }

    x = 0;
    while (x < 4 /* 4 nibbles in 16-bit word */ )
    {
        unsigned char c = (n16 >> shift) & 0x000F;

        if (c > 9)
        {
            c -= 10;
            c += 'A';
        }
        else
        {
            c += '0';
        }
        sbuf[x++] = c;
        shift -= 4;
    }
    sbuf[x] = 0;

    return sbuf;
}

/*
 * some of this bulk could be reduced with macros
*/
void GPIO_Config(void)
{
// OUTPUTS
// built-in LED
    GPIOD->ODR |= (1 << LED); //LED initial state is OFF (cathode driven to Vcc)
    GPIOD->DDR |= (1 << LED); //PD.n as output
    GPIOD->CR1 |= (1 << LED); //push pull output


// Controls the /SD pins of IR2104s on PC5, PC7, PG1
///////////  // tried E2, E0, D1 but E2 not work as output ... ???
// C5, C7, and G1 are CN2 pin 6, 8, 12 so 3 leads can go in one connector shell ;)
    GPIOC->ODR &=  ~(1<<5);
    GPIOC->DDR |=  (1<<5);
    GPIOC->CR1 |=  (1<<5);

    GPIOC->ODR &=  ~(1<<7);
    GPIOC->DDR |=  (1<<7);
    GPIOC->CR1 |=  (1<<7);

    GPIOG->ODR &=  ~(1<<1);
    GPIOG->DDR |=  (1<<1);
    GPIOG->CR1 |=  (1<<1);

////////////
// use PC6 (CN2-9) and PG0 (CN2-11) as test pins
    GPIOG->ODR &= ~(1<<0);
    GPIOG->DDR |=  (1<<0);
    GPIOG->CR1 |=  (1<<0);

    GPIOC->ODR |=  (1<<6); 				//  cathode of LED on PC6
    GPIOC->DDR |=  (1<<6);
    GPIOC->CR1 |=  (1<<6);

#if 0 // doesn't seem to matter ... these are set by TIM2 PWM API calls ??
// 3 PWM Channels
// T2.PWM.CH3
    GPIOA->ODR |=  (1<<3);  // PA3
    GPIOA->DDR |=  (1<<3);
    GPIOA->CR1 |=  (1<<3);
// T2.PWM.CH2
    GPIOD->ODR |=  (1<<3);  // PD3
    GPIOD->DDR |=  (1<<3);
    GPIOD->CR1 |=  (1<<3);
// T2.PWM.CH1
    GPIOD->ODR |=  (1<<4);  // PD4
    GPIOD->DDR |=  (1<<4);
    GPIOD->CR1 |=  (1<<4);
#endif
#if 0 // TIM1 CH1, CH2, CH3, CH4
// T1.PWM.CH1
    GPIOC->ODR |=  (1<<1);  // PC1
    GPIOC->DDR |=  (1<<1);
    GPIOC->CR1 |=  (1<<1);
// T1.PWM.CH2
    GPIOC->ODR |=  (1<<2);  // PC2
    GPIOC->DDR |=  (1<<2);
    GPIOC->CR1 |=  (1<<2);
// T1.PWM.CH3
    GPIOC->ODR |=  (1<<3);  // PC3
    GPIOC->DDR |=  (1<<3);
    GPIOC->CR1 |=  (1<<3);
// T1.PWM.CH4
    GPIOC->ODR |=  (1<<4);  // PC4
    GPIOC->DDR |=  (1<<4);
    GPIOC->CR1 |=  (1<<4);
#endif

// INPUTS
// PA4 as button input (Stop)
    GPIOA->DDR &= ~(1 << 4); // PA.4 as input
    GPIOA->CR1 |= (1 << 4);  // pull up w/o interrupts
// uses CN2.7 as GND

// PA6 as button input (B+)
    GPIOA->DDR &= ~(1 << 6); // PA.6 as input
    GPIOA->CR1 |= (1 << 6);  // pull up w/o interrupts
    GPIOA->DDR |= (1 << 5);  // PD.5 as output
    GPIOA->CR1 |= (1 << 5);  // push pull output
    GPIOA->ODR &= ~(1 << 5); // set pin off to use as gnd of button

// PE5 as button input (B-)
    GPIOE->DDR &= ~(1 << 5); // PE.5 as input
    GPIOE->CR1 |= (1 << 5);  // pull up w/o interrupts
    GPIOE->DDR |= (1 << 3);  // PE.3 as output
    GPIOE->CR1 |= (1 << 3);  // push pull output
    GPIOE->ODR &= ~(1 << 3); // set pin off to use as gnd of button


// UART2 D5: Rx, D6: Tx
    GPIOD->DDR &= ~(1 << 5); // PD.5 as input
    GPIOD->CR1 |= (1 << 5);  // pull up w/o interrupts
    GPIOD->DDR |= (1 << 6);  // PD.6 as output
    GPIOD->CR1 |= (1 << 6);  // push pull output
    GPIOD->ODR |= (1 << 6);  // use as hi-side of button


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
}

/*
 * http://embedded-lab.com/blog/starting-stm8-microcontrollers/24/
 */
void UART_setup(void)
{
    UART2_DeInit();

    UART2_Init(115200,
               UART2_WORDLENGTH_8D,
               UART2_STOPBITS_1,
               UART2_PARITY_NO,
               UART2_SYNCMODE_CLOCK_DISABLE,
               UART2_MODE_TXRX_ENABLE);

    UART2_Cmd(ENABLE);
}

/*
*  Send a message to the debug port (UART1).
*    (https://blog.mark-stevens.co.uk/2012/08/using-the-uart-on-the-stm8s-2/)
*/
void uart_print(char *message)
{
    char *ch = message;
    while (*ch)
    {
        UART2->DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
        while ( 0 == (UART2->SR & UART2_SR_TXE) ); //  Wait for transmission to complete.
        ch++;                               //  Grab the next character.
    }
}

/*
 * https://www.st.com/resource/en/application_note/cd00282842-rs232-communications-with-a-terminal-using-the-stm8sdiscovery-stmicroelectronics.pdf
 */
/*******************************************************************************
* Function Name  : SerialKeyPressed
* Description    : Test to see if a key has been pressed on the HyperTerminal
* Input          : - key: The key pressed
* Output         : None
* Return         : 1: Correct
*                  0: Error
*******************************************************************************/
u8 SerialKeyPressed(char *key)
{
    FlagStatus flag  ;
    /* First clear Rx buffer */
    flag = UART2_GetFlagStatus(UART2_FLAG_RXNE);
    if ( flag == SET)
    {
        *key = (char)UART2->DR;
        return 1;
    }
    else
    {
        return 0;
    }
}

/*******************************************************************************
* Function Name  : GetKey
* Description    : Get a key from the HyperTerminal
* Input          : None
* Output         : None
* Return         : The Key Pressed
*******************************************************************************/
char GetKey(void)
{
    char key = 0;
    /* Waiting for user input */
    while (1)
    {
        if (SerialKeyPressed((char*)&key)) break;
    }
    return key;
}


/*
 * https://community.st.com/s/question/0D50X00009XkbA1SAJ/multichannel-adc
 */
void ADC1_setup(void)
{
// Port B[0..7]=floating input no interr
// STM8 Discovery, all PortB pins are on CN3
//    GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_IN_FL_NO_IT); // all AIN pins setup explicityly with the GPIO init

    ADC1_DeInit();

// configured range of A/D input pins: CH0, CH1, CH2 to use as b-EMF sensors
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
              ADC1_CHANNEL_2,
              ADC1_PRESSEL_FCPU_D2,
              ADC1_EXTTRIG_TIM,      //  ADC1_EXTTRIG_GPIO
              DISABLE,               // ExtTriggerState
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_ALL,
              DISABLE);              // SchmittTriggerState

#ifdef ADC_EOCIE // interrupted ADC messes w/ motor
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
#endif

    /*
    A single conversion is performed for each channel starting with AIN0 and the data is stored
    in the data buffer registers ADC_DBxR.
    */
//ADC1_ConversionConfig(ADC1_CONVERSIONMODE_CONTINUOUS, ((ADC1_Channel_TypeDef)(ADC1_CHANNEL_0 | ADC1_CHANNEL_1)), ADC1_ALIGN_RIGHT);
    ADC1_ConversionConfig(ADC1_CONVERSIONMODE_CONTINUOUS,
                          ((ADC1_Channel_TypeDef)(ADC1_CHANNEL_0 | ADC1_CHANNEL_1 | ADC1_CHANNEL_2)),
                          ADC1_ALIGN_RIGHT);

//ADC1_DataBufferCmd(ENABLE);
    ADC1_ScanModeCmd(ENABLE); // Scan mode from channel 0 to 9 (as defined in ADC1_Init)

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion of all channels in sequence
    ADC1_StartConversion();
}

/*
 * Timer 1 Setup
 * from:
 *   http://www.emcu.it/STM8/STM8-Discovery/Tim1eTim4/TIM1eTIM4.html
 *
 * manual mode (ramp-up) commutation timing?
 * based on OTS ESC analysic, rampup start at
 *  pd = 0.008  sec
 *       0.003  sec  rampup to 3000 RPM
 *       0.004  sec  settle to 2500 RPM
 *       0.008  sec  1250 RPM slowest attainable after initial sync
 *
 *       0.0012 sec
*/
#ifdef CLOCK_16
#define TIM1_PRESCALER 8  //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
#define TIM1_PRESCALER 4  //    (1/8Mhz)  * 4 * 250 -> 0.000125 S
#endif

#define PWM_MODE  TIM1_OCMODE_PWM1

void TIM1_setup(void)
{
    const uint16_t T1_Period = TIM2_PWM_PD /* TIMx_PWM_PD */ ;  // 16-bit counter  ... 260 test

    CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER1, ENABLE);  // put this with clocks setup

    TIM1_DeInit();

    TIM1_TimeBaseInit(( TIM1_PRESCALER - 1 ), TIM1_COUNTERMODE_UP, T1_Period, 0); // ISR at and of "idle" time

    /* Channel 2 PWM configuration */
    TIM1_OC2Init( PWM_MODE,
                  TIM1_OUTPUTSTATE_ENABLE,
                  TIM1_OUTPUTNSTATE_ENABLE,
                  0,
                  TIM1_OCPOLARITY_HIGH,
                  TIM1_OCNPOLARITY_HIGH,
                  TIM1_OCIDLESTATE_RESET,
                  TIM1_OCNIDLESTATE_RESET);
    //   TIM2_OC2PreloadConfig(ENABLE); ??

    /* Channel 3 PWM configuration */
    TIM1_OC3Init( PWM_MODE,
                  TIM1_OUTPUTSTATE_ENABLE,
                  TIM1_OUTPUTNSTATE_ENABLE,
                  0,
                  TIM1_OCPOLARITY_HIGH,
                  TIM1_OCNPOLARITY_HIGH,
                  TIM1_OCIDLESTATE_RESET,
                  TIM1_OCNIDLESTATE_RESET);
// ?  TIM2_OC3PreloadConfig(ENABLE);

    /* Channel 4 PWM configuration */
    TIM1_OC4Init(PWM_MODE,
                 TIM1_OUTPUTSTATE_ENABLE,
                 0,
                 TIM1_OCPOLARITY_HIGH,
                 TIM1_OCIDLESTATE_RESET);
// TIM2_OC4PreloadConfig(ENABLE); // ???

    TIM1_CtrlPWMOutputs(ENABLE);

    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    TIM1_Cmd(ENABLE);
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
void TIM4_setup(void)
{
//    const uint8_t T4_Period = 32;     // Period =  256uS ... stable, any faster becomes jittery
//    const uint8_t T4_Period = 255;    // Period = 2.048mS
// set this for ~1mS to use in place of TIM1 as ramp timer
//const uint8_t T4_Period = 128;    // Period = 1.02mS
    const uint8_t T4_Period = 64;    // Period =  0.000512 S  (512 uS) ... double up on ADC ?

#ifdef CLOCK_16
    TIM4->PSCR = 0x07; // PS = 128  -> 0.0000000625 * 128 * p
#else
    TIM4->PSCR = 0x06; // PS =  64  -> 0.000000125  *  64 * p
#endif

    TIM4->ARR = T4_Period;

    TIM4->IER |= TIM4_IER_UIE; // Enable Update Interrupt
    TIM4->CR1 |= TIM4_CR1_CEN; // Enable TIM4
}

/*
 * Timers 2 3 & 5 are 16-bit general purpose timers
 *  Sets the commutation switching period.
 *
 *  @8Mhz, fMASTER period ==  0.000000125 S
 *   Timer Step:
 *     step = 1 / 8Mhz * prescaler = 0.000000125 * (2^6) = 0.000008 S
 */
// ideally, TIM3 prescaler would be tied to  BLDC_CT_SCALE

// now reduce the pre-scaler, while sub-tasking the TIM3 ISR

#ifdef CLOCK_16
#define TIM3_PSCR  0x03  // 2^3 == 8
#else
#define TIM3_PSCR  0x02  // 2^2 == 4
#endif


void TIM3_setup(uint16_t u_period)
{
    const uint16_t MAX_SWITCH_TIME = 0xfffe;
    const uint16_t MIN_SWITCH_TIME = 1;

    uint16_t period = u_period;  // uses all 3-bits of TIM3 prescaler

    if (period < MIN_SWITCH_TIME)
    {
        period = MIN_SWITCH_TIME;  // protect against setting a 0 period
    }

    if (period > MAX_SWITCH_TIME)
    {
        period = MAX_SWITCH_TIME; // lower limit
    }

// @8Mhz, fMASTER period ==  0.000000125 S
//  Timer Step:
//    step = 1 / 8Mhz * prescaler = 0.000000125 * (2^6) = 0.000008 S
    TIM3->PSCR = TIM3_PSCR;

    TIM3->ARRH = period >> 8;   // be sure to set byte ARRH first, see data sheet
    TIM3->ARRL = period & 0xff;

    TIM3->IER |= TIM3_IER_UIE; // Enable Update Interrupt
    TIM3->CR1 = TIM3_CR1_ARPE; // auto (re)loading the count
    TIM3->CR1 |= TIM3_CR1_CEN; // Enable TIM3
}

/*
 * http://embedded-lab.com/blog/starting-stm8-microcontrollers/13/
 * GN:  by default  microcontroller uses   internal 16MHz RC oscillator
 * ("HSI", or high-speed internal) divided by eight  as a clock source. This results in a base timer frequency of 2MHz.
 * Using this function just to show the library way to explicit clock setup.
 */
void clock_setup(void)
{
    CLK_DeInit();
#ifdef INTCLOCK
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
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
    CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER1, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
}


// hack, temp

static uint16_t Line_Count = 0;

extern uint16_t BLDC_OL_comm_tm;
extern uint16_t Back_EMF_R[];
extern uint16_t Back_EMF_F[];
extern uint16_t Back_EMF_F0_MA;
extern uint16_t Back_EMF_R0_MA;

extern uint16_t Back_EMF_F_tmp;
extern uint16_t Back_EMF_F0_MA_tmp;

/*
 * temp, todo better function
 */
void testUART(void)
{
    char sbuf[128] ;                     // am i big enuff?
    char cbuf[8] = { 0, 0 };

    sbuf[0] = 0;

    Line_Count  += 1;;

    strcat(sbuf, "(");
    itoa(Line_Count, cbuf, 16);
    strcat(sbuf, cbuf);
    strcat(sbuf, ")");

    strcat(sbuf, " CT=");
    itoa(BLDC_OL_comm_tm, cbuf, 16);
    strcat(sbuf, cbuf);

#if 1
// tmep ... log the test back-EMF falling-float transition
    strcat(sbuf, " bF0_ma=");
    itoa( Back_EMF_F0_MA_tmp,     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " bF0=");
    itoa( Back_EMF_F_tmp,     cbuf, 16);
    strcat(sbuf, cbuf);
#endif

    strcat(sbuf, " A0=");
    itoa(ADC1_GetBufferValue(0), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " A1=");
    itoa(ADC1_GetBufferValue(1), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " A2=");
    itoa(ADC1_GetBufferValue(2), cbuf, 16);
    strcat(sbuf, cbuf);
#if 0
    strcat(sbuf, " A8=");
    itoa(ADC1_GetBufferValue(8), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " A9=");
    itoa(ADC1_GetBufferValue(9), cbuf, 16);
    strcat(sbuf, cbuf);
#endif
#if 0
// A/D
    for (loop = 0; loop <  ADC_N_CHANNELS ; loop++)
    {
        strcat(sbuf, " A");
        cbuf[0] = 0x30 + loop;
        cbuf[1] = 0;
        strcat(sbuf, cbuf);
        strcat(sbuf, "=");

//itoa(AnalogInputs[loop], cbuf, 16);
        itoa( ADC1_GetBufferValue( loop ), cbuf, 16);
        strcat(sbuf, cbuf);
    }
#endif

    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
}

/*
 * Execution context is 'main()' so not to block ISR with ADC sampling.
 * TODO moving A/D acquisition to ramp-step (TIM3) to coordinate with PWM on/off cycling.
 * Servicing the UART (presently just a simply one-way logging stream)
 */
void Periodic_task(void)
{
    uint16_t duty_cycle = 0;
    char sbuf[16] = "DC=";
    char cbuf[8] = { 0, 0 };
    char key;

    /*
     * debug logging information can be "scheduled" by setting the level, which for now
     * simply designates number of reps ... spacing TBD? this task is now tied to
     * BLDC Update cycle ( about 1/2sec?)
     */
    if (Log_Level > 0)
    {
// allow log to sticky-on
        if (Log_Level < 255)
        {
            Log_Level -= 1;
        }

        testUART();// tmp test
    }

    if (SerialKeyPressed((char*)&key))
    {
        if (key == '+')
        {
            disableInterrupts();
            duty_cycle = BLDC_PWMDC_Plus();
            enableInterrupts();
            UARTputs("+++");
        }
        else if (key == '-')
        {
            disableInterrupts();
            duty_cycle = BLDC_PWMDC_Minus();
            enableInterrupts();
            UARTputs("---");
        }

        itoa(duty_cycle, cbuf, 16);
        strcat(sbuf, cbuf);
        strcat(sbuf, "\r\n");
        UARTputs(sbuf);
    }
}

/*
 * mainly looping
 */
main()
{
    clock_setup();
    GPIO_Config();
    UART_setup();
    ADC1_setup();
    TIM1_setup();
    TIM4_setup();

    BLDC_Stop();
    Log_Level = 0;

    enableInterrupts(); // interrupts are globally disabled by default

    uart_print("\033c"); // sends ANSI code to  clear the serial terminal
    uart_print("Program Startup....... \r\n");

    while(1)
    {
        uint16_t debounce_ct;
        uint16_t ainp;
//  button input either button would transition from OFF->RAMP
        if (! (( GPIOA->IDR)&(1<<4)))
        {
//            while( ! (( GPIOA->IDR)&(1<<4)) ); // no concern for debounce for a stop switch
            disableInterrupts();
            BLDC_Stop();
            enableInterrupts();
            // reset line couunter of the serial-port logger
            Line_Count = -1;
        }

        if (! (( GPIOA->IDR)&(1<<6)))
        {
            while( ! (( GPIOA->IDR)&(1<<6)) ); // wait for debounce (sorta works)
//  TEST DEV ONLY: manual adjustment of commutation cycle time
            disableInterrupts();
            BLDC_Spd_inc();
            enableInterrupts();
        }

        if ( ! (( GPIOE->IDR)&(1<<5)))
        {
            while( ! (( GPIOE->IDR)&(1<<5)) ) {;} // wait for debounce (sorta works)
//  TEST DEV ONLY: manual adjustment of commutation cycle time
            disableInterrupts();
            BLDC_Spd_dec();
            enableInterrupts();
        }

// while( FALSE == TaskRdy )
        if ( FALSE == TaskRdy )  // idk .. don't block here in case there were actually some background tasks to do
        {
            nop();
        }
        else
        {
            // theoretically no time-limit for the periodic task? ... periodic task should be interruptible
            TaskRdy = FALSE;
            Periodic_task();
        }
    } // while 1
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
