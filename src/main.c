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

// minimum PWM DC duration (manual adjustment)


/* Public variables  ---------------------------------------------------------*/
u8 Duty_cycle_pcnt_LED0;
u8 TaskRdy;           // flag for timer interrupt for BG task timing

uint16_t A0 = 0x1234;//tmp
uint16_t A1 = 0xabcd;//tmp


/* Private variables ---------------------------------------------------------*/
const u8 AINCH_COMMUATION_PD = 9;   // adjust pot to set "commutation" period
const u8 AINCH_PWM_DC        = 8;   // adjust pot to PWM D.C. on FET outputs


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

void GPIO_Config(void)
{
// OUTPUTS
// built-in LED
    GPIOD->ODR |= (1 << LED); //LED initial state is OFF (cathode driven to Vcc)
    GPIOD->DDR |= (1 << LED); //PD.n as output
    GPIOD->CR1 |= (1 << LED); //push pull output


// HB driver "ENABLES" (/SD input pin of IR2104) on PC5, PC7, PG1
///////////  // tried E2, E0, D1 but E2 not work as output ... ??? 
// C5, C7, and G1 are CN2 pin 6,8 ,12 so 3 leads can go in one connector shell ;)
// also this is to expose PC2 which is TIM1 CH2 output.
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
// test LED  
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

// INPUTS
// PA4 as button input 
    GPIOA->DDR &= ~(1 << 4); // PA.6 as input
    GPIOA->CR1 |= (1 << 4);  // pull up w/o interrupts

// PA5/6 as button input 
    GPIOA->DDR &= ~(1 << 6); // PA.6 as input
    GPIOA->CR1 |= (1 << 6);  // pull up w/o interrupts
    GPIOA->DDR |= (1 << 5);  // PD.5 as output
    GPIOA->CR1 |= (1 << 5);  // push pull output
    GPIOA->ODR &= ~(1 << 5); // set pin off to use as gnd of button

// PE5 as button input 
    GPIOE->DDR &= ~(1 << 5); // PE.5 as input
    GPIOE->CR1 |= (1 << 5);  // pull up w/o interrupts
    GPIOC->DDR |= (1 << 2);  // PC.2 as output
    GPIOC->CR1 |= (1 << 2);  // push pull output
    GPIOC->ODR &= ~(1 << 2); // set pin off to use as gnd of button


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

// PB.6 AIN6
    GPIOB->DDR &= ~(1 << 6);  // PB.6 as input
    GPIOB->CR1 &= ~(1 << 6);  // floating input
    GPIOB->CR2 &= ~(1 << 6);  // 0: External interrupt disabled   ???

// PB.0 AIN0
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
void UARTputs(char *message)
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
 * https://community.st.com/s/question/0D50X00009XkbA1SAJ/multichannel-adc
 */
void ADC1_setup(void)
{
// Port B[0..7]=floating input no interr
// STM8 Discovery, all PortB pins are on CN3
    GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_IN_FL_NO_IT);

    ADC1_DeInit();

// this example, single channel polling :
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
              ADC1_CHANNEL_9,
              ADC1_PRESSEL_FCPU_D18,
              ADC1_EXTTRIG_TIM,   //  ADC1_EXTTRIG_GPIO
              DISABLE,
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_ALL,
              DISABLE);

//    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);

    ADC1_ScanModeCmd(ENABLE); // Scan mode from channel 0 to 9 (as defined in ADC1_Init)

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);
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
#define TIM1_PRESCALER 4
#else
#define TIM1_PRESCALER 2
#endif

void TIM1_setup(void)
{
    const uint16_t T1_Period = 255; //     0.000064 S     16-bit counter 
    CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER1 , ENABLE);     
    TIM1_DeInit();

//fCK_CNT = fCK_PSC/(PSCR[15:0]+1) 

// @8Mhz  the ISR overruns ... is taking ~50 uS because of BLDC_Step()	
    TIM1_TimeBaseInit(( TIM1_PRESCALER - 1 ), TIM1_COUNTERMODE_DOWN, T1_Period, 0);

    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    TIM1_Cmd(ENABLE);
}

/*
 * Configure Timer 4 as general purpose fixed time-base reference
 * Timer 4 & 6 are 8-bit basic timers
 *
 *   https://lujji.github.io/blog/bare-metal-programming-stm8/
 *
 * Default setup for STM8S Discovery is 2Mhz HSI ... leave period @ 5mS for now I guesss.....
 */
void timer_config_task_rate(void)
{
    const uint8_t T4_Period = 255;     // @16Mhz  Period = 2.1ms 
#ifdef CLOCK_16
    TIM4->PSCR = 0x07; // Prescaler = 128
#else
    TIM4->PSCR = 0x06; // Prescaler = 64
#endif

    TIM4->ARR = T4_Period;

    TIM4->IER |= TIM4_IER_UIE; // Enable Update Interrupt
    TIM4->CR1 |= TIM4_CR1_CEN; // Enable TIM4
}

// Timers 2 3 & 5 are 16-bit general purpose timers
/*
 *  Sets the open-loop commutation switching period.
 *  Input: [0:1023]   (input may be set from analog trim-pot for test/dev)

 * @2Mhz, fMASTER period ==  0.5uS
 *  Timer Step: 
 *    step = 1/2Mhz * prescaler = 0.0000005 * (2^5) = 0.000016 seconds 

 *  1/2Mhz = 0.0000005
 *  1 Count Time = 0.0000005 * (2^5) = 0.000016 Sec
 *    125 counts * 0.000016 -> 500Hz
 *   1014 counts * 0.000016 ->  60Hz
 *
 *  1016:    0.016256 measure 0.0165 sec. ( precision 0.5mS at this range)
 *    12:         210 uS
 *    11:   wth ???????????
 */
void timer_config_channel_time(uint16_t u_period)
{
// 0x02F0 experimental
    const uint16_t MAX_SWITCH_TIME = 0xfffe;
    const uint16_t MIN_SWITCH_TIME = 1;

    uint16_t period = u_period;

    if (period < MIN_SWITCH_TIME)
    {
        period = MIN_SWITCH_TIME;  // protect against setting a 0 period
    }

    if (period > MAX_SWITCH_TIME) 
    {
        period = MAX_SWITCH_TIME; // lower limit 
    }

#ifdef CLOCK_16
    TIM3->PSCR = 0x07; // 128 ......    @ 16Mhz -> 1 bit-time == 0.000008 S
#else
    TIM3->PSCR = 0x06; //  64 ......    @  8Mhz ->               0.000008
#endif

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
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, DISABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
}

/*
 * periodic task is timed to 5mS timer, but stack context is
 * 'main()' and not in the interrupt
 */ 
void periodic_task(void)
{
    const uint16_t PWM_DC_MIN = 20;   // (10/125)-> 8%

    uint16_t period;
    uint16_t pwm_dc_count;

// todo, to synchronize A/D reading w/ the PWM  for proper coordintion w/  zero-cross events?

// ADON = 1 for the 2nd time => starts the ADC conversion of all channels in sequence
    ADC1_StartConversion();

// Wait until the conversion is done (no timeout => ... to be done)
    while ( ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET );

//    A1 = ADC1_GetConversionValue();
    A0 = ADC1_GetBufferValue( AINCH_PWM_DC );
    A1 = ADC1_GetBufferValue( AINCH_COMMUATION_PD );

    ADC1_ClearFlag(ADC1_FLAG_EOC);


// open-loop commutation switch period on TIMER 3 - passed period parameter uses ainput range [0:1023] 
    period = A1 >> 1; // divide A/D range [0:1023] -> [0:511] to match planned scale-facgor of 2 for comm. time

#if 0  // if MANUAL 
    timer_config_channel_time( period * 2);
#endif


// use LED0 for visual indication of period/A1 - ainput to ratio of arbitrary period
    Duty_cycle_pcnt_LED0 = (TIM2_COUNT_LED0_PD / 2) * period / (1024/2); //  divide out factor of 2 so that (80/2 * 1024) fit in 16-bit 

/// TODO: only use this for test ... needs to be on a +/- button
    // divide out factor of 2 so that (126/2 * 1024) fit in 16-bit
    pwm_dc_count = (TIM2_PWM_PD / 2) * A0 / (1024/2);

#if 0 // ifdef MANUAL_LIMITS ?
/* for the manual open-loop pwm adjustment, apply some limits */
    if (pwm_dc_count < PWM_DC_MIN)
    {
        pwm_dc_count = PWM_DC_MIN;
    }
    else if (pwm_dc_count > (TIM2_PWM_PD - PWM_DC_MIN) )
    {
        pwm_dc_count = (TIM2_PWM_PD - PWM_DC_MIN);
    }
#endif

    Manual_uDC = pwm_dc_count;
}


// hack, temp
extern uint16_t BLDC_OL_comm_tm;

/*
 * temp, todo better function
 */
void testUART(void)
{
    static unsigned char cnt = 0x30;
    char sbuf[64] ;                     // am i big enuff?
    char cbuf[8] = { 0, 0 };

    cnt = cnt < 126 ? cnt + 1 : 0x30;
    sbuf[0] = 0;

    strcat(sbuf, "hello");
    cbuf[0] = cnt;
    cbuf[1] = 0;
    strcat(sbuf, cbuf);

    strcat(sbuf, " A0= ");
    itoa(A0, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " A1= ");
    itoa(A1, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " _OL_comm_tm= ");
    itoa(BLDC_OL_comm_tm, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " global_uDC= ");
    itoa(global_uDC, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
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
    timer_config_task_rate(); // fixed at 5mS

    BLDC_Stop();

    enableInterrupts(); // Enable interrupts . Interrupts are globally disabled by default


    while(1)
    {
        uint16_t debounce_ct;
        uint16_t ainp;
//  button input either button would transition from OFF->RAMP
        if (! (( GPIOA->IDR)&(1<<4)))
        {
//            while( ! (( GPIOA->IDR)&(1<<4)) ); // no concern for debounce for a stop switch
                BLDC_Stop();
        }

        if (! (( GPIOA->IDR)&(1<<6)))
        {
            while( ! (( GPIOA->IDR)&(1<<6)) ); // wait for debounce (sorta works)

            BLDC_Spd_inc();
            testUART();// tmp test
        }

        if ( ! (( GPIOE->IDR)&(1<<5)))
        {
            while( ! (( GPIOE->IDR)&(1<<5)) ){;} // wait for debounce (sorta works)

            BLDC_Spd_dec();	
            testUART();// tmp test
        }

// while( FALSE == TaskRdy )
        if ( FALSE == TaskRdy )  // idk .. don't block here in case there were actually some background tasks to do
        {
            nop();
        }
        else
        {
            TaskRdy = FALSE;
            periodic_task();
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
