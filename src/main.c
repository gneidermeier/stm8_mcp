/**
  ******************************************************************************
  * @file main.c
  * @brief This file contains the main function for this template.
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date 15-March-2011
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "stm8s.h"

/* Private defines -----------------------------------------------------------*/
#define N_PHASES  3 
#define OL_DEV

/* Public variables  ---------------------------------------------------------*/

u8 forceCommutation; // switch input to test "commutation" logic
u8 latch_T4_is_zero;
u8 TaskRdy;           // flag for timer interrupt for BG task timing
u16 T4counter = 0;
//unsigned int T4_count_pd = 20; // LED0 @ 10 Hz so 20 steps of 5mS for DC?
u16 T4_count_pd = 65; // seems to be limited to around 70 counts ?? wtf
u16 duty_cycle;


/* Private variables ---------------------------------------------------------*/
const u8 COMMUATION_PD_ADJ_AINCH = 9;   // adjust pot to set duration of "phase" period

const unsigned  char LED = 0;
s8 buttonState = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void GPIO_Config(void)
{ 
// OUTPUTS
// built-in LED    
    GPIOD->DDR |= (1 << LED); //PD.n as output
    GPIOD->CR1 |= (1 << LED); //push pull output

// 3 "ENABLES" for the 3 fields on CN2 1-3-5  PE5, PC2, PC4
    GPIOE->ODR &=  ~(1<<5); 				//  PE5
    GPIOE->DDR |=  (1<<5);
    GPIOE->CR1 |=  (1<<5);

    GPIOC->ODR &=  ~(1<<2); 				//  PC2
    GPIOC->DDR |=  (1<<2);
    GPIOC->CR1 |=  (1<<2);

    GPIOC->ODR &=  ~(1<<4); 				//  PC4
    GPIOC->DDR |=  (1<<4);
    GPIOC->CR1 |=  (1<<4);

// test LED C6+C7
    GPIOC->ODR &= ~(1<<7); 				//  LED/OUT/CH3.TIM2.PWM on PC7
    GPIOC->DDR |=  (1<<7);
    GPIOC->CR1 |=  (1<<7);

    GPIOC->ODR |=  (1<<6); 				//  cathode of LED on C6 (to VCC)
    GPIOC->DDR |=  (1<<6);
    GPIOC->CR1 |=  (1<<6);


// 3 PWM Channels 
// PA4 
    GPIOA->ODR &= ~(1<<4); 				//  drive  low (GND for LED on PDx)
    GPIOA->DDR |=  (1<<4);
    GPIOA->CR1 |=  (1<<4);

    GPIOA->ODR |=  (1<<3); 				//  LED/OUT/CH1.TIM2.PWM on PA3
    GPIOA->DDR |=  (1<<3);
    GPIOA->CR1 |=  (1<<3);

// PD2
    GPIOD->ODR &= ~(1<<2); 				//  drive  low (GND for LED on PDx)
    GPIOD->DDR |=  (1<<2);
    GPIOD->CR1 |=  (1<<2);

    GPIOD->ODR |=  (1<<3); 				//  VDD for LED/OUT/CH2.TIM2.PWM on PD3
    GPIOD->DDR |=  (1<<3);
    GPIOD->CR1 |=  (1<<3);

// PD5
    GPIOD->ODR &= ~(1<<5); 				//  drive  low (GND for LED on PDx)
    GPIOD->DDR |=  (1<<5);
    GPIOD->CR1 |=  (1<<5);

    GPIOD->ODR |=  (1<<4); 				//  VDD for LED/OUT/CH3.TIM2.PWM on PD4
    GPIOD->DDR |=  (1<<4);
    GPIOD->CR1 |=  (1<<4);

// INPUTS
// PA5/6 as button input 
    GPIOA->DDR &= ~(1 << 6); // PB.2 as input
    GPIOA->CR1 |= (1 << 6);  // pull up w/o interrupts
    GPIOA->DDR |= (1 << 5);  // PD.n as output
    GPIOA->CR1 |= (1 << 5);  // push pull output
    GPIOA->ODR &= ~(1 << 5); // set "off" (not driven) to use as hi-side of button

// PE3/2 as test switch input 
    GPIOE->DDR &= ~(1 << 2); // PE.2 as input
    GPIOE->CR1 |= (1 << 2);  // pull up w/o interrupts
    GPIOE->DDR |= (1 << 3);  // PD.n as output
    GPIOE->CR1 |= (1 << 3);  // push pull output
    GPIOE->ODR |= (1 << 3);  // use as hi-side of button


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
 * http://www.electroons.com/blog/stm8-tutorials-3-adc-interfacing/
 */
unsigned int readADC1(unsigned int channel) 
{
    unsigned int csr = 0;
    unsigned int val = 0;
    //using ADC in single conversion mode
    ADC1->CSR  = 0;                // GN: singly read each channel for now
    ADC1->CSR |= ((0x0F)&channel); // select channel

//Single scan mode is started by setting the ADON bit while the SCAN bit is set and the CONT bit is cleared.
    ADC1->CR1 &= ~(1<<1); // CONT OFF
    ADC1->CR2 |=  (1<<1); // SCAN ON 

    ADC1->CR2 |= (1<<3); // Right Aligned Data
    ADC1->CR1 |= (1<<0); // ADC ON 
    ADC1->CR1 |= (1<<0); // ADC Start Conversion

    while(((ADC1->CSR)&(1<<7))== 0); // Wait till EOC

/*  correct way to clear the EOC flag in continuous scan mode is to load a byte in the ADC_CSR register from a RAM variable, clearing the EOC flag and reloading the last channel number for the scan sequence */
    csr = ADC1->CSR; // GN:   carefully clear EOC!
    csr &= ~(1<<7); 
    ADC1->CSR = csr;
/*
         val |= (unsigned int)ADC1->DRL;
         val |= (unsigned int)ADC1->DRH<<8;
*/
    val = ADC1_GetBufferValue(channel); // AINx

    ADC1->CR1 &= ~(1<<0); // ADC Stop Conversion

    val &= 0x03ff;
    return (val);
}


/**
  * @brief Validation firmware main entry point.
  * @par Parameters:
  * None
  * @retval void None
  *   GN: from UM0834 PWM example
  */
#define CCR1_Val  ((u16)500) // Configure channel 1 Pulse Width
#define CCR2_Val  ((u16)250) // Configure channel 2 Pulse Width
#define CCR3_Val  ((u16)750) // Configure channel 3 Pulse Width

void PWM_Config(uint16_t uDC, uint16_t *p_DC)
{
    uint16_t TIM2_pulse_0 ;// = *(p_DC + 0);
    uint16_t TIM2_pulse_1 ;// = *(p_DC + 1);
    uint16_t TIM2_pulse_2 ;// = *(p_DC + 2);
    TIM2_pulse_0 = \
                   TIM2_pulse_1 = \
                                  TIM2_pulse_2 = uDC;
//return; // tmp test

    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    /* Set TIM2 Frequency to 2Mhz */
    TIM2_TimeBaseInit(TIM2_PRESCALER_1, 999  );
//TIM2_TimeBaseInit(TIM2_PRESCALER_1, 10  );   // appears 1 cycle = 1uS, so fMASTER period == @ 0.5uS

    /* Channel 1 PWM configuration */
//TIM2_Pulse = CCR1_Val;
    TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_0 /* CCR1_Val */, TIM2_OCPOLARITY_LOW );
//TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 5, TIM2_OCPOLARITY_LOW );
    TIM2_OC1PreloadConfig(ENABLE);


    /* Channel 2 PWM configuration */
//TIM2_Pulse = CCR2_Val;
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_1, TIM2_OCPOLARITY_LOW );
    TIM2_OC2PreloadConfig(ENABLE);


    /* Channel 3 PWM configuration */
//TIM2_Pulse = CCR3_Val;
    TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_2, TIM2_OCPOLARITY_LOW );
    TIM2_OC3PreloadConfig(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);

    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);
}

/*
 * Configure Timer 4 as general purpose fixed time-base reference
 * Timer 4 & 6 are 8-bit basic timers
 *
 *   https://lujji.github.io/blog/bare-metal-programming-stm8/
 *
 * Default setup for STM8S Discovery is 2Mhz HSI ... leave period @ 5mS for now I guesss.....
 */
void TIM4_Config(void)
{
    /* Prescaler = 128 */
    TIM4->PSCR = 0x07; // 0b00000111;
    /* Period = 5ms */
    TIM4->ARR = 77;
    TIM4->IER |= TIM4_IER_UIE; // Enable Update Interrupt
    TIM4->CR1 |= TIM4_CR1_CEN; // Enable TIM4
}

// Timers 2 3 & 5 are 16-bit general purpose timers
void TIMx_Config(u16 period)
{
    // GN: trying to get this into a range where LED flashing is mostly visible (leave pre-scale fixed for now)
    TIM3->PSCR = 0x08;

    if (period > 1023)
    {
        period = 1023; // cap it to 10-bit so that is correspond to A/D channel
    }
    // this limit is hackaroun for funky slide-pot
    if (period < 10)
    {
        period = 10;
    }

    TIM3->ARRH = period >> 8;   // be sure to set byte ARRH first, see data sheet  
    TIM3->ARRL = period & 0xff;

    TIM3->IER |= TIM3_IER_UIE; // Enable Update Interrupt
    TIM3->CR1 = TIM3_CR1_ARPE; // auto (re)loading the count
    TIM3->CR1 |= TIM3_CR1_CEN; // Enable TIM4
}

void Timer_Config(void)
{
    TIM4_Config();
    TIMx_Config( 128 );
}

/*
 * returns AIN channel corresponing to selected phase
 */
u8 updateChannels(s8 selectedChannel)
{
    unsigned const int AIN9 = 9;  // PE6
    unsigned const int AIN8 = 8;  // PE7
    unsigned const int AIN6 = 6;  // PB6

    u8 AINch;

// set the lo-sides (HI) to turn off LED (pulls cathodes hi)...
// PA4  (VDD for LED/OUT/CH1.TIM2.PWM on PA3)
    GPIOA->ODR |= (1<<4);
// PD2  (VDD for LED/OUT/CH2.TIM2.PWM on PD3)
    GPIOD->ODR |= (1<<2);
// PD5  (VDD for LED/OUT/CH3.TIM2.PWM on PD4)
    GPIOD->ODR |= (1<<5);

    switch (selectedChannel)
    {
    case 0:
        AINch = AIN9;
        GPIOA->ODR &= ~(1<<4);
        break;
    case 1:
        AINch = AIN8;
        GPIOD->ODR &= ~(1<<2);
        break;
    case 2:
        AINch = AIN6;
        GPIOD->ODR &= ~(1<<5);
        break;
    default:
        break;
    }

    return AINch;
}

/*
 * determine duty-cycle counts based on analog input (ratio for 10 bit A/d)
 */
void updateLED0(u16 dwell)
{
    u16 dc_counts =  ( T4_count_pd * dwell ) / 1024;    // 10-bit A/D input

    if ( T4counter < dc_counts )
    {
        GPIOD->ODR |= (1 << LED);
    }
    else
    {
        GPIOD->ODR &= ~(1 << LED);
    }
}

/*
 * Tests the input analog reading to determine the "zero crossing"
 * Test LED output shows trim adjustment (solid when trimmed to neutral/half)
 */
u8 getZeroCross(u16 ainp, u16 counter_period)
{
    // set the LED off time (nearly 90% of the period) so that when not
    // trimmed, the LED will be flashing, albeit at a dimmed intensity
    u16 dwell_counter = (counter_period * 7) / 8;

// commutation experiment
    u8 zeroX = FALSE;

    if (ainp >= (512-50) && ainp <= (512+50) )
    {
        zeroX = TRUE;
    }

    if (FALSE != zeroX)
    {
// show "zero crossing" by making the test LED briter
//        dwell_counter = (counter_period * 1) / 8;     //  1/8 off time
        dwell_counter = 1 ; // practically nearly solid briteness
    }

    if ( T4counter < dwell_counter )
    {
        GPIOC->ODR |= (1 << 7); // drive hi i.e. LED off, as this is lo (cathode)side
    }
    else
    {
        GPIOC->ODR &= ~(1 << 7);
    }

    return zeroX;
}

/*
 * 
 */
void periodic_task(void)
{
    u16 a_input;
    u8 zero_x = FALSE;

    u8 AIN_channel = updateChannels(buttonState);

#ifndef OL_DEV
    a_input =  readADC1( AIN_channel );
#else
    a_input = readADC1( COMMUATION_PD_ADJ_AINCH );
    TIMx_Config( a_input );
#endif

    updateLED0(a_input);

// duty_cycle global to feed it to PWM config
    duty_cycle = a_input;

    zero_x = getZeroCross(a_input, T4_count_pd);

//button input test enable commutation
    forceCommutation = FALSE;
    if ( GPIOE->IDR & (1<<2) )   //if closed the switch
    {
        forceCommutation = TRUE;
    }

#ifdef OL_DEV
    zero_x = TRUE;
    forceCommutation = TRUE;
#endif

    if (FALSE != forceCommutation )
    {
        // do "commutation" at "time 0"
        if ( FALSE != latch_T4_is_zero )
        {
            latch_T4_is_zero = FALSE; // lame ... need to use a dedicated timer/counter

            if (FALSE != zero_x)
            {
                buttonState += 1;
            }

            if (buttonState >= N_PHASES)
            {
                buttonState = 0;
            }
            else  if (buttonState < 0)
            {
                buttonState = (N_PHASES - 1);
            }
        }
    }
}

/*
 * mainly looping
 */
main()
{
    uint16_t duty_cycles[N_PHASES] =
    {
        CCR1_Val, CCR2_Val, CCR3_Val
    } ;


    GPIO_Config();

    PWM_Config( duty_cycles[0], &duty_cycles[0] );

    Timer_Config();

    // Enable interrupts (no, really). Interrupts are globally disabled by default
    enableInterrupts();


    while(1)
    {
//  button input
        if (! (( GPIOA->IDR)&(1<<6)))
        {
            while( ! (( GPIOA->IDR)&(1<<6)) ); // wait for debounce

// WIP ... reconfig PWM only on button push.
// Before switch "channel":
//   store duty cycle of the presently selected  channel
/*
            duty_cycles[buttonState] = duty_cycle;   
*/
// let (ALL) PWM channels update to present DC setting
            PWM_Config(duty_cycle, &duty_cycles[0]);

            if (! forceCommutation)  // tmp test "commutation" enabled by switch/button
            {
                buttonState += 1;
                if (buttonState >= N_PHASES)
                {
                    buttonState = 0;
                }
            }
        }

// while( FALSE == TaskRdy )
        if ( FALSE == TaskRdy )  // idk .. don't block here in case there were actually some background tasks to do 
        {
            nop();
        }
        else
        {
            TaskRdy = TRUE;
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
