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

#include "parameter.h" // app defines


/* Private defines -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/
u8 duty_cycle_pcnt_20ms;
u8 TaskRdy;           // flag for timer interrupt for BG task timing


/* Private variables ---------------------------------------------------------*/
const u8 AINCH_COMMUATION_PD = 9;   // adjust pot to set "commutation" period
const u8 AINCH_PWM_DC        = 8;   // adjust pot to PWM D.C. on FET outputs


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void GPIO_Config(void)
{ 
// OUTPUTS
// built-in LED    
    GPIOD->DDR |= (1 << LED); //PD.n as output
    GPIOD->CR1 |= (1 << LED); //push pull output
    GPIOD->ODR |= (1 << LED); //LED initial state is OFF (cathod driven to Vcc)	

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
void PWM_Config(uint16_t uDC)
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

    /* Set TIM2 Frequency to 2Mhz ... and period to ?    ( @2Mhz, fMASTER period == @ 0.5uS) */
//    TIM2_TimeBaseInit(TIM2_PRESCALER_1, 999  ); // PS==1, 999   ->  2khz (period == .000500)
    TIM2_TimeBaseInit(TIM2_PRESCALER_1, ( TIM2_PWM_PD - 1 ) ); // PS==1, 499   ->  4khz (period == .000250)
//    TIM2_TimeBaseInit(TIM2_PRESCALER_1, 249  ); // PS==1, 249 ->  8khz (period == .000125)

    /* Channel 1 PWM configuration */
    TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_0, TIM2_OCPOLARITY_LOW );
    TIM2_OC1PreloadConfig(ENABLE);


    /* Channel 2 PWM configuration */
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_1, TIM2_OCPOLARITY_LOW );
    TIM2_OC2PreloadConfig(ENABLE);


    /* Channel 3 PWM configuration */
    TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_2, TIM2_OCPOLARITY_LOW );
    TIM2_OC3PreloadConfig(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);

    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);

#if 1
// GN: tmp test
    TIM2->IER |= TIM2_IER_UIE; // Enable Update Interrupt 
#endif
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
    /* Prescaler = 128 */
    TIM4->PSCR = 0x07; // 0b00000111;
    /* Period = 5ms */
    TIM4->ARR = 77;
    TIM4->IER |= TIM4_IER_UIE; // Enable Update Interrupt
    TIM4->CR1 |= TIM4_CR1_CEN; // Enable TIM4
}

// Timers 2 3 & 5 are 16-bit general purpose timers
/*
 * Control "Enable" signal of each channel, sequencing thru each of 
 *  3 phases. (this was for DRV8813 but we could still do an "Enable" 
 * on each channel thus allowing PWM to be controlled independently - PWM basically just left "ON")
 * Let the timer3 period be dynamically adjusted to the "speed pot"
 * Presently, to set this up to allow "RPM" range ruffly 60-400 Hz.  ***
 * (***actual RPM would be calculated from cycle-time reduced by factor of 3 since 3 phases -> 1 RPM)
 *
 *  1 Count Time = 0.0000005 * 32 = 0.000016 Sec
 *  (1/500) == 125 counts * 0.000016 
 *   (1/60) == 1014 counts * 0.000016
 *
 *   hmmm this is dumb ... forcing range of channel timer/counter to that of anlg input 
 */
void timer_config_channel_time(u16 period)
{
    if (period < 1){
        period = 1;  // protect against setting a 0 period
    }
//period=78;
TIM3->PSCR = 0x05;  // PSC==5 -> 1.25mS
//period=936;
TIM3->PSCR = 0x05;  // PSC==5 -> 15mS
    //period=936+78;//=1014;
    TIM3->PSCR = 0x05;  // PSC==5 -> 16.125mS

    TIM3->ARRH = period >> 8;   // be sure to set byte ARRH first, see data sheet  
    TIM3->ARRL = period & 0xff;

    TIM3->IER |= TIM3_IER_UIE; // Enable Update Interrupt
    TIM3->CR1 = TIM3_CR1_ARPE; // auto (re)loading the count
    TIM3->CR1 |= TIM3_CR1_CEN; // Enable TIM4
}

/*
 * 
 */
//u16 a_input; // tmp
void periodic_task(void)
{
    u16 period;
    u16 a_input;
    a_input = readADC1( AINCH_COMMUATION_PD );
//a_input = 1014;
    period = a_input; // the timer pre-scale is set such that 10-bit range of the ainput 
    timer_config_channel_time( period );

// 8-bit voodoo ... divide out factor of 2 so that (80/2 * 1024) fit in 16-bit :(
    duty_cycle_pcnt_20ms =   (TIM2_T20_MS/2) * a_input / (1024/2);
}

/*
 * mainly looping
 */
main()
{
    GPIO_Config();

// initialize to 50% DC (just because) 
    PWM_Config( TIM2_PWM_PD / 2 );

    timer_config_task_rate(  ); // fixed at 5mS 

    timer_config_channel_time( 1 /* value doesn't matter, is periodically reconfigured anyway */);

    enableInterrupts(); // Enable interrupts . Interrupts are globally disabled by default


    while(1)
    {
        u16 ainp;
        u16 pwm_dc_count;
//  button input
        if (! (( GPIOA->IDR)&(1<<6)))
        {
            while( ! (( GPIOA->IDR)&(1<<6)) ); // wait for debounce

// WIP ... reconfig PWM on button push.
            ainp = readADC1( AINCH_PWM_DC );

// 8-bit voodoo ... divide out factor of 8 so that (500/8 * 1024) fit in 16-bit :(
            pwm_dc_count = (TIM2_PWM_PD/8) * ainp / (1024/8);
            PWM_Config( pwm_dc_count );
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
