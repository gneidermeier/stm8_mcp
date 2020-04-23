/**
  ******************************************************************************
  * @file main.c
  * @brief support functions for the BLDC motor control
  * @author Neidermeier
  * @version 
  * @date March-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "parameter.h" // app defines

/* Private defines -----------------------------------------------------------*/

// see define of TIM2 PWM PD ... it set for 125uS @ clk 2Mhz
//#define PWM_TPRESCALER  TIM2_PRESCALER_1 //
// @ 8 Mhz
#define PWM_TPRESCALER  TIM2_PRESCALER_8 // 125 uS

#define PWM_DC_MIN 30
#define PWM_DC_MAX (TIM2_PWM_PD - 30)

// nbr of steps required to commutate 3 phase
#define N_CSTEPS   6


/* Public variables  ---------------------------------------------------------*/
uint16_t TIM2_pulse_0 ;
uint16_t TIM2_pulse_1 ;
uint16_t TIM2_pulse_2 ;


/* Private variables ---------------------------------------------------------*/
uint16_t global_uDC;
u8 bldc_step = 0;

/* Private function prototypes -----------------------------------------------*/
void PWM_Config(void);
void bldc_move(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
void PWM_Set_DC(uint16_t pwm_dc)
{
    global_uDC = pwm_dc;
}

void PWM_set_outputs(u8 state0, u8 state1, u8 state2)
{
    TIM2_pulse_0 = state0 != 0 ? global_uDC : 0;
    TIM2_pulse_1 = state1 != 0 ? global_uDC : 0;
    TIM2_pulse_2 = state2 != 0 ? global_uDC : 0;

    PWM_Config();
}

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   GN: from UM0834 PWM example
  */
void PWM_Config(void)
{
    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    /* Set TIM2 Frequency to 2Mhz ... and period to ?    ( @2Mhz, fMASTER period == @ 0.5uS) */
    TIM2_TimeBaseInit(PWM_TPRESCALER, ( TIM2_PWM_PD - 1 ) ); // PS==1, 499   ->  8khz (period == .000125)

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
    TIM2->IER |= TIM2_IER_UIE; // Enable Update Interrupt (sets manually-counted
    // pwm at 20mS with DC related to commutation/test-pot))
#endif
}

/*
 * drive /SD outputs and PWM channels
 */
void BLDC_Step(void)
{
    bldc_step += 1;
    bldc_step %= N_CSTEPS;

    if (global_uDC > 0)
    {
        bldc_move();
    }
    else // motor drive output has been disabled
    {
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(0, 0, 0);
    }
}

void bldc_move(void)
{
// /SD outputs on C5, C7, and G1
// wait until switch time arrives (watching for voltage on the floating line to cross 0)
    switch(bldc_step)
    {
    default:

    case 0:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(1, 0, 0);
        break;
    case 1:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(1, 0, 0);
        break;
    case 2:
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(0, 1, 0);
        break;
    case 3:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(0, 1, 0);
        break;
    case 4:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(0, 0, 1);
        break;
    case 5:
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(0, 0, 1);
        break;
    }
}
