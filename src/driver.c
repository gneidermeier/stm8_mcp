/**
  ******************************************************************************
  * @file driver.c
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

//#define PWM_IS_MANUAL


#define PWM_50PCNT  ( TIM2_PWM_PD / 2 )
#define PWM_25PCNT  ( TIM2_PWM_PD / 4 )
#define PWM_DC_RAMPUP  PWM_50PCNT // 52 // exp. determined


//#ifdef SYMETRIC_PWM
//  #define PWM_NOT_MANUAL_DEF  (PWM_50PCNT +  15  )

#ifndef PWM_IS_MANUAL
#define PWM_NOT_MANUAL_DEF  PWM_25PCNT //30 //0x20 // experimentally determined value (using manual adjustment)
#endif

// see define of TIM2 PWM PD ... it set for 8kHz (125uS)
#ifdef CLOCK_16
 #define PWM_PRESCALER  TIM2_PRESCALER_8 // @16Mhz
#else
 #define PWM_PRESCALER  TIM2_PRESCALER_4 // @8Mhz
#endif

// nbr of steps required to commutate 3 phase
#define N_CSTEPS   6


/*
 * commutation step time is on TIM3
 */
 #define BLDC_OL_TM_LO_SPD   (256) // start of ramp

 #define BLDC_OL_TM_HI_SPD    (40)  // end of ramp

 #define BLDC_OL_TM_MANUAL_HI_LIM   (26)   // stalls at 25 




/* Public variables  ---------------------------------------------------------*/
uint16_t BLDC_OL_comm_tm;   // could be private

uint16_t Manual_uDC;

BLDC_STATE_T BLDC_State;


/* Private variables ---------------------------------------------------------*/
static uint16_t TIM2_pulse_0 ;
static uint16_t TIM2_pulse_1 ;
static uint16_t TIM2_pulse_2 ;

static uint16_t Ramp_Step_Tm; // reduced x2 each time but can't start any slower
/* static */ uint16_t global_uDC;
static uint8_t bldc_step = 0;

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

    if ( BLDC_OFF == BLDC_State )
    {
//        global_uDC = 0;
    }
}

/*
 * intermediate function for setting PWM with positive or negative polarity
 */
uint16_t _set_output(int8_t state0)
{
    uint16_t pulse;
    if (state0 > 0)
    {
        pulse = global_uDC;
    }
    else if (state0 < 0)
    {
        pulse = TIM2_PWM_PD - global_uDC; // inverse pulse (for symetric PWM)
    }
    else
    {
        pulse = 0;
    }

    return pulse;
}

void PWM_set_outputs(int8_t state0, int8_t state1, int8_t state2)
{
    TIM2_pulse_0 = _set_output(state0);
    TIM2_pulse_1 = _set_output(state1);
    TIM2_pulse_2 = _set_output(state2);

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
    const uint16_t TIM2_Pulse_MAX = (TIM2_PWM_PD - 1); // PWM_MAX_LIMIT 

// dumb .. these should all be equal. 
// todo: re-config only if delta d.c. > threshold e.g. 1 
    u8 OC_1_pulse = TIM2_pulse_0;
    u8 OC_2_pulse = TIM2_pulse_1;
    u8 OC_3_pulse = TIM2_pulse_2;

    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    TIM2_TimeBaseInit(PWM_PRESCALER, ( TIM2_PWM_PD - 1 ) );

    if (OC_1_pulse > 0 && OC_1_pulse < TIM2_Pulse_MAX)
    {
        /* Channel 1 PWM configuration */
        TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, OC_1_pulse, TIM2_OCPOLARITY_LOW );
        TIM2_OC1PreloadConfig(ENABLE);
    }
    else if (OC_1_pulse >= TIM2_Pulse_MAX)
    {
        GPIOD->ODR |=  (1<<4);  // PD4 set HI
        GPIOD->DDR |=  (1<<4);
        GPIOD->CR1 |=  (1<<4);
    }
    else
    {
        GPIOD->ODR &=  ~(1<<4);  // PD4 set LO
        GPIOD->DDR |=  (1<<4);
        GPIOD->CR1 |=  (1<<4);
    }

    if (OC_2_pulse > 0 && OC_2_pulse < TIM2_Pulse_MAX)
    {
        /* Channel 2 PWM configuration */
        TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, OC_2_pulse, TIM2_OCPOLARITY_LOW );
        TIM2_OC2PreloadConfig(ENABLE);
    }
    else if (OC_2_pulse >= TIM2_Pulse_MAX)
    {
        GPIOD->ODR |=  (1<<3);  // PD3 set HI
        GPIOD->DDR |=  (1<<3);
        GPIOD->CR1 |=  (1<<3);
    }
    else
    {
        GPIOD->ODR &=  ~(1<<3);  // PD3 set LO
        GPIOD->DDR |=  (1<<3);
        GPIOD->CR1 |=  (1<<3);
    }

    if (OC_3_pulse > 0 && OC_3_pulse < TIM2_Pulse_MAX)
    {
        /* Channel 3 PWM configuration */
        TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, OC_3_pulse, TIM2_OCPOLARITY_LOW );
        TIM2_OC3PreloadConfig(ENABLE);
    }
    else if ( OC_3_pulse >= TIM2_Pulse_MAX)
    {
        GPIOA->ODR |=  (1<<3);  // PA3 set HI
        GPIOA->DDR |=  (1<<3);
        GPIOA->CR1 |=  (1<<3);
    }
    else
    {
        GPIOA->ODR &=  ~(1<<3);  // PA3 set LO
        GPIOA->DDR |=  (1<<3);
        GPIOA->CR1 |=  (1<<3);
    }

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);

    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);


    TIM2->IER |= TIM2_IER_UIE; // Enable Update Interrupt for use as hi-res timing reference
}


/*
 *
 */
void BLDC_Stop()
{
    BLDC_State = BLDC_OFF;
    PWM_Set_DC( 0 );
}

/*
 *
 */
void BLDC_Spd_dec()
{
    if (BLDC_OFF == BLDC_State)
    {
        BLDC_State = BLDC_RAMPUP;
    }

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm < 0xFFFF)
    {
        BLDC_OL_comm_tm += 1; // slower
    }
}

/*
 *
 */
void BLDC_Spd_inc()
{
    if (BLDC_OFF == BLDC_State)
    {
        BLDC_State = BLDC_RAMPUP;
    }

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm > BLDC_OL_TM_MANUAL_HI_LIM )
    {
        BLDC_OL_comm_tm -= 1; // faster
    }
}


void timer_config_channel_time(uint16_t u16period); // tmp

/*
 * BLDC Update: handle the BLDC state 
 *      Off: nothing
 *      Rampup: get BLDC up to sync speed to est. comm. sync.
 *              Once the HI OL speed (frequency) is reached, then the idle speed 
 *              must be established, i.e. controlling PWM DC to ? to achieve 2500RPM
 *              To do this closed loop, will need to internally time between the
 *              A/D or comparator input interrupts and adjust DC using e.g. Proportional
 *              control. When idle speed is reached, can transition to user control i.e. ON State
 *      On:  definition of ON state - user control (button inputs) has been enabled 
 *              1) ideally, does nothing - BLDC_Step triggered by A/D comparator event
 *              2) less ideal, has to check A/D or comp. result and do the comm. 
 *                 step ... but the resolution will be these discrete steps 
 *                 (of TIM1 reference)
 */
void BLDC_Update(void)
{
    switch (BLDC_State)
    {
    default:
    case BLDC_OFF:
        // reset commutation timer and ramp-up counters ready for ramp-up
        BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;
        break;

    case BLDC_ON:
        // do ON stuff
#ifdef PWM_IS_MANUAL
// doesn't need to set global uDC every time as it would be set once in the FSM
// transition ramp->on ... but it doesn't hurt to assert it
         PWM_Set_DC( Manual_uDC ) ;  // #ifdef SYMETRIC_PWM ...  (PWM_50PCNT +  Manual_uDC / 2)
#else
//         PWM_Set_DC( PWM_NOT_MANUAL_DEF ) ;
#endif
        break;
    case BLDC_RAMPUP:

         PWM_Set_DC( PWM_DC_RAMPUP ) ;

        if (BLDC_OL_comm_tm > BLDC_OL_TM_HI_SPD) // state-transition trigger?
        {
            BLDC_OL_comm_tm -= 1;  // TIM3 has been timed so that the ramp is realized by simple -=1 each timer period
                                   // so the ramp rate is  16uS/TIM1_period w/ TIM1 @ 1.024 mS 
        }
        else
        {
            // TODO: the actual transition to ON state would be seeing the ramp-to speed
// achieved in closed-loop operation
            BLDC_State = BLDC_ON;
            PWM_Set_DC( PWM_NOT_MANUAL_DEF );
        }
        break;
    }

#if 1 //    ! MANUAL TEST
//  set the OL commutation switch time
    timer_config_channel_time(BLDC_OL_comm_tm);
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

/*
 * TODO: schedule at 30degree intervals? (see TIM3)
 *
 */
void bldc_move(void)
{
 const int8_t foo = 99; 

// /SD outputs on C5, C7, and G1
    switch(bldc_step)
    {
    default:

    case 0:
        GPIOC->ODR |=   (1<<5);      // A+-+
        GPIOC->ODR |=   (1<<7);      // B---
        GPIOG->ODR &=  ~(1<<1);      // C.
        PWM_set_outputs(foo, 0, 0);	 
        break;                                    // C falling

    case 1:
        GPIOC->ODR |=   (1<<5);	     // A+-+
        GPIOC->ODR &=  ~(1<<7);      // B.
        GPIOG->ODR |=   (1<<1);      // C---
        PWM_set_outputs(foo, 0, 0);
        break;                                    //  B float 

    case 2:
        GPIOC->ODR &=  ~(1<<5);      // A.
        GPIOC->ODR |=   (1<<7);      // B+-+
        GPIOG->ODR |=   (1<<1);      // C---
        PWM_set_outputs(0, foo, 0);
        break;                                    // A Falling

    case 3:
        GPIOC->ODR |=   (1<<5);      // A---
        GPIOC->ODR |=   (1<<7);      // B+-+
        GPIOG->ODR &=  ~(1<<1);      // C.
        PWM_set_outputs(0, foo, 0);
        break;                                    // C float

    case 4:
        GPIOC->ODR |=   (1<<5);      // A---
        GPIOC->ODR &=  ~(1<<7);      // B.
        GPIOG->ODR |=   (1<<1);      // C+-+
        PWM_set_outputs(0, 0, foo);
        break;                                    // B falling

    case 5:
        GPIOC->ODR &=  ~(1<<5);      // A.
        GPIOC->ODR |=   (1<<7);      // B---
        GPIOG->ODR |=   (1<<1);      // C+-+
        PWM_set_outputs(0, 0, foo);
        break;                                    // A float
    }
}
