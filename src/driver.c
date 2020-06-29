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

#define LOWER_ARM_CHOPPING
#define PWM_DC_PLUS     1
#define PWM_DC_MINUS   -1
#define PWM_GP_HI    0x7F // 127 positive max signed 8 integer
#define PWM_GP_LO    0 // 0x80

#define PWM_100PCNT    TIM2_PWM_PD
#define PWM_50PCNT     ( PWM_100PCNT / 2 )
#define PWM_25PCNT     ( PWM_100PCNT / 4 )
#define PWM_0PCNT      0
#define PWM_DC_RAMPUP  PWM_50PCNT // 52 // exp. determined


#ifndef PWM_IS_MANUAL
#define PWM_NOT_MANUAL_DEF  PWM_25PCNT //30 //0x20 // experimentally determined value (using manual adjustment)
#endif

// see define of TIM2 PWM PD ... it set for 8kHz (125uS)
#ifdef CLOCK_16
//  (1/16Mhz) * 2 * 250 -> 0.000125 S
 #define TIM1_PRESCALER  8
 #define TIM2_PRESCALER  TIM2_PRESCALER_8
#else
//  (1/8Mhz) * 4 * 250 ->  0.000125 S
 #define TIM1_PRESCALER  4
 #define TIM2_PRESCALER  TIM2_PRESCALER_4
#endif



/*
 * These constants are the number of timer counts (TIM3) to achieve a given
 *  commutation step period.
 * See TIM3 setup - base period is 0.000008 Seconds (8 uSecs)
 * For the theoretical 15000 RPM motor: 
 *   15000 / 60 = 250 rps
 *   cycles per sec = 250 * 6 = 1500
 *   1 cycle = 1/1500 = .000667 S
 *
 * RPS = (1 / cycle_time * 6)
 */

// the OL comm time is shortened by 1 rammp-unit (e.g. 2 counts @ 0.000008S per count where 8uS is the TIM3 bit-time)
// the ramp shortens the OL comm-time by 1 RU each ramp-step with each ramp step is the TIM1 period of ~1mS
#define BLDC_ONE_RAMP_UNIT          1

// 1 cycle = 6 * 8uS * 512 = 0.024576 S
#define BLDC_OL_TM_LO_SPD         512  // start of ramp

// 1 cycle = 6 * 8uS * 80 = 0.00384 S
#define BLDC_OL_TM_HI_SPD          80  // end of ramp

// 1 cycle = 6 * 8uS * 50 = 0.0024 S
#define BLDC_OL_TM_MANUAL_HI_LIM   64 // 58   // stalls at 56 ... stop at 64? ( 0x40? ) (I want to push the button and see the data at this speed w/o actually chaniging the CT)

// any "speed" setting higher than HI_LIM would be by closed-loop control of 
// commutation period (manual speed-control input by adjusting PWM  duty-cycle) 
// The control loop will only have precision of 0.000008 S adjustment though (externally triggered comparator would be ideal )

// 1 cycle = 6 * 8uS * 13 = 0.000624 S
// 1 cycle = 6 * 8uS * 14 = 0.000672 S
#define LUDICROUS_SPEED (13) // 15kRPM would be ~13.8 counts


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

/* Private function prototypes -----------------------------------------------*/
void PWM_Config_T1(void);
void PWM_Config(void);
void bldc_move( uint8_t );

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
 * Provides an "inverted" (complimentary) duty-cycle if [state0 < 0]
 */
uint16_t _set_output(int8_t s0)
{
    uint16_t pulse = PWM_0PCNT;

    int8_t state0 = s0;

#ifdef LOWER_ARM_CHOPPING // lower chopping hackro ... 
// Invert the logic of the transistor drive, so as to essentially PWM the 
// lower arm. Drive the upper-arm transistor ON (setting PWM to 100% DC) and 
// setting the complement of the duty-cycle (1-DC) on the lower transistor
    if (PWM_DC_PLUS == s0)
    {
        state0 = PWM_DC_MINUS;
    }
    else if (PWM_GP_LO == s0)
    {
        state0 = PWM_GP_HI;
    }
#endif

    if ( PWM_GP_HI == state0)
    {
        pulse = PWM_100PCNT;
    }
    else if ( PWM_GP_LO == state0)
    {
        pulse = PWM_0PCNT;
    }
    else if (state0 > 0)
    {
        pulse = global_uDC;
    }
    else if (state0 < 0)
    {
        pulse = TIM2_PWM_PD - global_uDC; // inverse pulse
    }
    //  else ... == 0 

    return pulse;
}


void PWM_set_outputs(int8_t state0, int8_t state1, int8_t state2)
{
    TIM2_pulse_0 = _set_output(state0);
    TIM2_pulse_1 = _set_output(state1);
    TIM2_pulse_2 = _set_output(state2);

    PWM_Config();
    PWM_Config_T1();
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   reference:
  *    http://embedded-lab.com/blog/starting-stm8-microcontrollers/21/
  *
  * - pulse width modulation frequency determined by the value of the TIM1_ARR register 
  * - duty cycle determined by the value of the TIM1_CCRi register
  */
void PWM_Config_T1(void)
{
    const uint16_t Pulse_MAX = (TIM2_PWM_PD - 1);
    const uint16_t T1_Period = 250 /* TIMx_PWM_PD */ ;  // 16-bit counter

// todo: re-config only if delta d.c. > threshold e.g. 1
    u8 OC_1_pulse = TIM2_pulse_0;
    u8 OC_2_pulse = TIM2_pulse_1;
    u8 OC_3_pulse = TIM2_pulse_2;

/* todo: look into this?:
"For correct operation, preload registers must be enabled when the timer is in PWM mode. This
is not mandatory in one-pulse mode (OPM bit set in TIM1_CR1 register)."
*/
//    TIM1_CtrlPWMOutputs(DISABLE);
// tbd: issue #6?  make sure the floating phase gets asserts off - 1) make sure ir2104 on (/SD=1) and HIN==0


    if (OC_1_pulse > 0 && OC_1_pulse < Pulse_MAX)
    {
        /* Channel 1 PWM configuration */
        TIM1_CCxCmd(TIM1_CHANNEL_2, ENABLE);
        TIM1_SetCompare2(OC_1_pulse);
    }
    else
    {
        TIM1_CCxCmd(TIM1_CHANNEL_2, DISABLE);

        if (OC_1_pulse >= Pulse_MAX)
        {
            GPIOC->ODR |=  (1<<2);  // PC2 set HI
            GPIOC->DDR |=  (1<<2);
            GPIOC->CR1 |=  (1<<2);
        }
        else
        {
            GPIOC->ODR &=  ~(1<<2);  // PC2 set LO
            GPIOC->DDR |=  (1<<2);
            GPIOC->CR1 |=  (1<<2);
        }
    }

    if (OC_2_pulse > 0 && OC_2_pulse < Pulse_MAX)
    {
        TIM1_CCxCmd(TIM1_CHANNEL_3, ENABLE);
        TIM1_SetCompare3(OC_2_pulse);
    }
    else
    {
        TIM1_CCxCmd(TIM1_CHANNEL_3, DISABLE);

        if (OC_2_pulse >= Pulse_MAX)
        {
            GPIOC->ODR |=  (1<<3);  // PC3 set HI
            GPIOC->DDR |=  (1<<3);
            GPIOC->CR1 |=  (1<<3);
        }
        else
        {
            GPIOC->ODR &=  ~(1<<3);  // PC3 set LO
            GPIOC->DDR |=  (1<<3);
            GPIOC->CR1 |=  (1<<3);
        }
    }

    if (OC_3_pulse > 0 && OC_3_pulse < Pulse_MAX)
    {
        /* Channel 4 PWM configuration */
        TIM1_CCxCmd(TIM1_CHANNEL_4, ENABLE);
        TIM1_SetCompare4(OC_3_pulse);
    }
    else
    {
        TIM1_CCxCmd(TIM1_CHANNEL_4, DISABLE);

        if (OC_3_pulse >= Pulse_MAX)
        {
            GPIOC->ODR |=  (1<<4);  // PC4 set HI
            GPIOC->DDR |=  (1<<4);
            GPIOC->CR1 |=  (1<<4);
        }
        else
        {
            GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
            GPIOC->DDR |=  (1<<4);
            GPIOC->CR1 |=  (1<<4);
        }
    }


//    TIM1_CtrlPWMOutputs(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
//GN: probly     TIM1_ARRPreloadConfig(ENABLE);

    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    TIM1_Cmd(ENABLE);
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

// todo: re-config only if delta d.c. > threshold e.g. 1
    u8 OC_1_pulse = TIM2_pulse_0;
    u8 OC_2_pulse = TIM2_pulse_1;
    u8 OC_3_pulse = TIM2_pulse_2;

    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    TIM2_TimeBaseInit(TIM2_PRESCALER, ( TIM2_PWM_PD - 1 ) );

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
        // BLDC_OL_comm_tm ... init in OFF state to _OL_TM_LO_SPD, don't touch!
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
        // BLDC_OL_comm_tm ... init in OFF state to _OL_TM_LO_SPD, don't touch!
    }

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm > BLDC_OL_TM_MANUAL_HI_LIM )
    {
        BLDC_OL_comm_tm -= 1; // faster
    }
}


void TIM3_setup(uint16_t u16period); // tmp

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
            BLDC_OL_comm_tm -= BLDC_ONE_RAMP_UNIT; 
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
//  update the timer for the OL commutation switch time
    TIM3_setup(BLDC_OL_comm_tm);
#endif
}

/*
 * drive /SD outputs and PWM channels
 */
void BLDC_Step(void)
{
    const uint8_t N_CSTEPS = 6;

    static uint8_t bldc_step = 0;

    bldc_step += 1;
    bldc_step %= N_CSTEPS;

    if (global_uDC > 0)
    {
        bldc_move( bldc_step );
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
 * BUG? if PWM pulse is on at the step time to transition to floating, the PWM 
 * pulse is not turned off with good timing as the voltage just bleeds off then
 */
void bldc_move( uint8_t step )
{
/*
    each comm. step, need to shutdown all PWM for the "hold off" period (flyback settling)
*/
//        PWM_set_outputs(0, 0, 0);

// Start a short timer on which ISR will then  trigger the A/D with proper 
//  timing  .... at 1/4 of the comm. cycle ?
// So this TIM3 would not stepp 6 times but 6x4 times? (4 times precision?)
// .......


// /SD outputs on C5, C7, and G1
    switch( step )
    {
    default:

    case 0:
        GPIOC->ODR |=   (1<<5);      // A+-+
        GPIOC->ODR |=   (1<<7);      // B---
        GPIOG->ODR &=  ~(1<<1);      // C.
        PWM_set_outputs(PWM_DC_PLUS, PWM_GP_LO, 0);
        break;

    case 1:
        GPIOC->ODR |=   (1<<5);	     // A+-+
        GPIOC->ODR &=  ~(1<<7);      // B.
        GPIOG->ODR |=   (1<<1);      // C---
        PWM_set_outputs(PWM_DC_PLUS, 0, PWM_GP_LO);
        break;

    case 2:
        GPIOC->ODR &=  ~(1<<5);      // A.
        GPIOC->ODR |=   (1<<7);      // B+-+
        GPIOG->ODR |=   (1<<1);      // C---
        PWM_set_outputs(0, PWM_DC_PLUS, PWM_GP_LO);
        break;

    case 3:
        GPIOC->ODR |=   (1<<5);      // A---
        GPIOC->ODR |=   (1<<7);      // B+-+
        GPIOG->ODR &=  ~(1<<1);      // C.
        PWM_set_outputs(PWM_GP_LO, PWM_DC_PLUS, 0);
        break;

    case 4:
        GPIOC->ODR |=   (1<<5);      // A---
        GPIOC->ODR &=  ~(1<<7);      // B.
        GPIOG->ODR |=   (1<<1);      // C+-+
        PWM_set_outputs(PWM_GP_LO, 0, PWM_DC_PLUS);
        break;

    case 5:
        GPIOC->ODR &=  ~(1<<5);      // A.
        GPIOC->ODR |=   (1<<7);      // B---
        GPIOG->ODR |=   (1<<1);      // C+-+
        PWM_set_outputs(0, PWM_GP_LO, PWM_DC_PLUS);
        break;
    }
}
