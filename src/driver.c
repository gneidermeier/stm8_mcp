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
//#define LOWER_ARM_CHOPPING

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

#define TIM2_PRESCALER  TIM2_PRESCALER_8
#else
//  (1/8Mhz) * 4 * 250 ->  0.000125 S

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


#define THREE_PHASES 3

/* Private types -----------------------------------------------------------*/

// enumerates the PWM state of each channel
typedef enum DC_PWM_STATE
{
    DC_OUTP_OFF,
    DC_PWM_PLUS,
    DC_PWM_MINUS, // complimented i.e. (100% - DC)
    DC_OUTP_HI,
    DC_OUTP_LO,
    DC_OUTP_FLOAT
} DC_PWM_STATE_t;

typedef enum THREE_PHASE_CHANNELS
{
    PHASE_A,
    PHASE_B,
    PHASE_C
} THREE_PHASE_CHANNELS_t;

typedef enum PWM_MODE
{
    UPPER_ARM,
    LOWER_ARM
} PWM_MODE_t;


/* Public variables  ---------------------------------------------------------*/
uint16_t BLDC_OL_comm_tm;   // could be private

uint16_t Manual_uDC;

BLDC_STATE_T BLDC_State;

#ifdef LOWER_ARM_CHOPPING
  PWM_MODE_t PWM_Mode = LOWER_ARM;
#else // upper
  PWM_MODE_t PWM_Mode = UPPER_ARM;
#endif

/* Private variables ---------------------------------------------------------*/

static uint16_t Ramp_Step_Tm; // reduced x2 each time but can't start any slower
/* static */ uint16_t global_uDC;

/* Private function prototypes -----------------------------------------------*/
void PWM_Config_T1( uint16_t channels[] );
void PWM_Config( uint16_t channels[] );
void bldc_move_la( uint8_t ); // lower arm
void bldc_move_ua( uint8_t ); // upper arm

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
uint16_t _set_output(uint8_t chan, DC_PWM_STATE_t s0)
{
    uint16_t pulse = PWM_0PCNT;

    DC_PWM_STATE_t state0 = s0;


    switch(state0)
    {
    default:
    case DC_OUTP_FLOAT:
        pulse = PWM_0PCNT;
        break;
    case DC_OUTP_HI:
        pulse = PWM_100PCNT;
        break;
    case DC_OUTP_LO:
        pulse = PWM_0PCNT;
        break;
    case DC_PWM_PLUS:
        pulse = global_uDC;
        break;
    case DC_PWM_MINUS: // complimented i.e. (100% - DC)
        pulse = TIM2_PWM_PD - global_uDC; // inverse pulse
        break;
    }

    return pulse;
}

/*
 * when the motor runs, the output voltage during the sector will float between
 * the two driving phases.
 * PWM pin is first asserted to the top or bottom rail depending on
 * "XX_ARM_CHOPPING". The assertion is to avoid what will appear as a noise
 * spike on the transition -> floating.
*/
void PWM_set_phase_float(int8_t channel)
{
    if (PHASE_A == channel)
    {
        // assert PWM channel to 0
        TIM1_CCxCmd(TIM1_CHANNEL_2, DISABLE);

        if (LOWER_ARM == PWM_Mode )  // lower chopping hackro ...
        {
            GPIOC->ODR |=  (1<<2);  // PC2 set HI
            GPIOC->DDR |=  (1<<2);
            GPIOC->CR1 |=  (1<<2);
        }
        else // if (UPER_ARM
        {
            GPIOC->ODR &=  ~(1<<2);  // PC2 set LO
            GPIOC->DDR |=  (1<<2);
            GPIOC->CR1 |=  (1<<2);
        }
// set /SD A OFF
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);

    }
    else if (PHASE_B == channel)
    {
        TIM1_CCxCmd(TIM1_CHANNEL_3, DISABLE);

        if (LOWER_ARM == PWM_Mode )  // lower chopping hackro ...
        {
            GPIOC->ODR |=  (1<<3);  // PC3 set HI
            GPIOC->DDR |=  (1<<3);
            GPIOC->CR1 |=  (1<<3);
        }
        else // if (UPER_ARM
        {
            GPIOC->ODR &=  ~(1<<3);  // PC3 set LO
            GPIOC->DDR |=  (1<<3);
            GPIOC->CR1 |=  (1<<3);
        }
// set /SD B OFF
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR |=   (1<<1);

    }
    else if (PHASE_C == channel)
    {
        TIM1_CCxCmd(TIM1_CHANNEL_4, DISABLE);

        if (LOWER_ARM == PWM_Mode )
        {
            GPIOC->ODR |=  (1<<4);  // PC4 set HI
            GPIOC->DDR |=  (1<<4);
            GPIOC->CR1 |=  (1<<4);
        }
        else // if (UPER_ARM
        {
            GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
            GPIOC->DDR |=  (1<<4);
            GPIOC->CR1 |=  (1<<4);
        }
// set /SD C OFF
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR &=  ~(1<<1);
    }
}

void PWM_set_outputs(int8_t state0, int8_t state1, int8_t state2)
{
    uint16_t t_channels[ THREE_PHASES ];
    const uint16_t Pulse_MAX = (TIM2_PWM_PD - 1);

    // 1 phase floating (/SD -> OFF), and the other two /SD outputs are ON

    if ( DC_OUTP_FLOAT == state0)
    {
        PWM_set_phase_float(PHASE_A);
    }
    else if (DC_OUTP_FLOAT == state1)
    {
        PWM_set_phase_float(PHASE_B);
    }
    else if (DC_OUTP_FLOAT == state2)
    {
        PWM_set_phase_float(PHASE_C);
    }

    t_channels[0]= _set_output(0, state0);
    t_channels[1]= _set_output(1, state1);
    t_channels[2]= _set_output(2, state2);

#if 1
    PWM_Config_T1( t_channels );
#else
#endif




    PWM_Config( t_channels );
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
void PWM_Config_T1( uint16_t timer_channels[] )
{
    const uint16_t Pulse_MAX = (TIM2_PWM_PD - 1);
    const uint16_t T1_Period = 250 /* TIM2_PWM_PD */ ;  // 16-bit counter

    u8 OC_1_pulse = timer_channels[0];
    u8 OC_2_pulse = timer_channels[1];
    u8 OC_3_pulse = timer_channels[2];

    /* todo: look into this?:
    "For correct operation, preload registers must be enabled when the timer is in PWM mode. This
    is not mandatory in one-pulse mode (OPM bit set in TIM1_CR1 register)."

     TIM1_ARRPreloadConfig(ENABLE); ??
    */

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
//    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
//    TIM1_Cmd(ENABLE);
}

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   GN: from UM0834 PWM example
  */
void PWM_Config( uint16_t timer_channels[] )
{
    const uint16_t TIM2_Pulse_MAX = (TIM2_PWM_PD - 1); // PWM_MAX_LIMIT

// todo: re-config only if delta d.c. > threshold e.g. 1
    u8 OC_1_pulse = timer_channels[0];
    u8 OC_2_pulse = timer_channels[1];
    u8 OC_3_pulse = timer_channels[2];

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
 * TODO: schedule at 30degree intervals? (see TIM3)			????
 Start a short timer on which ISR will then  trigger the A/D with proper timing  .... at 1/4 of the comm. cycle ?
 So TIM3 would not stepp 6 times but 6x4 times? (4 times precision?)
 */
void BLDC_Step(void)
{
    const uint8_t N_CSTEPS = 6;

    static uint8_t bldc_step = 0;

    bldc_step += 1;
    bldc_step %= N_CSTEPS;

    if (global_uDC > 0)
    {
    /*
        each comm. step, need to shutdown all PWM for the "hold off" period (flyback settling) ???
           PWM_set_outputs(0, 0, 0);
    */
// TODO: set the PWM states (hi, lo plus minus) from look-up tables	???
        if (UPPER_ARM == PWM_Mode )
        {
            bldc_move_ua(bldc_step); // bldc_move_ra();
        }
        else
        {
            bldc_move_la(bldc_step);
        }
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
 * upper-arm chopping
 */
void bldc_move_ua(uint8_t step )
{
    switch( step )
    {
    default:
    case 0:
        PWM_set_outputs(DC_PWM_PLUS, DC_OUTP_LO, DC_OUTP_FLOAT);
        break;
    case 1:
        PWM_set_outputs(DC_PWM_PLUS, DC_OUTP_FLOAT, DC_OUTP_LO);
        break;
    case 2:
        PWM_set_outputs(DC_OUTP_FLOAT, DC_PWM_PLUS, DC_OUTP_LO);
        break;
    case 3:
        PWM_set_outputs(DC_OUTP_LO, DC_PWM_PLUS, DC_OUTP_FLOAT);
        break;
    case 4:
        PWM_set_outputs(DC_OUTP_LO, DC_OUTP_FLOAT, DC_PWM_PLUS);
        break;
    case 5:
        PWM_set_outputs(DC_OUTP_FLOAT, DC_OUTP_LO, DC_PWM_PLUS);
        break;
    }
}

/*
 * lower-arm chopping
 */
void bldc_move_la(uint8_t step )
{
    switch( step )
    {
    default:
    case 0:
        PWM_set_outputs(DC_PWM_MINUS, DC_OUTP_HI, DC_OUTP_FLOAT);
        break;
    case 1:
        PWM_set_outputs(DC_PWM_MINUS, DC_OUTP_FLOAT, DC_OUTP_HI);
        break;
    case 2:
        PWM_set_outputs(DC_OUTP_FLOAT, DC_PWM_MINUS, DC_OUTP_HI);
        break;
    case 3:
        PWM_set_outputs(DC_OUTP_HI, DC_PWM_MINUS, DC_OUTP_FLOAT);
        break;
    case 4:
        PWM_set_outputs(DC_OUTP_HI, DC_OUTP_FLOAT, DC_PWM_MINUS);
        break;
    case 5:
        PWM_set_outputs(DC_OUTP_FLOAT, DC_OUTP_HI, DC_PWM_MINUS);
        break;
    }
}
