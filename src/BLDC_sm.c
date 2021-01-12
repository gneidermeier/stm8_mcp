/**
  ******************************************************************************
  * @file BLDC.c
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "bldc_sm.h"
#include "mdata.h"
#include "pwm_stm8s.h" // motor phase control
#include "faultm.h"

/* Private defines -----------------------------------------------------------*/

#define PWM_0PCNT      0

#define PWM_10PCNT     ( PWM_100PCNT / 10 )
#define PWM_20PCNT     ( PWM_100PCNT / 5 )
#define PWM_50PCNT     ( PWM_100PCNT / 2 )

#define PWM_X_PCNT( _PCNT_ )   ( _PCNT_ * PWM_100PCNT / 100 )

/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
#define PWM_DC_RAMPUP    PWM_X_PCNT( 12.0 )  // 0x1E ... 30 * 0.4 = 12.0

#define PWM_DC_IDLE      PWM_X_PCNT( PWM_DC_RAMPUP )

/*
 * These constants are the number of timer counts (TIM3) to achieve a given
 *  commutation step period.
 * See TIM3 setup - base period is 0.000000250 seconds (0.25 usec) in order to
 * provide high precision for controlling the commutation time, and each commutation step
 * unit is 4x TIM3 periods for back-EMF sampling at 1/4 and 3/4 in the commutation period.
 *
 * For the theoretical 1100kv motor @ 13.8v -> ~15000 RPM:
 *   15000 / 60 = 250 rps
 *   "Electrical cycles" per sec = 250 * (12/2) = 1500 ... where (12/2) is nr. of pole-pairs.
 *   Time of 1 cycle = 1/1500 = 0.000667 seconds  (360 degrees of 1 electrical cycle)
 *
 *   1 commutation sector is 60 degrees.
 *   Using TIM3 to get 4 updates per sector, and 360/15degrees=24 so ..
 *
 *   0.000667 seconds / 24 = 0.00002778 sec  (the "1/4 sector time" is 27.78us )
 *   ... divided by TIM3 base period (0.25 us)  -> 111 counts
 */

//#define TIM3_RATE_MODULUS   4 // each commutation sector of 60-degrees spans 4x TIM3 periods
// the commutation timing constants (TIM3 period) effectively have a factor of
// 'TIM3_RATE_MODULUS' rolled into them since the timer fires 4x faster than the
// actual motor commutation frequency.
#define BLDC_OL_TM_LO_SPD       0x1000 // 4096d  // start of ramp

//#define BLDC_OL_TM_HI_SPD       0x03C0 //  960d

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED         0x006F // 111


/*
 * Slope of what is basically a linear startup ramp, commutation time (i.e. TIM3)
 * period) decremented by fixed amount each control-loop timestep. Slope
 * determined by experiment (conservative to avoid stalling the motor!)
 */
#define BLDC_ONE_RAMP_UNIT      2


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static BLDC_STATE_T BLDC_State;

static uint16_t BLDC_OL_comm_tm;

static uint16_t Commanded_Dutycycle; // PWM duty-cycle has to be ramped to this

static uint8_t Manual_Ovrd;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief     .
 *   Simple ramping of commutation time period. At each iteration the
 *   commutation time period is ramped to the target value stepped in increment
 *   of +/- step depending of the sign of the error.
 *
 * @param   tgt_commutation_per  Target value to track.
 * @param   step integer step    Increment of the ramp (the slope).
 *
 * @return  +1 if positive increment
 *          -1 if negative increment
 *          0 if control variable equal to target
 */
int timing_ramp_control(uint16_t tgt_commutation_per, int increment)
{
    int error;
    int ret = 0;
    uint16_t u16 = get_commutation_period();

    error = tgt_commutation_per - u16;

    // determine signage of error i.e. step increment
    if (error < 0)
    {
        // negate the step parameter
        u16 -= increment;
        ret = -1;
    }
    else if (error > 0)
    {
        u16 += increment;
        ret = 1;
    }

    set_commutation_period( u16 );

    return ret;
}

/* Public functions ---------------------------------------------------------*/

/*
 *    System reset / re-arm function (has to be called both at program startup
 *    as well as following a fault condition state.
 */
void BLDC_Stop(void)
{
// kill the driver signals
    All_phase_stop();

    set_bldc_state( BLDC_OFF );

    Commanded_Dutycycle = 0;

    Faultm_init();

// was going to set commutation period to zero (0) here, but then the motor wouldn't fire up
// (even tho the function seeemed by look of the terminal to be running .. )
// the commutation period (TIM3) apparantly has to be set to something (not 0) 
// or else something goes wrong ... this also was useful to observe effect on system load at hightes motor speed! and visually to see motor state is _OFF

    set_commutation_period( LUDICROUS_SPEED );
}

/*
 * increment set and return present motor speed value
 */
void BLDC_PWMDC_Plus()
{
// let bldc timing logic regain control of commutattion time
    Manual_Ovrd = 0;

    if (Commanded_Dutycycle < 0xFFFF) // prevent integer rollover
    {
        Commanded_Dutycycle += 1;
    }
}

/*
 * decrement set and return present motor speed value
 */
void BLDC_PWMDC_Minus()
{
// let bldc timing logic regain control of commutattion time
    Manual_Ovrd = 0;

    if (Commanded_Dutycycle > 0)
    {
        Commanded_Dutycycle -= 1;
    }
}

/*
 * sets motor speed from commanded throttle/UI setting  (experimental)
 */
void BLDC_PWMDC_Set(uint16_t dc)
{
    Commanded_Dutycycle = dc;
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_dec()
{
    Manual_Ovrd = 1;

    BLDC_OL_comm_tm += 1; // slower
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_inc()
{
    Manual_Ovrd = 1;

    BLDC_OL_comm_tm -= 1; // faster
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
void set_commutation_period(uint16_t u16pd)
{
    BLDC_OL_comm_tm = u16pd;
}

uint16_t get_commutation_period(void)
{
    return BLDC_OL_comm_tm;
}

// BLDC_get_state()
BLDC_STATE_T get_bldc_state(void)
{
    return BLDC_State;
}
// BLDC_set_state( state )
BLDC_STATE_T set_bldc_state( BLDC_STATE_T newstate)
{
    BLDC_State = newstate;
}

/*
 * BLDC Update:
 *  Called from ISR
 *  Handle the BLDC state:
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
    int itemp;

    BLDC_STATE_T bldc_state = get_bldc_state();

    switch ( bldc_state )
    {
    default:
    case BLDC_OFF:

        if (Commanded_Dutycycle > 0) // allows startup state-transition when '+' key pressed
        {
            // assert the ramp-up state DC
            Commanded_Dutycycle = PWM_DC_RAMPUP;

            set_bldc_state( BLDC_RAMPUP );

            set_commutation_period( BLDC_OL_TM_LO_SPD );
        }

        break;

    case BLDC_ON:
#if 1 // 
        // finally, check if fault is set
        if ( 1 == Faultm_update() )
        {
#if 1 // #if ENABLE_VLOW_FAULT
// 0 DC safely stops the motor, user must still press STOP to re-arm the program.
// kill the driver signals but does not change the state from OFF .. (needs to be error state)
            set_bldc_state( BLDC_FAULT );
#endif
        }
#endif

        if ( 0 == Manual_Ovrd )
        {
            const int step = 1;
            timing_ramp_control( Get_OL_Timing( get_dutycycle() ), step );
        }
        break;

    case BLDC_RAMPUP:

        itemp = timing_ramp_control(
                    Get_OL_Timing( get_dutycycle() ), BLDC_ONE_RAMP_UNIT );

        if ( itemp >= 0 ) // ( comm_time < target )
        {
            // state-transition trigger
            set_bldc_state( BLDC_ON );
        }
        break;

    case BLDC_FAULT:
        Commanded_Dutycycle = PWM_0PCNT;

        break;
    }

    set_dutycycle( Commanded_Dutycycle );
}
