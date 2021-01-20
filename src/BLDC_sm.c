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

static uint16_t Commanded_Dutycycle;
static uint16_t UI_speed;

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
 * If the integer step is 0, the target value is set directly with no ramping.
 */
void timing_ramp_control(uint16_t tgt_commutation_per, uint8_t stepi)
{
    int error;
    uint16_t u16 = BLDC_OL_comm_tm;

    if (stepi > 0)
    {
        stepi &= ~0x08; // arbitrary limit on step, doesn't need to be that big
        error = tgt_commutation_per - u16;

        // determine signage of error i.e. step increment
        if (error < 0)
        {
            u16 -= stepi;

            if (u16 > S16_MAX)
            {
                // out of range and likely underflow
                u16 = 0;
            }
        }
        else if (error > 0)
        {
            u16 += stepi;

            if (u16 > S16_MAX)
            {
                // clip to s16 max
                u16 = S16_MAX;
            }
        }
    }
    else
    {
        u16 = tgt_commutation_per;
    }

    BLDC_OL_comm_tm  = u16;
}

/*
 * common sub for stopping and fault states
 */
static void haltensie(void)
{
// have to clear the local UI_speed since that is the transition OFF->RAMP condition
    UI_speed = 0;

    // kill the driver signals
    All_phase_stop();
    Commanded_Dutycycle = PWM_0PCNT;
}

/*
 * only the state-machine is allowed to modify the state variable
 */
static BLDC_STATE_T set_bldc_state( BLDC_STATE_T newstate)
{
    BLDC_State = newstate;
}

/* Public functions ---------------------------------------------------------*/

/*
 *    System reset / re-arm function (has to be called both at program startup
 *    as well as following a fault condition state.
 */
void BLDC_Stop(void)
{
    // had to move these into here again
    haltensie();
    Faultm_init();

    set_bldc_state( BLDC_RESET );
}


/*
 * sets motor speed from commanded throttle/UI setting  (experimental)
 */
void BLDC_PWMDC_Set(uint8_t dc)
{
    //  dc scale [0:255] is close enough to PWM range [0:250]
    // but note ui_speed is u16 ... would argue that cast is not really needed
    // as the implicit  promotion of the rvalue should be well understood and
    // evident  according to  the rules of C.

    if (BLDC_FAULT != get_bldc_state() )
    {
        UI_speed = (uint16_t) dc;
    }
}

/*
 * getter for the Commanded Duty Cycle
 */
uint16_t BLDC_PWMDC_Get(void)
{
    return Commanded_Dutycycle;
}


/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_dec()
{
    Manual_Ovrd = TRUE;

    BLDC_OL_comm_tm += 1; // slower
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_inc()
{
    Manual_Ovrd = TRUE;

    BLDC_OL_comm_tm -= 1; // faster
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
uint16_t get_commutation_period(void)
{
    return BLDC_OL_comm_tm;
}

// BLDC_get_state()
BLDC_STATE_T get_bldc_state(void)
{
    return BLDC_State;
}

/*
 * BLDC Update:
 *  Called from ISR
 *  Handle the BLDC state:
 */
void BLDC_Update(void)
{
    const uint16_t _RampupDC_ = PWM_DC_RAMPUP; // TODO:  the macro in the if() expression causes strange linker error
// commutation timing value defaults to table lookup indexed by duty-cycle
    uint16_t timing_value = Get_OL_Timing( Commanded_Dutycycle );
    uint8_t ramp_step = 1; // default ramp step in on-state

    if ( 0 != Faultm_get_status() )
    {
// do not pass go
        set_bldc_state( BLDC_FAULT );
    }

    switch ( get_bldc_state() )
    {
    default:
    case BLDC_OFF:
        // allow motor to start when throttle has been raised
        if (UI_speed > _RampupDC_ )
        {
            Commanded_Dutycycle = PWM_DC_RAMPUP;
            timing_value = BLDC_OL_TM_LO_SPD;
            ramp_step = 0;
            set_bldc_state( BLDC_RAMPUP );
        }
        break;

    case BLDC_ON:
				// timing_value == Get_OL_Timing() from above and ramp step is default
        // if UI_speed is changing, then release manual commutation button mode
        if (Commanded_Dutycycle != UI_speed)
        {
            Manual_Ovrd = FALSE;
        }

        if ( FALSE == Manual_Ovrd )
        {
            Commanded_Dutycycle = UI_speed;
        }
        break;

    case BLDC_RAMPUP:
        // timing_value == Get_OL_Timing() from above but ramp step is different
        ramp_step = BLDC_ONE_RAMP_UNIT;

        if (timing_value > BLDC_OL_comm_tm)
        {
            set_bldc_state( BLDC_ON ); // state-transition trigger
        }
        break;

    case BLDC_RESET:
        /* inneffective ... seems to be a race resetting fault on reset
                haltensie();
                Faultm_init();
        */
// was going to set commutation period to zero (0) here, but then the motor wouldn't fire up
// (even tho the function seeemed by look of the terminal to be running .. )
// the commutation period (TIM3) apparantly has to be set to something (not 0)
// or else something goes wrong ... this also was useful to observe effect on system load at hightes motor speed! and visually to see motor state is _OFF
        timing_value = LUDICROUS_SPEED;
        ramp_step = 0;
        set_bldc_state( BLDC_OFF );
        break;

    case BLDC_FAULT:

        haltensie();
        break;
    }

    timing_ramp_control( timing_value, ramp_step );

    set_dutycycle( Commanded_Dutycycle );
}
