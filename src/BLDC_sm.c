/**
  ******************************************************************************
  * @file BLDC_sm.c
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
/**
 * \defgroup BLDC_sm BLDC State
 * @brief BLDC state management and timing control
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "bldc_sm.h" // external types used internally
#include "mdata.h"
#include "pwm_stm8s.h" // motor phase control
#include "faultm.h"
#include "sequence.h"

/* Private defines -----------------------------------------------------------*/

#define PWM_0PCNT      0

// cast arg to 16-bit and group the pcnt*100 term to retain precision
#define PWM_X_PCNT( _PCNT_ )   (uint16_t)( ( _PCNT_ * PWM_100PCNT ) / 100.0 )

/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
#define PWM_DC_RAMPUP    PWM_X_PCNT( 12.0 )  // 0x1E ... 30 * 0.4 = 12.0

#define PWM_DC_SHUTOFF   PWM_X_PCNT( 8.0 )   // stalls below 18 counts (7.4 %)

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
#define BLDC_OL_TM_LO_SPD         0x0B00 // start of ramp

//#define BLDC_OL_TM_HI_SPD       0x03C0 //  960d

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED         0x006F // 111


/*
 * Slope of what is basically a linear startup ramp, commutation time (i.e. TIM3)
 * period) decremented by fixed amount each control-loop timestep. Slope
 * determined by experiment (conservative to avoid stalling the motor!)
 */
#define BLDC_ONE_RAMP_UNIT       2     // working range [1:6] ?? (exper.)


/* Private types -----------------------------------------------------------*/
/**
 * @brief Typedef for state machine variable.
 */
/** @deprecated */
typedef enum
{
    BDC_RESET = 0,
    BDC_RUNNING
} BLDC_STATE_T;

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static BLDC_STATE_T BLDC_State;

static uint16_t BLDC_OL_comm_tm;     // persistent value of ramp timing

static uint16_t Commanded_Dutycycle; // copied select DC to global for logging

static uint8_t UI_speed;             // input from UI task, file-scope for sm_update

static uint8_t Manual_Ovrd;   // indicates manual commuation buttons are active


/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Commutation timing ramp control.
 *
 * At each iteration the commutation time period is ramped to the target value
 * stepped in increment of +/- step depending on the sign of the error.
 *
 * @param   tgt_commutation_per  Target value to track.
 */
static void timing_ramp_control(uint16_t tgt_commutation_per)
{
    const uint8_t stepi = BLDC_ONE_RAMP_UNIT;

    uint16_t u16 = BLDC_OL_comm_tm;

    // determine signage of error i.e. step increment
    if (u16 > tgt_commutation_per)
    {
        u16 -= stepi;
        if (u16 < tgt_commutation_per)
        {
            u16 = tgt_commutation_per;
        }
        BLDC_OL_comm_tm  = u16;
    }
    else if (u16 < tgt_commutation_per)
    {
        u16 += stepi;
        if (u16 > tgt_commutation_per)
        {
            u16 = tgt_commutation_per;
        }
        BLDC_OL_comm_tm  = u16;
    }
}

/*
 * BL_stop
 * common sub for stopping and fault states
 */
static void haltensie(void)
{
// have to clear the local UI_speed since that is the transition OFF->RAMP condition
    UI_speed = 0;

    // kill the driver signals
    All_phase_stop();
}

/*
 * only the state-machine is allowed to modify the state variable
 */
static void set_bldc_state( BLDC_STATE_T newstate)
{
    BLDC_State = newstate;
}

/* Public functions ---------------------------------------------------------*/

/**
 * @brief Initialize/reset motor
 *
 *    System reset / re-arm function (has to be called both at program startup
 *    as well as following a fault condition state.
 */
void BL_reset(void)
{
    // stop the system in case it is already running
    haltensie();  //  zeros the  UI speed 

    // reset the system

    // the commutation period (TIM3) apparantly has to be set to something (not 0)
    // If TIM3 IE, the period must be long enough to ensure not saturated by ISR!
    // Therefore, default the period to the highest expected speed to assert CPU
    // load is handled, this is also simpler than trying to keep track of what
    // state to enable the commutation control .
    BLDC_OL_comm_tm = LUDICROUS_SPEED;

    Faultm_init();

    set_bldc_state( BDC_RESET );
}


/**
 * @brief Sets motor speed from commanded throttle/UI setting
 *
 * @param dc Speed input which can be in the range [0:255] but 255 is to be
 *        reserved for special use (out of band value). If the PWM resolution
 *        changes then the input speed will have to be re-scaled accordingly.
 *
 * Now with low speed cut off !
 */
void BLDC_PWMDC_Set(uint8_t dc)
{
    // as long as requested speed is less than min, hold it in reset
    if (dc < PWM_DC_SHUTOFF)
    {
        // reset needed in case system was running, in which case there is no 
        // going back .. has to ramp again to get started. 
        BL_reset();

        UI_speed = 0; // note redundant if BL reset does it
    }
    else
    {
        UI_speed = dc;
    }
}

/**
 * @brief Accessor for Commanded Duty Cycle
 *
 * @return Commanded Duty Cycle
 */
uint16_t BLDC_PWMDC_Get(void)
{
    return Commanded_Dutycycle;
}


/** @cond */ // hide some developer/debug code

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
/** @endcond */


/**
  * @brief Accessor for commutation period.
  *
  * @return commutation period
  */
uint16_t get_commutation_period(void)
{
    return BLDC_OL_comm_tm;
}

/**
 * @brief Accessor for state variable.
 *
 * @return state value
 */
BL_RUNSTATE_t BL_get_state(void)
{
    if (BDC_RUNNING == BLDC_State)
    {
        return BL_IS_RUNNING;
    }
    // else
    return BL_NOT_RUNNING;
}

/**
 * @brief Periodic state machine update.
 *
 * Called from ISR. Evaluate state transition conditions and determine new
 * state.
 *
 * closed-loop control ... speed duty-cycle threshold, error switch sign? area under integratino curve
 */
void BLDC_Update(void)
{
    const uint16_t _RampupDC_ = PWM_DC_RAMPUP; // warning : truncating assignment`

    uint16_t inp_dutycycle = 0; // intialize to 0

    if ( 0 == Faultm_get_status() )
    {
        inp_dutycycle = UI_speed;
    }
    else
    {
        // do not pass go
        haltensie(); // sets UI speed 0
        // assert ... inp_dutycycle = 0;
    }

    if ( BDC_RUNNING != BLDC_State )
    {
        // allow motor to start when throttle has been raised
        if (UI_speed > _RampupDC_ )
        {
            set_bldc_state( BDC_RUNNING );

            // set initial conditions for ramp state
            inp_dutycycle = _RampupDC_ ;
            BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;
        }
    }

    // finally, refresh the duty-cycle and commutation period
    set_dutycycle( inp_dutycycle );

    // there isn't much point in enabling commuation timing contrl if speed is 0
    // however, it is simpler to just leave it running as long as ISR period no shorter than max speed see other assert warning)
    timing_ramp_control( Get_OL_Timing( inp_dutycycle ) );

    Commanded_Dutycycle = inp_dutycycle; // refresh the logger variable

#if 0
    if ( BLDC_RUNNING == state)
    {
        uint16_t timing_term = Get_OL_Timing( Commanded_Dutycycle );

// this needs to be in proportion/inverse to commutation period however you look at it but this mmight work for now ...
//        _error =  (  error * (int16_t)BLDC_OL_comm_tm / (int16_t)ramp_tm_pd  ) >> KPSCALE  ;
        error = error >> 1 ; // factor out some of the scale bits

// prop gain is really only meaningful if there are other controller terms
        Prop_timing_error =  error >> KPROP ; // factor out some of the scale bits

        if (Commanded_Dutycycle > FTL_THRSH ) // probably use a lower threshld to drop out of FTL
        {
            uint16_t timing_term = (int16_t)BLDC_OL_comm_tm + Prop_timing_error; // make sure signed add!
        }

        timing_ramp_control( timing_term ); // ramp to the new timing term
    }
#endif
}

/**@}*/ // defgroup

