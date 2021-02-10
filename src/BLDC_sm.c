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

//   0.000667 seconds / 24 / 0.25us = 111 counts
//#define LUDICROUS_SPEED         0x006F // 111


// ramp rate derived from control rate which is derived from overall system rate
#define BLDC_ONE_RAMP_UNIT    (1 * CTRL_RATEM)   // working range  ??


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

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

    // Set initial commutation timing period upon state transition.
    BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;

    Faultm_init();
}


/**
 * @brief Sets motor speed from commanded throttle/UI setting
 *
 * @details  Establishes the condition to transition from off->running. The motor
 *  is enabled to start once reaching the ramp speed threshold, and allowed to
 *  slow down to the low shutoff threshold.
 *  UI Speed is shared with background task so this function all should
 *  be invoked only from within a CS.
 *
 * @param dc Speed input which can be in the range [0:255] but 255 is to be
 *        reserved for special use (out of band value). If the PWM resolution
 *        changes then the input speed will have to be re-scaled accordingly.
 *
 * Now with low speed cut off !
 */
void BLDC_PWMDC_Set(uint8_t dc)
{
    if (dc > PWM_DC_SHUTOFF)
    {
        // Update the dc if speed input greater than ramp start, OR if system already running
        if ( dc > PWM_DC_RAMPUP  ||  0 != UI_speed )
        {
            UI_speed = dc;
        }
    }
    else
    {
        // reset needed in case system was running, in which case there is no
        // going back .. has to ramp again to get started.
        BL_reset(); // asserting this ... so what, system not running anyway!

        // assert (UI_speed == 0)  //  BL reset is supposed to set these initial conditions
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
    Manual_Ovrd = TRUE; //tbd

    BLDC_OL_comm_tm += 1; // slower
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_inc()
{
    Manual_Ovrd = TRUE; // tbd

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
 * @details
 *  External modules can query if the machine is running or not.
 *  There are only these two states bases on the set speed greater or less than
 *  the shutdown threshold.
 *
 * @return state value
 */
BL_RUNSTATE_t BL_get_state(void)
{
    if (UI_speed > PWM_DC_SHUTOFF )
    {
        return BL_IS_RUNNING;
    }
    // else
    return BL_NOT_RUNNING;
}

/**
 * @brief Periodic state machine update.
 *
 * @details
 * Called from TIM4 ISR. FRom the driver, the commutation-rate timer is being set
 * synchronous to this wihch is ideal, however, the control an probably be performed at
 * a lower rate (100Hz, 50Hz, 10Hz ..?). There is  no evident documentation of how this
 * timer rate came to be (TIM4 at ~ 0.5ms) Reducing it would give the commutation
 * time variable and therefore the control timing more precision .
 *
 * closed-loop control ... speed duty-cycle threshold, error switch sign? area under integratino curve
 */
void BLDC_Update(void)
{
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

    // refresh the duty-cycle and commutation period ... sets the pwm
    // which will be upated to the PWM timer peripheral at next commutation point.
    set_dutycycle( inp_dutycycle );

    // there isn't much point in enabling commuation timing contrl if speed is 0
    // and by leaving it along until the system is actually running, it can set
    // the initial condition in the global BL_Reset() above.
    if (inp_dutycycle > 0)
    {
        timing_ramp_control( Get_OL_Timing( inp_dutycycle ) );
    }

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

