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
 * \defgroup  BLDC_sm BLDC State
 * @brief  BLDC state management and timing control
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>  // NULL
#include "bldc_sm.h" // external types used internally
#include "mdata.h"
#include "pwm_stm8s.h" // motor phase control
#include "faultm.h"
#include "sequence.h"

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Compute PWM timer period from percent duty-cycle
 *
 * @details Should be PWM_PCNT_TO_PERIOD or something like that.
 *
 * @note 100 percent PWM time:
 *   fMaster * timer_prescaler * 250 counts = (1/16Mhz) * 8 * 250 -> 0.000125 S
 */
#define PWM_X_PCNT( _PCNT_ )   (uint16_t)( ( _PCNT_ * PWM_100PCNT ) / 100.0 )

/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
#define PWM_DC_STARTUP   11.5
#define PWM_DC_SHUTOFF    6.8   // stalls if slower

#define PWM_PD_STARTUP   PWM_X_PCNT( PWM_DC_STARTUP )
#define PWM_PD_SHUTOFF   PWM_X_PCNT( PWM_DC_SHUTOFF )


 // commutation period at start of ramp (est. @ 12v) - exp. det.
#define BLDC_OL_TM_LO_SPD     (0x1600 * CTIME_SCALAR)

// ramp rate derived from control rate which is derived from overall system rate
// commutation time factor is rolled in there as well
#define BLDC_ONE_RAMP_UNIT    (1.0 * CTRL_RATEM * CTIME_SCALAR)


/* Private types -----------------------------------------------------------*/

/**
 * @brief Type for BL operating state.
 */
typedef enum
{
  BL_MANUAL,
  BL_ALIGN,
  BL_RAMPUP,
  BL_OPN_LOOP,
  BL_CLS_LOOP,
  BL_STOPPED
}
BL_State_T;


/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint16_t BLDC_OL_comm_tm; // persistent value of ramp timing
static uint8_t BL_pwm_period; // input from UI - made static global for access in ISR thread
static BL_State_T Control_mode; // BL operation state

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
  const uint8_t stepi = (uint8_t)BLDC_ONE_RAMP_UNIT;

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

/**
 * @Brief common sub for stopping and fault states
 *
 * @Detail 
 * Allows motor to be stopped in a fault condition, while allowing the system to 
 * remain in whatever operating state - does not reset the control state, fault 
 * manageer etc. This is a developers "feature" allowing the fault state and 
 * other info to be examined. 
 */
static void BL_stop(void)
{
// have to clear the local UI_speed since that is the transition OFF->RAMP condition
  BL_pwm_period = 0;

  // kill the driver signals
  All_phase_stop();
}

/* Public functions ---------------------------------------------------------*/

/**
 * @brief Initialize/reset motor
 *
 *    System reset / re-arm function (has to be called both at program startup
 *    as well as following a fault condition state.
 *
 * @details
 *    expect to be called from non-ISR/CS context (i.e. from  UI handler)
 */
void BL_reset(void)
{
  // assert PWM channels to disabled
  BL_stop();

  // Set initial commutation timing period upon state transition.
  // TIM3 is left enabled, so the commutation period (TIM3) is simply set to a 
  // arbitrarily large number. The TIM3 ISR will still fire but the commutation 
  // step logic has no effect as long as the PWM is disabled. 	
  BLDC_OL_comm_tm =  0xFFFF;

  Faultm_init();

  Control_mode = BL_STOPPED;  // set the initial control-state
}


/**
 * @brief Sets motor speed from commanded throttle/UI setting
 *
 * @details
 *  The motor is started once reaching the ramp speed threshold, and allowed to
 *  slow down to the low shutoff threshold.
 *  UI Speed is shared with background task so this function should
 *  be invoked only from within a CS.
 *
 * @param dc Speed input which can be in the range [0:255]
 *            TODO: needs to be in terms of percent of speed range (0:100)
 */
void BLDC_PWMDC_Set(uint8_t dc)
{
  if (dc > PWM_PD_SHUTOFF)
  {
    // Update the dc if speed input greater than ramp start, OR if system already running
    if ( dc > PWM_PD_STARTUP || 0 != BL_pwm_period  /* if Control_mode != STOPPED */ )
    {
      BL_pwm_period = dc;
    }
  } // if dc > START_OF_RAMP
  else // if (BL_STOPPED != Control_mode) // extra logic to only assert this upon transition event
  {
    // commanded speed less than low limit so reset - has to ramp again to get started.
    BL_reset();
  }
}

/**
 * @brief Accessor for Commanded Duty Cycle
 *
 * @return Commanded Duty Cycle
 */
uint16_t BLDC_PWMDC_Get(void)
{
  return (uint16_t)BL_pwm_period;
}

#ifdef ENABLE_COMM_INP
/** @cond */ // hide some developer/debug code
/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_dec()
{
  Control_mode = BL_MANUAL; //tbd

  BLDC_OL_comm_tm += 1; // slower
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_inc()
{
  Control_mode = BL_MANUAL; // tbd

  BLDC_OL_comm_tm -= 1; // faster
}
/** @endcond */
#endif

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
  if (BL_pwm_period > PWM_PD_SHUTOFF )
  {
    return BL_IS_RUNNING;
  }
  // else
  return BL_NOT_RUNNING;
}

/**
 * @brief  Accessor for state variable.
 *
 * @details
 *  Control Mode is TRUE if transitioned from open-loop to closed-loop
 *  commutation control.
 *
 * @return  state value
 */
uint8_t BL_get_ct_mode(void)
{
  return Control_mode;
}

// TBD, test code
// 1 bit is /2 for sma, 1 bit is /2 for "gain"
#define SMA_SH  1 // tmp
#define GAIN_SH  2 // tmp
/**
 * @brief  Top level task which perform commutation timing update.
 */
void BLDC_Update(void)
{
  static const uint8_t CONTROL_GAIN_SH  = (GAIN_SH + SMA_SH);
  static int16_t timing_error;

  uint16_t inp_dutycycle = 0; // in case of error, PWM output remains 0

  if ( 0 != Faultm_get_status() )
  {
    BL_stop(); // sets BL pwm period to 0 and disables timer PWM channels but
               // doesn't re-init the system state
  }
  else
  {
    inp_dutycycle = BL_pwm_period; // set pwm period from UI

    if (BL_STOPPED == Control_mode)
    {
      if (BL_pwm_period > 0)
      {
        Control_mode = BL_OPN_LOOP; // state-transition

        // Set initial commutation timing period upon state transition.
        BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;
      }
    }
    else if (BL_OPN_LOOP == Control_mode)
    {
      uint16_t olt = Get_OL_Timing( inp_dutycycle );
      timing_ramp_control( olt );

      // check plausibility condition for transition to closed-loop
      // if ( Seq_get_timing_error_p() )
      //           Control_mode = BL_CLS_LOOP; // state-transition
    }
    else if (BL_CLS_LOOP == Control_mode)
    {
#if 0 // test code
      // the control gain is macro'd together with the unscaling of the error term and also /2 of the sma
      uint16_t t16 = BLDC_OL_comm_tm ;
      timing_error = ( Seq_get_timing_error() + timing_error ) >> CONTROL_GAIN_SH;
      t16 += timing_error;
// if this overshoots, the control runs away until the TIM3 becomes so low the system locks up and no faultm can work .. not failsafe!
      if (t16 > LUDICROUS_SPEED)
      {
        BLDC_OL_comm_tm  = t16;
      }
#endif // test code
    }
    // else ...check for conditions if necessary to unlatch CL control mode? (too slow, lost sync)
  }

  // pwm duty-cycle will be upated to the timer peripheral at next commutation step.
  set_dutycycle( inp_dutycycle );
}
/**@}*/ // defgroup

