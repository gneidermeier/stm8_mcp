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
#include "driver.h"

//#include "pwm_stm8s.h" // ng .. pulls in stm8 specific crap
extern void set_dutycycle(uint16_t);
extern uint16_t get_dutycycle(void);

extern void TIM3_setup(uint16_t u16period);


/* Private defines -----------------------------------------------------------*/

//#define V_SHUTDOWN_THR      0x0368 // experimental  ...startup stalls are still possible!
#define V_SHUTDOWN_THR      0x02c0


#define PWM_0PCNT      0

#define PWM_10PCNT     ( PWM_100PCNT / 10 )
#define PWM_20PCNT     ( PWM_100PCNT / 5 )
#define PWM_50PCNT     ( PWM_100PCNT / 2 )

#define PWM_X_PCNT( _PCNT_ )   ( _PCNT_ * PWM_100PCNT / 100 )

/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
#define PWM_DC_RAMPUP  PWM_X_PCNT( 14.0 )

#define PWM_DC_IDLE    PWM_X_PCNT( 12.0 )  // 0x1E ... 30 * 0.4 = 12.0

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

#define BLDC_OL_TM_HI_SPD       0x03C0 //  960d

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED         0x006F // 111

//#define BLDC_OL_TM_MANUAL_HI_LIM   LUDICROUS_SPEED

/*
 * Slope of what is basically a linear startup ramp, commutation time (i.e. TIM3)
 * period) decremented by fixed amount each control-loop timestep. Slope
 * determined by experiment (conservative to avoid stalling the motor!)
 */
#define BLDC_ONE_RAMP_UNIT      2


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

uint16_t Vsystem;


/* Private variables ---------------------------------------------------------*/

static int vsys_fault_bucket;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


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
    const int FAULT_BUCKET_INI = 128;

    // some actions only done on state transitions (prev state must be updated at end of this function)
    static BLDC_STATE_T prev_bldc_state = BLDC_OFF;

// if the voltage threshold is high enuff, the ramp delay time thingy not needed
//    const int RAMP_TIME = 1;   // fault arming delay time
//    static uint16_t fault_arming_time; // fault_arming_time

    uint16_t u16tmp;

    BLDC_STATE_T bldc_state = get_bldc_state();

    switch ( bldc_state )
    {
    default:
    case BLDC_OFF:
        // reset commutation timer and ramp-up counters ready for ramp-up
        set_commutation_period( BLDC_OL_TM_LO_SPD );

        vsys_fault_bucket = FAULT_BUCKET_INI;

        // delay to wait to stabillize at first DC setpoint post-ramp
//        fault_arming_time = RAMP_TIME;    /////// // reset the static ramp timer

        set_dutycycle( PWM_0PCNT );
        break;

    case BLDC_ON:
        // do ON stuff
        Vsystem = get_vbatt() / 2 + Vsystem / 2; // sma
#if 0  // .. Todo: needs to adjust threshold for in-ramp
        if ( fault_arming_time  > 0 )
        {
            fault_arming_time   -= 1;
        }
        else    // assert (Vbatt > VVV )
#endif
        {
            // check system voltage ... is motor stalled?
            if (Vsystem < V_SHUTDOWN_THR)
            {
                // voltage has sagged ... likely motor stall!
                if ( vsys_fault_bucket  > 0)
                {
                    vsys_fault_bucket -= 1; //
                }
            }
            else
            {
                if ( vsys_fault_bucket  < FAULT_BUCKET_INI )
                {
                    vsys_fault_bucket += 1; // refillin leaky bucket
                }
            }
// finally, check if fault is set
            if (0 == vsys_fault_bucket)
            {
                // 0 DC safely stops the motor, user must still press STOP to cycle the program.
                set_dutycycle( PWM_0PCNT );
            }
        }

// grab the "speed" number from the table, determine (sign of) error and incr. +/- 1
/*
basically theres a possiblity that at a high enough speed it could instead take
the error of the +/- back-EMF sensed ZC   and use e * Kp to determine the step
*/
        if ( 0 == get_op_mode() )
        {
            int error, step = 0;
            uint16_t u16ct = get_commutation_period();

            uint16_t table_value = Get_OL_Timing( get_dutycycle() );

            error = table_value - u16ct;
            /*
             * the C-T increments between PWM steps are rather large as speeding up
             * so it may be possible to comp. by reducing rate of this loop by /2
             */
            if (error > 0)
            {
                step = 1;
            }
            else if (error < 0)
            {
                step = -1;
            }

            if (table_value != 0) // assert
            {
                set_commutation_period( u16ct + step );  // incrementally adjust until error reduces to 0.
            }
        }

        break;

    case BLDC_RAMPUP:

        if (BLDC_RAMPUP != prev_bldc_state)
        {
            // set the ramp DC upon transition into ramp state
            set_dutycycle( PWM_DC_RAMPUP );
        }

        u16tmp = get_commutation_period();

        if ( u16tmp > BLDC_OL_TM_HI_SPD) // state-transition trigger?
        {
            set_commutation_period( u16tmp - BLDC_ONE_RAMP_UNIT );
        }
        else
        {
            set_bldc_state( BLDC_ON );

            Vsystem = get_vbatt(); // "pre-load" the avergae to avoid kicking out at end of ramp1

            set_op_mode( 0 ); // Manual Mode
            set_dutycycle( PWM_DC_IDLE );
        }
        break;
    }

    prev_bldc_state = bldc_state ;

//  update the timer for the OL commutation switch time
    TIM3_setup( get_commutation_period() );
}
