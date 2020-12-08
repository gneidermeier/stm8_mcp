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

#include <string.h>

#include "pwm_stm8s.h"
#include "mdata.h" // motor timing curve
#include "bldc_sm.h"
#include "sequence.h"

#include "driver.h" // internal declarations

/*
 * wanton abuse of globals hall of fame
 */
extern void TIM3_setup(uint16_t u16period); // from main.c


/* Private defines -----------------------------------------------------------*/

// divider: 33k/18k
//  18/(18+33)=0.35
// 0.35 * 14.1v = 4.98
// 4.98 / 2 = 2.48v ........... 1/2 Vdc in proportion to the resister divider
//  2.48v/5v =  x counts / 1024 ocunts so 1/2 Vdc is equivalent to x counts ...
//   x = 1024 * 2.48/5 = 509   (0x01FD)
#define DC_HALF_REF         0x01FD 

#define GET_BACK_EMF_ADC( ) \
    ( ADC_Global - DC_HALF_REF )


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

#define TIM3_RATE_MODULUS   4 // each commutation sector of 60-degrees spans 4x TIM3 periods


#define BLDC_OL_TM_LO_SPD       0x1000 // 4096d  // start of ramp

#define BLDC_OL_TM_HI_SPD       0x03C0 //  960d

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED         0x006F // 111 

#define BLDC_OL_TM_MANUAL_HI_LIM   LUDICROUS_SPEED



/* Private types -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


int Back_EMF_Falling_Int_PhX; // take whatever the favored (widest) machine signed int happens to be ...
                              // todo: stms8.h has  typedef   signed long     int32_t; 

uint16_t Back_EMF_Falling_4[4]; // 4 samples per commutation period


/* Private variables ---------------------------------------------------------*/

static uint16_t ADC_Global;

/* Vbatt is in this module because the measurement has to be timed to the 
 * PWM/commutation sequence */
static uint16_t Vbatt;

static uint16_t Back_EMF_15304560[4];


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * back-EMF single-channel start and let ISR to signal EOC
 * Only doing phase A right now if that will suffice.
 *
 * Noted AN2658 "sampling time is not customizable and
 * depends on the ADC clock (3 ADC clocks)"
 */
void bemf_samp_start( void )
{
    const ADC1_Channel_TypeDef ADC1_ChannelX = ADC1_CHANNEL_3; // ADC1_CHANNEL_0; // only phase A

    ADC1_ConversionConfig(
        ADC1_CONVERSIONMODE_SINGLE, ADC1_ChannelX, ADC1_ALIGN_RIGHT);

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion
    ADC1_StartConversion();
}

/*
 * back-EMF single-channel get sample - only CH 0
 * Called from ADC1 ISR
 */
void bemf_samp_get(void)
{
    ADC_Global = ADC1_GetBufferValue( ADC1_CHANNEL_0 ); // ADC1_GetConversionValue();
}


/*
 * need to define an interface to the ADC sample/conversion service
 */
uint16_t Driver_Get_ADC(void)
{
    return ADC_Global;
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   reference:
  *    http://embedded-lab.com/blog/starting-stm8-microcontrollers/21/
  *    - pulse width modulation frequency determined by the value of the TIM1_ARR register
  *    - duty cycle determined by the value of the TIM1_CCRi register
  *
  *    "120° Square-Wave Commutation for Brushless DC Motors" (Toshiba Electronic)
  *
  * PWM is carefully disabled and then to assert the state of the output pins.
  * This can mess with back-EMF component of phase voltage.

  * First: shutoff PWM (before setting any of the new FET states) to ensure PWM
  *  leg is turned off and flyback-diode of non-PWM conducts flyback current("demagnization time".)
  *
  * Second: assert /SD ==OFF  of (only!) the PWMd FET - to ensure that flyback
  *  diode is complete (de-energizing the coil that is now being transitioned
  *  to float). This seems to be the only way to ensure IR2104 set both switch non-conducting.
  *
  * Ideally, when a phase is a 60degress (half of its time) there should be
  * no change/disruption to  its PWM signal.
  *
  * TIM1 counter is not reset or cleared - only the PWM TIM1 channel is changed
  * for the phase, so the overall PWM rate should be maintained. 
  * This routine is getting excessively long (50us) and is quite possble to 
  * overrun the TIM1 time.for PWM pulse? That would add to jitter.
  */
static void comm_switch (COMMUTATION_SECTOR_t bldc_step)
{
    static COMMUTATION_SECTOR_t prev_bldc_step = 0;

    BLDC_PWM_STATE_t prev_A, prev_B, prev_C ;
    BLDC_PWM_STATE_t state0, state1, state2;

    // grab the phases states of previous sector 
    BLDC_COMM_STEP_t curr_step = Seq_Get_Step( bldc_step );
    BLDC_COMM_STEP_t prev_step = Seq_Get_Step( prev_bldc_step );
    prev_A = prev_step.phA;
    prev_B = prev_step.phB;
    prev_C = prev_step.phC;

    prev_bldc_step = bldc_step; // latch the state of the next previous ;)

    state0 = curr_step.phA;
    state1 = curr_step.phB;
    state2 = curr_step.phC;

    /*
     * Disable PWM of previous driving phase is finished (120 degrees). Note that
     * an active TIM1 PWM pulse could be interrupted. Probably adds to the overall jitter
     * It's possible that there could be benefit to a delay here to wait for 
     * the PWM pulse (as long as overal time of this function is not excessive...which  it already is!
     */
    if ( DC_OUTP_HI == prev_A  && ( DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0 ) )
    {
        PWM_PhA_Disable();
    }
    if ( DC_OUTP_HI == prev_B  && ( DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1 ) )
    {
        PWM_PhB_Disable();
    }
    if ( DC_OUTP_HI == prev_C  && ( DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2 ) )
    {
        PWM_PhC_Disable();
    }


    if (DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0)
    {
//    PWM_PhA_OUTP_LO( 0 ); ?
        PWM_PhA_HB_DISABLE(0);
    }
    else if (DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1)
    {
//    PWM_PhB_OUTP_LO( 0 ); ?
        PWM_PhB_HB_DISABLE(0);
    }
    else if (DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2)
    {
//    PWM_PhC_OUTP_LO( 0 ); ?
        PWM_PhC_HB_DISABLE(0);
    }


/*
 * The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off (IR2104 enabled)
 */
    if (DC_OUTP_LO == state0)
    {
// let the Timer PWM channel remain disabled, PC2 is LO, /SD.A is ON
        PWM_PhA_OUTP_LO( 0 );
        PWM_PhA_HB_ENABLE(1);
    }
    else if (DC_OUTP_LO == state1)
    {
// let the Timer PWM channel remain disabled, PC3 is LO, /SD.B is ON
        PWM_PhB_OUTP_LO( 0 );
        PWM_PhB_HB_ENABLE(1);
    }
    else if (DC_OUTP_LO == state2)
    {
// let the Timer PWM channel remain disabled, PC4 is LO, /SD.C is ON
        PWM_PhC_OUTP_LO( 0 );
        PWM_PhC_HB_ENABLE(1);
    }


    /*
     * reconfig and re-enable PWM of the driving channels. One driving channel is
     * PWMd, the other is continuously Off. Both driving IR2104s must be enabeld
     * by setting its /SD input line.
     */
    if (DC_OUTP_HI == state0)
    {
        PWM_PhA_Enable();
        PWM_PhA_HB_ENABLE(1);
    }

    if (DC_OUTP_HI == state1)
    {
        PWM_PhB_Enable();
        PWM_PhB_HB_ENABLE(1);
    }

    if (DC_OUTP_HI == state2)
    {
        PWM_PhC_Enable();
        PWM_PhC_HB_ENABLE(1);
    }
}



/*
 * low-level stop: turns off all PWM
 */
void Driver_Stop(void)
{
// kill the driver signals
    PWM_PhA_Disable();
    PWM_PhA_HB_DISABLE(0);

    PWM_PhB_Disable();
    PWM_PhB_HB_DISABLE(0);

    PWM_PhC_Disable();
    PWM_PhC_HB_DISABLE(0);
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */


uint16_t get_vbatt(void)
{
    return Vbatt;
}


/*
 * BLDC Update: 
 *  Called from ISR
 */
void Driver_Update(void)
{
    BLDC_Update();

//  update the timer for the OL commutation switch time
    TIM3_setup( get_commutation_period() );

}

/*
 * called from ISR
 *
// establish error-signal ...
// measures at 4 15-degree intervals - [1] and [2] are the valid ones to "integrate"
// Note at this low idle/open-loop speed there are only about 4 PWM @ 8k during
// a 60-degree sector.
// Pulses close to start/end of the secctor are problematic anyway as the ADC 
// ISR ends up getting blocked by the TIM3 ISR ... the BLDC_Step() takes about
// to 40us on case 3!
 */
void Driver_Step(void)
{
    static uint8_t bldc_step_modul; // internal counter for sub-dividing the TIM3 period

    int index = bldc_step_modul % TIM3_RATE_MODULUS;

// Note if wanting all 3 phases would need to coordinate here to set the
// correct ADC channel to sample.

    switch(index)
    {
    case 0:
        Back_EMF_15304560[0] = GET_BACK_EMF_ADC( );
        break;
    case 1:
        Back_EMF_15304560[1] = GET_BACK_EMF_ADC( );
        break;
    case 2:
        Back_EMF_15304560[2] = GET_BACK_EMF_ADC( );
        break;
    case 3:
        Back_EMF_15304560[3] = GET_BACK_EMF_ADC( );

        memcpy( Back_EMF_Falling_4, Back_EMF_15304560, sizeof(Back_EMF_Falling_4) );

        Sequence_Step();

        break;
    }
    bldc_step_modul += 1; // can allow rollover as modulus is power of 2
}

/*
 * high-level handler for commutation-step sequence
 */
void Sequence_Step(void)
{
    const uint8_t N_CSTEPS = 6;

    static COMMUTATION_SECTOR_t comm_step = 0;

    if (BLDC_OFF != get_bldc_state() )
    {
        // grab the state of previous sector (before advancing the 6-step sequence)
        BLDC_COMM_STEP_t curr_step = Seq_Get_Step( comm_step );
        BLDC_PWM_STATE_t prev_A = curr_step.phA;


        if (DC_OUTP_FLOAT_F == prev_A )
        {
// if phase-A previous sector was floating-falling transition, then the measurements are qualified by copying from the temp array

// sum the pre-ZCP and post-ZCP measurements
            Back_EMF_Falling_Int_PhX =
                Back_EMF_Falling_4[1] + Back_EMF_Falling_4[2];
        }

        if (DC_OUTP_HI == prev_A)
        {
            // if phase was driven pwm, then use the measurement as vbat
            Vbatt = Driver_Get_ADC();
        }

        comm_switch( comm_step );

        comm_step = (comm_step + 1) % N_CSTEPS;
    }
}


