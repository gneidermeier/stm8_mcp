/**
  ******************************************************************************
  * @file sequence.c
  * @brief support functions for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "pwm_stm8s.h"

#include "sequence.h" // exported types referenced internally


// tmp
extern uint16_t Back_EMF_Falling_4[4]; // 4 samples per commutation period


/* Private defines -----------------------------------------------------------*/




/* Private types -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/



/* Public variables  ---------------------------------------------------------*/

int Back_EMF_Falling_Int_PhX;


/* Private variables  ---------------------------------------------------------*/

static COMMUTATION_SECTOR_t Sequence_step;

/*
 * This table simply defines the "trapezoidal" waveform in 6-steps.
 * The underlying PWM management scheme would be introduced elsewheres.
 */
static const BLDC_COMM_STEP_t Commutation_Steps[] =
{
// sector 0:
    { DC_OUTP_HI,      DC_OUTP_LO,      DC_OUTP_FLOAT_F },
// sector 1:
    { DC_OUTP_HI,      DC_OUTP_FLOAT_R, DC_OUTP_LO },
// sector 2:
    { DC_OUTP_FLOAT_F,  DC_OUTP_HI,     DC_OUTP_LO },
// sector 3:
    { DC_OUTP_LO,       DC_OUTP_HI,     DC_OUTP_FLOAT_R },
// sector 4:
    { DC_OUTP_LO,       DC_OUTP_FLOAT_F, DC_OUTP_HI },
// sector 5:
    { DC_OUTP_FLOAT_R,  DC_OUTP_LO,      DC_OUTP_HI }
};



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

/*
 * public accessor for the fixed table
 */
BLDC_COMM_STEP_t Seq_Get_Step( COMMUTATION_SECTOR_t sec )
{
    int index = (int) sec;

    BLDC_COMM_STEP_t step =  Commutation_Steps[ index ];
    return step;
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
 * high-level handler for commutation-step sequence
 */
void Sequence_Step(void)
{
    const uint8_t N_CSTEPS = 6;

    // grab the state of previous sector (before advancing the 6-step sequence)
    BLDC_COMM_STEP_t curr_step = Seq_Get_Step( Sequence_step );
    BLDC_PWM_STATE_t prev_A = curr_step.phA;

    if (DC_OUTP_FLOAT_F == prev_A )
    {
// if phase-A previous sector was floating-falling transition, then the measurements are qualified by copying from the temp array

// sum the pre-ZCP and post-ZCP measurements
        Back_EMF_Falling_Int_PhX =
            Back_EMF_Falling_4[1] + Back_EMF_Falling_4[2];
    }

    comm_switch( Sequence_step );

    Sequence_step = (Sequence_step + 1) % N_CSTEPS;
}
