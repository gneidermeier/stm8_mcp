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
#include "driver.h"
#include "bldc_sm.h"


/* Private defines -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/

/*
 * Each commutation step sequence is implemented as a simple function -
 * aggregating the function pointers into a table.
 */
typedef void (*step_ptr_t)( void /* int */ );


/* Private function prototypes -----------------------------------------------*/

static void sector_0(void /* int iarg */);
static void sector_1(void /* int iarg */);
static void sector_2(void /* int iarg */);
static void sector_3(void /* int iarg */);
static void sector_4(void /* int iarg */);
static void sector_5(void /* int iarg */);


/* Public variables  ---------------------------------------------------------*/

uint16_t Back_EMF_Falling_PhX;
uint16_t Back_EMF_Riseing_PhX;

/* Private variables  ---------------------------------------------------------*/

static uint16_t Vbatt_;

static const step_ptr_t step_ptr_table[] =
{
    sector_0,
    sector_1,
    sector_2,
    sector_3,
    sector_4,
    sector_5
};


/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/


/*
 * public accessor for the system voltage measurement
 * Vbatt is in this module because the measurement has to be timed to the
 * PWM/commutation sequence
 */
uint16_t Seq_Get_Vbatt(void)
{
    return Vbatt_;
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
  * Third: turn on LO phase
  * The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off (IR2104 enabled)
  *
  * Fourth: turn on HI (pwm) phase
  *
  * Ideally, when a phase is at 60 degrees (half of its pwm-on sector) there should be
  * no change/disruption to its PWM signal.
  *
  * TIM2 counter is not reset or cleared - only the TIM2 PWM channel is switched
  * between the 3 motor phases, so the overall PWM cycle should remain consistent.
  * This routine is getting excessively long (50us) and is quite possble to
  * overrun the TIM2 time for PWM pulse? That would add to jitter.
  */

static void sector_0(void)
{
//    { DC_OUTP_HI,       DC_OUTP_LO,       DC_OUTP_FLOAT_F,

// previously phase-A was floating-rising transition
    Back_EMF_Riseing_PhX = 
        ( Back_EMF_Riseing_PhX + Driver_Get_Back_EMF_Avg() ) >> 1 ;

//PWM OFF: C
    PWM_PhC_Disable();
//Float: C
    PWM_PhC_HB_DISABLE(0);
//PWM_LO: B
    PWM_PhB_OUTP_LO( 0 );
    PWM_PhB_HB_ENABLE(1);
//PWM_HI: A
    PWM_PhA_Enable();
    PWM_PhA_HB_ENABLE(1);
}

static void sector_1(void)
{
//    { DC_OUTP_HI,       DC_OUTP_FLOAT_R,  DC_OUTP_LO,

//PWM OFF:
//        NOP
//Float: B
    PWM_PhB_HB_DISABLE(0);
//PWM_LO: C
    PWM_PhC_OUTP_LO( 0 );
    PWM_PhC_HB_ENABLE(1);
//PWM_HI:
//      NOP (A)
}

static void sector_2(void)
{
    // Phase A was driven pwm, so use the ADC measurement as vbat
    Vbatt_ = Driver_Get_ADC();

//    { DC_OUTP_FLOAT_F,  DC_OUTP_HI,       DC_OUTP_LO,

//PWM OFF: A
    PWM_PhA_Disable();
//Float: A
    PWM_PhA_HB_DISABLE(0);
//PWM_LO: C
    PWM_PhC_OUTP_LO( 0 );
    PWM_PhC_HB_ENABLE(1);
//PWM_HI: B
    PWM_PhB_Enable();
    PWM_PhB_HB_ENABLE(1);
}

static void sector_3(void)
{
// previously phase-A was floating-falling transition
    Back_EMF_Falling_PhX = 
       ( Back_EMF_Falling_PhX + Driver_Get_Back_EMF_Avg() ) >> 1;

//    { DC_OUTP_LO,       DC_OUTP_HI,       DC_OUTP_FLOAT_R,

//PWM OFF:
//        NOP
//Float: C
    PWM_PhC_HB_DISABLE(0);
//PWM_LO: A
    PWM_PhA_OUTP_LO( 0 );
    PWM_PhA_HB_ENABLE(1);
//PWM_HI:
//      NOP (B)
}

static void sector_4(void)
{
//    { DC_OUTP_LO,       DC_OUTP_FLOAT_F,  DC_OUTP_HI,

//PWM OFF: B
    PWM_PhB_Disable();
//Float: B
    PWM_PhB_HB_DISABLE(0);
//PWM_LO: A
    PWM_PhA_OUTP_LO( 0 );
    PWM_PhA_HB_ENABLE(1);
//PWM_HI: C
    PWM_PhC_Enable();
    PWM_PhC_HB_ENABLE(1);
}

static void sector_5(void)
{
//    { DC_OUTP_FLOAT_R,  DC_OUTP_LO,       DC_OUTP_HI,

//PWM OFF:
//        NOP
//Float: A
    PWM_PhA_HB_DISABLE(0);
//PWM_LO: B
    PWM_PhB_OUTP_LO( 0 );
    PWM_PhB_HB_ENABLE(1);
//PWM_HI:
//        NOP (C)
}

/*
 * high-level handler for commutation-step sequence
 */
void Sequence_Step(void)
{
    // note this sizeof and divide done in preprocessor - verified in the assembly
    const uint8_t N_CSTEPS = sizeof(step_ptr_table) / sizeof(step_ptr_t);

    static uint8_t Sequence_step;

    Sequence_step = (Sequence_step + 1) % N_CSTEPS;

// motor freewheels when switch to off
    if (BLDC_OFF == get_bldc_state() )
    {
        // stopped ... make things be initialized for the next startup ...
        Back_EMF_Riseing_PhX = 0;
        Back_EMF_Falling_PhX = 0;
    }
    else
    {
        // let'er rip!
        step_ptr_table[Sequence_step]();
    }
}
