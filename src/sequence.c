/**
  ******************************************************************************
  * @file sequence.c
  * @brief  Coordinates commutation switching sequence.
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */
/**
 * @defgroup sequencer Commutation Sequencer
 * @brief  Coordinates commutation switching sequence.
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "pwm_stm8s.h"
#include "driver.h"
#include "bldc_sm.h"


/* Private defines -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/

 /**
 * @brief Pointer to step handler function.
 *
 * Each commutation step sequence is implemented as a simple function -
 * aggregating the function pointers into a table.
 * @return  void
 */
typedef void (*step_ptr_t)( void );


/* Private function prototypes -----------------------------------------------*/

static void sector_0(void /* int iarg */);
static void sector_1(void /* int iarg */);
static void sector_2(void /* int iarg */);
static void sector_3(void /* int iarg */);
static void sector_4(void /* int iarg */);
static void sector_5(void /* int iarg */);


/* Public variables  ---------------------------------------------------------*/

/** @cond */ // hide some developer/debug code
uint16_t Back_EMF_Falling_PhX;
uint16_t Back_EMF_Riseing_PhX;
/** @endcond */

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

/* Public functions ---------------------------------------------------------*/

/**
 * @brief Accessor for system voltage measurement.
 *
 * @details Vbatt is in this module because the measurement has to be timed to
 * the PWM signal as well as the commutation sequence.
 */
uint16_t Seq_Get_Vbatt(void)
{
    return Vbatt_;
}

/**
 * @brief  Updates the commutation-step sequence.
 *
 * @details  Called from ISR. The 6 steps of the  commutation sequence are
 * implemented as individual functions to reduce the amount of code i.e.
 * optimize the timing which is critical to the motor performance and stability.
 *
 * If the series of steps to assert the state of the output pins is not performed
 * in a specific sequence the back-EMF component of phase voltage may be impacted.
 *
 * Ideally, when a phase is at 60 degrees (half of its pwm-on sector) there should
 * be no change/disruption to its PWM signal. Also to maintain timing the TIM2
 * capture/counter register not reset or cleared - only the TIM2 PWM channel is
 * switched between the 3 motor phases, so the overall PWM cycle should remain consistent.

 *
 * First: shutoff PWM (before setting any of the new FET states) to ensure PWM
 * leg is turned off and flyback-diode of non-PWM conducts flyback current
 * ("demagnization time".)
 *
 * Second: assert /SD ==OFF  of (only!) the PWMd FET - to ensure that flyback
 * diode action is complete (de-energizing the coil that is now being transitioned
 * to floating). This seems to be the only way to ensure IR2104 set both switch
 * non-conducting.
 *
 * Third: turn on LO phase
 * The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off (IR2104 enabled)
 *
 * Fourth: turn on HI (pwm) phase
 */
void Sequence_Step(void)
{
    // note this sizeof and divide done in preprocessor - verified in the assembly
    const uint8_t N_CSTEPS = sizeof(step_ptr_table) / sizeof(step_ptr_t);

    static uint8_t Sequence_step;

    BLDC_STATE_T bldc_state = get_bldc_state();

    Sequence_step = (Sequence_step + 1) % N_CSTEPS;

// intentionally letting motor windmill (i.e. not braking) when switched off
// normally
    if (BLDC_RUNNING == bldc_state || BLDC_RAMPUP == bldc_state )
    {
        // let'er rip!
        step_ptr_table[Sequence_step]();
    }
    else
    {
        // intitialize the averages measurements
        Back_EMF_Riseing_PhX = Back_EMF_Falling_PhX = Vbatt_ = 0;
    }
}

/**@}*/ // defgroup
