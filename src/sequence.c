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
#include <stddef.h> // NULL
#include "pwm_stm8s.h"
#include "driver.h"
#include "bldc_sm.h"


/* Private defines -----------------------------------------------------------*/
/**
 * Plausibility of back-EMF measurement: the threshold (somewhat arbitrary) is
 * based on taking the average of the latest back-EMF leading-side and trailing-
 * side measurements, i.e.:
 * [ (Back_EMF_Falling_PhX + Back_EMF_Riseing_PhX) > BACK_EMF_PLAUS_THR ]
 * The open-loop ramp-to speed should ensure this condition.
 */
#define  BACK_EMF_PLAUS_THR  0x03F8

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

/*
 * Timing error term -  magnitude of the "falling"
 * (right) side is proportional to the degree of timing advance .. advanced
 * condition is seen when there is greater distribution of back-emf area on the
 * right side.
 */
// There isn't a timing control point to use to determine the error - however
// ratio of the leading and falling slopes (integration) indicates the direction
// and varies according to the magnitude of the timing error.
//  ratio = ( L / F  ) - 1
static int16_t comm_tm_err_ratio;

#define SCALE_64_LSH   6
#define SCALE_64_ONE  (1 << SCALE_64_LSH)


/* Private functions ---------------------------------------------------------*/

static void sector_0(void)
{
//    { DC_OUTP_HI,       DC_OUTP_LO,       DC_OUTP_FLOAT_F,

// previously phase-A was floating-rising transition
#ifdef BUFFER_ADC_BEMF
  Back_EMF_Riseing_PhX = ( Back_EMF_Riseing_PhX + Driver_Get_Back_EMF_Avg() ) >> 1 ;
#else
  Back_EMF_Riseing_PhX = ( Back_EMF_Riseing_PhX + Driver_Get_ADC() ) >> 1 ;
#endif
//PWM OFF: C
  PWM_PhC_Disable();

//Float: C
  PWM_PhC_HB_DISABLE();

//PWM_LO: B
  PWM_PhB_OUTP_LO();
  PWM_PhB_HB_ENABLE();

//PWM_HI: A
  PWM_PhA_Enable();
  PWM_PhA_HB_ENABLE();
}

static void sector_1(void)
{
//    { DC_OUTP_HI,       DC_OUTP_FLOAT_R,  DC_OUTP_LO,

//PWM OFF:
//        NOP

//Float: B
  PWM_PhB_HB_DISABLE();

//PWM_LO: C
  PWM_PhC_OUTP_LO();
  PWM_PhC_HB_ENABLE();

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
  PWM_PhA_HB_DISABLE();

//PWM_LO: C
  PWM_PhC_OUTP_LO();
  PWM_PhC_HB_ENABLE();

//PWM_HI: B
  PWM_PhB_Enable();
  PWM_PhB_HB_ENABLE();
}

static void sector_3(void)
{
// previously phase-A was floating-falling transition
#ifdef BUFFER_ADC_BEMF
  Back_EMF_Falling_PhX = ( Back_EMF_Falling_PhX + Driver_Get_Back_EMF_Avg() ) >> 1;
#else
  Back_EMF_Falling_PhX = ( Back_EMF_Falling_PhX + Driver_Get_ADC() ) >> 1;
#endif
//    { DC_OUTP_LO,       DC_OUTP_HI,       DC_OUTP_FLOAT_R,

//PWM OFF:
//        NOP

//Float: C
  PWM_PhC_HB_DISABLE();

//PWM_LO: A
  PWM_PhA_OUTP_LO();
  PWM_PhA_HB_ENABLE();

//PWM_HI:
//      NOP (B)
}

static void sector_4(void)
{
//    { DC_OUTP_LO,       DC_OUTP_FLOAT_F,  DC_OUTP_HI,

//PWM OFF: B
  PWM_PhB_Disable();
//Float: B
  PWM_PhB_HB_DISABLE();
//PWM_LO: A
  PWM_PhA_OUTP_LO();
  PWM_PhA_HB_ENABLE();
//PWM_HI: C
  PWM_PhC_Enable();
  PWM_PhC_HB_ENABLE();
}

static void sector_5(void)
{
//    { DC_OUTP_FLOAT_R,  DC_OUTP_LO,       DC_OUTP_HI,

//PWM OFF:
//        NOP
//Float: A
  PWM_PhA_HB_DISABLE();
//PWM_LO: B
  PWM_PhB_OUTP_LO();
  PWM_PhB_HB_ENABLE();
//PWM_HI:
//        NOP (C)

// update the timing error once per frame
//  comm_timing_error = (comm_timing_error + TIMING_ERROR_TERM) > 1; // sma

  // signed_error_ratio = ( post / pre ) - 1
  // Uses scalar of 64 to get most precision from ADC 10-bit terms (assuming max 0x03ff).
  // ADC 10-bit i.e. 0x03FF << 6 = 0xFFC0
  // Calculation result gets scaled down in conjunction with factoring in of
  //  controller gain term(s).
  comm_tm_err_ratio =
    (int16_t)( ( Back_EMF_Falling_PhX << SCALE_64_LSH ) / Back_EMF_Riseing_PhX )
    - (int16_t)SCALE_64_ONE;
}

/* Public functions ---------------------------------------------------------*/
#if 0
/**
 * @brief  Determine plausibility of Control error term.
 *
 * @details If the motor timing is advanced, the control error should be
 *  positive i.e.  increasing commutation period slows the motor.
 *
 * @param[in]  p16term
 *             pointer to the 16-bit integer error term allocated by the caller
 *
 * @return signed error which at its extreme should be equal to or less than the
 *  range of the initial ADC measurement i.e. 0x0400
 */
int Seq_get_timing_error_p(int16_t * p16term)
{
//    int16_t perror = comm_timing_error; // positive if advanced
  int err = -1; // report error on measurement plausibility

  if ( (Back_EMF_Falling_PhX + Back_EMF_Riseing_PhX) > BACK_EMF_PLAUS_THR )
  {
    err = 0;
// 4/30: turns out there is only one caller, and it is not using the error term (passes in NULL)
#if 0 
    if ( NULL != int16p)
    {
      *int16p = Seq_get_timing_error(); // comm_tm_err_ratio;  // positive if advanced
    }
#endif
  }
  return err;
}
#endif
/**
 * @brief Accessor for control error term
 *
 * @details If the motor timing is advanced, the control error should be
 *  positive i.e. increasing commutation period slows the motor.
 *
 * @return signed error which at its extreme should be equal to or less than the
 *  range of the initial ADC measurement i.e. 0x0400
 */
int16_t Seq_get_timing_error(void)
{
  return comm_tm_err_ratio; // positive if advanced
}

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

  static uint8_t s_step;

// has to cast modulus expression to uint8
  s_step = (uint8_t)((s_step + 1) % N_CSTEPS);

// intentionally letting motor windmill (i.e. not braking) when switched off
// normally
  if (BL_IS_RUNNING == BL_get_state() )
  {
    // let'er rip!
    step_ptr_table[s_step]();
  }
  else
  {
    // intitialize the averages measurements 
    Back_EMF_Riseing_PhX = Back_EMF_Falling_PhX = Vbatt_ = 0;
  }
}

/**@}*/ // defgroup
