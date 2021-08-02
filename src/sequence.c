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
typedef enum
{
  SECTOR_0 = 0,
  SECTOR_1,
  SECTOR_2,
  SECTOR_3,
  SECTOR_4,
  SECTOR_5,
  SECTOR_INVALID = -1
}
Seq_sector_t;

/**
* @brief  Table of handler functions for 6 commutation steps.
*
* @details  Each commutation step sequence is implemented as a simple function and
*  function pointers are aggregated into a single table.
*
* @return  void
*/
typedef void (*step_ptr_t)( void );


/* Private function prototypes -----------------------------------------------*/

static void sector_0(void);
static void sector_1(void);
static void sector_2(void);
static void sector_3(void);
static void sector_4(void);
static void sector_5(void);


/* Public variables  ---------------------------------------------------------*/

/** 
 * @brief state variable for the sequencer
 */
Seq_sector_t Seq_step;

/** @cond */ // hide some developer/debug code
uint16_t Back_EMF_Falling_PhX;
uint16_t Back_EMF_Riseing_PhX;
/** @endcond */

/* Private variables  ---------------------------------------------------------*/

static uint16_t Vbatt_;

/**
 * @brief commutation timing steps (6)
 * @details
 *  Each commutation sector spans 60 "electrical" degrees:
 *                      | 60 | 60 | 60 | 60 | 60 | 60 | 60 |
 *  Phase high-drive                   |<--120-->| 60 |<--
 *        ---- ----                     ---- ----
 *                  \                 /           \
 *                   \               /             \
 *                    \             /               \
 *                     \ ____ ____ /                 \
 *  Phase low-drive     |<--120-->| 60 |<--
 *
 *  Each of 3 phases is driven according to the waveform shown above but offset
 *  from one another by 120 degrees.
 *  A PWM drive scheme is used to control the phase output in proportion to the
 *  speed reference signal (0:100 percent). The are a number of strategies
 *  involving combinations of driving PWM on either the low-side switch, the
 *  high-side switch, and combinations of both.The PWM rate must be high enough
 *  that the high-side is not driven for any time longer than the boost capacitor
 *  can hold up the gate voltage (i.e. the PWM period is not too long - in this
 *  project at least 8 Mhz will be used). A simple scheme is adopted here
 *  whereby PWM signal is applied to the gate of the high-side switch during the
 *  high-side drive sector of 120-degrees.
 *  During the sectors low-side sector (120 degrees) the phase output is at
 *  0 V DC.
 *  when the low-side switch is driven, the logic signal to the FET gate is 100%
 *  "On" across the entire 120-degrees, resulting in the phase of 0 V DC.

 *  RPM of 6 pole-pair motor would be the commutation rate divided by 6.

 *  Each switch can be in 1 of 2 states:   ON / OFF
 *  Each 1/2 bridge has 2 logic inputs:
 *    IN: logic input which is typically driven by PWM signal from the uC
 *    /SD: logic input which essential enables or disables the bridge
 *  Each 1/2 bridge has 2 outputs:
 *    OUT_H: output to gate of high-side switch (typically ~20v for 2S-3S voltage range)
 *    OUT_L: output to gate of low-side switch (close to battery voltage)
 *
 *  The high-side and low-side switch of the half-bridge must never switch on \
 *  simultaneously which would practically result in a dead short Vbat->GND. The
 *  half-bridge drives the high and low outputs in a complementary fashion and by
 *  design provides a minimum specified dead-time between switching the output
 *  states. The signal to the HB from the micro discrete outputs is represented by
 *  the following truth table where each motor phase output may be in 1 of 3
 *  possible states:
 *    IN   /SD   |  OUT_H   OUT_L    PHASE OUTPUT  |  NOTE
 *     0     1   |    OFF      ON    OFF           |  DC 0 V (HB is enabled, low-side driven off/0v)
 *     1     1   |     ON     OFF    ON            |  PWM on High Side switch
 *     x     0   |     ON     OFF    FLOAT         |  phase output floats (negative-going or positive-going)
 *
 *  A simple PWM drive scheme is used whereby PWM signal is applied to the gate
 *  of the high-side switch during its 120-degrees of "On" time. During the sectors
 *  when the low-side switch is driven, the logic signal to the FET gate is 100%
 *  "On" across the entire 120-degrees, resulting in the phase of 0 V DC.
 *
 * Ideally, when a phase is at 60 degrees (half of its pwm-on sector) there should
 * be no change/disruption to its PWM signal. The timer capture/counter register
 * should not be reset or cleared - only the active timer/PWM channel is switched
 * between the 3 motor phases, so the overall PWM cycle should remain consistent.
 *
 * (1): turn off PWM of high-side phase (N-1) to start the "demagnization time" and
 * flyback-diode of the non-PWM switch conducts flyback current
 *
 * (2): assert /SD ==OFF  of (only!) the phase (N-1) PWM drive half-bridge - to ensure
 * that flyback diode action is complete (de-energizing the coil that is now being
 * transitioned to floating) - seems to be the only way to ensure the IR2104
 * asserts both switches non-conducting.
 *
 * Third: turn on low-side switch of phase (N+0) ^H^H^H^H - low-side phase (N+0) is
 * already active from the previous 60-degrees sector.
 *
 * Fourth: enable PWM on high-side switch phase (N+0).
 */
static const step_ptr_t step_ptr_table[] =
{
  sector_0, //    { A_PWM_HS,       B_OFF_LS,       C_FLOAT_NEG
  sector_1, //    { A_PWM_HS,       B_FLOAT_POS,    C_OFF_LS,
  sector_2, //    { A_FLOAT_NEG,    B_PWM_HS,       C_OFF_LS,
  sector_3, //    { A_OFF_LS,       B_PWM_HS,       C_FLOAT_POS,
  sector_4, //    { A_OFF_LS,       B_FLOAT_NEG,    C_PWM_HS,
  sector_5  //    { A_FLOAT_POS,    B_OFF_LS,       C_PWM_HS,
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

/*
 * Sector 0:  A_PWM_HS | B_OFF_LS | C_FLOAT_NEG
 *
 * Previous sector (5) phase-A was floating (positive-going transition) so ADC
 * input from phase A resistor divider is taken as average back-EMF voltage
 * during the positive-going float sector.
 */
static void sector_0(void)
{
#ifdef BUFFER_ADC_BEMF
  Back_EMF_Riseing_PhX = ( Back_EMF_Riseing_PhX + Driver_Get_Back_EMF_Avg() ) >> 1 ;
#else
  Back_EMF_Riseing_PhX = ( Back_EMF_Riseing_PhX + Driver_Get_ADC() ) >> 1 ;
#endif

// C FLOAT NEG
  PWM_PhC_Disable(); // phase C PWM asserted off (negative-going float)
  PWM_PhC_HB_DISABLE(); // half-bridge disable

// B_OFF_LS  (NO-OP?)
  PWM_PhB_OUTP_LO();
  PWM_PhB_HB_ENABLE(); // half-bridge enable

// A PWM HS
  PWM_PhA_Enable(); // PWM on
  PWM_PhA_HB_ENABLE(); // half-bridge enable
}

/*
 * Sector 1:  A_PWM_HS | B_FLOAT_POS | C_OFF_LS
 */
static void sector_1(void)
{
// B FLOAT POS
  PWM_PhC_Disable(); // Phase C PWM asserted off (positive-going float)
  PWM_PhB_HB_DISABLE();

// C OFF LS
  PWM_PhC_OUTP_LO();
  PWM_PhC_HB_ENABLE();

// A PWM HS
//  PWM_PhA_Enable(); // NO
//  PWM_PhA_HB_ENABLE(); // NO-OP?
}

/*
 * Sector 2:  A_FLOAT_NEG | B_PWM_HS | C_OFF_LS
 *
 * Phase A was driven PWM, so that latest ADC input from phase A resistor divider
 * (during PWM on-time is taken as battery/system voltage.
 */
static void sector_2(void)
{
  Vbatt_ = Driver_Get_ADC();

// A FLOAT NEG
  PWM_PhA_Disable();  // phase A PWM asserted off (negative-going float)
  PWM_PhA_HB_DISABLE();

// C OFF LS  (NO-OP?)
  PWM_PhC_OUTP_LO();
  PWM_PhC_HB_ENABLE();

// B PWM HS
  PWM_PhB_Enable();
  PWM_PhB_HB_ENABLE();
}

/*
 * Sector 3:  A_OFF_LS | B_PWM_HS | C_FLOAT_POS
 *
 * ADC input from phase A resistor divider is taken as average back-EMF voltage
 * (previously phase-A was floating-falling transition).

 * Previous sector (2) phase-A was floating (negative-going transition) so ADC
 * input from phase A resistor divider is taken as average back-EMF voltage
 * during the negative-going float sector.
 */
static void sector_3(void)
{
#ifdef BUFFER_ADC_BEMF
  Back_EMF_Falling_PhX = ( Back_EMF_Falling_PhX + Driver_Get_Back_EMF_Avg() ) >> 1;
#else
  Back_EMF_Falling_PhX = ( Back_EMF_Falling_PhX + Driver_Get_ADC() ) >> 1;
#endif

// C FLOAT POS
  PWM_PhC_Disable(); //phase C PWM asserted off (positive-going float)
  PWM_PhC_HB_DISABLE();

// A OFF LS
  PWM_PhA_OUTP_LO();
  PWM_PhA_HB_ENABLE();

// B PWM HS
//  PWM_PhB_Enable(); // NO
//  PWM_PhB_HB_ENABLE(); // NO-OP?
}

/*
 * Sector 4:  A_OFF_LS | B_FLOAT_NEG | C_PWM_HS,
 */
static void sector_4(void)
{
// B FLOAT NEG
  PWM_PhB_Disable(); // phase B PWM asserted off (negative going float)
  PWM_PhB_HB_DISABLE();

// A OFF LS  (NO-OP?)
  PWM_PhA_OUTP_LO();
  PWM_PhA_HB_ENABLE();

// C PWM HS
  PWM_PhC_Enable();
  PWM_PhC_HB_ENABLE();
}

/*
 * Sector 5:  A_FLOAT_POS | B_OFF_LS | C_PWM_HS
 *
 * Note:  update the timing error term once per frame
 */
static void sector_5(void)
{
// A FLOAT POS
  PWM_PhA_OUTP_LO(); // phase A PWM asserted off (positive-going float)
  PWM_PhA_HB_DISABLE();

// B OFF LS
  PWM_PhB_OUTP_LO();
  PWM_PhB_HB_ENABLE();

// C PWM HS
//  PWM_PhC_Enable();  // NO
//  PWM_PhC_HB_ENABLE(); NO-OP?


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
/**
 * @brief  Determine plausibility of Control error term.
 *
 * @details If the motor timing is advanced, the control error should be
 *  positive i.e. increasing commutation period slows the motor.
 *
 * @return  Signed integer error.
 */
int8_t Seq_get_timing_error_p(void)
{
  if ( (Back_EMF_Falling_PhX + Back_EMF_Riseing_PhX) > BACK_EMF_PLAUS_THR )
  {
    return (int8_t)0;
  }
  return (int8_t)(-1);
}

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
 * @brief  Accessor for back-EMF measurement.
 */
uint16_t Seq_Get_bemfR(void)
{
  return Back_EMF_Riseing_PhX ;
}

/**
 * @brief  Accessor for back-EMF measurement.
 */
uint16_t Seq_Get_bemfF(void)
{
  return Back_EMF_Falling_PhX ;
}

/**
 * @brief  Accessor for system voltage measurement.
 *
 * @details  Vbatt is in this module because the measurement must be coordinated
 *           with PWM On time.
 */
uint16_t Seq_Get_Vbatt(void)
{
  return Vbatt_;
}


/**
 * @brief Public accessor for step 0 in the commutation sequence function table
 * 
 * @details The sequence is initialized by the Alignment to Ramp transition. The
 *     Alignment sets the Sector to 0 and waits some time for the motor to align.
 *     After alignment, the control state machine enables the sequence which will
 *     be pointed to Sector 1 as the next step. 
 */
void Sequence_Step_0(void)
{
  Seq_step = SECTOR_0;
  step_ptr_table[ Seq_step ]();

  // point to sector 1 for next sequence step
  Seq_step = SECTOR_1;
}

/**
 * @brief  Updates the commutation-step sequence.
 *
 * @details  Handler for commutation-sequencing ISR.
 */
void Sequence_Step(void)
{
  // note this sizeof and divide done in preprocessor - verified in the assembly
  const uint8_t N_CSTEPS = sizeof(step_ptr_table) / sizeof(step_ptr_t);


// has to cast modulus expression to uint8
  Seq_step = (uint8_t)(( Seq_step + 1 ) % N_CSTEPS);

// intentionally letting motor windmill (i.e. not braking) when switched off
// normally
  if (BL_IS_RUNNING == BL_get_state() )
  {
    // let'er rip!
    step_ptr_table[Seq_step]();
  }
  else
  {
    // intitialize the average
    Back_EMF_Riseing_PhX = Back_EMF_Falling_PhX = Vbatt_ = 0;
  }
}

/**@}*/ // defgroup
