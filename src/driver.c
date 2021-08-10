/**
  ******************************************************************************
  * @file driver.c
  * @brief  Interface functions for motor control.
  * @author Neidermeier
  * @version
  * @date March-2020
  ******************************************************************************
  */
/**
 * \defgroup driver  Driver
 * @brief  Interface functions for motor control.
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "mcu_stm8s.h"
#include "bldc_sm.h"
#include "per_task.h"

#include "pwm_stm8s.h"

/* Private defines -----------------------------------------------------------*/


/**
  * @brief convert throttle position timer-counts to percent motor speed
  *
  *   speed % = 100 % * throttle_position_counts / throttle_range_counts
  *
  * Range of throttle position timer counts is approx. (0:1600)
  *
  *   1/1600 == 0.000625 which would give the pwm step a bit more precision than 
  * needed (most ESCs are probably providing 1024 steps of pwm resolution.
  *
  * Constant terms are typically grouped to put as much of the calculation in 
  * the preprocesor as possible (including the division of factor of 4, enough
  *	to prevent any intermediate result from overflowing out of uint16).
  */

#define _THROTTLE_POSITION_COUNT( _PULSE_TIMER_COUNTS_ )  \
                         (uint16_t)( _PULSE_TIMER_COUNTS_ - TCC_THRTTLE_0PCNT )



//#define PH0_ADC_TBUF_SZ  8

/*
 TODO: system voltage should be measured at startup
*/
// divider: 33k/18k
//  18/(18+33)=0.35
// 0.35 * 14.1v = 4.98
// 4.98 / 2 = 2.48v ........... 1/2 Vdc in proportion to the resister divider
//  2.48v/5v =  x counts / 1024 ocunts so 1/2 Vdc is equivalent to x counts ...
//   x = 1024 * 2.48/5 = 509   (0x01FD)
#define DC_HALF_REF         0 // 0x01FD

#define GET_BACK_EMF_ADC( ) \
    ( ADC_Global - DC_HALF_REF )


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

#define FOUR_SECTORS  4 // each commutation sector of 60-degrees spans 4x TIM3 periods


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint16_t ADC_Global;

// Accummulates a string of 10-bit ADC samples for averaging - could reduce
// to 8 bits as possibly the 2 lsb's are not that significant anyway.
//static uint16_t ph0_adc_fbuf[PH0_ADC_TBUF_SZ];
//static uint8_t  ph0_adc_tbct;
//static uint16_t phase_average;

static uint16_t prev_pulse_start_tm;
static uint16_t curr_pulse_start_tm;

static uint16_t Pulse_perd;
static uint16_t Pulse_dur;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
#if 0 // BUFFER_ADC_BEMF
/*
 * averag 8 samples .. could be inline or macro
 */
static void udpate_phase_average(void)
{
  int n;
// use midpoint  as initial condition which yields neutral control action
  phase_average = MID_ADC;

  for(n  = 0 ; n < PH0_ADC_TBUF_SZ; n++)
  {
    phase_average = (phase_average  + ph0_adc_fbuf[n]) >> 1;

    ph0_adc_fbuf[n] = MID_ADC; // initialize each element to midpoint
  }

  ph0_adc_tbct = 0; // reset the sample index
}
#endif

/* External functions ---------------------------------------------------------*/

/** @cond */

#if defined( S105_DEV )

uint16_t get_pulse_start(void)
{
    return TIM2_GetCapture1();
}

uint16_t get_pulse_end(void)
{
    return TIM2_GetCapture2();
}

#elif defined(S105_DISCOVERY)

uint16_t get_pulse_start(void)
{
    return TIM1_GetCapture4();
}

uint16_t get_pulse_end(void)
{
    return TIM1_GetCapture3();
}

#else // unimplemented ... stm8s003

uint16_t get_pulse_start(void)
{
    return (uint16_t)-1;
}

uint16_t get_pulse_end(void)
{
    return (uint16_t)-1;
}
#endif
/** @endcond */

/**
 * @brief Call from timer/capture ISR on capture of rising edge of servo pulse
 *
*/
void Driver_on_capture_rise(void)
{
// 16-bit counter setup to wrap at 0xffff so no concern for sign of result
  prev_pulse_start_tm = curr_pulse_start_tm;
  curr_pulse_start_tm = get_pulse_start();
  Pulse_perd = curr_pulse_start_tm  - prev_pulse_start_tm;

// set test pin
//    GPIO_WriteHigh(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
}

/**
 * @brief Call from timer/capture ISR on capture of falling edge of servo pulse
 */
void Driver_on_capture_fall(void)
{

// noise on this signal when motor running?

    uint16_t t16 = get_pulse_end() - curr_pulse_start_tm /* get_pulse_start() */;

// apply exponential filter (simple moving average) to smoothe the signal
    Pulse_dur = (Pulse_dur + t16) >> 1; // sma

// clear test pin
//    GPIO_WriteLow(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
}

/**
 * @brief Accessor for measured pulse period.
 */
uint16_t Driver_get_pulse_perd(void)
{
  return Pulse_perd;
}

/**
 * @brief Accessor for measured pulse duration.
 */
uint16_t Driver_get_pulse_dur(void)
{
  return Pulse_dur;
}

/**
 * @brief converts servo pulse width to motor speed percent
 * @details Commanded motor speed is derived from proportional servo pulse.
 *  Motor speed is to be at least 0.1% i.e integer scale factor of 16 provides 
 *  precision of 0.0625% (per bit). 
 *  Note: it is possible that the industry requirement for PWM resolution might 
 *  be 1024 steps - however, that could be a bit of a challenge with 16-bit 
 *  word size.
 * @return Motor speed percent scaled to SPEED PCNT SCALE.
 */
uint16_t Driver_get_motor_spd_pcnt(void)
{
  uint16_t motor_pcnt_speed = 0;
  uint16_t pulse_period_count = Driver_get_pulse_perd();
  uint16_t pulse_duration_count = Driver_get_pulse_dur();

  if (pulse_duration_count > TCC_THRTTLE_0PCNT)
  {
    uint16_t thr_posn_cnt = 
      (uint16_t )_THROTTLE_POSITION_COUNT( pulse_duration_count );

// PWM percent duty-cycle is only for display purpose so some loss of precision 
// is ok here and necessary to prevent overflow out of 16-bit unsigned
    motor_pcnt_speed  = 
      (uint16_t )( (100.0 / 4) * thr_posn_cnt) / (TCC_THRTTLE_RANGE / 4);
  }
  return motor_pcnt_speed;
}

/**
 * @brief  Hook for synchronizing to the PWM pulse.
 *
 * @details  Invoked from timer ISR.
 * Phase voltage measurements must be synchronized to PWM.
 * Initiates ADC sample sequence on start of PWM pulse.
 */
void Driver_on_PWM_edge(void)
{
#if 0 // BUFFER_ADC_BEMF
  ph0_adc_tbct += 1 ; // advance the buffer index
#endif
// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
  ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion
  ADC1_StartConversion();
}

/**
 * @brief  Capture ADC conversion channel 0 to buffer
 *
 * @details  Captures phase voltage measurement from ADC Channel 0, to be
 * used as back-EMF sensing or system voltage.
 * Called from ADC1 ISR.
 */
void Driver_on_ADC_conv(void)
{
  ADC_Global = ADC1_GetBufferValue( ADC1_CHANNEL_0 );

#if 0 // BUFFER_ADC_BEMF
  if (ph0_adc_tbct < PH0_ADC_TBUF_SZ)
  {
    ph0_adc_fbuf[ph0_adc_tbct] = ADC_Global ;
  }
#endif
}

#if 0 // BUFFER_ADC_BEMF
/** @cond */
/**
 * @brief Get Back-EMF buffer averaged.
 *
 * @details 4 samples are captured within a single commutation sector.
 *
 * @return  Calculated average of 4 samples stored in back-EMF frame buffer
 */
uint16_t Driver_Get_Back_EMF_Avg(void)
{
  return phase_average;
}
/** @endcond */
#endif

/**
 * @brief Accessor for system voltage measurement.
 * @details Phase voltage measurement from ADC Channel 0 is to be used as
 * back-EMF sensing or system voltage.
 * @return  Most recent captured ADC conversion value from Channel 0
 */
uint16_t Driver_Get_ADC(void)
{
  return ADC_Global;
}

/**
 * @brief  Invoke background task and control task.
 *
 * @details  Called from timer ISR. The timer reload register value is refreshed
 * from latest calculated commutation time period.
 *
 * Driven by PWM timer ISR
 *  e.g.
 *   timer period = fMaster * PS * 100%DC 
 *                                   = (1/16 Mhz) * 8 * 250 counts -> 0.000125 S
 *
 * The ISR multiplexes the PWM capture with Driver upate, but the driver update 
 * only requires x/hz
 *
 *   timer period * ISR Frame Count * CT_FRAME 
 *                               = 0.000125 * 4 ISRs * 2 count -> 0.001 seconds 
 *
 * The Periodic Task (UI) update is targetted to ~60 Hz (.0167 ms) i.e.
 *   timer period * ISR Frame Count * UI_FRAME 
 *                              = 0.000125 * 4 ISRs * 32 count -> 0.016 seconds 
 *   
 */
void Driver_Update(void)
{
  static const uint8_t UI_FRAME = 0x20; // UI task every 32 steps (even)
  static const uint8_t CT_FRAME = 0x01; // control task on odd steps

  static uint8_t trate = 0;
  trate += 1;

  // the controller and the UI are updated on alternate frames (doubled the
  // timer rate) presently ths update done every 1.024mS so the controller rate ~1Khz
  if ( 0 != (trate & CT_FRAME))
  {
    BL_State_Ctrl();  // update commutation timing controller 

    // refresh the timer with the updated commutation time period
    MCU_set_comm_timer( BL_get_timing() );

#if 0
    /* Toggles LED to verify task timing */
    GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
#endif
  }
  else if (0 == (trate % UI_FRAME))
  {
    Periodic_Task_Wake();

#if 0
    /* Toggles LED to verify task timing */
    GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
#endif
  }
}


/**
 * @brief  Top-level task for commutation switching sequence
 *
 * @details  Invoked from timer ISR
 * Each SequenceStep() is at 60-electrical-degrees.
 * Timing is established by:
 *  timer period * commutation period 
 * The Sequence Step is called every 4th iteration (equivalent to 60-degrees)
 *  ( i.e. the timer period is 1/4 the time duration of 60-degrees).
 * e.g.
 *  frame time = fMaster * PS * timer period
 *  60 electrical degrees = timer period * 4 frames
 */
void Driver_Step(void)
{
  static const int SectorC = FOUR_SECTORS;
  static int index = 0;

// Since the modulus being used (4) is a power of 2, then a bitwise & can be used
// instead of a MOD (%) to save a few instructions, which is actually significant
// as this is a very high frequency ISR!
  index = (index + 1) & (SectorC - 1);

// Distribute the work done in the ISR by partitioning
//  sequence_step, memcpy,  get_ADC into  separate sub-steps
// Logically the call to Sequence_Step() occurs following the memcpy()

  switch(index)
  {
  case 0:
#if 0 // BUFFER_ADC_BEMF
    udpate_phase_average(); // average 8 samples from frame buffer
#endif
    BL_Commutation_Step();
    break;

  case 1:
  case 2:
  case 3:
    break;
  }

#if 0
    /* Toggles LED */
    GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
#endif
}
/**@}*/ // defgroup
