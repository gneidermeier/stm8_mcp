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

#include <string.h>

#include "mcu_stm8s.h"
#include "bldc_sm.h"
#include "sequence.h"
#include "per_task.h"


/* Private defines -----------------------------------------------------------*/

#define PH0_ADC_TBUF_SZ  8

/*
 TODO: resistor divider values must be updated for 3.3v operation
*/

// half of 10-bit ADC which is also assumed to be exactly half DC thanks to perfectly sized resistor divider, unflinching supply voltage set exactly to 13.8v
#define MID_ADC 0x0200  // tbd ... DC HALF

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
static uint16_t ph0_adc_fbuf[PH0_ADC_TBUF_SZ];

static uint8_t  ph0_adc_tbct;

static uint16_t phase_average;

static uint16_t prev_pulse_start_tm;
static uint16_t curr_pulse_start_tm;

static uint16_t Pulse_perd;
static uint16_t Pulse_dur;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
#ifdef BUFFER_ADC_BEMF
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
}

/**
 * @brief Call from timer/capture ISR on capture of falling edge of servo pulse
 */
void Driver_on_capture_fall(void)
{
// noise on this signal when motor running
//    uint16_t t16 =  TIM1_GetCapture3() - TIM1_GetCapture4();
//    Pulse_dur = (Pulse_dur + t16) >> 1; // sma
  Pulse_dur = get_pulse_end() - get_pulse_start();
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
 * @brief  Hook for synchronizing to the PWM pulse.
 *
 * @details  Invoked from timer ISR.
 * Phase voltage measurements must be synchronized to PWM.
 * Initiates ADC sample sequence on start of PWM pulse.
 */
void Driver_on_PWM_edge(void)
{
#ifdef BUFFER_ADC_BEMF
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
#ifdef BUFFER_ADC_BEMF
// assert (buffer should be sized big enough for slowest speed)

// TODO: ph0_adc_fbuf can simply be a running sma ... there is no real need to
// reset the average at each commutation switch????!!!???

  if (ph0_adc_tbct < PH0_ADC_TBUF_SZ)
  {
    ph0_adc_fbuf[ph0_adc_tbct] = ADC_Global ;
  }
#endif
}

#ifdef BUFFER_ADC_BEMF
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
 * @details  Called from timer ISR. The commutation time is updated each time the
 * control task is updated, and in turn the timer reload register value is refreshed
 * from latest calculated commutation time period.
 */
void Driver_Update(void)
{
#ifdef CLOCK_16
  static const uint8_t UI_UPDATEM = 32; // 16 Mhz sysclock
#else
  static const uint8_t UI_UPDATEM = 16; // 8 Mhz sysclock
#endif

  static uint8_t trate = 0;

  // the controller and the UI are updated on alternate frames (doubled the
  // timer rate) presently ths update done every 1.024mS so the controller rate ~1Khz
  if ( 0 != ( ++trate & 0x01 ) )
  {
    BLDC_Update();
  }
  else if ( 0 == (trate % UI_UPDATEM))
  {
#if 1
    /* Toggles LED */
    GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
#endif
    Periodic_Task_Wake();
  }

  // update the commutation switch timer period
  MCU_set_comm_timer( get_commutation_period() );
}


/**
 * @brief  Top-level task for commutation switching sequence
 *
 * @details  Invoked from timer ISR
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
#ifdef BUFFER_ADC_BEMF
    udpate_phase_average(); // average 8 samples from frame buffer
#endif
    Sequence_Step();
    break;

  case 1:
  case 2:
  case 3:
    break;
  }
}
/**@}*/ // defgroup
