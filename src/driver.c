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
#include "driver.h"

/* Private defines -----------------------------------------------------------*/
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

/* Private types -----------------------------------------------------------*/
/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t ADC_Global;
static uint16_t prev_pulse_start_tm;
static uint16_t curr_pulse_start_tm;
static uint16_t Pulse_perd;
static uint16_t Pulse_dur;

static uint8_t rxReceive[RX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* External functions ---------------------------------------------------------*/

/** @cond */

/*
 * accessors ********************************
 */
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

void Driver_set_pulse_dur(uint16_t duration)
{
  Pulse_dur = duration;
}

/**
 * @brief Converts servo pulse width to servo position
 * @details 100% of servo throttle range resides in the portion of the servo
 *    pulse i.e. (1.1 ms : 1.9 ms) i.e.
 *      servo position = servo pulse time - 1.1 ms
 *
 * @return duration of servo pulse expressed as timer counts
 */
uint16_t Driver_get_servo_position_counts(void)
{
  uint16_t pulse_duration_counts = Driver_get_pulse_dur();
  uint16_t thr_posn_cnt =
    PWM_get_servo_position_counts( pulse_duration_counts );

  return thr_posn_cnt;
}

/**
 * @brief converts servo pulse width to motor speed percent
 * @details Commanded motor speed is derived from proportional servo pulse
 *  so it is converted to percent of throttle/speed range.
 * @return Motor speed percent (integer).
 */
uint16_t Driver_get_motor_spd_pcnt(void)
{
  uint16_t motor_pcnt_speed = 0;
  uint16_t pulse_period_counts = Driver_get_pulse_perd();
  uint16_t pulse_duration_counts = Driver_get_pulse_dur();

// PWM percent duty-cycle is only for display purpose so some loss of precision
// is ok here and necessary to prevent overflow out of 16-bit unsigned
  motor_pcnt_speed  = PWM_get_motor_spd_pcnt(
                        pulse_period_counts, pulse_duration_counts);

  return motor_pcnt_speed;
}

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
 * @brief  Fill Rx Buffer in ISR Context
 *
 * @details  Invoked from Rx ISR
 */
void Driver_Get_Rx_It(void)
{
  static uint8_t rxPos = 0;

#if defined( S105_DEV ) || defined( S105_DISCOVERY )

  rxReceive[rxPos] = UART2_ReceiveData8();

#elif defined( S003_DEV )

  rxReceive[rxPos] = UART1_ReceiveData8();

#endif

  rxPos++;

  if (rxPos > (RX_BUFFER_SIZE - 1))
  {
    rxPos = 0;
  }
}

/**
 * @brief  Return Rx Buffer
 */

uint8_t Driver_Return_Rx_Buffer(void)
{
  static uint8_t rxReadLoc = 0;

  uint8_t tmp = rxReceive[rxReadLoc];

  rxReadLoc += 1;

  if (rxReadLoc > (RX_BUFFER_SIZE - 1))
  {
    rxReadLoc = 0;
  }

  return tmp;
}

/**
 * @brief  Clear Rx Buffer Element
 */

void Driver_Clear_Rx_Buffer_Element(uint8_t Location)
{
  rxReceive[Location] = 0;
}

/*
 * event handlers ********************************
 */

/**
 * @brief Call from timer/capture ISR on capture of rising edge of servo pulse
 *
*/
/*
 * need to save global/static of "raw" value so that can check for lost signal
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
  uint16_t t16 = get_pulse_end() - curr_pulse_start_tm /* get_pulse_start() */;

// apply exponential filter (simple moving average) to smoothe the signal
  Pulse_dur = (Pulse_dur + t16) >> 1; // sma

// clear test pin
//    GPIO_WriteLow(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
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
  /* Toggles LED to verify task timing */
//  GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);

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
  ADC_Global = ADC1_GetBufferValue( PH0_BEMF_IN_CH );
}

/**
 * @brief  System Timer event handler
 *
 * @details
 *   Event handler for System Timer ISR. The event handler multiplexes the
 *   BL Control Task and the Periodic Task which occur at different rates.
 *   Responsible for updating the Commutation Timer period - invokes accessors
 *   from both MCU and BL classes.
 *
 *   System Timer period = fMaster * PS * 100%DC
 *                                   = (1/16 Mhz) * 8 * 250 counts -> 0.000125 S
 *
 *    BL Control Timer frequency                            =
 *      timer period * ISR frame count * CT_FRAME       =
 *      0.000125 sec * 4 ISRs          * 2 timer events = 0.001 seconds (1000 Hz)
 *
 *    Periodic Task Timer frequency                     =
 *      timer period * ISR frame count * UI_FRAME       =
 *      0.000125     * 4 ISRs          * 32 timer events = 0.0167 seconds (60 Hz)
 *
 */
void Driver_Update(void)
{
  static const uint8_t UI_FRAME = 0x20; // UI task every 32 steps (even) i.e. 32 * 0.5 ms
  static const uint8_t CT_FRAME = 0x01; // control task on odd steps i.e. @ 1 ms

  static uint8_t trate = 0;
  trate += 1;

  // the controller and the UI are updated on alternate frames (doubled the
  // timer rate) presently ths update done every 1.024mS so the controller rate ~1Khz
  if ( 0 != (trate & CT_FRAME))
  {
    BL_State_Ctrl();  // update commutation timing controller

    // refresh the timer with the updated commutation time period
    MCU_set_comm_timer( BL_get_timing() );

    /* Toggles LED to verify task timing */
    //GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
  }
  else if (0 == (trate % UI_FRAME))
  {
    Periodic_Task_Wake();

    /* Toggles LED to verify task timing */
    // GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
  }
}

/**
 * @brief  Top-level task for commutation switching sequence
 *
 * @details  Invoked from timer ISR
 *
 *   Every 4th timer event constitutes a 60-degree commutation "sector" at which
 *   time _Commutation_Step() is invoked.
 *   The timer was set up 4x faster than the commutation rate as a provision to
 *   coordinate PWM sampling for zero-crossing detection (still TBD).
 */
void Driver_Step(void)
{
  static const uint8_t Modulus = 4;
  static int index = 0;

  // The modulus being used (4) is a power of 2, so a bitwise & can be used
  // instead of a MOD (%) to save a few instructions, which is actually significant
  // as this is a very high frequency ISR!
  index = (index + 1) & (Modulus - 1);

  switch(index)
  {
  case 0:
    BL_Commutation_Step();
    break;

  case 1:
  case 2:
  case 3:
  default:
    break;
  }

  /* Toggles LED */
  //GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);
}

/**@}*/ // defgroup
