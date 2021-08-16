/**
  ******************************************************************************
  * @file pwm_stm8s.c
  * @brief BLDC motor control - PWM, STM8s specific
  * @details
  * @author Neidermeier
  * @version
  * @date Sept-2020
  ******************************************************************************
  */
/**
 * @defgroup  MCU Platform STM8S
 * @brief STM8S platform and peripheral configuration.
 * @{
 */

/* Includes ------------------------------------------------------------------*/

// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>
#include "pwm_stm8s.h" // externalized macros used internally


/* Private defines -----------------------------------------------------------*/

/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t global_uDC;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ---------------------------------------------------------*/

/**
 * @brief Turn off PWM and disable all 3 phases.
 */
void All_phase_stop(void)
{
// kill the driver signals
    PWM_PhA_Disable();
    PWM_PhA_HB_DISABLE();

    PWM_PhB_Disable();
    PWM_PhB_HB_DISABLE();

    PWM_PhC_Disable();
    PWM_PhC_HB_DISABLE();
}

/**
 * @brief Accessor to update the duty cycle of the running PWM timer
 */
void PWM_set_dutycycle(uint16_t global_dutycycle)
{
    global_uDC = global_dutycycle;
}

/** @cond */ // hide the low-level code

/*
 * The S105 dev board unfortunately does not let the TIM2 CH3 pin (unless by alt. fundtion)
 */
#if defined ( S105_DISCOVERY ) || defined ( S003_DEV )

/*
 * Setup TIM2 PWM
 * Reference: AN3332
 *
 * 100% spped (0:1600) 100% PWM  results in 10 kHz PWM w/ prescale=1 @ 16Mhz
 *               0.000125 S / (1/16Mhz) * 1600 = 0.0001
 *               1 / 0.0001 = 10000  
 */
#ifdef PWM_8K
  #define TIM2_PRESCALER   TIM2_PRESCALER_8   //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
  #define TIM2_PRESCALER   TIM2_PRESCALER_2
#endif

#define PWM_TIMER_CHAN_A  TIM2_CHANNEL_1
#define PWM_TIMER_CHAN_B  TIM2_CHANNEL_2
#define PWM_TIMER_CHAN_C  TIM2_CHANNEL_3

void PWM_setup(void)
{
/* TIM2 Peripheral Configuration */
  TIM2_DeInit();

  /* Set TIM2 Frequency to 2Mhz */  /*  5/6/21 :  argument TIM2_Prescaler is 8-bit */
  TIM2_TimeBaseInit(TIM2_PRESCALER, PWM_PERIOD_COUNTS);
  /* Channel 1 PWM configuration */
  TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
//  TIM2_OC1PreloadConfig(ENABLE);

  /* Channel 2 PWM configuration */
  TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
//  TIM2_OC2PreloadConfig(ENABLE);

  /* Channel 3 PWM configuration */
  TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW );
//  TIM2_OC3PreloadConfig(ENABLE);

  /* Enables TIM2 peripheral Preload register on ARR */
//  TIM2_ARRPreloadConfig(ENABLE);

  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);  // for triggering ADC capture
  TIM2_Cmd(ENABLE);
}

/*
 * Operate /SD inputs to IR2104
 */
void PWM_PhA_Disable(void)
{
    TIM2_CCxCmd( PWM_TIMER_CHAN_A, DISABLE );
}

void PWM_PhB_Disable(void)
{
    TIM2_CCxCmd( PWM_TIMER_CHAN_B, DISABLE );
}

void PWM_PhC_Disable(void)
{
    TIM2_CCxCmd( PWM_TIMER_CHAN_C, DISABLE );
}

void PWM_PhA_Enable(void)
{
    TIM2_SetCompare1( global_uDC );
    TIM2_CCxCmd( PWM_TIMER_CHAN_A, ENABLE );
}

void PWM_PhB_Enable(void)
{
    TIM2_SetCompare2( global_uDC );
    TIM2_CCxCmd( PWM_TIMER_CHAN_B, ENABLE );
}

void PWM_PhC_Enable(void)
{
    TIM2_SetCompare3( global_uDC );
    TIM2_CCxCmd( PWM_TIMER_CHAN_C, ENABLE );
}

#elif defined ( S105_DEV )

#ifdef PWM_8K
  #define TIM1_PRESCALER 8  //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
  #define TIM1_PRESCALER 2
#endif

#define PWM_MODE  TIM1_OCMODE_PWM2

#define PWM_TIMER_CHAN_A  TIM1_CHANNEL_2
#define PWM_TIMER_CHAN_B  TIM1_CHANNEL_3
#define PWM_TIMER_CHAN_C  TIM1_CHANNEL_4

void PWM_setup(void)
{
    const uint16_t T1_Period = PWM_PERIOD_COUNTS;

//    CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER1, ENABLE);  // with clocks setup

    TIM1_DeInit();
/*
 * The counter clock frequency fCK_CNT is equal to fCK_PSC / (PSCR[15:0]+1)  (RM0016)
 */
    TIM1_TimeBaseInit(( TIM1_PRESCALER - 1 ), TIM1_COUNTERMODE_UP, T1_Period, 0);

    /* Channel 2 PWM configuration */
    TIM1_OC2Init( PWM_MODE,
                  TIM1_OUTPUTSTATE_ENABLE,
                  TIM1_OUTPUTNSTATE_ENABLE,
                  0,
                  TIM1_OCPOLARITY_LOW,
                  TIM1_OCNPOLARITY_LOW,
                  TIM1_OCIDLESTATE_RESET,
                  TIM1_OCNIDLESTATE_RESET);

    /* Channel 3 PWM configuration */
    TIM1_OC3Init( PWM_MODE,
                  TIM1_OUTPUTSTATE_ENABLE,
                  TIM1_OUTPUTNSTATE_ENABLE,
                  0,
                  TIM1_OCPOLARITY_LOW,
                  TIM1_OCNPOLARITY_LOW,
                  TIM1_OCIDLESTATE_RESET,
                  TIM1_OCNIDLESTATE_RESET);

    /* Channel 4 PWM configuration */
    TIM1_OC4Init(PWM_MODE,
                 TIM1_OUTPUTSTATE_ENABLE,
                 0,
                 TIM1_OCPOLARITY_LOW,
                 TIM1_OCIDLESTATE_RESET);

    TIM1_CtrlPWMOutputs(ENABLE);

    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);  // for triggering ADC capture
    TIM1_Cmd(ENABLE);
}
/**
 * Control /SD inputs to IR2104
 */
void PWM_PhA_Disable(void)
{
    TIM1_CCxCmd( PWM_TIMER_CHAN_A, DISABLE );
}

void PWM_PhB_Disable(void)
{
    TIM1_CCxCmd( PWM_TIMER_CHAN_B, DISABLE );
}

void PWM_PhC_Disable(void)
{
    TIM1_CCxCmd( PWM_TIMER_CHAN_C, DISABLE );
}

void PWM_PhA_Enable(void)
{
    TIM1_SetCompare2( global_uDC );
    TIM1_CCxCmd( PWM_TIMER_CHAN_A, ENABLE );
}

void PWM_PhB_Enable(void)
{
    TIM1_SetCompare3( global_uDC );
    TIM1_CCxCmd( PWM_TIMER_CHAN_B, ENABLE );
}

void PWM_PhC_Enable(void)
{
    TIM1_SetCompare4( global_uDC );
    TIM1_CCxCmd( PWM_TIMER_CHAN_C, ENABLE );
}

#endif // S105

/** @endcond */


/**
 * @brief Converts servo pulse width to servo position
 * @details 100% of servo throttle range resides in the portion of the servo
 *    pulse i.e. (1.1 ms : 1.9 ms) i.e. 
 *      servo position = servo pulse time - 1.1 ms
 *
 * @return duration of servo pulse expressed as timer counts, range (0:1600)
 */
uint16_t PWM_get_servo_position_counts( uint16_t pulse_duration_counts )
{
  uint16_t servo_position_counts = 0;

  if (pulse_duration_counts > TCC_TIME_ARMING)
  {
    servo_position_counts = pulse_duration_counts - TCC_TIME_ARMING;
  }
  return servo_position_counts;
}

/**
 * @brief converts servo pulse width to motor speed percent
 * @details Commanded motor speed is derived from proportional servo pulse
 *  so it is converted to percent of throttle/speed range.
 *
 * @param pulse_period_count  servo pulse period in timer counts
 * @param pulse_duration_count servo pulse duration in timer counts 
 *
 * @return Motor speed percent, integer range (0:1:100)
 */
uint16_t PWM_get_motor_spd_pcnt(
         uint16_t pulse_period_counts, uint16_t pulse_duration_counts)
{
  uint16_t motor_pcnt_speed = 0;
  uint16_t servo_position_counts = 
                     PWM_get_servo_position_counts( pulse_duration_counts );

// PWM percent duty-cycle is only for display purpose so some loss of precision 
// is ok here and necessary to prevent overflow out of 16-bit unsigned

  motor_pcnt_speed = (uint16_t)PWM_MSPEED_PERCENT( servo_position_counts );

  return motor_pcnt_speed;
}

/**@}*/ // defgroup
