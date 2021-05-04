/**
  ******************************************************************************
  * @file pwm_stm8s.c
  * @brief BLDC motor control - PWM, STM8s specific
  *
  *   TIM2 is being used despite TIM1 being the capable motor control features,
  *   but PC1 PC2 PC3 pins are taken up by  cap touch demo on the STM8-Discovery board
  *
  * Comments:
  *   Using the STM8 Peripheral Library, the diffrence between using TIM1 and TIM2
  *   API becomes practically a matter of '/TIM1/TIM2/s' i.e. search and replace.
  *   One difference, there is no TIM1_BKR register so only 'TIM1CtrlPWMOutpus()'
  *   TIMx may not have the other motor drive capabilities that are not being using anyway.
  *
  *   For some time, the nature of timing back-EMF to the flyback interval was
  *   not understood wrt to the various device switching steps, so the
  *   commutation switching logic may be excessively chopped up.
  *
  *	  For a while now I have been noticing that the output PWM CH pins would
  *   sometimes seem to float/decay when transition to floating 60-degree sector.
  *   As an extra step, one could assertively set those GPIOs direction/state to
  *   output off/low. However, once I also realized that the state of the pin isn't
  *   so critical as is the asserting the /SD of the IR2104 device, which is what
  *   actually makes the phase voltage float.
  *
  * @author Neidermeier
  * @version
  * @date Sept-2020
  ******************************************************************************
  */
/**
 * @defgroup mcu Platform STM8S
 * @brief STM8S platform and peripheral configuration.
 * @{
 */

/* Includes ------------------------------------------------------------------*/

// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>
#include "system.h" // PWM PERIOD
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
 * @brief Putter accessor for PWM duty cycle
 * @details Motor speed is controlled through the UI and converted to PWM duty cycle .
 */
void set_dutycycle(uint16_t global_dutycycle)
{
    global_uDC = global_dutycycle;
}


/*
 * The S105 dev board unfortunately does not let the TIM2 CH3 pin (unless by alt. fundtion)
 */
#if defined ( S105_DEV ) //   || defined ( S105_DISCOVERY )


#elif defined ( S003_DEV ) || defined ( S105_DISCOVERY )
/*
 * Setup TIM2 PWM
 * Reference: AN3332
 */
#ifdef CLOCK_16
#define TIM2_PRESCALER TIM2_PRESCALER_8  //    (1/16Mhz) * 8 * 250 -> 0.000125 S
#else
#define TIM2_PRESCALER TIM2_PRESCALER_4  //    (1/8Mhz)  * 4 * 250 -> 0.000125 S
#endif

void PWM_setup(void)
{
  const uint16_t CCR1_init = 0;
  const uint16_t CCR1_Val = CCR1_init;
  const uint16_t CCR2_Val = CCR1_init;
  const uint16_t CCR3_Val = CCR1_init;
  const uint16_t period = TIM2_PWM_PD;
  const uint16_t prescaler = TIM2_PRESCALER;
  const TIM2_OCMode_TypeDef mode = TIM2_OCMODE_PWM2;
  const TIM2_OCPolarity_TypeDef polarity= TIM2_OCPOLARITY_LOW;

/* TIM2 Peripheral Configuration */
  TIM2_DeInit();

  /* Set TIM2 Frequency to 2Mhz */
  TIM2_TimeBaseInit(prescaler, period);
  /* Channel 1 PWM configuration */
  TIM2_OC1Init(mode, TIM2_OUTPUTSTATE_ENABLE, CCR1_Val, polarity );
  TIM2_OC1PreloadConfig(ENABLE);

  /* Channel 2 PWM configuration */
  TIM2_OC2Init(mode, TIM2_OUTPUTSTATE_ENABLE, CCR2_Val, polarity );
  TIM2_OC2PreloadConfig(ENABLE);

  /* Channel 3 PWM configuration */
  TIM2_OC3Init(mode, TIM2_OUTPUTSTATE_ENABLE, CCR3_Val, polarity );
  TIM2_OC3PreloadConfig(ENABLE);

  /* Enables TIM2 peripheral Preload register on ARR */
  TIM2_ARRPreloadConfig(ENABLE);
/** 
 * Enable the ISR - initiates the back-EMF ADC measurement at the starting edge of
 * the PWM pulse. The timer is required to be left free running as it drives the
 * the periodic UI and Controller tasks (unless the timer is explicitly stopped or
 * reset, there is effect from i.e. starting/stopping the PWM output channels as is
 * done for the motor commutation control).
 */
  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE); // GN: for eventual A/D triggering

  /* Enable TIM2 */
  TIM2_Cmd(ENABLE);
}

/*
 * simple wrappers for PWM management on STM8s
 * Might be improved using tbl of fn pointers to various implementations
 * since there could be different selections for timer periopheral, channel allocation
 * depending upon device caps.
 */
/** @cond */ // hide the low-level code
void PWM_PhA_Disable(void)
{
    TIM2_CCxCmd( TIM2_CHANNEL_1, DISABLE );
}

void PWM_PhB_Disable(void)
{
    TIM2_CCxCmd( TIM2_CHANNEL_2, DISABLE );
}

void PWM_PhC_Disable(void)
{
    TIM2_CCxCmd( TIM2_CHANNEL_3, DISABLE );
}

void PWM_PhA_Enable(void)
{
    TIM2_SetCompare1( global_uDC );
    TIM2_CCxCmd( TIM2_CHANNEL_1, ENABLE );
}

void PWM_PhB_Enable(void)
{
    TIM2_SetCompare2( global_uDC );
    TIM2_CCxCmd( TIM2_CHANNEL_2, ENABLE );
}

void PWM_PhC_Enable(void)
{
    TIM2_SetCompare3( global_uDC );
    TIM2_CCxCmd( TIM2_CHANNEL_3, ENABLE );
}
#endif // S103

/** @endcond */

/**@}*/ // defgroup
