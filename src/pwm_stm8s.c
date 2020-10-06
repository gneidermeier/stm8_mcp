/**
  ******************************************************************************
  * @file pwm_stm8s.c
  * @brief BLDC motor control - PWM, STM8s specific 
  *   TIM2 is being used despide TIM1 being the capable motor control features,
  *   but PC1 PC2 PC3 pins are taken up by some cap touch feature demo unless 
  *   modify board and remove solder beidges.
  *
  * Comments:
  *   Using the STM8 Peripheral Library, the diffrence between using TIM1 and TIM2
  *   API becomes practically a matter of '/TIM1/TIM2/s' i.e. search and replace. 
  *   One difference, there is no TIM1_BKR register so only 'TIM1CtrlPWMOutpus()'
  *   TIMx may not have the other motor drive capabilities that are not being using anyway.	
  *
  *   For some time, the nature of timing back-EMF to the flyback interval was 
  *   not understood wrt to the various device swithcing steps, so the 
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

/* Includes ------------------------------------------------------------------*/

// stm8s header is provided by the tool chain and is needed for typedefs of uint etc.
#include <stm8s.h>


/* Private defines -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/



/*
 * 3 electrical phases 
 */
typedef enum /* THREE_PHASE_CHANNELS */
{
    PHASE_NONE,
    PHASE_A,
    PHASE_B,
    PHASE_C
} BLDC_PHASE_t;


/* Public variables  ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/


/*
 * simple wrappers for PWM management on STM8s
 * Might be improved using tbl of fn pointers to various implementations
 * since there could be different selections for timer periopheral, channel allocation
 * depending upon device caps.
 */

#if 1
//           TIM2      CH 1   CH 2   CH 3
/**
  * @brief  TIM1 PWM Disable
  * @par Parameters:
  * None
  * @retval void None
  */
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

/**
  * @brief  TIM2 PWM Enable and set duty-cycle
  * @par Parameters:
  * None
  * @retval void None
  */
void PWM_PhA_Enable(uint16_t dc)
{
    TIM2_SetCompare1( dc );               // CH2
    TIM2_CCxCmd( TIM2_CHANNEL_1, ENABLE );
//    TIM2_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}

void PWM_PhB_Enable(uint16_t dc)
{
    TIM2_SetCompare2( dc );               // CH3
    TIM2_CCxCmd( TIM2_CHANNEL_2, ENABLE );
//    TIM2_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}

void PWM_PhC_Enable(uint16_t dc)
{
    TIM2_SetCompare3( dc );              // CH4
    TIM2_CCxCmd( TIM2_CHANNEL_3, ENABLE );
//    TIM2_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}

#else
//           TIM1      CH 2   CH 3   CH 4
/**
  * @brief  TIM1 PWM Disable
  * @par Parameters:
  * None
  * @retval void None
  */
void PWM_PhA_Disable(void)
{
    TIM1_CCxCmd( TIM1_CHANNEL_2, DISABLE );
}

void PWM_PhB_Disable(void)
{
    TIM1_CCxCmd( TIM1_CHANNEL_3, DISABLE );
}

void PWM_PhC_Disable(void)
{
    TIM1_CCxCmd( TIM1_CHANNEL_4, DISABLE );
}

/**
  * @brief  TIM1 PWM Enable and set duty-cycle
  * @par Parameters:
  * None
  * @retval void None
  */
void PWM_PhA_Enable(uint16_t dc)
{
    TIM1_SetCompare2( dc );               // CH2
    TIM1_CCxCmd( TIM1_CHANNEL_2, ENABLE );
    TIM1_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}

void PWM_PhB_Enable(uint16_t dc)
{
    TIM1_SetCompare3( dc );               // CH3
    TIM1_CCxCmd( TIM1_CHANNEL_3, ENABLE );
    TIM1_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}

void PWM_PhC_Enable(uint16_t dc)
{
    TIM1_SetCompare4( dc );              // CH4
    TIM1_CCxCmd( TIM1_CHANNEL_4, ENABLE );
    TIM1_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}

#endif
