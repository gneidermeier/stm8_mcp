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

#define TIM3_RATE_MODULUS   4 // each commutation sector of 60-degrees spans 4x TIM3 periods


/* Private types -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

static uint16_t ADC_Global;

static uint16_t Back_EMF_fbuf[4]; // 4 samples per commutation period


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Capture ADC conversion channel 0
 *
 * @details  Captures phase voltage measurement from ADC Channel 0, to be
 * used as back-EMF sensing or system voltage.
 * Called from ADC1 ISR
 */
void bemf_samp_get(void)
{
    ADC_Global = ADC1_GetBufferValue( ADC1_CHANNEL_0 );
}

/**
 * @brief Get Back-EMF buffer averaged.
 *
 * @details 4 samples are captured within a single commutation sector.
 *
 * @return  Calculated average of 4 samples stored in back-EMF frame buffer
 */
uint16_t Driver_Get_Back_EMF_Avg(void)
{
    uint16_t u16tmp =
        ( Back_EMF_fbuf[0] + \
          Back_EMF_fbuf[1] + \
          Back_EMF_fbuf[2] + \
          Back_EMF_fbuf[3] ) >> 2 ; // divide by 4

    return u16tmp;
}

/**
 * @brief Accessor for system voltage measurement.
 * @details the phase voltage measurement from ADC Channel 0 is to be used as
 * back-EMF sensing or system voltage.
 * @return  Most recent captured ADC conversion value from Channel 0
 */
uint16_t Driver_Get_ADC(void)
{
    return ADC_Global;
}

/**
 * @brief  Update background task and system state.
 *
 * @details  Called from TIM4 ISR. The commutation time is updated each time the
 * control task is updated, and in turn the TIM3 reload register value is refreshed
 * from latest calculated commutation time period.
 */
void Driver_Update(void)
{
    BLDC_Update();

    Periodic_Task_Wake();

//  update the timer for the OL commutation switch time
    TIM3_setup( get_commutation_period() );
}

#define MID_ADC 0x0200 // half vref?

/**
 * @brief
 *
 * @details  Called from TIM3 ISR.
 *
 * Establish error-signal by integrating (averaging) as many back-EMF samples as
 * are buffered during the span of a single commutation frame - ideally a sample
 * would be available at each 15-degree interval. However, at higher rotation
 * speed there are progressively fewer PWM samples available within the time span
 * of a single commutation period.
 * The algorithm initializes the buffer to equivalent of 1/2 DC voltage (ideal
 * zero-cross point) so that un-filled sample slots don't affect the average.
 */
void Driver_Step(void)
{
    // adc at each 15-degree sector is double-buffered to ensure integrity
    // across samples set of 1 motor frame
    static uint16_t adc_15304560[4];

    static uint8_t index = 0;

// Since the modulus being used (4) is a power of 2, then a bitwise & can be used
// instead of a MOD (%) to save a few instructions, which is actually significant
// as this is a very high frequency ISR!
//    index = (index + 1) % TIM3_RATE_MODULUS;
    index = (index + 1) & (TIM3_RATE_MODULUS - 1) /* 0x03 */;

// Distribute the work done in the ISR by partitioning
//  sequence_step, memcpy,  get_ADC into  separate sub-steps
// Logically the call to Sequence_Step() occurs following the memcpy()

    switch(index)
    {
    case 0:
// does the step first (using Back_EMF from memcpy in prev. step)
        Sequence_Step();

        adc_15304560[0] = GET_BACK_EMF_ADC( );
        break;
    case 1:
        adc_15304560[1] = GET_BACK_EMF_ADC( );
        break;
    case 2:
        adc_15304560[2] = GET_BACK_EMF_ADC( );
        break;
    case 3:
        adc_15304560[3] = GET_BACK_EMF_ADC( );
// unrolling this memcpy saves about 20uS !
//        memcpy( Back_EMF_Falling_4, Back_EMF_15304560, sizeof(Back_EMF_Falling_4) );
        Back_EMF_fbuf[0] = adc_15304560[0];
        Back_EMF_fbuf[1] = adc_15304560[1];
        Back_EMF_fbuf[2] = adc_15304560[2];
        Back_EMF_fbuf[3] = adc_15304560[3];

// reset the accumulation buffer ... reset value is 1/2 because there may be
// less than 4 PWM cycles occuring. Instead of trying to sort it out, just let
// any elements that dont get refreshed to be added into the average with no effect.
        adc_15304560[0] = // MID_ADC;
            adc_15304560[1] = // MID_ADC;
                adc_15304560[2] = // MID_ADC;
                    adc_15304560[3] = MID_ADC;

        break;
    }
}
/**@}*/ // defgroup
