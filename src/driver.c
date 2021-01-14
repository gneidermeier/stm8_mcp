/**
  ******************************************************************************
  * @file driver.c
  * @brief support functions for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2020
  ******************************************************************************
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

/*
 * back-EMF single-channel get sample - only CH 0
 * Called from ADC1 ISR
 */
void bemf_samp_get(void)
{
    ADC_Global = ADC1_GetBufferValue( ADC1_CHANNEL_0 ); // ADC1_GetConversionValue();
}

/*
 * average the 4 back-EMF samples from the back-EMF frame buffer
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

/*
 * public accessor for the system voltage measurement
 * need to define an interface to the ADC sample/conversion service
 */
uint16_t Driver_Get_ADC(void)
{
    return ADC_Global;
}

/*
 * BLDC Update: 
 *  Called from ISR
 */
void Driver_Update(void)
{
    BLDC_Update();

    Periodic_Task_Wake();

//  update the timer for the OL commutation switch time
    TIM3_setup( get_commutation_period() );
}

/*
 * called from ISR
 *
// establish error-signal ...
// measures at 4 15-degree intervals - [1] and [2] are the valid ones to "integrate"
// Note at this low idle/open-loop speed there are only about 4 PWM @ 8k during
// a 60-degree sector.
// Pulses close to start/end of the secctor are problematic anyway as the ADC 
// ISR ends up getting blocked by the TIM3 ISR ... the BLDC_Step() takes about
// to 40us on case 3!
 */
#define MID_ADC 0x0200 // half vref?
 
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
