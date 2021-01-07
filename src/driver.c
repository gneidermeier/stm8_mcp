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
#define DC_HALF_REF         0x01FD 

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

uint16_t Back_EMF_Falling_4[4]; // 4 samples per commutation period


/* Private variables ---------------------------------------------------------*/

static uint16_t Vbatt;

static uint16_t ADC_Global;

static uint16_t Back_EMF_15304560[4];


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * back-EMF single-channel start and let ISR to signal EOC
 * Only doing phase A right now if that will suffice.
 *
 * Noted AN2658 "sampling time is not customizable and
 * depends on the ADC clock (3 ADC clocks)"
 */
void bemf_samp_start( void )
{
    const ADC1_Channel_TypeDef ADC1_ChannelX = ADC1_CHANNEL_3; // ADC1_CHANNEL_0; // only phase A

    ADC1_ConversionConfig(
        ADC1_CONVERSIONMODE_SINGLE, ADC1_ChannelX, ADC1_ALIGN_RIGHT);

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion
    ADC1_StartConversion();
}

/*
 * back-EMF single-channel get sample - only CH 0
 * Called from ADC1 ISR
 */
void bemf_samp_get(void)
{
    ADC_Global = ADC1_GetBufferValue( ADC1_CHANNEL_0 ); // ADC1_GetConversionValue();
}


/*
 * public accessor for the system voltage measurement
 * Vbatt is in this module because the measurement has to be timed to the 
 * PWM/commutation sequence
 */
uint16_t Driver_Get_Vbatt(void)
{
    return Vbatt;
}

/*
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
void Driver_Step(void)
{
    static uint8_t bldc_step_modul; // internal counter for sub-dividing the TIM3 period

    uint16_t tmp16;
    int index = bldc_step_modul % TIM3_RATE_MODULUS;

// Note if wanting all 3 phases would need to coordinate here to set the
// correct ADC channel to sample.

    switch(index)
    {
    case 0:
        Back_EMF_15304560[0] = GET_BACK_EMF_ADC( );
        break;
    case 1:
        Back_EMF_15304560[1] = GET_BACK_EMF_ADC( );
        break;
    case 2:
        Back_EMF_15304560[2] = GET_BACK_EMF_ADC( );
        break;
    case 3:
        Back_EMF_15304560[3] = GET_BACK_EMF_ADC( );

        memcpy( Back_EMF_Falling_4, Back_EMF_15304560, sizeof(Back_EMF_Falling_4) );

// using sample at 60-degrees by convention ... the sequencer will have to further qualify 
// the measurement (by determingin that the bldc state was "ON" and not off or floating
        Vbatt  = Driver_Get_ADC();

// motor freewheels when switch to off
        if (BLDC_OFF != get_bldc_state() )
        {
            Sequence_Step();
        }

        break;
    }
    bldc_step_modul += 1; // can allow rollover as modulus is power of 2
}
