/**
  ******************************************************************************
  * @file faultm.c
  * @brief Fault manager for BLDC
  * @author Neidermeier
  * @version
  * @date Jan-2021
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "sequence.h"


/* Private defines -----------------------------------------------------------*/

#define V_SHUTDOWN_THR      0x0368 // experimentally determined!

#define  FAULT_BUCKET_INI  64 // 16


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

uint16_t Vsystem;


/* Private variables ---------------------------------------------------------*/

static int vsys_fault_bucket;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

/*
 * initialize
 */
void Faultm_init(void)
{
    vsys_fault_bucket = FAULT_BUCKET_INI;
}

/*
 */
int Faultm_update(void)
{
// if the voltage threshold is high enuff, the ramp delay time thingy not needed
//    const int RAMP_TIME = 1;   // fault arming delay time
//    static uint16_t fault_arming_time; // fault_arming_time

// update system voltage but this could be done in Seq_Get_Vsystem()?
    Vsystem = Seq_Get_Vbatt() / 2 + Vsystem / 2; // sma

//#if 0
#if 0  // .. Todo: needs to adjust threshold for in-ramp
    if ( fault_arming_time  > 0 )
    {
        fault_arming_time   -= 1;
    }
    else    // assert (Vbatt > VVV )
#endif
    {
        // check system voltage ... is motor stalled?
        if (Vsystem < V_SHUTDOWN_THR)
        {
            // voltage has sagged ... likely motor stall!
            if ( vsys_fault_bucket  > 0)
            {
                vsys_fault_bucket -= 1; //
            }
        }
        else
        {
            if ( vsys_fault_bucket  < FAULT_BUCKET_INI )
            {
                vsys_fault_bucket += 1; // refillin leaky bucket
            }
        }
// finally, check if fault is set
        if (0 == vsys_fault_bucket)
        {
#if 1 // #if ENABLE_VLOW_FAULT
            // 0 DC safely stops the motor, user must still press STOP to re-arm the program.
// kill the driver signals but does not change the state from OFF .. (needs to be error state)
            return 1;//                Commanded_Dutycycle = PWM_0PCNT;
#endif
        }
    }
    return 0;
}
