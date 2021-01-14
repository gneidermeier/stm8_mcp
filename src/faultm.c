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
#include "faultm.h"  // needed to use fault_t typedef internally 

#include "sequence.h"


/* Private defines -----------------------------------------------------------*/

/*
 * threshold and bucket establish the sensitivity of the fault action.
 * the bucket value is a counter related to the update rate of the function.
 */
#define V_SHUTDOWN_THR      0x0340 // experimentally determined!

#define  FAULT_BUCKET_INI  32 // this macro should be derived from the faultm update rate somehow 


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
FAULT_STATUS_t Faultm_update(void)
{
// if the voltage threshold is high enuff, the ramp delay time thingy not needed
//    const int RAMP_TIME = 1;   // fault arming delay time
//    static uint16_t fault_arming_time; // fault_arming_time

// update system voltage 
    Vsystem = ( Seq_Get_Vbatt() + Vsystem ) / 2; // sma

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
            return FAULT_SET;
        }
    }
    return 0;
}
