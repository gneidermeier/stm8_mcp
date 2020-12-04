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

//#include <string.h>

//#include "parameter.h" // app defines

//#include "pwm_stm8s.h"
//#include "driver.h"
//#include "bldc_sm.h"



#include "sequence.h" // exported types referenced internally


/* Private defines -----------------------------------------------------------*/




/* Private types -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/



/* Public variables  ---------------------------------------------------------*/


/* Private variables  ---------------------------------------------------------*/

/*
 * This table simply defines the "trapezoidal" waveform in 6-steps.
 * The underlying PWM management scheme would be introduced elsewheres.
 */
static const BLDC_COMM_STEP_t Commutation_Steps[] =
{
// sector 0:
    { DC_OUTP_HI,      DC_OUTP_LO,      DC_OUTP_FLOAT_F },
// sector 1:
    { DC_OUTP_HI,      DC_OUTP_FLOAT_R, DC_OUTP_LO },
// sector 2:
    { DC_OUTP_FLOAT_F,  DC_OUTP_HI,     DC_OUTP_LO },
// sector 3:
    { DC_OUTP_LO,       DC_OUTP_HI,     DC_OUTP_FLOAT_R },
// sector 4:
    { DC_OUTP_LO,       DC_OUTP_FLOAT_F, DC_OUTP_HI },
// sector 5:
    { DC_OUTP_FLOAT_R,  DC_OUTP_LO,      DC_OUTP_HI }
};



/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

/*
 * public accessor for the fixed table
 */
BLDC_COMM_STEP_t Seq_Get_Step(int index)
{
    BLDC_COMM_STEP_t step =  Commutation_Steps[ index ];
    return step;
}

