/**
  ******************************************************************************
  * @file sequence.c
  * @brief support functions for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

//#include "pwm_stm8s.h"
//#include "driver.h" // low level driver definitions
//#include "bldc_sm.h"

#include "sequence.h" // exported types referenced internally


// tmp
extern uint16_t Back_EMF_Falling_4[4]; // 4 samples per commutation period


/* Private defines -----------------------------------------------------------*/




/* Private types -----------------------------------------------------------*/


/* Private types -----------------------------------------------------------*/



/* Public variables  ---------------------------------------------------------*/


/* Private variables  ---------------------------------------------------------*/

static uint16_t Vbatt;


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

/*
 * public accessor for the system voltage measurement
 * Vbatt is in this module because the measurement has to be timed to the 
 * PWM/commutation sequence
 */
uint16_t Seq_Get_Vbatt(void)
{
    return Vbatt;
}

