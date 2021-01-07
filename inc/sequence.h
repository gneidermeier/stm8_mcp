/**
  ******************************************************************************
  * @file driver.h
  * @brief  
  * @author
  * @version 
  * @date    
  ******************************************************************************
  *
  * BLAH BLAH BLAH
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef SEQUENCE_H
#define SEQUENCE_H

//#ifndef UNIT_TEST // is it broken?
  #include "system.h"
//#endif


/*
 * defines
 */


/* types -----------------------------------------------------------*/

// enumerates the PWM state of each channel
typedef enum DC_PWM_STATE
{
    DC_OUTP_OFF,
    DC_OUTP_HI,
    DC_OUTP_LO,
    DC_OUTP_FLOAT_R,
    DC_OUTP_FLOAT_F,
    DC_NONE
} BLDC_PWM_STATE_t;


/*
 * One commutation step consists of the states of the 3 phases - condensed into 
 * a struct for easy param passing and aggregating into a table.
 */
typedef struct /* COMM_STEP */
{
    BLDC_PWM_STATE_t phA;
    BLDC_PWM_STATE_t phB;
    BLDC_PWM_STATE_t phC;
}
BLDC_COMM_STEP_t;


// commutation "sectors" (steps)
typedef enum /* COMMUTATION_SECTOR */
{
    SECTOR_1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} COMMUTATION_SECTOR_t;
/*
 * variables
 */


/*
 * prototypes
 */

BLDC_COMM_STEP_t Seq_Get_Step( COMMUTATION_SECTOR_t index );

void Sequence_Step(void);


#endif // SEQUENCE_H
