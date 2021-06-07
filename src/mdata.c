/**
  ******************************************************************************
  * @file mdata.c
  * @brief  Motor data table lookup
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
/**
 * \defgroup mdata Motor Model Data
  * @brief  Motor data table lookup
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "system.h" // dependency of motor data on cpu clock specific timer rate

/*
 * The table is indexed by PWM duty cycle counts (i.e. [0:1:250)
 * The function generates the data in Scilab and imported from csv:
 *
 *   y =  ( 4000 * EXP( -t/25 ) ) + 150
 *
 * Excel was used  to find initial parameters using some data obtained experimentally
 * sync range seems to be [32-68] (duty-cycle) with parameters roughly in these ranges:
 *  A     = [ 1800 : 2000 ]
 *  TAU   = [ 38 : 34 ]
 *  Y_INT = [ 25 : 30 ]
 *  X_INT = 0
 *
 *  (32-80)/(788-268) =  -0.0923077
 *
 * The range that needs to be synced is between "Idle" (DC=32) and some higher speed where b-emf is measureable.
 * The exponential seems to do a better job at tracking over the needed
 * interval. Taking the coordinates at the start and end of the range over which
 * the motor is able to stay in sync would give the slope that could be tried for
 * a linear tracking function.
 *
 */
static const uint16_t OL_Timing[ /* TABLE_SIZE */ ] =
{
  #include "model.h" // hack   headers are horrible
};

#define OL_TIMING_TBL_SIZE    ( sizeof(OL_Timing) / sizeof(uint16_t) )


/**
 * @brief Table lookup for open-loop commutation timing
 *
 * @param index  Index into the table - motor speed i.e. PWM duty-cycle
 *
 * @return LUT value @ index
 * @retval -1 error
 */
uint16_t Get_OL_Timing(uint16_t index)
{
    // assert index < OL_TIMING_TBL_SIZE
    if ( index < OL_TIMING_TBL_SIZE )
    {
        return OL_Timing[ index ] * CTIME_SCALAR;
    }
    return (U16_MAX); // error
}

/**@}*/ // defgroup
