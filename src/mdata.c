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

#include "pwm_stm8s.h"
#include "mdata.h"


// table size originated from 250 step PWM confiugration
#define MDATA_TBL_SIZE            250

/**
 * @brief rescale index parameter to scale of original table
 * @details
 *   PWM period counts must be a multiple of original table size
 *
 *   rescaled_index = index * PWM_PERIOD_COUNTS / MDATA_TBL_SIZE

 */
#define PWM_PERIOD_SCALAR    (PWM_PERIOD_COUNTS / MDATA_TBL_SIZE)
 
#define MDATA_TBL_INDEX_PCNT_SCALE( _index_ ) \
                                     ( _index_ / PWM_PERIOD_SCALAR )

/*
 * The table is indexed by PWM duty cycle counts (i.e. [0:1:250)
 * The function generates the data in Scilab and imported from csv.
 * Gain and time constant experimentally determined for timing 1100kV motor @ 12.4v
 *
 *   Y =  A * exp( -t / TAU )
 *
 *   A     = 3400
 *   TAU   = 50
 *
 * Note that the first 16 or so table entries aren't really used as the motor 
 * can't run at such low duty-cycle!
 */
static const uint16_t OL_Timing[ /* TABLE_SIZE */ ] =
{
#if defined (S003_DEV)
1500, //   0, 0, 0, 5DC
1470, //   1, 1, 1, 5BE
1441, //   2, 1, 2, 5A1
1413, //   3, 2, 3, 585
1385, //   4, 2, 4, 569
1357, //   5, 3, 5, 54D
1330, //   6, 3, 6, 532
1304, //   7, 4, 7, 518
1278, //   8, 4, 8, 4FE
1253, //   9, 5, 9, 4E5
1228, //   10, 6, A, 4CC
1204, //   11, 6, B, 4B4
1180, //   12, 7, C, 49C
1157, //   13, 7, D, 485
1134, //   14, 8, E, 46E
1111, //   15, 8, F, 457
1089, //   16, 9, 10, 441
1068, //   17, 9, 11, 42C
1047, //   18, 10, 12, 417
1026, //   19, 11, 13, 402
1005, //   20, 11, 14, 3ED
986, //   21, 12, 15, 3DA
966, //   22, 12, 16, 3C6
947, //   23, 13, 17, 3B3
928, //   24, 13, 18, 3A0
910, //   25, 14, 19, 38E
892, //   26, 14, 1A, 37C
874, //   27, 15, 1B, 36A
857, //   28, 16, 1C, 359
840, //   29, 16, 1D, 348
823, //   30, 17, 1E, 337
807, //   31, 17, 1F, 327
791, //   32, 18, 20, 317
775, //   33, 18, 21, 307
760, //   34, 19, 22, 2F8
745, //   35, 19, 23, 2E9
730, //   36, 20, 24, 2DA
716, //   37, 21, 25, 2CC
701, //   38, 21, 26, 2BD
688, //   39, 22, 27, 2B0
674, //   40, 22, 28, 2A2
661, //   41, 23, 29, 295
648, //   42, 23, 2A, 288
635, //   43, 24, 2B, 27B
622, //   44, 24, 2C, 26E
610, //   45, 25, 2D, 262
598, //   46, 26, 2E, 256
586, //   47, 26, 2F, 24A
574, //   48, 27, 30, 23E
563, //   49, 27, 31, 233
552, //   50, 28, 32, 228
541, //   51, 28, 33, 21D
530, //   52, 29, 34, 212
520, //   53, 29, 35, 208
509, //   54, 30, 36, 1FD
499, //   55, 31, 37, 1F3
489, //   56, 31, 38, 1E9
480, //   57, 32, 39, 1E0
470, //   58, 32, 3A, 1D6
461, //   59, 33, 3B, 1CD
452, //   60, 33, 3C, 1C4
443, //   61, 34, 3D, 1BB
434, //   62, 34, 3E, 1B2
425, //   63, 35, 3F, 1A9
#else
  #include "model.h" // hack   headers are horrible
#endif	
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
uint16_t Get_OL_Timing(uint16_t table_index)
{
    uint16_t t16 = (U16_MAX); // error

    // rescxale index range to that of  original table size
    uint16_t index = MDATA_TBL_INDEX_PCNT_SCALE(table_index);

    // assert index < OL_TIMING_TBL_SIZE
    if ( index < OL_TIMING_TBL_SIZE )
    {
        t16 = OL_Timing[ index ] * CTIME_SCALAR;
    }
    return t16;
}

/**@}*/ // defgroup
