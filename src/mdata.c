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

#define TABLE_SIZE    TIM2_PWM_PD

/*
 * The table is indexed by PWM duty cycle counts (i.e. [0:1:250)
 * The function generates the data in Scilab and imported from csv:
 *
 *   y =  ( 4000 * EXP( -t/25 ) ) + 150
 *
 * Excel was used  to find initial parameters using some data obtained by
 * experimentaiton with running the motor and manually syncing it using the
 * butttons and terminal inputs for ocmmutiation timing and PWM DC.
 *
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
static const uint16_t OL_Timing[ TABLE_SIZE ] =
{
#if 0
// leave the old data
    0x0000,	 // 00 0
    0x0000,	 // 01 1
    0x0000,	 // 02 2
    0x0000,	 // 03 3
    0x0000,	 // 04 4
    0x0000,	 // 05 5
    0x0000,	 // 06 6
    0x0000,	 // 07 7
    0x0000,	 // 08 8
    0x0CC0,	 // 09 9   /////////////////////// ooozez to a halt
    0x0BC0,	 // 0A 10
    0x09E0,	 // 0B 11
    0x0900,	 // 0C 12
    0x0880,	 // 0D 13
    0x0800,	 // 0E 14
    0x0780,	 // 0F 15
    0x0700,	 // 10 16
    0x0670,	 // 11 17
    0x0640,	 // 12 18  // 660      680 632
    0x05E1,	 // 13 19  // 600                /// 5e1
    0x0560,	 // 14 20
    0x04F0,	 // 15 21
    0x04C0,	 // 16 22
    0x0490,	 // 17 23
    0x0460,	 // 18 24
    0x0430,	 // 19 25
    0x03F0,	 // 1A 26
    0x03E0,	 // 1B 27
    0x03D0,	 // 1C 28
    0x03C0,	 // 1D 29  // end of ramp           /// 3C0
    0x03AC,	 // 1E 30  /                        /// 3AC
    0x038B,	 // 1F 31  // 37E 390               /// 38b
    0x036B,	 // 20 32  // 36b 378               /// 36e
    0x0350,	 // 21 33  // 34E 360               /// 34e
    0x0333,	 // 22 34  // 330 343         .     /// 330
    0x031C,	 // 23 35  // 319 32F               /// 319
    0x0302,	 // 24 36  // 302 31F               /// 302
    0x02F8,	 // 25 37  // 2f1 308               /// 2f1
    0x02E0,	 // 26 38  // 2d8 2f1               /// 2e0
    0x02C9,	 // 27 39  // 2c9 2E0               /// 2c9
    0x02B7,	 // 28 40  // 2b7 2C6               /// 2b7
    0x02b0,	 // 29 41  // 2b0 2B8               /// 2b0
    0x029B,	 // 2A 42  // 296 2B0               /// 296
    0x0288,	 // 2B 43  // 288 2A0               /// 288
    0x0279,	 // 2C 44  // 279 288               /// 279
    0x0271,	 // 2D 45  // 271 280               /// 271
    0x0260,	 // 2E 46  // 25E 271  .            /// 25E
    0x0253,	 // 2F 47  // 253 25b               /// 253
    0x0245,	 // 30 48  // 245 252               /// 245
    0x023A,	 // 31 49  // 23a 23e               /// 23A
    0x022F,	 // 32 50  // 22f 234               /// 22F
    0x0225,	 // 33 51  // 225 227               /// 225
    0x021A,	 // 34 52  // 21a 220               /// 21a
    0x0210,	 // 35 53  // 210 213               /// 210  x
    0x0205,	 // 36 54  // 205 211               /// 205
    0x01FC,	 // 37 55  // 1fc 202               /// 1FC
    0x01F6,	 // 38 56  // 1f6 1F8               /// 1f6
    0x01EA,	 // 39 57  // 1ea 1F2               /// 1ea  x
    0x01E2,	 // 3A 58  // 1e2 1e8               /// 1e2
    0x01DA,	 // 3B 59  // 1da 1E0               /// 1da
    0x01D7,	 // 3C 60  // 1d2 1d7
    0x01D2,	 // 3D 61  //
    0x01C4,	 // 3E 62  // 1C4 1cb               /// 1c4
    0x01BF,	 // 3F 63  //                       /// 1bf
    0x01BA,	 // 40 64  // 1b7 1b8               /// 1b7
    0x01B5,	 // 41 65  // 1b5 1b7               /// 1b5
    0x01A8,	 // 42 66  // 1a8 1Ac               /// 1A8
    0x01A2,	 // 43 67  // 1a2 1A8               /// 1A2  x
    0x019C,	 // 44 68  // 18c 1A2               /// 19C
    0x0197,	 // 45 69  // 197 19C               /// 197
    0x0191,	 // 46 70  // 191 197               /// 191
    0x018C,	 // 47 71  // 18c 196               /// 18C
    0x0187,	 // 48 72  // 187 18c               /// 187
    0x0181,	 // 49 73  // 181 189               /// 181
    0x017C,	 // 4A 74  // 17c 1a2
    0x0179,	 // 4B 75  // 179           ???????????????
    0x0178,	 // 4C 76                    NO /////////////////////
    0x0177,	 // 4D 77
    0x0176,	 // 4E 78
    0x0175,	 // 4F 79
    0x0174,	 // 50 80
    0x0173,	 // 50 80
    0x0172,	 // 50 80
    0x0171,	 // 50 80
    0x0170,	 // 50 80
//
// truncated ... needs more values more precision needed of commutation time period!

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
