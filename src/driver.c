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

//#include "parameter.h" // app defines

#include "pwm_stm8s.h"
#include "mdata.h" // motor timing curve

/*
 * wanton abuse of globals hall of fame
 */
extern void TIM3_setup(uint16_t u16period); // from main.c


extern uint8_t Log_Level; // global log-level

extern uint16_t Back_EMF_Falling_4[4]; // 4 samples per commutation period


/* Private defines -----------------------------------------------------------*/

// divider: 33k/18k
//  18/(18+33)=0.35
// 0.35 * 14.1v = 4.98
// 4.98 / 2 = 2.48v ........... 1/2 Vdc in proportion to the resister divider
//  2.48v/5v =  x counts / 1024 ocunts so 1/2 Vdc is equivalent to x counts ...
//   x = 1024 * 2.48/5 = 509   (0x01FD)
#define DC_HALF_REF         0x01FD 

//#define V_SHUTDOWN_THR      0x0368 // experimental  ...startup stalls are still possible!
#define V_SHUTDOWN_THR      0x02c0

#define GET_BACK_EMF_ADC( ) \
    ( _ADC_Global - DC_HALF_REF )

#define GET_ADC() _ADC_Global // bah

#define PWM_0PCNT      0

#define PWM_10PCNT     ( PWM_100PCNT / 10 )
#define PWM_20PCNT     ( PWM_100PCNT / 5 )
#define PWM_50PCNT     ( PWM_100PCNT / 2 )

#define PWM_X_PCNT( _PCNT_ )   ( _PCNT_ * PWM_100PCNT / 100 )

/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
#define PWM_DC_RAMPUP  PWM_X_PCNT( 14.0 )

#define PWM_DC_IDLE    PWM_X_PCNT( 12.0 )  // 0x1E ... 30 * 0.4 = 12.0

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


#define BLDC_OL_TM_LO_SPD       0x1000 // 4096d  // start of ramp

#define BLDC_OL_TM_HI_SPD       0x03C0 //  960d

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED         0x006F // 111 

#define BLDC_OL_TM_MANUAL_HI_LIM   LUDICROUS_SPEED

/*
 * Slope of what is basically a linear startup ramp, commutation time (i.e. TIM3)
 * period) decremented by fixed amount each control-loop timestep. Slope 
 * determined by experiment (conservative to avoid stalling the motor!) 
 */
#define BLDC_ONE_RAMP_UNIT      2


/* Private types -----------------------------------------------------------*/

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
 * bitfield mappings for sector (experiment, not presently used):
 *  :2 High drive
 *  :2 Low drive
 *  :2 Rising float transition
 *  :2 Falling float transition
 *  typedef uint_8 SECTOR_BITF_t
 */
typedef uint8_t _SECTOR_PHASE_MAPPING_t ;
// e.g.
// BLDC_PHASE_t bar = PHASE_A;
// SECTOR_PHASE_MAPPING_t foo = (SECTOR_PHASE_MAPPING_t) bar;
#define _SECTOR( _H_ , _L_, _R_, _F_ ) ( _H_ << 6 | _L_ << 4 | _R_ << 2 | _F_ )
// SECTOR_PHASE_MAPPING_t foo = SECTOR( _PHASE_A, _PHASE_B, _PHASE_NONE, _PHASE_C );

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


// motor running-cycle state machine
typedef enum
{
    BLDC_OFF,
    BLDC_RAMPUP,
    BLDC_ON
} BLDC_STATE_T;


/* Public variables  ---------------------------------------------------------*/

uint16_t _ADC_Global;
static uint16_t Back_EMF_15304560[4];

int Back_EMF_Falling_Int_PhX; // take whatever the favored (widest) machine signed int happens to be ...
                              // todo: stms8.h has  typedef   signed long     int32_t; 

uint16_t BLDC_OL_comm_tm;   // could be private


uint16_t Vsystem;
static uint16_t Vbatt;

static BLDC_STATE_T BLDC_State;


/* Private variables ---------------------------------------------------------*/

static int Manual_Mode; // test flag to indicate if manual control override toggled

static uint16_t Ramp_Step_Tm; // reduced x2 each time but can't start any slower

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

/*
 * crude
 */
static void delay(int time)
{
    int d;
    for (d = 0 ; d < time; d++)
    {
        nop();
    }
}

/*
 * back-EMF single-channel ADC start and polls on ADC1_FLAG_EOC end-of-conversion
 */
static uint16_t _sample(ADC1_Channel_TypeDef adc_channel)
{
    uint16_t u16tmp;

    ADC1_ConversionConfig(
        ADC1_CONVERSIONMODE_SINGLE, adc_channel,  ADC1_ALIGN_RIGHT);

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion of all channels in sequence
    ADC1_StartConversion();

// Wait until the conversion is done ... delay in an ISR .. blah

    while ( ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET /* 0 */ );
//    delay(15); // check time on scope .. thia delay can probably avoid the while() ??!?

    u16tmp = ADC1_GetBufferValue(adc_channel); // ADC1_GetConversionValue();

    ADC1_ClearFlag(ADC1_FLAG_EOC);

    return u16tmp;
}

/*
 * back-EMF single-channel start and let ISR to signal EOC
 * Only doing phase A right now if that will suffice.
 *
 * Noted AN2658 "sampling time is not customizable and
 * depends on the ADC clock (3 ADC clocks)"
 */
uint16_t bemf_samp_start( void )
{
    const ADC1_Channel_TypeDef ADC1_ChannelX = ADC1_CHANNEL_3; // ADC1_CHANNEL_0; // only phase A

    uint16_t u16tmp;

    ADC1_ConversionConfig(
        ADC1_CONVERSIONMODE_SINGLE, ADC1_ChannelX, ADC1_ALIGN_RIGHT);

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion
    ADC1_StartConversion();

    return u16tmp;
}

/*
 * back-EMF single-channel get sample - only CH 0
 * Called from ADC1 ISR
 */
void bemf_samp_get(void)
{
    _ADC_Global = ADC1_GetBufferValue( ADC1_CHANNEL_0 ); // ADC1_GetConversionValue();
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   reference:
  *    http://embedded-lab.com/blog/starting-stm8-microcontrollers/21/
  *    - pulse width modulation frequency determined by the value of the TIM1_ARR register
  *    - duty cycle determined by the value of the TIM1_CCRi register
  *
  *    "120° Square-Wave Commutation for Brushless DC Motors" (Toshiba Electronic)
  *
  * PWM is carefully disabled and then to assert the state of the output pins.
  * This can mess with back-EMF component of phase voltage.

  * First: shutoff PWM (before setting any of the new FET states) to ensure PWM
  *  leg is turned off and flyback-diode of non-PWM conducts flyback current("demagnization time".)
  *
  * Second: assert /SD ==OFF  of (only!) the PWMd FET - to ensure that flyback
  *  diode is complete (de-energizing the coil that is now being transitioned
  *  to float). This seems to be the only way to ensure IR2104 set both switch non-conducting.
  *
  * Ideally, when a phase is a 60degress (half of its time) there should be
  * no change/disruption to  its PWM signal.
  *
  * TIM1 counter is not reset or cleared - only the PWM TIM1 channel is changed
  * for the phase, so the overall PWM rate should be maintained. 
  * This routine is getting excessively long (50us) and is quite possble to 
  * overrun the TIM1 time.for PWM pulse? That would add to jitter.
  */
static void comm_switch (uint8_t bldc_step)
{
    static COMMUTATION_SECTOR_t prev_bldc_step = 0;

    BLDC_PWM_STATE_t prev_A, prev_B, prev_C ;
    BLDC_PWM_STATE_t state0, state1, state2;

    // grab the phases states of previous sector 
    prev_A = Commutation_Steps[ prev_bldc_step ].phA;
    prev_B = Commutation_Steps[ prev_bldc_step ].phB;
    prev_C = Commutation_Steps[ prev_bldc_step ].phC;
    prev_bldc_step = bldc_step;

    state0 = Commutation_Steps[ bldc_step ].phA;
    state1 = Commutation_Steps[ bldc_step ].phB;
    state2 = Commutation_Steps[ bldc_step ].phC;

    /*
     * Disable PWM of previous driving phase is finished (120 degrees). Note that
     * an active TIM1 PWM pulse could be interrupted. Probably adds to the overall jitter
     * It's possible that there could be benefit to a delay here to wait for 
     * the PWM pulse (as long as overal time of this function is not excessive...which  it already is!
     */
    if ( DC_OUTP_HI == prev_A  && ( DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0 ) )
    {
        PWM_PhA_Disable();
    }
    if ( DC_OUTP_HI == prev_B  && ( DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1 ) )
    {
        PWM_PhB_Disable();
    }
    if ( DC_OUTP_HI == prev_C  && ( DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2 ) )
    {
        PWM_PhC_Disable();
    }


    if (DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0)
    {
//    PWM_PhA_OUTP_LO( 0 ); ?
        PWM_PhA_HB_DISABLE(0);
    }
    else if (DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1)
    {
//    PWM_PhB_OUTP_LO( 0 ); ?
        PWM_PhB_HB_DISABLE(0);
    }
    else if (DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2)
    {
//    PWM_PhC_OUTP_LO( 0 ); ?
        PWM_PhC_HB_DISABLE(0);
    }


/*
 * The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off (IR2104 enabled)
 */
    if (DC_OUTP_LO == state0)
    {
// let the Timer PWM channel remain disabled, PC2 is LO, /SD.A is ON
//        GPIOC->ODR &=  ~(1<<2);  // PC2 set LO
        PWM_PhA_OUTP_LO( 0 );
//        GPIOC->ODR |=   (1<<5);  // set /SD A
        PWM_PhA_HB_ENABLE(1);
    }
    else if (DC_OUTP_LO == state1)
    {
// let the Timer PWM channel remain disabled, PC3 is LO, /SD.B is ON
//        GPIOC->ODR &=  ~(1<<3);  // PC3 set LO
        PWM_PhB_OUTP_LO( 0 );
//        GPIOC->ODR |=   (1<<7); // set  /SD B
        PWM_PhB_HB_ENABLE(1);
    }
    else if (DC_OUTP_LO == state2)
    {
// let the Timer PWM channel remain disabled, PC4 is LO, /SD.C is ON
//        GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
        PWM_PhC_OUTP_LO( 0 );
//        GPIOG->ODR |=   (1<<1); // set /SD C
        PWM_PhC_HB_ENABLE(1);
    }

/*
 * This delay waits for settling of flyback effect after the PWM transition - only needed for getting 
 * falling Back-EMF signal 
 */
// delay( 10  );

    /*
     * reconfig and re-enable PWM of the driving channels. One driving channel is
     * PWMd, the other is continuously Off. Both driving IR2104s must be enabeld
     * by setting its /SD input line.
     */
    if (DC_OUTP_HI == state0)
    {
        PWM_PhA_Enable();
//        GPIOC->ODR |=   (1<<5);  // set /SD A
        PWM_PhA_HB_ENABLE(1);
    }

    if (DC_OUTP_HI == state1)
    {
        PWM_PhB_Enable();
//        GPIOC->ODR |=   (1<<7); // set  /SD B
        PWM_PhB_HB_ENABLE(1);
    }

    if (DC_OUTP_HI == state2)
    {
        PWM_PhC_Enable();
        GPIOG->ODR |=   (1<<1); // set /SD C
        PWM_PhC_HB_ENABLE(1);
    }
}


extern uart_print( char * sbuf ); // tmp
/*
 *
 */
void BLDC_Stop(void)
{
// kill the driver signals
        PWM_PhA_Disable();
        PWM_PhA_HB_DISABLE(0);

        PWM_PhB_Disable();
        PWM_PhB_HB_DISABLE(0);

        PWM_PhC_Disable();
        PWM_PhC_HB_DISABLE(0);

    if (BLDC_OFF != BLDC_State)
    {
        Log_Level = 0;
        uart_print( "STOP\r\n");
// tmp grab some test data ...
        Back_EMF_Falling_4[0] = Back_EMF_15304560[0];
        Back_EMF_Falling_4[1] = Back_EMF_15304560[1];
        Back_EMF_Falling_4[2] = Back_EMF_15304560[2];
        Back_EMF_Falling_4[3] = Back_EMF_15304560[3];
    }

    BLDC_State = BLDC_OFF;
}

/*
 * increment set and return present motor speed value
 */
uint16_t BLDC_PWMDC_Plus()
{
    if (BLDC_OFF == BLDC_State)
    {
        uart_print( "OFF->RAMP-\r\n");
        BLDC_State = BLDC_RAMPUP;
        return 0;
    }
    else if (BLDC_ON == BLDC_State )
    {
//if (DC < PWM_DC_RAMPUP)
        inc_dutycycle();
    }
    return 0;
}

/*
 * decrement set and return present motor speed value
 */
uint16_t BLDC_PWMDC_Minus()
{
    if (BLDC_OFF == BLDC_State)
    {
        uart_print( "OFF->RAMP-\r\n");
        BLDC_State = BLDC_RAMPUP;
        return 0;
    }
    else if (BLDC_ON == BLDC_State)
    {
// if (DC > PWM_20PCNT)
        dec_dutycycle();
    }
    return 0;
}

/*
 * sets motor speed from commanded throttle/UI setting  (experimental)
 */
void BLDC_PWMDC_Set(uint16_t dc)
{
        if (dc < 0x10)
        {
            BLDC_Stop();
        }

// if presently OFF, then capture the commanded throttle value
    if (BLDC_OFF == BLDC_State)
    {
        // todo: must be off for X to (re)enable startup.

        if ( dc > 10 )
        {
            BLDC_State = BLDC_RAMPUP;
            uart_print( "THR ... OFF->RAMP-\r\n");
        }
    }

    if (BLDC_ON == BLDC_State )
    {
        if (dc > 0x1d)
        {
            if ( get_dutycycle() < dc )
            {
                // TODO:  rate-limit on this ...
                inc_dutycycle();
            }
            else if ( get_dutycycle() > dc )
            {
                dec_dutycycle();
            }
        }
    }
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_dec()
{
    if (BLDC_OFF == BLDC_State)
    {
        BLDC_State = BLDC_RAMPUP;
        uart_print( "OFF->RAMP-\r\n");
    }

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm < 0xFFFF)
    {
        Manual_Mode = 1;
        BLDC_OL_comm_tm += 1; // slower
    }

    Log_Level = 255;// enable continous/verbous log
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_inc()
{
    Log_Level = 1; // default on INC button is just print one line

    if (BLDC_OFF == BLDC_State)
    {
        BLDC_State = BLDC_RAMPUP;
        // BLDC_OL_comm_tm ... init in OFF state to _OL_TM_LO_SPD, don't touch!

        uart_print( "OFF->RAMP+\r\n");
        Log_Level = 0xFF; // log enuff output to span the startup (logger is slow, doesn't take that many)
    }

    if (BLDC_ON == BLDC_State /* && BLDC_OL_comm_tm > BLDC_OL_TM_MANUAL_HI_LIM */ )
    {
        Manual_Mode = 1;
        BLDC_OL_comm_tm -= 1; // faster
    }
}


/*
 * BLDC Update: 
 *  Called from ISR
 *  Handle the BLDC state: 
 *      Off: nothing
 *      Rampup: get BLDC up to sync speed to est. comm. sync.
 *              Once the HI OL speed (frequency) is reached, then the idle speed
 *              must be established, i.e. controlling PWM DC to ? to achieve 2500RPM
 *              To do this closed loop, will need to internally time between the
 *              A/D or comparator input interrupts and adjust DC using e.g. Proportional
 *              control. When idle speed is reached, can transition to user control i.e. ON State
 *      On:  definition of ON state - user control (button inputs) has been enabled
 *              1) ideally, does nothing - BLDC_Step triggered by A/D comparator event
 *              2) less ideal, has to check A/D or comp. result and do the comm.
 *                 step ... but the resolution will be these discrete steps
 *                 (of TIM1 reference)
 */
static uint16_t   Table_value; //whats up w/ stupid compiler optimizing
static int vsys_fault_bucket;

void BLDC_Update(void)
{
    const int FAULT_BUCKET_INI = 128;

// if the voltage threshold is high enuff, the ramp delay time thingy not needed
    const int RAMP_TIME = 1;   // fault arming delay time
    static uint16_t fault_arming_time; // fault_arming_time  

    switch (BLDC_State)
    {
    default:
    case BLDC_OFF:
        // reset commutation timer and ramp-up counters ready for ramp-up
        BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;

        vsys_fault_bucket = FAULT_BUCKET_INI;

        // delay to wait to stabillize at first DC setpoint post-ramp
        fault_arming_time = RAMP_TIME;    /////// // reset the static ramp timer

        set_dutycycle( PWM_0PCNT );
        break;

    case BLDC_ON:
        // do ON stuff
        Vsystem = Vbatt / 2 + Vsystem / 2; // sma
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
                // 0 DC safely stops the motor, user must still press STOP to cycle the program.
                set_dutycycle( PWM_0PCNT );
            }
        }

// grab the "speed" number from the table, determine (sign of) error and incr. +/- 1
/*
basically theres a possiblity that at a high enough speed it could instead take
the error of the +/- back-EMF sensed ZC   and use e * Kp to determine the step 
*/
        if ( 0 == Manual_Mode )
        {
            int error, step = 0;
            Table_value =  Get_OL_Timing( get_dutycycle() );

            error =  Table_value  - BLDC_OL_comm_tm;
            /*
             * the C-T increments between PWM steps are rather large as speeding up 
             * so it may be possible to comp. by reducing rate of this loop by /2
             */
            if (error > 0)
            {
                step = 1;
            }
            else if (error < 0)
            {
                step = -1;
            }

            if (Table_value != 0) // assert
            {
                BLDC_OL_comm_tm += step; // incrementally adjust until error reduces to 0.
            }
        }

        break;

    case BLDC_RAMPUP:

        set_dutycycle( PWM_DC_RAMPUP ); // shouldn't need to keep setting the ramp DC every iteration .. do it once at -transition into ramp state

        if (BLDC_OL_comm_tm > BLDC_OL_TM_HI_SPD) // state-transition trigger?
        {
            BLDC_OL_comm_tm -= BLDC_ONE_RAMP_UNIT;
        }
        else
        {
            BLDC_State = BLDC_ON;

            Vsystem = Vbatt; // "pre-load" the avergae to avoid kicking out at end of ramp1

            Manual_Mode = 0;
            set_dutycycle( PWM_DC_IDLE );

//            Log_Level = 16; // tmp debug
        }
        break;
    }

//  update the timer for the OL commutation switch time
    TIM3_setup(BLDC_OL_comm_tm);
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
void BLDC_Step(void)
{
    const uint8_t N_CSTEPS = 6;

    static uint8_t bldc_step_modul; // internal counter for sub-dividing the TIM3 period

    static COMMUTATION_SECTOR_t comm_step = 0;


    if (BLDC_OFF != BLDC_State )
    {
        // grab the state of previous sector (before advancing the 6-step sequence)
        BLDC_PWM_STATE_t  prev_A = Commutation_Steps[ comm_step ].phA;

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

            if ( DC_OUTP_FLOAT_R == prev_A )
            {
            }
            else  if (DC_OUTP_FLOAT_F == prev_A )
            {
// if phase-A previous sector was floating-falling transition, then the measurements are qualified by copying from the temp array 
                memcpy( Back_EMF_Falling_4, Back_EMF_15304560, sizeof(Back_EMF_Falling_4) );
/*
                Back_EMF_Falling_4[0] = Back_EMF_15304560[0]; // unused
                Back_EMF_Falling_4[1] = Back_EMF_15304560[1]; // pre-ZCP
                Back_EMF_Falling_4[2] = Back_EMF_15304560[2]; // post-ZCP
                Back_EMF_Falling_4[3] = Back_EMF_15304560[3]; // unused
*/
// sum the pre-ZCP and post-ZCP measurements
                Back_EMF_Falling_Int_PhX = 
                       Back_EMF_Falling_4[1] + Back_EMF_Falling_4[2];
            }
            else  if (DC_OUTP_HI == prev_A)
            {
                // if phase was driven pwm, then use the measurement as vbat
                Vbatt = GET_ADC();
            }

            comm_switch( comm_step );

            comm_step = (comm_step + 1) % N_CSTEPS;

            break;
        }

        bldc_step_modul += 1; // can allow rollover as modulus is power of 2
    }
    else
    {
        // motor drive output is not active
        GPIOC->ODR &=  ~(1<<5); //  /SD A
        GPIOC->ODR &=  ~(1<<7); //  /SD B
        GPIOG->ODR &=  ~(1<<1); //  /SD C

        TIM1_CtrlPWMOutputs(DISABLE);
    }
}
