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
#include "stm8s.h"
#include "parameter.h" // app defines
#include "pwm_stm8s.h"

extern uint8_t Log_Level; // tmp

/* Private defines -----------------------------------------------------------*/

#define PWM_100PCNT    TIM2_PWM_PD
#define PWM_0PCNT      0

#define PWM_10PCNT     ( PWM_100PCNT / 10 )
#define PWM_20PCNT     ( PWM_100PCNT / 5 )
#define PWM_25PCNT     ( PWM_100PCNT / 4 )
#define PWM_50PCNT     ( PWM_100PCNT / 2 )


#define PWM_X_PCNT( _PCNT_ )   ( _PCNT_ * PWM_100PCNT / 100 )

#define PWM_DC_RAMPUP  PWM_X_PCNT( 45 )

#define PWM_DC_IDLE    PWM_X_PCNT( 22 )

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


#define BLDC_OL_TM_LO_SPD          (512 * 2 * TIM3_RATE_MODULUS) // start of ramp

//  1 / (3ms * (12/2) ) = motor freq.
#define BLDC_OL_TM_HI_SPD          (64 * 2 * TIM3_RATE_MODULUS)

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED            (13.875f * 2 * TIM3_RATE_MODULUS) // note; won't be possible until D.C. and comm-time are sync'd i.e. closed-loop

#define BLDC_OL_TM_MANUAL_HI_LIM   LUDICROUS_SPEED

/*
 * Slope of what is basically a linear startup ramp, commutation time (i.e. TIM3)
 * period) decremented by fixed amount each control-loop timestep. Slope 
 * determined by experiment (conservative to avoid stalling the motor!) 
 */ 
#define BLDC_ONE_COMM_STEP         TIM3_RATE_MODULUS  // each commutation step unit is 4x TIM3 periods
#define BLDC_ONE_RAMP_UNIT         BLDC_ONE_COMM_STEP

/* Private types -----------------------------------------------------------*/

// enumerates the PWM state of each channel
typedef enum DC_PWM_STATE
{
    DC_OUTP_OFF,
//    DC_PWM_PLUS,
//    DC_PWM_MINUS, // complimented i.e. (100% - DC)
    DC_OUTP_HI,
    DC_OUTP_LO,
    DC_OUTP_FLOAT_R,
    DC_OUTP_FLOAT_F,
    DC_NONE
} BLDC_PWM_STATE_t;


/*
 * bitfield mappings for sector:
 *  :2 High drive
 *  :2 Low drive
 *  :2 Rising float transition
 *  :2 Falling float transition
 *  typedef uint_8 SECTOR_BITF_t
 */
typedef uint8_t SECTOR_PHASE_MAPPING_t ;
// e.g.
// BLDC_PHASE_t bar = PHASE_A;
// SECTOR_PHASE_MAPPING_t foo = (SECTOR_PHASE_MAPPING_t) bar;
#define SECTOR( _H_ , _L_, _R_, _F_ ) ( _H_ << 6 | _L_ << 4 | _R_ << 2 | _F_ )
// SECTOR_PHASE_MAPPING_t foo = SECTOR( _PHASE_A, _PHASE_B, _PHASE_NONE, _PHASE_C );

/*
 * aggregate 3 phases into a struct for easy param passing and putting in table
 * There could be other information put in there if needed to ease the step transitions logic.
 */
typedef struct /* COMM_STEP */
{
    BLDC_PWM_STATE_t phA;
    BLDC_PWM_STATE_t phB;
    BLDC_PWM_STATE_t phC;
}
BLDC_COMM_STEP_t;



// PWM drive modes
typedef enum /* PWM_MODE */
{
    UPPER_ARM,
    LOWER_ARM
    // SYMETRICAL ... upper and lower arms driven (complementary) .. maybe no use for it
} PWM_MODE_t;

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




/* Public variables  ---------------------------------------------------------*/




uint16_t Back_EMF_F_tmp;
uint16_t Back_EMF_F0_MA_tmp;

uint16_t Back_EMF_R_tmp;
uint16_t Back_EMF_R0_MA_tmp;


uint16_t BLDC_OL_comm_tm;   // could be private

uint16_t global_uDC;

BLDC_STATE_T BLDC_State;


/* Private variables ---------------------------------------------------------*/

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


static uint16_t Ramp_Step_Tm; // reduced x2 each time but can't start any slower


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
static void set_dutycycle(uint16_t global_dutycycle)
{
    global_uDC = global_dutycycle;
}

/*
 * these functions don't do real range checking, they just assert against 
 * integer rollover (which shouldn't happen anyway?)
 * see BLDC_Spd_dec() etc.
 */
static void inc_dutycycle(void)
{
    if (global_uDC < 0xFFFE)
    {
        global_uDC += 1;
    }
}

static void dec_dutycycle(void)
{
    if (global_uDC > 0)
    {
        global_uDC -= 1;
    }
}

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
 * for now the ADC channels for back-EMF are single-channel conversions
 */
static uint16_t sample(ADC1_Channel_TypeDef adc_channel)
{
    uint16_t u16tmp;

    ADC1_ConversionConfig(
        ADC1_CONVERSIONMODE_SINGLE, adc_channel,  ADC1_ALIGN_RIGHT);

// Enable the ADC: 1 -> ADON for the first time it just wakes the ADC up
    ADC1_Cmd(ENABLE);

// ADON = 1 for the 2nd time => starts the ADC conversion of all channels in sequence
    ADC1_StartConversion();

// Wait until the conversion is done ... delay in an ISR .. blah

// GPIOG->ODR |=  (1<<0); // set test pin

        while ( ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET /* 0 */ );
//    delay(15); // check time on scope .. thia delay can probably avoid the while() ??!?
// GPIOG->ODR &=  ~(1<<0); // clear test pin

    u16tmp = ADC1_GetBufferValue(adc_channel); // ADC1_GetConversionValue();

    ADC1_ClearFlag(ADC1_FLAG_EOC);

    return u16tmp;
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

    uint16_t u16tmp;

    GPIOG->ODR &=  ~(1<<0); // clear test pin


    // grab the phases states of previous sector 
    prev_A = Commutation_Steps[ prev_bldc_step ].phA;
    prev_B = Commutation_Steps[ prev_bldc_step ].phB;
    prev_C = Commutation_Steps[ prev_bldc_step ].phC;
    prev_bldc_step = bldc_step;


    state0 = Commutation_Steps[ bldc_step ].phA;
    state1 = Commutation_Steps[ bldc_step ].phB;
    state2 = Commutation_Steps[ bldc_step ].phC;


// before messing w/ PWM, firgure out if the previous float phase was A and if
// so (only ph. A for now) then collect the back-EMF sample(s) . 
// [1] and [3]  should add to 0 (15 degree and 45 degree)
// [2] should be at 0-crossing (30 degree)


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
//    PWM_PhA_OUTP_LO( 0 );
        GPIOC->ODR &=  ~(1<<5);      // /SD A OFF
    }
    else if (DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1)
    {
//    PWM_PhB_OUTP_LO( 0 );
        GPIOC->ODR &=   ~(1<<7);     // /SD B OFF
    }
    else if (DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2)
    {
//    PWM_PhC_OUTP_LO( 0 );
        GPIOG->ODR &=   ~(1<<1);     // /SD C OFF
    }


/*
 * The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off (IR2104 enabled)
 */
    if (DC_OUTP_LO == state0)
    {
// let the Timer PWM channel remain disabled, PC2 is LO, /SD.A is ON
//        GPIOC->ODR &=  ~(1<<2);  // PC2 set LO
        PWM_PhA_OUTP_LO( 0 );
        GPIOC->ODR |=   (1<<5);  // set /SD A

    }
    else if (DC_OUTP_LO == state1)
    {
// let the Timer PWM channel remain disabled, PC3 is LO, /SD.B is ON
//        GPIOC->ODR &=  ~(1<<3);  // PC3 set LO
        PWM_PhB_OUTP_LO( 0 );
        GPIOC->ODR |=   (1<<7); // set  /SD B
    }
    else if (DC_OUTP_LO == state2)
    {
// let the Timer PWM channel remain disabled, PC4 is LO, /SD.C is ON
//        GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
        PWM_PhC_OUTP_LO( 0 );
        GPIOG->ODR |=   (1<<1); // set /SD C
    }


/*
 * This delay waits for settling of flyback effect after the PWM transition - only needed for getting 
 * falling Back-EMF signal 
 */
#ifdef COMM_TIME_KLUDGE_DELAYS

 GPIOG->ODR |=  (1<<0); // set test pin

#ifdef PWM_8K // 8k PWM
// this is "only" about 25us here, and seems flyback from PWM-on takes almost 20us anyway!!!
 //   delay( 20  );   //                  CT=0201 bF0_ma=00E9
 delay( 10  ); // yes I seem to keep messing with this
#else
 asdf
#endif

 GPIOG->ODR &=  ~(1<<0); // clear test pin 
#endif // COMM_TIME_KLUDGE_DELAYS


/*
 * Back-EMF reading hardcoded to phase "A" (ADC_0)
 */
    if (  DC_OUTP_FLOAT_F == state0 )
    {
GPIOG->ODR |=  (1<<0); // set test pin
        u16tmp = sample( ADC1_CHANNEL_0 );
GPIOG->ODR &=  ~(1<<0); // clear test pin ... should have 14 us ???

        Back_EMF_F_tmp = u16tmp;

        Back_EMF_F0_MA_tmp  = ( u16tmp + Back_EMF_F0_MA_tmp ) / 2;
    }
    else // should not have 2 adjacent float sectors on the same phase
    if (  DC_OUTP_FLOAT_R == prev_A ) // 
    {
GPIOG->ODR |=  (1<<0); // set test pin
        u16tmp = sample( ADC1_CHANNEL_0 );
GPIOG->ODR &=  ~(1<<0); // clear test pin ... should have 14 us ???

        Back_EMF_R_tmp = u16tmp;

        Back_EMF_R0_MA_tmp  = ( u16tmp + Back_EMF_R0_MA_tmp ) / 2;
    }


    /*
     * reconfig and re-enable PWM of the driving channels. One driving channel is
     * PWMd, the other is continuously Off. Both driving IR2104s must be enabeld
     * by setting its /SD input line.
     */
    if (DC_OUTP_HI == state0)
    {
        PWM_PhA_Enable( global_uDC );
        GPIOC->ODR |=   (1<<5);  // set /SD A
    }

    if (DC_OUTP_HI == state1)
    {
        PWM_PhB_Enable( global_uDC );
        GPIOC->ODR |=   (1<<7); // set  /SD B
    }

    if (DC_OUTP_HI == state2)
    {
        PWM_PhC_Enable( global_uDC );
        GPIOG->ODR |=   (1<<1); // set /SD C
    }
}


extern uart_print( char * sbuf ); // tmp
/*
 *
 */
void BLDC_Stop()
{
    if (BLDC_OFF != BLDC_State)
    {
// should probably assert what the /SD pins are doing as well (depends wether motor
// should be braked or left windmilling??
        PWM_PhA_Disable();
        PWM_PhB_Disable();
        PWM_PhC_Disable();

        Log_Level = 0;
        uart_print( "STOP\r\n");
    }

    BLDC_State = BLDC_OFF;
    set_dutycycle( PWM_0PCNT );
}

/*
 * increment set and return present motor speed value
 */
uint16_t BLDC_PWMDC_Plus()
{
    if (BLDC_OFF == BLDC_State)
    {
//        BLDC_State = BLDC_RAMPUP;
//        uart_print( "OFF->RAMP-\r\n");
    }
    else if (BLDC_ON == BLDC_State )
    {
//if (DC < PWM_DC_RAMPUP)
        inc_dutycycle();
    }
    return global_uDC;
}

/*
 * decrement set and return present motor speed value
 */
uint16_t BLDC_PWMDC_Minus()
{
    if (BLDC_ON == BLDC_State)
    {
// if (DC > PWM_20PCNT)
        dec_dutycycle();
    }
    return global_uDC;
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

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm > BLDC_OL_TM_MANUAL_HI_LIM )
    {
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
void BLDC_Update(void)
{
    switch (BLDC_State)
    {
    default:
    case BLDC_OFF:
        // reset commutation timer and ramp-up counters ready for ramp-up
        BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;
        break;

    case BLDC_ON:
        // do ON stuff

        break;

    case BLDC_RAMPUP:

        set_dutycycle( PWM_DC_RAMPUP ) ;

        if (BLDC_OL_comm_tm > BLDC_OL_TM_HI_SPD) // state-transition trigger?
        {
            BLDC_OL_comm_tm -= BLDC_ONE_RAMP_UNIT;
        }
        else
        {
            BLDC_State = BLDC_ON;
            set_dutycycle( PWM_DC_IDLE );

            Log_Level = 16; // tmp debug
        }
        break;
    }

//  update the timer for the OL commutation switch time
    TIM3_setup(BLDC_OL_comm_tm);
}

/*
 */
static void bldc_comm_step(void)
{
    const uint8_t N_CSTEPS = 6;

    static COMMUTATION_SECTOR_t bldc_step = 0;

    if (BLDC_OFF == BLDC_State )
    {
        // motor drive output is not active
        GPIOC->ODR &=  ~(1<<5); //  /SD A
        GPIOC->ODR &=  ~(1<<7); //  /SD B
        GPIOG->ODR &=  ~(1<<1); //  /SD C

        TIM1_CtrlPWMOutputs(DISABLE);
    }
    else // if (BLDC_ON == BLDC_State || BLDC_RAMP == BLDC_State)
    {
        bldc_step += 1;
        bldc_step %= N_CSTEPS;

        comm_switch( bldc_step );
    }
}

/*
 * called from ISR
 */
void BLDC_Step(void)
{
    static uint8_t bldc_step_modul; // internal counter for sub-tasking the TIM3 period

    if (BLDC_OFF != BLDC_State )
    {
        int index = bldc_step_modul % TIM3_RATE_MODULUS;

        switch(index)
        {
        case 0:
            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            // commutation step obviously done only once on the base-period
            bldc_comm_step();
            break;
        }

        bldc_step_modul += 1; // can be allowed to rollover as modulus is pwrOf2
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
