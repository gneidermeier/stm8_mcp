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

extern uint8_t Log_Level; // tmp

/* Private defines -----------------------------------------------------------*/

#define PWM_100PCNT    TIM2_PWM_PD
#define PWM_50PCNT     ( PWM_100PCNT / 2 )
#define PWM_25PCNT     ( PWM_100PCNT / 4 )
#define PWM_20PCNT     ( PWM_100PCNT / 5 )
#define PWM_10PCNT     ( PWM_100PCNT / 10 )
#define PWM_0PCNT      0

#define PWM_X_PCNT( _PCNT_ )   ( _PCNT_ * PWM_100PCNT / 100 )

#define PWM_DC_RAMPUP  PWM_X_PCNT( 45 ) // from 50% ... better? (stalls 4/5)

#define PWM_DC_IDLE    PWM_X_PCNT( 22 ) // stall 50% of the time

/*
 * These constants are the number of timer counts (TIM3) to achieve a given
 *  commutation step period.
 * See TIM3 setup - base period is 0.000008 Seconds (8 uSecs)
 * For the theoretical 15000 RPM motor:
 *   15000 / 60 = 250 rps
 *   cycles per sec = 250 * 6 = 1500
 *   1 cycle = 1/1500 = .000667 S
 *
 * RPS = (1 / cycle_time * 6)
 */

//   BLDC_CT_SCALE  ... needs to be linked to TIM3 Presacalar! (changing this is not working right now ... )
// BLDC_CT_SCALE must be > 2 (and must be power of 2)
//#define RAMP_SCALAR   ( BLDC_CT_SCALE / 2 )   // ... 2/5 stalls   :(
//#define RAMP_SCALAR   ( 2 ) // somehow this relates to the BLDC Commutation Timer scaling?
#define RAMP_SCALAR   ( 1 ) // not sure how much this matters .. stalls 40-50% of the time ?? L:J:???

// the OL comm time is shortened by 1 rammp-unit (e.g. 2 counts @ 0.000008S per count where 8uS is the TIM3 bit-time)
// the ramp shortens the OL comm-time by 1 RU each ramp-step with each ramp step is the TIM1 period of ~1mS
#define BLDC_ONE_RAMP_UNIT          (1 * BLDC_CT_SCALE)

// 1 cycle = 6 * 8uS * 512 = 0.024576 S
#define BLDC_OL_TM_LO_SPD         (512 * BLDC_CT_SCALE)  // start of ramp

#define BLDC_OL_TM_HI_SPD          (69 * BLDC_CT_SCALE)  // CT=$0230   // 560/8=70

// 1 cycle = 6 * 8uS * 13 = 0.000624 S    (needs updated info here)
#define LUDICROUS_SPEED            (13 * BLDC_CT_SCALE)  // 15kRPM would be ~13.8 counts

#define BLDC_OL_TM_MANUAL_HI_LIM   LUDICROUS_SPEED


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

// tmp?
#define BLDC_PWM_CH1   TIM1_CHANNEL_1//                     = ((uint8_t)0x00),
#define BLDC_PWM_CH2   TIM1_CHANNEL_2//                     = ((uint8_t)0x01),
#define BLDC_PWM_CH3   TIM1_CHANNEL_3//                     = ((uint8_t)0x02),
#define BLDC_PWM_CH4   TIM1_CHANNEL_4//                     = ((uint8_t)0x03)
#define BLDC_PWM_CH_Z  (-1)  // help for my kludgey lookup table indexing

typedef TIM1_Channel_TypeDef BLDC_Channel_TypeDef ;


/* Public variables  ---------------------------------------------------------*/

/*
 * can be indexed by BLDC_PHASE_t
 */
static const BLDC_Channel_TypeDef BLDC_PWM_Chann_Cfg[ ] =
{

// note PWM 0 at index 0
    -1, // make sure this can be element 0 is invalid (allows index initializers to default to 0 i.e. index OOR)
        BLDC_PWM_CH2, // PWM 1
        BLDC_PWM_CH3, // PWM 2
        BLDC_PWM_CH4, // PWM 3
        BLDC_PWM_CH_Z  //  meh
    };
#define BLDC_CH_NG  (sizeof( BLDC_PWM_Chann_Cfg ) - 1)  // idk ... get the max index, make it invalid so that element 0 can be valid default

// back-EMF measurements are acquired 4x each commutation sector 
uint16_t Back_EMF_R[4];
uint16_t Back_EMF_F[4];

uint16_t Back_EMF_F0_MA;
uint16_t Back_EMF_R0_MA;

uint16_t Back_EMF_F_tmp;
uint16_t Back_EMF_F0_MA_tmp;


uint16_t Global_ADC_Phase_A[ ADC_PHASE_BUF_SZ ];  // 3 phases ADC are read each TIM1 ISR
uint16_t Global_ADC_Phase_B[ ADC_PHASE_BUF_SZ ];  // 3 phases ADC are read each TIM1 ISR
uint16_t Global_ADC_Phase_C[ ADC_PHASE_BUF_SZ ];  // 3 phases ADC are read each TIM1 ISR

// prefer to work with an array of pointers rather than a double-array
Global_ADC_Phase_t Global_ADC_PhaseABC_ptr[] = { Global_ADC_Phase_A, Global_ADC_Phase_B, Global_ADC_Phase_C };

BLDC_PHASE_t Float_phase = PHASE_NONE;
BLDC_PWM_STATE_t Float_value = DC_NONE;

uint8_t BackEMF_Sample_Index;



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
void set_dutycycle(uint16_t global_dutycycle)
{
    global_uDC = global_dutycycle;
}

void inc_dutycycle(void)
{
    global_uDC += 1;
}

void dec_dutycycle(void)
{
    global_uDC -= 1;
}

/*
 * intermediate function for setting PWM with positive or negative polarity
 * Provides an "inverted" (complimentary) duty-cycle if [state0 < 0]
 *
 * This could probably be done better given a more coherent read of the PWM/timer confugration in the MCU reference manual!!
 */
uint16_t get_pwm_dc(uint8_t chan /* unused */, BLDC_PWM_STATE_t state)
{
    uint16_t pulse = PWM_0PCNT;

    switch(state)
    {
    default:
    case DC_OUTP_FLOAT_R:
    case DC_OUTP_FLOAT_F:
    case DC_OUTP_LO:
        pulse = PWM_0PCNT;
        break;
    case DC_OUTP_HI:
//        pulse = PWM_100PCNT;
//        break;
//    case DC_PWM_PLUS:
        pulse = global_uDC;
        break;
//    case DC_PWM_MINUS: // complimented i.e. (100% - DC)
//        pulse = TIM2_PWM_PD - global_uDC; // inverse pulse
//        break;
    }

    return pulse;
}

/*
 * crude
 */
void delay(int time)
{
    int d;
    for (d = 0 ; d < time; d++)
    {
        nop();
    }
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
  */
void comm_switch (uint8_t bldc_step)
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
     * disable PWM if previous driving phase is finished (120 degrees)
     */
    if ( DC_OUTP_HI == prev_A  && ( DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0 ) )
    {
        TIM1_CCxCmd( BLDC_PWM_Chann_Cfg[ PHASE_A ], DISABLE );
    }
    if ( DC_OUTP_HI == prev_B  && ( DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1 ) )
    {
        TIM1_CCxCmd( BLDC_PWM_Chann_Cfg[ PHASE_B ], DISABLE );
    }
    if ( DC_OUTP_HI == prev_C  && ( DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2 ) )
    {
        TIM1_CCxCmd( BLDC_PWM_Chann_Cfg[ PHASE_C ], DISABLE );
    }


    Float_value = DC_NONE;
    Float_phase = PHASE_NONE;

    if (DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0)
    {
        Float_phase = PHASE_A;
        Float_value = state0;
        GPIOC->ODR &=  ~(1<<5);      // /SD A OFF
    }
    else if (DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1)
    {
        Float_phase = PHASE_B;
        Float_value = state1;
        GPIOC->ODR &=   ~(1<<7);     // /SD B OFF
    }
    else if (DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2)
    {
        Float_phase = PHASE_C;
        Float_value = state2;
        GPIOG->ODR &=   ~(1<<1);     // /SD C OFF
    }


/*
 * The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off (IR2104 enabled)
 */
    if (DC_OUTP_LO == state0)
    {
// let the Timer PWM channel remain disabled, PC2 is LO, /SD.A is ON
        GPIOC->ODR &=  ~(1<<2);  // PC2 set LO
        GPIOC->ODR |=   (1<<5);  // set C5 ( /SD A )
    }
    else if (DC_OUTP_LO == state1)
    {
// let the Timer PWM channel remain disabled, PC3 is LO, /SD.B is ON
        GPIOC->ODR &=  ~(1<<3);  // PC3 set LO
        GPIOC->ODR |=   (1<<7);  // set C7 ( /SD B )
    }
    else if (DC_OUTP_LO == state2)
    {
// let the Timer PWM channel remain disabled, PC4 is LO, /SD.C is ON
        GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
        GPIOG->ODR |=   (1<<1);  // set G1 ( /SD C )
    }


    /*
     This delay waits for settling of flyback effect after the PWM transition. Back-EMF
     can be read from the (previously energized) phase which is now floating (and of course for now, PWM is off!)
    */
#ifdef COMM_TIME_KLUDGE_DELAYS

#ifdef PWM_8K // 8k PWM

//    delay(  75 ); // back-EMF "window" of severl steps, but uugghhh ! delay is very noticeable on scope trace!!!!
//    delay(  15 );        // about 20us
    delay(  45 );        // make sure the back-EMF is somewhat settled after demag-time
#else //  12k PWM .. longer delay makes the back-EMF "window" wider by a couple steps ... but uuugghhh delay !
//    delay(  40 ); //   41 40 ... stall ?
    delay(  30 );        // 
#endif

#endif // COMM_TIME_KLUDGE_DELAYS



    /* Back-EMF reading hardcoded to phase "A" (ADC_0)
     * The falling Back-EMF should be readable at 0 degrees sector.
     * Rising back-EMF would need to be read beginning at the 30 degreess
     * sector to see if it is above threshold ( around 0.7 v ).
     */
    if (  DC_OUTP_FLOAT_F == state0 )
    {
        uint16_t u16tmp = Back_EMF_F_tmp;
        Back_EMF_F_tmp = ADC1_GetBufferValue( 0 /* hardcoded to phase "A" (ADC_0) */);
        Back_EMF_F0_MA_tmp  = ( u16tmp +  Back_EMF_F_tmp ) / 2;
    }


    /*
     * reconfig and re-enable PWM of the driving channels. One driving channel is
     * PWMd, the other is continuously Off. Both driving IR2104s must be enabeld
     * by setting its /SD input line.
     */
    if (DC_OUTP_HI == state0)
    {
        TIM1_SetCompare2( get_pwm_dc(0, state0) );
        TIM1_CCxCmd(TIM1_CHANNEL_2, ENABLE);
        GPIOC->ODR |=   (1<<5);  // set /SD A
    }

    if (DC_OUTP_HI == state1)
    {
        TIM1_SetCompare3( get_pwm_dc(1, state1) );
        TIM1_CCxCmd(TIM1_CHANNEL_3, ENABLE);
        GPIOC->ODR |=   (1<<7); // set  /SD B
    }

    if (DC_OUTP_HI == state2)
    {
        TIM1_SetCompare4( get_pwm_dc(2, state2) );
        TIM1_CCxCmd(TIM1_CHANNEL_4, ENABLE);
        GPIOG->ODR |=   (1<<1); // set /SD C
    }

    TIM1_CtrlPWMOutputs(ENABLE);  // apparently this is required after re-config PWM
}


extern uart_print( char * sbuf ); // tmp
/*
 *
 */
void BLDC_Stop()
{
    if (BLDC_OFF != BLDC_State)
    {
        Log_Level = 0;
        uart_print( "STOP\r\n");
    }

    BLDC_State = BLDC_OFF;
    set_dutycycle( 0 );
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
 * BLDC Update: handle the BLDC state
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
            // TODO: the actual transition to ON state would be seeing the ramp-to speed achieved in closed-loop operation
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
 */
int sample(int arg){
}


#define TIM3_RATE_MODULUS   4 // each modulus factor of 2 must relate to a reduction factor of 2 in TIM3 prescale

/*
 */
void BLDC_Step(void)
{
    static uint8_t bldc_step_modul; // internal counter for sub-tasking the TIM3 period

    if (BLDC_OFF != BLDC_State )
    {
        int index = bldc_step_modul % TIM3_RATE_MODULUS;

        sample(index);

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
