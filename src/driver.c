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
#define PWM_0PCNT      0
#define PWM_DC_RAMPUP  PWM_50PCNT // 52 // exp. determined


#define PWM_20PCNT     ( PWM_100PCNT / 5)          // 50
#define PWM_21PCNT     ( 21 * PWM_100PCNT / 100)   // 52.5
#define PWM_22PCNT     ( 22 * PWM_100PCNT / 100)   // 55    // $37
#define PWM_23PCNT     ( 23 * PWM_100PCNT / 100)
#define PWM_24PCNT     ( 24 * PWM_100PCNT / 100)

#define PWM_DC_IDLE    PWM_22PCNT // stall 50% of the time
 
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

// 1 cycle = 6 * 8uS * 80 = 0.00384 S
//#define BLDC_OL_TM_HI_SPD          (80 * BLDC_CT_SCALE)  // end of ramp ... period ~ 4mS
//#define BLDC_OL_TM_HI_SPD          (68 * BLDC_CT_SCALE)  // end of ramp ... period ~ 3.2ms
#define BLDC_OL_TM_HI_SPD          (69 * BLDC_CT_SCALE)  // CT=$0230   // 560/8=70

// 1 cycle = 6 * 8uS * 50 = 0.0024 S
#define BLDC_OL_TM_MANUAL_HI_LIM   (63 * BLDC_CT_SCALE) // 64 // stalls in the range of 62-64 dependng on delays
// advance to (and slightly past) "ideal" timing point

// any "speed" setting higher than HI_LIM would be by closed-loop control of
// commutation period (manual speed-control input by adjusting PWM  duty-cycle)
// The control loop will only have precision of 0.000008 S adjustment though (externally triggered comparator would be ideal )

// 1 cycle = 6 * 8uS * 13 = 0.000624 S
// 1 cycle = 6 * 8uS * 14 = 0.000672 S
#define LUDICROUS_SPEED            (13 * BLDC_CT_SCALE)  // 15kRPM would be ~13.8 counts


/* Private types -----------------------------------------------------------*/

// enumerates the PWM state of each channel
typedef enum DC_PWM_STATE
{
    DC_OUTP_OFF,
    DC_PWM_PLUS,
    DC_PWM_MINUS, // complimented i.e. (100% - DC)
    DC_OUTP_HI,
    DC_OUTP_LO,
    DC_OUTP_FLOAT_R,
    DC_OUTP_FLOAT_F,
    DC_NONE
} DC_PWM_STATE_t;

/*
 * aggregate 3 phases into a struct for easy param passing and putting in table
 */
typedef struct
{
    DC_PWM_STATE_t phaseA;
    DC_PWM_STATE_t phaseB;
    DC_PWM_STATE_t phaseC;
}
DC_PWM_PH_STATES_t;

/*
 * define an integer "triplet" type for storing accumulated back-EMF values
 */
typedef struct
{
    uint16_t phaseA;
    uint16_t phaseB;
    uint16_t phaseC;
}
BACK_EMF_AD_t;


// enumerate 3 phases
typedef enum THREE_PHASE_CHANNELS
{
    PHASE_NONE,
    PHASE_A,
    PHASE_B,
    PHASE_C
} THREE_PHASE_CHANNELS_t;

//enumerate available PWM drive modes
typedef enum PWM_MODE
{
    UPPER_ARM,
    LOWER_ARM
    // SYMETRICAL ... upper and lower arms driven (complementary) .. maybe no use for it
} PWM_MODE_t;

//enumerate commutation "sectors" (steps)
typedef enum COMMUTATION_SECTOR
{
    SECTOR_1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} COMMUTATION_SECTOR_t;


/* Public variables  ---------------------------------------------------------*/

// 2 elements (sample 1 and sample 2)
uint16_t Back_EMF_R[2];
uint16_t Back_EMF_F[2];
uint16_t Back_EMF_F0_MA;

uint16_t BLDC_OL_comm_tm;   // could be private

uint16_t global_uDC;

uint16_t Manual_uDC; // user speed input should be controlling PWM duty-cycle eventually ...

BLDC_STATE_T BLDC_State;


/* Private variables ---------------------------------------------------------*/


// max nr of back-EMF readings (3? 4? how many PWMs will be readale during each 60 degree float (of which only 30degree is seen w/ upp-erarm driving anyay!!)


/*
 * table of commutation states: confirmed that the elements of type
 * DC_PWM_PH_STATES t are occurpying 3-bytes each and packed (odd-aligned bytes should generally not be a problem for the CPU or the comp/linker)
 */
static const DC_PWM_PH_STATES_t Commutation_States[] =
{
// sector 0:
    { DC_PWM_PLUS,      DC_OUTP_LO,      DC_OUTP_FLOAT_F },
// sector 1:
    { DC_PWM_PLUS,      DC_OUTP_FLOAT_R, DC_OUTP_LO },
// sector 2:
    { DC_OUTP_FLOAT_F,  DC_PWM_PLUS,     DC_OUTP_LO },
// sector 3:
    { DC_OUTP_LO,       DC_PWM_PLUS,     DC_OUTP_FLOAT_R },
// sector 4:
    { DC_OUTP_LO,       DC_OUTP_FLOAT_F, DC_PWM_PLUS },
// sector 5:
    { DC_OUTP_FLOAT_R,  DC_OUTP_LO,      DC_PWM_PLUS }
};


static uint16_t Ramp_Step_Tm; // reduced x2 each time but can't start any slower

/*
 * RUNNING OUT OF RAM!!!!!!!!!!!!!!!
 */


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

    if ( BLDC_OFF == BLDC_State )
    {
//        global_uDC = 0;
    }
}


/*
 * intermediate function for setting PWM with positive or negative polarity
 * Provides an "inverted" (complimentary) duty-cycle if [state0 < 0]
 *
 * This could probably be done better given a more coherent read of the PWM/timer confugration in the MCU reference manual!!
 */
uint16_t get_pwm_dc(uint8_t chan /* unused */, DC_PWM_STATE_t state)
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
        pulse = PWM_100PCNT;
        break;
    case DC_PWM_PLUS:
        pulse = global_uDC;
        break;
    case DC_PWM_MINUS: // complimented i.e. (100% - DC)
        pulse = TIM2_PWM_PD - global_uDC; // inverse pulse
        break;
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


/*
 * see Issue #6
 * At the end of driven sectors (2 x 60 degree = 120 degrees driving duration),
 * the PWM would be in an indeterminate state (hi or lo) depending on TIM1 duty-cycle
 * and how the comm. switch timing (TIM3) happens to line up with it (if comm. switch
 * happends during on or off segment)..
 * Only by calling TIM1_DeInit() has it been possible to
 * assert the state of the PWM signal on the sector that is being transitioned ->FLOAT .
 * But this has been a problem in that, the 2 consecutive driving sectors should not
 * really be reconfigured unless there is a way to do it w/o causing a disruption during
 * the 120 driving semgnet which appears as voltage noise spike on the motor phase output
 */

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
void comm_switch ( DC_PWM_PH_STATES_t  states )
// void comm_switch ( uint8_t bldc_step)
{
    THREE_PHASE_CHANNELS_t float_phase = PHASE_NONE;
    DC_PWM_STATE_t float_value= DC_NONE;

//    DC_PWM_PH_STATES_t  states = Commutation_States[ bldc_step ];

    DC_PWM_STATE_t state0 = states.phaseA;
    DC_PWM_STATE_t state1 = states.phaseB;
    DC_PWM_STATE_t state2 = states.phaseC;

    uint16_t u16tmp;

    /* todo: look into this?:
        "For correct operation, preload registers must be enabled when the timer is in PWM mode. This
        is not mandatory in one-pulse mode (OPM bit set in TIM1_CR1 register)."
        */
    TIM1_ITConfig(TIM1_IT_UPDATE, DISABLE); // dsable interrupts ???? .. (there is an ISR triggered as soon as the channel/PWM disabled

    TIM1_CCxCmd(TIM1_CHANNEL_2, DISABLE);
    TIM1_CCxCmd(TIM1_CHANNEL_3, DISABLE);
    TIM1_CCxCmd(TIM1_CHANNEL_4, DISABLE);
    TIM1_CtrlPWMOutputs(DISABLE);



    if (DC_OUTP_FLOAT_R == state0 || DC_OUTP_FLOAT_F == state0)
    {
        float_phase = PHASE_A;
        float_value= state0;

        GPIOC->ODR &=  ~(1<<5);      // /SD A OFF
        // other two legs have /SD enabled
// /SD A OFF
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);
    }
    else if (DC_OUTP_FLOAT_R == state1 || DC_OUTP_FLOAT_F == state1)
    {
        float_phase = PHASE_B;
        float_value= state1;

        GPIOC->ODR &=   ~(1<<7);     // /SD B OFF
        // other two legs have /SD enabled
        GPIOC->ODR |=   (1<<5);
// /SD B OFF
        GPIOG->ODR |=   (1<<1);
    }
    else if (DC_OUTP_FLOAT_R == state2 || DC_OUTP_FLOAT_F == state2)
    {
        float_phase = PHASE_C;
        float_value= state2;

        GPIOG->ODR &=   ~(1<<1);     // /SD C OFF
        // other two legs have /SD enabled
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
// /SD C OFF
    }


    /* delay ...pin setting
     */
    GPIOG->ODR |=  (1<<0); // TEST PIN ON


// The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off
// (this  phase was with already off, or was floating )
    if (DC_OUTP_LO == state0)
    {
// let the Timer PWM channel remain disabled, PC2 is LO, /SD.A is ON
        GPIOC->ODR &=  ~(1<<2);  // PC2 set LO
        GPIOC->DDR |=  (1<<2);
        GPIOC->CR1 |=  (1<<2);
    }
    else if (DC_OUTP_LO == state1)
    {
// let the Timer PWM channel remain disabled, PC3 is LO, /SD.B is ON
        GPIOC->ODR &=  ~(1<<3);  // PC3 set LO
        GPIOC->DDR |=  (1<<3);
        GPIOC->CR1 |=  (1<<3);
    }
    else if (DC_OUTP_LO == state2)
    {
// let the Timer PWM channel remain disabled, PC4 is LO, /SD.C is ON
        GPIOC->ODR &=  ~(1<<4);  // PC4 set LO
        GPIOC->DDR |=  (1<<4);
        GPIOC->CR1 |=  (1<<4);
    }


/*
 This delay waits for settling of flyback effect after the PWM transition. Back-EMF
 can be read from the (previously energized) phase which is now floating (and of course for now, PWM is off!)
*/
#ifdef COMM_TIME_KLUDGE_DELAYS

#ifdef PWM_8K // 8k PWM

//    delay(  75 ); // back-EMF "window" of severl steps, but uugghhh ! delay is very noticeable on scope trace!!!!
    delay(  15 );        // about 20us

#else //  12k PWM .. longer delay makes the back-EMF "window" wider by a couple steps ... but uuugghhh delay !
    delay(  40 ); //   41 40 ... stall ?
#endif

#endif // COMM_TIME_KLUDGE_DELAYS



// Finished with the counter, so disble TIM1 completely, prior to doing PWM reconfig
    TIM1_Cmd(DISABLE);



    GPIOG->ODR &=  ~(1<<0); // TEST PIN OFF


    /* Back-EMF reading hardcoded to phase "A" (ADC_0)
		 * The falling Back-EMF should be readable at 0 degrees sector.
		 * Rising back-EMF would need to be read beginning at the 30 degreess 
		 * sector to see if it is above threshold ( around 0.7 v ).
    */
    if (  DC_OUTP_FLOAT_F == state0 )
    {
        u16tmp = Back_EMF_F[0];
        Back_EMF_F[0] = ADC1_GetBufferValue( 0 /* hardcoded to phase "A" (ADC_0) */);
        Back_EMF_F0_MA  = (u16tmp +  Back_EMF_F[0]) / 2;
    }


    /*
     * reconfig and re-enable PWM of the driving channels. One driving channel is
     * PWMd, the other is continuously Off. Both driving IR2104s must be enabeld
     * by setting its /SD input line.
     */
    if (DC_OUTP_OFF != state0)
    {
        if (DC_OUTP_FLOAT_R != state0 && DC_OUTP_FLOAT_F != state0)
        {
            if (DC_PWM_PLUS == state0 /* MINUS? */)
            {
                TIM1_SetCompare2( get_pwm_dc(0, state0) );
                TIM1_CCxCmd(TIM1_CHANNEL_2, ENABLE);
                // set /SD A ON
//        GPIOC->ODR |=  (1<<5);      // A.
            }
            else if (DC_OUTP_LO == state0)
            {
// let the Timer PWM channel remain disabled, PC2 is LO, /SD.A is ON
            }

//            GPIOC->ODR |=  (1<<5);      // set /SD A On  ... assert On below ...
        }
//        else // floating phase, assert /SD A OFF and other two phase-legs enabled
    }

    if (DC_OUTP_OFF != state1)
    {
        if (DC_OUTP_FLOAT_R != state1 && DC_OUTP_FLOAT_F != state1)
        {
            if (DC_PWM_PLUS == state1 /* MINUS? */)
            {
                TIM1_SetCompare3( get_pwm_dc(1, state1) );
                TIM1_CCxCmd(TIM1_CHANNEL_3, ENABLE);
                // set /SD B ON
//        GPIOG->ODR |=   (1<<7);
            }
            else if (DC_OUTP_LO == state1)
            {
// let the Timer PWM channel remain disabled, PC3 set LO, set /SD B  On
            }

//            GPIOG->ODR |=   (1<<7);     // set /SD B  On  ... assert On below ...
        }
//        else // floating phase, assert /SD B OFF and other two phase-legs enabled
    }

    if (DC_OUTP_OFF != state2)
    {
        if (DC_OUTP_FLOAT_R != state2 && DC_OUTP_FLOAT_F != state2)
        {
            if (DC_PWM_PLUS == state2 /* MINUS? */)
            {
                TIM1_SetCompare4( get_pwm_dc(2, state2) );
                TIM1_CCxCmd(TIM1_CHANNEL_4, ENABLE);
                // set /SD C ON
//        GPIOG->ODR |=   (1<<1);     // C
            }
            else if (DC_OUTP_LO == state2)
            {
// let the Timer PWM channel remain disabled, PC4 set LO, set /SD C  On
            }
//            GPIOG->ODR |=   (1<<1);     // set /SD C  On ... assert On below ...
        }
//        else // floating phase, assert /SD C OFF and other two phase-legs enabled
    }

// counterparts to Disable commands above
    TIM1_SetCounter(0);
    TIM1_CtrlPWMOutputs(ENABLE);

    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
    TIM1_Cmd(ENABLE);
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
 *
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
 *
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
            // TODO: the actual transition to ON state would be seeing the ramp-to speed
// achieved in closed-loop operation
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
 * TODO: schedule at 15degree intervals? (see TIM3)	????
 Start a short timer on which ISR will then  trigger the A/D with proper timing  .... at 1/4 of the comm. cycle ?
 So TIM3 would not stepp 6 times but 6x4 times? (4 times precision?)
 */
void BLDC_Step(void)
{
    const DC_PWM_PH_STATES_t OFF_State = { DC_OUTP_OFF, DC_OUTP_OFF, DC_OUTP_OFF };
    const uint8_t N_CSTEPS = 6;

    static COMMUTATION_SECTOR_t bldc_step = 0;

    bldc_step += 1;
    bldc_step %= N_CSTEPS;

    if ( 0 == global_uDC )
    {
        // motor drive output is not active
        GPIOC->ODR &=  ~(1<<5); //  /SD A
        GPIOC->ODR &=  ~(1<<7); //  /SD B
        GPIOG->ODR &=  ~(1<<1); //  /SD C

        TIM1_CtrlPWMOutputs(DISABLE);
    }
    else
    {
        comm_switch( Commutation_States[ bldc_step ] );
    }
}
