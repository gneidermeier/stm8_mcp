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


/* Private defines -----------------------------------------------------------*/

/*
 * a 30-50 uS delay is required for flyback diode action
 */
#ifdef CLOCK_16
#define FLYBACK_DELAY  40
#else
#define FLYBACK_DELAY  20
#endif

#define PWM_100PCNT    TIM2_PWM_PD
#define PWM_50PCNT     ( PWM_100PCNT / 2 )
#define PWM_25PCNT     ( PWM_100PCNT / 4 )
#define PWM_0PCNT      0
#define PWM_DC_RAMPUP  PWM_50PCNT // 52 // exp. determined


#ifndef PWM_IS_MANUAL
#define PWM_NOT_MANUAL_DEF  PWM_25PCNT //30 //0x20 // experimentally determined value (using manual adjustment)
#endif



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

// the OL comm time is shortened by 1 rammp-unit (e.g. 2 counts @ 0.000008S per count where 8uS is the TIM3 bit-time)
// the ramp shortens the OL comm-time by 1 RU each ramp-step with each ramp step is the TIM1 period of ~1mS
#define BLDC_ONE_RAMP_UNIT          1

// 1 cycle = 6 * 8uS * 512 = 0.024576 S
#define BLDC_OL_TM_LO_SPD         512  // start of ramp

// 1 cycle = 6 * 8uS * 80 = 0.00384 S
#define BLDC_OL_TM_HI_SPD          80  // end of ramp

// 1 cycle = 6 * 8uS * 50 = 0.0024 S
#define BLDC_OL_TM_MANUAL_HI_LIM   64 // stalls at ... 56 ? hi-enuff to manually 
                                      // advance to (and slightly past) "ideal" timing point

// any "speed" setting higher than HI_LIM would be by closed-loop control of
// commutation period (manual speed-control input by adjusting PWM  duty-cycle)
// The control loop will only have precision of 0.000008 S adjustment though (externally triggered comparator would be ideal )

// 1 cycle = 6 * 8uS * 13 = 0.000624 S
// 1 cycle = 6 * 8uS * 14 = 0.000672 S
#define LUDICROUS_SPEED (13) // 15kRPM would be ~13.8 counts


#define THREE_PHASES 3

/* Private types -----------------------------------------------------------*/

// enumerates the PWM state of each channel
typedef enum DC_PWM_STATE
{
    DC_OUTP_OFF,
    DC_PWM_PLUS,
    DC_PWM_MINUS, // complimented i.e. (100% - DC)
    DC_OUTP_HI,
    DC_OUTP_LO,
    DC_OUTP_FLOAT,
    DC_NONE
} DC_PWM_STATE_t;

// enumerate 3 phases
typedef enum THREE_PHASE_CHANNELS
{
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
uint16_t BLDC_OL_comm_tm;   // could be private

uint16_t Manual_uDC;

BLDC_STATE_T BLDC_State;


/* Private variables ---------------------------------------------------------*/

static uint16_t Ramp_Step_Tm; // reduced x2 each time but can't start any slower
/* static */ uint16_t global_uDC;

/* Private function prototypes -----------------------------------------------*/
void bldc_move( COMMUTATION_SECTOR_t );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
void PWM_Set_DC(uint16_t pwm_dc)
{
    global_uDC = pwm_dc;

    if ( BLDC_OFF == BLDC_State )
    {
//        global_uDC = 0;
    }
}

/*
 * intermediate function for setting PWM with positive or negative polarity
 * Provides an "inverted" (complimentary) duty-cycle if [state0 < 0]
 */
uint16_t _set_output(uint8_t chan, DC_PWM_STATE_t state0)
{
    uint16_t pulse = PWM_0PCNT;

    switch(state0)
    {
    default:
    case DC_OUTP_FLOAT:
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
  *  The way it's described in Toshiba AN, as if the PWMd leg turns off,
  *  the (non-PWM) also remains off, and flyback-diode becomes active.
  *  Does IR2104 dead-time account for this ?:
  *
  * Second: assert /SD ==OFF  of (only!) the PWMd FET - to ensure that flyback
  *  diode is complete (de-energizing the coil that is now being transitioned
  *  to float). This seems to be the only way to ensure IR2104 set both switch non-conducting.
  *
  * Ideally, when a phase is a 60degress (half of its time) there should be
  * no change/disruption to  its PWM signal.

  */
void PWM_set_outputs(DC_PWM_STATE_t state0, DC_PWM_STATE_t state1, DC_PWM_STATE_t state2)
{
    int8_t cntr; // make it signed so I get the counter "underflow"
    int8_t pre_cntr ;// test counter delta

    /* todo: look into this?:
    "For correct operation, preload registers must be enabled when the timer is in PWM mode. This
    is not mandatory in one-pulse mode (OPM bit set in TIM1_CR1 register)."
    */

    /*
     * disable PWM channels in order to assert the PWM timer channels and set
     * the GPIO pin config to non-PWM state (OFF, for upper-arm chopping).
     * Complete de-Init and re-initialization of Timer 1 doesn't seem necessary,
     * and ideally, to not to disrupt the PWM signal on the
     * driven phase (which continues for 2 "sectors" (120 electrical degrees).
     *
     * A coneivable improvement might be to better sync commutation switching with PWM period:
     * Ideally, when a phase is a 60degress (half of its time) there should be
     * no change/disruption to  its PWM signal.
     * Maybe TIM1 ocunter could better be used for this and also to avoid the hard delay?
     * 1) set a flag that commuatation time has arrived
     * 2) wait for the next TIM1 period to expire (up to 125uS)
     * 3) perform the commutation (call "PWM_set_outputs()", but from within the TIM1 ISR context.
     * 4) update PWM duty-cycle in TIM1 config .
     *
     * If PWM is ON, and turns off, then the output goes low when the Channel
     * and the PWM Outputs is Disabled.
     * However, when the peripheral module sets the output low, the back-EMF is 
     *    also biased negatively, and a delay must be allowed for flyback diode action
     *to allow back-EMF to "recover".
     */

		 // stop PWM channels (but don't disable TIM1 yet so we can use the freerunning timer for de-mag delay
    TIM1_CCxCmd(TIM1_CHANNEL_2, DISABLE);
    TIM1_CCxCmd(TIM1_CHANNEL_3, DISABLE);
    TIM1_CCxCmd(TIM1_CHANNEL_4, DISABLE);
    TIM1_CtrlPWMOutputs(DISABLE);

    /*
     * When a phase is transitioned to Floating, then the other 2 IR2104s are enabled.
     *    This is where the "stepdown" of back-EMF occurs.
     * DEFINATELY LAST THING TO DO before turning on the driving channel. ... .. no change of plan ... 
		                ... the channel that is floating (about to be PWM) will "echo" 
										   the comm. switch pulse, so FIRST (after stop PWM) set the driving channels to GND
if a phase is floating up from low-drive, when the lower-switch turn off it flyback clamp to HV
 When the PWM on the upper-arm turn-off, the flyback is to clamp to GND so this is ideal discharge of flaoting coil.
 
 If a phase is floating down from high-pwm, when the upper-switch turn off the flyback is to GND by the lower body diode.
 Ideally the lower-arm is then PWMd and when the low-side switch off the flback of upper body diode is to HV. 
 This does not keep the previous state, so the direction of the float transition is not known.
 The flyback discharge of this 1100kv motor seems to need ~50uS holdoff before
 restarting the PWM, otherwise the back-EMF may get squashed until the higher speed is reached.
 THere would not seem to be enough energy in the small motor to worrry about trying 
 to coordinate the flyback discharge with the oncoming (next-leg) PWM.
*/

   if (DC_OUTP_FLOAT == state0)
    {
            GPIOC->ODR &=  ~(1<<5);      // /SD A OFF 			
        // other two legs have /SD enabled
// /SD A OFF 
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);
    }
    else if (DC_OUTP_FLOAT == state1)
    {
            GPIOC->ODR &=   ~(1<<7);     // /SD B OFF 			
        // other two legs have /SD enabled
        GPIOC->ODR |=   (1<<5);
// /SD B OFF 
        GPIOG->ODR |=   (1<<1);
    }
    else if (DC_OUTP_FLOAT == state2)
    {
            GPIOG->ODR &=   ~(1<<1);     // /SD C OFF 			
        // other two legs have /SD enabled
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
// /SD C OFF 
    }


// mark the start of commutation trigger
#if 0 //
    GPIOG->ODR |=  (1<<0); // tmp test
#endif


#if 1 // can I put here ?????????????????????????? yes apparently so, with neither good or ill effect.

// The "OFF" (non-PWMd) phase is asserted output pins to GPIO, driven Off
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
#endif

#if 1 // start of dead-time
    GPIOG->ODR |=  (1<<0); // tmp test
#endif
#if 1

    /*
     * allow TIM1 freerunning counter to provide small delay to wait for back-emf voltage to settle, before setting
     * the other 2 phases /SD in the section below. See note above, the 
     * action of the PWM module (as Channel and TIM1 PWM module is disabled) 
     * snaps the phase voltage to either   + or -  1/2 Vsys.
     */
//    delay( FLYBACK_DELAY );
//    cntr = 60; // 42 uS ... good @45

//    cntr = 70; // 46 uS scope triggers better @ 20uS ct@45

 cntr = 50; // 46 uS scope triggers better @ 20uS ct@45


    TIM1_SetCounter(cntr);
    while( cntr > 0 ) //  @ 8Mhz, 0.0000005 (1/2 uS)
    {
        pre_cntr = cntr; // seems to be counting off about 10 counts (or 5uS?) per pass
        cntr = TIM1_GetCounter();
    }
#endif

// Finished with the counter, so disble TIM1 completely, prior to doing PWM reconfig
    TIM1_Cmd(DISABLE);
    TIM1_SetCounter(0);

#if 1 // end of dead-time
    GPIOG->ODR &=  ~(1<<0); // tmp test
#endif

/*
 * reconfig and re-enable PWM of the driving channels. One driving channel is 
 * PWMd, the other is continuously Off. Both driving IR2104s must be enabeld 
 * by setting its /SD input line.
 */
    if (DC_OUTP_OFF != state0)
    {
        if (DC_OUTP_FLOAT != state0)
        {
            if (DC_PWM_PLUS == state0 /* MINUS? */)
            {
                TIM1_SetCompare2(_set_output(0, state0));
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
        if (DC_OUTP_FLOAT != state1)
        {
            if (DC_PWM_PLUS == state1 /* MINUS? */)
            {
                TIM1_SetCompare3(_set_output(1, state1));
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
        if (DC_OUTP_FLOAT != state2)
        {
            if (DC_PWM_PLUS == state2 /* MINUS? */)
            {
                TIM1_SetCompare4(_set_output(2, state2));
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

#if 0
    /*
     * When a phase is transitioned to Floating, then the other 2 IR2104s are enabled.
     *    This is where the "stepdown" of back-EMF occurs.
     * DEFINATELY LAST THING TO DO before turning on the driving channel.
     */
   if (DC_OUTP_FLOAT == state0)
    {
			
#endif

    /*
    When TIM1 PWM is re-enabled, then see the "big spike" as the next phase
     Channel output ON momentarily as the counter is reset.
     */
// counterparts to Disable commands above
    TIM1_CtrlPWMOutputs(ENABLE);
    TIM1_Cmd(ENABLE);
}


/*
 *
 */
void BLDC_Stop()
{
    BLDC_State = BLDC_OFF;
    PWM_Set_DC( 0 );
}

/*
 *
 */
void BLDC_Spd_dec()
{
    if (BLDC_OFF == BLDC_State)
    {
        BLDC_State = BLDC_RAMPUP;
        // BLDC_OL_comm_tm ... init in OFF state to _OL_TM_LO_SPD, don't touch!
    }

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm < 0xFFFF)
    {
        BLDC_OL_comm_tm += 1; // slower
    }
}

/*
 *
 */
void BLDC_Spd_inc()
{
    if (BLDC_OFF == BLDC_State)
    {
        BLDC_State = BLDC_RAMPUP;
        // BLDC_OL_comm_tm ... init in OFF state to _OL_TM_LO_SPD, don't touch!
    }

    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm > BLDC_OL_TM_MANUAL_HI_LIM )
    {
        BLDC_OL_comm_tm -= 1; // faster
    }
}


void TIM3_setup(uint16_t u16period); // tmp

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
#ifdef PWM_IS_MANUAL
// doesn't need to set global uDC every time as it would be set once in the FSM
// transition ramp->on ... but it doesn't hurt to assert it
        PWM_Set_DC( Manual_uDC ) ;  // #ifdef SYMETRIC_PWM ...  (PWM_50PCNT +  Manual_uDC / 2)
#else
//         PWM_Set_DC( PWM_NOT_MANUAL_DEF ) ;
#endif
        break;
    case BLDC_RAMPUP:

        PWM_Set_DC( PWM_DC_RAMPUP ) ;

        if (BLDC_OL_comm_tm > BLDC_OL_TM_HI_SPD) // state-transition trigger?
        {
            BLDC_OL_comm_tm -= BLDC_ONE_RAMP_UNIT;
        }
        else
        {
            // TODO: the actual transition to ON state would be seeing the ramp-to speed
// achieved in closed-loop operation
            BLDC_State = BLDC_ON;
            PWM_Set_DC( PWM_NOT_MANUAL_DEF );
        }
        break;
    }

#if 1 //    ! MANUAL TEST
//  update the timer for the OL commutation switch time
    TIM3_setup(BLDC_OL_comm_tm);
#endif
}

/*
 * TODO: schedule at 15degree intervals? (see TIM3)	????
 Start a short timer on which ISR will then  trigger the A/D with proper timing  .... at 1/4 of the comm. cycle ?
 So TIM3 would not stepp 6 times but 6x4 times? (4 times precision?)
 */
void BLDC_Step(void)
{
    const uint8_t N_CSTEPS = 6;

    static COMMUTATION_SECTOR_t bldc_step = 0;

    bldc_step += 1;
    bldc_step %= N_CSTEPS;

    if (global_uDC > 0)
    {
        bldc_move( bldc_step );
    }
    else // motor drive output is not active
    {
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(DC_OUTP_OFF, DC_OUTP_OFF, DC_OUTP_OFF);
    }
}

/*
 * TODO: set output states (hi, lo, float, plus, minus) from look-up tables	?
 */
void bldc_move(COMMUTATION_SECTOR_t step )
{
// Start a short timer on which ISR will then  trigger the A/D with proper
//  timing  .... at 1/4 of the comm. cycle ?
// So this TIM3 would not stepp 6 times but 6x4 times? (4 times precision?)
// ....... (yes seems necessary (refer to SiLabs appnote)

    switch( step )
    {
    default:
        PWM_set_outputs(DC_OUTP_OFF, DC_OUTP_OFF, DC_OUTP_OFF);
        break;

    case 0: // SECTOR_1 etc.
        PWM_set_outputs(DC_PWM_PLUS, DC_OUTP_LO, DC_OUTP_FLOAT);
        break;

    case 1:
        PWM_set_outputs(DC_PWM_PLUS, DC_OUTP_FLOAT, DC_OUTP_LO);
        break;

    case 2:
        PWM_set_outputs(DC_OUTP_FLOAT, DC_PWM_PLUS, DC_OUTP_LO);
        break;

    case 3:
        PWM_set_outputs(DC_OUTP_LO, DC_PWM_PLUS, DC_OUTP_FLOAT);
        break;

    case 4:
        PWM_set_outputs(DC_OUTP_LO, DC_OUTP_FLOAT, DC_PWM_PLUS);
        break;

    case 5:
        PWM_set_outputs(DC_OUTP_FLOAT, DC_OUTP_LO, DC_PWM_PLUS);
        break;
    }
}
