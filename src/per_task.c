/**
  ******************************************************************************
  * @file per_task.c
  * @brief Background task / periodic task
  * @author Neidermeier
  * @version
  * @date Dec-2020
  ******************************************************************************
  */
/**
 * \defgroup per_task Periodic Task
 * @brief Background task / periodic task
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

// app headers
#include "mcu_stm8s.h"
#include "sequence.h"
#include "bldc_sm.h"
#include "faultm.h"


/* Private defines -----------------------------------------------------------*/

#if 1 // test/dev
#define TRIM_DEFAULT 28 // close to the minimum ramp DC
#else
#define TRIM_DEFAULT  0 //
#endif

#define V_SHUTDOWN_THR      0x0340 // experimentally determined!

#define LOW_SPEED_THR       20     // turn off before low-speed low-voltage occurs

/**
 * @brief  Internal type for working with the state enumerations.
 * @details If the declaration for the state-type changes, there will be only one
 * place to change it in this module.
 */
typedef BLDC_STATE_T  STATEM_T;

/* Public variables  ---------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

static uint16_t Analog_slider; // input var for 10-bit ADC conversions
static uint8_t UI_Speed;       // speed setting in 8-bits
static int8_t Digital_trim_switch; // trim switches have + and - extents

static uint8_t TaskRdy;  // flag for timer interrupt for BG task timing

static uint8_t Log_Level;

static  uint16_t Vsystem; // persistent for averaging


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * home-made itoa function (16-bits only, hex-only)
 * seems Cosmic only provide atoi in stdlib and not itoa
 */
static char * itoa(uint16_t u16in, char *sbuf, int base)
{
    int x;
    int shift = 16 - 4;	/* 4-bits to 1 nibble */
    uint16_t n16 = u16in;

    if (16 != base)
    {
        return NULL;
    }

    x = 0;
    while (x < 4 /* 4 nibbles in 16-bit word */ )
    {
        unsigned char c = (uint8_t)( (n16 >> shift) & 0x000F );

        if (c > 9)
        {
            c -= 10;
            c += 'A';
        }
        else
        {
            c += '0';
        }
        sbuf[x++] = c;
        shift -= 4;
    }
    sbuf[x] = 0;

    return sbuf;
}


/*******************************************************************************
* Function Name  : GetKey
* Description    : Get a key from the HyperTerminal
* Input          : None
* Output         : None
* Return         : The Key Pressed
*******************************************************************************/
static char GetKey(void)
{
    char key = 0;
    /* Waiting for user input */
    while (1)
    {
        if (SerialKeyPressed((char*)&key)) break;
    }
    return key;
}


/** @cond */ // hide some developer/debug code
// hack, temp
extern int Back_EMF_Falling_PhX;
extern int Back_EMF_Riseing_PhX;
/** @endcond */

/*
 * temp, todo better function
 */
static void testUART(int clear)
{
    static uint16_t Line_Count = 0;

    char sbuf[256] ;                     // am i big enuff?
    char cbuf[8] = { 0, 0 };

// TODO yes probably use DI/EI to do all shared access in one swoop
// DI
int16_t timing_error = Seq_get_timing_error();
//  BLDC_PWMDC_Get()
//  get_commutation_period()
//  Faultm_get_status
//  Seq_Get_Vbatt()
//  Faultm_upd()
// EI

    sbuf[0] = 0;

    if ( 0 != clear)
    {
        Line_Count  = 0;
    }

    Line_Count  += 1;;

    strcat(sbuf, "(");
    itoa(Line_Count, cbuf, 16);
    strcat(sbuf, cbuf);
    strcat(sbuf, ")");

    strcat(sbuf, " CT=");
    itoa(get_commutation_period(), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " DC=");
    itoa( BLDC_PWMDC_Get(),     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " Vs=");
    itoa( Vsystem,     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " SF=");
    itoa( Faultm_get_status(),     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " TTE=");
    itoa( timing_error,     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " bRi=");
    itoa( Back_EMF_Riseing_PhX,     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " bFi=");
    itoa( Back_EMF_Falling_PhX,     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " UI=");
    itoa(UI_Speed, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " AS=");
    itoa(Analog_slider, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " SM=");
    itoa(get_bldc_state(), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
}

/*
 * monitors system voltage (stalled, stopped motor, over-current)
 * The voltage measurement is understood to be while PWM cycle is driving one or more
 * motor phases, so close to the DC rail voltage.
 */
static void sys_voltage_mon(STATEM_T sm_state)
{
    // eventually the sequencer should be averaging all 3 phases for this
    Vsystem = ( Seq_Get_Vbatt() + Vsystem ) / 2; // sma

// system voltage is only available and useable when machine is on
    if (BLDC_RUNNING == get_bldc_state() )
    {
        Faultm_upd(VOLTAGE_NG, (faultm_assert_t)( Vsystem < V_SHUTDOWN_THR) );
    }
}

/*
 * Service the slider and trim inputs for speed setting.
 * The UI Speed value is a uint8 and represents the adjustment range of e.g. a
 * proportional RC radio control signal (eventually), and alternatively the
 * slider-pot (the developer h/w) - the UI Speed is passed to PWMDC_set() where
 * is expected to be rescaled to suite the range/precision required for PWM timer.
 *
 * TODO: rate limit of speed input!
 */
static void set_ui_speed(STATEM_T sm_state)
{
    int16_t tmp_sint16;

//   svc a UI potentiometer
    uint16_t adc_tmp16 = ADC1_GetBufferValue( ADC1_CHANNEL_3 );
    Analog_slider = adc_tmp16 / 4; // [ 0: 1023 ] -> [ 0: 255 ]

// careful with expression containing signed int ... ui_speed is defaulted
// to 0 and only assign from temp sum if positive and clip to INT8 MAX S8.
    UI_Speed = 0; // obviously not doing any averaging w/ this

    tmp_sint16 = Digital_trim_switch;
    tmp_sint16 += Analog_slider; // neutered for now

    if (tmp_sint16 > 0)
    {
        // clip to INT8 MAX S8
        if (tmp_sint16 > U8_MAX)
        {
            tmp_sint16 = U8_MAX;
        }

        UI_Speed = (uint8_t)tmp_sint16;
    }

// only OFF state is of interest for Throttle-high diagnostic.
// There is no check for the error condition to go away - the user
// would need to lower the throttle stick and then the system to reset.
    if (BLDC_RESET == sm_state)
    {
        // require stick to be put down before arming/ready
        if (Analog_slider > 0)
        {
            Faultm_set(THROTTLE_HI);
            UI_Speed = 0;
        }
    }
    else if (BLDC_RUNNING == sm_state)
    {
        if (Analog_slider > 0) //
        {
            // don't get to slow or it will stall and throw a low-voltage fault
            if (UI_Speed < LOW_SPEED_THR)
            {
// u8_max to be used as an out-of-band value which can signify "Reset/Stop"
                UI_Speed = U8_MAX;
                BLDC_Stop(); // ... PWMDC_Set() can  call Stop() ?

                // log some info
                Log_Level = 10;
            }
        }
    }

    BLDC_PWMDC_Set(UI_Speed);
}

/**
 * @brief stop the system
 *
 * needs to be externable to main because the hard-button stop button is polled there
 */
void UI_Stop(void)
{
// reset the simulated trim swtich between system runs
    Digital_trim_switch = TRIM_DEFAULT;
    UI_Speed = 0;
 
 // reset the machine
    BLDC_Stop();
}


/*
 * Execution context is 'main()' so not to block ISR with ADC sampling.
 * TODO moving A/D acquisition to ramp-step (TIM3) to coordinate with PWM on/off cycling.
 * Servicing the UART (presently just a simply one-way logging stream)
 */
static void Periodic_task(void)
{
    char sbuf[16] = "";
    char cbuf[8] = { 0, 0 };
    int16_t tmp_sint16;
    char key;
    BLDC_STATE_T sm_state;

//disableInterrupts();

    sm_state = get_bldc_state();

//enableInterrupts();


    // update system voltage diagnostic
    sys_voltage_mon( (STATEM_T) sm_state );

    // update the UI speed input slider+trim
    set_ui_speed( (STATEM_T) sm_state );

    /*
     * debug logging information can be "scheduled" by setting the level, which for now
     * simply designates number of reps ... spacing TBD? this task is now tied to
     * BLDC Update cycle ( about 1/2sec?)
     */
    if (Log_Level > 0)
    {
// allow log to sticky-on
        if (Log_Level < 255)
        {
            Log_Level -= 1;
        }

        testUART(0);// tmp test
    }

    if (SerialKeyPressed(&key))
    {
//        Log_Level = 1; // show some info

        if (key == ' ') // space character
        {
            // reset the machine
            UI_Stop();

            UARTputs("###\r\n");

            Log_Level = 1; // stop the logger output
            testUART(1 /* clear line count */ );

// reset the simulated trim swtich between system runs
            Digital_trim_switch = TRIM_DEFAULT;
        }
        else if (key == '+')
        {
// if fault/throttle-high ... diag msg?
            if (Digital_trim_switch < S8_MAX)
            {
                Digital_trim_switch += 1;
            }
            UARTputs("+++\r\n");
//            Log_Level = 255; // ahow some more info
        }
        else if (key == '-')
        {
// if fault/throttle-high ... diag msg?
            if (Digital_trim_switch > S8_MIN)
            {
                Digital_trim_switch -= 1;
                Log_Level = 1;
            }
            UARTputs("---\r\n");
        }
        else // anykey
        {
            Log_Level = 255;// enable continous/verbous log
        }

        itoa(UI_Speed, cbuf, 16);
        strcat(sbuf, cbuf);
        strcat(sbuf, "\r\n");
        UARTputs(sbuf);
    }
}

/**
 * @brief Run Periodic Task if ready
 * @details
 * Called in non-ISR context - checks the background task ready flag which if !0
 * will invoke the Periodic Task function.
 * @note referred to as Pertask_chk_ready
 */
uint8_t Task_Ready(void)
{
    if (0 != TaskRdy)
    {
        TaskRdy = FALSE;
        Periodic_task();
    }
    return TaskRdy;
}

/**
 * @brief  Trigger background task.
 * @details
 * Called in ISR context - sets the background task ready flag which when seen
 * by polling Task_Ready in background task will invoke the Periodic Task function.
 * @note referred to as Pertask_set_ready
 */
void Periodic_Task_Wake(void)
{
    TaskRdy = TRUE; // notify background process
}

/**@}*/ // defgroup
