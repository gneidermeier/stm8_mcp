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
#include "driver.h"


/* Private defines -----------------------------------------------------------*/

#define TRIM_DEFAULT  0 //

#define V_SHUTDOWN_THR      0x0340 // experimentally determined!

#define LOW_SPEED_THR       20     // turn off before low-speed low-voltage occurs


/* Private function prototypes -----------------------------------------------*/

static void comm_plus(void);
static void comm_minus(void);
static void spd_plus(void);
static void spd_minus(void);
static void m_stop(void);
static void set_ctlm(void);


/* Public variables  ---------------------------------------------------------*/


/* Private types     ---------------------------------------------------------*/

/**
 * @brief Data type for the key handler function.
 */
typedef void (*ui_handlrp_t)( void );

// local enum only for setting enumerated order to UI event dispatcher
enum
{
    SET_CTLM   = 'z',
    COMM_PLUS  = ']',
    COMM_MINUS = '[',
    SPD_PLUS   = '.', //'>',
    SPD_MINUS  = ',', //'<',
    M_STOP     = ' '  // one space character
};

/**
 * @brief Data type for the keycode lookup table.
 */ 
typedef char ui_keycode_t;

/**
 * @brief Data type for the key handler table.
 */
typedef struct
{
    ui_keycode_t   key_code;  /**< Key code. */
    ui_handlrp_t   phandler;  /**< Pointer to handler function. */
} ui_key_handler_t;


/* Private variables ---------------------------------------------------------*/

static uint16_t UI_pulse_dc;
static uint16_t UI_pulse_perd;
static uint16_t UI_pulse_dur;

static uint16_t Analog_slider; // input var for 10-bit ADC conversions
static uint8_t UI_Speed;       // speed setting in 8-bits
static int8_t Digital_trim_switch; // trim switches have + and - extents

static uint8_t TaskRdy;  // flag for timer interrupt for BG task timing

static uint8_t Log_Level;

static  uint16_t Vsystem; // persistent for averaging

static const ui_key_handler_t ui_keyhandlers_tb[] =
{
    {SET_CTLM,   set_ctlm},
    {COMM_PLUS,  comm_plus},
    {COMM_MINUS, comm_minus},
    {SPD_PLUS,   spd_plus},
    {SPD_MINUS,  spd_minus},
    {M_STOP,     m_stop}
};

// macros to help make the LUT slightly more encapsulateed
#define _SIZE_K_LUT  ( sizeof( ui_keyhandlers_tb ) / sizeof( ui_key_handler_t ) )
#define _GET_KEY_CODE( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].key_code
#define _GET_UI_HDLRP( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].phandler


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


/** @cond */ // hide some developer/debug code
// hack, temp
extern uint8_t Control_mode;
extern int Back_EMF_Falling_PhX;
extern int Back_EMF_Riseing_PhX;
/** @endcond */

/**
 * @brief Print one line to the debug serial port.
 *
 * @param clearf set 1 to zero the line count
 */
static void dbg_println(int zrof)
{
    static uint16_t Line_Count = 0;

    char sbuf[256] ;                     // am i big enuff?
    char cbuf[8] = { 0, 0 };

// globals are accessed (read) outside of CS so be it
    int16_t timing_error = Seq_get_timing_error();

    sbuf[0] = 0;

    if ( 0 != zrof)
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
    itoa( BLDC_PWMDC_Get(), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " Vs=");
    itoa( Vsystem, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " SF=");
    itoa( Faultm_get_status(), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " TTE=");
    itoa( timing_error, cbuf, 16);
    strcat(sbuf, cbuf);

#if 0
    strcat(sbuf, " CM=");
    itoa( Control_mode, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " bRi=");
    itoa( Back_EMF_Riseing_PhX, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " bFi=");
    itoa( Back_EMF_Falling_PhX, cbuf, 16);
    strcat(sbuf, cbuf);
#endif
    strcat(sbuf, " UI=");
    itoa(UI_Speed, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " AS=");
    itoa(Analog_slider, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " RF=");
    itoa(UI_pulse_dc, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " PPD=");
    itoa(UI_pulse_perd, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " DUR=");
    itoa(UI_pulse_dur, cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
}

//$0768 - $044A  = $031E
#define RF_PCNT_ZERO   0x044A
#define RF_PCNT_100    0x0768
#define RF_RANGE       (RF_PCNT_100 - RF_PCNT_ZERO) // 0x031E // hardcode range of pulse
#define RF_NORDO_THR   0x3000 // arbitrary 0x2BB0 -> 0x55C0 when receiving
/*
 * Service the slider and trim inputs for speed setting.
 * The UI Speed value is a uint8 and represents the adjustment range of e.g. a
 * proportional RC radio control signal (eventually), and alternatively the
 * slider-pot (the developer h/w) - the UI Speed is passed to PWMDC_set() where
 * is expected to be rescaled to suite the range/precision required for PWM timer.
 *
 * TODO: rate limit of speed input!
 */
static void set_ui_speed(void)
{
    uint16_t tmp_u16;
    int16_t tmp_sint16;
    uint16_t adc_tmp16 = ADC1_GetBufferValue( ADC1_CHANNEL_3 ); // ISR safe ... hmmmm
    Analog_slider = adc_tmp16 / 4; // [ 0: 1023 ] -> [ 0: 255 ]


    UI_pulse_perd = Driver_get_pulse_perd();

    UI_pulse_dur = Driver_get_pulse_dur();
// lose 1 bit of precision here ....
    tmp_u16 = ( UI_pulse_dur - RF_PCNT_ZERO ) >> 1;  //  /2

//   scale factors ...     ( 1/2   +   1/2 )  /     (1/4)
    UI_pulse_dc = (PWM_100PCNT>>1) * tmp_u16 / (RF_RANGE>>2);


// if RF pulse qualified, then use it - needs more thought
    if (UI_pulse_perd > RF_NORDO_THR)
    {
//	Analog_slider = UI_pulse_dc;
    }


// careful with expression containing signed int ... UI Speed is defaulted
// to 0 and only assign from temp sum if positive and clip to INT8 MAX S8.
    UI_Speed = 0;

    tmp_sint16 = Digital_trim_switch;
    tmp_sint16 += Analog_slider; // comment out to disable analog slider (throttle hi protection is WIPO)

    if (tmp_sint16 > 0)
    {
        // clip to INT8 MAX S8
        if (tmp_sint16 > U8_MAX)
        {
            tmp_sint16 = U8_MAX;
        }

        UI_Speed = (uint8_t)tmp_sint16;
    }
}

/**
 * @brief stop the system
 *
 * This is not particularly ISR safe but stopping the system so what then.
 * needs to be externable to main because the hard-button stop button is polled there
 */
void UI_Stop(void)
{
// reset the machine
    BL_reset();
}

/*
 * handlers for UI events must be short as they are invoked in ISR context
 */
// for development user only
static void comm_plus(void)
{
    BLDC_Spd_inc();
}
// for development user only
static void comm_minus(void)
{
    BLDC_Spd_dec();
}

// stop key from terminal ... merge w/ UI_stop?
static void m_stop(void)
{
    // reset the machine
    UI_Stop();

    // reset the simulated trim swich between system runs
    Digital_trim_switch = TRIM_DEFAULT;

    UARTputs("###\r\n");

    Log_Level = 1; // stop the logger output
    dbg_println(1 /* clear line count */ );
}

static void spd_plus(void)
{
    // if fault/throttle-high ... diag msg?
    if (Digital_trim_switch < S8_MAX)
    {
        Digital_trim_switch += 1;
    }
//    UARTputs("+++\r\n");               // no not inside CS
//            Log_Level = 255; // ahow some more info
}
static void spd_minus(void)
{
    // if fault/throttle-high ... diag msg?
    if (Digital_trim_switch > S8_MIN)
    {
        Digital_trim_switch -= 1;
        Log_Level = 1;
    }
//    UARTputs("---\r\n");                // no not inside CS
}

/** @cond */ // hide some developer/debug code
void BL_set_ctlm(void); // tmp

static void set_ctlm(void)
{
// toggle or rest the ftl state??
    BL_set_ctlm();
}
/** @endcond */


static ui_handlrp_t handle_term_inp(void)
{
    ui_handlrp_t fp = NULL;
    char sbuf[16] = "";
    char cbuf[8] = { 0, 0 };
    char key;

    if (SerialKeyPressed(&key))
    {
        int n;
        for (n = 0; n < _SIZE_K_LUT ; n++)
        {
            if (key == _GET_KEY_CODE( n ))
            {
//any terminal output specific to the key or handelr need to be done here and
// not in the handler itself because the handler is to be called from w/i the CS
                fp =_GET_UI_HDLRP( n );
                break;
            }
        }
// anykey ...
        Log_Level = 255;// default anykey enable continous/verbous log
// 1 line output on terminal ... term output does whiles but that ok here (not in CS)
        itoa(UI_Speed, cbuf, 16);
        strcat(sbuf, cbuf);
        strcat(sbuf, "\r\n");
        UARTputs(sbuf);
    }
    return fp;
}

/*
 * Execution context is 'main()'
 * Servicing the UI and communication handlers
 */
static void Periodic_task(void)
{
    BL_RUNSTATE_t bl_state;

// invoke the terminal input and ui speed subs, updates from their globals to occur in the CS
    ui_handlrp_t fp = handle_term_inp();

    disableInterrupts();  //////////////// DI

    if (NULL != fp)
    {
        fp();
    }

    // update the UI speed input slider+trim
    set_ui_speed();

    BLDC_PWMDC_Set(UI_Speed);

    bl_state = BL_get_state();

    Vsystem = ( Seq_Get_Vbatt() + Vsystem ) / 2; // sma

    enableInterrupts();  ///////////////// EI EI O


    // update system voltage diagnostic - should be interrupt safe, the status word
    // is the only access in ISR context, and it is a byte so shoud be atomic.
    if (BL_IS_RUNNING == bl_state )
    {
        Faultm_upd(VOLTAGE_NG, (faultm_assert_t)( Vsystem < V_SHUTDOWN_THR) );
    }

    /*
     * debug logging to terminal
     */
    if (Log_Level > 0)
    {
        // if log level less than <threshold> then decrement the count
        if (Log_Level < 255)
        {
            Log_Level -= 1;
        }
        dbg_println(0);
    }
}

/**
 * @brief Run Periodic Task if ready
 * @details
 * Called in non-ISR context - checks the background task ready flag which if !0
 * will invoke the Periodic Task function.
 * @note referred to as Pertask_chk_ready
 * @return true if task ran (allows caller to also sync w/ the time period)
 */
uint8_t Task_Ready(void)
{
    if (0 != TaskRdy)
    {
        TaskRdy = FALSE;
        Periodic_task();
        return TRUE;
    }
    return FALSE;
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
