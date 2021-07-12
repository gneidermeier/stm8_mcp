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
#include <stdio.h>
#include <stddef.h> // NULL

// app headers
#include "mcu_stm8s.h"
#include "sequence.h"
#include "bldc_sm.h"
#include "faultm.h"
#include "driver.h"
#include "spi_stm8s.h"


/* Private defines -----------------------------------------------------------*/

#define TRIM_DEFAULT  0 //

// Threshold is set low enuogh that the machine doesn't stall
// thru the lower speed transition into closed-loop control.
// The fault can be tested by letting the spinning prop disc strike a flimsy
// obstacle like a 3x5 index card.
#if defined ( S105_DEV )
//  Vcc==3.3v  33k/10k @ Vbatt==12.4v
  #define V_SHUTDOWN_THR      0x0390    // experimentally determined @ 12.4v 
#else
  // applies presently only to the stm8s-Discovery, at 14.2v and ADCref == 5v
  #define V_SHUTDOWN_THR      0x0340    // experimentally determined!
#endif

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
#if 0 // tmp?
  COMM_PLUS  = ']',
  COMM_MINUS = '[',
#endif
  SPD_PLUS   = '.', //'>',
  SPD_MINUS  = ',', //'<',
  M_STOP     = ' '  // one space character
};

/**
 * @brief Data type for the key code lookup table.
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
static uint8_t UI_Speed;       // speed setting (needs to be in terms of pcnt of servo position)
static int8_t Digital_trim_switch; // trim switches have + and - extents

static uint8_t TaskRdy;  // flag for timer interrupt for BG task timing

static uint8_t Log_Level;

static  uint16_t Vsystem; // persistent for averaging

static const ui_key_handler_t ui_keyhandlers_tb[] =
{
//  {COMM_PLUS,  comm_plus},
//  {COMM_MINUS, comm_minus},
  {SPD_PLUS,   spd_plus},
  {SPD_MINUS,  spd_minus},
  {M_STOP,     m_stop}
};

// macros to help make the LUT slightly more encapsulated
#define _SIZE_K_LUT  ( sizeof( ui_keyhandlers_tb ) / sizeof( ui_key_handler_t ) )
#define _GET_KEY_CODE( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].key_code
#define _GET_UI_HDLRP( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].phandler


/* Private functions ---------------------------------------------------------*/

/**
 * @brief Print one line to the debug serial port.
 *
 * @param clearf set 1 to zero the line count
 */
static void dbg_println(int zrof)
{
  static uint16_t Line_Count = 0;
  // whats going on with the prinf format specifiers when lvalues are cast and/or promoted?
  int faults = (int)Faultm_get_status();
  int uispd = (int)UI_Speed;

  char sbuf[80] ;                     // am i big enuff?

// globals are accessed (read) outside of CS so be it
  int16_t timing_error = Seq_get_timing_error();

  if ( 0 != zrof)
  {
    Line_Count  = 0;
  }

  Line_Count  += 1;;

  printf(
    "{%04X) UI=%X CT=%04X DC=%04X Vs=%04X SF=%X RC=%04X ERR=%04X \r\n",
    Line_Count,
    uispd,
    get_commutation_period(),
    BLDC_PWMDC_Get(),
    Vsystem,
    faults,
    UI_pulse_dur,
    Seq_get_timing_error()
  );
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
#ifdef ANLG_SLIDER
  Analog_slider = adc_tmp16 / 4; // [ 0: 1023 ] -> [ 0: 255 ]
#else
  Analog_slider = 0;
#endif

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
    if (tmp_sint16 > U8_MAX)    //  TODO: limit should equate to 100% servo position (motor will die first!)
    {
      tmp_sint16 = U8_MAX;
    }

    UI_Speed = (uint8_t)tmp_sint16;
  }
}

/**
 * @brief  Stop the system
 */
void UI_Stop(void)
{
// reset the machine
  BL_reset();
}
#if 0
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
#endif

// stop key from terminal ... merge w/ UI_stop?
static void m_stop(void)
{
  // reset the machine
  UI_Stop();

  // reset the simulated trim swich between system runs
  Digital_trim_switch = TRIM_DEFAULT;

  printf("###\r\n");

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
}

static void spd_minus(void)
{
  // if fault/throttle-high ... diag msg?
  if (Digital_trim_switch > S8_MIN)
  {
    Digital_trim_switch -= 1;
    Log_Level = 1;
  }
}

static ui_handlrp_t handle_term_inp(void)
{
  ui_handlrp_t fp = NULL;
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
  }
  return fp;
}

/**
 * @brief  The User Interface task
 *
 * @details   Service the UI and communication handlers. Invoked in the
 *   execution context of 'main()' (background task)
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

  Vsystem = Seq_Get_Vbatt();

  enableInterrupts();  ///////////////// EI EI O

#if defined( UNDERVOLTAGE_FAULT_ENABLED )
  // update system voltage diagnostic - check plausibilty of Vsys
  if (BL_IS_RUNNING == bl_state  && Vsystem > 0  )
  {
    Faultm_upd(VOLTAGE_NG, (faultm_assert_t)( Vsystem < V_SHUTDOWN_THR) );
  }
#endif
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
 * @brief  Run Periodic Task if ready
 *
 * @details
 * Called in non-ISR context - checks the background task ready flag which if !0
 * will invoke the Periodic Task function.
 * @note  Referred to as Pertask_chk_ready
 * @return  True if task ran (allows caller to also sync w/ the time period)
 */
uint8_t Task_Ready(void)
{
  static uint8_t framecount = 0;

  if (0 != TaskRdy)
  {
    TaskRdy = FALSE;
    Periodic_task();

// periodic task is enabled at ~60 Hz ... the modulus provides a time reference of
// approximately 2 Hz at which time the master attempts to read a few bytes from SPI

    if ( ! ((framecount++) % 0x20) )
    {
#if SPI_ENABLED == SPI_STM8_MASTER
      SPI_controld();
#endif
    }
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief  Trigger background task.
 *
 * @details
 * Called in ISR context - sets the background task ready flag which when seen
 * by polling Task_Ready in background task will invoke the Periodic Task function.
 */
void Periodic_Task_Wake(void)
{
  TaskRdy = TRUE; // notify background process
}

/**@}*/ // defgroup
