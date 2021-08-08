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

#define TRIM_DEFAULT  0

// Threshold is set low enuogh that the machine doesn't stall
// thru the lower speed transition into closed-loop control.
// The fault can be tested by letting the spinning prop disc strike a flimsy
// obstacle like a 3x5 index card.
#if defined ( S105_DEV )
//  Vcc==3.3v  33k/10k @ Vbatt==12.4v
  #define V_SHUTDOWN_THR      0x02C0    // experimentally determined @ 12.4v
#else
  // applies presently only to the stm8s-Discovery, at 14.2v and ADCref == 5v
  #define V_SHUTDOWN_THR      0x02C0    // experimentally determined!
#endif


//#define ANLG_SLIDER


/* Private function prototypes -----------------------------------------------*/

// forward declarations for UI input handers
static void timing_plus(void);
static void timing_minus(void);
static void spd_plus(void);
static void spd_minus(void);
static void m_stop(void);
static void m_start(void);


/* Public variables  ---------------------------------------------------------*/

/* Private types     ---------------------------------------------------------*/

/**
 * @brief Data type for the key handler function.
 */
typedef void (*ui_handlrp_t)( void );

/**
 * @brief Data type for the key code lookup table.
 */
typedef enum
{
  COMM_PLUS   = ']',
  COMM_MINUS  = '[',
  M_STOP      = ' ', // space bar
  M_START     = '/', // /
  SPD_PLUS    = '.', // >
  SPD_MINUS   = ',', // <
  K_UNDEFINED = -1
} 
ui_keycode_t;

/**
 * @brief Data type for the key handler table.
 */
typedef struct
{
  ui_keycode_t   key_code;  /**< Key code. */
  ui_handlrp_t   phandler;  /**< Pointer to handler function. */
} 
ui_key_handler_t;


/* Private variables ---------------------------------------------------------*/

static uint16_t Analog_slider; // input var for 10-bit ADC conversions
static uint8_t UI_Speed;       // speed setting (needs to be in terms of pcnt of servo position)
static int8_t Digital_trim_switch; // trim switches have + and - extents

static uint8_t TaskRdy;  // flag for timer interrupt for BG task timing

static uint8_t Log_Level;

static  uint16_t Vsystem; // persistent for averaging

/**
 * @brief Lookup table for UI input handlers
 */
static const ui_key_handler_t ui_keyhandlers_tb[] =
{
#ifdef ENABLE_MAN_TIMING
  {COMM_PLUS,  timing_plus},
  {COMM_MINUS, timing_minus},
#endif
  {SPD_PLUS,   spd_plus},
  {SPD_MINUS,  spd_minus},
  {M_STOP,     m_stop},
  {M_START,    m_start}	
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
  int faults = (int)Faultm_get_status();
  uint16_t ui_speed = UI_Speed;
  uint16_t bl_speed = BL_get_speed(); 
  uint16_t timing_error = Seq_get_timing_error();
  uint16_t comm_period = BL_get_timing();
  uint16_t servo_pulse_period = Driver_get_pulse_perd();
  uint16_t servo_pulse_duration = Driver_get_pulse_dur();
  uint16_t display_speed_pcnt = 
	   (uint16_t)( Driver_get_motor_spd_pcnt() / SPEED_PCNT_SCALE );

  if ( 0 != zrof)
  {
    Line_Count = 0;
  }

  Line_Count += 1;;

  printf(
    "{%04X) UI=%X CT=%04X DC=%04X Vs=%04X SF=%X RCd=%04X RCp=%u ERR=%04X \r\n",
//    "{%04X) UI=%X CT=%04X DC=%04X Vs=%04X SF=%X RCd=%04X ERR=%04X \r\n",
    Line_Count,
    ui_speed,
    comm_period,
    bl_speed,
    Vsystem,
    faults,
    servo_pulse_duration,
    display_speed_pcnt,
    timing_error
  );
}

/*
 * Service the slider and trim inputs for speed setting.
 * The UI Speed value is a uint8 and represents the adjustment range of e.g. a
 * proportional RC radio control signal (eventually), and alternatively the
 * slider-pot (the developer h/w) - the UI Speed is passed to PWMDC_set() where
 * is expected to be rescaled to suite the range/precision required for PWM timer.
 *
 * Update the PWM/speed. Done here because in the UI is where user vs. dev mode 
 * is set, so the logic to gate either the servo signal or the remote-UI input 
 * to the motor-speed setting. Also possibility of analog slider. Since pertask
 * is updating at 60Hz, it is adequate and in fact obviously faster frame-rate 
 * than the RC radio standard (45-50Hz).
 */
static void set_ui_speed(void)
{
  uint16_t tmp_u16;
  int16_t tmp_s16;

#ifdef ANLG_SLIDER
  uint16_t adc_tmp16 = ADC1_GetBufferValue( ADC1_CHANNEL_3 ); // ISR safe ... hmmmm
  Analog_slider = adc_tmp16 / 4; // [ 0: 1023 ] -> [ 0: 255 ]
#endif
#if 0
  Servo_period = Driver_get_pulse_perd();
  Servo_duration = Driver_get_pulse_dur();

  if (Servo_duration > TCC_THRTTLE_0PCNT )
  {
    Throttle_pcnt_speed = (uint16_t )TCC_THRTTLE_PCNT_SPD( Servo_duration );
  }
// todo 
// BL_set_speed( Throttle_pcnt_speed );
#endif
// careful with expression containing signed int ... UI Speed is defaulted
// to 0 and only assign from temp sum if positive and clip to INT8 MAX S8.
  UI_Speed = 0;

  tmp_s16 = Digital_trim_switch;
#ifdef ANLG_SLIDER
  tmp_s16 += Analog_slider; // comment out to disable analog slider (throttle hi protection is WIPO)
#endif

  if (tmp_s16 > 0)
  {
    // clip to INT8 MAX S8
    if (tmp_s16 > U8_MAX)    //  TODO: limit should equate to 100% servo position (motor will die first!)
    {
      tmp_s16 = U8_MAX;
    }

    UI_Speed = (uint8_t)tmp_s16;
    BL_set_speed(UI_Speed);
  }
}

/*
 * handlers for UI events must be short as they are invoked in ISR context
 */
#ifdef ENABLE_MAN_TIMING
// for development user only
static void timing_plus(void)
{
  BL_timing_step_slower();
}
// for development user only
static void timing_minus(void)
{
  BL_timing_step_faster();
}
#endif

/*
 * motor start
 */
static void m_start(void)
{
}

/*
 * motor stop
 */
static void m_stop(void)
{
  // reset the machine
  BL_reset();

  // reset the simulated trim swich between system runs
  Digital_trim_switch = TRIM_DEFAULT;

  printf("###\r\n");

  Log_Level = 1; // stop the logger output
  dbg_println(1 /* clear line count */ );
}

/*
 * motor speed up
 */
static void spd_plus(void)
{
  // if fault/throttle-high ... diag msg?
  if (Digital_trim_switch < S8_MAX)
  {
    Digital_trim_switch += 1;
  }
}

/*
 * motor speed down
 */
static void spd_minus(void)
{
  // if fault/throttle-high ... diag msg?
  if (Digital_trim_switch > S8_MIN)
  {
    Digital_trim_switch -= 1;
    Log_Level = 1;
  }
}

/*
 * handle terminal input - these are simple 1-key inputs for now
 */
static ui_handlrp_t handle_term_inp(void)
{
  ui_handlrp_t fp = NULL;
  char key;

// Uses non-blocking/non-buffered scan for key input similar to getch()
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

  bl_state = BL_get_state();

  Vsystem = Seq_Get_Vbatt();

  enableInterrupts();  ///////////////// EI EI O

#if defined( UNDERVOLTAGE_FAULT_ENABLED )
  // update system voltage diagnostic - check plausibilty of Vsys
  if( BL_IS_RUNNING == bl_state && Vsystem > 0 )
  {
    Faultm_upd(VOLTAGE_NG, (faultm_assert_t)( Vsystem < V_SHUTDOWN_THR) );
  }
#endif
  /*
   * debug logging to terminal - note: globals that are updated from ISR context
   * are not going to be read inside a CS (don't want printf inside a CS)
   */
  if (Log_Level > 0)
  {
    // if log level less than <threshold> then decrement the count
    if (Log_Level < 255)
    {
      Log_Level -= 1;
    }
    dbg_println(0); // note: would not want to put printf inside a CS (DI/EI)
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
