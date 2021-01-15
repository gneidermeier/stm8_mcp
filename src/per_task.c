/**
  ******************************************************************************
  * @file periodic_task.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date Dec-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

// app headers
#include "system.h" // platform specific delarations
#include "mcu_stm8s.h"

#include "bldc_sm.h"
#include "pwm_stm8s.h"
#include "faultm.h"


/* Private defines -----------------------------------------------------------*/

//#include <stdint.h> ... no .. compile error
#ifndef INT8_MIN 
  #define INT8_MIN S8_MIN
#endif

#ifndef INT8_MAX
  #define INT8_MAX S8_MAX 
#endif


#define TRIM_DEFAULT 28 // close to the minimum ramp DC

/* Public variables  ---------------------------------------------------------*/




/* Private variables ---------------------------------------------------------*/

static uint16_t Analog_slider; // input var for 10-bit ADC conversions
static uint8_t UI_Speed;       // speed setting in 8-bits
static int8_t Digital_trim_switch; // trim switches have + and - extents

static uint8_t TaskRdy;  // flag for timer interrupt for BG task timing

static uint8_t Log_Level;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 * home-made itoa function (16-bits only, hex-only)
 * seems Cosmic only provide atoi in stdlib and not itoa
 */
char * itoa(uint16_t u16in, char *sbuf, int base)
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
        unsigned char c = (n16 >> shift) & 0x000F;

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
char GetKey(void)
{
    char key = 0;
    /* Waiting for user input */
    while (1)
    {
        if (SerialKeyPressed((char*)&key)) break;
    }
    return key;
}


// hack, temp

extern int Back_EMF_Falling_PhX;
extern int Back_EMF_Riseing_PhX;

extern uint16_t Vsystem;


/*
 * temp, todo better function
 */
void testUART(void)
{
    static uint16_t Line_Count = 0;

    char sbuf[256] ;                     // am i big enuff?
    char cbuf[8] = { 0, 0 };

    sbuf[0] = 0;

    Line_Count  += 1;;

    strcat(sbuf, "(");
    itoa(Line_Count, cbuf, 16);
    strcat(sbuf, cbuf);
    strcat(sbuf, ")");

    strcat(sbuf, " CT=");
    itoa(get_commutation_period(), cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " DC=");
    itoa( get_dutycycle(),     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " Vs=");
    itoa( Vsystem,     cbuf, 16);
    strcat(sbuf, cbuf);

#if 1
    strcat(sbuf, " UI=");
    itoa(UI_Speed, cbuf, 16);
    strcat(sbuf, cbuf);
#endif

    strcat(sbuf, "\r\n");
    UARTputs(sbuf);
}

/*
 * Execution context is 'main()' so not to block ISR with ADC sampling.
 * TODO moving A/D acquisition to ramp-step (TIM3) to coordinate with PWM on/off cycling.
 * Servicing the UART (presently just a simply one-way logging stream)
 */
void Periodic_task(void)
{
    char sbuf[16] = "";
    char cbuf[8] = { 0, 0 };
    int16_t tmp_sint16;
    char key;

    BLDC_STATE_T bldc_state = get_bldc_state();

    // only do low-voltage/stalled diagnostic in ON state for now ... should it keep track
    // of the fault state or poll it?
    if (BLDC_ON == bldc_state )
    {
        if ( FAULT_SET == Faultm_update() )
        {
#if 1 // #if ENABLE_VLOW_FAULT
            set_bldc_state( BLDC_FAULT ); // eerrggg you are violating the state machine 
#endif
        }
    }

//   svc a UI potentiometer
    Analog_slider = ADC1_GetBufferValue( ADC1_CHANNEL_3 ); // ADC1_GetConversionValue();
    Analog_slider /= 4; // [ 0: 1023 ] -> [ 0: 255 ]

 Analog_slider  = 0; // tmp ... zero it out

// careful with expression containing signed int ... ui_speed is defaulted 
// to 0 and only assign from temp sum if positive.
    UI_Speed = 0;
    tmp_sint16 = Analog_slider + Digital_trim_switch;

    if (tmp_sint16 > 0)
    {
        UI_Speed = Analog_slider + Digital_trim_switch;
    }  

    BLDC_PWMDC_Set(UI_Speed);

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

        testUART();// tmp test
    }

    if (SerialKeyPressed(&key))
    {
        Log_Level = 1; // show some info

        if (key == ' ') // space character
        {
            BLDC_Stop();

            UARTputs("###\r\n");

            Log_Level = 0;
// reset the simulated trim swtich between system runs
            Digital_trim_switch = TRIM_DEFAULT;
        }
        else if (key == '+')
        {
            if (Digital_trim_switch < INT8_MAX)
            {
                Digital_trim_switch += 1;
            }
            BLDC_PWMDC_Plus();          ///// todo: go away

            UARTputs("+++\r\n");

            Log_Level = 20; // ahow some more info
        }
        else if (key == '-')
        {
            if (Digital_trim_switch > INT8_MIN)
            {
                Digital_trim_switch -= 1;
            }
            BLDC_PWMDC_Minus();             ///// todo: go away

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

/*
 * retrieve status of global flag set by ISR
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

/*
 * activate task ready flag (from ISR context)
 */
void Periodic_Task_Wake(void)
{
    TaskRdy = TRUE; // notify background process
}
