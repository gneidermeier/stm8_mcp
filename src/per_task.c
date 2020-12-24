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


/* Private defines -----------------------------------------------------------*/


/* Public variables  ---------------------------------------------------------*/

//static tmp
uint8_t TaskRdy;  // flag for timer interrupt for BG task timing
                  // use 8-bit (atomic) so no concurrency concern



/* Private variables ---------------------------------------------------------*/

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

static uint16_t UI_Speed;

extern int Back_EMF_Falling_Int_PhX;
extern uint16_t Vsystem;

extern uint16_t Back_EMF_Falling_4[4]; // driver writes to the global - it is a bad-actor

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
    strcat(sbuf, " bFi=");
    itoa( Back_EMF_Falling_Int_PhX,     cbuf, 16);
    strcat(sbuf, cbuf);

// tmep ... log the test back-EMF falling-float transition
    strcat(sbuf, " 000=");
    itoa( Back_EMF_Falling_4[0],     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " 001=");
    itoa( Back_EMF_Falling_4[1],     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " 002=");
    itoa( Back_EMF_Falling_4[2],     cbuf, 16);
    strcat(sbuf, cbuf);

    strcat(sbuf, " 003=");
    itoa( Back_EMF_Falling_4[3],     cbuf, 16);
    strcat(sbuf, cbuf);
#endif

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
    char key;

#if 0
//   svc a UI potentiometer
    UI_Speed = ADC1_GetBufferValue( ADC1_CHANNEL_3 ); // ADC1_GetConversionValue();
    UI_Speed /= 16; // use [ 0: 63 ]

    if (0 == manual_mode_start )
    {
// was NOT started in dev/test mode, so go ahead and use "UI" (servo-pulse?) input

        if (0 == Fault)
        {
            BLDC_PWMDC_Set(UI_Speed); // note only does anything if BLDC_ON
        }
        else
        {
            BLDC_PWMDC_Set(0);
        }
    }
#endif
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
        if (key == ' ') // space character
        {
            // disable/enable interrupts is done to protect against concurrent access from ISR
            disableInterrupts();
            BLDC_Stop();
            enableInterrupts();
            UARTputs("###\r\n");

            Log_Level = 0;
        }
        else if (key == '+')
        {
            // disable/enable interrupts is done to protect against concurrent access from ISR
            disableInterrupts();
            BLDC_PWMDC_Plus();
            enableInterrupts();
            UARTputs("+++\r\n");

            Log_Level = 255;// enable continous/verbous log

        }
        else if (key == '-')
        {
            // disable/enable interrupts is done to protect against concurrent access from ISR
            disableInterrupts();
            BLDC_PWMDC_Minus();
            enableInterrupts();
            UARTputs("---\r\n");
        }
        else // anykey
        {
            Log_Level = 5; // show some info
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
        Periodic_task();
    }
    return TaskRdy;
}
