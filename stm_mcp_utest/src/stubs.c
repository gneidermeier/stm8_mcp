/*
 * horrors of unit testing
 */

#include <stdint.h>

#include "driver.h"


static uint16_t BLDC_OL_comm_tm;
static BLDC_STATE_T BLDC_State;
static uint16_t Vbatt;
static int Manual_Mode;



void TIM3_setup(uint16_t u16)
{
#ifdef DEBUG
    // doesn't need to do anything so just show a message
    printf("stubs::TIM3_setup((): $1 == %04X\n", u16);
#endif
}

void set_commutation_period(uint16_t u16)
{
    BLDC_OL_comm_tm = u16;
}

uint16_t get_commutation_period(void)
{
    return BLDC_OL_comm_tm;
}

BLDC_STATE_T get_bldc_state(void)
{
    return BLDC_State;
}

BLDC_STATE_T set_bldc_state( BLDC_STATE_T newstate)
{
    BLDC_State = newstate;
}

uint16_t get_vbatt(void)
{
    return Vbatt;
}

void set_vbatt(uint16_t u16)
{
    return Vbatt = u16;
}

int get_op_mode(void)
{
    return Manual_Mode;
}

void set_op_mode(int mode)
{
    Manual_Mode = mode;
}

void set_dutycycle(uint16_t global_dutycycle)
{
}

uint16_t get_dutycycle(void)
{
}



