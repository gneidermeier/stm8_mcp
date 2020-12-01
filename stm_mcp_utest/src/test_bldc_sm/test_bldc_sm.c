/**
  ******************************************************************************
  * @file    test_bldc_sm.c
  * @brief   test driver for bldc_sm.c
  * @author  Neidermeier
  * @version 1.0.0
  * @date DEC-2020
  ******************************************************************************
  */
/*
 * host system dependencies
 */
// needs an suitable source of 'uint16_t' etc before #including driver.h
#include <stdint.h>

/*
 * unit test framework headers
 */
#include "putf.h"

/*
 * application headers ... external defines, types, declarations
 */
#include "driver.h"


/*
 * implements a test case iteration
 * this test case is to run the sm thru entire normal ramp-up until
 * state transitions to ON.
 */
int test_case_1_iteration(void)
{
    static steps = 0; // local counter to track sequence in debug log output

    BLDC_STATE_T bldc_state = get_bldc_state();

    BLDC_Update();


    bldc_state = get_bldc_state();

    if (BLDC_ON == bldc_state) // assert
    {
        return TEST_DONE; // test iteration passed normally - stop sequence
    }

    else if (BLDC_RAMPUP == bldc_state) // assert
    {
        uint16_t commutation_per = get_commutation_period();

//#ifdef DEBUG
// log this to the terminal in such a way it can be grabbed into a data file
        printf(" commutation_per = %d %d %04X\n", steps, commutation_per, commutation_per);
//#endif
        steps += 1;
    }
    // iteration completed normally
    return TEST_OK;
}

/*
 * top-level test_driver
 * generic name .. individual makefile will link the test_driver() implementation
 */
void test_driver_1(void)
{
    /*
     * this test case is to run the sm thru entire normal ramp-up until
     * state transitions to ON.
     */
    set_bldc_state( BLDC_RAMPUP );

    // set to the starting/off time (normally done by sm in the "OFF" state handler
    set_commutation_period( /* BLDC_OL_TM_LO_SPD */ 0x1000 ); // todo: internal declaration in bldc_sm.c

// have to defeat diagnostics
    set_vbatt(0xFFFF /* V_SHUTDOWN_THR */); // todo: internal declaration in bldc_sm.c

    putf_n_iterations(10000, &test_case_1_iteration, "test_case_1_iteration");


}

/*
 * generic implementation of test suite
 */
void test_suite(void)
{
    test_driver_1();
}



