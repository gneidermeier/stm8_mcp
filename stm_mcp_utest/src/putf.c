/**
  ******************************************************************************
  * @file    main.c
  * @brief   Preliminary unit test framework.
  * @author  Neidermeier
  * @version 1.0.0
  * @date DEC-2020
  ******************************************************************************
  */
#include <stdio.h>

#include "putf.h"

 /**
 * @brief     Calls the given function the specified number of times .
 *
 *
 * @param     n_cycles Number of cycles to execute function.
 * @param[in] pFunc    Pointer to function to be called.
 *
 * @return
 *      return status of called function
 */
int putf_n_iterations(
    unsigned short n_cycles, test_iteration_fn_t pFunc, char *description)
{
    int test_result;
    unsigned short n;

    // assert description != null
    printf("putf_n_iterations(): \"%s\"\n", description);

    for (n = 0; n < n_cycles; n++)
    {
//        test_result =  (*pFunc)( );
        int (*pfn)(void) = *pFunc;
        test_result =  (*pfn)( );

        if ( 0 != test_result ) // assert
        {
            break;
        }
    }

    printf(
        "test_bldc_sm::test_driver(): %d / %u cycles completed. Status %d ",
        n, n_cycles, test_result);

    if (TEST_FAIL == test_result)
    {
        printf("status: NG\n");
    }

    return n;
}
