/**
  ******************************************************************************
  * @file    putf.h
  * @brief   Preliminary unit test framework.
  * @author  Neidermeier
  * @version 1.0.0
  * @date DEC-2020
  ******************************************************************************
  */
#ifndef PUTF_H
#define PUTF_H


#define TEST_OK    0  // test iteration passed normally
#define TEST_DONE  1  // test iteration passed normally - stop sequence
#define TEST_FAIL -1  // assertion failed

/*
 * use this type to declare test function iteration
 */
typedef int (*test_iteration_fn_t)(void);


/*
 * Executes the arbitrary number of cycles of the user's iteration
 * function.
 * A test case is basically a series of invocations of a piece of code.
 * Usually a single call to the function has little value.
 * Before invoking an IF series, the user would set the expected
  * entry conditions for the test case.
  * The IF is free to return either pass (0) or fail (-1) return
  * status at its discretion. Since the IF is invoked at each iteration.
  *
  * Each time the IF is invoked the result is checked to see if an exit
  * condition has been returned which would mean the test has
  * either executed the number of cycles or exited prematurely.
  * Either way the outcome is logged and returns
 *  boolean status of the asserted of the outcome conditions
 */
int putf_n_iterations(
  unsigned short n_cycles, test_iteration_fn_t, char *description );

#endif // PUTF_H
