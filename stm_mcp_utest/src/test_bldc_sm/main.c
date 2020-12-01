#include <stdio.h>
#include <stdlib.h>


int test_suite(void);


int main()
{
    printf("Unit test suite ...\n");

    // generic name .. individual makefile will link the implementation
    test_suite();

    return 0;
}


