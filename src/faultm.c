/**
  ******************************************************************************
  * @file faultm.c
  * @brief Fault manager for BLDC
  * @author Neidermeier
  * @version
  * @date Jan-2021
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h> // memset
#include "faultm.h"  // public types used internally



/* Private defines -----------------------------------------------------------*/

//this macro should be derived from the faultm update rate somehow
#define  FAULT_BUCKET_INI  63 // 6-bits (largets allowed by the bit-field)

#define NR_DEFINED_FAULTS  8   // 8-bits provided by fault status bitmap


/* Private types -----------------------------------------------------------*/

typedef enum
{
    FCLR = 0,
    FSET = !FCLR
} faultm_state_t;

typedef enum
{
    DISABLED = 0,
    ENABLED = !DISABLED
} faultm_enable_t;

typedef struct fault_matrix
{
    unsigned char bucket: 6;
    faultm_enable_t enabled: 1;
    faultm_state_t state:  1;

} faultm_mat_t;


/* Public variables  ---------------------------------------------------------*/

fault_status_reg_t fault_status_reg;


/* Private variables ---------------------------------------------------------*/

static faultm_mat_t fault_matrix[ NR_DEFINED_FAULTS ];


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

/*
 * initialize
 */
void Faultm_init(void)
{
    int nnn;

    // intialize fault matrix
    memset(fault_matrix, 0, sizeof(fault_matrix) /* size in bytes */ );

//fault_matrix[VOLTAGE_NG ].enabled = 1;
//fault_matrix[THROTTLE_HI ].enabled = 1;

    for (nnn= 0; nnn < NR_DEFINED_FAULTS; nnn++)
    {
        fault_matrix[nnn].enabled = TRUE;
    }

// reset the fault status bitmap
    fault_status_reg = 0;
}

/*
 * return the system status bitmap boolean state
 * 0: all faults cleeared
 * !0: at least one fault is set
 */
fault_status_reg_t Faultm_get_status(void)
{
    return fault_status_reg;
}

/*
 * set the fault matrix bit and status word
 */
void Faultm_set(faultm_ID_t faultm_ID)
{
// assert (fault_ID < MAX)

// use a pointer to cleanup (and optimize away the array-access?)
    faultm_mat_t * pfaultm  = &fault_matrix[ faultm_ID ];

//    fault_status_reg_t  mask = (1 << faultm_ID); // maybe ... not necessary for now
    uint8_t  mask = (uint8_t) faultm_ID;

    pfaultm->state =  (FALSE != pfaultm->enabled);

// set bucket full ... wouldn't really need the "state" varaible
    pfaultm->bucket = -1; // FAULT_BUCKET_INI

    // note: OR allows multiple faults to be indicated in the status-word not that it
    // makes much difference
    fault_status_reg = mask; // |= mask;
}


/*
 * set the fault matrix bit and status word
 * If a single fault shuts down the system, it doesn't really need the means to 
 * flag multiple conditions.
 */
void Faultm_upd(faultm_ID_t faultm_ID, faultm_assert_t tcondition)
{
// assert (fault_ID < MAX)
//    fault_status_reg_t  mask = (1 << faultm_ID); // maybe ... not necessary for now
    fault_status_reg_t  mask = (fault_status_reg_t) faultm_ID;

// use a pointer to cleanup (and optimize away the array-access?)
    faultm_mat_t * pfaultm  = &fault_matrix[ faultm_ID ];

    if (tcondition)
    {
        // if bucket < thr, then increment it else latch the fault
        if ( pfaultm->bucket < FAULT_BUCKET_INI )
        {
            pfaultm->bucket += 1;
        }
        else
        {
            // if the fault is enabled, then set it
            Faultm_set(faultm_ID);
        }
    }
    else
    {
        if ( pfaultm->bucket > 0 )
        {
            pfaultm->bucket -= 1; // leaky bucket
        }
        else
        {
// shouldnt need to do anything here ... we don't clear faults (requires system reset)
        }
    }
}
