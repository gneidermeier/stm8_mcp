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

#include "sequence.h"


/* Private defines -----------------------------------------------------------*/

/*
 * threshold and bucket establish the sensitivity of the fault action.
 * the bucket value is a counter related to the update rate of the function.
 */
#define V_SHUTDOWN_THR      0x0340 // experimentally determined!

//this macro should be derived from the faultm update rate somehow
#define  FAULT_BUCKET_INI  63 // 6-bits (largets allowed by the bit-field)


/* Private types -----------------------------------------------------------*/


#define NR_DEFINED_FAULTS  8   // 8-bits provided by fault status bitmap


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



static faultm_mat_t fault_matrix[ NR_DEFINED_FAULTS ];



/* Public variables  ---------------------------------------------------------*/

fault_status_reg_t fault_status_reg;


/* Private variables ---------------------------------------------------------*/

static int vsys_fault_bucket;

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
    vsys_fault_bucket = FAULT_BUCKET_INI;

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
    return ( fault_status_reg != 0 );
}

/*
 * public function for fault matrix ... inline or possibly to be
 * converted to a macro
 */
void Faultm_setf(faultm_ID_t faultm_ID, faultm_assert_t tcondition)
{
// assert (fault_ID < MAX)
    fault_status_reg_t  mask = 1 << faultm_ID;

// use a pointer to cleanup (and optimize away the array-access?)
    faultm_mat_t * pfaultm  = &fault_matrix[ faultm_ID ];

    if (tcondition)
    {
        // voltage has sagged ... likely motor stall!
        if ( pfaultm->bucket < FAULT_BUCKET_INI )
        {
            pfaultm->bucket += 1;
        }
        else
        {
            // if the fault is enabled, then set it
            pfaultm->state =  (FALSE != pfaultm->enabled);
            fault_status_reg |= mask;
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
            pfaultm->state = FALSE ;
            fault_status_reg &= ~mask;
        }
    }
}
