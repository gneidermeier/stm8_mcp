/**
  ******************************************************************************
  * @file faultm.c
  * @brief Fault manager for BLDC
  * @author Neidermeier
  * @version
  * @date Jan-2021
  ******************************************************************************
  */
/**
 * \defgroup faultm  Faultmon
 * @brief Fault manager for BLDC
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h> // memset
#include "faultm.h"  // public types used internally


/* Private defines -----------------------------------------------------------*/

/**
 * @brief Initialization value for fault counter bucket. The fault matrix defines
 * the bucket as a 6-bits unsigned integer bitfield.
 * @note This value should be derived from the faultm update rate somehow
 * @warning This is a warning!
 */
#define  FAULT_BUCKET_INI  63 // 6-bits (largets allowed by the bit-field)

/**
 * @brief The fault status word is 8-bits wide.
 */
#define NR_DEFINED_FAULTS  8


/* Private types -----------------------------------------------------------*/

/**
 * @brief Fault state - 1 bit
 *
 * Indicates that the fault condition has been detected frequently enough to
 * fill the bucket.
 */
typedef enum
{
    FCLR = 0,
    FSET = !FCLR
} faultm_state_t;

/**
 * @brief Indicate the fault is enabled - 1 bit
 */
typedef enum
{
    DISABLED = 0,
    ENABLED = !DISABLED
} faultm_enable_t;

/**
 * @brief Fault tracking table. The fault matrix shall be instantiated as an
 * array of 8-bit words
 *
 * Compact table for tracking faults with leaky-bucket algorithm.
 * @note  the matrix is to be instantiated as array of 8-bit words to save RAM
 * @warning the CPU could blow!
 * @todo verify that these bitfields are effective!
 */
typedef struct fault_matrix
{
    unsigned char bucket: 6; /**< bucket counter (6 bits). */
    faultm_enable_t enabled: 1; /**< Enable bit. */
    faultm_state_t state:  1; /**< Fault activation state. */

} faultm_mat_t;


/* Public variables  ---------------------------------------------------------*/

/**
 * @brief Fault status system word
 */
fault_status_reg_t fault_status_reg;


/* Private variables ---------------------------------------------------------*/

static faultm_mat_t fault_matrix[ NR_DEFINED_FAULTS ];


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

/**
 * @brief Initialize the Fault Manager
 *
 * Must be called each time the motor state changes from off to running.
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

/**
 * @brief Returns the status word
 *
 * @details Returns the system status bitmap boolean state.
 * @retval  0  all faults cleared
 * @retval  !0  at least one fault is set
 */
fault_status_reg_t Faultm_get_status(void)
{
    return fault_status_reg;
}

/**
 * @brief Set the fault matrix bit and status word.
 * @note TBD a single fault shuts down the system, i.e. no need to flag/mask multiple
 * faults?
 *
 * @param faultm_ID  Numerical ID of the fault to be set.
 *
 * @par Returns
 *    Nothing.
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


/**
 * @brief Manage fault status with leaky bucket.
 * @param faultm_ID  Numerical ID of the fault to be set.
 * @param tcondition  Boolean condition indicating if the fault condition was detected.
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

/**@}*/ // defgroup
