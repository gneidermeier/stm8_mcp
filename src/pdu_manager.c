/**
  ******************************************************************************
  * @file temp.c
  * @brief
  * @author Shearer
  * @version
  * @date August-2021
  ******************************************************************************
  */
/**
 * \defgroup 
 * @brief
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "driver.h"

/* Private defines -----------------------------------------------------------*/

#define SOF 52

/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Find SOF of Rx buffer
 *
*/

uint8_t Find_Frame(void)
{
    if(SOF != Driver_Return_Rx_Buffer())
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

/* External functions ---------------------------------------------------------*/

/**
 * @brief Call from timer/capture ISR on capture of rising edge of servo pulse
 *
*/

void Pdu_Manager_Handle_Rx(void)
{
    uint8_t size;
  
    while(FALSE != Find_Frame());
    
    size = Driver_Return_Rx_Buffer();
}
