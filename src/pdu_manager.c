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

#define RX_BUFFER_SIZE  16  //how big should this be? Also, shouldn't be defined in two places.

/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint8_t size;
static uint8_t command;
static uint8_t data[DATA_MAX];
static uint8_t crc8;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Find SOF of Rx buffer
 *
*/

uint8_t Find_Frame(void)
{
    uint8_t count;
    
    for(count = 0; count < RX_BUFFER_SIZE; count++)
    {
       if(SOF == Driver_Return_Rx_Buffer())
       {
           return TRUE;
       }       
    }
    return FALSE;
}



/* External functions ---------------------------------------------------------*/

/**
 * @brief Handle Rx Buffer
 *
*/

void Pdu_Manager_Handle_Rx(void)
{
    uint8_t frameFound;
  
    frameFound = Find_Frame();
}
