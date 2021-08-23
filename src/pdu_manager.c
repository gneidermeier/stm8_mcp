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
#define MAX_RX_DATA_SIZE 8  //how big should this be?

#define NO_FRAME 255

/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint8_t size;
static uint8_t command;
static uint8_t data[MAX_RX_DATA_SIZE];
static uint8_t checkSum;

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
           return count;
       }       
    }
    return NO_FRAME;
}

/**
 * @brief Reads data off Rx buffer
 *
*/

uint8_t Read_Data(void)
{
    uint8_t i;
    
    size = Driver_Return_Rx_Buffer();
    command = Driver_Return_Rx_Buffer();
    
    for(i = 0; i < size; i++)
    {
        data[i] = Driver_Return_Rx_Buffer();
    }
    
    checkSum = Driver_Return_Rx_Buffer();
}


/* External functions ---------------------------------------------------------*/

/**
 * @brief Handle Rx Buffer
 *
*/

void Pdu_Manager_Handle_Rx(void)
{
    uint8_t frameLocation;
  
    frameLocation = Find_Frame();
  
    if(NO_FRAME != frameLocation)
    {
        Read_Data();
    }
}
