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

#define MAX_RX_DATA_SIZE 8  //how big should this be?

#define NO_FRAME 255

/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint8_t data[MAX_RX_DATA_SIZE];

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Find SOF of Rx buffer
 *
*/

static uint8_t Find_Frame(void)
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
 * @brief Reads data off Rx buffer and adds together checkSum
 *
*/

static uint8_t Read_Data(void)
{
  uint8_t i;
  uint8_t activeCheck;
  uint8_t size;
  uint8_t command;
  uint8_t receivedCheckSum;
    
  size = Driver_Return_Rx_Buffer();
  command = Driver_Return_Rx_Buffer();
    
  activeCheck = size + command;
    
  for(i = 0; i < size; i++)
  {
    data[i] = Driver_Return_Rx_Buffer();
    activeCheck += data[i];
  }
    
  receivedCheckSum = Driver_Return_Rx_Buffer();
    
  return activeCheck;
}

/**
 * @brief Checksum data off Rx buffer
 *
*/

static uint8_t Check_Data(uint8_t activeCheck)
{
  if(activeCheck == receivedCheckSum)
  {
    return TRUE;
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
  uint8_t frameLocation;
  uint8_t checkSum;
  
  frameLocation = Find_Frame();
  
  if(NO_FRAME != frameLocation)
  {
    checkSum = Read_Data();
    if(Check_Data(checkSum))  //return TRUE if good
    {
      Driver_Clear_Rx_Buffer_Element(frameLocation);
    }
  }
}
