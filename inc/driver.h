/**
  ******************************************************************************
  * @file driver.h
  * @brief  
  * @author
  * @version 
  * @date    
  ******************************************************************************
  *
  * BLAH BLAH BLAH
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef DRIVER_H
#define DRIVER_H

//#ifndef UNIT_TEST // is it broken?
  #include "system.h"
//#endif


/*
 * defines
 */


/*
 * types
 */


/*
 * variables
 */


/*
 * prototypes
 */

void Driver_Step(void);
void Driver_Update(void);
void Driver_Stop(void);

void bemf_samp_start( void );
void bemf_samp_get(void);

uint16_t get_vbatt(void);


#endif // DRIVER_H
