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
#ifndef MDATA_H
#define MDATA_H

// if building in Cosmic (embedded) environment let it use system headers 
#ifdef _COSMIC_

#else
 // in gcc or whatever, let it use the standard header file for uint types etc.
 #include <stdint.h>
#endif

//#include <parameter.h> //  TIM2_PWM_PD

/*
 * defines
 */

#define TABLE_SIZE   250 // TIM2_PWM_PD

#define PWM_100PCNT  TABLE_SIZE

/*
 * types
 */

/*
 * variables
 */

/*
 * prototypes
 */

//extern const uint16_t OL_Timing[ PWM_100PCNT ] ;
uint16_t Get_OL_Timing(uint16_t index);


#endif // MDATA_H
