/**
  ******************************************************************************
  * @file sequence.h
  * @brief
  * @author
  * @version
  * @date
  ******************************************************************************
  *
  * TBD
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "system.h"


/* prototypes -----------------------------------------------------------*/

uint16_t Seq_Get_Vbatt(void);
int16_t Seq_get_timing_error(void);

void Sequence_Step(void);


#endif // SEQUENCE_H
