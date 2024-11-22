/*
 * debug.c
 *
 *  Created on: Aug 23, 2024
 *      Author: brian
 */

#include "debug.h"

#ifdef DEBUG
// overrides write function to send data back through st-link ITM
// means that writes to functions like printf will be sent to debugger
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}
#endif

