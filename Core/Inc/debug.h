/*
 * debug.h
 *
 *  Created on: Aug 23, 2024
 *      Author: brian
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "main.h"

#ifdef DEBUG
  #include <stdio.h>
  #define DEBUG_LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
  #define DEBUG_UART_TRANSMIT HAL_UART_Transmit
#else
  #define DEBUG_UART_TRANSMIT(...) // No-op
  #define DEBUG_LOG(fmt, ...) // No-op
#endif

#define IN
#define OUT

#endif /* INC_DEBUG_H_ */
