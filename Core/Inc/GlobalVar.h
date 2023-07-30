/*
 * GlobalVar.h
 *
 *  Created on: Jul 16, 2023
 *      Author: Nikhil
 */

#ifndef INC_GLOBALVAR_H_
#define INC_GLOBALVAR_H_

#include <string.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;


#define DEBUG_UART         &huart3
#define BL_CMD_UART        &huart4


void debugMsg(char *format,...);



#endif /* INC_GLOBALVAR_H_ */
